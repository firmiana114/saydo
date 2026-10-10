"""相机服务与取图程序之间的约定：原始画面怎么放进共享内存、怎么取。

要解决的问题：一台相机同一时刻只能被一个进程打开，而要用画面的程序
不止一个（预览、标定、定位与跟踪）。所以相机统一由相机服务（8088）
打开，别的程序向它要画面。预览和标定每秒取一两次，走网页接口、传
压缩过的图就够了；定位与跟踪要的是每秒 15 帧、未压缩的彩色图和深度
图，一帧约 4.5 MB，走网页接口太慢。

做法：相机服务把最新的一帧写进一块共享内存（shared memory，两个
进程都能读写的同一块内存，这里是 `/dev/shm` 下的一个文件），取图
程序把它映射进自己的地址空间直接读，不经过网络，取一帧只是一次内存
复制，约 1 毫秒。

本模块有三部分：

- `FrameWriter`：相机服务用，把一帧写进共享内存。
- `FrameReader`：按约定的排布读出一帧。
- `Feed`：取图程序用的完整客户端。它定期告诉相机服务“我还在用”
  （相机服务据此保持采集；程序退出后到期自动释放相机），并在相机
  服务重新打开相机后自动接上新的共享内存。

读写不加锁，靠一个计数器保证读到的一帧是完整的：写之前把计数器加一
（变成奇数），写完再加一（变回偶数）；读的一方复制前后各看一次计数
器，两次相同且为偶数才算数，否则重读。这种办法叫顺序锁（seqlock）。
"""

from __future__ import annotations

import json
import mmap
import os
import pathlib
import struct
import time
import urllib.error
import urllib.request
from typing import Any, Callable, Dict, Optional

import numpy as np

MAGIC = b'SAYDOCAM'
VERSION = 1
# 文件开头留给说明信息的字节数，之后依次是彩色图和深度图。
HEADER_SIZE = 512
# 不变的部分：标识、版本、宽、高、有无深度、深度单位（米）、
# fx fy cx cy、5 个畸变系数、畸变模型名、序列号、型号。
STATIC = struct.Struct('<8sIIIId4d5d32s32s32s')
# 每帧都变的部分：计数器、帧序号、曝光时刻、主机收到的时刻。
DYNAMIC = struct.Struct('<QQdd')
DYNAMIC_OFFSET = 256
# 读一帧时最多重读几次；写一帧约 1 毫秒，连续撞上的可能性极小。
READ_ATTEMPTS = 20
# 收到时刻距今超过这么久的帧不算数，单位秒：相机服务已经不在写了。
STALE_S = 1.
# 等新帧时每隔多久看一次，单位秒。
POLL_S = .002
# 这么久没有新帧就向相机服务确认一次状态，单位秒。
SILENCE_S = .5


class CameraUnavailable(RuntimeError):
    """相机服务此刻给不出这台相机的画面；原因在异常信息里。"""


def _text(raw: bytes) -> str:
    return raw.rstrip(b'\0').decode()


class FrameWriter:
    """相机服务用：把最新的一帧写进共享内存。

    Attributes:
        path: 共享内存文件的路径。
    """

    def __init__(
        self,
        path: pathlib.Path,
        width_px: int,
        height_px: int,
        intrinsics: Dict[str, Any],
        serial: str,
        name: str,
        depth_scale_m: Optional[float] = None,
    ) -> None:
        """创建共享内存并写入不变的说明信息。

        先在临时文件里准备好再改名，取图程序不会看到写了一半的文件。
        同名的旧文件（上次异常退出留下的）会被替换。

        Args:
            path: 共享内存文件的路径。
            width_px: 图像宽度。
            height_px: 图像高度。
            intrinsics: 彩色画面的内参，含 `fx`、`fy`、`cx`、`cy`、
                `distortion`（5 个系数）和 `distortion_model`。
            serial: 相机序列号。
            name: 相机型号。
            depth_scale_m: 深度图里一个单位是多少米；None 表示这台
                相机不提供深度图。
        """
        self.path = pathlib.Path(path)
        self.has_depth = depth_scale_m is not None
        pixels = width_px * height_px
        size = HEADER_SIZE + pixels * 3 + (pixels * 2 if self.has_depth else 0)
        temporary = self.path.with_name(self.path.name + '.tmp')
        descriptor = os.open(
            temporary, os.O_CREAT | os.O_RDWR | os.O_TRUNC, 0o600
        )
        try:
            os.ftruncate(descriptor, size)
            self._map = mmap.mmap(descriptor, size)
        finally:
            os.close(descriptor)
        STATIC.pack_into(
            self._map, 0, MAGIC, VERSION, width_px, height_px,
            int(self.has_depth), float(depth_scale_m or 0.),
            intrinsics['fx'], intrinsics['fy'],
            intrinsics['cx'], intrinsics['cy'],
            *intrinsics['distortion'],
            intrinsics['distortion_model'].encode(),
            serial.encode(), name.encode(),
        )
        self._counter = 0
        self._color = np.frombuffer(
            self._map, np.uint8, pixels * 3, HEADER_SIZE
        ).reshape(height_px, width_px, 3)
        self._depth = (
            np.frombuffer(
                self._map, np.uint16, pixels, HEADER_SIZE + pixels * 3
            ).reshape(height_px, width_px)
            if self.has_depth
            else None
        )
        os.replace(temporary, self.path)

    def write(
        self,
        bgr: np.ndarray,
        depth_raw: Optional[np.ndarray],
        sequence: int,
        captured_at: float,
        received_at: float,
    ) -> None:
        """写入新的一帧。

        Args:
            bgr: 彩色图（蓝绿红）。
            depth_raw: 已对齐到彩色图的深度图（相机给出的原始单位）；
                这台相机不提供深度图时为 None。
            sequence: 帧序号，必须逐帧增大。
            captured_at: 曝光时刻（`time.time()` 的时间基准）。
            received_at: 主机收到这一帧的时刻。
        """
        self._counter += 1
        struct.pack_into('<Q', self._map, DYNAMIC_OFFSET, self._counter)
        self._color[...] = bgr
        if self._depth is not None:
            self._depth[...] = depth_raw
        self._counter += 1
        DYNAMIC.pack_into(
            self._map, DYNAMIC_OFFSET, self._counter, sequence,
            captured_at, received_at,
        )

    def close(self) -> None:
        """停止写入并删除共享内存文件；已经接上的取图程序会发现画面过期。"""
        self._color = self._depth = None
        self._map.close()
        try:
            self.path.unlink()
        except FileNotFoundError:
            pass


class FrameReader:
    """按约定的排布从共享内存读出一帧。

    Attributes:
        width_px: 图像宽度。
        height_px: 图像高度。
        camera_matrix: 彩色画面的内参矩阵。
        distortion: 5 个畸变系数。
        distortion_model: 畸变模型名。
        serial: 相机序列号。
        name: 相机型号。
        depth_scale_m: 深度图里一个单位是多少米；没有深度图时为 None。
    """

    def __init__(self, path: pathlib.Path) -> None:
        """接上一块共享内存。

        Args:
            path: 共享内存文件的路径。

        Raises:
            CameraUnavailable: 文件不存在，或不是本约定这一版写的。
        """
        try:
            with open(path, 'rb') as file:
                self._map = mmap.mmap(
                    file.fileno(), 0, access=mmap.ACCESS_READ
                )
        except (OSError, ValueError) as error:
            raise CameraUnavailable(
                f'接不上相机服务的共享内存 {path}（{error}）'
            ) from error
        if len(self._map) < HEADER_SIZE:
            self._map.close()
            raise CameraUnavailable(f'共享内存 {path} 不完整')
        (
            magic, version, self.width_px, self.height_px, has_depth,
            depth_scale_m, fx, fy, cx, cy, *rest
        ) = STATIC.unpack_from(self._map, 0)
        pixels = self.width_px * self.height_px
        expected = HEADER_SIZE + pixels * (5 if has_depth else 3)
        if (
            magic != MAGIC
            or version != VERSION
            or len(self._map) != expected
        ):
            self._map.close()
            raise CameraUnavailable(
                f'共享内存 {path} 的格式与本程序不符；相机服务和取图'
                '程序须是同一版代码'
            )
        self.camera_matrix = np.array(
            [[fx, 0., cx], [0., fy, cy], [0., 0., 1.]]
        )
        self.distortion = np.array(rest[:5])
        self.distortion_model = _text(rest[5])
        self.serial, self.name = _text(rest[6]), _text(rest[7])
        self.depth_scale_m = depth_scale_m if has_depth else None
        self._color = np.frombuffer(
            self._map, np.uint8, pixels * 3, HEADER_SIZE
        ).reshape(self.height_px, self.width_px, 3)
        self._depth = (
            np.frombuffer(
                self._map, np.uint16, pixels, HEADER_SIZE + pixels * 3
            ).reshape(self.height_px, self.width_px)
            if has_depth
            else None
        )

    def read(self, after_sequence: int = 0) -> Optional[dict]:
        """读出最新的一帧。

        Args:
            after_sequence: 只要帧序号比它大的；用来避免重复取同一帧。

        Returns:
            含 `bgr`、`depth_raw`（没有深度图时为 None）、
            `depth_scale_m`、`K`、`distortion`、`distortion_model`、
            `serial`、`name`、`sequence`、`captured_at`、`received_at`
            的字典，图像是复制出来的；没有更新的帧时为 None。
        """
        for _ in range(READ_ATTEMPTS):
            counter, sequence, captured_at, received_at = (
                DYNAMIC.unpack_from(self._map, DYNAMIC_OFFSET)
            )
            if counter % 2:
                # 相机服务正在写这一帧。
                time.sleep(POLL_S / 4)
                continue
            if sequence <= after_sequence:
                return None
            bgr = self._color.copy()
            depth_raw = None if self._depth is None else self._depth.copy()
            if struct.unpack_from('<Q', self._map, DYNAMIC_OFFSET)[0] == (
                counter
            ):
                return {
                    'bgr': bgr,
                    'depth_raw': depth_raw,
                    'depth_scale_m': self.depth_scale_m,
                    'K': self.camera_matrix,
                    'distortion': self.distortion,
                    'distortion_model': self.distortion_model,
                    'serial': self.serial,
                    'name': self.name,
                    'sequence': sequence,
                    'captured_at': captured_at,
                    'received_at': received_at,
                }
        return None

    def close(self) -> None:
        """断开共享内存。"""
        self._color = self._depth = None
        self._map.close()


class Feed:
    """取图程序用：从相机服务持续取一台相机的原始画面。

    本对象不打开相机。第一次取图时相机没开，由相机服务打开（约两秒）；
    之后每隔几秒续一次期限，程序退出或不再取图后，期限一到相机服务
    自行释放相机，不需要归还。

    Attributes:
        base_url: 相机服务的地址。
        key: 相机在相机服务里的键，例如 `d405`。
    """

    def __init__(
        self,
        base_url: str,
        key: str,
        keep_s: float = 15.,
        clock: Callable[[], float] = time.monotonic,
    ) -> None:
        """记下相机服务的地址；不发起连接。

        Args:
            base_url: 相机服务的地址。
            key: 相机在相机服务里的键。
            keep_s: 每次让相机服务再保持采集多久，单位秒。
            clock: 计时函数，测试时替换。
        """
        self.base_url, self.key = base_url.rstrip('/'), key
        self.keep_s = keep_s
        self._clock = clock
        self._opener = urllib.request.build_opener(
            urllib.request.ProxyHandler({})
        )
        self._reader: Optional[FrameReader] = None
        self._shared_id: Optional[str] = None
        self._kept_at: Optional[float] = None
        self._last_sequence = 0

    def _keep(self) -> None:
        """向相机服务续期，并在它换了共享内存时重新接上。"""
        request = urllib.request.Request(
            f'{self.base_url}/api/cameras/{self.key}/keep',
            data=json.dumps({'keep_s': self.keep_s}).encode(),
            method='POST',
        )
        try:
            with self._opener.open(request, timeout=15) as reply:
                status = json.load(reply)
        except urllib.error.HTTPError as error:
            try:
                reason = json.load(error).get('error')
            except ValueError:
                reason = None
            raise CameraUnavailable(
                reason or f'相机服务返回 {error.code}'
            ) from error
        except (OSError, ValueError) as error:
            raise CameraUnavailable(
                f'连不上相机服务 {self.base_url}（{error}）'
            ) from error
        self._kept_at = self._clock()
        shared = status.get('shared_frames')
        if not shared:
            raise CameraUnavailable(
                f'相机服务不提供 {self.key} 的原始画面（它转接的是别的'
                '程序的画面）'
            )
        if self._reader is None or shared['id'] != self._shared_id:
            self._drop()
            self._reader = FrameReader(pathlib.Path(shared['path']))
            self._shared_id = shared['id']

    def _drop(self) -> None:
        if self._reader is not None:
            self._reader.close()
        self._reader, self._shared_id, self._last_sequence = None, None, 0

    def frame(self, timeout_s: float = 5.) -> dict:
        """取一帧比上次取到的更新的画面，必要时等它。

        Args:
            timeout_s: 最多等多久，单位秒；要留出相机服务打开相机的
                时间。

        Returns:
            见 `FrameReader.read`。

        Raises:
            CameraUnavailable: 相机服务没有运行、相机打不开，或限时内
                没有新画面。
        """
        started = self._clock()
        asked_at = started
        while True:
            now = self._clock()
            if (
                self._reader is None
                or now - self._kept_at > self.keep_s / 3
                or now - asked_at > SILENCE_S
            ):
                # 相机服务可能重新打开过相机；续期时会发现并重新接上。
                self._keep()
                asked_at = now
            frame = self._reader.read(self._last_sequence)
            if frame is not None:
                self._last_sequence = frame['sequence']
                if time.time() - frame['received_at'] <= STALE_S:
                    return frame
            if now - started > timeout_s:
                self._drop()
                raise CameraUnavailable(
                    f'相机服务在 {timeout_s:.0f} 秒内没有给出 {self.key} '
                    '的新画面'
                )
            time.sleep(POLL_S)

    def close(self) -> None:
        """断开共享内存；相机由相机服务到期后自行释放。"""
        self._drop()

