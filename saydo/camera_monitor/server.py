#!/usr/bin/env python3
"""Read-only, on-demand dual-camera MJPEG monitor. No robot writes."""

import argparse
import base64
import fcntl
import json
import logging
import os
import pathlib
import signal
import socket
import subprocess
import sys
import threading
import time
import uuid
import http.server
import urllib.parse
from typing import Optional

from saydo import paths
from saydo.contracts import camera_frames

# 预览页面与页面脚本所在的目录。
WEB = pathlib.Path(__file__).resolve().with_name('web')
PROJECT = paths.ROOT
LOG = logging.getLogger('camera_monitor')
# 页面与接口里各路相机的键。2 臂手上的相机只有配置了序列号才出现。
ARM2_CAMERA_KEY = 'd435i_arm2'
# 把相机让给采集程序的最长时间，单位秒。采集程序结束时会主动归还；
# 这个上限只是防止它异常退出后预览永远连不上。
MAX_HOLD_S = 60.
# 程序每次取画面时最多能让相机再保持采集多久，单位秒。程序每取一次
# 就续一次；它退出后相机在这段时间内自行释放。
MAX_KEEP_S = 60.
# 相机由取画面的请求启动时，最多等它多久出第一帧，单位秒。
START_TIMEOUT_S = 10.
# 等相机出一帧最多等多久，单位毫秒；超过就认为相机停帧了。
FRAME_TIMEOUT_MS = 3000
# 相机停帧后连续重开采集这么多次仍不出图，才认定相机不可用。USB 传输
# 拥堵时相机会停几秒到十几秒，重开一两次通常就恢复。
MAX_RESTARTS = 5
# 画面停了这么久之后，状态里的 `stalled` 为真，单位秒。
STALLED_AFTER_S = 2.
# 原始画面的共享内存放在哪个目录；`/dev/shm` 是系统提供的内存文件系统。
SHARED_DIRECTORY = pathlib.Path('/dev/shm')


def exposure_time(rs: object, color: object, received_at: float) -> float:
    """估计一帧的曝光时刻。

    相机给出的时间戳若已换算到主机时钟，就用它（比主机收到的时刻早
    几十毫秒，更接近实际曝光）；否则退回主机收到的时刻。

    Args:
        rs: `pyrealsense2` 模块。
        color: 彩色帧。
        received_at: 主机收到这一帧的时刻（`time.time()` 的值）。

    Returns:
        曝光时刻，与 `time.time()` 同一时间基准。
    """
    timestamp_s = color.get_timestamp() / 1000
    on_host_clock = color.get_frame_timestamp_domain() in (
        rs.timestamp_domain.global_time, rs.timestamp_domain.system_time
    )
    if on_host_clock and 0 <= received_at - timestamp_s < 10:
        return timestamp_s
    return received_at


def camera_users(serial):
    nodes = []
    for item in pathlib.Path('/sys/class/video4linux').glob('video*'):
        for parent in item.resolve().parents:
            vendor = parent / 'idVendor'
            device_serial = parent / 'serial'
            if vendor.exists() and device_serial.exists():
                if (
                    vendor.read_text().strip() == '8086'
                    and device_serial.read_text().strip() == serial
                ):
                    nodes.append('/dev/' + item.name)
                break
    if not nodes:
        return []
    result = subprocess.run(
        ['fuser', *nodes], capture_output=True, text=True, timeout=3
    )
    if result.returncode not in (0, 1):
        raise RuntimeError('无法检查相机占用状态，请检查 fuser。')
    return sorted(
        {
            int(p)
            for p in result.stdout.split()
            if p.isdigit() and int(p) != os.getpid()
        }
    )


class Camera:
    """一路相机的按需预览：有人看时才打开设备，最后一个人离开就释放。

    Attributes:
        camera_id: 这路相机在页面和接口里的键。
        name: 相机型号，用来在已连接的设备里找到它。
        role: `eye_in_hand`（装在机械臂手上）或 `movable_global`
            （不固定的全局相机）。
        arm: 装在哪台机械臂手上（`arm1`、`arm2`）；全局相机为 None。
        hold_until: 让给采集程序的截止时刻（`time.monotonic()` 的值）；
            在此之前不接受新的预览连接。
        keep_until: 程序要求保持采集到的时刻（`time.monotonic()` 的
            值）；在此之前即使没有人看预览也不释放相机。
        image: 本服务直接采集时最近一帧未压缩的图像（蓝绿红）；共享
            8089 的画面时为 None。
        intrinsics: 本服务直接采集时彩色画面的内参与畸变参数；共享
            8089 的画面时为 None。
        has_depth: 直接采集时是否同时采集深度图。默认不采：2026-10-10
            实测全局相机同时开彩色和深度（都是 1280×720）时，在现在的
            USB 接法下约四成的帧丢失，还会连续停十几秒。
        shared_frames: 本服务直接采集时，原始画面所在共享内存的路径
            和标识（每次重新打开相机都会变）；没有在采集或共享 8089
            的画面时为 None。读写约定见 `saydo/contracts/camera_frames.py`。
    """

    def __init__(
        self,
        width: int,
        height: int,
        fps: int,
        camera_id: str = 'd435i',
        serial: Optional[str] = None,
        tracker: Optional[object] = None,
        arm: Optional[str] = None,
        depth: bool = False,
    ) -> None:
        """创建一路预览；不打开设备。

        Args:
            width: 请求的图像宽度，单位像素。
            height: 请求的图像高度，单位像素。
            fps: 请求的帧率。
            camera_id: 这路相机的键；`d405` 是全局相机，其余都是手上
                的 D435i。
            serial: 相机序列号；None 表示只连着一台该型号时就用它。
            tracker: 8089 的客户端；给出时优先共享 8089 的画面而不
                打开设备。
            arm: 相机装在哪台机械臂手上；全局相机为 None。
            depth: 直接采集时是否同时采集深度图。
        """
        self.width, self.height, self.target_fps = width, height, fps
        self.requested_size = (width, height)
        self.cv = threading.Condition()
        self.thread = None
        self.stop_event = threading.Event()
        self.epoch = 0
        self.clients = 0
        self.state = 'idle'
        self.error = None
        self.jpeg = None
        self.sequence = 0
        self.frame_at = None
        self.fps = 0.0
        self.camera_id, self.serial, self.tracker = camera_id, serial, tracker
        is_global = camera_id == 'd405'
        self.name = 'RealSense D405' if is_global else 'RealSense D435I'
        self.role = 'movable_global' if is_global else 'eye_in_hand'
        self.arm = arm
        self.hold_until = 0.
        self.keep_until = 0.
        # 每次“停止预览”加一，用来让当时的预览连接结束。
        self.viewers = 0
        self.image = None
        self.intrinsics = None
        self.has_depth = depth
        self.shared_frames = None
        # 向 8089 要画面时的路径：1 臂是缺省的臂，不用写编号。
        self.frame_path = '/api/agent/frame' + (
            f'?arm={arm}' if arm not in (None, 'arm1') else ''
        )
        self.source = None
        self.frame_metadata = {}
        self.usb = None

    def status(self) -> dict:
        """返回这路相机的当前状态；不打开设备。

        Returns:
            状态、相机身份、帧率和最近一帧的时间等。`held_s` 是还要
            让给采集程序多少秒，0 表示没有被让出；`kept_s` 是程序
            要求再保持采集多少秒，0 表示没有程序在用；`stalled` 为真
            表示相机开着但已经有一阵没出图，本服务正在等它恢复。
        """
        with self.cv:
            return dict(
                state=self.state,
                arm=self.arm,
                held_s=round(
                    max(0., self.hold_until - time.monotonic()), 1
                ),
                kept_s=round(
                    max(0., self.keep_until - time.monotonic()), 1
                ),
                intrinsics=self.intrinsics,
                stalled=bool(
                    self.state == 'streaming'
                    and self.frame_at
                    and time.time() - self.frame_at > STALLED_AFTER_S
                ),
                shared_frames=self.shared_frames,
                error=self.error,
                camera=self.name,
                usb=self.usb,
                camera_id=self.camera_id,
                camera_serial=self.serial,
                camera_role=self.role,
                source=self.source,
                width=self.width,
                height=self.height,
                target_fps=self.target_fps,
                fps=round(self.fps, 1),
                clients=self.clients,
                sequence=self.sequence,
                frame_at=self.frame_at,
                frame_age_ms=(
                    round((time.time() - self.frame_at) * 1000)
                    if self.frame_at
                    else None
                ),
            )

    def hold(self, seconds: float) -> None:
        """把相机让给采集程序一段时间：这期间不接受新的预览连接。

        为什么需要它：页面在预览断开后会自动重连。如果不先挡住，
        重连会和采集程序抢同一台相机，让采集失败。

        Args:
            seconds: 让出的时长，单位秒；0 表示立即归还。超过上限
                `MAX_HOLD_S` 时按上限计。
        """
        seconds = min(max(0., float(seconds)), MAX_HOLD_S)
        with self.cv:
            self.hold_until = time.monotonic() + seconds if seconds else 0.

    def direct_codes(self) -> tuple:
        """8089 以哪些错误码拒绝时，改为由本服务直接打开相机。

        8089 没有运行时总是可以直接打开。2 臂的相机还多两种情况：8089
        没有启用 2 臂，或启用了但没有接管它的相机，这时相机是空闲的。
        其余的拒绝（认证失败、相机身份不符等）不能靠抢相机来解决。

        Returns:
            允许改为直接采集的错误码。
        """
        if self.arm in (None, 'arm1'):
            return ('BACKEND_UNAVAILABLE',)
        return ('BACKEND_UNAVAILABLE', 'UNKNOWN_ARM', 'CAMERA_UNAVAILABLE')

    def subscribe(self) -> int:
        """登记一个预览连接，必要时启动采集线程。

        Returns:
            本次采集的代次，断开时原样交还给 `unsubscribe`。

        Raises:
            RuntimeError: 相机已让给采集程序、正在释放，或连接数已满。
        """
        with self.cv:
            remaining = self.hold_until - time.monotonic()
            if remaining > 0:
                raise RuntimeError(
                    f'相机已让给采集程序，约 {remaining:.0f} 秒内归还。'
                )
            if self.clients >= 4:
                raise RuntimeError('同时预览上限为 4 个连接。')
            self.start()
            self.clients += 1
            return self.epoch

    def start(self) -> None:
        """没有采集线程时启动一个；调用者须持有 `cv`。

        Raises:
            RuntimeError: 上一次采集还在释放相机。
        """
        if self.thread and self.thread.is_alive():
            if self.stop_event.is_set():
                raise RuntimeError('相机正在释放，请稍后重试。')
            return
        self.stop_event = threading.Event()
        self.epoch += 1
        self.state, self.error, self.jpeg = 'starting', None, None
        self.frame_at, self.fps = None, 0.0
        self.thread = threading.Thread(
            target=self.capture,
            args=(self.epoch, self.stop_event),
            daemon=True,
        )
        self.thread.start()

    def keep(self, seconds: float) -> None:
        """应程序的要求保持采集一段时间，必要时启动相机。

        为什么需要它：一台相机同一时刻只能被一个进程打开。标定之类的
        程序如果自己去打开相机，预览就得先让出来，操作者摆放物品时
        看不到画面。让程序改为向本服务要画面，相机始终由本服务一家
        打开，预览和程序可以同时用。程序每取一次画面续一次期限；它
        退出后期限一到，没有人看预览的相机自行释放。

        Args:
            seconds: 从现在起至少再保持多久，单位秒；超过 `MAX_KEEP_S`
                时按上限计。

        Raises:
            RuntimeError: 相机已让给采集程序，或正在释放。
        """
        seconds = min(max(0., float(seconds)), MAX_KEEP_S)
        with self.cv:
            remaining = self.hold_until - time.monotonic()
            if remaining > 0:
                raise RuntimeError(
                    f'相机已让给采集程序，约 {remaining:.0f} 秒内归还。'
                )
            self.start()
            self.keep_until = max(
                self.keep_until, time.monotonic() + seconds
            )

    def wait_started(self, timeout_s: float) -> None:
        """等正在启动的相机出第一帧。

        Args:
            timeout_s: 最多等多久，单位秒。

        Raises:
            RuntimeError: 相机没有在采集（打不开或限时内没有出图）。
        """
        with self.cv:
            self.cv.wait_for(
                lambda: self.state != 'starting', timeout=timeout_s
            )
            if self.state != 'streaming':
                raise RuntimeError(
                    '相机不可用：' + (self.error or '限时内没有出图')
                )

    def keep_expired(self) -> bool:
        """程序要求的期限已过、又没有人在看预览时为真：该释放相机了。"""
        with self.cv:
            return (
                self.keep_until > 0
                and self.clients == 0
                and time.monotonic() >= self.keep_until
            )

    def publish(
        self,
        jpeg: bytes,
        captured_at: float,
        sequence: int,
        metadata: dict,
        source: str,
        width: int,
        height: int,
        image: Optional[object] = None,
    ) -> None:
        """发布新的一帧，唤醒等着它的预览连接和取图请求。

        Args:
            jpeg: 压缩后的图像。
            captured_at: 拍到这一帧的时刻（`time.time()` 的值）。
            sequence: 帧序号。
            metadata: 随取图请求一并给出的附加信息。
            source: 画面来源，`device`（本服务直接采集）或 `tracker`
                （转接 8089）。
            width: 图像宽度，单位像素。
            height: 图像高度，单位像素。
            image: 未压缩的图像；转接 8089 的画面没有，为 None。

        Raises:
            RuntimeError: 图像不是 JPEG，或拍摄时刻距今超过一秒。
        """
        if (
            not jpeg.startswith(b'\xff\xd8')
            or not 0 <= time.time() - captured_at <= 1
        ):
            raise RuntimeError('相机图像无效或超过一秒时效。')
        with self.cv:
            self.jpeg, self.frame_at, self.sequence = (
                jpeg, captured_at, sequence
            )
            self.image = image
            self.frame_metadata = metadata
            self.source, self.width, self.height = source, width, height
            self.state = 'streaming'
            self.cv.notify_all()

    def fresh_frame(
        self, lossless: bool = False, start_timeout_s: float = 0.
    ) -> dict:
        """等一帧在本次请求之后拍到的画面；本身不启动相机。

        Args:
            lossless: 为真时给出无损的 PNG 图像，供标定这类对像素位置
                敏感的用途；否则给出预览用的 JPEG。
            start_timeout_s: 相机正在启动时最多等它多久，单位秒；0
                表示不等，没有画面就立即报错。

        Returns:
            带相机身份、拍摄时刻、内参和图像（base64）的字典。

        Raises:
            RuntimeError: 相机没有在采集、没有等到新的一帧，或要求
                无损图像而这一路是转接 8089 的画面。
        """
        after = time.time()
        with self.cv:
            if start_timeout_s:
                self.cv.wait_for(
                    lambda: self.state != 'starting', timeout=start_timeout_s
                )
            if self.state != 'streaming':
                raise RuntimeError(
                    '相机不可用：' + self.error
                    if self.error
                    else '该相机没有活动预览。'
                )
            if lossless and self.image is None:
                raise RuntimeError(
                    '这一路是转接 8089 的画面，没有无损图像。'
                )
            self.cv.wait_for(
                lambda: (
                    self.frame_at is not None and self.frame_at >= after
                ) or self.state != 'streaming' or self.stop_event.is_set(),
                timeout=2,
            )
            if (
                self.stop_event.is_set()
                or self.state != 'streaming'
                or self.frame_at is None
                or self.frame_at < after
            ):
                raise RuntimeError('没有取得请求之后的新鲜相机帧。')
            image, encoded = self.image, self.jpeg
            frame = {
                **self.frame_metadata,
                'camera_role': self.role,
                'camera_serial': self.serial,
                'camera_name': self.name,
                'captured_at': self.frame_at,
                'sequence': self.sequence,
                'image_size': [self.width, self.height],
                'intrinsics': self.intrinsics,
                'mime_type': 'image/jpeg',
                'observation_id': uuid.uuid4().hex,
                'timestamp_basis': 'host_frame_received_unix_seconds',
                'capture_source': 'camera_monitor',
            }
        if lossless:
            import cv2
            # 压缩级别取 1：编码只要十几毫秒，图像仍是无损的。
            ok, png = cv2.imencode(
                '.png', image, [cv2.IMWRITE_PNG_COMPRESSION, 1]
            )
            if not ok:
                raise RuntimeError('图像编码失败。')
            frame['mime_type'], encoded = 'image/png', png.tobytes()
        frame['image_base64'] = base64.b64encode(encoded).decode()
        return frame

    def unsubscribe(self, epoch: int, viewers: Optional[int] = None) -> None:
        """注销一个预览连接；没有人看也没有程序在用时关闭相机。

        Args:
            epoch: `subscribe` 返回的代次；已不是当前代次时不做任何事。
            viewers: 连接建立时的 `viewers` 值；之后有人点过“停止预览”
                时这个连接已经被注销过，不再重复计数。
        """
        with self.cv:
            if epoch != self.epoch or viewers not in (None, self.viewers):
                return
            self.clients = max(0, self.clients - 1)
            # 程序还在用这台相机时，最后一个观看者离开也不释放。
            if self.clients == 0 and time.monotonic() >= self.keep_until:
                self.stop_event.set()
                self.cv.notify_all()

    def end_previews(self) -> dict:
        """结束这一路的全部预览连接；没有程序在取图时同时关闭相机。

        页面上的“停止预览”走这里。相机是预览和取图程序共用的：程序
        （标定、全局观察等）还在取图时相机继续采集，程序不受影响。

        Returns:
            这一路相机的状态。
        """
        with self.cv:
            shared = (
                time.monotonic() < self.keep_until
                and self.state in ('starting', 'streaming')
                and not self.stop_event.is_set()
            )
            if shared:
                self.clients = 0
                self.viewers += 1
                self.cv.notify_all()
        return self.status() if shared else self.stop()

    def stop(self) -> dict:
        """关闭相机并结束所有预览连接，不管有没有程序在取图。

        把相机整台借给采集程序、以及服务退出时用。

        Returns:
            这一路相机的状态。
        """
        with self.cv:
            self.stop_event.set()
            if self.state in ('starting', 'streaming'):
                self.state = 'stopping'
            self.cv.notify_all()
            thread = self.thread
        if thread:
            thread.join(timeout=6)
        return self.status()

    def capture(self, epoch: int, stop: threading.Event) -> None:
        """采集线程的入口：优先共享 8089 的画面，否则直接打开相机。

        本函数不抛出异常：出错时把原因写进 `error`、状态置为 `error`，
        并释放已经打开的设备。

        Args:
            epoch: 本次采集的代次。
            stop: 置位后结束采集。
        """
        pipeline = None
        device_lock = None
        started = False
        writer = None
        try:
            if self.tracker is not None:
                # Only GET the raw camera frame; never pause or reconfigure
                # 8089.
                from saydo.agent import client
                try:
                    first = self.tracker.request(self.frame_path)
                except client.BackendError as exc:
                    if exc.payload.get('code') not in self.direct_codes():
                        raise RuntimeError(
                            '8089 原始画面不可用：' + str(exc)
                        ) from exc
                else:
                    self.capture_tracker(first, stop)
                    return
            import cv2
            import numpy as np
            import pyrealsense2 as rs
            devices = rs.context().query_devices()
            matches = [
                d
                for d in devices
                if d.get_info(rs.camera_info.name) == self.name and (
                    self.serial is None
                    or d.get_info(rs.camera_info.serial_number) == self.serial
                )
            ]
            if len(matches) != 1:
                raise RuntimeError(
                    '没有找到唯一的指定相机，请检查连接或指定序列号。'
                )
            self.serial = matches[0].get_info(rs.camera_info.serial_number)
            if self.camera_id == 'd405':
                directory = PROJECT / 'work/global_camera'
                directory.mkdir(parents=True, exist_ok=True)
                device_lock = (directory / (self.serial + '.lock')).open('a')
                try:
                    fcntl.flock(device_lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
                except BlockingIOError as exc:
                    raise RuntimeError(
                        'D405 正在被全局观察使用，请稍后重试。'
                    ) from exc
            if camera_users(self.serial):
                raise RuntimeError(
                    '该相机已被其他程序占用；不会抢占或复位设备。'
                )
            pipeline = rs.pipeline()
            config = rs.config()
            config.enable_device(self.serial)
            config.enable_stream(
                rs.stream.color,
                *self.requested_size,
                rs.format.bgr8,
                self.target_fps,
            )
            align = None
            if self.has_depth:
                config.enable_stream(
                    rs.stream.depth,
                    *self.requested_size,
                    rs.format.z16,
                    self.target_fps,
                )
                # 把深度图逐像素对齐到彩色图，两者才能按同一组内参使用。
                align = rs.align(rs.stream.color)
            profile = pipeline.start(config)
            started = True
            if (
                (
                    profile.get_device().get_info(rs.camera_info.serial_number)
                    != self.serial
                )
                or (
                    profile.get_device().get_info(rs.camera_info.name)
                    != self.name
                )
            ):
                raise RuntimeError('实际打开的相机身份不匹配。')
            calibration = (
                profile.get_stream(rs.stream.color)
                .as_video_stream_profile()
                .get_intrinsics()
            )
            with self.cv:
                self.name = profile.get_device().get_info(rs.camera_info.name)
                self.usb = profile.get_device().get_info(
                    rs.camera_info.usb_type_descriptor
                )
                self.intrinsics = {
                    'fx': float(calibration.fx),
                    'fy': float(calibration.fy),
                    'cx': float(calibration.ppx),
                    'cy': float(calibration.ppy),
                    'distortion_model': str(calibration.model),
                    'distortion': [float(v) for v in calibration.coeffs],
                }
            if self.camera_id == 'd405':
                for _ in range(15):
                    if stop.is_set():
                        return
                    pipeline.wait_for_frames(3000)
            writer = camera_frames.FrameWriter(
                SHARED_DIRECTORY / f'saydo-camera-{self.camera_id}',
                *self.requested_size,
                self.intrinsics,
                self.serial,
                self.name,
                (
                    float(
                        profile.get_device()
                        .first_depth_sensor()
                        .get_depth_scale()
                    )
                    if self.has_depth
                    else None
                ),
            )
            with self.cv:
                self.shared_frames = {
                    'path': str(writer.path),
                    'id': f'{os.getpid()}-{epoch}',
                }
            last, count, written, restarts = time.monotonic(), 0, 0, 0
            while not stop.is_set():
                try:
                    frames = pipeline.wait_for_frames(FRAME_TIMEOUT_MS)
                except RuntimeError as exc:
                    # 相机一时不出图。重开采集而不是结束：观看连接和取图
                    # 程序都留着，相机恢复后接着用，不用人去重新点预览。
                    restarts += 1
                    if restarts > MAX_RESTARTS:
                        raise RuntimeError(
                            f'相机连续 {MAX_RESTARTS} 次重开后仍不出图：{exc}'
                        ) from exc
                    LOG.warning(
                        '%s 超过 %d 秒没有出图，重开采集（第 %d 次）：%s',
                        self.camera_id, FRAME_TIMEOUT_MS // 1000, restarts,
                        exc,
                    )
                    pipeline.stop()
                    started = False
                    if stop.wait(1):
                        break
                    pipeline.start(config)
                    started = True
                    continue
                if stop.is_set():
                    break
                if restarts:
                    LOG.info('%s 恢复出图', self.camera_id)
                    restarts = 0
                if align is not None:
                    frames = align.process(frames)
                color = frames.get_color_frame()
                depth = frames.get_depth_frame() if self.has_depth else None
                if not color or (self.has_depth and not depth):
                    continue
                captured_at = time.time()
                # 复制一份：相机驱动会回收这一帧的内存，而无损图像要
                # 留到有程序来取。
                image = np.asanyarray(color.get_data()).copy()
                written += 1
                writer.write(
                    image,
                    np.asanyarray(depth.get_data()) if depth else None,
                    written,
                    exposure_time(rs, color, captured_at),
                    captured_at,
                )
                ok, encoded = cv2.imencode(
                    '.jpg', image, [cv2.IMWRITE_JPEG_QUALITY, 82]
                )
                if not ok:
                    raise RuntimeError('图像编码失败。')
                count += 1
                now = time.monotonic()
                with self.cv:
                    if now - last >= 1:
                        self.fps, count, last = count / (now - last), 0, now
                # Discard a frame delayed by encoding/scheduling; keep waiting
                # for
                # fresh device frames instead of permanently ending this
                # preview.
                if not 0 <= time.time() - captured_at <= 1:
                    continue
                self.publish(
                    encoded.tobytes(),
                    captured_at,
                    color.get_frame_number(),
                    {
                        'sensor_timestamp_ms': color.get_timestamp(),
                        'sensor_timestamp_domain': str(
                            color.get_frame_timestamp_domain()
                        ),
                    },
                    'device',
                    color.get_width(),
                    color.get_height(),
                    image,
                )
                if self.keep_expired():
                    break
        except Exception as exc:
            LOG.warning('Camera stream stopped: %s', exc)
            with self.cv:
                self.error = str(exc)
                self.state = 'error'
        finally:
            if started:
                try:
                    pipeline.stop()
                except Exception as exc:
                    LOG.warning('Camera release: %s', exc)
            if device_lock:
                device_lock.close()
            if writer is not None:
                writer.close()
            with self.cv:
                self.shared_frames = None
                self.clients = 0
                if self.state != 'error':
                    self.state = 'idle'
                self.jpeg, self.frame_at, self.fps = None, None, 0.0
                self.image, self.keep_until = None, 0.
                self.cv.notify_all()

    def capture_tracker(self, first: dict, stop: threading.Event) -> None:
        """反复向 8089 取它的最新原始画面并发布；不打开设备。

        Args:
            first: 已经取到的第一帧。
            stop: 置位后结束。

        Raises:
            RuntimeError: 8089 给出的相机身份不符、图像无效或已过期。
        """
        last, count, data = time.monotonic(), 0, first
        while not stop.is_set():
            identity = data.get('camera') or {}
            if (
                identity.get('name') != self.name
                or not identity.get('serial')
                or (self.serial and identity['serial'] != self.serial)
            ):
                raise RuntimeError('8089 相机身份不匹配；不会显示另一台设备。')
            self.serial = identity['serial']
            jpeg = base64.b64decode(data['image_base64'], validate=True)
            if (
                len(jpeg) > 4 * 1024 * 1024
                or data.get('mime_type') != 'image/jpeg'
            ):
                raise RuntimeError('8089 相机图像格式无效。')
            if data['sequence'] != self.sequence or self.jpeg is None:
                self.publish(
                    jpeg,
                    data['captured_at'],
                    data['sequence'],
                    {},
                    'tracker',
                    data['width'],
                    data['height'],
                )
                count += 1
            elif not 0 <= time.time() - data['captured_at'] <= 1:
                raise RuntimeError('8089 相机帧已过期。')
            now = time.monotonic()
            if now - last >= 1:
                with self.cv:
                    self.fps = count / (now - last)
                count, last = 0, now
            if self.keep_expired() or stop.wait(1 / self.target_fps):
                break
            data = self.tracker.request(self.frame_path)


def monitor_status(cameras):
    rows = {key: camera.status() for key, camera in cameras.items()}
    state = next(
        (
            state
            for state in ('streaming', 'starting', 'stopping', 'error')
            if any(row['state'] == state for row in rows.values())
        ),
        'idle',
    )
    return {**rows['d435i'], 'state': state, 'cameras': rows}


class Handler(http.server.BaseHTTPRequestHandler):
    """预览服务的请求处理：页面、状态、画面流、截图和释放预览。"""

    protocol_version = 'HTTP/1.1'

    def log_message(self, fmt: str, *args: object) -> None:
        """记录一次请求；周期性的查询和取图不记，免得日志无限增长。

        页面每秒查一次状态，取图程序每秒取图、每几秒续一次期限。
        相机因取图请求而启动时另有一条日志。
        """
        request = str(args[0])
        routine = (
            '/status ' in request
            or '/keep ' in request
            or 'keep_s=' in request
        )
        if not routine or not str(args[1]).startswith('2'):
            LOG.info('%s %s', self.client_address[0], fmt % args)

    def send_body(
        self,
        code,
        body,
        kind='application/json; charset=utf-8',
        disposition=None,
    ):
        self.send_response(code)
        self.send_header('Content-Type', kind)
        self.send_header('Content-Length', str(len(body)))
        self.send_header('Cache-Control', 'no-store')
        self.send_header('X-Content-Type-Options', 'nosniff')
        if disposition:
            self.send_header('Content-Disposition', disposition)
        self.end_headers()
        self.wfile.write(body)

    def send_frame(self, camera: Camera, query: dict) -> None:
        """回应取一帧画面的请求。

        不带参数时只在已有预览的情况下给出 JPEG，不启动相机。带
        `keep_s` 时必要时启动相机，并让它至少再保持采集这么多秒；
        带 `format=png` 时给出无损图像。

        Args:
            camera: 目标相机。
            query: 地址里的查询参数。
        """
        try:
            keep_s = float(query.get('keep_s', ['0'])[0])
            image_format = query.get('format', ['jpeg'])[0]
            if not 0 <= keep_s <= MAX_KEEP_S:
                raise ValueError(f'keep_s 必须在 0 到 {MAX_KEEP_S:.0f} 之间')
            if image_format not in ('jpeg', 'png'):
                raise ValueError('format 只能是 jpeg 或 png')
        except ValueError as exc:
            return self.send_body(
                400, json.dumps({'error': str(exc)}).encode()
            )
        if keep_s and self.headers.get('Sec-Fetch-Site') == 'cross-site':
            return self.send_body(
                403, b'{"error":"Cross-site camera start denied"}'
            )
        try:
            if keep_s:
                self.start_for_program(camera, keep_s)
            frame = camera.fresh_frame(
                lossless=image_format == 'png',
                start_timeout_s=START_TIMEOUT_S if keep_s else 0.,
            )
        except RuntimeError as exc:
            return self.send_body(
                409, json.dumps({'error': str(exc)}).encode()
            )
        self.send_body(200, json.dumps(frame).encode())

    def keep_camera(self, camera: Camera, keep_s: float) -> None:
        """应取图程序的要求保持采集，并回应这一路相机的状态。

        Args:
            camera: 目标相机。
            keep_s: 至少再保持采集多久，单位秒。
        """
        try:
            self.start_for_program(camera, keep_s)
            camera.wait_started(START_TIMEOUT_S)
        except RuntimeError as exc:
            return self.send_body(
                409, json.dumps({'error': str(exc)}).encode()
            )
        self.send_body(200, json.dumps(camera.status()).encode())

    def start_for_program(self, camera: Camera, keep_s: float) -> None:
        """续上程序要求的期限；相机没开时打开它并记一条日志。

        Args:
            camera: 目标相机。
            keep_s: 至少再保持采集多久，单位秒。

        Raises:
            RuntimeError: 相机已让给采集程序，或正在释放。
        """
        if camera.status()['state'] not in ('starting', 'streaming'):
            LOG.info('%s 应程序的取图请求启动采集', camera.camera_id)
        camera.keep(keep_s)

    def do_GET(self) -> None:
        """处理页面、状态、预览流、截图和取图请求；不控制机械臂。"""
        url = urllib.parse.urlsplit(self.path)
        path = url.path
        if path == '/':
            return self.send_body(
                200,
                (WEB / 'index.html').read_bytes(),
                'text/html; charset=utf-8',
            )
        if path == '/app.js':
            return self.send_body(
                200,
                (WEB / 'app.js').read_bytes(),
                'text/javascript; charset=utf-8',
            )
        if path == '/api/status':
            return self.send_body(
                200, json.dumps(monitor_status(self.server.cameras)).encode()
            )
        routes = {
            '/stream': ('d435i', 'stream'),
            '/snapshot.jpg': ('d435i', 'snapshot'),
        }
        for key in self.server.cameras:
            routes.update(
                {
                    f'/stream/{key}': (key, 'stream'),
                    f'/snapshot/{key}.jpg': (key, 'snapshot'),
                    f'/api/cameras/{key}/status': (key, 'status'),
                    f'/api/cameras/{key}/frame': (key, 'frame'),
                }
            )
        if path not in routes:
            return self.send_body(404, b'{"error":"Not found"}')
        key, operation = routes[path]
        camera = self.server.cameras[key]
        if operation == 'status':
            return self.send_body(200, json.dumps(camera.status()).encode())
        if operation == 'frame':
            return self.send_frame(
                camera, urllib.parse.parse_qs(url.query)
            )
        if operation == 'snapshot':
            with camera.cv:
                jpeg = (
                    camera.jpeg
                    if (
                        camera.state == 'streaming'
                        and camera.frame_at
                        and 0 <= time.time() - camera.frame_at <= 1
                    )
                    else None
                )
            return (
                self.send_body(
                    200,
                    jpeg,
                    'image/jpeg',
                    f'attachment; filename="{key}-snapshot.jpg"',
                )
                if jpeg
                else self.send_body(409, b'{"error":"Start preview first"}')
            )
        if self.headers.get('Sec-Fetch-Site') == 'cross-site':
            return self.send_body(
                403, b'{"error":"Cross-site streaming denied"}'
            )
        try:
            epoch = camera.subscribe()
            viewers = camera.viewers
        except RuntimeError as exc:
            return self.send_body(409, json.dumps({'error': str(exc)}).encode())
        try:
            self.connection.settimeout(5)
            self.send_response(200)
            self.send_header(
                'Content-Type', 'multipart/x-mixed-replace; boundary=frame'
            )
            self.send_header('Cache-Control', 'no-store')
            self.send_header('Connection', 'close')
            self.end_headers()
            previous = -1
            while True:
                with camera.cv:
                    camera.cv.wait_for(
                        lambda: (
                            camera.jpeg is not None
                            and camera.sequence != previous
                        ) or camera.stop_event.is_set() or camera.state in (
                            'error', 'idle'
                        ) or camera.viewers != viewers,
                        timeout=4,
                    )
                    if (
                        epoch != camera.epoch
                        or camera.viewers != viewers
                        or camera.stop_event.is_set()
                        or camera.state in ('error', 'idle')
                    ):
                        break
                    jpeg, sequence = camera.jpeg, camera.sequence
                if not jpeg or sequence == previous:
                    continue
                self.wfile.write(
                    b'--frame\r\nContent-Type: image/jpeg\r\nContent-Length: '
                    + str(len(jpeg)).encode()
                    + b'\r\nX-Sequence: '
                    + str(sequence).encode()
                    + b'\r\n\r\n'
                    + jpeg
                    + b'\r\n'
                )
                self.wfile.flush()
                previous = sequence
        except (
            BrokenPipeError, ConnectionResetError, TimeoutError, socket.timeout
        ):
            pass
        finally:
            camera.unsubscribe(epoch, viewers)
            self.close_connection = True

    def do_POST(self) -> None:
        """处理停止预览、让出相机、归还相机和保持采集；不控制机械臂。

        `/api/pause` 停止全部预览，`/api/cameras/<键>/pause` 停止一路：
        结束预览连接，没有程序在取图的相机同时关闭。请求体里带
        `{"hold_s": 秒数}` 时是把相机整台让给采集程序，一定关闭。
        `/api/cameras/<键>/resume` 提前归还。`/api/cameras/<键>/keep`
        带 `{"keep_s": 秒数}`：应取图程序的要求保持采集，必要时打开
        相机，等它出图后返回这一路的状态（含共享内存的位置）。
        """
        cameras = self.server.cameras
        routes = {'/api/pause': ('pause', list(cameras))}
        for key in cameras:
            routes[f'/api/cameras/{key}/pause'] = ('pause', [key])
            routes[f'/api/cameras/{key}/resume'] = ('resume', [key])
            routes[f'/api/cameras/{key}/keep'] = ('keep', [key])
        if self.path not in routes:
            return self.send_body(404, b'{"error":"Not found"}')
        origin = self.headers.get('Origin')
        if origin and origin != 'http://' + self.headers.get('Host', ''):
            return self.send_body(403, b'{"error":"Origin mismatch"}')
        operation, keys = routes[self.path]
        hold_s = 0.
        field = 'keep_s' if operation == 'keep' else 'hold_s'
        try:
            size = int(self.headers.get('Content-Length') or 0)
            if not 0 <= size <= 200:
                raise ValueError('请求体过大')
            body = json.loads(self.rfile.read(size) or b'{}')
            if not isinstance(body, dict) or set(body) - {field}:
                raise ValueError(f'只接受 {field}')
            hold_s = body.get(field, 0)
            if type(hold_s) not in (int, float) or not 0 <= hold_s:
                raise ValueError(f'{field} 必须是非负数')
            if operation == 'keep' and not 0 < hold_s <= MAX_KEEP_S:
                raise ValueError(
                    f'keep_s 必须大于 0、不超过 {MAX_KEEP_S:.0f}'
                )
        except ValueError as exc:
            return self.send_body(
                400, json.dumps({'error': str(exc)}).encode()
            )
        if operation == 'keep':
            return self.keep_camera(cameras[keys[0]], hold_s)
        for key in keys:
            if operation == 'resume':
                cameras[key].hold(0)
                continue
            # 先挡住新连接再停，避免页面在两步之间重连进来。
            if hold_s and len(keys) == 1:
                cameras[key].hold(hold_s)
                LOG.info('%s 让给采集程序 %.0f 秒', key, hold_s)
                cameras[key].stop()
            else:
                cameras[key].end_previews()
        self.send_body(200, json.dumps(monitor_status(cameras)).encode())


def main() -> None:
    """启动相机预览服务；只读取相机，不控制机械臂。"""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--host', default='0.0.0.0')
    parser.add_argument('--port', type=int, default=8088)
    parser.add_argument('--width', type=int, default=1280)
    parser.add_argument('--height', type=int, default=720)
    parser.add_argument('--fps', type=int, default=15)
    parser.add_argument('--eye-in-hand-serial')
    parser.add_argument('--fixed-serial')
    parser.add_argument(
        '--arm2-serial',
        help=(
            '2 臂手上 D435i 的序列号；缺省读取配置里的 '
            'arm2_eye_in_hand_serial，都没有时不提供这一路'
        ),
    )
    parser.add_argument(
        '--global-depth',
        action='store_true',
        help=(
            '直接采集全局相机时同时采集深度图并写入共享内存；默认不开，'
            '见 docs/camera_monitor.md 里的丢帧说明'
        ),
    )
    parser.add_argument(
        '--tracker-url',
        help=(
            '8089 HTTP origin; share its raw D435i frames without opening that '
            'device'
        ),
    )
    parser.add_argument(
        '--token-file',
        type=pathlib.Path,
        default=(
            pathlib.Path.home() / '.local/share/saydo-mcp/robot-backend-token'
        ),
    )
    args = parser.parse_args()
    if min(args.width, args.height, args.fps) <= 0:
        parser.error('width、height、fps 必须为正整数')
    config_path = PROJECT / 'config/cameras/global.json'
    settings = (
        json.loads(config_path.read_text()) if config_path.exists() else {}
    )
    eye_serial = args.eye_in_hand_serial or settings.get('eye_in_hand_serial')
    fixed_serial = (
        args.fixed_serial
        or (
            settings.get('serial')
            if settings.get('camera_name') == 'RealSense D405'
            else None
        )
    )
    arm2_serial = args.arm2_serial or settings.get('arm2_eye_in_hand_serial')
    known = [s for s in (eye_serial, fixed_serial, arm2_serial) if s]
    if len(set(known)) != len(known):
        parser.error('各路相机的序列号不能相同')
    if arm2_serial and not eye_serial:
        # 两台 D435i 同时连着时，1 臂那一路也必须写明序列号，否则它在
        # 8089 停止时会因为“找不到唯一的 D435I”而无法直接采集。
        parser.error('提供 2 臂相机时，必须同时指定 1 臂相机的序列号')
    tracker = None
    if args.tracker_url:
        from saydo.agent import client
        tracker = client.RobotBackend(
            args.tracker_url, args.token_file, timeout=2
        )
    logging.basicConfig(
        level=logging.INFO, format='%(asctime)s %(levelname)s %(message)s'
    )
    server = http.server.ThreadingHTTPServer((args.host, args.port), Handler)
    server.daemon_threads = True
    size = (args.width, args.height, args.fps)
    server.cameras = {
        'd435i': Camera(*size, 'd435i', eye_serial, tracker, arm='arm1'),
        'd405': Camera(
            *size, 'd405', fixed_serial, depth=args.global_depth
        ),
    }
    if arm2_serial:
        # 8089 接管了 2 臂相机时共享它的画面，否则由本服务直接采集。
        server.cameras[ARM2_CAMERA_KEY] = Camera(
            *size, ARM2_CAMERA_KEY, arm2_serial, tracker, arm='arm2'
        )

    def shutdown(*_):
        threading.Thread(target=server.shutdown, daemon=True).start()

    signal.signal(signal.SIGTERM, shutdown)
    signal.signal(signal.SIGINT, shutdown)
    LOG.info(
        'Monitor listening on %s:%s; camera stays idle until /stream is opened',
        args.host,
        args.port,
    )
    LOG.info(
        '提供的相机：%s；全局相机的深度图：%s',
        '，'.join(
            f'{key}={camera.serial or "自动"}'
            for key, camera in server.cameras.items()
        ),
        '采集' if args.global_depth else '不采集',
    )
    try:
        server.serve_forever()
    finally:
        for camera in server.cameras.values():
            camera.stop()
        server.server_close()


if __name__ == '__main__':
    main()
