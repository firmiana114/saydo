"""系统边界上的替身：扮演 RealSense 库，按固定节奏给出彩色与深度帧。

只实现取图线程用到的那部分接口。被测的取图、整理画面和场景采集代码
全部是真实的。
"""

from __future__ import annotations

import threading
import types
from typing import Any, Dict, Optional

import numpy as np

COLOR_INTRINSICS = types.SimpleNamespace(
    width=1280, height=720, fx=910., fy=908., ppx=646., ppy=368.,
    coeffs=[0.] * 5,
)
# 深度原始值乘以这个系数得到米；这里取 1 毫米一个单位。
DEPTH_SCALE_M = .001
DEPTH_RAW = 400


class _Frame:
    def __init__(self, number: int, data: np.ndarray) -> None:
        self.number, self.data = number, data
        self.profile = types.SimpleNamespace(
            as_video_stream_profile=lambda: types.SimpleNamespace(
                intrinsics=COLOR_INTRINSICS
            )
        )

    def get_frame_number(self) -> int:
        """返回相机给这一帧的编号。"""
        return self.number

    def get_timestamp(self) -> float:
        """返回这一帧的时间戳，单位毫秒；替身里只用来比较彩色与深度。"""
        return float(self.number)

    def get_frame_timestamp_domain(self) -> str:
        """返回时间戳的时钟来源；替身用硬件时钟，取图代码会改用接收时间。"""
        return 'hardware_clock'

    def get_data(self) -> np.ndarray:
        """返回图像数据。"""
        return self.data


class _Frames:
    def __init__(self, number: int) -> None:
        self.color = _Frame(number, np.full((720, 1280, 3), 90, np.uint8))
        self.depth = _Frame(
            number, np.full((720, 1280), DEPTH_RAW, np.uint16)
        )

    def get_color_frame(self) -> _Frame:
        """返回彩色帧。"""
        return self.color

    def get_depth_frame(self) -> _Frame:
        """返回深度帧。"""
        return self.depth


class FakeRealSense:
    """RealSense 库的替身。

    Attributes:
        devices: 已连接相机的序列号到型号名；测试可以增删来模拟插拔。
        started: 每次打开相机时记录的序列号。
        stopped: 关闭相机的次数。
        fail_after: 给出这么多帧之后让取帧报错，模拟相机掉线；None 表示
            不报错。
    """

    def __init__(self, devices: Dict[str, str]) -> None:
        """创建替身。

        Args:
            devices: 已连接相机的序列号到型号名。
        """
        self.devices = dict(devices)
        self.started = []
        self.stopped = 0
        self.fail_after: Optional[int] = None
        self.frame_ready = threading.Event()
        self.camera_info = types.SimpleNamespace(
            serial_number='serial_number', name='name'
        )
        self.stream = types.SimpleNamespace(color='color', depth='depth')
        self.format = types.SimpleNamespace(bgr8='bgr8', z16='z16')
        self.timestamp_domain = types.SimpleNamespace(
            global_time='global_time', system_time='system_time'
        )

    def context(self) -> Any:
        """返回可以枚举已连接相机的对象。"""
        found = [
            types.SimpleNamespace(
                get_info=lambda key, s=serial, n=name: (
                    s if key == 'serial_number' else n
                )
            )
            for serial, name in self.devices.items()
        ]
        return types.SimpleNamespace(query_devices=lambda: found)

    def config(self) -> Any:
        """返回一份取流配置；替身只记下要打开哪台相机。"""
        owner = self
        settings = types.SimpleNamespace(serial=None)

        def enable_device(serial: str) -> None:
            settings.serial = serial

        settings.enable_device = enable_device
        settings.enable_stream = lambda *unused: None
        settings.owner = owner
        return settings

    def align(self, unused_stream: str) -> Any:
        """返回对齐器；替身里彩色与深度本来就是同尺寸，原样返回。"""
        return types.SimpleNamespace(process=lambda frames: frames)

    def pipeline(self) -> Any:
        """返回一条取流管线。"""
        owner = self
        count = [0]

        def start(settings: Any) -> Any:
            owner.started.append(settings.serial)
            sensor = types.SimpleNamespace(
                get_depth_scale=lambda: DEPTH_SCALE_M
            )
            device = types.SimpleNamespace(first_depth_sensor=lambda: sensor)
            return types.SimpleNamespace(get_device=lambda: device)

        def wait_for_frames(unused_timeout_ms: int) -> _Frames:
            if owner.fail_after is not None and count[0] >= owner.fail_after:
                raise RuntimeError("Frame didn't arrive within 4000")
            count[0] += 1
            # 稍等一下，免得取图线程空转占满处理器。
            owner.frame_ready.wait(.005)
            return _Frames(count[0])

        def stop() -> None:
            owner.stopped += 1

        return types.SimpleNamespace(
            start=start,
            wait_for_frames=wait_for_frames,
            poll_for_frames=lambda: None,
            stop=stop,
        )
