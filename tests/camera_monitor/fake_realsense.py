"""测试用：代替相机驱动 `pyrealsense2` 的假设备。

相机是系统边界。这里只模拟预览服务用到的那几个驱动调用，让它“打开”
一台给出固定画面的相机；预览服务的采集、编码、发布和释放都是真实
代码。
"""

from __future__ import annotations

import pathlib
import time
import types
from typing import List, Optional
from unittest import mock

import numpy as np

# 假设备每隔这么久出一帧，单位秒。
FRAME_PERIOD_S = .01
# 假设备的曝光时刻比主机收到的时刻早这么多，单位秒。
EXPOSURE_LEAD_S = .03
# 深度图里一个单位是多少米；与 D405 的出厂设置相同。
DEPTH_SCALE_M = .0001


class Device:
    """一台假相机：只有型号、序列号和接口类型。"""

    def __init__(self, name: str, serial: str) -> None:
        """记下型号与序列号。

        Args:
            name: 型号。
            serial: 序列号。
        """
        self.name, self.serial = name, serial

    def get_info(self, key: str) -> str:
        """按驱动的写法返回一项设备信息。"""
        return {'name': self.name, 'serial_number': self.serial}.get(
            key, '3.2'
        )

    def first_depth_sensor(self) -> types.SimpleNamespace:
        """返回深度传感器；只能问它深度图的单位。"""
        return types.SimpleNamespace(get_depth_scale=lambda: DEPTH_SCALE_M)


class Module:
    """假的 `pyrealsense2` 模块；用 `mock.patch.dict('sys.modules')` 换入。

    Attributes:
        image: 假相机拍到的画面（蓝绿红），可随时替换。
        depth: 假相机给出的深度图（原始单位），可随时替换。
        stall: 接下来有几次等帧会超时，模拟相机一时不出图。
        started: 相机被打开过几次。
        stopped: 相机被释放过几次。
    """

    camera_info = types.SimpleNamespace(
        name='name',
        serial_number='serial_number',
        usb_type_descriptor='usb_type_descriptor',
    )
    stream = types.SimpleNamespace(color='color', depth='depth')
    format = types.SimpleNamespace(bgr8='bgr8', z16='z16')
    timestamp_domain = types.SimpleNamespace(
        global_time='global_time', system_time='system_time'
    )

    def __init__(
        self,
        image: np.ndarray,
        name: str = 'RealSense D405',
        serial: str = '405123456789',
        intrinsics: Optional[types.SimpleNamespace] = None,
    ) -> None:
        """准备一台假相机。

        Args:
            image: 它拍到的画面。
            name: 型号。
            serial: 序列号。
            intrinsics: 彩色画面的内参与畸变；缺省给一组无畸变的值。
        """
        self.image = image
        self.depth = np.full(image.shape[:2], 8000, np.uint16)
        self.device = Device(name, serial)
        self.intrinsics = intrinsics or types.SimpleNamespace(
            fx=650., fy=651., ppx=640., ppy=360.,
            model='distortion.inverse_brown_conrady', coeffs=[0.] * 5,
        )
        self.started = 0
        self.stopped = 0
        self.sequence = 0
        self.stall = 0

    def context(self) -> types.SimpleNamespace:
        """返回只连着这一台相机的设备列表。"""
        return types.SimpleNamespace(query_devices=lambda: [self.device])

    def config(self) -> types.SimpleNamespace:
        """返回采集配置；假设备不看它的内容。"""
        return types.SimpleNamespace(
            enable_device=lambda serial: None,
            enable_stream=lambda *settings: None,
        )

    def pipeline(self) -> types.SimpleNamespace:
        """返回一条采集管线。"""
        return types.SimpleNamespace(
            start=self._start, wait_for_frames=self._wait, stop=self._stop
        )

    def align(self, target: str) -> types.SimpleNamespace:
        """返回对齐器；假设备的深度图本来就与彩色图对齐。"""
        return types.SimpleNamespace(process=lambda frames: frames)

    def _start(self, config: object) -> types.SimpleNamespace:
        self.started += 1
        stream = types.SimpleNamespace(
            as_video_stream_profile=lambda: types.SimpleNamespace(
                get_intrinsics=lambda: self.intrinsics
            )
        )
        return types.SimpleNamespace(
            get_device=lambda: self.device, get_stream=lambda kind: stream
        )

    def _wait(self, timeout_ms: int) -> types.SimpleNamespace:
        time.sleep(FRAME_PERIOD_S)
        if self.stall > 0:
            self.stall -= 1
            raise RuntimeError(f"Frame didn't arrive within {timeout_ms}")
        self.sequence += 1
        height, width = self.image.shape[:2]
        sequence, image, depth = self.sequence, self.image, self.depth
        exposed_ms = (time.time() - EXPOSURE_LEAD_S) * 1000
        color = types.SimpleNamespace(
            get_data=lambda: image,
            get_frame_number=lambda: sequence,
            get_timestamp=lambda: exposed_ms,
            get_frame_timestamp_domain=lambda: 'system_time',
            get_width=lambda: width,
            get_height=lambda: height,
        )
        return types.SimpleNamespace(
            get_color_frame=lambda: color,
            get_depth_frame=lambda: types.SimpleNamespace(
                get_data=lambda: depth
            ),
        )

    def _stop(self) -> None:
        self.stopped += 1


def patches(module: Module, monitor: object, directory: str) -> List[object]:
    """返回让预览服务用上假相机所需的全部替换。

    Args:
        module: 假的驱动模块。
        monitor: 预览服务模块 `saydo.camera_monitor.server`。
        directory: 临时目录，全局相机的设备锁文件和原始画面的共享
            内存文件放在里面，不碰正在运行的相机服务的那一份。

    Returns:
        一组 `mock.patch` 对象，由调用者逐个启动和停止。
    """
    return [
        mock.patch.dict('sys.modules', {'pyrealsense2': module}),
        mock.patch.object(monitor, 'PROJECT', pathlib.Path(directory)),
        mock.patch.object(
            monitor, 'SHARED_DIRECTORY', pathlib.Path(directory)
        ),
        # 查占用要读系统的设备文件并运行 fuser，假相机没有这些。
        mock.patch.object(monitor, 'camera_users', return_value=[]),
    ]
