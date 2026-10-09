"""第二台及以后的机械臂手上相机的取图线程。

8089 的跟踪器（`live_tracker.Tracker`）自己读取 1 臂手上的相机。其余的臂
由本模块各开一条取图线程，把最新的一帧彩色和深度图交给这台臂的视觉
来源（`arm_vision.ArmVision`，本类继承它）。这一帧有两个用途：

- 场景快照：拿它去定位物品。定位用的模型是跟踪器里那一份（显存只够
  放一份），所以定位请求转交给跟踪器处理。
- 实时跟踪：操作者在页面上把当前机械臂切换到这台臂时，画面同时送进
  跟踪器，由它做实时跟踪。

与 1 臂的取图不同，这里相机出错不会让整个服务停止：记录后隔几秒重试，
其他臂的工作不受影响。
"""

from __future__ import annotations

import time
import traceback
from typing import Any, Optional

import cv2
import numpy as np

from saydo.perception import vision_refinement
from saydo.robot import arms
from saydo.workbench import arm_vision

# 与跟踪器对 1 臂相机的取流设置一致：彩色 1280×720、深度 640×360、15 帧/秒，
# 对齐后缩到 640×360 交给定位模型。两台臂的画面规格相同，同一套模型和
# 内参检查才能通用。
COLOR_SIZE_PX = (1280, 720)
DEPTH_SIZE_PX = (640, 360)
IMAGE_SIZE_PX = (640, 360)
FRAME_RATE = 15
# 打开相机后先丢掉的帧数，等自动曝光稳定。
WARMUP_FRAMES = 20
FRAME_TIMEOUT_MS = 4000
# 彩色与深度帧时间戳相差超过这个值就不配对，单位毫秒。
MAX_COLOR_DEPTH_GAP_MS = 100
# 相机出错后隔多久重试，单位秒。
RETRY_INTERVAL_S = 5.


class ArmCamera(arm_vision.ArmVision):
    """一台机械臂手上相机的取图线程，同时是这台臂的视觉来源。

    Attributes:
        serial: 相机序列号。
        depth_quadratic_per_m: 深度读数的二次项校正系数。
    """

    def __init__(
        self,
        tracker: Any,
        arm: arms.Arm,
        serial: str,
        depth_quadratic_per_m: float = 0.,
    ) -> None:
        """创建取图对象；不打开设备，调用 `run` 后才开始取图。

        Args:
            tracker: 8089 的跟踪器。用它的定位模型、事件日志、停止标志，
                并从它那里取得已加载的 RealSense 库。
            arm: 相机装在哪台臂手上。
            serial: 相机序列号。
            depth_quadratic_per_m: 这台相机深度读数的二次项校正系数，
                取自该臂的相机校正文件；0 表示不校正。
        """
        super().__init__(tracker, arm)
        self.depth_quadratic_per_m = float(depth_quadratic_per_m)
        self.serial = serial
        self._last_logged_error: Optional[str] = None

    def run(self) -> None:
        """取图线程的入口：持续取图，出错后隔几秒重试，直到服务停止。"""
        realsense = self.tracker.realsense_module()
        while not self.tracker.stop.is_set():
            if realsense is None:
                self._record_failure('RealSense 库未能加载（1 臂相机未启动）')
                if self.tracker.stop.wait(RETRY_INTERVAL_S):
                    return
                realsense = self.tracker.realsense_module()
                continue
            try:
                self._stream(realsense)
            except Exception as error:
                # 线程入口：这台相机的任何错误都不能带停整个服务，记录后
                # 重试。
                self._record_failure(str(error), traceback.format_exc())
            self.tracker.stop.wait(RETRY_INTERVAL_S)

    def _record_failure(self, message: str, trace: str = '') -> None:
        """把相机错误写进状态；同一条错误只记一次日志，避免重试时刷屏。"""
        self.camera_failed(message)
        if message != self._last_logged_error:
            self._last_logged_error = message
            self.tracker.event(
                'arm_camera_failed',
                arm=self.arm.arm_id,
                serial=self.serial,
                error=message,
                traceback=trace,
            )

    def _stream(self, rs: Any) -> None:
        """打开相机并持续发布画面；相机出错时抛出异常。"""
        pipeline = rs.pipeline()
        started = False
        try:
            names = {
                device.get_info(rs.camera_info.serial_number): device.get_info(
                    rs.camera_info.name
                )
                for device in rs.context().query_devices()
            }
            if self.serial not in names:
                connected = '、'.join(sorted(names)) or '无'
                raise ValueError(
                    f'没有找到{self.arm.label}的相机 {self.serial}；'
                    f'已连接的相机：{connected}'
                )
            identity = {'serial': self.serial, 'name': names[self.serial]}
            config = rs.config()
            config.enable_device(self.serial)
            config.enable_stream(
                rs.stream.color, *COLOR_SIZE_PX, rs.format.bgr8, FRAME_RATE
            )
            config.enable_stream(
                rs.stream.depth, *DEPTH_SIZE_PX, rs.format.z16, FRAME_RATE
            )
            profile = pipeline.start(config)
            started = True
            depth_scale_m = (
                profile.get_device().first_depth_sensor().get_depth_scale()
            )
            align = rs.align(rs.stream.color)
            for _ in range(WARMUP_FRAMES):
                pipeline.wait_for_frames(FRAME_TIMEOUT_MS)
            sequence = 0
            last_number = None
            while not self.tracker.stop.is_set():
                frames = pipeline.wait_for_frames(FRAME_TIMEOUT_MS)
                # 取队列里最新的一组，不用排在前面的旧帧。
                for _ in range(8):
                    newer = pipeline.poll_for_frames()
                    if not newer:
                        break
                    frames = newer
                received_at = time.time()
                color = frames.get_color_frame()
                depth = frames.get_depth_frame()
                if color.get_frame_number() == last_number:
                    continue
                if (
                    abs(color.get_timestamp() - depth.get_timestamp())
                    > MAX_COLOR_DEPTH_GAP_MS
                ):
                    continue
                last_number = color.get_frame_number()
                sequence += 1
                frame = self._frame(
                    rs, align.process(frames), identity, depth_scale_m,
                    sequence, received_at,
                )
                self._publish(frame)
        finally:
            if started:
                pipeline.stop()

    def _frame(
        self,
        rs: Any,
        aligned: Any,
        identity: dict,
        depth_scale_m: float,
        sequence: int,
        received_at: float,
    ) -> dict:
        """把一组已对齐的彩色与深度帧整理成与跟踪器相同格式的一帧。"""
        color, depth = aligned.get_color_frame(), aligned.get_depth_frame()
        intrinsics = color.profile.as_video_stream_profile().intrinsics
        if any(abs(value) > 1e-12 for value in intrinsics.coeffs):
            raise ValueError('彩色图带有畸变参数，需要先做去畸变，当前不支持')
        # 图像从 1280×720 缩到 640×360，内参随之减半。
        intrinsic_matrix = np.array(
            [
                [intrinsics.fx / 2, 0, intrinsics.ppx / 2],
                [0, intrinsics.fy / 2, intrinsics.ppy / 2],
                [0, 0, 1.],
            ]
        )
        timestamp_s = color.get_timestamp() / 1000
        wall_clock = color.get_frame_timestamp_domain() in (
            rs.timestamp_domain.global_time, rs.timestamp_domain.system_time
        )
        captured_at = (
            timestamp_s
            if wall_clock and 0 <= received_at - timestamp_s < 10
            else received_at
        )
        return {
            'bgr': cv2.resize(
                np.asanyarray(color.get_data()),
                IMAGE_SIZE_PX,
                interpolation=cv2.INTER_AREA,
            ),
            # 先修正深度再交给定位模型：物品位姿是从深度图算出来的。
            'depth': vision_refinement.corrected_depth(
                cv2.resize(
                    np.asanyarray(depth.get_data()).astype(np.float32)
                    * depth_scale_m,
                    IMAGE_SIZE_PX,
                    interpolation=cv2.INTER_NEAREST,
                ),
                self.depth_quadratic_per_m,
            ),
            'depth_quadratic_per_m': self.depth_quadratic_per_m,
            'K': intrinsic_matrix,
            'camera': {
                **identity,
                'native_size': [intrinsics.width, intrinsics.height],
                'image_size': list(IMAGE_SIZE_PX),
                'frame': 'color_optical_frame',
                'K': intrinsic_matrix.tolist(),
                'distortion_coefficients': list(intrinsics.coeffs),
            },
            'sequence': sequence,
            'camera_frame_number': int(color.get_frame_number()),
            'time': captured_at,
            'received_at': received_at,
            'sensor_age_at_receive_s': received_at - captured_at,
        }

    def _publish(self, frame: dict) -> None:
        """发布一帧；第一帧（或出错恢复后的第一帧）时记录相机已连接。"""
        if self.publish(frame):
            self._last_logged_error = None
            self.tracker.event(
                'arm_camera_connected',
                arm=self.arm.arm_id,
                camera=frame['camera'],
            )
