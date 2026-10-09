"""第二台及以后的机械臂手上相机：持续取图，供场景快照使用。

8089 的跟踪器（`live_tracker.Tracker`）读取 1 臂手上的相机并做实时跟踪。
其余的臂由本模块各开一条取图线程，只做一件事：把最新的一帧彩色和深度
图放在 `latest` 里。场景快照拿这一帧去定位物品，定位用的模型仍是跟踪器
里那一份（显存只够放一份），所以定位请求转交给跟踪器处理。

本模块不做实时跟踪：这台臂只支持“先采集场景快照、再按快照动作”的流程。
需要实时跟踪位姿的操作会得到一句明确的报错。

与 1 臂的取图不同，这里相机出错不会让整个服务停止：记录后隔几秒重试，
1 臂的工作不受影响。
"""

from __future__ import annotations

import pathlib
import threading
import time
import traceback
from typing import Any, Dict, Optional

import cv2
import numpy as np

from saydo.robot import arms

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


class ArmCamera:
    """一台机械臂手上相机的取图线程，对外的属性与跟踪器同名。

    动作控制器和 Agent 接口把它当作这台臂的“视觉来源”使用。

    Attributes:
        arm: 这台相机装在哪台臂手上。
        serial: 相机序列号。
        models_root: 物品模型目录。
        lock: 保护 `latest` 与 `state` 的可重入锁。
        state: 视觉状态。`camera`（相机身份与内参）、`camera_frame_at`
            （最近一帧的时间）、`camera_state`（`starting`、`streaming`、
            `error`）、`model_ready`（定位模型是否就绪，取自跟踪器）。
        latest: 最近一帧；还没有画面时为 None。
        frozen: 冻结的候选画面；本类不做实时跟踪，恒为 None。
        generation: 跟踪代次；场景采集会改它，本类不使用。
        command: 待处理的跟踪指令；本类不使用。
        grasp: 使用本对象的动作控制器，由创建方在控制器建好后填入。
    """

    def __init__(self, tracker: Any, arm: arms.Arm, serial: str) -> None:
        """创建取图对象；不打开设备，调用 `run` 后才开始取图。

        Args:
            tracker: 8089 的跟踪器。用它的定位模型、事件日志、停止标志，
                并从它那里取得已加载的 RealSense 库。
            arm: 相机装在哪台臂手上。
            serial: 相机序列号。
        """
        self.tracker = tracker
        self.arm = arm
        self.serial = serial
        self.models_root = pathlib.Path(tracker.models_root)
        self.lock = threading.RLock()
        self.unsupported = (
            f'{arm.label}只支持先采集场景快照再动作，'
            '不支持实时跟踪位姿（实时跟踪只接 1 臂的相机）'
        )
        self.state: Dict[str, Any] = {
            'state': 'scene_snapshot_only',
            'error': None,
            'camera_state': 'starting',
            'sequence': 0,
            'model_ready': False,
        }
        self.latest: Optional[dict] = None
        self.frozen: Optional[dict] = None
        self.generation = 0
        self.command: Optional[tuple] = None
        self.grasp: Any = None
        self._last_logged_error: Optional[str] = None

    def snapshot(self) -> dict:
        """返回与跟踪器同形的状态；不含实时跟踪的位姿。

        Returns:
            视觉状态、最近一帧距今的秒数，以及这台臂动作控制器的状态。
        """
        with self.lock:
            self.state['model_ready'] = bool(
                self.tracker.state.get('model_ready')
            )
            result = dict(self.state)
        frame_at = result.get('camera_frame_at')
        result['camera_age_s'] = (
            time.time() - frame_at if frame_at is not None else None
        )
        result['pose_age_s'] = None
        result['grasp'] = self.grasp.snapshot()
        return result

    def localize_frozen_scene(
        self, frame: dict, targets: list, check: Any, progress: Any
    ) -> list:
        """在一帧冻结的画面里定位全部目标；交给跟踪器的定位模型处理。

        Args:
            frame: 本相机的一帧画面。
            targets: 要定位的目标（物品编号、实例编号、可选的框）。
            check: 定位期间反复调用的检查函数，抛出异常即取消。
            progress: 接收进度文字的函数。

        Returns:
            各目标在相机坐标系里的位姿与质量信息。

        Raises:
            ValueError: 已有另一个场景采集在进行、被取消，或定位失败。
        """
        return self.tracker.localize_frozen_scene(
            frame, targets, check, progress
        )

    def _unsupported(self, *unused_args: Any, **unused_kwargs: Any) -> Any:
        raise ValueError(self.unsupported)

    # 以下操作都依赖实时跟踪，本类一律报同一句错。逐个列出而不是用
    # __getattr__ 兜底，是为了让拼错的属性名照常报 AttributeError。
    grasp_observation = _unsupported
    switch_object = _unsupported
    automatic = _unsupported
    freeze = _unsupported
    initialize = _unsupported
    choose = _unsupported

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
        with self.lock:
            self.latest = None
            self.state.update(camera_state='error', error=message)
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
            'depth': cv2.resize(
                np.asanyarray(depth.get_data()).astype(np.float32)
                * depth_scale_m,
                IMAGE_SIZE_PX,
                interpolation=cv2.INTER_NEAREST,
            ),
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
        with self.lock:
            connected = self.state['camera_state'] != 'streaming'
            self.latest = frame
            self.state.update(
                camera_state='streaming',
                error=None,
                sequence=frame['sequence'],
                camera=frame['camera'],
                camera_frame_at=frame['time'],
                model_ready=bool(self.tracker.state.get('model_ready')),
            )
        if connected:
            self._last_logged_error = None
            self.tracker.event(
                'arm_camera_connected',
                arm=self.arm.arm_id,
                camera=frame['camera'],
            )
