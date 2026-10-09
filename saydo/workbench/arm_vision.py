"""一台机械臂的视觉来源：实时跟踪接在它上面时转给跟踪器，否则只供快照。

8089 里只有一份定位模型（显存只够放一份），所以实时跟踪同一时间只能
接一台臂手上的相机，这台臂称为“当前实时跟踪的臂”。每台臂各有一个
本类的对象，动作控制器和 Agent 接口把它当作这台臂的视觉来源：

- 这台臂是当前实时跟踪的臂时，读写的状态、画面、冻结的候选和各种
  跟踪操作都直接落在跟踪器上，与只有一台臂时完全一样。
- 不是时，本对象只提供这台相机最近的一帧，供场景快照使用；需要实时
  跟踪位姿的操作会得到一句明确的报错。

相机画面由取图线程通过 `publish` 送进来：1 臂的取图线程在跟踪器里，
其余臂的在 `arm_camera.ArmCamera` 里。
"""

from __future__ import annotations

import pathlib
import threading
import time
from typing import Any, Dict, Optional

from saydo.robot import arms


class ArmVision:
    """一台臂的视觉来源，对外的属性和方法与跟踪器同名。

    Attributes:
        tracker: 8089 的跟踪器。
        arm: 这是哪台臂的视觉来源。
        models_root: 物品模型目录。
        grasp: 使用本对象的动作控制器，由创建方在控制器建好后填入。
    """

    def __init__(self, tracker: Any, arm: arms.Arm) -> None:
        """创建视觉来源；不打开任何设备。

        Args:
            tracker: 8089 的跟踪器。用它的定位模型、事件日志，以及
                `live_arm_id`（当前实时跟踪接在哪台臂上）。
            arm: 这是哪台臂的视觉来源。
        """
        self.tracker = tracker
        self.arm = arm
        self.models_root = pathlib.Path(tracker.models_root)
        self.grasp: Any = None
        # 以下带 own 的成员是这台相机自己的一份，不做实时跟踪时对外提供。
        self._own_lock = threading.RLock()
        self._own_state: Dict[str, Any] = {
            'state': 'scene_snapshot_only',
            'error': None,
            'camera_state': 'starting',
            'sequence': 0,
            'model_ready': False,
        }
        self._own_latest: Optional[dict] = None
        self._own_generation = 0
        self._own_command: Optional[tuple] = None

    @property
    def live(self) -> bool:
        """这台臂是不是当前实时跟踪的臂。"""
        return self.tracker.live_arm_id == self.arm.arm_id

    @property
    def lock(self) -> Any:
        """保护状态与画面的可重入锁；实时跟踪时就是跟踪器的那把。"""
        return self.tracker.lock if self.live else self._own_lock

    @property
    def state(self) -> Dict[str, Any]:
        """视觉状态；实时跟踪时就是跟踪器的状态。

        不做实时跟踪时含 `camera`（相机身份与内参）、`camera_frame_at`
        （最近一帧的时间）、`camera_state`（`starting`、`streaming`、
        `error`）和 `model_ready`（定位模型是否就绪）。
        """
        return self.tracker.state if self.live else self._own_state

    @property
    def latest(self) -> Optional[dict]:
        """这台相机最近的一帧；还没有画面时为 None。"""
        return self.tracker.latest if self.live else self._own_latest

    @property
    def frozen(self) -> Optional[dict]:
        """冻结的候选画面；不做实时跟踪时恒为 None。"""
        return self.tracker.frozen if self.live else None

    @property
    def generation(self) -> int:
        """跟踪代次；场景采集会把它加一，让正在进行的跟踪结果作废。"""
        return (
            self.tracker.generation if self.live else self._own_generation
        )

    @generation.setter
    def generation(self, value: int) -> None:
        """写入跟踪代次；实时跟踪时写到跟踪器上。"""
        if self.live:
            self.tracker.generation = value
        else:
            self._own_generation = value

    @property
    def command(self) -> Optional[tuple]:
        """待处理的跟踪指令；不做实时跟踪时没有用处。"""
        return self.tracker.command if self.live else self._own_command

    @command.setter
    def command(self, value: Optional[tuple]) -> None:
        """写入待处理的跟踪指令；实时跟踪时写到跟踪器上。"""
        if self.live:
            self.tracker.command = value
        else:
            self._own_command = value

    def unsupported_message(self) -> str:
        """返回“这台臂现在不能做实时跟踪”的报错文字。"""
        live_arm = arms.by_id(self.tracker.live_arm_id)
        return (
            f'实时跟踪现在接在{live_arm.label}的相机上，{self.arm.label}'
            '此时只支持先采集场景快照再动作；要对它做实时跟踪，'
            '请先在 8089 页面把当前机械臂切换过来'
        )

    def snapshot(self) -> dict:
        """返回这台臂的视觉状态和动作控制器状态。

        Returns:
            实时跟踪时是跟踪器的完整状态；否则是相机状态、最近一帧
            距今的秒数（`pose_age_s` 恒为 None）和动作控制器的状态。
        """
        if self.live:
            return self.tracker.snapshot()
        with self._own_lock:
            self._own_state['model_ready'] = bool(
                self.tracker.state.get('model_ready')
            )
            result = dict(self._own_state)
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

    def _live_only(self, name: str, *args: Any, **kwargs: Any) -> Any:
        """调用跟踪器上只在实时跟踪时才有意义的操作。"""
        if not self.live:
            raise ValueError(self.unsupported_message())
        return getattr(self.tracker, name)(*args, **kwargs)

    # 以下操作都依赖实时跟踪。逐个列出而不是用 __getattr__ 兜底，是为了
    # 让拼错的属性名照常报 AttributeError。

    def grasp_observation(self) -> dict:
        """返回实时跟踪给出的物品位姿观测。

        Raises:
            ValueError: 实时跟踪不在这台臂上，或当前没有可用的位姿。
        """
        return self._live_only('grasp_observation')

    def switch_object(self, object_id: str) -> None:
        """让实时跟踪换一个物品模型。

        Args:
            object_id: 物品编号。

        Raises:
            ValueError: 实时跟踪不在这台臂上，或有任务在处理。
        """
        return self._live_only('switch_object', object_id)

    def automatic(self, *args: Any, **kwargs: Any) -> None:
        """让实时跟踪自动找框并开始跟踪。

        Raises:
            ValueError: 实时跟踪不在这台臂上，或当前不能自动识别。
        """
        return self._live_only('automatic', *args, **kwargs)

    def freeze(self) -> dict:
        """冻结当前画面，供人工或智能体给框。

        Returns:
            冻结画面的信息。

        Raises:
            ValueError: 实时跟踪不在这台臂上，或有任务在处理。
        """
        return self._live_only('freeze')

    def initialize(self, value: dict) -> None:
        """用给定的框在冻结画面上开始跟踪。

        Args:
            value: 含框坐标的请求。

        Raises:
            ValueError: 实时跟踪不在这台臂上，或请求无效。
        """
        return self._live_only('initialize', value)

    def choose(self, value: dict) -> None:
        """在多个候选里选定一个开始跟踪。

        Args:
            value: 含候选序号的请求。

        Raises:
            ValueError: 实时跟踪不在这台臂上，或请求无效。
        """
        return self._live_only('choose', value)

    def publish(self, frame: dict) -> bool:
        """接收取图线程送来的一帧；实时跟踪在这台臂上时同时交给跟踪器。

        Args:
            frame: 一帧画面，含彩色图、深度图、内参、相机身份和时间。

        Returns:
            这是不是相机（重新）连上后的第一帧，取图线程据此记日志。
        """
        with self._own_lock:
            connected = self._own_state['camera_state'] != 'streaming'
            self._own_latest = frame
            self._own_state.update(
                camera_state='streaming',
                error=None,
                sequence=frame['sequence'],
                camera=frame['camera'],
                camera_frame_at=frame['time'],
                model_ready=bool(self.tracker.state.get('model_ready')),
            )
        # 是否仍是实时跟踪的臂由跟踪器在自己的锁里再判断一次。
        self.tracker.accept_frame(self.arm.arm_id, frame)
        return connected

    def camera_failed(self, message: str) -> None:
        """记录这台相机出错：清掉最近一帧，状态里写明原因。

        实时跟踪在这台臂上时，跟踪器会因为画面过期而自行转入丢失状态。

        Args:
            message: 给操作者看的出错原因。
        """
        with self._own_lock:
            self._own_latest = None
            self._own_state.update(camera_state='error', error=message)

    def own_frame(self) -> Optional[dict]:
        """返回这台相机自己最近的一帧，不论实时跟踪在不在这台臂上。"""
        with self._own_lock:
            return self._own_latest
