"""尚未接入视觉的机械臂使用的占位视觉来源。

动作控制器和 Agent 接口都需要一个“视觉来源”：提供当前画面、物品位姿
和相机身份。8089 目前只读取 1 臂手上的相机。其余的臂在接入各自的相机
之前使用本模块的占位对象，这样它们不需要画面的操作（读取状态、复位、
夹爪、停止）照常可用，而需要画面的操作会得到一句明确的中文报错，
不会误用 1 臂相机的画面去换算另一台臂的坐标。
"""

from __future__ import annotations

import pathlib
import threading
from typing import Any, Dict, Optional

from saydo.robot import arms


class NoVision:
    """没有相机的视觉来源，对外的属性和方法与真正的跟踪器同名。

    Attributes:
        models_root: 物品模型目录；夹法库和几何查询要用，与相机无关。
        lock: 与真正的跟踪器一致的可重入锁，控制器按固定顺序加锁。
        state: 跟踪状态；这里只有 `state` 和 `error` 两项，没有相机身份
            和正在跟踪的物品。
        latest: 最近一帧画面，恒为 None。
        frozen: 冻结的候选画面，恒为 None。
        generation: 跟踪代次，仅为与跟踪器保持同名属性。
        command: 待处理的跟踪指令，恒为 None。
        grasp: 使用本对象的动作控制器，由创建方在控制器建好后填入。
        live: 实时跟踪是否接在这台臂上，恒为 False。
    """

    def __init__(self, models_root: pathlib.Path, arm: arms.Arm) -> None:
        """创建占位视觉来源；不打开任何设备。

        Args:
            models_root: 物品模型目录。
            arm: 使用本对象的那台臂，报错里用它的名字。
        """
        self.models_root = pathlib.Path(models_root)
        self.arm = arm
        self.lock = threading.RLock()
        self.message = (
            f'{arm.label}尚未接入视觉，不能使用需要相机画面的操作'
        )
        self.state: Dict[str, Any] = {
            'state': 'vision_unavailable', 'error': self.message
        }
        self.latest: Optional[dict] = None
        self.frozen: Optional[dict] = None
        self.generation = 0
        self.command: Optional[dict] = None
        self.grasp: Any = None
        self.live = False

    def snapshot(self) -> dict:
        """返回与跟踪器同形的状态，其中不含画面和位姿。

        Returns:
            视觉状态（恒为不可用）以及这台臂动作控制器的状态。
        """
        return {**self.state, 'grasp': self.grasp.snapshot()}

    def _unavailable(self, *unused_args: Any, **unused_kwargs: Any) -> Any:
        raise ValueError(self.message)

    # 以下都是需要相机画面的操作，一律报同一句错。逐个列出而不是用
    # __getattr__ 兜底，是为了让拼错的属性名照常报 AttributeError。
    grasp_observation = _unavailable
    localize_frozen_scene = _unavailable
    switch_object = _unavailable
    automatic = _unavailable
    freeze = _unavailable
    initialize = _unavailable
    choose = _unavailable
