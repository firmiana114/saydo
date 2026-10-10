"""两台机械臂互相避让：每台臂占着哪块空间。

要解决的问题：两台臂的底座只相距约 0.52 米，工作范围大片重叠。每台臂
规划时只知道桌面和立杆，不知道另一台臂在哪里，可能撞上去。

做法分两种情况：

- 另一台臂没有在执行动作：读取它当前的关节角，把它当前姿态下的外形
  当作固定的障碍物。
- 另一台臂正在执行动作：它在每一段运动发令之前，都把“从当前位置起
  即将要走的路径”扫过的空间登记在这里（称为占位）。本臂不得进入这块
  空间。两臂各自的占位互不重叠，所以不论谁先走到哪里都不会相撞。
  代价是偏保守：时间上本来错得开的两条路径也会被拒绝。

登记和检查在同一把锁里完成：一台臂登记占位时，如果另一台臂的占位在它
检查之后变过，登记失败，它须按新的占位重新检查。

本模块只在一个进程内有效（两台臂的控制器都在 8089 服务里）。绕过 8089
直接指挥机械臂的程序，以及用示教器手动移动机械臂，不会登记占位；此时
另一台臂只能靠“读取当前关节角”看到它，看到的是读取那一刻的姿态。

外形的换算用到两底座的相对位置（`base_frames`）。本机没有这份文件时，
不做两臂之间的检查。
"""

from __future__ import annotations

import dataclasses
import json
import logging
import pathlib
import threading
import time
from typing import Callable, Dict, Optional, Sequence, Tuple

import numpy as np
import yaml

from saydo.robot import arms
from saydo.robot import base_frames
from saydo.robot import collision_model
from saydo.robot import grasp_robot

LOG = logging.getLogger(__name__)

# 另一台臂静止时，它的关节角读数最多沿用多久，单位秒。规划一次要检查
# 几十上百个候选方案，不必每个都重新读一遍。
STATIC_MAX_AGE_S = .5
# “读不到另一台臂、按设置忽略”这条警告最少隔多久记一次，单位秒。
WARNING_INTERVAL_S = 30.

Capsules = Tuple[np.ndarray, np.ndarray, np.ndarray]


@dataclasses.dataclass(frozen=True)
class OtherArm:
    """另一台臂占着的空间，坐标在本臂的基座坐标系里。

    Attributes:
        capsules: `(起点, 终点, 半径)`，单位米。
        margin_m: 本臂与它之间至少要留的距离。
        label: 报错里怎么称呼它。
        moving: True 表示这是它登记的占位（正在执行动作），False 表示
            是它当前静止的姿态。
    """

    capsules: Capsules
    margin_m: float
    label: str
    moving: bool


def read_controller_posture(
    root: pathlib.Path, arm: arms.Arm
) -> Tuple[np.ndarray, np.ndarray]:
    """向一台臂的控制器查询当前关节角，并读取它的工具记录；只读。

    Args:
        root: 项目根目录。
        arm: 要查询的臂。

    Returns:
        `(关节角, flange_T_tool)`：七个关节角（度），以及夹爪参考点在
        法兰坐标系里的位姿（4×4）。

    Raises:
        OSError: 连不上控制器。
        RuntimeError: 控制器没有给出有效应答。
        ValueError: 配置、工具记录或关节角无效。
    """
    config = json.loads(arm.config_path(root).read_text())
    tool = yaml.safe_load(
        (pathlib.Path(root) / config['tool_file']).read_text()
    )
    state = grasp_robot.RobotClient(
        config['robot_ip'], config['robot_port'], writes=False
    ).query('get_current_arm_state')['arm_state']
    joint_deg = np.asarray(state['joint'], dtype=float) / 1000
    flange_T_tool = np.asarray(tool['flange_T_grip_tcp'], dtype=float)
    if (
        joint_deg.shape != (7,)
        or not np.isfinite(joint_deg).all()
        or flange_T_tool.shape != (4, 4)
    ):
        raise ValueError(f'{arm.label}的关节角或工具记录无效')
    return joint_deg, flange_T_tool


def transformed(capsules: Capsules, target_T_source: np.ndarray) -> Capsules:
    """把一组胶囊体换到另一个坐标系。

    Args:
        capsules: `(起点, 终点, 半径)`，坐标在 source 坐标系里，米。
        target_T_source: 把 source 坐标换到 target 坐标的 4×4 变换。

    Returns:
        换到 target 坐标系后的 `(起点, 终点, 半径)`。
    """
    starts, ends, radii = capsules
    rotation, offset = target_T_source[:3, :3], target_T_source[:3, 3]
    return starts @ rotation.T + offset, ends @ rotation.T + offset, radii


class Occupancy:
    """一个进程里全部机械臂的占位登记处。"""

    def __init__(
        self,
        root: pathlib.Path,
        read_posture: Optional[
            Callable[[pathlib.Path, arms.Arm], Tuple[np.ndarray, np.ndarray]]
        ] = None,
        clock: Callable[[], float] = time.monotonic,
    ) -> None:
        """创建登记处；不连接控制器。

        Args:
            root: 项目根目录。
            read_posture: 读取一台臂当前关节角和工具记录的函数，缺省
                为 `read_controller_posture`。
            clock: 取当前时间（秒）的函数，只用于读数的保鲜期。
        """
        self.root = pathlib.Path(root)
        self._read_posture = read_posture or read_controller_posture
        self._clock = clock
        self._lock = threading.RLock()
        self._claims: Dict[str, Capsules] = {}
        self._static: Dict[str, Tuple[float, Capsules]] = {}
        self._version = 0
        self._warned_s: Optional[float] = None

    def version(self) -> int:
        """返回占位的版本号：任何一台臂登记或撤销占位，它都会加一。

        Returns:
            当前版本号。先取版本号、再检查路径、最后用
            `claim_if_unchanged` 登记，就能发现检查期间别的臂动过占位。
        """
        with self._lock:
            return self._version

    def claim_if_unchanged(
        self,
        arm_id: str,
        joint_path_deg: Sequence[Sequence[float]],
        flange_T_tool: np.ndarray,
        version: int,
    ) -> bool:
        """登记一台臂即将要走的路径扫过的空间，取代它原先的占位。

        Args:
            arm_id: 登记占位的臂。
            joint_path_deg: 即将要走的路径，每个路点七个关节角（度）；
                第一个是当前关节角。
            flange_T_tool: 这台臂的夹爪参考点在法兰坐标系里的位姿。
            version: 检查这条路径之前取的版本号。

        Returns:
            True 表示已登记；False 表示版本号已变（别的臂在这期间登记
            或撤销过占位），没有登记，调用方须重新检查。
        """
        capsules = collision_model.swept_capsules(
            joint_path_deg, flange_T_tool
        )
        with self._lock:
            if version != self._version:
                return False
            self._claims[arm_id] = capsules
            self._version += 1
            return True

    def release(self, arm_id: str) -> None:
        """撤销一台臂的占位；它此后按“静止在当前姿态”对待。

        Args:
            arm_id: 撤销占位的臂。没有占位时什么也不做。
        """
        with self._lock:
            if self._claims.pop(arm_id, None) is not None:
                self._version += 1
            # 它刚动过，原先缓存的静止姿态不能再用。
            self._static.pop(arm_id, None)

    def claimed(self, arm_id: str) -> bool:
        """这台臂当前是否登记了占位。

        Args:
            arm_id: 臂编号。

        Returns:
            是否有占位。
        """
        with self._lock:
            return arm_id in self._claims

    def other_arm(self, arm_id: str) -> Optional[OtherArm]:
        """返回本臂规划时要避开的、另一台臂占着的空间。

        Args:
            arm_id: 本臂的编号。

        Returns:
            另一台臂占着的空间，坐标在本臂的基座坐标系里。本机没有
            两底座的相对位置、只配置了一台臂，或者读不到另一台臂而
            设置为忽略时，返回 None。

        Raises:
            ValueError: 读不到另一台臂的当前姿态而设置为拒绝（缺省），
                或者配置文件无效。
        """
        relation = base_frames.load(self.root)
        others = [
            arm for arm in arms.configured(self.root) if arm.arm_id != arm_id
        ]
        if relation is None or not others:
            return None
        settings = collision_model.other_arm_settings(self.root)
        parts, labels, moving = [], [], False
        for other in others:
            with self._lock:
                capsules = self._claims.get(other.arm_id)
            if capsules is not None:
                moving = True
                labels.append(f'{other.label}正在走的路径')
            else:
                capsules = self._static_capsules(other, settings)
                if capsules is None:
                    continue
                labels.append(other.label)
            parts.append(
                transformed(
                    capsules,
                    base_frames.target_base_T_source_base(
                        relation, arm_id, other.arm_id
                    ),
                )
            )
        if not parts:
            return None
        return OtherArm(
            capsules=tuple(
                np.concatenate([part[k] for part in parts]) for k in range(3)
            ),
            margin_m=settings['margin_m'],
            label='、'.join(labels),
            moving=moving,
        )

    def _static_capsules(
        self, other: arms.Arm, settings: dict
    ) -> Optional[Capsules]:
        """读取另一台臂当前姿态下的外形，短时间内重复调用沿用上次读数。"""
        with self._lock:
            cached = self._static.get(other.arm_id)
            if cached and self._clock() - cached[0] <= STATIC_MAX_AGE_S:
                return cached[1]
            try:
                joint_deg, flange_T_tool = self._read_posture(self.root, other)
            except (OSError, RuntimeError, ValueError, KeyError) as error:
                if settings['when_unreadable'] != 'ignore':
                    raise ValueError(
                        f'读不到{other.label}的当前姿态（{error}），无法判断'
                        '会不会与它相撞，未发送运动。请给它上电并确认网络'
                        '连通；如果它已断电并且确认停在碰不到的地方，可在 '
                        f'{collision_model.OBSTACLES_FILE} 里设置 '
                        'other_arm: {when_unreadable: ignore}'
                    ) from error
                now = self._clock()
                if (
                    self._warned_s is None
                    or now - self._warned_s >= WARNING_INTERVAL_S
                ):
                    self._warned_s = now
                    LOG.warning(
                        '读不到%s的当前姿态（%s），按设置忽略：本次规划'
                        '不检查与它的碰撞',
                        other.label,
                        error,
                    )
                return None
            capsules = collision_model.swept_capsules(
                [joint_deg], flange_T_tool
            )
            self._static[other.arm_id] = (self._clock(), capsules)
            return capsules


_SHARED: Dict[pathlib.Path, Occupancy] = {}
_SHARED_LOCK = threading.Lock()


def shared(root: pathlib.Path) -> Occupancy:
    """返回这个进程里某个项目根目录对应的那一个登记处。

    同一进程里两台臂的控制器必须用同一个登记处，才看得到彼此的占位。

    Args:
        root: 项目根目录。

    Returns:
        该根目录的登记处；首次调用时创建。
    """
    key = pathlib.Path(root).resolve()
    with _SHARED_LOCK:
        if key not in _SHARED:
            _SHARED[key] = Occupancy(key)
        return _SHARED[key]
