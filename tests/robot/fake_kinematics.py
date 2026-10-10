"""系统边界上的替身：成对扮演离线逆解检查器和控制器的运动学。

真实系统里这两样东西是对得上的：逆解检查器把夹爪位姿换成一组关节角，
程序把这组关节角发给控制器，控制器到位后报告的夹爪位姿就是原来那个。
测试里没有真实的运动学，本替身给每个出现过的位姿分配一组独有的关节角
并记住对应关系，于是“求解 → 下发关节角 → 读回位姿”这一圈能在测试里
闭合，被测的流程代码不需要任何改动。

它只替换逆解子进程和控制器这两处边界，不做任何可达性或碰撞判断：
交给它的路点一律“有解”。
"""

from __future__ import annotations

from typing import Dict, List, Optional, Tuple


class PairedKinematics:
    """记住“位姿 ↔ 关节角”对应关系的逆解与控制器替身。"""

    def __init__(self) -> None:
        self._pose_by_joint: Dict[Tuple[int, ...], List[int]] = {}
        self._joint_by_pose: Dict[Tuple[int, ...], List[int]] = {}

    def joint_raw_for(self, controller_pose: List[int]) -> List[int]:
        """返回某个位姿对应的关节角，首次见到时分配一组新的。

        Args:
            controller_pose: 夹爪位姿，控制器原始单位（微米与毫弧度）。

        Returns:
            七个关节角，控制器原始单位（0.001 度）。第 n 个不同的位姿
            得到第 1 关节为 n 度、其余为 0 的一组，彼此至少相差 1 度。
        """
        key = tuple(int(value) for value in controller_pose)
        if key not in self._joint_by_pose:
            joint = [1000 * (len(self._joint_by_pose) + 1)] + [0] * 6
            self._joint_by_pose[key] = joint
            self._pose_by_joint[tuple(joint)] = list(key)
        return list(self._joint_by_pose[key])

    def pose_for(self, joint_raw: List[int]) -> List[int]:
        """返回一组关节角对应的夹爪位姿，供控制器替身“运动到位”时使用。

        Args:
            joint_raw: 七个关节角，控制器原始单位（0.001 度）。

        Returns:
            对应的夹爪位姿，控制器原始单位。

        Raises:
            AssertionError: 这组关节角不是本替身解出来的，说明被测代码
                下发了未经检查的关节目标。
        """
        key = tuple(int(value) for value in joint_raw)
        if key not in self._pose_by_joint:
            raise AssertionError(f'下发了未经逆解检查的关节目标：{joint_raw}')
        return list(self._pose_by_joint[key])

    def detail(self, waypoints: List[dict]) -> dict:
        """生成与真实检查器同形的通过记录。

        Args:
            waypoints: 被检查的路点，每个含 `controller_pose`。

        Returns:
            含 `joint_solutions_deg`（单位度）的检查记录。
        """
        return {
            'verified': True,
            'joint_travel_deg': 1.,
            'joint_solutions_deg': [
                [
                    value / 1000
                    for value in self.joint_raw_for(w['controller_pose'])
                ]
                for w in waypoints
            ],
        }

    def check_waypoints(
        self,
        executable: object,
        tool: dict,
        arm: dict,
        waypoints: List[dict],
        timeout: float = 3.,
        environment: Optional[object] = None,
    ) -> Tuple[bool, dict]:
        """替换 `motion_validation.check_waypoints`：所有路点都有解。

        Args:
            executable: 检查器路径，替身不使用。
            tool: 工具记录，替身不使用。
            arm: 机械臂当前状态，替身不使用。
            waypoints: 被检查的路点。
            timeout: 时间上限，单位秒，替身不使用。
            environment: 障碍物设置，替身不使用。

        Returns:
            `(True, 检查记录)`。
        """
        return True, self.detail(waypoints)

    def verify_motion_targets(
        self,
        plan: dict,
        config: dict,
        report: dict,
        arm: dict,
        waypoints: List[dict],
        timeout: float = 3.,
    ) -> dict:
        """替换动作控制器的 `verify_motion_targets`，供沿用旧写法的测试用。

        新测试应当改为替换 `motion_validation.check_waypoints`，让真实的
        `verify_motion_targets` 运行。

        Args:
            plan: 方案，替身不使用。
            config: 抓取配置，替身不使用。
            report: 诊断记录，替身不使用。
            arm: 机械臂当前状态，替身不使用。
            waypoints: 被检查的路点。
            timeout: 时间上限，单位秒，替身不使用。

        Returns:
            含 `joint_solutions_deg` 的检查记录。
        """
        return self.detail(waypoints)
