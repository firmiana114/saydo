"""给一串关节角路点配上时间：每一时刻机械臂该在哪。

要解决的问题：路径规划（逆解、碰撞检查）给出的只是“经过哪些姿态”，
没有“什么时候到哪”。把这些姿态一个个交给控制器的关节运动指令去走，
每个点都会减速停一下。要走得连贯，就得自己把整条路径连成一条平滑的
曲线，并规定沿着它的速度怎么变，再按固定的节拍把每一时刻的关节角
发给控制器照着走（角度透传，见 `grasp_robot.RobotClient.stream_joints`）。

做法分两步：

1. 路径的形状：用三次样条（cubic spline，一种过全部给定点、连接处
   连同一阶二阶变化率都连续的分段三次曲线）把路点连起来。
2. 沿路径的进度随时间怎么变：用五次多项式
   `10τ³ − 15τ⁴ + 6τ⁵`（τ 是已用时间占总时间的比例）。它在起点和
   终点的速度、加速度都是零，中间平滑地加速再减速，没有突变。

总时间取“各关节的速度和加速度都不超过上限”所需的最短时间。

本模块只做计算，不连接控制器。
"""

from __future__ import annotations

import dataclasses
from typing import Optional, Sequence

import numpy as np
from scipy import interpolate

# 算速度、加速度峰值时沿时间取多少个点。
PROFILE_SAMPLES = 400


def progress(fraction: np.ndarray) -> np.ndarray:
    """已用时间的比例对应走完了路径的多大比例。

    Args:
        fraction: 已用时间占总时间的比例，0～1 之外的值按 0 或 1 算。

    Returns:
        走完的比例，0～1。起点和终点处的变化率和二阶变化率都是零。
    """
    tau = np.clip(fraction, 0., 1.)
    return tau ** 3 * (10 - 15 * tau + 6 * tau ** 2)


def _progress_rates(tau: np.ndarray) -> tuple:
    """`progress` 对时间比例的一阶和二阶变化率。"""
    first = 30 * tau ** 2 * (1 - tau) ** 2
    second = 60 * tau * (1 - tau) * (1 - 2 * tau)
    return first, second


@dataclasses.dataclass(frozen=True)
class JointTrajectory:
    """一条带时间的关节角轨迹。

    Attributes:
        duration_s: 从起点到终点的总时间，秒。
        peak_speed_deg_s: 全程中转得最快的关节的最高转速，度每秒。
        peak_acceleration_deg_s2: 全程中各关节加速度的最大值，度每二次方秒。
        spline: 路径的形状：走完的比例（0～1）到七个关节角（度）。
    """

    duration_s: float
    peak_speed_deg_s: float
    peak_acceleration_deg_s2: float
    spline: interpolate.CubicSpline

    def position_deg(self, time_s: float) -> np.ndarray:
        """返回某一时刻的关节角。

        Args:
            time_s: 从轨迹开始算起的时间，秒；早于 0 按起点、晚于总
                时间按终点。

        Returns:
            七个关节角，度。
        """
        return self.spline(float(progress(time_s / self.duration_s)))

    def sample(self, period_s: float) -> np.ndarray:
        """按固定间隔取出整条轨迹上的关节角。

        Args:
            period_s: 间隔，秒。

        Returns:
            N×7 的数组，度；第一行是起点，最后一行正好是终点。
        """
        count = max(1, int(np.ceil(self.duration_s / period_s)))
        times = np.minimum(np.arange(count + 1) * period_s, self.duration_s)
        return self.spline(progress(times / self.duration_s))


def plan_through(
    waypoints_deg: Sequence[Sequence[float]],
    max_speed_deg_s: float,
    max_acceleration_deg_s2: float,
    min_duration_s: float = .2,
    knots: Optional[Sequence[float]] = None,
) -> JointTrajectory:
    """把一串关节角路点连成一条从静止到静止的平滑轨迹。

    Args:
        waypoints_deg: 路点，M×7（M ≥ 2），第一个是起点，度。轨迹
            精确经过每一个路点。
        max_speed_deg_s: 任一关节转速的上限，度每秒。
        max_acceleration_deg_s2: 任一关节加速度的上限，度每二次方秒。
        min_duration_s: 总时间的下限，秒。
        knots: 各路点处“走完了路径的多大比例”，M 个从 0 递增到 1 的
            数。缺省按相邻路点之间的关节角变化量累计；路点是沿一条
            直线等距取的时，传等间隔的数，夹爪沿直线的速度就按
            `progress` 的规律变化。

    Returns:
        轨迹。全程各关节的速度和加速度都不超过给定上限。

    Raises:
        ValueError: 路点少于两个、不是 7 个关节角、含有非有限数，
            相邻路点重合，或上限不是正数。
    """
    points = np.asarray(waypoints_deg, dtype=float)
    if (
        points.ndim != 2
        or points.shape[1] != 7
        or len(points) < 2
        or not np.isfinite(points).all()
    ):
        raise ValueError('轨迹至少需要两个路点，每个 7 个有限的关节角')
    if not (
        max_speed_deg_s > 0
        and max_acceleration_deg_s2 > 0
        and min_duration_s > 0
    ):
        raise ValueError('速度、加速度上限和最短时间必须是正数')
    if knots is None:
        steps = np.linalg.norm(np.diff(points, axis=0), axis=1)
        if not (steps > 0).all():
            raise ValueError('相邻两个路点重合，无法确定路径')
        position = np.concatenate([[0.], np.cumsum(steps)]) / steps.sum()
    else:
        position = np.asarray(knots, dtype=float)
        if (
            position.shape != (len(points),)
            or position[0] != 0
            or position[-1] != 1
            or not (np.diff(position) > 0).all()
        ):
            raise ValueError('knots 须与路点一一对应，从 0 严格递增到 1')
    spline = interpolate.CubicSpline(position, points, bc_type='natural')
    tau = np.linspace(0., 1., PROFILE_SAMPLES + 1)
    first, second = _progress_rates(tau)
    along = progress(tau)
    slope = spline(along, 1)
    curvature = spline(along, 2)
    # 总时间为 1 秒时的速度和加速度；总时间为 T 时分别除以 T 和 T²。
    unit_speed = np.abs(slope * first[:, None]).max()
    unit_acceleration = np.abs(
        curvature * first[:, None] ** 2 + slope * second[:, None]
    ).max()
    duration_s = max(
        min_duration_s,
        unit_speed / max_speed_deg_s,
        float(np.sqrt(unit_acceleration / max_acceleration_deg_s2)),
    )
    return JointTrajectory(
        duration_s=float(duration_s),
        peak_speed_deg_s=float(unit_speed / duration_s),
        peak_acceleration_deg_s2=float(unit_acceleration / duration_s ** 2),
        spline=spline,
    )
