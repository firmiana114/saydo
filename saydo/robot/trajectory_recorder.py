"""记录机械臂运动时实际走过的关节角，并与计划的路径比较。

要解决的问题：碰撞检查查的是“程序算出来的那条路径”——相邻路点之间
各关节同步转动、走关节角直线。控制器实际是不是这样走的，此前没有
实测过；把几条指令连成一条不停顿的轨迹之后更需要核对。本模块在运动
期间高频读取实际关节角（只读查询），事后算出它偏离计划路径多少、
途中有没有停顿。

本模块只读取，不发送任何运动指令。
"""

from __future__ import annotations

import json
import socket
import threading
import time
from typing import Callable, List, Optional, Sequence, Tuple

import numpy as np

from saydo.robot import collision_model

# 两次读取之间的间隔，单位秒。一次查询约 5 毫秒。
DEFAULT_PERIOD_S = .01
# 判断“停住了”用的速度门槛：变化最快的那个关节每秒转不到这么多度。
STOP_SPEED_DEG_S = .5
# 停住至少这么久才算一次停顿，单位秒。
STOP_MIN_DURATION_S = .05


class TrajectoryRecorder:
    """在后台线程里按固定间隔读取一台臂的关节角。"""

    def __init__(
        self,
        address: Tuple[str, int],
        period_s: float = DEFAULT_PERIOD_S,
        connect: Callable = socket.create_connection,
        clock: Callable[[], float] = time.monotonic,
    ) -> None:
        """创建记录器；不连接控制器。

        Args:
            address: 控制器的 `(地址, 端口)`。
            period_s: 两次读取之间的间隔，单位秒。
            connect: 建立连接的函数，测试时替换。
            clock: 取当前时间（秒）的函数。
        """
        self.address = address
        self.period_s = period_s
        self._connect = connect
        self._clock = clock
        self._stop = threading.Event()
        self._thread: Optional[threading.Thread] = None
        self.samples: List[dict] = []
        self.error: Optional[str] = None
        self.started_s = 0.

    def start(self) -> None:
        """开始记录；只发送只读的状态查询。"""
        self._stop.clear()
        self.samples = []
        self.error = None
        self.started_s = self._clock()
        self._thread = threading.Thread(target=self._run, daemon=True)
        self._thread.start()

    def stop(self) -> List[dict]:
        """停止记录。

        Returns:
            全部样本，每个含 `t_s`（距开始记录的秒数）、`joint_deg`
            （七个关节角，度）和 `pose`（控制器报告的夹爪位姿原始值）。
            记录中途出错时已有的样本仍会返回，原因在 `self.error`。
        """
        self._stop.set()
        if self._thread is not None:
            self._thread.join(5)
        return self.samples

    def elapsed_s(self) -> float:
        """返回从开始记录到现在的秒数。"""
        return self._clock() - self.started_s

    def _run(self) -> None:
        """循环查询关节角直到被要求停止。"""
        request = b'{"command":"get_current_arm_state"}\r\n'
        try:
            with self._connect(self.address, timeout=2) as connection:
                connection.settimeout(2)
                with connection.makefile('rb') as stream:
                    while not self._stop.is_set():
                        began = self._clock()
                        connection.sendall(request)
                        state = None
                        for _ in range(16):
                            line = stream.readline(16384)
                            if not line:
                                raise RuntimeError('控制器断开连接')
                            reply = json.loads(line)
                            if isinstance(reply, dict) and isinstance(
                                reply.get('arm_state'), dict
                            ):
                                state = reply['arm_state']
                                break
                        if state is None:
                            raise RuntimeError('没有收到关节角应答')
                        self.samples.append(
                            {
                                't_s': self._clock() - self.started_s,
                                'joint_deg': [
                                    value / 1000 for value in state['joint']
                                ],
                                'pose': list(state['pose']),
                            }
                        )
                        remaining = self.period_s - (self._clock() - began)
                        if remaining > 0:
                            self._stop.wait(remaining)
        except (OSError, RuntimeError, ValueError, KeyError) as error:
            self.error = f'{type(error).__name__}: {error}'


def deviation_from_polyline(
    joint_deg: Sequence[Sequence[float]],
    waypoints_deg: Sequence[Sequence[float]],
) -> np.ndarray:
    """算出每个实测姿态离计划路径有多远。

    计划路径是依次连接各路点的关节角折线。

    Args:
        joint_deg: 实测的关节角，N×7，度。
        waypoints_deg: 计划的路点，M×7（M ≥ 2），第一个是起点，度。

    Returns:
        长度 N 的数组：每个实测姿态到折线上最近一点，偏差最大的那个
        关节相差多少度。
    """
    actual = np.asarray(joint_deg, dtype=float)
    path = np.asarray(waypoints_deg, dtype=float)
    best = np.full(len(actual), np.inf)
    for start, end in zip(path[:-1], path[1:]):
        step = end - start
        length_squared = float(step @ step)
        fraction = (
            np.clip((actual - start) @ step / length_squared, 0., 1.)
            if length_squared > 0
            else np.zeros(len(actual))
        )
        residual = np.abs(actual - (start + fraction[:, None] * step))
        best = np.minimum(best, residual.max(axis=1))
    return best


def tcp_line_deviation_m(
    joint_deg: Sequence[Sequence[float]],
    flange_T_tool: np.ndarray,
    line_start_m: Sequence[float],
    line_end_m: Sequence[float],
) -> np.ndarray:
    """算出每个实测姿态下夹爪参考点离一条直线段有多远。

    Args:
        joint_deg: 实测的关节角，N×7，度。
        flange_T_tool: 夹爪参考点在法兰坐标系里的位姿。
        line_start_m: 直线段起点，基座坐标，米。
        line_end_m: 直线段终点，基座坐标，米。

    Returns:
        长度 N 的数组，单位米。
    """
    start = np.asarray(line_start_m, dtype=float)
    line = np.asarray(line_end_m, dtype=float) - start
    length_squared = float(line @ line)
    result = []
    for joint in joint_deg:
        tip = (
            collision_model.link_frames(joint)[7]
            @ np.asarray(flange_T_tool, dtype=float)
        )[:3, 3]
        offset = tip - start
        fraction = (
            np.clip(offset @ line / length_squared, 0., 1.)
            if length_squared > 0
            else 0.
        )
        result.append(float(np.linalg.norm(offset - fraction * line)))
    return np.array(result)


def speed_deg_s(samples: Sequence[dict]) -> Tuple[np.ndarray, np.ndarray]:
    """算出相邻两次读取之间变化最快的那个关节的转速。

    Args:
        samples: `TrajectoryRecorder.stop` 返回的样本，至少两个。

    Returns:
        `(时刻, 转速)`：各区间中点的时刻（秒）和该区间内变化最快的
        关节的转速（度每秒），长度都是样本数减一。
    """
    times = np.array([sample['t_s'] for sample in samples])
    joints = np.array([sample['joint_deg'] for sample in samples])
    interval = np.maximum(np.diff(times), 1e-6)
    speed = np.abs(np.diff(joints, axis=0)).max(axis=1) / interval
    return (times[:-1] + times[1:]) / 2, speed


def stops_during_motion(
    samples: Sequence[dict],
    speed_threshold_deg_s: float = STOP_SPEED_DEG_S,
    min_duration_s: float = STOP_MIN_DURATION_S,
) -> List[dict]:
    """找出运动途中的停顿：开始动之后、最后一次停下之前的静止时段。

    Args:
        samples: `TrajectoryRecorder.stop` 返回的样本。
        speed_threshold_deg_s: 低于这个转速算静止。
        min_duration_s: 静止至少这么久才算一次停顿。

    Returns:
        每次停顿的 `start_s`、`end_s`、`duration_s`；没有停顿或样本
        太少时为空列表。
    """
    if len(samples) < 3:
        return []
    times, speed = speed_deg_s(samples)
    moving = np.flatnonzero(speed >= speed_threshold_deg_s)
    if not len(moving):
        return []
    stops, began = [], None
    for index in range(moving[0], moving[-1] + 1):
        if speed[index] < speed_threshold_deg_s:
            began = index if began is None else began
            continue
        if began is not None:
            start_s = samples[began]['t_s']
            end_s = samples[index]['t_s']
            if end_s - start_s >= min_duration_s:
                stops.append(
                    {
                        'start_s': float(start_s),
                        'end_s': float(end_s),
                        'duration_s': float(end_s - start_s),
                    }
                )
            began = None
    return stops


def closest_approach_deg(
    joint_deg: Sequence[Sequence[float]],
    waypoints_deg: Sequence[Sequence[float]],
) -> List[float]:
    """算出实测轨迹离每个计划路点最近时相差多少。

    连接成一条轨迹后，控制器可能不精确经过中间的路点而是抄近路；
    这个数就是抄了多少。

    Args:
        joint_deg: 实测的关节角，N×7，度。
        waypoints_deg: 计划的路点，M×7，度。

    Returns:
        每个路点一个数：实测轨迹上离它最近的那个姿态，偏差最大的关节
        相差多少度。
    """
    actual = np.asarray(joint_deg, dtype=float)
    return [
        float(np.abs(actual - np.asarray(point, dtype=float)).max(axis=1).min())
        for point in waypoints_deg
    ]
