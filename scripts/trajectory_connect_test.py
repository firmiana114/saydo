"""试验：让机械臂沿直线连贯地走，实测控制器到底怎么走。

名字是历史遗留：最初只试“轨迹连接”，后来加入了自己规划速度、按固定
节拍把关节角发给控制器的做法（`--mode stream`，角度透传）。

带 `--execute` 时会让机械臂运动；不带时只做计算并打印计划，不发送
任何运动指令。夹爪始终不动。

要回答的问题（厂家文档没有写）：

1. 连起来之后，中间那些点是精确经过，还是抄近路？实际轨迹偏离“逐点
   之间走关节角直线”多少？碰撞检查查的是后者。
2. 途中还停不停？
3. 控制器的到位消息怎么给？
4. 中途发停止指令，已经排队的后续轨迹会不会接着走？

做法：从机械臂当前位置出发，夹爪朝向不变，沿一条直线（缺省向下
100 毫米）取若干个点，求出一条连续且不碰撞的关节角路径，然后用两种
方式之一去走，并在运动期间每 10 毫秒读取一次实际关节角：

- `--mode connected`：前几条 `movej` 带轨迹连接标志，最后一条不带；
- `--mode stepwise`：逐条发送、每条等到位，即现在抓取流程的做法，
  用来对照；
- `--mode stream`：把路点连成一条平滑轨迹，自己规定每一时刻走到哪
  （见 `saydo/robot/joint_trajectory.py`），每隔 `--period-ms` 把当时
  的关节角发给控制器，控制器不做规划、直接照着走。

结果（计划、控制器的全部应答、实测轨迹、分析）保存在
`work/trajectory_tests/` 下。

用法（在项目根目录）：

    work/foundationpose_env/bin/python -m scripts.trajectory_connect_test
    work/foundationpose_env/bin/python -m scripts.trajectory_connect_test \\
        --mode connected --execute

注意：本脚本不经过 8089，另一台臂看不到它登记的占位。试验期间不要让
另一台臂运动。
"""

import argparse
import dataclasses
import fcntl
import json
import logging
import pathlib
import threading
import time
from typing import List, Optional

import numpy as np
import yaml

from saydo.robot import arm_occupancy
from saydo.robot import arms
from saydo.robot import collision_model
from saydo.robot import grasp_robot
from saydo.robot import joint_trajectory
from saydo.robot import motion_validation
from saydo.robot import trajectory_recorder

LOG = logging.getLogger('trajectory_connect_test')
ROOT = pathlib.Path(__file__).resolve().parents[1]
OUTPUT_DIRECTORY = 'work/trajectory_tests'
# 试验允许的范围：直线最长 200 毫米，速度最高 30%。
MAX_DISTANCE_MM = 200.
MAX_SPEED_PERCENT = 30
# 透传试验允许的关节转速和加速度上限。
MAX_STREAM_SPEED_DEG_S = 60.
MAX_STREAM_ACCEL_DEG_S2 = 200.
# 等待一段运动完成的时间上限，单位秒。
MOTION_TIMEOUT_S = 30.
# 开始前机械臂必须静止：两次读数之间任一关节变化不超过这么多度。
STATIONARY_DEG = .03


def line_waypoints(
    pose_raw: List[int], offset_mm: List[float], points: int
) -> List[dict]:
    """沿一条直线生成等间隔的路点，夹爪朝向保持不变。

    Args:
        pose_raw: 起点的夹爪位姿，控制器原始单位（微米与毫弧度）。
        offset_mm: 直线的位移，基座坐标，毫米。
        points: 路点个数，最后一个是直线终点。

    Returns:
        路点列表，每个含 `name`、`controller_pose` 和 `method`
        （`linear`：相邻关节解变化不得超过 10 度）。
    """
    waypoints = []
    for index in range(1, points + 1):
        pose = list(pose_raw)
        for axis in range(3):
            pose[axis] = int(
                round(pose_raw[axis] + offset_mm[axis] * 1000 * index / points)
            )
        waypoints.append(
            {
                'name': f'line:{index}/{points}',
                'controller_pose': pose,
                'method': 'linear',
            }
        )
    return waypoints


def read_stationary_state(robot: grasp_robot.RobotClient) -> dict:
    """读取机械臂状态并确认它没有报错、没有在动；只读。

    Args:
        robot: 只读的控制器连接。

    Returns:
        机械臂状态，关节为原始单位（0.001 度）。

    Raises:
        ValueError: 机械臂报错，或两次读数之间在动。
    """
    first = robot.query('get_current_arm_state')['arm_state']
    time.sleep(.2)
    second = robot.query('get_current_arm_state')['arm_state']
    if any(second.get('err', [1])):
        raise ValueError(f'机械臂存在错误状态：{second.get("err")}')
    moved_deg = np.abs(
        np.asarray(second['joint'], dtype=float) - first['joint']
    ).max() / 1000
    if moved_deg > STATIONARY_DEG:
        raise ValueError(
            f'机械臂没有静止（0.2 秒内关节变化 {moved_deg:.3f} 度）'
        )
    return second


def plan(arm: arms.Arm, offset_mm: List[float], points: int) -> dict:
    """读取当前状态并求出沿直线的关节角路径；只做计算。

    Args:
        arm: 做试验的臂。
        offset_mm: 直线的位移，基座坐标，毫米。
        points: 路点个数。

    Returns:
        含 `config`、`tool`、`state`（当前状态）、`waypoints`、
        `detail`（路径检查记录，含各路点的关节解）的字典。

    Raises:
        ValueError: 机械臂没有静止或报错，或这条直线找不到连续且
            不碰撞的关节角路径。
    """
    config = json.loads(arm.config_path(ROOT).read_text())
    tool = yaml.safe_load((ROOT / config['tool_file']).read_text())
    state = read_stationary_state(
        grasp_robot.RobotClient(
            config['robot_ip'], config['robot_port'], writes=False
        )
    )
    environment = collision_model.load_environment(
        ROOT, arm.arm_id, config['table_z_m']
    )
    other = arm_occupancy.Occupancy(ROOT).other_arm(arm.arm_id)
    if other is not None:
        environment = dataclasses.replace(
            environment,
            other_arm_capsules=other.capsules,
            other_arm_margin_m=other.margin_m,
            other_arm_label=other.label,
        )
    waypoints = line_waypoints(state['pose'], offset_mm, points)
    valid, detail = motion_validation.check_waypoints(
        ROOT / config['ik_checker'], tool, state, waypoints,
        environment=environment,
    )
    if not valid:
        raise ValueError('这条直线走不了：' + detail['reason'])
    return {
        'config': config,
        'tool': tool,
        'state': state,
        'waypoints': waypoints,
        'detail': detail,
        'environment': environment,
    }


def plan_stream(
    planned: dict,
    summary: dict,
    max_speed_deg_s: float,
    max_acceleration_deg_s2: float,
    period_s: float,
) -> joint_trajectory.JointTrajectory:
    """给路径配上时间，并检查这条带时间的轨迹；只做计算。

    Args:
        planned: `plan` 的返回值。
        summary: `describe_plan` 的返回值。
        max_speed_deg_s: 任一关节转速的上限，度每秒。
        max_acceleration_deg_s2: 任一关节加速度的上限。
        period_s: 发送间隔，秒。

    Returns:
        轨迹。

    Raises:
        ValueError: 按发送间隔取出的各帧会碰到障碍物。
    """
    path_deg = np.asarray(summary['path_deg'])
    trajectory = joint_trajectory.plan_through(
        path_deg,
        max_speed_deg_s,
        max_acceleration_deg_s2,
        # 路点沿直线等距，夹爪沿直线的速度就按平滑的加速—减速变化。
        knots=np.linspace(0., 1., len(path_deg)),
    )
    frames = trajectory.sample(period_s)
    flange_T_tool = planned['tool']['flange_T_grip_tcp']
    deviation_mm = trajectory_recorder.tcp_line_deviation_m(
        frames[::5], flange_T_tool, summary['line_start_m'],
        summary['line_end_m'],
    ).max() * 1000
    check = collision_model.check_path(
        frames[::5], flange_T_tool, planned['environment']
    )
    if check['violation'] is not None:
        raise ValueError(
            '配上时间后的轨迹不安全：' + check['violation']['message']
        )
    LOG.info(
        '透传轨迹：全程 %.2f 秒，共 %d 帧（每 %.0f 毫秒一帧）；最快的关节 '
        '%.1f 度每秒，加速度最大 %.0f 度每二次方秒，相邻两帧最多转 %.3f 度',
        trajectory.duration_s, len(frames), period_s * 1000,
        trajectory.peak_speed_deg_s, trajectory.peak_acceleration_deg_s2,
        float(np.abs(np.diff(frames, axis=0)).max()),
    )
    LOG.info(
        '这条轨迹上夹爪参考点偏离直线最多 %.3f 毫米；离立杆最近 %s 毫米、'
        '离另一台臂 %s 毫米',
        deviation_mm, _mm(check['min_pole_clearance_m']),
        _mm(check['min_other_arm_clearance_m']),
    )
    return trajectory


def describe_plan(planned: dict) -> dict:
    """把计划整理成给人看的几个数，并打印出来。

    Args:
        planned: `plan` 的返回值。

    Returns:
        含 `path_deg`（起点加各路点的关节角）、每一步关节变化、理想
        情况下夹爪参考点偏离直线多少，以及离各障碍物最近多少。
    """
    start_deg = np.asarray(planned['state']['joint'], dtype=float) / 1000
    path_deg = np.vstack([start_deg, planned['detail']['joint_solutions_deg']])
    flange_T_tool = np.asarray(planned['tool']['flange_T_grip_tcp'])
    tips = [
        (collision_model.link_frames(joint)[7] @ flange_T_tool)[:3, 3]
        for joint in path_deg
    ]
    # 理想情况（严格走关节角直线）下夹爪参考点偏离直线多少。
    dense = np.vstack(
        [
            start + (end - start) * fraction
            for start, end in zip(path_deg[:-1], path_deg[1:])
            for fraction in np.linspace(0., 1., 21)
        ]
    )
    ideal_mm = trajectory_recorder.tcp_line_deviation_m(
        dense, flange_T_tool, tips[0], tips[-1]
    ).max() * 1000
    check = planned['detail']['collision_check']
    summary = {
        'path_deg': path_deg.tolist(),
        'line_start_m': tips[0].tolist(),
        'line_end_m': tips[-1].tolist(),
        'largest_step_deg': float(np.abs(np.diff(path_deg, axis=0)).max()),
        'total_change_deg': float(np.abs(path_deg[-1] - path_deg[0]).max()),
        'ideal_tcp_deviation_mm': float(ideal_mm),
        'min_table_clearance_mm': _mm(check['min_table_clearance_m']),
        'min_pole_clearance_mm': _mm(check['min_pole_clearance_m']),
        'min_other_arm_clearance_mm': _mm(check['min_other_arm_clearance_m']),
    }
    LOG.info(
        '起点 %s 毫米 → 终点 %s 毫米，共 %d 个路点',
        np.round(tips[0] * 1000).tolist(),
        np.round(tips[-1] * 1000).tolist(),
        len(path_deg) - 1,
    )
    LOG.info(
        '关节：全程最多转 %.1f 度，单步最多 %.1f 度；严格走关节角直线时'
        '夹爪参考点偏离直线最多 %.2f 毫米',
        summary['total_change_deg'],
        summary['largest_step_deg'],
        summary['ideal_tcp_deviation_mm'],
    )
    LOG.info(
        '全程离桌面最近 %s 毫米、离立杆 %s 毫米、离另一台臂 %s 毫米',
        summary['min_table_clearance_mm'],
        summary['min_pole_clearance_mm'],
        summary['min_other_arm_clearance_mm'],
    )
    return summary


def _mm(value_m: Optional[float]) -> Optional[float]:
    """把米换成毫米并保留一位小数；None 原样返回。"""
    return None if value_m is None else round(value_m * 1000, 1)


def execute(
    planned: dict,
    summary: dict,
    mode: str,
    speed: int,
    stop_after_s: Optional[float],
    blend_percent: int = 0,
    keep: Optional[List[int]] = None,
    trajectory: Optional[joint_trajectory.JointTrajectory] = None,
    period_s: float = .01,
    follow: bool = False,
) -> dict:
    """按计划让机械臂运动，并记录实际轨迹：会发送运动指令。

    出任何错都会立即发送停止指令。

    Args:
        planned: `plan` 的返回值。
        summary: `describe_plan` 的返回值。
        mode: `connected` 连成一条轨迹；`stepwise` 逐条发送；`stream`
            按固定节拍透传 `trajectory`。
        speed: 速度百分比。
        stop_after_s: 不为 None 时，在开始发送运动指令这么多秒之后
            要求停下：前两种方式发一次停止指令，用来观察排队的后续
            轨迹会不会接着走；`stream` 方式置取消标志，用来观察沿
            原轨迹减速停下的效果。
        blend_percent: 连接方式下中间各点的交融半径，0～100。
        keep: 只把路径上这些序号的点发给控制器（见
            `trajectory_recorder.thin_joint_path`）；None 表示全部。
        trajectory: `stream` 方式要走的带时间的轨迹。
        period_s: `stream` 方式的发送间隔，秒。
        follow: `stream` 方式下控制器的跟随方式，True 为高跟随。

    Returns:
        试验记录：控制器的应答、实测样本和出错信息（如果有）。
    """
    config = planned['config']
    robot = grasp_robot.RobotClient(
        config['robot_ip'], config['robot_port'], writes=True
    )
    robot.motion_stage = f'trajectory_connect_test:{mode}'
    targets = [
        np.rint(np.asarray(summary['path_deg'][index]) * 1000)
        .astype(int)
        .tolist()
        for index in (keep or range(1, len(summary['path_deg'])))
    ]
    recorder = trajectory_recorder.TrajectoryRecorder(
        (config['robot_ip'], config['robot_port'])
    )
    cancelled = threading.Event()
    record = {
        'mode': mode,
        'speed_percent': speed,
        'blend_percent': blend_percent,
        'kept_points': keep,
        'targets_raw': targets,
    }
    recorder.start()
    time.sleep(.3)
    record['motion_started_s'] = recorder.elapsed_s()
    stopper = None
    if stop_after_s is not None:

        def send_stop() -> None:
            record['stop_sent_s'] = recorder.elapsed_s()
            if mode == 'stream':
                cancelled.set()
                record['stop_result'] = 'cancel_flag_set'
                return
            try:
                robot.stop()
                record['stop_result'] = 'acknowledged'
            except RuntimeError as error:
                record['stop_result'] = str(error)

        stopper = threading.Timer(stop_after_s, send_stop)
        stopper.start()
    try:
        if mode == 'stream':
            # 发送循环与记录器用同一个时钟，指令和实测才能按时刻比较。
            robot.stream_joints(
                trajectory.position_deg, trajectory.duration_s, cancelled,
                lambda: None, period_s, follow,
                actual_joint_deg=recorder.latest_joint_deg,
                record_frames=True, clock=recorder.elapsed_s,
            )
        elif mode == 'connected':
            robot.move_joints_chain(
                targets, speed, cancelled, lambda: None, MOTION_TIMEOUT_S,
                blend_percent,
            )
        else:
            for target in targets:
                robot.move_joints(
                    target, speed, cancelled, lambda: None, MOTION_TIMEOUT_S
                )
        record['outcome'] = 'completed'
    except BaseException as error:
        record.update(
            outcome='failed', error=f'{type(error).__name__}: {error}'
        )
        LOG.error('运动没有正常完成：%s', record['error'])
        try:
            robot.stop()
            record.setdefault('stop_result', 'acknowledged_after_failure')
        except RuntimeError as stop_error:
            LOG.error('停止指令未确认：%s', stop_error)
            record['stop_error'] = str(stop_error)
        if not isinstance(error, Exception):
            raise
    finally:
        record['motion_ended_s'] = recorder.elapsed_s()
        if stopper is not None:
            stopper.cancel()
        # 多记一会儿：看停下之后还会不会自己接着动。
        time.sleep(2.5 if stop_after_s is not None else .8)
        record['samples'] = recorder.stop()
        record['recorder_error'] = recorder.error
        record['motion_events'] = robot.motion_log
    return record


def analyse(planned: dict, summary: dict, record: dict) -> dict:
    """把实测轨迹与计划比较。

    Args:
        planned: `plan` 的返回值。
        summary: `describe_plan` 的返回值。
        record: `execute` 的返回值。

    Returns:
        各项结论：偏离计划路径多少、夹爪参考点偏离直线多少、离每个
        路点最近时差多少、途中停了几次、用时、终点误差。
    """
    samples = record['samples']
    if len(samples) < 3:
        return {'samples': len(samples)}
    joint_deg = [sample['joint_deg'] for sample in samples]
    path_deg = summary['path_deg']
    stops = trajectory_recorder.stops_during_motion(samples)
    _, speed = trajectory_recorder.speed_deg_s(samples)
    moving = np.flatnonzero(speed >= trajectory_recorder.STOP_SPEED_DEG_S)
    times = [sample['t_s'] for sample in samples]
    return {
        'samples': len(samples),
        'sample_period_ms': float(
            np.median(np.diff([sample['t_s'] for sample in samples])) * 1000
        ),
        'max_joint_deviation_from_plan_deg': float(
            trajectory_recorder.deviation_from_polyline(
                joint_deg, path_deg
            ).max()
        ),
        'max_tcp_deviation_from_line_mm': float(
            trajectory_recorder.tcp_line_deviation_m(
                joint_deg,
                planned['tool']['flange_T_grip_tcp'],
                summary['line_start_m'],
                summary['line_end_m'],
            ).max()
            * 1000
        ),
        'closest_approach_to_each_waypoint_deg': (
            trajectory_recorder.closest_approach_deg(joint_deg, path_deg[1:])
        ),
        'stops_during_motion': stops,
        'moving_time_s': (
            float(times[moving[-1]] - times[moving[0]])
            + trajectory_recorder.SPEED_SPAN_S
            if len(moving)
            else 0.
        ),
        'peak_joint_speed_deg_s': float(speed.max()),
        'final_error_deg': float(
            np.abs(np.asarray(joint_deg[-1]) - path_deg[-1]).max()
        ),
        'tracking': _stream_tracking(record),
    }


def _stream_tracking(record: dict) -> dict:
    """透传方式下实测与指令的比较；其他方式返回空字典。"""
    frames = next(
        (e['frames'] for e in record['motion_events'] if 'frames' in e), None
    )
    if not frames:
        return {}
    result = trajectory_recorder.tracking(record['samples'], frames)
    gaps = np.diff([frame[0] for frame in frames])
    result.update(
        frames_sent=len(frames),
        send_period_ms_median=float(np.median(gaps) * 1000),
        send_period_ms_max=float(gaps.max() * 1000),
    )
    return result


def report(analysis: dict, record: dict) -> None:
    """把分析结果打印出来。

    Args:
        analysis: `analyse` 的返回值。
        record: `execute` 的返回值。
    """
    if analysis.get('samples', 0) < 3:
        LOG.error(
            '记录到的样本太少（%s 个），无法分析', analysis.get('samples')
        )
        return
    LOG.info(
        '结果：%s；运动用时 %.2f 秒，最快 %.1f 度每秒；每 %.1f 毫秒记录一次，'
        '共 %d 个样本',
        record['outcome'],
        analysis['moving_time_s'],
        analysis['peak_joint_speed_deg_s'],
        analysis['sample_period_ms'],
        analysis['samples'],
    )
    LOG.info(
        '实际轨迹偏离计划的关节角折线最多 %.3f 度；夹爪参考点偏离直线最多 '
        '%.2f 毫米',
        analysis['max_joint_deviation_from_plan_deg'],
        analysis['max_tcp_deviation_from_line_mm'],
    )
    LOG.info(
        '离各路点最近时相差（度）：%s',
        [
            round(value, 3)
            for value in analysis['closest_approach_to_each_waypoint_deg']
        ],
    )
    LOG.info(
        '途中停顿 %d 次：%s',
        len(analysis['stops_during_motion']),
        [round(s['duration_s'], 2) for s in analysis['stops_during_motion']],
    )
    LOG.info('终点与目标相差 %.3f 度', analysis['final_error_deg'])
    followed = analysis.get('tracking')
    if followed:
        LOG.info(
            '透传：发了 %d 帧，间隔中位 %.1f 毫秒、最长 %.1f 毫秒；实测与'
            '同一时刻的指令最多差 %.2f 度，相当于滞后 %.0f 毫秒；扣除滞后'
            '后最多差 %.3f 度',
            followed['frames_sent'], followed['send_period_ms_median'],
            followed['send_period_ms_max'],
            followed.get('max_error_deg', float('nan')),
            followed.get('lag_s', float('nan')) * 1000,
            followed.get('max_error_after_shift_deg', float('nan')),
        )
    for event in record['motion_events']:
        for response in event['responses']:
            LOG.info(
                '控制器应答 %.3f 秒：%s',
                response['elapsed_s'],
                response['raw'].strip(),
            )


def main() -> int:
    """解析命令行，做计划；带 `--execute` 时让机械臂运动并记录。"""
    parser = argparse.ArgumentParser(description=__doc__.split('\n')[0])
    parser.add_argument('--arm', default=arms.DEFAULT_ARM_ID)
    parser.add_argument(
        '--offset-mm', type=float, nargs=3, default=[0., 0., -100.],
        metavar=('X', 'Y', 'Z'),
        help='直线的位移，基座坐标，毫米；缺省向下 100',
    )
    parser.add_argument('--points', type=int, default=5, help='路点个数')
    parser.add_argument('--speed', type=int, default=10, help='速度百分比')
    parser.add_argument(
        '--mode', choices=('connected', 'stepwise', 'stream'),
        default='connected',
    )
    parser.add_argument(
        '--max-speed-deg-s', type=float, default=15.,
        help='stream 方式：任一关节转速的上限，度每秒',
    )
    parser.add_argument(
        '--max-acceleration-deg-s2', type=float, default=40.,
        help='stream 方式：任一关节加速度的上限，度每二次方秒',
    )
    parser.add_argument(
        '--period-ms', type=float, default=10.,
        help='stream 方式：发送间隔，毫秒',
    )
    parser.add_argument(
        '--follow', choices=('low', 'high'), default='low',
        help='stream 方式：控制器的跟随方式',
    )
    parser.add_argument(
        '--blend-percent', type=int, default=0,
        help='连接方式下中间各点的交融半径，0～100；0 时每个点都会停一下',
    )
    parser.add_argument(
        '--max-deviation-mm', type=float, default=None,
        help=(
            '按这个允许偏差从取样点里只挑必须保留的发给控制器；'
            '不给时全部发送'
        ),
    )
    parser.add_argument(
        '--keep', type=int, nargs='+', default=None, metavar='N',
        help='只把这几个序号的取样点发给控制器（从 1 起，须含最后一个）',
    )
    parser.add_argument(
        '--stop-after-s', type=float, default=None,
        help='开始运动这么多秒后发一次停止指令（观察排队的轨迹）',
    )
    parser.add_argument(
        '--execute', action='store_true',
        help='让机械臂运动；不带时只打印计划',
    )
    args = parser.parse_args()
    logging.basicConfig(
        level=logging.INFO, format='%(asctime)s %(levelname)s %(message)s',
        datefmt='%H:%M:%S',
    )
    distance_mm = float(np.linalg.norm(args.offset_mm))
    if (
        not 0 < distance_mm <= MAX_DISTANCE_MM
        or not 1 <= args.points <= 30
        or not 1 <= args.speed <= MAX_SPEED_PERCENT
        or not 0 <= args.blend_percent <= 100
        or not 0 < args.max_speed_deg_s <= MAX_STREAM_SPEED_DEG_S
        or not 0 < args.max_acceleration_deg_s2 <= MAX_STREAM_ACCEL_DEG_S2
        or not 2 <= args.period_ms <= 20
        or (args.stop_after_s is not None and args.stop_after_s <= 0)
        or (
            args.max_deviation_mm is not None
            and not 0 < args.max_deviation_mm <= 50
        )
        or (
            args.keep is not None
            and (
                args.max_deviation_mm is not None
                or args.keep != sorted(set(args.keep))
                or args.keep[0] < 1
                or args.keep[-1] != args.points
            )
        )
    ):
        parser.error(
            f'直线须在 {MAX_DISTANCE_MM:g} 毫米以内，路点 1～30 个，速度 '
            f'1～{MAX_SPEED_PERCENT}%，交融半径 0～100，停止时刻须大于 0；'
            '--keep 须升序、以最后一个取样点结尾，且不与 --max-deviation-mm '
            f'同用；透传的转速上限不超过 {MAX_STREAM_SPEED_DEG_S:g} 度每秒、'
            f'加速度不超过 {MAX_STREAM_ACCEL_DEG_S2:g}，间隔 2～20 毫秒'
        )
    arm = arms.by_id(args.arm)
    LOG.info(
        '%s：沿 %s 毫米走 %d 个路点，方式 %s，速度 %d%%，交融半径 %d',
        arm.label, args.offset_mm, args.points, args.mode, args.speed,
        args.blend_percent,
    )
    try:
        planned = plan(arm, args.offset_mm, args.points)
    except (ValueError, OSError, RuntimeError) as error:
        LOG.error('%s；未发送运动', error)
        return 1
    summary = describe_plan(planned)
    keep = args.keep
    if keep is not None:
        sent = np.asarray(summary['path_deg'])[[0] + keep]
        LOG.info(
            '只发第 %s 个取样点，共 %d 条指令；相邻两条之间关节最多转 '
            '%.1f 度',
            keep, len(keep), float(np.abs(np.diff(sent, axis=0)).max()),
        )
    if args.max_deviation_mm is not None:
        keep = trajectory_recorder.thin_joint_path(
            summary['path_deg'],
            planned['tool']['flange_T_grip_tcp'],
            args.max_deviation_mm / 1000,
        )
        LOG.info(
            '允许偏离直线 %g 毫米：%d 个取样点里只发第 %s 个，共 %d 条指令',
            args.max_deviation_mm, args.points, keep, len(keep),
        )
    trajectory = None
    if args.mode == 'stream':
        try:
            trajectory = plan_stream(
                planned, summary, args.max_speed_deg_s,
                args.max_acceleration_deg_s2, args.period_ms / 1000,
            )
        except ValueError as error:
            LOG.error('%s；未发送运动', error)
            return 1
    if not args.execute:
        LOG.info('只做了计算，未发送运动。确认无误后加 --execute 执行。')
        return 0
    lock_path = arm.lock_path(ROOT)
    lock_path.parent.mkdir(parents=True, exist_ok=True)
    with lock_path.open('a') as lock:
        try:
            fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
        except BlockingIOError:
            LOG.error('%s的设备锁被占用（有任务在执行）；未发送运动', arm.label)
            return 1
        LOG.info('开始运动，Ctrl-C 可随时停止')
        record = execute(
            planned, summary, args.mode, args.speed, args.stop_after_s,
            args.blend_percent, keep, trajectory, args.period_ms / 1000,
            args.follow == 'high',
        )
    analysis = analyse(planned, summary, record)
    report(analysis, record)
    directory = ROOT / OUTPUT_DIRECTORY
    directory.mkdir(parents=True, exist_ok=True)
    path = directory / (
        f'{time.strftime("%Y%m%d_%H%M%S")}_{arm.arm_id}_{args.mode}.json'
    )
    path.write_text(
        json.dumps(
            {
                'arm': arm.arm_id,
                'arguments': vars(args),
                'initial_state': planned['state'],
                'waypoints': planned['waypoints'],
                'plan': summary,
                'record': record,
                'analysis': analysis,
            },
            ensure_ascii=False,
            indent=1,
        )
    )
    LOG.info('记录已保存：%s', path.relative_to(ROOT))
    return 0 if record['outcome'] == 'completed' else 1


if __name__ == '__main__':
    raise SystemExit(main())
