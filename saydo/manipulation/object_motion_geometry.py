"""General held-object goals. Object references, not TCP origins, define targets."""

import concurrent.futures
import subprocess
from typing import Callable, Optional, Tuple

import numpy as np
import scipy.spatial

from saydo.geometry import rotation_steps
from saydo.geometry import grasp_geometry
from saydo.objects import grasp_library
from saydo.manipulation import motion_speed
from saydo.manipulation import placement_geometry
from saydo.geometry import pose_symmetry
from saydo.manipulation import reorientation_geometry
from saydo.contracts import motion_contract

MAX_WAYPOINTS = 64
MAX_VALIDATION_POINTS = 480
POSITION_TOLERANCE = .002
ANGLE_TOLERANCE_DEG = 2.
# 同时检查的候选方案数。每个候选主要耗在一个独立的逆解子进程里；电脑有
# 10 个物理核心，取 8 个：再多会让各子进程互相抢处理器，反而触发它们
# 内部的时间上限。
SEARCH_WORKERS = 8
# 转移路线：(比起止点再抬高多少米, 向侧面绕开多少米)。先试直接过去。
ROUTE_OFFSETS = (
    (0., 0.),
    (0., .08),
    (0., -.08),
    (0., .16),
    (0., -.16),
    (.05, 0.),
    (.10, 0.),
    (.05, .08),
    (.05, -.08),
    (.10, .08),
    (.10, -.08),
    (.15, 0.),
)
# 松爪前把物品转到目标朝向时，在“当前位置到预放置点”这段直线的哪个比例处
# 转到位。0 是原地转（原来唯一的做法，最先试）；1 是边走边转、到预放置点
# 时刚好转完；0.5 是走到一半时转完。原地转不了的位置（例如紧挨着立杆），
# 边走边转往往可以。
TURN_FRACTIONS = (0., 1., .5)


def object_at_reference(reference, rotation, point):
    result = np.eye(4)
    result[:3, :3] = rotation
    result[:3, 3] = np.asarray(reference) - rotation @ point
    return result


def validate_tcp(tcp, held, vertices, config):
    placement_geometry.check_tcp_workspace(tcp, config)
    reorientation_geometry.validate_object_pose(
        tcp, held, vertices, config, False
    )


def segment_samples(start, end, held, vertices, config, method, name):
    distance = float(np.linalg.norm(end[:3, 3] - start[:3, 3]))
    angle = grasp_geometry.rotation_error(start[:3, :3], end[:3, :3])
    steps = (
        max(
            1, int(np.ceil(distance / .01)), int(np.ceil(angle / np.deg2rad(5)))
        )
        if method == 'linear'
        else 1
    )
    if steps > MAX_VALIDATION_POINTS:
        raise ValueError('单段采样数量过多')
    points = []
    for k in range(1, steps + 1):
        fraction = k / steps
        tcp = np.eye(4)
        tcp[:3, 3] = start[:3, 3] + fraction * (end[:3, 3] - start[:3, 3])
        tcp[:3, :3] = rotation_steps.interpolate(
            start[:3, :3], end[:3, :3], fraction
        )
        if held is not None:
            validate_tcp(tcp, held, vertices, config)
        else:
            placement_geometry.check_tcp_workspace(tcp, config)
        w = reorientation_geometry.pose_waypoint(f'{name}_sample_{k}', tcp)
        w['method'] = method
        points.append(w)
    return points


def object_error(actual, target, point, pose_mode):
    actual, target = (
        grasp_geometry.transform(actual), grasp_geometry.transform(target)
    )
    point = np.asarray(point)
    return {
        'reference_position_error_m': float(
            np.linalg.norm(
                (actual[:3, 3] + actual[:3, :3] @ point)
                - (target[:3, 3] + target[:3, :3] @ point)
            )
        ),
        'orientation_error_deg': float(
            np.rad2deg(
                pose_symmetry.object_rotation_error(
                    actual[:3, :3], target[:3, :3], pose_mode
                )
            )
        ),
    }


def build_candidate(
    held: dict,
    vertices: np.ndarray,
    arm: dict,
    config: dict,
    goal: dict,
    frame: np.ndarray,
    spin: int,
    shift: np.ndarray,
    release: bool,
    retreat: np.ndarray,
    route: tuple = (0., 0.),
    turn_at: float = 0.,
) -> dict:
    """按一组候选参数生成一条完整的持物运动路径；只做几何计算。

    路径由若干段组成：把物品转到目标朝向、平移到目标位置；松爪时再加上
    “到预放置点（目标正上方）”和“竖直下降”两段。每一段按最大转角和
    10 毫米间距取样，逐个姿态检查夹爪在工作范围内、物品不低于桌面。
    是否走得到（逆解、连续性、碰撞）由调用方另行检查。

    Args:
        held: 夹持记录，取 `object_T_tcp` 和 `pose_mode`。
        vertices: 物品模型顶点，N×3，物品坐标系，单位米。
        arm: 机械臂状态，路径从它的夹爪位姿开始。
        config: 这台臂生效的抓取配置。
        goal: 已校验的目标。
        frame: 目标所在坐标系在基座坐标系里的位姿，4×4。
        spin: 夹持方向的序号。5D 物品绕自身轴线转动不改变朝向，夹爪
            可以从 24 个方向夹着它；其他物品只有 0。
        shift: 自适应位置模式下目标位置的偏移，基座坐标系，单位米。
        release: 终点是否松爪。
        retreat: 松爪后的退出位移，基座坐标系，单位米。
        route: 转移路线 `(再抬高多少米, 向侧面绕开多少米)`。
        turn_at: 松爪前在“当前位置到预放置点”这段直线的哪个比例处
            把物品转到目标朝向。0 为原地转，1 为到预放置点时转完。

    Returns:
        候选方案：路点 `waypoints`、检查用的取样点 `validation_points`、
        目标位姿、退出路点等。

    Raises:
        ValueError: 目标或参考点无效，某个姿态超出工作范围或物品低于
            桌面，或路点数超过上限。
    """
    current = grasp_geometry.controller_pose(arm['pose'])
    attachment = grasp_geometry.transform(held['object_T_tcp'])
    obj = current @ np.linalg.inv(attachment)
    point = np.asarray(goal['reference_point_object_m'])
    low, high = grasp_library.point_limits(vertices)
    if np.any(point < low) or np.any(point > high):
        raise ValueError('物品参考点超出模型附近允许范围')
    if held.get('pose_mode') == '5d' and np.any(np.abs(point[:2]) > 1e-9):
        raise ValueError('5D物品参考点必须位于模型Z轴（X/Y为0）')
    validate_tcp(current, held, vertices, config)
    start_obj = obj.copy()
    start_ref = obj[:3, 3] + obj[:3, :3] @ point
    rows = list(goal['waypoints'])
    if (
        goal['target_position_m'] is not None
        or goal['target_rpy_deg'] is not None
        or not rows
    ):
        rows.append(
            {
                'position_m': goal['target_position_m'],
                'rpy_deg': goal['target_rpy_deg'],
            }
        )
    # One final orientation for the whole operation; intermediate points may
    # change position only. Resolve it before any translation can be planned.
    rows = [dict(row) for row in rows]
    orientations = [
        row['rpy_deg'] for row in rows if row['rpy_deg'] is not None
    ]
    if orientations:
        final_rpy = orientations[-1]
        if any(
            (
                grasp_geometry.rotation_error(
                    reorientation_geometry.rotation_from_rpy_deg(r),
                    reorientation_geometry.rotation_from_rpy_deg(final_rpy),
                )
                > 1e-6
            )
            for r in orientations
        ):
            raise ValueError(
                '中间路点不得改变最终目标朝向；先调姿，再保持朝向搬运'
            )
        rows[0]['rpy_deg'] = final_rpy
    chosen_rotation = None
    waypoints = []
    validation = []
    previous_tcp = current
    target_requests = []
    for segment, row in enumerate(rows):
        previous_ref = obj[:3, 3] + obj[:3, :3] @ point
        reference = (
            (frame @ np.r_[row['position_m'], 1])[:3]
            if row['position_m'] is not None
            else start_ref + shift
        )
        if goal['position_mode'] == 'fixed' and row['position_m'] is None:
            reference = previous_ref
        requested_rotation = (
            (
                frame[:3, :3]
                @ reorientation_geometry.rotation_from_rpy_deg(row['rpy_deg'])
            )
            if row['rpy_deg'] is not None
            else obj[:3, :3]
        )
        # Only explicit orientation goals permit free-spin search. Position-only
        # moves keep the current representative/TCP orientation unchanged.
        if chosen_rotation is not None:
            rotation = chosen_rotation
        elif row['rpy_deg'] is not None and held.get('pose_mode') == '5d':
            rotations = pose_symmetry.orientation_variants(
                requested_rotation, held, previous_tcp
            )
            rotation = rotations[spin % len(rotations)]
        else:
            rotation = requested_rotation
        chosen_rotation = rotation.copy()
        target_requests.append(
            {
                'reference_position_base_m': reference.tolist(),
                'requested_rotation_base': requested_rotation.tolist(),
            }
        )
        method = 'linear' if goal['path_mode'] == 'linear' else 'joint_pose'
        stages = [
            (previous_ref, method, f'orient_{segment+1}'),
            (reference, 'linear', f'transfer_{segment+1}'),
        ]
        if release and segment == len(rows) - 1:
            height = float(config.get('release_preplacement_height_m', .1))
            if not np.isfinite(height) or not .01 <= height <= .3:
                raise ValueError('预放置高度须为10至300毫米')
            preplacement = reference + np.array([0, 0, height])
            # turn_at 为 0 时原地把物品转到目标朝向，再平移过去；大于 0 时
            # 边走边转，走完这段直线的 turn_at 比例时转到位。松爪用的
            # 参考点（例如试管底端）始终在这条直线上，物品其余部分只会
            # 在它的上方或同一高度摆动。
            stages = [
                (
                    previous_ref + turn_at * (preplacement - previous_ref),
                    method,
                    'release_orientation',
                ),
                (
                    preplacement,
                    'linear',
                    'release_preplacement',
                ),
                (reference, 'linear', 'release_descent'),
            ]
        if route != (0., 0.):
            expanded = []
            for destination, stage_method, label in stages:
                if (
                    label.startswith('transfer_')
                    or label == 'release_preplacement'
                ):
                    start = obj[:3, 3] + obj[:3, :3] @ point
                    height = max(start[2], destination[2]) + route[0]
                    delta = destination[:2] - start[:2]
                    length = np.linalg.norm(delta)
                    side = (
                        np.array([-delta[1], delta[0]]) / length
                        if length > 1e-9
                        else np.array([1., 0.])
                    )
                    offset = np.r_[side * route[1], 0.]
                    a = np.array([start[0], start[1], height])
                    b = np.array([destination[0], destination[1], height])
                    for index, via in enumerate(
                        (a, a + offset, b + offset, b), 1
                    ):
                        expanded.append(
                            (via, 'linear', f'{label}_detour_{index}')
                        )
                expanded.append((destination, stage_method, label))
            stages = expanded
        for stage_ref, stage_method, label in stages:
            previous_ref = obj[:3, 3] + obj[:3, :3] @ point
            steps = rotation_steps.fractions(
                obj[:3, :3], rotation, goal['max_rotation_step_deg']
            )
            origin_obj = obj.copy()
            for k, f in enumerate(steps, 1):
                rot = rotation_steps.interpolate(
                    origin_obj[:3, :3], rotation, f
                )
                reference_k = previous_ref + f * (stage_ref - previous_ref)
                target_obj = object_at_reference(reference_k, rot, point)
                tcp = target_obj @ attachment
                w = reorientation_geometry.pose_waypoint(f'{label}_{k}', tcp)
                w['method'] = stage_method
                quantized = grasp_geometry.controller_pose(w['controller_pose'])
                validate_tcp(quantized, held, vertices, config)
                w['object_pose'] = target_obj.tolist()
                w['reference_position_base_m'] = reference_k.tolist()
                validation.extend(
                    segment_samples(
                        previous_tcp,
                        quantized,
                        held,
                        vertices,
                        config,
                        w['method'],
                        w['name'],
                    )
                )
                if (
                    (
                        np.linalg.norm(quantized[:3, 3] - previous_tcp[:3, 3])
                        > 1e-7
                    )
                    or (
                        grasp_geometry.rotation_error(
                            quantized[:3, :3], previous_tcp[:3, :3]
                        )
                        > 1e-6
                    )
                ):
                    waypoints.append(w)
                previous_tcp = quantized
            obj = object_at_reference(stage_ref, rotation, point)
    if (
        goal['position_mode'] == 'adaptive'
        and previous_tcp[2, 3] <= config['table_z_m'] + .10
    ):
        raise ValueError('自适应姿态最终TCP须高于桌面10厘米')
    if len(waypoints) > MAX_WAYPOINTS:
        raise ValueError('方案超过64个运动路点，请拆分任务')
    retreat_target = None
    if release and np.linalg.norm(retreat) > 0:
        endpoint = previous_tcp.copy()
        endpoint[:3, 3] += retreat
        placement_geometry.check_tcp_workspace(endpoint, config)
        retreat_target = reorientation_geometry.pose_waypoint(
            'release_retreat', endpoint
        )
        retreat_target['method'] = 'linear'
        validation.extend(
            segment_samples(
                previous_tcp,
                endpoint,
                None,
                None,
                config,
                'linear',
                'release_retreat',
            )
        )
    if len(validation) > MAX_VALIDATION_POINTS:
        raise ValueError('方案超过480个检查采样点，请拆分任务')
    if not validation:
        validation = [
            reorientation_geometry.pose_waypoint('stationary', current)
        ]
    final_ref = obj[:3, 3] + obj[:3, :3] @ point
    return {
        'kind': 'release' if release else 'held_object_motion',
        'goal': goal,
        'pose_mode': held.get('pose_mode', '6d'),
        'current_arm': arm,
        'base_T_object_start': start_obj.tolist(),
        'base_T_object_target': obj.tolist(),
        'reference_point_object_m': point.tolist(),
        'target_reference_position_base_m': final_ref.tolist(),
        'reference_frame_pose': frame.tolist(),
        'target_requests': target_requests,
        'waypoints': waypoints,
        'waypoint_count': len(waypoints),
        'position_constraint_scope': 'commanded_waypoint_endpoints',
        'stage_policy': 'orient_then_translate_then_release',
        'route_offset_m': list(route),
        'validation_points': validation,
        'retreat_waypoint': retreat_target,
        'retreat_vector_base_m': retreat.tolist(),
        'speed_percent': motion_speed.checked_speed(
            config['arm_speed_percent']
        ),
        'position_tolerance_m': POSITION_TOLERANCE,
        'orientation_tolerance_deg': ANGLE_TOLERANCE_DEG,
        'gripper_action': 'open_after_arrival' if release else 'keep_holding',
        'max_execution_duration_s': 90,
        'validation_scope': {
            'ik_fk': True,
            'self_collision_at_samples': True,
            'tcp_workspace': True,
            'object_table_clearance': True,
            'linear_path_sampled': True,
            'joint_intermediate_path_checked': False,
            'gripper_collision_checked': False,
            'environment_collision_checked': False,
            'object_support_checked': False,
        },
        'release_preplacement_height_m': (
            float(config.get('release_preplacement_height_m', .1))
            if release
            else None
        ),
        'release_semantics': (
            (
                'target pose before opening; settled pose after release not '
                'verified'
            )
            if release
            else None
        ),
    }


def hull_vertices(vertices: np.ndarray) -> np.ndarray:
    """只保留模型凸包上的顶点。

    规划时对每个取样姿态都要算物品的包围范围（会不会低于桌面、是否
    超出工作范围）。包围范围只由最外面的点决定，凸包以内的点不起作用；
    绕轴自由旋转（5D）的包围范围同样只由凸包顶点决定。试管模型有 1.2 万
    个顶点，凸包上只有几百个，检查快几十倍而结果不变。

    Args:
        vertices: 模型顶点，N×3。

    Returns:
        凸包顶点，M×3。点太少或共面、求不出凸包时原样返回。
    """
    points = np.asarray(vertices, dtype=float)
    if points.ndim != 2 or points.shape[1] != 3 or len(points) < 16:
        return points
    try:
        return points[scipy.spatial.ConvexHull(points).vertices]
    except scipy.spatial.QhullError:
        return points


class OrientationUnreachable(ValueError):
    """所有夹持方向都卡在调姿段：换一个目标位置也救不回来。

    调姿在原地进行，排在转移之前，与要去哪里无关。固定抓放据此判断
    不必再换孔位重试。
    """


def outward_scores(
    held: dict, arm: dict, goal: dict, frame: np.ndarray, count: int
) -> list:
    """给各个夹持方向打分：终点处夹爪越是从机械臂这一侧伸出去，分越高。

    夹爪轴线朝外（背向底座）时手臂是自然伸展的姿态，最容易求出连续的
    关节解；朝向底座或横着伸时往往要把手肘甩到别扭的一侧。2026-10-10
    统计 21 次成功的固定抓放规划，成功的都是朝外的那几个夹持方向。

    只用目标朝向和夹持关系做几何计算，不生成路径。

    Args:
        held: 夹持记录，取 `object_T_tcp` 和 `pose_mode`。
        arm: 机械臂状态，取当前夹爪位姿。
        goal: 已校验的目标。
        frame: 目标所在坐标系在基座坐标系里的位姿，4×4。
        count: 夹持方向的个数。

    Returns:
        长度为 `count` 的得分列表：夹爪轴线的水平分量与“底座指向目标
        位置”方向的点积，范围 −1 到 1。夹爪竖直时各方向得分相近。没有
        指定朝向时全为 0。
    """
    rows = list(goal['waypoints']) + [
        {'position_m': goal['target_position_m'],
         'rpy_deg': goal['target_rpy_deg']}
    ]
    orientations = [r['rpy_deg'] for r in rows if r['rpy_deg'] is not None]
    if not orientations:
        return [0.] * count
    current = grasp_geometry.controller_pose(arm['pose'])
    attachment = grasp_geometry.transform(held['object_T_tcp'])
    variants = pose_symmetry.orientation_variants(
        frame[:3, :3]
        @ reorientation_geometry.rotation_from_rpy_deg(orientations[-1]),
        held,
        current,
    )
    positions = [r['position_m'] for r in rows if r['position_m'] is not None]
    target_xy = (
        (frame @ np.r_[positions[-1], 1])[:2] if positions else current[:2, 3]
    )
    radius_m = float(np.linalg.norm(target_xy))
    if radius_m < 1e-6:
        return [0.] * count
    scores = []
    for spin in range(count):
        tool_axis = (variants[spin % len(variants)] @ attachment[:3, :3])[:, 2]
        scores.append(float(tool_axis[:2] @ target_xy / radius_m))
    return scores


def plan_object_motion(
    held: dict,
    vertices: np.ndarray,
    arm: dict,
    config: dict,
    goal: dict,
    frame: np.ndarray,
    reachable: Callable[[list, float], Tuple[bool, dict]],
    cancelled: Callable[[], None],
    release: bool = False,
    retreat: Optional[list] = None,
    diagnostics: Optional[list] = None,
    selected_points: Optional[list] = None,
) -> dict:
    """为夹着的物品找一条全程可行的调姿与搬运路径；只做计算。

    候选方案由三样东西组合而成：目标位置的偏移（仅自适应模式）、夹爪绕
    物品轴线转到哪个方向（仅 5D 物品且指定了朝向时，24 个，每 15 度一个）、
    转移时走哪条路（直接过去，或抬高、向侧面绕行，共 12 条）。逐个交给
    `reachable` 做逆解、连续性和碰撞检查，取优先顺序里第一个通过的。

    优先顺序：先试不绕行的路线；同一条路线里先试夹爪转动最少的方向，
    再按夹爪朝外伸的程度依次试其余方向（见 `outward_scores`）。

    候选之间互不依赖，所以每批同时检查 `SEARCH_WORKERS` 个（第一个单独
    检查），再按优先顺序取结果：与按同样顺序逐个检查选出的方案相同，
    只是更快。某个方向卡在调姿段时，换路线也没用，不再试它。

    Args:
        held: 夹持记录，取 `object_T_tcp` 和 `pose_mode`。
        vertices: 物品模型的顶点，N×3，物品坐标系，单位米。
        arm: 机械臂状态，路径从这里开始。
        config: 这台臂生效的抓取配置。
        goal: 目标：参考点、目标位置与朝向、途经点、路径与位置模式。
        frame: 目标位置所在坐标系在基座坐标系里的位姿，4×4。
        reachable: 路径检查函数，参数为取样点和时间上限（秒），返回
            `(通过与否, 记录)`。会被多个线程同时调用。
        cancelled: 取消检查，需要中止时由它抛出异常。
        release: 终点是否松爪（会加上预放置和下降两段）。
        retreat: 松爪后的退出位移，基座坐标系，单位米。
        diagnostics: 给出时，每个候选的结果按优先顺序追加到这个列表，
            最多保留 64 条。
        selected_points: 给出时，填入被选中方案的全部检查取样点。

    Returns:
        方案：路点、目标位姿、`reachability_check`（含各取样点的关节解）
        和搜索记录 `search_attempts`。

    Raises:
        OrientationUnreachable: 所有夹持方向都卡在调姿段。
        ValueError: 目标无效，或没有候选通过检查。
    """
    goal = motion_contract.checked_goal(goal, release)
    frame = grasp_geometry.transform(frame)
    retreat = np.asarray(
        motion_contract.vector(
            retreat or [0, 0, 0], 'retreat_vector_base_m', .3
        )
    )
    if np.linalg.norm(retreat) > .3:
        raise ValueError('退出位移长度不得超过300毫米')
    attempts = diagnostics if diagnostics is not None else []
    current = grasp_geometry.controller_pose(arm['pose'])
    vertices = hull_vertices(vertices)
    spins = (
        range(24)
        if (
            held.get('pose_mode') == '5d'
            and (
                goal['target_rpy_deg'] is not None
                or any(r['rpy_deg'] is not None for r in goal['waypoints'])
            )
        )
        else range(1)
    )
    adaptive = goal['position_mode'] == 'adaptive'
    shifts = (
        reorientation_geometry.position_offsets(
            current[:3, 3], config['table_z_m'] + .103, .025
        )
        if adaptive
        else [np.zeros(3)]
    )

    turns = TURN_FRACTIONS if release and len(spins) > 1 else (0.,)

    def build(
        spin: int, shift: np.ndarray, route: tuple, turn_at: float
    ) -> dict:
        return build_candidate(
            held, vertices, arm, config, goal, frame, spin, shift, release,
            retreat, route, turn_at,
        )

    def evaluate(candidate: tuple) -> tuple:
        """检查一个候选；在工作线程里运行。

        返回 (记录, 方案或 None, 取样点)。
        """
        shift, spin, route, turn_at = candidate
        attempt = {
            'spin_candidate': spin,
            'shift_m': shift.tolist(),
            'route_offset_m': list(route),
            'turn_at': turn_at,
        }
        try:
            plan = build(spin, shift, route, turn_at)
            plan['turn_at'] = turn_at
            points = plan.pop('validation_points')
            valid, detail = reachable(points, 3.)
            if not valid:
                attempt['failure'] = detail.get('failure')
                attempt['beam_max_width'] = max(
                    (
                        row['candidates']
                        for row in detail.get('beam_widths', [])
                    ),
                    default=0,
                )
                for key in (
                    'search_ik_calls',
                    'initial_fk_error',
                    'continuity_failure',
                ):
                    attempt[key] = detail.get(key)
                raise ValueError(
                    detail.get('reason', '逆解/正解或自身碰撞检查失败')
                )
            plan['reachability_check'] = detail
            return attempt, plan, points
        except (
            ValueError, TimeoutError, subprocess.TimeoutExpired
        ) as error:
            attempt['reason'] = str(error)[:1200]
            return attempt, None, None

    scores = outward_scores(held, arm, goal, frame, len(spins))
    # 第 0 个方向是夹爪转动最少的，仍然最先试：它可行时动作最短，与原来
    # 选出的方案相同。其余按“朝外”的程度排；得分相差不到 0.02 的视为
    # 一样，保持转动少的在前。
    order = [0] + sorted(
        spins[1:], key=lambda spin: (-round(scores[spin] / .02), spin)
    )

    with concurrent.futures.ThreadPoolExecutor(SEARCH_WORKERS) as pool:
        for shift in shifts:
            pending = [
                (shift, spin, route, turn_at)
                for route in ROUTE_OFFSETS
                # 绕行路线是从当前位置出发算的，只与原地转搭配。
                for turn_at in (turns if route == (0., 0.) else turns[:1])
                for spin in order
            ]
            # 卡在调姿段的（夹持方向, 转到位的位置）：转移路线帮不上忙，
            # 不再试它的其他路线。
            dead = set()
            while pending:
                cancelled()
                batch = []
                # 第一个候选单独检查：它通过的话（常见情形）不必为其余
                # 候选白费处理器，耗时与只检查一个相同。
                size = SEARCH_WORKERS if attempts else 1
                while pending and len(batch) < size:
                    key = (pending[0][1], pending[0][3])
                    if key in dead:
                        pending.pop(0)
                    elif any(key == (other[1], other[3]) for other in batch):
                        # 同一个方向的下一条路线要等这一批的结果：它若卡在
                        # 调姿段，后面的路线就不必试了。
                        break
                    else:
                        batch.append(pending.pop(0))
                if not batch:
                    break
                for attempt, plan, points in pool.map(evaluate, batch):
                    attempts.append(attempt)
                    if len(attempts) > 64:
                        del attempts[32]
                    if plan is not None:
                        cancelled()
                        plan['search_attempts'] = attempts
                        if selected_points is not None:
                            selected_points[:] = points
                        return plan
                    failure = attempt.get('failure') or {}
                    if failure.get('waypoint', '').startswith(
                        ('orient_', 'release_orientation')
                    ):
                        dead.add(
                            (attempt['spin_candidate'], attempt['turn_at'])
                        )
            if not adaptive:
                if len(dead) == len(spins) * len(turns):
                    raise OrientationUnreachable(
                        '全部 %d 个夹持方向都无法完成调姿：%s'
                        % (len(spins), attempts[-1].get('reason', ''))
                    )
                break
    raise ValueError(
        '没有通过检查的持物运动方案：'
        + (attempts[-1].get('reason', '无候选') if attempts else '无候选')
    )
