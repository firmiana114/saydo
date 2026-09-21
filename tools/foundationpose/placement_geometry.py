"""Place a held mesh in a Base-frame rectangle, keeping its measured orientation at waypoints."""
import heapq
import subprocess
import time

import numpy as np

from grasp_geometry import controller_pose, transform, minimum_table_clearance
from reorientation_geometry import pose_waypoint


DEFAULTS = {
    'region_xy_m': None, 'release_gap_m': .003,
    'approach_height_m': .05, 'retreat_height_m': .05,
    'transfer_speed_percent': 10, 'descent_speed_percent': 5,
    'planning_timeout_s': 20, 'search_step_m': .025, 'linear_sample_step_m': .02,
    'position_tolerance_m': .002, 'orientation_tolerance_deg': 2,
}


def placement_settings(value, require_region=True):
    if not isinstance(value, dict):
        raise ValueError('放置配置必须为 JSON 对象')
    settings = {**DEFAULTS, **value}
    settings.pop('edge_margin_m', None)  # Legacy settings must not shrink the operator's rectangle.
    for key, low, high in [
        ('release_gap_m', 0, .06),
        ('approach_height_m', .02, .3), ('retreat_height_m', .02, .3),
        ('planning_timeout_s', 1, 60), ('search_step_m', .005, .1),
        ('linear_sample_step_m', .005, .05), ('position_tolerance_m', .0001, .002),
        ('orientation_tolerance_deg', .1, 2),
    ]:
        number = settings[key]
        if type(number) not in (int, float) or not np.isfinite(number) or not low <= number <= high:
            raise ValueError(f'放置参数 {key} 必须在 {low}–{high} 范围内')
    for key in ('transfer_speed_percent', 'descent_speed_percent'):
        if type(settings[key]) is not int or not 1 <= settings[key] <= 25:
            raise ValueError(f'{key} 必须为 1–25 的整数速度百分比')
    region = settings['region_xy_m']
    if region is None and not require_region:
        return settings
    if not isinstance(region, dict) or set(region) != {'x_min', 'x_max', 'y_min', 'y_max'}:
        raise ValueError('请先设置 Base 坐标系下的矩形桌面范围')
    if any(type(v) not in (int, float) or not np.isfinite(v) or abs(v) > 2 for v in region.values()):
        raise ValueError('桌面范围必须是 -2～2 米内的有限数值')
    if region['x_min'] >= region['x_max'] or region['y_min'] >= region['y_max']:
        raise ValueError('桌面 X、Y 最小值必须分别小于最大值')
    return settings


def object_vertices(tcp, held, vertices):
    points = np.asarray(vertices, dtype=float)
    if points.ndim != 2 or points.shape[1] != 3 or not len(points) or not np.isfinite(points).all():
        raise ValueError('物品模型顶点无效')
    obj = transform(tcp) @ np.linalg.inv(transform(held['object_T_tcp']))
    return points @ obj[:3, :3].T + obj[:3, 3]


def region_bounds(settings):
    region = settings['region_xy_m']
    return (np.array([region['x_min'], region['y_min']]),
            np.array([region['x_max'], region['y_max']]))


def check_footprint(world, settings):
    low, high = region_bounds(settings)
    if np.any(world[:, :2].min(axis=0) < low - 1e-8) or np.any(world[:, :2].max(axis=0) > high + 1e-8):
        raise ValueError('物品水平投影超出输入的桌面放置范围')


def check_tcp_workspace(tcp, config):
    low = np.asarray(config.get('workspace_min_m'), dtype=float)
    high = np.asarray(config.get('workspace_max_m'), dtype=float)
    if (not config.get('workspace_reviewed') or low.shape != (3,) or high.shape != (3,)
            or not np.isfinite([low, high]).all() or np.any(low >= high)):
        raise ValueError('尚未配置并核对机械臂工作区')
    if np.any(tcp[:3, 3] < low - 1e-8) or np.any(tcp[:3, 3] > high + 1e-8):
        raise ValueError('放置路径的工具中心超出机械臂工作区')
    if tcp[2, 3] < config['table_z_m'] + minimum_table_clearance(config) - 1e-8:
        raise ValueError('放置路径的工具中心离桌距离不足')


def linear_waypoint(name, matrix):
    result = pose_waypoint(name, matrix)
    result['method'] = 'linear'
    return result


def sample_linear_path(start_raw, targets, step):
    """Sample the requested straight TCP path; this is not a full joint-path proof."""
    start = controller_pose(start_raw); result = []
    for target in targets:
        end = controller_pose(target['controller_pose'])
        if not np.allclose(start[:3, :3], end[:3, :3], atol=1e-8):
            raise ValueError('放置直线路径必须保持当前朝向')
        count = max(1, int(np.ceil(np.linalg.norm(end[:3, 3] - start[:3, 3]) / step)))
        if len(result) + count > 480:
            raise ValueError('放置路径过长，超过直线采样检查数量上限')
        for i in range(1, count + 1):
            pose = start.copy();pose[:3, 3] += (end[:3, 3] - start[:3, 3]) * i / count
            waypoint = linear_waypoint(f"{target['name']}:{i}/{count}", pose)
            waypoint['controller_pose'][3:] = list(start_raw[3:])
            result.append(waypoint)
        start = end
    return result


def placement_motion_method(name):
    if name in ('raise', 'transfer'):
        return 'joint_pose'
    if name in ('lower', 'withdraw', 'retreat'):
        return 'linear'
    raise ValueError('未知的放置运动阶段')


def placement_validation_points(start_raw, targets, step):
    """Joint moves validate endpoints; only commanded linear moves are sampled."""
    previous = list(start_raw); points = []
    for target in targets:
        if placement_motion_method(target['name']) == 'joint_pose':
            points.append(target)
        else:
            points.extend(sample_linear_path(previous, [target], step))
        previous = list(target['controller_pose'])
    if len(points) > 480:
        raise ValueError('放置路径超过采样检查数量上限')
    return points


def candidate_xy(low, high, current, step):
    nearest = np.clip(current, low, high)
    yield nearest
    axes = [sorted(np.unique(np.r_[np.arange(low[i], high[i], step), high[i], nearest[i]]),
                   key=lambda n: abs(n - current[i])) for i in range(2)]
    # Enumerate nearby grid points lazily so a large region cannot spend the
    # planning budget sorting its entire Cartesian product before cancellation.
    pending = [(0., 0, 0)];seen = {(0, 0)}
    while pending:
        _, i, j = heapq.heappop(pending)
        pair = np.array([axes[0][i], axes[1][j]])
        if not np.allclose(pair, nearest, atol=1e-9, rtol=0):
            yield pair
        for a, b in ((i + 1, j), (i, j + 1)):
            if a < len(axes[0]) and b < len(axes[1]) and (a, b) not in seen:
                seen.add((a, b))
                distance = (axes[0][a] - current[0]) ** 2 + (axes[1][b] - current[1]) ** 2
                heapq.heappush(pending, (distance, a, b))


def plan_placement(held, vertices, arm, config, settings, reachable, cancelled, diagnostics=None):
    settings = placement_settings(settings)
    table = config.get('table_z_m')
    if type(table) not in (int, float) or not np.isfinite(table):
        raise ValueError('桌面高度必须为有限米制数值')
    withdraw_distance = config.get('pregrasp_distance_m')
    if type(withdraw_distance) not in (int, float) or not np.isfinite(withdraw_distance) or not 0 < withdraw_distance <= .3:
        raise ValueError('预抓取距离 pregrasp_distance_m 必须大于 0 且不超过 0.3 米，用于松爪后沿工具 -Z 后撤')
    current = controller_pose(arm['pose'])
    check_tcp_workspace(current, config)
    world = object_vertices(current, held, vertices)
    if world[:, 2].min() < table - 1e-6:
        raise ValueError('当前物品模型已低于桌面，不能自动放置')
    offsets = world - current[:3, 3]
    lower, upper = offsets.min(axis=0), offsets.max(axis=0)
    release_z = table + settings['release_gap_m'] - lower[2]
    # Round upwards to a controller micrometre, never below the requested gap.
    release_z = np.ceil(release_z * 1e6) / 1e6
    hover_z = max(current[2, 3], release_z + settings['approach_height_m'])
    low, high = region_bounds(settings)
    low = np.maximum(low - lower[:2], np.asarray(config['workspace_min_m'])[:2])
    high = np.minimum(high - upper[:2], np.asarray(config['workspace_max_m'])[:2])
    low = np.ceil(low * 1e6) / 1e6;high = np.floor(high * 1e6) / 1e6
    if np.any(low > high):
        raise ValueError('当前朝向的物品无法完整放入指定桌面范围')
    attempts = diagnostics if diagnostics is not None else []
    deadline = time.monotonic() + settings['planning_timeout_s']; attempted = 0
    for xy in candidate_xy(low, high, current[:2, 3], settings['search_step_m']):
        cancelled()
        if time.monotonic() >= deadline:
            raise TimeoutError('放置位置搜索超时，未发送运动')
        attempted += 1
        attempt = {'candidate_index': attempted, 'tcp_xy_m': xy.tolist()}; attempts.append(attempt)
        if len(attempts) > 128:
            del attempts[64]
        release_xyz = np.array([*xy, release_z])
        withdraw_xyz = release_xyz - current[:3, 2] * withdraw_distance
        retreat_xyz = withdraw_xyz + [0, 0, settings['retreat_height_m']]
        targets = []
        last = current
        for name, xyz in [
            ('raise', [*current[:2, 3], hover_z]), ('transfer', [*xy, hover_z]),
            ('lower', release_xyz), ('withdraw', withdraw_xyz), ('retreat', retreat_xyz),
        ]:
            matrix = current.copy();matrix[:3, 3] = xyz
            target = pose_waypoint(name, matrix)
            target['method'] = placement_motion_method(name)
            target['controller_pose'][3:] = list(arm['pose'][3:])
            actual = controller_pose(target['controller_pose'])
            target['position_m'] = actual[:3, 3].tolist()
            if name in ('raise', 'transfer') and np.linalg.norm(actual[:3, 3] - last[:3, 3]) < 1e-6:
                continue
            targets.append(target); last = actual
        try:
            for target in targets:
                check_tcp_workspace(controller_pose(target['controller_pose']), config)
            release = next(w for w in targets if w['name'] == 'lower')
            released_world = object_vertices(controller_pose(release['controller_pose']), held, vertices)
            check_footprint(released_world, settings)
            samples = placement_validation_points(arm['pose'], targets, settings['linear_sample_step_m'])
        except ValueError as error:
            attempt['reason'] = str(error)
            # Z and path limits may fail for every XY, but retain a bounded search.
            continue
        remaining = deadline - time.monotonic()
        if remaining <= 0:
            raise TimeoutError('放置位置搜索超时，未发送运动')
        try:
            valid, detail = reachable(samples, min(remaining, 3.))
        except subprocess.TimeoutExpired:
            attempt['reason'] = '候选运动端点/直线路径逆解检查超时'
            continue
        cancelled()
        if time.monotonic() >= deadline:
            raise TimeoutError('放置位置搜索超时，未发送运动')
        attempt.update(reachable=bool(valid), reason=None if valid else detail.get('reason', '路径不可达'))
        if not valid:
            continue
        obj = controller_pose(release['controller_pose']) @ np.linalg.inv(transform(held['object_T_tcp']))
        return {
            'waypoints': targets, 'validation_points': samples, 'current_arm': arm,
            'base_T_object_target': obj.tolist(), 'object_T_tcp': held['object_T_tcp'],
            'target_tcp_position_m': release['position_m'], 'table_z_m': table,
            'release_gap_m': settings['release_gap_m'], 'lowest_point_offset_z_m': float(lower[2]),
            'object_bottom_z_m': float(released_world[:, 2].min()),
            'footprint_min_xy_m': released_world[:, :2].min(axis=0).tolist(),
            'footprint_max_xy_m': released_world[:, :2].max(axis=0).tolist(),
            'region_xy_m': settings['region_xy_m'], 'region_constraint_scope': 'candidate_selection_only',
            'withdraw_distance_m': withdraw_distance,
            'withdraw_direction_base': (-current[:3, 2]).tolist(),
            'release_position': 1000, 'orientation_policy': 'keep_measured_object_orientation_at_waypoints',
            'motion_command': 'mixed',
            'motion_commands': {w['name']: 'movej_p' if w['method'] == 'joint_pose' else 'movel' for w in targets},
            'joint_path_checked': False, 'reachability_check': detail,
            'search_attempt_count': attempted, 'search_attempts': attempts,
            'validation': 'joint_endpoints_and_sampled_linear_tcp_ik_fk_with_live_feedback_not_full_paths',
            'contact_confirmed': False, 'stability_verified': False,
            'gripper_collision_checked': False,
        }
    reason = attempts[-1].get('reason', '路径不可达') if attempts else '区域内无候选位置'
    raise ValueError('桌面范围内未找到可执行的放置方案：' + reason)
