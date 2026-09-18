"""SI frame transforms and bounded first-trial grasp planning."""
import math
import numpy as np


def transform(value):
    T = np.asarray(value, dtype=float)
    if T.shape != (4, 4) or not np.isfinite(T).all() or not np.allclose(T[3], [0, 0, 0, 1]):
        raise ValueError('变换矩阵无效')
    if not np.allclose(T[:3, :3].T @ T[:3, :3], np.eye(3), atol=1e-4) or not np.isclose(np.linalg.det(T[:3, :3]), 1, atol=1e-4):
        raise ValueError('旋转矩阵不合法')
    return T


def controller_pose(raw):
    if len(raw) != 6:
        raise ValueError('控制器位姿长度错误')
    x, y, z = np.asarray(raw[:3], dtype=float) / 1e6
    rx, ry, rz = np.asarray(raw[3:], dtype=float) / 1e3
    cx, sx, cy, sy, cz, sz = math.cos(rx), math.sin(rx), math.cos(ry), math.sin(ry), math.cos(rz), math.sin(rz)
    T = np.eye(4)
    T[:3, :3] = [[cz*cy, cz*sy*sx-sz*cx, cz*sy*cx+sz*sx], [sz*cy, sz*sy*sx+cz*cx, sz*sy*cx-cz*sx], [-sy, cy*sx, cy*cx]]
    T[:3, 3] = [x, y, z]
    return transform(T)


def object_in_base(base_tcp, flange_tcp, flange_camera, camera_object):
    return transform(base_tcp) @ np.linalg.inv(transform(flange_tcp)) @ transform(flange_camera) @ transform(camera_object)


def plan_grasp(raw_tcp, base_object, object_profile, config):
    current = controller_pose(raw_tcp)
    point = np.asarray(object_profile['grasp_point_object_m'], dtype=float)
    if point.shape != (3,) or not np.isfinite(point).all() or np.linalg.norm(point) > .2:
        raise ValueError('物体抓取点配置无效')
    if object_profile['orientation'] != 'keep_current_tcp' or config['action'] != 'grasp_and_lift':
        raise ValueError('仅支持保持当前朝向、夹持后竖直抬升')
    target = (transform(base_object) @ np.r_[point, 1])[:3]
    distance = float(np.linalg.norm(target - current[:3, 3]))
    if distance > config['max_target_distance_m']:
        raise ValueError('目标距离超过本次试抓范围')
    pre = target - current[:3, 2] * config['pregrasp_distance_m']
    z = max(current[2, 3], pre[2], target[2]) + config['clearance_height_m']
    lift_height = config['lift_height_m']
    if not isinstance(lift_height, (int, float)) or isinstance(lift_height, bool) or not np.isfinite(lift_height) or lift_height <= 0:
        raise ValueError('抬升高度必须为有限正数，单位米')
    points = [('above_pregrasp', np.array([pre[0], pre[1], z])),
              ('pregrasp', pre), ('grasp', target), ('lift', target + [0, 0, lift_height])]
    waypoints = []
    for name, xyz in points:
        pose = [int(round(v * 1e6)) for v in xyz] + [int(v) for v in raw_tcp[3:]]
        waypoints.append({'name': name, 'position_m': xyz.tolist(), 'controller_pose': pose})
    return {'base_T_object': transform(base_object).tolist(), 'target_position_m': target.tolist(),
            'current_position_m': current[:3, 3].tolist(),
            'orientation_policy': 'keep_current_tcp', 'waypoints': waypoints, 'target_distance_m': distance,
            'lift_after_grasp': True, 'lift_height_m': lift_height}


def workspace_blockers(plan, config):
    if not config.get('workspace_reviewed') or config.get('workspace_min_m') is None or config.get('workspace_max_m') is None or config.get('table_z_m') is None:
        return ['尚未配置并核对抓取工作区和桌面高度']
    low, high = np.asarray(config['workspace_min_m']), np.asarray(config['workspace_max_m'])
    if low.shape != (3,) or high.shape != (3,) or not np.isfinite(np.r_[low, high, config['table_z_m']]).all() or np.any(low >= high):
        return ['工作区配置无效']
    for waypoint in [{'position_m': plan['current_position_m']}] + plan['waypoints']:
        p = np.asarray(waypoint['position_m'])
        if np.any(p < low) or np.any(p > high) or p[2] < config['table_z_m'] + .01:
            return ['抓取路径超出工作区或 TCP 离桌面不足 10 mm']
    return []


def contact_result(state, gripper):
    if state.get('sys_err', state.get('sys_error', -1)) or state.get('sys_state') != 0 or state.get('dof_err') != [0]:
        return 'fault'
    dof = state.get('dof_state', [-1])[0]
    # Feedback and the setting have no established common physical unit.
    if dof in (5, 6) or dof not in (0, 1, 2, 3, 4):
        return 'fault'
    pos = state.get('pos', [0])[0]
    if dof == 3 and state.get('speed') == [0] and pos > gripper['closed_position'] + gripper['contact_position_margin']:
        return 'force_contact' if state.get('force', [0])[0] > 0 else 'force_stop'
    if dof == 2 and pos <= gripper['closed_position'] + gripper['contact_position_margin']:
        return 'empty_close'
    return 'pending'
