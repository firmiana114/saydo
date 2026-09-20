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


def rotation_error(a, b):
    return float(np.arccos(np.clip((np.trace(np.asarray(a).T @ np.asarray(b)) - 1) / 2, -1, 1)))


def pose_values(matrix):
    """Convert a rigid transform to metres and Rz @ Ry @ Rx Euler radians."""
    T = transform(matrix)
    R = T[:3, :3]
    pitch = math.atan2(-R[2, 0], math.hypot(R[0, 0], R[1, 0]))
    if math.hypot(R[0, 0], R[1, 0]) > 1e-8:
        roll, yaw = math.atan2(R[2, 1], R[2, 2]), math.atan2(R[1, 0], R[0, 0])
    else:
        roll, yaw = 0., math.atan2(-R[0, 1], R[1, 1])
    return np.r_[T[:3, 3], roll, pitch, yaw]


def object_in_base(base_tcp, flange_tcp, flange_camera, camera_object):
    return transform(base_tcp) @ np.linalg.inv(transform(flange_tcp)) @ transform(flange_camera) @ transform(camera_object)


def return_joint_target(config):
    if config.get('return_after_lift') is None:
        return None
    raw = config['return_after_lift'].get('joint_deg')
    if not isinstance(raw, list) or len(raw) != 7 or any(type(v) not in (int, float) for v in raw):
        raise ValueError('返回目标必须包含七个有限关节角度，单位度')
    values = np.asarray(raw, dtype=float)
    if not np.isfinite(values).all() or np.any(np.abs(values) > 360):
        raise ValueError('返回目标必须包含七个有限关节角度，单位度')
    return np.rint(values * 1000).astype(int).tolist()


def plan_grasp(raw_tcp, base_object, object_profile, config):
    current = controller_pose(raw_tcp)
    point = np.asarray(object_profile['grasp_point_object_m'], dtype=float)
    if point.shape != (3,) or not np.isfinite(point).all() or np.linalg.norm(point) > .2:
        raise ValueError('物体抓取点配置无效')
    if config['action'] != 'grasp_and_lift':
        raise ValueError('仅支持夹持后竖直抬升')
    policy = object_profile['orientation']
    if policy == 'follow_object':
        relative = np.eye(4)
        rotation = np.asarray(object_profile.get('object_R_tcp'), dtype=float)
        if rotation.shape != (3, 3):
            raise ValueError('当前物品尚未记录抓取朝向，请先记录夹法')
        relative[:3, :3] = rotation
        target_rotation = transform(base_object)[:3, :3] @ transform(relative)[:3, :3]
    elif policy == 'keep_current_tcp':
        target_rotation = current[:3, :3]
    else:
        raise ValueError('未知抓取朝向策略')
    approach = np.asarray(object_profile.get('approach_axis_tcp', [0, 0, 1]), dtype=float)
    if approach.shape != (3,) or not np.isfinite(approach).all() or not np.isclose(np.linalg.norm(approach), 1):
        raise ValueError('夹爪接近方向必须是单位向量')
    approach = target_rotation @ approach
    target = (transform(base_object) @ np.r_[point, 1])[:3]
    distance = float(np.linalg.norm(target - current[:3, 3]))
    if distance > config['max_target_distance_m']:
        raise ValueError('目标距离超过本次试抓范围')
    distance_pre = config['pregrasp_distance_m']
    if not np.isfinite(distance_pre) or not 0 < distance_pre <= .3:
        raise ValueError('预抓距离必须大于 0 且不超过 0.3 米')
    pre = target - approach * distance_pre
    lift_height = config['lift_height_m']
    if not isinstance(lift_height, (int, float)) or isinstance(lift_height, bool) or not np.isfinite(lift_height) or lift_height <= 0:
        raise ValueError('抬升高度必须为有限正数，单位米')
    points = [('pregrasp', pre), ('grasp', target), ('lift', target + [0, 0, lift_height])]
    waypoints = []
    for name, xyz in points:
        T = np.eye(4);T[:3, :3] = target_rotation;T[:3, 3] = xyz
        pose = np.rint(pose_values(T) * np.array([1e6]*3 + [1e3]*3)).astype(int).tolist()
        waypoints.append({'name': name, 'position_m': xyz.tolist(), 'controller_pose': pose})
    return {'base_T_object': transform(base_object).tolist(), 'target_position_m': target.tolist(),
            'current_position_m': current[:3, 3].tolist(),
            'orientation_policy': policy, 'target_rotation': target_rotation.tolist(),
            'approach_direction_base': approach.tolist(), 'grasp_point_object_m': point.tolist(), 'waypoints': waypoints, 'target_distance_m': distance,
            'lift_after_grasp': True, 'lift_height_m': lift_height}


def minimum_table_clearance(config):
    value = config.get('min_table_clearance_m', .01)
    if type(value) not in (int, float) or not np.isfinite(value) or value < 0:
        raise ValueError('min_table_clearance_m 必须为有限非负数，单位米')
    return value


def workspace_blockers(plan, config):
    try:
        minimum = minimum_table_clearance(config)
    except ValueError as error:
        return [str(error)]
    if not config.get('workspace_reviewed') or config.get('workspace_min_m') is None or config.get('workspace_max_m') is None or config.get('table_z_m') is None:
        return ['尚未配置并核对抓取工作区和桌面高度']
    low, high = np.asarray(config['workspace_min_m']), np.asarray(config['workspace_max_m'])
    if low.shape != (3,) or high.shape != (3,) or not np.isfinite(np.r_[low, high, config['table_z_m']]).all() or np.any(low >= high):
        return ['工作区配置无效']
    reasons = []
    names = {'current':'起始点', 'pregrasp':'预抓点', 'grasp':'夹持点', 'lift':'抬升点', 'return':'返回路径点'}
    for waypoint in [{'name':'current', 'position_m': plan['current_position_m']}] + plan['waypoints']:
        p = np.asarray(waypoint['position_m'], dtype=float)
        name = names.get(waypoint.get('name'), waypoint.get('name', '路径点'))
        if p.shape != (3,) or not np.isfinite(p).all():
            reasons.append(f'{name}：位置不是有效的三个有限数值')
            continue
        for i, axis in enumerate('XYZ'):
            if p[i] < low[i] or p[i] > high[i]:
                reasons.append(f'{name}：{axis}={p[i]*1000:.3f} 毫米，超出允许范围 [{low[i]*1000:.3f}, {high[i]*1000:.3f}] 毫米')
        clearance = p[2] - config['table_z_m']
        if clearance < minimum:
            reasons.append(f'{name}：工具参考点距配置桌面 {clearance*1000:.3f} 毫米，要求至少 {minimum*1000:.3f} 毫米，差 {(minimum-clearance)*1000:.3f} 毫米（Z={p[2]*1000:.3f}，桌面 Z={config["table_z_m"]*1000:.3f} 毫米）')
    return reasons



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
