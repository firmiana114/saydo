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


def sample_motion(start_raw, end_raw, distance_step=.01, angle_step=math.radians(3)):
    """Linear position / shortest rotation samples, including rotation-only moves."""
    start, end = controller_pose(start_raw), controller_pose(end_raw)
    relative = start[:3, :3].T @ end[:3, :3]
    angle = rotation_error(start[:3, :3], end[:3, :3])
    steps = max(1, int(math.ceil(np.linalg.norm(end[:3, 3] - start[:3, 3]) / distance_step)),
                int(math.ceil(angle / angle_step)))
    if angle < 1e-8:
        axis = np.array([1., 0, 0])
    elif abs(math.sin(angle)) > 1e-5:
        axis = np.array([relative[2,1]-relative[1,2], relative[0,2]-relative[2,0],
                         relative[1,0]-relative[0,1]]) / (2 * math.sin(angle))
    else:
        _, vectors = np.linalg.eigh((relative + relative.T + 2 * np.eye(3)) / 4)
        axis = vectors[:, -1]
    x, y, z = axis
    cross = np.array([[0., -z, y], [z, 0., -x], [-y, x, 0.]])
    samples = []
    for n in range(1, steps + 1):
        t = n / steps
        T = np.eye(4)
        T[:3, 3] = start[:3, 3] + t * (end[:3, 3] - start[:3, 3])
        T[:3, :3] = start[:3, :3] @ (np.eye(3) + math.sin(t*angle)*cross + (1-math.cos(t*angle))*(cross @ cross))
        if n == steps:
            T = end
        samples.append(pose_values(T).tolist())
    return samples


def object_in_base(base_tcp, flange_tcp, flange_camera, camera_object):
    return transform(base_tcp) @ np.linalg.inv(transform(flange_tcp)) @ transform(flange_camera) @ transform(camera_object)


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
    return_target = None
    if config.get('return_after_lift') is not None:
        values = np.asarray(config['return_after_lift']['joint_deg'], dtype=float)
        if values.shape != (7,) or not np.isfinite(values).all() or np.any(np.abs(values) > 360):
            raise ValueError('返回目标必须包含七个有限关节角度，单位度')
        return_target = np.rint(values * 1000).astype(int).tolist()
    return {'base_T_object': transform(base_object).tolist(), 'target_position_m': target.tolist(),
            'current_position_m': current[:3, 3].tolist(),
            'orientation_policy': policy, 'target_rotation': target_rotation.tolist(),
            'approach_direction_base': approach.tolist(), 'grasp_point_object_m': point.tolist(), 'waypoints': waypoints, 'target_distance_m': distance,
            'lift_after_grasp': True, 'lift_height_m': lift_height, 'return_joint_raw': return_target}


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
