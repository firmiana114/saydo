"""Object-frame reorientation at a fixed table location and endpoint reachability checks."""
import hashlib
from functools import lru_cache
from pathlib import Path
import subprocess

import numpy as np

from grasp_geometry import transform, controller_pose, pose_values, minimum_table_clearance
from object_catalog import select_object


@lru_cache(maxsize=8)
def _mesh_geometry(path, modified_ns, size):
    import trimesh
    mesh = trimesh.load(path, force='mesh', process=False)
    vertices, faces = np.asarray(mesh.vertices, dtype=float), np.asarray(mesh.faces, dtype=int)
    if (vertices.ndim != 2 or vertices.shape[1] != 3 or len(vertices) == 0
            or not np.isfinite(vertices).all() or not .01 < np.ptp(vertices, axis=0).max() < 1
            or faces.ndim != 2 or faces.shape[1] != 3 or len(faces) == 0):
        raise ValueError('物品模型必须是有效的米制三角网格')
    return {'vertices': vertices.tolist(), 'faces': faces.tolist(),
            'model_sha256': hashlib.sha256(Path(path).read_bytes()).hexdigest()}


def model_geometry(models_root, object_id):
    item = select_object(models_root, object_id)
    path = Path(item['mesh']);stat = path.stat()
    return {**_mesh_geometry(str(path), stat.st_mtime_ns, stat.st_size),
            'object_id': item['id'], 'object_name': item['name'], 'units': 'metres'}


def checked_rotation(value):
    rotation = np.asarray(value, dtype=float)
    if rotation.shape != (3, 3):
        raise ValueError('目标朝向必须为 3×3 旋转矩阵')
    matrix = np.eye(4);matrix[:3, :3] = rotation
    return transform(matrix)[:3, :3]


def reorientation_settings(value):
    for key, low, high in [('planning_timeout_s', 1, 60), ('rotation_step_deg', 1, 10),
                          ('position_tolerance_m', .0001, .005), ('orientation_tolerance_deg', .1, 3)]:
        n = value.get(key)
        if type(n) not in (int, float) or not np.isfinite(n) or not low <= n <= high:
            raise ValueError(f'姿态调整配置 {key} 无效')
    if type(value.get('arm_speed_percent')) is not int or not 1 <= value['arm_speed_percent'] <= 100:
        raise ValueError('姿态调整速度必须为 1–100 的整数百分比')
    return value


def pose_waypoint(name, matrix):
    return {'name': name, 'method': 'linear', 'position_m': matrix[:3, 3].tolist(),
            'controller_pose': np.rint(pose_values(matrix)*np.array([1e6]*3+[1e3]*3)).astype(int).tolist()}


def reorientation_waypoints(held, vertices, rotation, current_raw, config, settings):
    from scipy.spatial.transform import Rotation, Slerp
    attachment = transform(held['object_T_tcp'])
    current = controller_pose(current_raw)
    target_tcp = current.copy()
    target_tcp[:3, :3] = checked_rotation(rotation) @ attachment[:3, :3]
    # A held object rotates around the TCP; its own origin generally translates.
    target_object = target_tcp @ np.linalg.inv(attachment)
    angle = (Rotation.from_matrix(current[:3, :3]).inv()*Rotation.from_matrix(target_tcp[:3, :3])).magnitude()
    count = max(1, int(np.ceil(np.rad2deg(angle)/settings['rotation_step_deg'])))
    rotations = Slerp([0, 1], Rotation.from_matrix(np.array([current[:3,:3], target_tcp[:3,:3]])))(np.linspace(0,1,count+1)).as_matrix()
    points = np.asarray(vertices, dtype=float)
    if points.ndim != 2 or points.shape[1] != 3 or not len(points) or not np.isfinite(points).all():
        raise ValueError('物品模型顶点无效')
    low, high = np.asarray(config['workspace_min_m']), np.asarray(config['workspace_max_m'])
    minimum = minimum_table_clearance(config)
    tcp = current[:3, 3]
    if np.any(tcp < low) or np.any(tcp > high) or tcp[2] < config['table_z_m']+minimum:
        raise ValueError('当前工具中心超出工作区或离桌距离不足，不能原地调整姿态')
    samples = []
    for i, rot in enumerate(rotations):
        pose = current.copy();pose[:3,:3] = rot
        object_pose = pose @ np.linalg.inv(attachment)
        world = points @ object_pose[:3,:3].T + object_pose[:3,3]
        if np.any(world < low-1e-8) or np.any(world > high+1e-8) or world[:,2].min() < config['table_z_m']-1e-6:
            raise ValueError(f'姿态路径采样 {i}/{count} 的物品超出工作区或碰到桌面；未移动末端避让')
        waypoint = pose_waypoint(f'rotation_sample_{i}', pose)
        waypoint['controller_pose'][:3] = list(current_raw[:3])
        samples.append(waypoint)
    target = pose_waypoint('reorient', target_tcp)
    target['controller_pose'][:3] = list(current_raw[:3])
    return {'base_T_object_target': target_object.tolist(), 'object_T_tcp': attachment.tolist(),
            'fixed_tcp_position_m': tcp.tolist(), 'fixed_tcp_position_raw': list(current_raw[:3]),
            'current_arm': {'pose': list(current_raw)}, 'rotation_angle_deg': float(np.rad2deg(angle)),
            'waypoints': [target], 'rotation_samples': samples,
            'tcp_position_fixed': True, 'gripper_action': 'keep_holding'}


def endpoint_reachability(executable, tool, arm, waypoints, timeout):
    tool_values = pose_values(tool['flange_T_grip_tcp'])
    joints = np.asarray(arm['joint'], dtype=float) / 1000
    values = [*tool_values, *joints, len(waypoints)]
    for waypoint in waypoints:
        values.extend(pose_values(controller_pose(waypoint['controller_pose'])))
    result = subprocess.run([str(executable)], input=' '.join(str(x) for x in values)+'\n',
                            text=True, capture_output=True, timeout=timeout)
    rows = [line.split() for line in result.stdout.splitlines() if line.startswith('IK ')]
    valid = result.returncode == 0 and len(rows) == len(waypoints) and all(
        len(row) == 10 and row[1:3] == ['0','0'] and np.isfinite([float(x) for x in row[3:]]).all() for row in rows)
    return valid, {'returncode':result.returncode, 'stdout':result.stdout[-2048:], 'stderr':result.stderr[-512:]}


def plan_reorientation(held, vertices, rotation, arm, config, settings, reachable, cancelled):
    settings = reorientation_settings(settings)
    if not config.get('workspace_reviewed'):
        raise ValueError('尚未核对姿态调整工作区')
    low, high = np.asarray(config.get('workspace_min_m'), dtype=float), np.asarray(config.get('workspace_max_m'), dtype=float)
    if low.shape != (3,) or high.shape != (3,) or not np.isfinite(np.r_[low,high,config.get('table_z_m')]).all() or np.any(low >= high):
        raise ValueError('姿态调整工作区配置无效')
    cancelled()
    plan = reorientation_waypoints(held, vertices, rotation, arm['pose'], config, settings)
    valid, detail = reachable(plan['rotation_samples'], settings['planning_timeout_s'])
    cancelled()
    if not valid:
        raise ValueError(f'原地姿态调整不可达或采样检查失败，未发送运动：{detail}')
    plan.update(reachability_check=detail, current_arm=arm,
                validation='sampled_rotations_not_full_path_or_environment_collision')
    return plan
