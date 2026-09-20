"""Object-frame placement geometry and bounded endpoint reachability search."""
import hashlib
from functools import lru_cache
from pathlib import Path
import subprocess
import time

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
        raise ValueError('放置模型必须是有效的米制三角网格')
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
        raise ValueError('放置朝向必须为 3×3 旋转矩阵')
    matrix = np.eye(4);matrix[:3, :3] = rotation
    return transform(matrix)[:3, :3]


def placement_settings(value):
    for key, low, high in [('approach_height_m', .01, .30), ('release_clearance_m', 0, .01),
                          ('retreat_distance_m', .01, .30), ('search_radius_m', 0, .20),
                          ('search_step_m', .005, .05), ('release_timeout_s', 1, 30),
                          ('planning_timeout_s', 1, 60)]:
        n = value.get(key)
        if type(n) not in (int, float) or not np.isfinite(n) or not low <= n <= high:
            raise ValueError(f'放置配置 {key} 无效')
    for key, low, high in [('arm_speed_percent',1,100),('release_position',0,1000)]:
        if type(value.get(key)) is not int or not low <= value[key] <= high:
            raise ValueError(f'放置配置 {key} 无效')
    return value


def nearby_offsets(grasp_position, radius, step):
    direction = np.asarray(grasp_position, dtype=float)[:2].copy()
    offsets = [[0., 0.]]
    length = np.linalg.norm(direction)
    if length < 1e-6:
        return offsets  # The base-to-grasp line has no horizontal direction.
    direction /= length
    for i in range(1, int(np.floor(radius / step))+1):
        offsets.extend([(-direction*i*step).tolist(), (direction*i*step).tolist()])
    return offsets


def pose_waypoint(name, matrix, method):
    return {
        'name': name, 'method': method, 'position_m': matrix[:3, 3].tolist(),
        'controller_pose': np.rint(pose_values(matrix)*np.array([1e6]*3+[1e3]*3)).astype(int).tolist()}


def placement_candidate(held, vertices, rotation, current_raw, config, settings, offset):
    original = transform(held['base_T_object_at_grasp'])
    attachment = transform(held['object_T_tcp'])
    current = controller_pose(current_raw)
    points = np.asarray(vertices, dtype=float)
    target = np.eye(4);target[:3, :3] = rotation
    target[:2, 3] = original[:2, 3] + offset
    target[2, 3] = config['table_z_m'] + settings['release_clearance_m'] - np.min((points @ rotation.T)[:, 2])
    place_tcp = target @ attachment
    # A sphere around the held TCP bounds object orientation changes at the travel height.
    tcp_points = (points-attachment[:3,3]) @ attachment[:3,:3]
    radius = float(np.max(np.linalg.norm(tcp_points, axis=1)))
    travel_z = max(current[2,3], place_tcp[2,3]+settings['approach_height_m'],
                   config['table_z_m']+radius+settings['approach_height_m'])
    raised = current.copy();raised[2,3] = travel_z
    above = place_tcp.copy();above[2,3] = travel_z
    retreat = place_tcp.copy();retreat[:3,3] -= place_tcp[:3,2]*settings['retreat_distance_m']
    matrices = [('raise',raised,'linear'),('above',above,'joint_pose'),
                ('place',place_tcp,'linear'),('retreat',retreat,'linear')]
    low, high = np.asarray(config['workspace_min_m']), np.asarray(config['workspace_max_m'])
    minimum = minimum_table_clearance(config)
    for name, matrix, _ in [('current',current,'')]+matrices:
        p = matrix[:3,3]
        if np.any(p<low) or np.any(p>high) or p[2] < config['table_z_m']+minimum:
            raise ValueError(f'放置{name}点超出工具工作区或离桌距离不足')
        if name != 'retreat':
            object_pose = matrix @ np.linalg.inv(attachment)
            world = points @ object_pose[:3,:3].T + object_pose[:3,3]
            if np.any(world<low-1e-8) or np.any(world>high+1e-8) or world[:,2].min()<config['table_z_m']-1e-6:
                raise ValueError(f'放置{name}点的物品越界或低于桌面')
    if np.linalg.norm(place_tcp[:3,3]-current[:3,3]) > config['max_target_distance_m']:
        raise ValueError('放置目标距离超出配置范围')
    return {'base_T_object_target': target.tolist(), 'object_T_tcp': attachment.tolist(),
            'original_xy_m': original[:2,3].tolist(), 'selected_xy_m': target[:2,3].tolist(),
            'offset_m': list(offset), 'offset_distance_m': float(np.linalg.norm(offset)),
            'waypoints': [pose_waypoint(*row) for row in matrices],
            'release_clearance_m': settings['release_clearance_m']}


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


def choose_placement(held, vertices, rotation, arm, config, settings, reachable, cancelled):
    settings = placement_settings(settings)
    rotation = checked_rotation(rotation)
    if not config.get('workspace_reviewed'):
        raise ValueError('尚未核对放置工作区')
    low, high = np.asarray(config.get('workspace_min_m'),dtype=float), np.asarray(config.get('workspace_max_m'),dtype=float)
    if low.shape!=(3,) or high.shape!=(3,) or not np.isfinite(np.r_[low,high,config.get('table_z_m')]).all() or np.any(low>=high):
        raise ValueError('放置工作区配置无效')
    started = time.monotonic();attempts=[]
    for offset in nearby_offsets(held['grasp_position_base_m'],settings['search_radius_m'],settings['search_step_m']):
        cancelled()
        remaining = settings['planning_timeout_s']-(time.monotonic()-started)
        if remaining<=0:
            raise TimeoutError('放置落点搜索超时，未发送运动')
        try:
            candidate = placement_candidate(held,vertices,rotation,arm['pose'],config,settings,offset)
        except ValueError as error:
            attempts.append({'offset_m':offset,'reason':str(error)});continue
        valid, detail = reachable(candidate['waypoints'], min(remaining, 3.))
        attempts.append({'offset_m':offset,'reachable':valid,'detail':detail})
        if valid:
            candidate.update(search_attempts=attempts, search_step_m=settings['search_step_m'],
                             search_radius_m=settings['search_radius_m'],
                             validation='waypoints_only_not_full_path_or_environment_collision')
            return candidate
    raise ValueError(f"原位置及周围 {settings['search_radius_m']*100:g} 厘米内未找到可达放置点；最后原因：{attempts[-1] if attempts else '无候选'}")
