"""Object orientation with bounded position search and sampled path checks."""
import hashlib
from functools import lru_cache
from pathlib import Path
import subprocess
import itertools
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
    value = {'adaptive_max_shift_m': .10, 'search_step_m': .025,
             'translation_step_m': .01, 'max_joint_step_deg': 20., **value}
    for key, low, high in [('planning_timeout_s', 1, 60), ('rotation_step_deg', 1, 10),
                          ('position_tolerance_m', .0001, .005), ('orientation_tolerance_deg', .1, 3),
                          ('adaptive_max_shift_m', .005, .20), ('search_step_m', .005, .05),
                          ('translation_step_m', .002, .02), ('max_joint_step_deg', 1, 30)]:
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
    detail = {'returncode':result.returncode, 'stdout':result.stdout[-2048:], 'stderr':result.stderr[-512:]}
    if valid:
        solutions = np.asarray([[float(x) for x in row[3:]] for row in rows])
        differences = np.abs(np.diff(np.vstack([joints, solutions]), axis=0))
        detail.update(max_joint_step_deg=float(differences.max()), joint_travel_deg=float(differences.sum()))
    return valid, detail


def checked_position_request(mode, radius, settings):
    if mode not in ('fixed', 'adaptive'):
        raise ValueError('位置模式必须是 fixed 或 adaptive')
    if radius is None:
        radius = settings['adaptive_max_shift_m']
    if type(radius) not in (int,float) or not np.isfinite(radius) or not 0 < radius <= settings['adaptive_max_shift_m']:
        raise ValueError(f"最大位移必须大于 0 且不超过 {settings['adaptive_max_shift_m']*100:g} 厘米")
    return mode, float(radius)


def position_offsets(radius, step):
    yield np.zeros(3)
    # Finite radial directions, nearest shell first. Never search below the initial TCP.
    directions = [np.asarray(v,dtype=float) for v in itertools.product([-1,0,1],[-1,0,1],[0,1]) if any(v)]
    directions.sort(key=lambda v:(-v[2],np.count_nonzero(v),tuple(v)))
    radii = list(np.arange(step, radius+1e-9, step))
    if not radii or radii[-1] < radius-1e-9:
        radii.append(radius)
    for distance in radii:
        for direction in directions:
            yield direction/np.linalg.norm(direction)*distance


def segment_distance(point, start, end):
    point,start,end = [np.asarray(x,dtype=float) for x in (point,start,end)]
    delta=end-start;length2=float(delta@delta)
    t=float(np.clip((point-start)@delta/length2,0,1)) if length2>1e-16 else 0.
    return float(np.linalg.norm(point-(start+t*delta)))


def adaptive_candidate(held, vertices, rotation, arm, config, settings, offset):
    current=controller_pose(arm['pose']);shifted=current.copy();shifted[:3,3]+=offset
    shifted_raw=pose_waypoint('translate',shifted)['controller_pose']
    plan=reorientation_waypoints(held,vertices,rotation,shifted_raw,config,settings)
    samples=[];points=np.asarray(vertices);attachment=transform(held['object_T_tcp'])
    low,high=np.asarray(config['workspace_min_m']),np.asarray(config['workspace_max_m'])
    offset=np.asarray(shifted_raw[:3])/1e6-current[:3,3]
    distance=float(np.linalg.norm(offset))
    if distance>1e-9:
        count=max(1,int(np.ceil(distance/settings['translation_step_m'])))
        for i,fraction in enumerate(np.linspace(0,1,count+1)):
            pose=current.copy();pose[:3,3]+=offset*fraction
            xyz=pose[:3,3];obj=pose@np.linalg.inv(attachment)
            world=points@obj[:3,:3].T+obj[:3,3]
            if np.any(xyz<low) or np.any(xyz>high) or xyz[2]<config['table_z_m']+minimum_table_clearance(config):
                raise ValueError('移位路径的工具中心超出工作区或离桌距离不足')
            if np.any(world<low-1e-8) or np.any(world>high+1e-8) or world[:,2].min()<config['table_z_m']-1e-6:
                raise ValueError('移位路径的物品超出工作区或碰到桌面')
            samples.append(pose_waypoint(f'translation_sample_{i}',pose))
        translation=pose_waypoint('translate',controller_pose(shifted_raw))
        plan['waypoints'].insert(0,translation)
    plan['path_samples']=samples+plan['rotation_samples'][1:] if samples else plan['rotation_samples']
    plan.update(start_tcp_position_m=current[:3,3].tolist(),target_tcp_position_m=plan.pop('fixed_tcp_position_m'),
                offset_m=offset.tolist(),offset_distance_m=distance,tcp_position_fixed=distance<1e-9)
    plan.pop('fixed_tcp_position_raw',None)
    return plan


def plan_reorientation(held, vertices, rotation, arm, config, settings, reachable, cancelled,
                       position_mode='fixed', max_shift_m=None, diagnostics=None):
    settings = reorientation_settings(settings)
    position_mode,radius=checked_position_request(position_mode,max_shift_m,settings)
    checked_rotation(rotation)
    if not config.get('workspace_reviewed'):
        raise ValueError('尚未核对姿态调整工作区')
    low, high = np.asarray(config.get('workspace_min_m'), dtype=float), np.asarray(config.get('workspace_max_m'), dtype=float)
    if low.shape != (3,) or high.shape != (3,) or not np.isfinite(np.r_[low,high,config.get('table_z_m')]).all() or np.any(low >= high):
        raise ValueError('姿态调整工作区配置无效')
    attempts=diagnostics if diagnostics is not None else []
    deadline=time.monotonic()+settings['planning_timeout_s']
    offsets=position_offsets(radius,settings['search_step_m']) if position_mode=='adaptive' else [np.zeros(3)]
    for offset in offsets:
        cancelled()
        if time.monotonic()>=deadline:
            raise TimeoutError('姿态位置搜索超时，未发送运动')
        attempt={'offset_m':offset.tolist()};attempts.append(attempt)
        try:
            plan=adaptive_candidate(held,vertices,rotation,arm,config,settings,offset)
        except ValueError as error:
            attempt['reason']=str(error)
            if position_mode=='fixed':raise
            continue
        remaining=deadline-time.monotonic()
        if remaining<=0:raise TimeoutError('姿态位置搜索超时，未发送运动')
        try:
            valid,detail=reachable(plan['path_samples'],min(remaining,3.))
        except subprocess.TimeoutExpired:
            attempt['reason']='单个候选逆解检查超时'
            cancelled()
            if position_mode=='fixed':raise TimeoutError('原地姿态检查超时，未发送运动')
            continue
        cancelled()
        if time.monotonic()>deadline:raise TimeoutError('姿态位置搜索超时，未发送运动')
        jump=detail.get('max_joint_step_deg',0.)
        attempt.update(reachable=bool(valid),max_joint_step_deg=jump,
                       reason=None if valid else str(detail)[-700:])
        if valid and jump>settings['max_joint_step_deg']:
            attempt['reason']=f"相邻采样关节变化 {jump:.2f}° 超过 {settings['max_joint_step_deg']:g}°"
            continue
        if valid:
            plan.update(reachability_check=detail,current_arm=arm,position_mode=position_mode,
                        max_shift_m=radius if position_mode=='adaptive' else 0.,search_attempts=attempts,
                        validation='sampled_translation_and_rotation_not_full_environment_collision')
            return plan
    reason=attempts[-1].get('reason') if attempts else '无候选'
    raise ValueError(f"{'限定范围内姿态调整' if position_mode=='adaptive' else '原地姿态调整'}不可达或采样检查失败，未发送运动：{reason}")
