"""Object orientation with position search and endpoint checks for controller-planned joint motion and sampled path checks."""

import hashlib
import functools
import pathlib
import subprocess
import itertools
import time

import numpy as np

from saydo.geometry import grasp_geometry
from saydo.objects import object_catalog
from saydo.robot import motion_validation
from saydo.geometry import pose_symmetry
from saydo.geometry import rotation_steps


@functools.lru_cache(maxsize=8)
def _mesh_geometry(path, modified_ns, size):
    import trimesh
    mesh = trimesh.load(path, force='mesh', process=False)
    vertices, faces = (
        np.asarray(mesh.vertices, dtype=float),
        np.asarray(mesh.faces, dtype=int),
    )
    if (
        vertices.ndim != 2
        or vertices.shape[1] != 3
        or len(vertices) == 0
        or not np.isfinite(vertices).all()
        or not .01 < np.ptp(vertices, axis=0).max() < 1
        or faces.ndim != 2
        or faces.shape[1] != 3
        or len(faces) == 0
    ):
        raise ValueError('物品模型必须是有效的米制三角网格')
    return {
        'vertices': vertices.tolist(),
        'faces': faces.tolist(),
        'model_sha256': hashlib.sha256(
            pathlib.Path(path).read_bytes()
        ).hexdigest(),
    }


def model_geometry(models_root, object_id):
    item = object_catalog.select_object(models_root, object_id)
    path = pathlib.Path(item['mesh'])
    stat = path.stat()
    return {
        **_mesh_geometry(str(path), stat.st_mtime_ns, stat.st_size),
        'object_id': item['id'],
        'object_name': item['name'],
        'units': 'metres',
        'pose_mode': item.get('pose_mode', '6d'),
        'pose_mode_revision': item.get('pose_mode_revision'),
    }


def checked_rotation(value):
    rotation = np.asarray(value, dtype=float)
    if rotation.shape != (3, 3):
        raise ValueError('目标朝向必须为 3×3 旋转矩阵')
    matrix = np.eye(4)
    matrix[:3, :3] = rotation
    return grasp_geometry.transform(matrix)[:3, :3]


def rotation_from_rpy_deg(value):
    """Absolute base-frame object orientation, Rz(z) @ Ry(y) @ Rx(x)."""
    if (
        not isinstance(value, list)
        or len(value) != 3
        or any(
            (
                type(v) not in (int, float)
                or not np.isfinite(v)
                or not -180 <= v <= 180
            )
            for v in value
        )
    ):
        raise ValueError(
            'target_rpy_deg 必须为三个有限角度，每项在 -180 到 180 度之间'
        )
    x, y, z = np.deg2rad(value)
    cx, cy, cz = np.cos([x, y, z])
    sx, sy, sz = np.sin([x, y, z])
    return checked_rotation(
        [
            [cz * cy, cz * sy * sx - sz * cx, cz * sy * cx + sz * sx],
            [sz * cy, sz * sy * sx + cz * cx, sz * sy * cx - cz * sx],
            [-sy, cy * sx, cy * cx],
        ]
    )


def reorientation_settings(value):
    value = {'search_step_m': .025, **value}
    for key in (
        'adaptive_max_shift_m',
        'translation_step_m',
        'rotation_step_deg',
        'max_joint_step_deg',
    ):
        # Cartesian interpolation settings do not describe a movej_p trajectory.
        value.pop(key, None)
    for key, low, high in [
        ('planning_timeout_s', 1, 60),
        ('position_tolerance_m', .0001, .005),
        ('orientation_tolerance_deg', .1, 3),
        ('search_step_m', .005, .05),
    ]:
        n = value.get(key)
        if (
            type(n) not in (int, float)
            or not np.isfinite(n)
            or not low <= n <= high
        ):
            raise ValueError(f'姿态调整配置 {key} 无效')
    if (
        type(value.get('arm_speed_percent')) is not int
        or not 1 <= value['arm_speed_percent'] <= 100
    ):
        raise ValueError('姿态调整速度必须为 1–100 的整数百分比')
    return value


def pose_waypoint(name, matrix):
    return {
        'name': name,
        'method': 'joint_pose',
        'position_m': matrix[:3, 3].tolist(),
        'controller_pose': np.rint(
            grasp_geometry.pose_values(matrix) * np.array([1e6] * 3 + [1e3] * 3)
        ).astype(int).tolist(),
    }


def validate_object_pose(tcp, held, vertices, config, adaptive):
    attachment = grasp_geometry.transform(held['object_T_tcp'])
    points = np.asarray(vertices, dtype=float)
    if (
        points.ndim != 2
        or points.shape[1] != 3
        or not len(points)
        or not np.isfinite(points).all()
    ):
        raise ValueError('物品模型顶点无效')
    obj = tcp @ np.linalg.inv(attachment)
    lower, upper = pose_symmetry.object_bounds(
        obj, points, held.get('pose_mode', '6d')
    )
    if not adaptive:
        low, high = (
            np.asarray(config['workspace_min_m']),
            np.asarray(config['workspace_max_m']),
        )
        xyz = tcp[:3, 3]
        if (
            np.any(xyz < low)
            or np.any(xyz > high)
            or (
                xyz[2]
                < (
                    config['table_z_m']
                    + grasp_geometry.minimum_table_clearance(config)
                )
            )
        ):
            raise ValueError('固定模式工具端点超出工作区或离桌距离不足')
        if np.any(lower < low - 1e-8) or np.any(upper > high + 1e-8):
            raise ValueError('固定模式物品端点超出工作区')
    if lower[2] < config['table_z_m'] - 1e-6:
        raise ValueError('物品模型在端点碰到桌面')


def reorientation_waypoints(
    held, vertices, rotation, current_raw, config, settings, adaptive=False
):
    attachment = grasp_geometry.transform(held['object_T_tcp'])
    current = grasp_geometry.controller_pose(current_raw)
    target_tcp = current.copy()
    target_tcp[:3, :3] = checked_rotation(rotation) @ attachment[:3, :3]
    if adaptive and target_tcp[2, 3] <= config['table_z_m'] + .10:
        raise ValueError('自适应姿态的最终末端高度必须高于桌面 10 厘米')
    validate_object_pose(current, held, vertices, config, adaptive)
    validate_object_pose(target_tcp, held, vertices, config, adaptive)
    step = rotation_steps.checked_step(
        settings.get('max_rotation_step_deg', 15.)
    )
    waypoints = []
    parts = rotation_steps.fractions(current[:3, :3], target_tcp[:3, :3], step)
    if len(parts) > 64:
        raise ValueError('调姿方案超过64段，请增大步长或拆分明确的目标')
    for index, fraction in enumerate(parts, 1):
        pose = current.copy()
        pose[:3, :3] = rotation_steps.interpolate(
            current[:3, :3], target_tcp[:3, :3], fraction
        )
        validate_object_pose(pose, held, vertices, config, adaptive)
        target = pose_waypoint(
            'reorient' if len(parts) == 1 else f'reorient_{index}', pose
        )
        target['controller_pose'][:3] = list(current_raw[:3])
        target['method'] = 'joint_pose'
        waypoints.append(target)
    angle = np.arccos(
        np.clip(
            (np.trace(current[:3, :3].T @ target_tcp[:3, :3]) - 1) / 2, -1, 1
        )
    )
    return {
        'base_T_object_target': (
            target_tcp @ np.linalg.inv(attachment)
        ).tolist(),
        'object_T_tcp': attachment.tolist(),
        'fixed_tcp_position_m': current[:3, 3].tolist(),
        'fixed_tcp_position_raw': list(current_raw[:3]),
        'current_arm': {'pose': list(current_raw)},
        'rotation_angle_deg': float(np.rad2deg(angle)),
        'waypoints': waypoints,
        'max_rotation_step_deg': step,
        'tcp_position_fixed': True,
        'position_constraint_scope': 'endpoints_only',
        'motion_command': 'movej_verified_joint_solutions',
        'gripper_action': 'keep_holding',
    }


def checked_position_request(mode):
    if mode not in ('fixed', 'adaptive'):
        raise ValueError('位置模式必须是 fixed 或 adaptive')
    return mode


def position_offsets(current, minimum_z, step):
    """Expand indefinitely within the planning time budget; no workspace or radius cap."""
    current = np.asarray(current, dtype=float)
    if current[2] >= minimum_z:
        yield np.zeros(3)
    else:
        yield np.array([0., 0., minimum_z - current[2]])
    directions = [
        np.asarray(v, dtype=float)
        for v in itertools.product([-1, 0, 1], repeat=3)
        if any(v)
    ]
    directions.sort(key=lambda v: (np.count_nonzero(v), -v[2], tuple(v)))
    index = max(1, int(np.ceil(max(0., minimum_z - current[2]) / step)))
    while True:
        radius = index * step
        vectors = [v / np.linalg.norm(v) for v in directions]
        # Additional rotated sphere directions avoid searching only a fixed set
        # of rays.
        for k in range(16):
            z = 1 - 2 * (k + .5) / 16
            angle = (k + index * 16) * np.pi * (3 - np.sqrt(5))
            vectors.append(
                np.array(
                    [
                        np.sqrt(1 - z * z) * np.cos(angle),
                        np.sqrt(1 - z * z) * np.sin(angle),
                        z,
                    ]
                )
            )
        for direction in vectors:
            offset = direction * radius
            if current[2] + offset[2] >= minimum_z - 1e-10:
                yield offset
        index += 1


def adaptive_candidate(
    held, vertices, rotation, arm, config, settings, offset, adaptive=False
):
    current = grasp_geometry.controller_pose(arm['pose'])
    shifted = current.copy()
    shifted[:3, 3] += offset
    shifted_raw = pose_waypoint('translate', shifted)['controller_pose']
    plan = reorientation_waypoints(
        held,
        vertices,
        rotation,
        shifted_raw,
        config,
        settings,
        adaptive=adaptive,
    )
    validate_object_pose(current, held, vertices, config, adaptive)
    offset = np.asarray(shifted_raw[:3]) / 1e6 - current[:3, 3]
    distance = float(np.linalg.norm(offset))
    if distance > 1e-9:
        plan['waypoints'].insert(
            0,
            pose_waypoint(
                'translate', grasp_geometry.controller_pose(shifted_raw)
            ),
        )
    plan['validation_points'] = plan['waypoints']
    plan.update(
        start_tcp_position_m=current[:3, 3].tolist(),
        target_tcp_position_m=plan.pop('fixed_tcp_position_m'),
        offset_m=offset.tolist(),
        offset_distance_m=distance,
        tcp_position_fixed=distance < 1e-9,
        table_z_m=float(config['table_z_m']),
    )
    plan.pop('fixed_tcp_position_raw', None)
    if adaptive:
        plan['minimum_final_clearance_m'] = .10
    return plan


def plan_reorientation(
    held,
    vertices,
    rotation,
    arm,
    config,
    settings,
    reachable,
    cancelled,
    position_mode='fixed',
    diagnostics=None,
):
    settings = reorientation_settings(settings)
    position_mode = checked_position_request(position_mode)
    checked_rotation(rotation)
    table = config.get('table_z_m')
    if type(table) not in (int, float) or not np.isfinite(table):
        raise ValueError('桌面高度必须为有限米制数值')
    if position_mode == 'fixed':
        if not config.get('workspace_reviewed'):
            raise ValueError('尚未核对姿态调整工作区')
        low, high = (
            np.asarray(config.get('workspace_min_m'), dtype=float),
            np.asarray(config.get('workspace_max_m'), dtype=float),
        )
        if (
            low.shape != (3,)
            or high.shape != (3,)
            or not np.isfinite(np.r_[low, high]).all()
            or np.any(low >= high)
        ):
            raise ValueError('姿态调整工作区配置无效')
    attempts = diagnostics if diagnostics is not None else []
    attempted = 0
    deadline = time.monotonic() + settings['planning_timeout_s']
    current = grasp_geometry.controller_pose(arm['pose'])
    # Include feedback tolerance and one controller position unit so final
    # measured Z can be strictly above 10 cm.
    minimum_z = (
        np.ceil((table + .10 + settings['position_tolerance_m'] + 1e-6) * 1e6)
        / 1e6
    )
    offsets = (
        position_offsets(current[:3, 3], minimum_z, settings['search_step_m'])
        if position_mode == 'adaptive'
        else [np.zeros(3)]
    )
    rotations = pose_symmetry.orientation_variants(rotation, held, current)
    single_fixed = (
        position_mode == 'fixed' and held.get('pose_mode', '6d') != '5d'
    )
    for offset, candidate_rotation in (
        (offset, candidate) for offset in offsets for candidate in rotations
    ):
        cancelled()
        if time.monotonic() >= deadline:
            raise TimeoutError(
                '姿态位置搜索达到时间预算，未找到可执行方案，未发送运动'
            )
        attempted += 1
        attempt = {
            'candidate_index': attempted,
            'offset_m': offset.tolist(),
            'candidate_rotation': candidate_rotation.tolist(),
        }
        attempts.append(attempt)
        # Keep the first and latest 64 diagnostics, regardless of search
        # duration.
        if len(attempts) > 128:
            del attempts[64]
        try:
            plan = adaptive_candidate(
                held,
                vertices,
                candidate_rotation,
                arm,
                config,
                settings,
                offset,
                adaptive=position_mode == 'adaptive',
            )
        except ValueError as error:
            attempt['reason'] = str(error)
            if single_fixed:
                raise
            continue
        remaining = deadline - time.monotonic()
        if remaining <= 0:
            raise TimeoutError(
                '姿态位置搜索达到时间预算，未找到可执行方案，未发送运动'
            )
        try:
            valid, detail = reachable(
                plan['validation_points'], min(remaining, 3.)
            )
        except subprocess.TimeoutExpired:
            attempt['reason'] = '单个候选逆解检查超时'
            cancelled()
            if single_fixed:
                raise TimeoutError('原地姿态检查超时，未发送运动')
            continue
        cancelled()
        if time.monotonic() > deadline:
            raise TimeoutError(
                '姿态位置搜索达到时间预算，未找到可执行方案，未发送运动'
            )
        jump = detail.get('max_joint_step_deg', 0.)
        attempt.update(
            reachable=bool(valid),
            max_joint_step_deg=jump,
            reason=None if valid else detail.get('reason', str(detail)[-700:]),
        )
        if valid:
            plan.update(
                reachability_check=detail,
                current_arm=arm,
                position_mode=position_mode,
                pose_mode=held.get('pose_mode', '6d'),
                requested_axis_z=np.asarray(rotation)[:, 2].tolist(),
                orientation_goal=(
                    'object_z_axis'
                    if held.get('pose_mode') == '5d'
                    else 'full_rotation'
                ),
                search_attempts=attempts,
                search_attempt_count=attempted,
                search_attempts_truncated=attempted > len(attempts),
                validation=(
                    'endpoints_ik_fk_only_controller_joint_path_not_checked'
                ),
                intermediate_path_checked=False,
            )
            return plan
    reason = attempts[-1].get('reason') if attempts else '无候选'
    raise ValueError(
        f"{'自适应姿态调整' if position_mode=='adaptive' else '原地姿态调整'}不"
        f"可达或采样检查失败，未发送运动：{reason}"
    )
