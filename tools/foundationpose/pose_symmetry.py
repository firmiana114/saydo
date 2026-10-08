"""Continuous object-Z symmetry; TCP poses remain fully constrained rigid poses."""

import copy
import math

import numpy as np

from grasp_geometry import transform, rotation_error


HORIZONTAL_AXIS_TOLERANCE_DEG = 10.


def checked_mode(value):
    if value not in ('6d', '5d'):
        raise ValueError('位姿模式必须是 6d 或 5d')
    return value


def axis_rotation(axis):
    z = np.asarray(axis, dtype=float)
    if (
        z.shape != (3,)
        or not np.isfinite(z).all()
        or not np.isclose(np.linalg.norm(z), 1, atol=1e-4)
    ):
        raise ValueError('物品 Z 轴必须是单位方向向量')
    z = z / np.linalg.norm(z)
    reference = np.eye(3)[int(np.argmin(np.abs(z)))]
    x = reference - z * np.dot(reference, z)
    x /= np.linalg.norm(x)
    return np.column_stack((x, np.cross(z, x), z))


def canonical_pose(pose, mode='6d'):
    result = transform(pose).copy()
    if mode == '5d':
        result[:3, :3] = axis_rotation(result[:3, 2])
    return result


def object_rotation_error(a, b, mode='6d'):
    if mode == '5d':
        return float(
            np.arccos(
                np.clip(np.dot(np.asarray(a)[:, 2], np.asarray(b)[:, 2]), -1, 1)
            )
        )
    return rotation_error(a, b)


def pose_fields(pose, mode='6d'):
    matrix = transform(pose)
    return {
        'pose_mode': mode,
        'spin_observable': mode != '5d',
        'position_m': matrix[:3, 3].tolist(),
        'axis_z': matrix[:3, 2].tolist(),
        'rotation_semantics': (
            'representative_only_object_z_spin_free'
            if mode == '5d'
            else 'full_rotation'
        ),
    }


def z_rotation(angle):
    c, s = math.cos(angle), math.sin(angle)
    return np.array([[c, -s, 0], [s, c, 0], [0, 0, 1.]])


def grasp_rotation(angle_deg):
    """Choose a representative: jaw-closing TCP X is tangent to the object axis."""
    if (
        type(angle_deg) not in (int, float)
        or not np.isfinite(angle_deg)
        or not 0 <= angle_deg <= 180
    ):
        raise ValueError('接近方向与物品 Z 轴夹角必须在 0–180 度之间')
    angle = np.deg2rad(angle_deg)
    z = np.array([np.sin(angle), 0., np.cos(angle)])
    x = np.array([0., 1., 0.])
    return np.column_stack((x, np.cross(z, x), z))


def checked_axis_profile(profile):
    if not isinstance(profile, dict):
        raise ValueError('缺少5D夹法高度和接近夹角')
    point = np.asarray(profile.get('grasp_point_object_m'), dtype=float)
    if (
        point.shape != (3,)
        or not np.isfinite(point).all()
        or np.any(np.abs(point[:2]) > 1e-9)
    ):
        raise ValueError('5D夹法的抓取中心必须位于物品 Z 轴上（X=Y=0）')
    angle = profile.get('approach_angle_deg')
    rotation = grasp_rotation(angle)
    return {
        'orientation': 'axisymmetric_z',
        'grasp_point_object_m': [0., 0., float(point[2])],
        'approach_angle_deg': float(angle),
        'object_R_tcp': rotation.tolist(),
        'approach_axis_tcp': [0, 0, 1],
    }


def effective_profile(entry, mode='6d'):
    if mode != '5d':
        return copy.deepcopy(entry['profile'])
    if entry.get('profile_5d') is not None:
        return checked_axis_profile(entry['profile_5d'])
    old = entry['profile']
    if 'object_R_tcp' not in old:
        raise ValueError('5D夹法尚无接近夹角，请在夹法编辑器中新建或记录夹法')
    matrix = np.eye(4)
    matrix[:3, :3] = old['object_R_tcp']
    rotation = transform(matrix)[:3, :3]
    return checked_axis_profile(
        {
            'grasp_point_object_m': [0, 0, old['grasp_point_object_m'][2]],
            'approach_angle_deg': float(
                np.rad2deg(np.arccos(np.clip(rotation[2, 2], -1, 1)))
            ),
        }
    )


def grasp_variants(base_object, profile, current_rotation):
    """Search a bounded azimuth set, seeded by the current approach direction."""
    base_object = transform(base_object)
    axis = base_object[:3, 2]
    elevation = float(np.rad2deg(np.arcsin(np.clip(axis[2], -1., 1.))))
    if abs(elevation) <= HORIZONTAL_AXIS_TOLERANCE_DEG + 1e-10:
        # Horizontal 5D objects must be approached from above, regardless of the
        # saved approach angle. Keep jaw closing perpendicular to the object
        # axis.
        down = np.array([0., 0., -1.])
        x = np.cross(axis, down)
        x /= np.linalg.norm(x)
        target = np.column_stack((x, np.cross(down, x), down))
        angle = float(
            np.rad2deg(np.arccos(np.clip(np.dot(axis, down), -1., 1.)))
        )
        candidates = []
        for roll in (0., np.pi):
            rotation = target @ z_rotation(roll)
            resolved = {
                **profile,
                'orientation': 'follow_object',
                'approach_angle_deg': angle,
                'object_R_tcp': (base_object[:3, :3].T @ rotation).tolist(),
            }
            details = {
                'approach_policy': 'horizontal_5d_vertical_down',
                'object_axis_elevation_deg': elevation,
                'horizontal_tolerance_deg': HORIZONTAL_AXIS_TOLERANCE_DEG,
                'configured_approach_angle_deg': profile['approach_angle_deg'],
                'effective_approach_angle_deg': angle,
                'jaw_roll_deg': float(np.rad2deg(roll)),
            }
            candidates.append(
                (rotation_error(current_rotation, rotation), resolved, details)
            )
        return [
            (resolved, details)
            for _, resolved, details in sorted(
                candidates, key=lambda item: item[0]
            )
        ]
    basis = axis_rotation(axis)
    seed = basis.T @ np.asarray(current_rotation)[:, 2]
    phase = math.atan2(seed[1], seed[0])
    representative = grasp_rotation(profile['approach_angle_deg'])
    candidates = []
    for azimuth in (phase + k * np.pi / 12 for k in range(24)):
        for roll in (0., np.pi):
            target = (
                basis @ z_rotation(azimuth) @ representative @ z_rotation(roll)
            )
            relative = np.asarray(base_object)[:3, :3].T @ target
            resolved = {
                **profile,
                'orientation': 'follow_object',
                'object_R_tcp': relative.tolist(),
            }
            candidates.append(
                (
                    rotation_error(current_rotation, target),
                    resolved,
                    {
                        'azimuth_deg': float(np.rad2deg(azimuth) % 360),
                        'jaw_roll_deg': float(np.rad2deg(roll)),
                    },
                )
            )
    return [
        (profile, details)
        for _, profile, details in sorted(candidates, key=lambda item: item[0])
    ]


def orientation_variants(rotation, held, current_tcp):
    """Spin around target object Z is free; first test the closest TCP rotation."""
    if held.get('pose_mode', '6d') != '5d':
        return [np.asarray(rotation)]
    basis = axis_rotation(np.asarray(rotation)[:, 2])
    attachment = transform(held['object_T_tcp'])[:3, :3]
    reference = np.asarray(current_tcp)[:3, :3] @ attachment.T
    # Maximize trace(reference.T @ basis @ Rz(phi)), the continuous nearest
    # twist.
    relative = reference.T @ basis
    phase = math.atan2(
        relative[0, 1] - relative[1, 0], relative[0, 0] + relative[1, 1]
    )
    variants = [basis @ z_rotation(phase + k * np.pi / 12) for k in range(24)]
    return sorted(
        variants,
        key=lambda r: rotation_error(
            np.asarray(current_tcp)[:3, :3], r @ attachment
        ),
    )


def object_bounds(pose, vertices, mode='6d'):
    """Exact support bounds of the mesh's continuous rotational sweep, not samples."""
    matrix = transform(pose)
    points = np.asarray(vertices, dtype=float)
    if (
        points.ndim != 2
        or points.shape[1] != 3
        or not len(points)
        or not np.isfinite(points).all()
    ):
        raise ValueError('物品模型顶点无效')
    if mode != '5d':
        world = points @ matrix[:3, :3].T + matrix[:3, 3]
        return world.min(axis=0), world.max(axis=0)
    z = matrix[:3, 2]
    centers = matrix[:3, 3] + points[:, 2, None] * z
    radius = (
        np.linalg.norm(points[:, :2], axis=1)[:, None]
        * np.sqrt(np.maximum(0., 1 - z * z))
    )
    return (centers - radius).min(axis=0), (centers + radius).max(axis=0)
