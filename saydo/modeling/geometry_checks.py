"""Geometry for static multi-view capture; no robot or camera commands."""

import math

import numpy as np


def rigid(value):
    t = np.asarray(value, dtype=float)
    if t.shape != (4, 4) or not np.isfinite(t).all():
        raise ValueError('Expected finite 4x4 transform')
    r = t[:3, :3]
    if (
        not np.allclose(t[3], [0, 0, 0, 1])
        or not np.allclose(r.T @ r, np.eye(3), atol=1e-5)
        or not np.isclose(np.linalg.det(r), 1, atol=1e-5)
    ):
        raise ValueError('Invalid rigid transform')
    return t


def camera_pose(base_T_tcp, flange_T_tcp, flange_T_camera):
    return (
        rigid(base_T_tcp)
        @ np.linalg.inv(rigid(flange_T_tcp))
        @ rigid(flange_T_camera)
    )


def distance(a, b):
    a, b = rigid(a), rigid(b)
    mm = float(np.linalg.norm(a[:3, 3] - b[:3, 3]) * 1000)
    deg = math.degrees(
        math.acos(
            float(np.clip((np.trace(a[:3, :3].T @ b[:3, :3]) - 1) / 2, -1, 1))
        )
    )
    return mm, deg


def duplicate(pose, samples):
    for sample in samples:
        mm, deg = distance(pose, sample['base_T_color_optical_camera'])
        if mm < 10 and deg < 5:
            return {
                'sample_id': sample['sample_id'],
                'translation_mm': mm,
                'rotation_deg': deg,
            }
    return None


def depth_millimeters(raw, scale):
    if not np.isfinite(scale) or scale <= 0:
        raise ValueError('Invalid device depth scale')
    raw = np.asarray(raw)
    if raw.dtype != np.uint16 or raw.ndim != 2:
        raise ValueError('Expected uint16 depth image')
    mm = np.rint(raw.astype(np.float64) * scale * 1000)
    if np.any(mm > 65535):
        raise ValueError('Depth exceeds uint16 millimeter range')
    return mm.astype(np.uint16)


def validate_configs(handeye, tool):
    for config in (handeye, tool):
        if (
            config['schema_version'] != 1
            or config['units'] != {'translation': 'm', 'rotation': 'rad'}
        ):
            raise ValueError('Unsupported calibration schema or units')
    if handeye['installation'] != tool['installation']:
        raise ValueError('Calibration installation mismatch')
    if handeye['acceptance_status'] != 'user_accepted_usable':
        raise ValueError('Hand-eye result is not accepted')
    if (
        handeye['frames']
        != {'parent': 'flange', 'child': 'color_optical_frame'}
    ):
        raise ValueError('Unexpected hand-eye frame direction')
    if (
        tool['frames'] != {'parent': 'flange', 'child': 'grip_tcp'}
        or tool['tool_name'] != 'grip_tcp'
    ):
        raise ValueError('Unexpected tool frame')
    rigid(handeye['flange_T_color_optical_camera'])
    rigid(tool['flange_T_grip_tcp'])
