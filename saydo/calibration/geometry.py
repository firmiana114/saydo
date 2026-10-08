"""Frame conventions and quality checks for stationary hand-eye samples."""

import math
import cv2
import numpy as np


def pose_matrix(raw):
    """RealMan JSON: xyz micrometers, roll/pitch/yaw milliradians; Rz Ry Rx."""
    if len(raw) != 6 or not np.isfinite(raw).all():
        raise ValueError('Expected six finite pose values')
    rx, ry, rz = np.asarray(raw[3:], dtype=float) / 1000
    cx, sx, cy, sy, cz, sz = (
        math.cos(rx),
        math.sin(rx),
        math.cos(ry),
        math.sin(ry),
        math.cos(rz),
        math.sin(rz),
    )
    matrix = np.eye(4)
    matrix[:3, :3] = (
        np.array([[cz, -sz, 0], [sz, cz, 0], [0, 0, 1]])
        @ np.array([[cy, 0, sy], [0, 1, 0], [-sy, 0, cy]])
        @ np.array([[1, 0, 0], [0, cx, -sx], [0, sx, cx]])
    )
    matrix[:3, 3] = np.asarray(raw[:3], dtype=float) / 1e6
    return matrix


def zero_reference_frames(tool, work):
    if (
        tool.get('tool_name') not in {'Arm_Tip', 'he_ref'}
        or work.get('frame_name') != 'Base'
    ):
        raise ValueError(
            'Only zero-offset Arm_Tip or he_ref in Base is supported; no frame '
            'changes are performed'
        )
    for frame in [tool, work]:
        pose = frame.get('pose')
        if (
            not isinstance(pose, list)
            or len(pose) != 6
            or any(v != 0 for v in pose)
        ):
            raise ValueError(
                'Nonzero or missing tool/work transform; explicit conversion '
                'is required'
            )


def stationarity(
    states, max_joint_deg=.03, max_translation_m=.0002, max_rotation_deg=.15
):
    if len(states) < 3:
        raise ValueError('At least three state observations are required')
    poses = []
    joints = []
    for state in states:
        if 'err' not in state or any(state['err']):
            raise ValueError('Controller error present or missing')
        q = np.asarray(state.get('joint', []), dtype=float) / 1000
        if q.shape != (7,) or not np.isfinite(q).all():
            raise ValueError('Expected seven finite joint values')
        joints.append(q)
        poses.append(pose_matrix(state['pose']))
    # Pairwise excursions, not just equal start and end poses.
    joint_delta = float(np.max(np.ptp(joints, axis=0)))
    translation = max(
        float(np.linalg.norm(a[:3, 3] - b[:3, 3])) for a in poses for b in poses
    )
    rotation = max(
        math.degrees(
            math.acos(
                float(
                    np.clip((np.trace(a[:3, :3].T @ b[:3, :3]) - 1) / 2, -1, 1)
                )
            )
        )
        for a in poses
        for b in poses
    )
    metrics = dict(
        joint_span_deg=joint_delta,
        translation_span_m=translation,
        rotation_span_deg=rotation,
    )
    if (
        joint_delta > max_joint_deg
        or translation > max_translation_m
        or rotation > max_rotation_deg
    ):
        raise ValueError('Robot not stationary: ' + repr(metrics))
    return metrics


def object_points(columns=8, rows=11, square_m=.03):
    if columns < 2 or rows < 2 or square_m <= 0:
        raise ValueError('Invalid board dimensions')
    points = np.zeros((columns * rows, 3), np.float64)
    points[:, :2] = np.mgrid[0:columns, 0:rows].T.reshape(-1, 2) * square_m
    return points


def board_pose(corners, matrix, distortion, columns=8, rows=11, square_m=.03):
    points = object_points(columns, rows, square_m)
    if np.asarray(corners).size != columns * rows * 2:
        raise ValueError('Incorrect number of corners')
    result = cv2.solvePnPGeneric(
        points,
        np.asarray(corners, dtype=np.float64),
        matrix,
        distortion,
        flags=cv2.SOLVEPNP_IPPE,
    )
    if not result[0]:
        raise ValueError('Planar pose estimation failed')
    candidates = []
    for rv, tv in zip(result[1], result[2]):
        rotation = cv2.Rodrigues(rv)[0]
        camera_points = (rotation @ points.T).T + tv.reshape(1, 3)
        if (
            not np.isfinite(camera_points).all()
            or np.any(camera_points[:, 2] <= 0)
        ):
            continue
        projected = cv2.projectPoints(
            points, rv, tv, matrix, distortion
        )[0].reshape(-1, 2)
        error = projected - np.asarray(corners).reshape(-1, 2)
        transform = np.eye(4)
        transform[:3, :3], transform[:3, 3] = rotation, tv.ravel()
        candidates.append(
            dict(
                camera_T_board=transform.tolist(),
                reprojection_rms_px=float(
                    np.sqrt(np.mean(np.sum(error ** 2, axis=1)))
                ),
            )
        )
    if not candidates:
        raise ValueError('No positive-depth pose solution')
    return sorted(candidates, key=lambda x: x['reprojection_rms_px'])
