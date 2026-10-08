#!/usr/bin/env python3
"""Offline eye-in-hand fit on frozen training IDs, evaluated on untouched holdouts."""

import argparse
import datetime
import hashlib
import itertools
import json
import math
from pathlib import Path
import uuid

import cv2
import numpy as np
from saydo.calibration.geometry import object_points, pose_matrix, zero_reference_frames

from saydo import paths

ROOT = paths.ROOT
# Fixed before inspecting held-out residuals; no validation-based selection.
METHOD = 'PARK'


def rigid(value):
    t = np.asarray(value, dtype=float)
    if t.shape != (4, 4) or not np.isfinite(t).all():
        raise ValueError('Expected a finite 4x4 transform')
    if (
        not np.allclose(t[3], [0, 0, 0, 1])
        or not np.allclose(t[:3, :3].T @ t[:3, :3], np.eye(3), atol=1e-5)
        or not np.isclose(np.linalg.det(t[:3, :3]), 1, atol=1e-5)
    ):
        raise ValueError('Invalid rigid transform')
    return t


def rotation_degrees(r):
    return math.degrees(math.acos(float(np.clip((np.trace(r) - 1) / 2, -1, 1))))


def excitation(samples):
    matrices = [rigid(s['base_T_flange']) for s in samples]
    rows = [
        a[:3, :3].T @ b[:3, :3] - np.eye(3)
        for a, b in itertools.combinations(matrices, 2)
    ]
    if not rows:
        raise ValueError('Too few poses')
    values = np.linalg.svd(np.vstack(rows), compute_uv=False)
    if values[-1] < 1e-6:
        raise ValueError(
            'Degenerate rotations: nonparallel rotation axes are required'
        )
    return {
        'singular_values': values.tolist(),
        'condition_number': float(values[0] / values[-1]),
    }


def fit(samples):
    if len(samples) < 3:
        raise ValueError('At least three distinct poses are required')
    excitation(samples)
    if not hasattr(cv2, 'calibrateHandEye'):
        raise RuntimeError(
            'Use the separate solver environment with requirements-solve.txt'
        )
    g = [rigid(s['base_T_flange']) for s in samples]
    c = [rigid(s['camera_T_board_candidate']) for s in samples]
    r, t = cv2.calibrateHandEye(
        [x[:3, :3] for x in g],
        [x[:3, 3:4] for x in g],
        [x[:3, :3] for x in c],
        [x[:3, 3:4] for x in c],
        method=cv2.CALIB_HAND_EYE_PARK,
    )
    result = np.eye(4)
    result[:3, :3], result[:3, 3] = r, t.ravel()
    return rigid(result)


def mean_pose(poses):
    result = np.eye(4)
    u, _, vt = np.linalg.svd(sum(t[:3, :3] for t in poses))
    result[:3, :3] = u @ np.diag([1., 1., np.linalg.det(u @ vt)]) @ vt
    result[:3, 3] = np.mean([t[:3, 3] for t in poses], axis=0)
    return result


def board_in_base(sample, handeye):
    return (
        rigid(sample['base_T_flange'])
        @ handeye
        @ rigid(sample['camera_T_board_candidate'])
    )


def statistics(values):
    a = np.asarray(values, dtype=float)
    return {
        'rms': float(np.sqrt(np.mean(a * a))),
        'median': float(np.median(a)),
        'max': float(np.max(a)),
    }


def evaluate(samples, handeye, training_board_reference):
    rows = []
    for sample in samples:
        board = board_in_base(sample, handeye)
        shift = (board[:3, 3] - training_board_reference[:3, 3]) * 1000
        row = {
            'sample_id': sample['sample_id'],
            'translation_error_mm': float(np.linalg.norm(shift)),
            'translation_error_xyz_mm': shift.tolist(),
            'rotation_error_deg': rotation_degrees(
                training_board_reference[:3, :3].T @ board[:3, :3]
            ),
        }
        if 'camera' in sample and 'corners_px' in sample:
            prediction = (
                np.linalg.inv(handeye)
                @ np.linalg.inv(rigid(sample['base_T_flange']))
                @ training_board_reference
            )
            points = object_points()
            camera_points = (
                (prediction[:3, :3] @ points.T).T + prediction[:3, 3]
            )
            if np.any(camera_points[:, 2] <= 0):
                raise ValueError('Global board prediction falls behind camera')
            projected = cv2.projectPoints(
                points,
                cv2.Rodrigues(prediction[:3, :3])[0],
                prediction[:3, 3],
                np.array(sample['camera']['K']),
                np.zeros(5),
            )[0].reshape(-1, 2)
            difference = projected - np.asarray(sample['corners_px'])
            row['global_reprojection_rms_px'] = float(
                np.sqrt(np.mean(np.sum(difference * difference, axis=1)))
            )
        rows.append(row)
    summary = {
        'translation_error_mm': statistics(
            [x['translation_error_mm'] for x in rows]
        ),
        'rotation_error_deg': statistics(
            [x['rotation_error_deg'] for x in rows]
        ),
    }
    if all('global_reprojection_rms_px' in x for x in rows):
        summary['global_reprojection_rms_px'] = statistics(
            [x['global_reprojection_rms_px'] for x in rows]
        )
    return {'summary': summary, 'samples': rows}


def solve_dataset(training, validation):
    if not training or not validation:
        raise ValueError('Both fitting and held-out samples are required')
    training_ids = [s['sample_id'] for s in training]
    validation_ids = [s['sample_id'] for s in validation]
    if (
        len(set(training_ids + validation_ids))
        != len(training_ids + validation_ids)
    ):
        raise ValueError('Duplicate IDs or training/validation leakage')
    handeye = fit(training)
    board_reference = mean_pose([board_in_base(s, handeye) for s in training])
    # Stability is evaluated only on training subsets, never on validation data.
    stability = []
    if len(training) >= 5:
        for i, sample in enumerate(training):
            alternative = fit(training[:i] + training[i + 1:])
            stability.append(
                {
                    'omitted_training_sample': sample['sample_id'],
                    'translation_change_mm': float(
                        np.linalg.norm(alternative[:3, 3] - handeye[:3, 3])
                        * 1000
                    ),
                    'rotation_change_deg': rotation_degrees(
                        handeye[:3, :3].T @ alternative[:3, :3]
                    ),
                }
            )
    return {
        'method': METHOD,
        'opencv_version': cv2.__version__,
        'flange_T_color_optical_camera': handeye.tolist(),
        'color_optical_camera_T_flange': np.linalg.inv(handeye).tolist(),
        'base_T_board_training_reference': board_reference.tolist(),
        'training_candidate_ids': training_ids,
        'validation_candidate_ids': validation_ids,
        'training_excitation': excitation(training),
        'training': evaluate(training, handeye, board_reference),
        'heldout_validation': evaluate(validation, handeye, board_reference),
        'training_leave_one_out': stability,
        'acceptance_status': (
            'not_decided_no_task_tolerance_or_absolute_reference'
        ),
        'deployed': False,
        'validation_used_for_fitting_or_tuning': False,
    }


def load_dataset(session):
    plan_path = session / 'dataset_split.json'
    plan = json.loads(plan_path.read_text())
    training_ids, validation_ids = (
        plan['training_candidate_ids'], plan['validation_candidate_ids']
    )
    ids = training_ids + validation_ids
    if not training_ids or not validation_ids or len(set(ids)) != len(ids):
        raise ValueError('Invalid or overlapping data split')
    if len(validation_ids) != plan['validation_target_count']:
        raise ValueError('Held-out collection is incomplete')
    samples, hashes = (
        {},
        {
            'dataset_split.json': hashlib.sha256(
                plan_path.read_bytes()
            ).hexdigest()
        },
    )
    reference = None
    for sample_id in ids:
        if (
            not sample_id
            or any(
                (
                    c
                    not in (
                        '0123456789abcdefghijklmnopqrstuvwxyzABCDEFGHIJKLMNOPQR'
                        'STUVWXYZ_-'
                    )
                )
                for c in sample_id
            )
        ):
            raise ValueError('Invalid sample ID')
        directory = session / sample_id
        sample_path = directory / 'sample.json'
        sample = json.loads(sample_path.read_text())
        if (
            sample.get('status') != 'candidate'
            or sample.get('sample_id') != sample_id
        ):
            raise ValueError(
                'Missing, mismatched or rejected sample: ' + sample_id
            )
        image_path = directory / 'color.png'
        digest = hashlib.sha256(image_path.read_bytes()).hexdigest()
        if digest != sample['image_sha256']:
            raise ValueError('Image integrity mismatch: ' + sample_id)
        image = cv2.imread(str(image_path))
        if (
            image is None
            or list(image.shape[1::-1]) != sample['camera']['size']
        ):
            raise ValueError('Image dimensions mismatch: ' + sample_id)
        corners = np.asarray(sample['corners_px'])
        if corners.shape != (88, 2) or not np.isfinite(corners).all():
            raise ValueError('Invalid board corners: ' + sample_id)
        if (
            sample['board']['inner_corners'] != [8, 11]
            or sample['board']['square_size_m'] != .03
        ):
            raise ValueError('Unexpected board definition')
        if any(x != 0 for x in sample['camera']['distortion_coefficients']):
            raise ValueError('Unsupported nonzero distortion')
        preflight = sample['preflight']
        zero_reference_frames(
            preflight['get_current_tool_frame'],
            preflight['get_current_work_frame'],
        )
        if reference is None:
            reference = sample
        for field in ['camera', 'board']:
            if sample[field] != reference[field]:
                raise ValueError('Session metadata differs: ' + field)
        for field in [
            'get_current_tool_frame',
            'get_current_work_frame',
            'get_arm_software_info',
        ]:
            if preflight[field] != reference['preflight'][field]:
                raise ValueError('Robot/frame metadata differs: ' + field)
        before = [
            o
            for o in sample['observations']
            if o['request_monotonic'] == sample['timing']['before']
        ]
        if (
            len(before) != 1
            or not np.allclose(
                pose_matrix(before[0]['reply']['arm_state']['pose']),
                sample['base_T_flange'],
                atol=1e-10,
            )
        ):
            raise ValueError(
                'Robot transform does not match paired raw observation'
            )
        g, c = (
            rigid(sample['base_T_flange']),
            rigid(sample['camera_T_board_candidate']),
        )
        projected = cv2.projectPoints(
            object_points(),
            cv2.Rodrigues(c[:3, :3])[0],
            c[:3, 3],
            np.array(sample['camera']['K']),
            np.zeros(5),
        )[0].reshape(-1, 2)
        rms = float(
            np.sqrt(np.mean(np.sum((projected - corners) ** 2, axis=1)))
        )
        stored = sample['board_pose_candidates'][0]['reprojection_rms_px']
        if rms > 1 or abs(rms - stored) > 1e-4:
            raise ValueError('PnP quality record mismatch: ' + sample_id)
        hashes[sample_id + '/sample.json'] = hashlib.sha256(
            sample_path.read_bytes()
        ).hexdigest()
        hashes[sample_id + '/color.png'] = digest
        samples[sample_id] = sample
    return (
        [samples[i] for i in training_ids],
        [samples[i] for i in validation_ids],
        hashes,
    )


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--session', required=True)
    args = parser.parse_args()
    if (
        not args.session
        or any(
            (
                c
                not in (
                    '0123456789abcdefghijklmnopqrstuvwxyzABCDEFGHIJKLMNOPQRSTUV'
                    'WXYZ_-'
                )
            )
            for c in args.session
        )
    ):
        parser.error('Invalid session')
    session = ROOT / 'work' / 'handeye' / args.session
    training, validation, hashes = load_dataset(session)
    report = solve_dataset(training, validation)
    report.update(
        {
            'created_at': datetime.datetime.now().astimezone().isoformat(),
            'source_sha256': hashes,
            'units': {
                'translation': 'meters', 'rotation_matrix': 'dimensionless'
            },
            'frame_convention': 'A_T_B maps coordinates in B into A',
            'camera': training[0]['camera'],
            'board': training[0]['board'],
            'limitations': [
                'Board square size is user-stated, not independently measured.',
                (
                    'Fixed-board consistency is not absolute positioning or '
                    'grasp accuracy.'
                ),
                'No task-specific acceptance tolerance has been provided.',
                'No calibration or robot parameter is written automatically.',
            ],
        }
    )
    out = (
        session
        / 'solutions'
        / (
            datetime.datetime.now().strftime('%H%M%S')
            + '_'
            + uuid.uuid4().hex[:8]
        )
    )
    out.mkdir(parents=True, exist_ok=False)
    (out / 'result.json').write_text(
        json.dumps(report, indent=2, allow_nan=False)
    )
    print(
        json.dumps(
            {
                'directory': str(out),
                'method': METHOD,
                'flange_T_camera': report['flange_T_color_optical_camera'],
                'training': report['training']['summary'],
                'heldout_validation': report['heldout_validation']['summary'],
                'max_leave_one_out_translation_change_mm': max(
                    (
                        x['translation_change_mm']
                        for x in report['training_leave_one_out']
                    ),
                    default=None,
                ),
                'max_leave_one_out_rotation_change_deg': max(
                    (
                        x['rotation_change_deg']
                        for x in report['training_leave_one_out']
                    ),
                    default=None,
                ),
                'acceptance_status': report['acceptance_status'],
                'deployed': False,
            }
        )
    )


if __name__ == '__main__':
    main()
