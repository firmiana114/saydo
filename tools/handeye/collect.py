#!/usr/bin/env python3
"""One-step, non-moving validation-data collection with automatic duplicate checks."""

import argparse
import fcntl
import json
import math
import os
from pathlib import Path
import subprocess
import sys
import tempfile
import time
import urllib.error
import urllib.request

import numpy as np
from capture import ROOT, Robot
from geometry import pose_matrix, zero_reference_frames


def atomic_json(path, data):
    with tempfile.NamedTemporaryFile(
        'w', dir=path.parent, prefix='.collect-', delete=False
    ) as stream:
        temporary = Path(stream.name)
        try:
            json.dump(
                data, stream, indent=2, ensure_ascii=False, allow_nan=False
            )
            stream.flush()
            os.fsync(stream.fileno())
        except BaseException:
            temporary.unlink(missing_ok=True)
            raise
    try:
        os.replace(temporary, path)
    finally:
        temporary.unlink(missing_ok=True)


def load_samples(session):
    samples = {}
    for path in sorted(session.glob('*/sample.json')):
        sample = json.loads(path.read_text())
        if sample.get('sample_id') != path.parent.name:
            raise ValueError('Sample ID/path mismatch: ' + str(path))
        if sample.get('status') == 'candidate':
            samples[sample['sample_id']] = sample
    return samples


def load_plan(session, samples):
    plan = json.loads((session / 'dataset_split.json').read_text())
    training, validation = (
        plan['training_candidate_ids'], plan['validation_candidate_ids']
    )
    if (
        not training
        or len(set(training + validation)) != len(training + validation)
    ):
        raise ValueError(
            'Missing fitting pool or overlapping/duplicate split membership'
        )
    if any(sample_id not in samples for sample_id in training + validation):
        raise ValueError('Split references a missing or rejected sample')
    target = plan['validation_target_count']
    if not isinstance(target, int) or target < 1 or len(validation) > target:
        raise ValueError('Invalid validation target count')
    # Recover a fully reviewed admission interrupted before the atomic split
    # update.
    for sample_id, sample in samples.items():
        admission = sample.get('collection_admission', {})
        if (
            sample_id in training + validation
            or admission.get('role') != 'validation'
        ):
            continue
        if (
            admission.get('passed') is True
            and admission.get('training_candidate_ids') == training
        ):
            if len(validation) >= target:
                raise ValueError(
                    'Unassigned validation sample exceeds target; review '
                    'before proceeding'
                )
            validation.append(sample_id)
    atomic_json(session / 'dataset_split.json', plan)
    return plan


def transform(value):
    matrix = np.asarray(value, dtype=float)
    if matrix.shape != (4, 4) or not np.isfinite(matrix).all():
        raise ValueError('Expected finite 4x4 transform')
    if (
        not np.allclose(matrix[3], [0, 0, 0, 1])
        or not np.allclose(
            matrix[:3, :3].T @ matrix[:3, :3], np.eye(3), atol=1e-5
        )
        or not np.isclose(np.linalg.det(matrix[:3, :3]), 1, atol=1e-5)
    ):
        raise ValueError('Invalid rigid transform')
    return matrix


def comparisons(pose, samples, duplicate_mm=10, duplicate_deg=5):
    current = transform(pose)
    results = []
    for sample_id, sample in samples.items():
        previous = transform(sample['base_T_flange'])
        mm = float(np.linalg.norm(current[:3, 3] - previous[:3, 3]) * 1000)
        cosine = float(
            np.clip(
                (np.trace(previous[:3, :3].T @ current[:3, :3]) - 1) / 2, -1, 1
            )
        )
        degrees = math.degrees(math.acos(cosine))
        results.append(
            {
                'sample': sample_id,
                'translation_mm': mm,
                'rotation_deg': degrees,
                'near_duplicate': mm < duplicate_mm and degrees < duplicate_deg,
            }
        )
    return results


def pause_preview():
    base = 'http://127.0.0.1:8088'

    def status():
        with urllib.request.urlopen(
            base + '/api/status', timeout=3
        ) as response:
            return json.load(response)

    try:
        value = status()
    except urllib.error.URLError as exc:
        if isinstance(exc.reason, ConnectionRefusedError):
            return 'not_running'
        raise RuntimeError('Could not establish preview status') from exc
    if value['state'] in ('streaming', 'starting', 'stopping'):
        request = urllib.request.Request(
            base + '/api/pause', data=b'', method='POST'
        )
        with urllib.request.urlopen(request, timeout=8) as response:
            value = json.load(response)
        deadline = time.monotonic() + 6
        while value['state'] != 'idle' and time.monotonic() < deadline:
            time.sleep(.2)
            value = status()
    if value['state'] not in ('idle', 'error'):
        raise RuntimeError('Preview has not released camera')
    return value['state']


def run_capture(session, robot_ip):
    result = subprocess.run(
        [
            sys.executable,
            str(Path(__file__).with_name('capture.py')),
            '--session',
            session.name,
            '--robot-ip',
            robot_ip,
        ],
        capture_output=True,
        text=True,
        timeout=45,
    )
    lines = result.stdout.strip().splitlines()
    if not lines:
        raise RuntimeError(
            'Capture returned no result: ' + result.stderr[-1000:]
        )
    outcome = json.loads(lines[-1])
    directory = Path(outcome['directory']).resolve()
    if directory.parent != session.resolve():
        raise ValueError('Unexpected capture result directory')
    sample = json.loads((directory / 'sample.json').read_text())
    if sample.get('sample_id') != directory.name:
        raise ValueError('Capture sample ID/path mismatch')
    (
        directory / 'capture-process.log'
    ).write_text(result.stdout + result.stderr)
    if result.returncode and sample.get('status') == 'candidate':
        raise RuntimeError('Capture process failed despite candidate status')
    return directory, sample


def check_compatibility(reference, sample):
    for key in ['board', 'camera']:
        if sample.get(key) != reference.get(key):
            raise ValueError(
                'Camera or board parameters differ from the frozen fitting pool'
            )
    for key in [
        'get_current_tool_frame',
        'get_current_work_frame',
        'get_arm_software_info',
    ]:
        if sample['preflight'][key] != reference['preflight'][key]:
            raise ValueError(
                'Robot identity or frame configuration differs from fitting '
                'pool'
            )
    if len(sample.get('corners_px', [])) != 88:
        raise ValueError('Expected 88 corners')


def collect_once(session, robot_ip='192.168.1.19', settle_seconds=2):
    with (session / '.collect.lock').open('a') as lock:
        try:
            fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
        except BlockingIOError as exc:
            raise RuntimeError(
                'Another collector is using this session'
            ) from exc
        samples = load_samples(session)
        plan = load_plan(session, samples)

        def finish(state, message, **details):
            result = {
                'state': state,
                'message': message,
                'candidate_count': len(load_samples(session)),
                'training_candidate_count': len(plan['training_candidate_ids']),
                'validation_candidate_count': len(
                    plan['validation_candidate_ids']
                ),
                'validation_target_count': plan['validation_target_count'],
                'preview_policy': 'never_restore',
                **details,
            }
            result['rejected_count'] = sum(
                json.loads(p.read_text()).get('status') == 'rejected'
                for p in session.glob('*/sample.json')
            )
            atomic_json(session / 'latest_progress.json', result)
            return result

        if (
            len(plan['validation_candidate_ids'])
            >= plan['validation_target_count']
        ):
            return finish(
                'complete',
                '独立验证候选集已达到计划数量，未再打开相机或新增样本。',
            )
        reference = samples[plan['training_candidate_ids'][0]]
        robot = Robot(robot_ip)
        tool = robot.query('get_current_tool_frame')['reply']
        work = robot.query('get_current_work_frame')['reply']
        zero_reference_frames(tool, work)
        if (
            tool != reference['preflight']['get_current_tool_frame']
            or work != reference['preflight']['get_current_work_frame']
        ):
            raise ValueError(
                'Active tool/work configuration differs from fitting pool'
            )
        before = robot.query('get_current_arm_state')['reply']['arm_state']
        if 'err' not in before or any(before['err']):
            raise ValueError('Robot error present or status missing')
        initial = comparisons(pose_matrix(before['pose']), samples)
        duplicate = [x for x in initial if x['near_duplicate']]
        if duplicate:
            return finish(
                'duplicate',
                '当前姿态接近已有样本，跳过采集。',
                duplicates=duplicate,
            )
        preview = pause_preview()
        time.sleep(settle_seconds)
        directory, sample = run_capture(session, robot_ip)
        if sample['status'] != 'candidate':
            return finish(
                'rejected',
                '样本未通过采集质量检查，原始记录已保留。',
                directory=str(directory),
                reason=sample.get('error'),
                preview_before_capture=preview,
            )
        try:
            current_samples = load_samples(session)
            current_samples.pop(sample['sample_id'])
            post = comparisons(sample['base_T_flange'], current_samples)
            duplicate = [x for x in post if x['near_duplicate']]
            if duplicate:
                raise ValueError(
                    'Captured pose duplicates an existing sample: '
                    + json.dumps(duplicate)
                )
            check_compatibility(reference, sample)
        except ValueError as exc:
            sample['status'], sample['error'] = 'rejected', str(exc)
            atomic_json(directory / 'sample.json', sample)
            return finish(
                'rejected',
                '采集后去重或参数一致性检查未通过。',
                directory=str(directory),
                reason=str(exc),
            )
        sample['collection_admission'] = {
            'role': 'validation',
            'passed': True,
            'training_candidate_ids': list(plan['training_candidate_ids']),
        }
        atomic_json(directory / 'sample.json', sample)
        atomic_json(
            directory / 'novelty_check.json',
            {
                'comparisons': post,
                'duplicate_rule': {
                    'translation_mm_less_than': 10,
                    'rotation_deg_less_than': 5,
                    'operator': 'AND',
                },
            },
        )
        plan['validation_candidate_ids'].append(sample['sample_id'])
        atomic_json(session / 'dataset_split.json', plan)
        return finish(
            'captured',
            '已保存一组不重复的独立验证候选样本。',
            directory=str(directory),
            sample_id=sample['sample_id'],
            corners=len(sample['corners_px']),
            reprojection_rms_px=sample[
                'board_pose_candidates'
            ][0]['reprojection_rms_px'],
            stationarity=sample['stationarity'],
            preview_before_capture=preview,
        )


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--session', required=True)
    parser.add_argument('--robot-ip', default='192.168.1.19')
    args = parser.parse_args()
    if (
        not args.session
        or any(
            (
                c
                not in (
                    'abcdefghijklmnopqrstuvwxyzABCDEFGHIJKLMNOPQRSTUVWXYZ012345'
                    '6789_-'
                )
            )
            for c in args.session
        )
    ):
        parser.error('Invalid session name')
    session = ROOT / 'work' / 'handeye' / args.session
    if not (session / 'dataset_split.json').is_file():
        parser.error('Session needs an existing frozen dataset_split.json')
    try:
        result = collect_once(session, args.robot_ip)
    except Exception as exc:
        print(
            json.dumps(
                {'state': 'error', 'message': str(exc)}, ensure_ascii=False
            )
        )
        return 1
    print(json.dumps(result, ensure_ascii=False))
    return 0 if result['state'] in ('captured', 'complete') else 2


if __name__ == '__main__':
    raise SystemExit(main())
