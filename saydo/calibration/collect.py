#!/usr/bin/env python3
"""One-step, non-moving validation-data collection with automatic duplicate checks."""

import argparse
import fcntl
import json
import math
import os
import pathlib
import subprocess
import sys
import tempfile
import time
import urllib.error
import urllib.request
from typing import Optional, Tuple

import numpy as np
from saydo.calibration import capture
from saydo.calibration import geometry


def atomic_json(path, data):
    with tempfile.NamedTemporaryFile(
        'w', dir=path.parent, prefix='.collect-', delete=False
    ) as stream:
        temporary = pathlib.Path(stream.name)
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


def run_capture(
    session: pathlib.Path, robot_ip: str, serial: Optional[str] = None
) -> Tuple[pathlib.Path, dict]:
    """启动一次采集子进程并读回它保存的样本；不重试。

    Args:
        session: 会话目录。
        robot_ip: 机械臂控制器地址。
        serial: 相机序列号；None 表示只连着一台 D435i 时就用它。

    Returns:
        `(样本目录, 样本内容)`。样本可能是被拒收的，由调用方看
        `status` 判断。

    Raises:
        RuntimeError: 子进程没有给出结果，或结果与保存的样本矛盾。
        ValueError: 子进程报告的样本不在本会话目录下。
    """
    command = [
        sys.executable,
        '-m',
        'saydo.calibration.capture',
        '--session',
        session.name,
        '--robot-ip',
        robot_ip,
    ]
    if serial is not None:
        command += ['--serial', serial]
    result = subprocess.run(
        command,
        capture_output=True,
        text=True,
        timeout=45,
        cwd=capture.ROOT,
    )
    lines = result.stdout.strip().splitlines()
    if not lines:
        raise RuntimeError(
            'Capture returned no result: ' + result.stderr[-1000:]
        )
    outcome = json.loads(lines[-1])
    directory = pathlib.Path(outcome['directory']).resolve()
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


FITTING_ROLE = 'training'
VALIDATION_ROLE = 'validation'


def fitting_plan(
    session: pathlib.Path,
    samples: dict,
    fitting_count: int,
    validation_count: int,
) -> dict:
    """整理“求解样本阶段”的进度；只读磁盘，不写文件。

    求解样本阶段是一个新会话的开头：一组一组地采，凑够 `fitting_count`
    组后把名单冻结成 `dataset_split.json`。名单一旦冻结就不能再往里
    加，这样后面采的验证样本才是真正没参与求解的。

    Args:
        session: 会话目录。
        samples: 会话里现有的候选样本，编号到样本内容。
        fitting_count: 求解样本的目标组数。
        validation_count: 之后要采的验证样本组数，冻结时写入名单。

    Returns:
        与冻结名单同形的进度：已登记的求解样本编号、空的验证名单和
        两个目标数量。

    Raises:
        ValueError: 名单已经冻结，或会话里有不是本流程登记的候选样本
            （无法判断它们该不该算进求解样本）。
    """
    if (session / 'dataset_split.json').exists():
        raise ValueError(
            '这个会话的分组名单已经冻结，求解样本不能再增加；'
            '采集验证样本请去掉 --fitting'
        )
    unregistered = sorted(
        sample_id
        for sample_id, sample in samples.items()
        if (
            sample.get('collection_admission', {}).get('role') != FITTING_ROLE
            or sample['collection_admission'].get('passed') is not True
        )
    )
    if unregistered:
        raise ValueError(
            '会话里有不是求解样本阶段登记的候选样本：'
            + '、'.join(unregistered)
            + '；请换一个新的会话名，或先处理这些样本目录'
        )
    return {
        'training_candidate_ids': sorted(samples),
        'validation_candidate_ids': [],
        'training_target_count': fitting_count,
        'validation_target_count': validation_count,
    }


def freeze_plan(session: pathlib.Path, plan: dict) -> None:
    """把求解样本名单写成 `dataset_split.json`，此后不再改动这份名单。

    Args:
        session: 会话目录。
        plan: `fitting_plan` 的进度，求解样本已凑够目标数量。
    """
    count = len(plan['training_candidate_ids'])
    atomic_json(
        session / 'dataset_split.json',
        {
            'schema_version': 1,
            'status': 'collection_in_progress_no_handeye_solution',
            'training_candidate_ids': list(plan['training_candidate_ids']),
            'validation_candidate_ids': [],
            'validation_target_count': plan['validation_target_count'],
            'policy': (
                f'最先登记的 {count} 组候选样本冻结为求解样本；之后采集 '
                f"{plan['validation_target_count']} 组姿态不同的样本作为"
                '独立验证，不参与求解和调参。所有候选样本仍需人工审阅质量。'
            ),
        },
    )


def collect_once(
    session: pathlib.Path,
    robot_ip: str = '192.168.1.19',
    settle_seconds: float = 2,
    serial: Optional[str] = None,
    fitting_count: Optional[int] = None,
    validation_count: int = 5,
) -> dict:
    """检查当前姿态并采集一组样本；只读查询机械臂，不发送任何指令。

    一次调用只处理一组：姿态与已有样本太接近就跳过，否则采集、复查并
    登记。`fitting_count` 为 None 时登记为验证样本（要求名单已冻结）；
    给出数值时登记为求解样本，凑够后冻结名单。

    Args:
        session: 会话目录，必须已存在。
        robot_ip: 机械臂控制器地址。
        settle_seconds: 采集前的等待时间，单位秒，让相机和手臂稳定。
        serial: 相机序列号；None 表示只连着一台 D435i 时就用它。
        fitting_count: 求解样本的目标组数；None 表示采集验证样本。
        validation_count: 冻结名单时写入的验证样本组数。

    Returns:
        本次的结果与进度。`state` 为 `captured`（已登记）、`duplicate`
        （姿态接近已有样本，未采集）、`rejected`（样本不合格）或
        `complete`（验证样本已满）。

    Raises:
        RuntimeError: 另一个采集进程正在使用这个会话，或相机状态无法
            确认。
        ValueError: 名单、坐标系设置或机械臂状态不符合要求。
    """
    with (session / '.collect.lock').open('a') as lock:
        try:
            fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
        except BlockingIOError as exc:
            raise RuntimeError(
                'Another collector is using this session'
            ) from exc
        samples = load_samples(session)
        fitting = fitting_count is not None
        plan = (
            fitting_plan(session, samples, fitting_count, validation_count)
            if fitting
            else load_plan(session, samples)
        )
        role = FITTING_ROLE if fitting else VALIDATION_ROLE

        def finish(state, message, **details):
            result = {
                'state': state,
                'message': message,
                'role': role,
                'candidate_count': len(load_samples(session)),
                'training_candidate_count': len(plan['training_candidate_ids']),
                'validation_candidate_count': len(
                    plan['validation_candidate_ids']
                ),
                'validation_target_count': plan['validation_target_count'],
                'preview_policy': 'never_restore',
                **details,
            }
            if fitting:
                result['training_target_count'] = fitting_count
            result['rejected_count'] = sum(
                json.loads(p.read_text()).get('status') == 'rejected'
                for p in session.glob('*/sample.json')
            )
            atomic_json(session / 'latest_progress.json', result)
            return result

        if (
            not fitting
            and len(plan['validation_candidate_ids'])
            >= plan['validation_target_count']
        ):
            return finish(
                'complete',
                '独立验证候选集已达到计划数量，未再打开相机或新增样本。',
            )
        if fitting and len(plan['training_candidate_ids']) >= fitting_count:
            # 上一次在登记最后一组之后、冻结名单之前中断：补上冻结即可。
            freeze_plan(session, plan)
            return finish(
                'complete',
                '求解样本已凑够，分组名单已冻结；接下来去掉 --fitting '
                '采集验证样本。未打开相机或新增样本。',
            )
        # 求解样本阶段的第一组还没有可比对的基准，由它自己成为基准。
        reference = (
            samples[plan['training_candidate_ids'][0]]
            if plan['training_candidate_ids']
            else None
        )
        robot = capture.Robot(robot_ip)
        tool = robot.query('get_current_tool_frame')['reply']
        work = robot.query('get_current_work_frame')['reply']
        geometry.zero_reference_frames(tool, work)
        if reference is not None and (
            tool != reference['preflight']['get_current_tool_frame']
            or work != reference['preflight']['get_current_work_frame']
        ):
            raise ValueError(
                'Active tool/work configuration differs from fitting pool'
            )
        before = robot.query('get_current_arm_state')['reply']['arm_state']
        if 'err' not in before or any(before['err']):
            raise ValueError('Robot error present or status missing')
        initial = comparisons(geometry.pose_matrix(before['pose']), samples)
        duplicate = [x for x in initial if x['near_duplicate']]
        if duplicate:
            return finish(
                'duplicate',
                '当前姿态接近已有样本，跳过采集。',
                duplicates=duplicate,
            )
        # 只有目标相机被占用时才去暂停预览：预览服务读的是别的相机时，
        # 没有必要打断它。
        camera = capture.select_camera(serial)
        preview = (
            pause_preview()
            if capture.camera_users(camera)
            else 'camera_free'
        )
        time.sleep(settle_seconds)
        directory, sample = run_capture(session, robot_ip, camera['serial'])
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
            if reference is not None:
                check_compatibility(reference, sample)
            elif len(sample.get('corners_px', [])) != 88:
                raise ValueError('Expected 88 corners')
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
            'role': role,
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
        if fitting:
            plan['training_candidate_ids'].append(sample['sample_id'])
            count = len(plan['training_candidate_ids'])
            if count >= fitting_count:
                freeze_plan(session, plan)
                message = (
                    f'已保存第 {count}/{fitting_count} 组求解样本，求解样本'
                    '已凑够，分组名单已冻结；接下来去掉 --fitting 采集验证'
                    '样本。'
                )
            else:
                message = f'已保存第 {count}/{fitting_count} 组求解样本。'
        else:
            plan['validation_candidate_ids'].append(sample['sample_id'])
            atomic_json(session / 'dataset_split.json', plan)
            message = '已保存一组不重复的独立验证候选样本。'
        return finish(
            'captured',
            message,
            directory=str(directory),
            sample_id=sample['sample_id'],
            corners=len(sample['corners_px']),
            reprojection_rms_px=sample[
                'board_pose_candidates'
            ][0]['reprojection_rms_px'],
            stationarity=sample['stationarity'],
            preview_before_capture=preview,
        )


def main() -> int:
    """解析命令行并采集一组样本；只读查询机械臂，不发送任何指令。

    Returns:
        进程退出码：0 表示已登记一组或验证样本已满，2 表示本次跳过或
        拒收，1 表示运行出错。
    """
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--session', required=True)
    parser.add_argument('--robot-ip', default='192.168.1.19')
    parser.add_argument(
        '--serial',
        help='要采集的 D435i 的序列号；只连着一台时可以不写',
    )
    parser.add_argument(
        '--fitting',
        action='store_true',
        help='求解样本阶段：新会话从这里开始，凑够后自动冻结分组名单',
    )
    parser.add_argument(
        '--fitting-count',
        type=int,
        default=20,
        help='求解样本的目标组数，只在 --fitting 时使用',
    )
    parser.add_argument(
        '--validation-count',
        type=int,
        default=5,
        help='验证样本的目标组数，冻结名单时写入',
    )
    args = parser.parse_args()
    if args.fitting_count < 3 or args.validation_count < 1:
        parser.error('求解样本至少 3 组，验证样本至少 1 组')
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
    session = capture.ROOT / 'work' / 'handeye' / args.session
    if args.fitting:
        session.mkdir(parents=True, exist_ok=True)
    elif not (session / 'dataset_split.json').is_file():
        parser.error(
            '这个会话还没有冻结的分组名单 dataset_split.json；'
            '新会话请先加 --fitting 采集求解样本'
        )
    try:
        result = collect_once(
            session,
            args.robot_ip,
            serial=args.serial,
            fitting_count=args.fitting_count if args.fitting else None,
            validation_count=args.validation_count,
        )
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
