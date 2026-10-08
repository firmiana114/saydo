"""Fail-closed IK/FK checks and bounded stable TCP arrival feedback."""

import copy
import subprocess
import time
import numpy as np
from grasp_geometry import controller_pose, pose_values, rotation_error

POSITION_TOLERANCE_M = .002
ORIENTATION_TOLERANCE_RAD = np.deg2rad(2)


def si_pose(values):
    """Use the existing rotation convention without quantizing the FK Euler angles."""
    values = np.asarray(values, dtype=float)
    if values.shape != (6,) or not np.isfinite(values).all():
        raise ValueError('正解位姿格式无效')
    return controller_pose(values * np.array([1e6] * 3 + [1e3] * 3))


def parse_kinematics(result, arm, waypoints):
    detail = {
        'returncode': result.returncode,
        'stdout': result.stdout[-4096:],
        'stderr': result.stderr[-512:],
        'fk_checks': [],
        'verified': False,
        'seed_search': [],
    }

    def location(index, kind):
        if 0 <= index < len(waypoints):
            detail['failure'] = {
                'kind': kind,
                'sample_index': index,
                'waypoint': waypoints[index].get('name', str(index)),
                'controller_pose': waypoints[index]['controller_pose'],
            }

    def fail(reason):
        detail['reason'] = reason
        return False, detail

    lines = [line.split() for line in result.stdout.splitlines()]
    if ['PROTOCOL', 'IK_FK_V1'] not in lines:
        return fail('离线检查器缺少正解回验协议，请重新构建 grasp_ik.cpp')
    ik = [row for row in lines if row and row[0] == 'IK']
    fk = [row for row in lines if row and row[0] == 'FK']
    verified = [row for row in lines if row and row[0] == 'VERIFY']
    try:
        detail['seed_search'] = [
            {
                'sample_index': int(row[1]),
                'attempts': int(row[2]),
                'valid_candidates': int(row[3]),
            }
            for row in lines
            if len(row) == 4 and row[0] == 'SEEDS'
        ]
        if len(fk) != 1 or len(fk[0]) != 7:
            return fail('缺少有效的起始正解反馈')
        initial = si_pose([float(x) for x in fk[0][1:]])
        current = controller_pose(arm['pose'])
        initial_distance = float(
            np.linalg.norm(initial[:3, 3] - current[:3, 3])
        )
        initial_angle = rotation_error(initial[:3, :3], current[:3, :3])
        detail['initial_fk_error'] = {
            'position_mm': initial_distance * 1000,
            'orientation_deg': float(np.rad2deg(initial_angle)),
        }
        if (
            initial_distance > POSITION_TOLERANCE_M
            or initial_angle > ORIENTATION_TOLERANCE_RAD
        ):
            return fail('离线模型/工具与当前机械臂反馈不一致，不能进行逆解回验')
        detail['candidate_rejections'] = [
            dict(
                zip(
                    (
                        'sample_index',
                        'ik',
                        'limits',
                        'self_collision',
                        'fk',
                        'continuity',
                    ),
                    map(int, row[1:]),
                )
            )
            for row in lines
            if len(row) == 7 and row[0] == 'REJECT'
        ]
        detail['beam_widths'] = [
            {'sample_index': int(row[1]), 'candidates': int(row[2])}
            for row in lines
            if len(row) == 3 and row[0] == 'BEAM'
        ]
        failures = [
            row for row in lines if len(row) == 4 and row[0] == 'SEARCH_FAIL'
        ]
        if failures:
            index = int(failures[-1][1])
            location(index, failures[-1][2])
            detail['search_ik_calls'] = int(failures[-1][3])
            return fail(
                f"{detail.get('failure',{}).get('waypoint','未知阶段')}："
                f"{failures[-1][2]}，多分支搜索未获得完整连续路径"
            )
        seen = set()
        for row in verified:
            if len(row) != 8:
                return fail('正解回验数据长度错误')
            index = int(row[1])
            if index in seen or not 0 <= index < len(waypoints):
                return fail('正解回验索引重复或越界')
            seen.add(index)
            actual = si_pose([float(x) for x in row[2:]])
            target = controller_pose(waypoints[index]['controller_pose'])
            distance = float(np.linalg.norm(actual[:3, 3] - target[:3, 3]))
            angle = rotation_error(actual[:3, :3], target[:3, :3])
            name = waypoints[index].get('name', str(index))
            check = {
                'index': index,
                'name': name,
                'position_error_mm': distance * 1000,
                'orientation_error_deg': float(np.rad2deg(angle)),
            }
            detail['fk_checks'].append(check)
            if (
                distance > POSITION_TOLERANCE_M
                or angle > ORIENTATION_TOLERANCE_RAD
            ):
                location(index, 'fk_residual')
                label = {
                    'pregrasp': '预抓点',
                    'grasp': '夹持点',
                    'lift': '抬升点',
                    'reorient': '姿态目标',
                }.get(name, name)
                return fail(
                    f'{label}：逆解返回成功，但正解位置误差 '
                    f'{distance*1000:.2f} 毫米、朝向误差 '
                    f'{np.rad2deg(angle):.2f}°，超出 2 毫米/2°'
                )
        if result.returncode != 0:
            index = max(0, len(ik) - 1)
            kind = (
                'ik_failed'
                if ik and ik[-1][1] != '0'
                else 'self_collision'
                if ik and ik[-1][2] != '0'
                else 'checker_failed'
            )
            location(index, kind)
            return fail(
                (
                    f"{detail.get('failure',{}).get('waypoint','未知阶段')}："
                    f"{kind}；"
                )
                + (str(ik[-1][:3]) if ik else '没有逆解结果')
            )
        if len(ik) != len(waypoints) or seen != set(range(len(waypoints))):
            return fail('逆解或正解回验结果不完整')
        if any(len(row) != 10 or row[1:3] != ['0', '0'] for row in ik):
            return fail('逆解失败或检测到自身碰撞')
        solutions = np.asarray([[float(x) for x in row[3:]] for row in ik])
        if not np.isfinite(solutions).all():
            return fail('逆解关节角不是有限数值')
        differences = np.abs(
            np.diff(
                np.vstack([np.asarray(arm['joint']) / 1000, solutions]), axis=0
            )
        )
        detail.update(
            joint_solutions_deg=solutions.tolist(),
            max_joint_step_deg=float(differences.max()),
            joint_travel_deg=float(differences.sum()),
        )
        # Cartesian samples are at most 10 mm / 5 degrees apart. A large
        # branch change is not a valid continuous movel path, even if FK passes.
        for index, waypoint in enumerate(waypoints):
            if (
                waypoint.get('method') == 'linear'
                and differences[index].max() > 10.
            ):
                joint = int(np.argmax(differences[index]))
                location(index, 'joint_discontinuity')
                detail['continuity_failure'] = {
                    'sample_index': index,
                    'joint_index': joint + 1,
                    'delta_deg': float(differences[index, joint]),
                    'limit_deg': 10.,
                }
                return fail(
                    f'直线采样关节解不连续：采样{index+1}关节{joint+1}变化{differences[index,joint]:.2f}°，超过10°；请更换接近路点或姿态，不能直接执行该直线'
                )
        detail.update(
            verified=True,
            linear_joint_continuity_checked=any(
                w.get('method') == 'linear' for w in waypoints
            ),
        )
        return True, detail
    except (ValueError, TypeError, IndexError, KeyError) as error:
        return fail('离线正解回验数据无效：' + str(error))


def note_joint_change(report, stage, basis, reference, actual, limit_deg=.5):
    """Record that a redundant arm reached, or re-solved to, a joint set other than the earlier reference."""
    deviation = (
        float(
            np.max(
                np.abs(
                    np.asarray(actual, dtype=float)
                    - np.asarray(reference, dtype=float)
                )
            )
        )
        / 1000
    )
    if deviation > limit_deg:
        report.setdefault('joint_configuration_changes', []).append(
            {
                'stage': stage,
                'basis': basis,
                'reference_joint_raw': [int(round(v)) for v in reference],
                'actual_joint_raw': [int(round(v)) for v in actual],
                'max_deviation_deg': deviation,
            }
        )
    return deviation


def check_waypoints(executable, tool, arm, waypoints, timeout=3.):
    if not waypoints:
        raise ValueError('至少需要一个待验证航点')
    joints = np.asarray(arm['joint'], dtype=float)
    if joints.shape != (7,) or not np.isfinite(joints).all():
        raise ValueError('起始关节角无效')
    values = [
        *pose_values(tool['flange_T_grip_tcp']),
        *(joints / 1000),
        len(waypoints),
        max(1, min(2800, int(timeout * 850))),
    ]
    for waypoint in waypoints:
        values.extend(pose_values(controller_pose(waypoint['controller_pose'])))
        values.append(10 if waypoint.get('method') == 'linear' else 360)
    result = subprocess.run(
        [str(executable), '--beam-ik-v1'],
        input=' '.join(str(x) for x in values) + '\n',
        text=True,
        capture_output=True,
        timeout=timeout,
    )
    if 'CAPABILITY BEAM_IK_V1' not in result.stdout.splitlines():
        return (
            False,
            {
                'verified': False,
                'reason': (
                    '离线检查器尚未支持多分支路径搜索协议，请重新构建，'
                    '不发送运动'
                ),
            },
        )
    return parse_kinematics(result, arm, waypoints)


def wait_for_pose(
    read_state,
    target_raw,
    cancelled,
    record,
    position_tolerance=POSITION_TOLERANCE_M,
    orientation_tolerance=ORIENTATION_TOLERANCE_RAD,
    clock=time.monotonic,
    sleep=time.sleep,
):
    """Require three stable in-tolerance samples; never retry a gross pose error."""
    target = controller_pose(target_raw)
    started = clock()
    deadline = started + .8
    stable = 0
    stable_since = None
    previous = None
    record.update(target_pose=list(target_raw), samples=[], settled=False)
    while True:
        cancelled()
        state = read_state()
        cancelled()
        actual = controller_pose(state['pose'])
        now = clock()
        distance = float(np.linalg.norm(actual[:3, 3] - target[:3, 3]))
        angle = rotation_error(actual[:3, :3], target[:3, :3])
        record['actual_arm_state'] = copy.deepcopy(state)
        record['samples'].append(
            {
                'elapsed_s': now - started,
                'pose': list(state['pose']),
                'position_error_mm': distance * 1000,
                'orientation_error_deg': float(np.rad2deg(angle)),
            }
        )
        if distance > .010:
            raise ValueError(
                f'机械臂实际位置与目标不符：偏差 {distance*1000:.2f} 毫米，'
                f'超过 10 毫米立即停止界限'
            )
        if angle > np.deg2rad(10):
            raise ValueError(
                f'机械臂实际朝向与目标不符：偏差 {np.rad2deg(angle):.2f}°，'
                f'超过 10° 立即停止界限'
            )
        good = distance <= position_tolerance and angle <= orientation_tolerance
        steady = (
            previous is None
            or (
                np.linalg.norm(actual[:3, 3] - previous[:3, 3]) <= .0005
                and (
                    rotation_error(actual[:3, :3], previous[:3, :3])
                    <= np.deg2rad(.5)
                )
            )
        )
        if good:
            if stable and steady:
                stable += 1
            else:
                stable = 1
                stable_since = now
        else:
            stable = 0
            stable_since = None
        if now <= deadline and stable >= 3 and now - stable_since >= .099:
            record.update(settled=True, settle_seconds=now - started)
            return state
        if now >= deadline or len(record['samples']) >= 20:
            if distance > position_tolerance:
                raise ValueError(
                    f'机械臂实际位置与目标不符：等待反馈稳定后仍偏差 '
                    f'{distance*1000:.2f} 毫米（要求 '
                    f'≤{position_tolerance*1000:g} 毫米）'
                )
            if angle > orientation_tolerance:
                raise ValueError(
                    f'机械臂实际朝向与目标不符：等待反馈稳定后仍偏差 '
                    f'{np.rad2deg(angle):.2f}°'
                )
            raise ValueError('机械臂到位反馈未形成连续稳定窗口，停止后续动作')
        previous = actual
        sleep(min(.05, max(0., deadline - clock())))
