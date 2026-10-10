"""TCP arrival and IK->FK regression tests; no robot connections."""

import copy
import json
import os
import pathlib
import tempfile
import types
import unittest
from unittest import mock

import numpy as np
from saydo.manipulation import grasp_control
from saydo.robot import collision_model
from saydo.robot import motion_validation
from tests.manipulation import test_grasp


NATIVE_TOOL = np.eye(4)
NATIVE_TOOL[2, 3] = .1905


def protocol(pose='0 0 0 0 0 0', index=0):
    return (
        f'PROTOCOL IK_FK_V1\nFK 0 0 0 0 0 0\nIK 0 0 0 0 0 0 0 0 0\nVERIFY '
        f'{index} {pose}\nRESIDUAL {index} 0 0\n'
    )


class Clock:
    def __init__(self):
        self.t = 0.
        self.sleeps = []

    def now(self):
        return self.t

    def sleep(self, seconds):
        self.sleeps.append(seconds)
        self.t += seconds


class MotionValidationTests(unittest.TestCase):
    def setUp(self):
        self.arm = {'joint': [0] * 7, 'pose': [0] * 6, 'err': [0]}
        self.points = [{'name': 'pregrasp', 'controller_pose': [0] * 6}]

    def parse(self, stdout, code=0):
        return motion_validation.parse_kinematics(
            types.SimpleNamespace(stdout=stdout, stderr='', returncode=code),
            self.arm,
            self.points,
        )

    def test_failure_identifies_stage_sample_and_error_type(self):
        data = protocol().replace('IK 0 0', 'IK 1 -1').split('VERIFY')[0]
        ok, d = self.parse(data, 1)
        self.assertFalse(ok)
        self.assertEqual(d['failure']['kind'], 'ik_failed')
        self.assertEqual(d['failure']['waypoint'], 'pregrasp')
        ok, d = self.parse(protocol('.2 0 0 0 0 0'), 3)
        self.assertEqual(d['failure']['kind'], 'fk_residual')
        self.assertEqual(d['failure']['sample_index'], 0)

    def test_seed_diagnostics_and_missing_binary_capability(self):
        ok, d = self.parse(protocol() + 'SEEDS 0 31 2\n')
        self.assertTrue(ok)
        self.assertEqual(d['seed_search'][0]['valid_candidates'], 2)
        with mock.patch(
            'saydo.robot.motion_validation.subprocess.run',
            return_value=types.SimpleNamespace(
                stdout=protocol(), stderr='', returncode=0
            ),
        ):
            ok, d = motion_validation.check_waypoints(
                'fixture',
                {'flange_T_grip_tcp': np.eye(4).tolist()},
                self.arm,
                self.points,
            )
        self.assertFalse(ok)
        self.assertIn('缺少 ARM_ANGLE_V2', d['reason'])

    def test_checker_receives_the_arm_shape_and_obstacles(self):
        environment = collision_model.Environment(
            table_z_m=-.02, pole_xy_m=(.1, -.2), pole_radius_m=.014,
            pole_margin_m=.03,
        )
        tool = np.eye(4)
        tool[2, 3] = .2
        for given in (environment, None):
            with mock.patch(
                'saydo.robot.motion_validation.subprocess.run',
                return_value=types.SimpleNamespace(
                    stdout='', stderr='', returncode=2
                ),
            ) as run:
                motion_validation.check_waypoints(
                    'fixture',
                    {'flange_T_grip_tcp': tool.tolist()},
                    self.arm,
                    self.points,
                    environment=given,
                )
            self.assertEqual(
                run.call_args.args[0], ['fixture', '--arm-angle-v2']
            )
            sent = run.call_args.kwargs['input'].split()
            model = collision_model.native_model_values(tool, given)
            # 工具 6 个数、关节 7 个、路点数与预算 2 个、每个路点 7 个，
            # 其后就是外形和障碍物。
            self.assertEqual(len(sent), 6 + 7 + 2 + 7 + len(model))
            np.testing.assert_allclose(
                [float(value) for value in sent[-len(model):]], model
            )
        # 没有障碍物时两个开关都是 0，另一台臂的胶囊体个数也是 0，
        # 检查器不做碰撞判断。
        flags = 1 + 7 * 3 + 12
        self.assertEqual((model[flags], model[flags + 2]), (0, 0))
        self.assertEqual(model[-2:], [0., 0])

    def test_checker_receives_the_other_arm(self):
        environment = collision_model.Environment(
            other_arm_capsules=(
                np.array([[.1, -.5, .2], [.2, -.4, .3]]),
                np.array([[.1, -.5, .4], [.3, -.4, .3]]),
                np.array([.05, .04]),
            ),
            other_arm_margin_m=.03,
        )
        model = collision_model.native_model_values(np.eye(4), environment)
        # 末尾依次是：余量、个数、每个胶囊体的起点、终点、半径。
        self.assertEqual(
            model[-16:],
            [
                .03, 2,
                .1, -.5, .2, .1, -.5, .4, .05,
                .2, -.4, .3, .3, -.4, .3, .04,
            ],
        )

    def test_search_failure_kinds_are_explained_to_the_operator(self):
        cases = {
            'out_of_reach': '任何摆法下都够不到或超出关节限位',
            'blocked_by_obstacle': '所有摆法都会碰到桌面、立杆或另一台臂',
            'no_continuous_solution': '从前面的路径走不过去',
            'budget_exhausted': '超出时间预算',
        }
        for kind, text in cases.items():
            with self.subTest(kind=kind):
                ok, d = self.parse(
                    protocol()
                    + 'SEEDS 0 120 40\nREJECT 0 1 2 3 4 5 6\n'
                    + f'SEARCH_FAIL 0 {kind} 900\n',
                    1,
                )
                self.assertFalse(ok)
                self.assertEqual(d['failure']['kind'], kind)
                self.assertIn(text, d['reason'])
                self.assertTrue(d['reason'].startswith('pregrasp：'))
                self.assertEqual(
                    d['candidate_rejections'],
                    [
                        {
                            'sample_index': 0,
                            'ik': 1,
                            'limits': 2,
                            'self_collision': 3,
                            'fk': 4,
                            'continuity': 5,
                            'obstacle': 6,
                        }
                    ],
                )

    def test_arm_angles_are_recorded_when_complete(self):
        ok, d = self.parse(protocol() + 'ARM_ANGLE 0 -79.5\n')
        self.assertTrue(ok)
        self.assertEqual(d['arm_angles_deg'], [-79.5])
        ok, d = self.parse(protocol())
        self.assertTrue(ok)
        self.assertNotIn('arm_angles_deg', d)

    def test_beam_failure_never_accepts_valid_prefix(self):
        data = (
            protocol() + 'BEAM 0 2\nSEARCH_FAIL 0 no_continuous_solution 62\n'
        )
        ok, d = self.parse(data, 1)
        self.assertFalse(ok)
        self.assertFalse(d['verified'])
        self.assertEqual(d['failure']['kind'], 'no_continuous_solution')
        self.assertEqual(d['search_ik_calls'], 62)

    def test_verified_success(self):
        ok, detail = self.parse(protocol())
        self.assertTrue(ok)
        self.assertTrue(detail['verified'])
        self.assertEqual(detail['fk_checks'][0]['position_error_mm'], 0)

    def test_linear_branch_jump_rejected_but_joint_endpoint_allowed(self):
        data = protocol().replace(
            'IK 0 0 0 0 0 0 0 0 0', 'IK 0 0 38.3 0 0 0 0 0 0'
        )
        self.points[0]['method'] = 'linear'
        ok, detail = self.parse(data)
        self.assertFalse(ok)
        self.assertFalse(detail['verified'])
        self.assertEqual(detail['continuity_failure']['joint_index'], 1)
        self.points[0]['method'] = 'joint_pose'
        self.assertTrue(self.parse(data)[0])
        self.points[0]['method'] = 'linear'
        self.assertTrue(self.parse(data.replace('38.3', '9.9'))[0])

    def test_ik_success_and_false_residual_cannot_hide_wrong_fk(self):
        for pose in ['.16 0 0 0 0 0', '0 0 0 0 0 .1']:
            ok, detail = self.parse(protocol(pose))
            self.assertFalse(ok)
            self.assertIn('逆解返回成功', detail['reason'])

    def test_rejects_legacy_missing_duplicate_and_nonfinite_proof(self):
        samples = [
            protocol().replace('PROTOCOL IK_FK_V1\n', ''),
            protocol().replace('VERIFY 0 0 0 0 0 0 0\n', ''),
            protocol() + 'VERIFY 0 0 0 0 0 0 0\n',
            protocol(index=1),
            protocol('nan 0 0 0 0 0'),
            protocol().replace('IK 0 0', 'IK 0 1'),
            protocol().replace(
                'IK 0 0 0 0 0 0 0 0 0', 'IK 0 0 nan 0 0 0 0 0 0'
            ),
        ]
        for data in samples:
            with self.subTest(data=data):
                self.assertFalse(self.parse(data)[0])

    def test_initial_fk_must_match_controller_feedback(self):
        ok, detail = self.parse(
            protocol().replace('FK 0 0 0 0 0 0', 'FK .01 0 0 0 0 0')
        )
        self.assertFalse(ok)
        self.assertIn('当前机械臂反馈不一致', detail['reason'])

    def test_bad_fk_diagnostic_survives_nonzero_exit_and_incomplete_path(self):
        self.points.append({'name': 'grasp', 'controller_pose': [0] * 6})
        ok, detail = self.parse(protocol('.2 0 0 0 0 0'), 3)
        self.assertFalse(ok)
        self.assertIn('200.00', detail['reason'])

    def test_valid_residual_still_requires_success_exit(self):
        self.assertFalse(self.parse(protocol(), 1)[0])

    def test_joint_continuity_retained_after_fk_validation(self):
        ok, detail = self.parse(
            protocol().replace('IK 0 0 0 0 0 0 0 0 0', 'IK 0 0 30 0 0 0 0 0 0')
        )
        self.assertTrue(ok)
        self.assertEqual(detail['max_joint_step_deg'], 30)

    def wait(self, poses):
        clock = Clock()
        record = {}
        rows = iter(poses)
        last = [poses[-1]]

        def read():
            last[0] = next(rows, last[0])
            return {'pose': last[0], 'joint': [0] * 7, 'err': [0]}

        read = mock.Mock(side_effect=read)
        return clock, record, read

    def test_small_transient_settles_without_loosening_final_tolerance(self):
        clock, record, read = self.wait(
            [
                [0, 0, 5690, 0, 0, 0],
                [0, 0, 2080, 0, 0, 0],
                [0, 0, 900, 0, 0, 0],
                [0, 0, 850, 0, 0, 0],
                [0, 0, 810, 0, 0, 0],
            ]
        )
        result = motion_validation.wait_for_pose(
            read,
            [0] * 6,
            lambda: None,
            record,
            clock=clock.now,
            sleep=clock.sleep,
        )
        self.assertTrue(record['settled'])
        self.assertEqual(result['pose'][2], 810)
        self.assertEqual(len(record['samples']), 5)
        self.assertAlmostEqual(clock.t, .2)

    def test_large_error_fails_on_first_read(self):
        clock, record, read = self.wait([[160000, 0, 0, 0, 0, 0]])
        with self.assertRaisesRegex(ValueError, '立即停止'):
            motion_validation.wait_for_pose(
                read,
                [0] * 6,
                lambda: None,
                record,
                clock=clock.now,
                sleep=clock.sleep,
            )
        self.assertEqual(read.call_count, 1)
        self.assertEqual(clock.sleeps, [])

    def test_persistent_millimetre_error_is_not_accepted(self):
        clock, record, read = self.wait([[0, 0, 2080, 0, 0, 0]])
        with self.assertRaisesRegex(ValueError, '位置与目标不符'):
            motion_validation.wait_for_pose(
                read,
                [0] * 6,
                lambda: None,
                record,
                clock=clock.now,
                sleep=clock.sleep,
            )
        self.assertFalse(record['settled'])
        self.assertLessEqual(len(record['samples']), 20)

    def test_in_tolerance_but_oscillating_feedback_is_rejected(self):
        clock, record, read = self.wait(
            [[(-1) ** i * 1800, 0, 0, 0, 0, 0] for i in range(25)]
        )
        with self.assertRaisesRegex(ValueError, '稳定窗口'):
            motion_validation.wait_for_pose(
                read,
                [0] * 6,
                lambda: None,
                record,
                clock=clock.now,
                sleep=clock.sleep,
            )

    def test_orientation_grace_and_gross_rejection(self):
        for angle, word in [(100, '朝向'), (300, '立即停止')]:
            clock, record, read = self.wait([[0, 0, 0, angle, 0, 0]])
            with self.assertRaisesRegex(ValueError, word):
                motion_validation.wait_for_pose(
                    read,
                    [0] * 6,
                    lambda: None,
                    record,
                    clock=clock.now,
                    sleep=clock.sleep,
                )
            if angle == 300:
                self.assertEqual(read.call_count, 1)

    def test_cancel_during_settling_does_not_read_again(self):
        clock, record, read = self.wait([[0, 0, 1000, 0, 0, 0]])

        def cancel():
            if clock.t >= .05:
                raise ValueError('cancel')

        with self.assertRaisesRegex(ValueError, 'cancel'):
            motion_validation.wait_for_pose(
                read,
                [0] * 6,
                cancel,
                record,
                clock=clock.now,
                sleep=clock.sleep,
            )
        self.assertEqual(read.call_count, 1)

    def test_fk_failure_at_stage_never_sends_arm_command(self):
        with tempfile.TemporaryDirectory() as directory:
            root = pathlib.Path(directory)
            config = test_grasp.GraspTests().config()
            config['ik_checker'] = 'checker'
            (root / 'checker').touch()
            cfg = root / 'config.json'
            cfg.write_text(json.dumps(config))
            service = grasp_control.GraspController(root, None, cfg)
            service.robot = mock.Mock()
            report = {}
            plan = {'flange_T_grip_tcp': np.eye(4).tolist()}
            with mock.patch(
                'saydo.robot.motion_validation.check_waypoints',
                return_value=(False, {'reason': '正解误差 160 毫米'}),
            ):
                with self.assertRaisesRegex(ValueError, '正解误差'):
                    service.verify_motion_targets(
                        plan, config, report, self.arm, self.points
                    )
            service.robot.move_joint_pose.assert_not_called()
            service.robot.move_linear.assert_not_called()
            service.robot.move_gripper.assert_not_called()
            self.assertEqual(
                report['kinematics_checks'][0]['reason'], '正解误差 160 毫米'
            )

    @unittest.skipUnless(
        os.environ.get('SAYDO_IK_CHECKER'),
        'Set SAYDO_IK_CHECKER to the newly built offline verifier',
    )
    def test_native_historical_false_successes_and_valid_target(self):
        tool = np.eye(4)
        tool[2, 3] = .1905
        cases = [
            (
                [-43681, -71383, 763, 132967, -2416, 67925, 120557],
                [135992, -147311, 209749, 2315, -356, 440],
                [458760, -317868, 90712, 2323, -393, 2414],
                False,
            ),
            (
                [-84386, -71385, 90103, 59959, 20346, 94939, 200354],
                [300868, 139500, 154484, 2276, -385, 301],
                [463924, -61884, 55889, 2218, -432, -811],
                False,
            ),
            (
                [-84387, -71385, 90104, 59962, 20348, 94941, 200352],
                [300853, 139481, 154464, 2276, -385, 301],
                [463300, -60614, 54824, 2189, -415, -762],
                False,
            ),
            (
                [-43683, -71382, 763, 132968, -2415, 67928, 120559],
                [135983, -147300, 209720, 2315, -356, 440],
                [363798, -267227, 6124, 2205, -117, 463],
                True,
            ),
        ]
        for joints, pose, target, expected in cases:
            with self.subTest(target=target):
                valid, detail = motion_validation.check_waypoints(
                    os.environ['SAYDO_IK_CHECKER'],
                    {'flange_T_grip_tcp': tool.tolist()},
                    {'joint': joints, 'pose': pose},
                    [{'name': 'pregrasp', 'controller_pose': target}],
                )
                self.assertEqual(valid, expected, detail)
                if not expected:
                    self.assertFalse(detail['verified'])
                    # 这几个目标都是太远够不到：任何摆法都算不出真正到达
                    # 它的关节角，不能报成“有可行摆法但走不过去”。
                    self.assertEqual(
                        detail['failure']['kind'], 'out_of_reach'
                    )

    @unittest.skipUnless(
        os.environ.get('SAYDO_IK_CHECKER'),
        'Set SAYDO_IK_CHECKER to the newly built offline verifier',
    )
    def test_native_staying_in_place_keeps_the_joint_angles(self):
        joints = [-43683, -71382, 763, 132968, -2415, 67928, 120559]
        pose = [135983, -147300, 209720, 2315, -356, 440]
        valid, detail = motion_validation.check_waypoints(
            os.environ['SAYDO_IK_CHECKER'],
            {'flange_T_grip_tcp': NATIVE_TOOL.tolist()},
            {'joint': joints, 'pose': pose},
            [{'name': 'stay', 'controller_pose': pose, 'method': 'linear'}],
        )
        self.assertTrue(valid, detail)
        # 目标就是当前位姿：按当前臂角求解，应当回到当前的关节角。位姿
        # 读数取整到 0.001 弧度，换算回关节角会差零点几度。
        np.testing.assert_allclose(
            detail['joint_solutions_deg'][0], np.array(joints) / 1000,
            atol=.5,
        )
        self.assertEqual(len(detail['arm_angles_deg']), 1)

    @unittest.skipUnless(
        os.environ.get('SAYDO_IK_CHECKER'),
        'Set SAYDO_IK_CHECKER to the newly built offline verifier',
    )
    def test_native_elbow_moves_aside_for_a_pole(self):
        joints = [-43683, -71382, 763, 132968, -2415, 67928, 120559]
        arm = {
            'joint': joints,
            'pose': [135983, -147300, 209720, 2315, -356, 440],
        }
        tool = {'flange_T_grip_tcp': NATIVE_TOOL.tolist()}
        points = [
            {
                'name': 'pregrasp',
                'controller_pose': [363798, -267227, 6124, 2205, -117, 463],
            }
        ]
        checker = os.environ['SAYDO_IK_CHECKER']
        valid, free = motion_validation.check_waypoints(
            checker, tool, arm, points
        )
        self.assertTrue(valid, free)
        # 不考虑障碍物时手肘所在的位置旁边 10 厘米处立一根杆：原来的
        # 摆法离它太近，但手肘可以向另一侧转开。（杆正好立在手肘的位置
        # 时挡住的是整条上臂，转动手肘躲不开。）
        frames = collision_model.link_frames(free['joint_solutions_deg'][0])
        shoulder, elbow, wrist = (frames[i][:2, 3] for i in (2, 4, 6))
        along = (wrist - shoulder) / np.linalg.norm(wrist - shoulder)
        pole = elbow + .10 * np.array([-along[1], along[0]])
        blocked = collision_model.Environment(
            pole_xy_m=(float(pole[0]), float(pole[1])),
            pole_radius_m=.0141,
            pole_margin_m=.03,
        )
        report = collision_model.check_path(
            [np.array(joints) / 1000, free['joint_solutions_deg'][0]],
            NATIVE_TOOL,
            blocked,
        )
        self.assertIsNotNone(report['violation'])
        valid, detail = motion_validation.check_waypoints(
            checker, tool, arm, points, environment=blocked
        )
        self.assertTrue(valid, detail)
        # 换了一种手肘摆法，全程离杆不少于要求的 30 毫米。
        self.assertIsNone(detail['collision_check']['violation'])
        self.assertGreaterEqual(
            detail['collision_check']['min_pole_clearance_m'], .03
        )
        self.assertGreater(
            abs(detail['arm_angles_deg'][0] - free['arm_angles_deg'][0]), 4.
        )
        # 杆立在目标位置上时，任何摆法都躲不开。
        target = np.array(points[0]['controller_pose'][:2]) / 1e6
        valid, detail = motion_validation.check_waypoints(
            checker,
            tool,
            arm,
            points,
            environment=collision_model.Environment(
                pole_xy_m=(float(target[0]), float(target[1])),
                pole_radius_m=.0141,
                pole_margin_m=.03,
            ),
        )
        self.assertFalse(valid)
        self.assertEqual(detail['failure']['kind'], 'blocked_by_obstacle')

    @unittest.skipUnless(
        os.environ.get('SAYDO_IK_CHECKER'),
        'Set SAYDO_IK_CHECKER to the newly built offline verifier',
    )
    def test_native_elbow_moves_aside_for_the_other_arm(self):
        joints = [-43683, -71382, 763, 132968, -2415, 67928, 120559]
        arm = {
            'joint': joints,
            'pose': [135983, -147300, 209720, 2315, -356, 440],
        }
        tool = {'flange_T_grip_tcp': NATIVE_TOOL.tolist()}
        points = [
            {
                'name': 'pregrasp',
                'controller_pose': [363798, -267227, 6124, 2205, -117, 463],
            }
        ]
        checker = os.environ['SAYDO_IK_CHECKER']
        valid, free = motion_validation.check_waypoints(
            checker, tool, arm, points
        )
        self.assertTrue(valid, free)
        # 另一台臂的一段（一个竖直的胶囊体）立在本臂手肘旁边 12 厘米处。
        frames = collision_model.link_frames(free['joint_solutions_deg'][0])
        shoulder, elbow, wrist = (frames[i][:3, 3] for i in (2, 4, 6))
        along = (wrist - shoulder)[:2] / np.linalg.norm((wrist - shoulder)[:2])
        beside = elbow[:2] + .12 * np.array([-along[1], along[0]])
        other = collision_model.Environment(
            other_arm_capsules=(
                np.array([[beside[0], beside[1], elbow[2] - .1]]),
                np.array([[beside[0], beside[1], elbow[2] + .1]]),
                np.array([.04]),
            ),
            other_arm_margin_m=.03,
            other_arm_label='2 臂',
        )
        report = collision_model.check_path(
            [np.array(joints) / 1000, free['joint_solutions_deg'][0]],
            NATIVE_TOOL,
            other,
        )
        self.assertIn('2 臂', report['violation']['message'])
        valid, detail = motion_validation.check_waypoints(
            checker, tool, arm, points, environment=other
        )
        self.assertTrue(valid, detail)
        self.assertIsNone(detail['collision_check']['violation'])
        self.assertGreaterEqual(
            detail['collision_check']['min_other_arm_clearance_m'], .03
        )
        self.assertGreater(
            abs(detail['arm_angles_deg'][0] - free['arm_angles_deg'][0]), 4.
        )
        # 它正好挡在目标位置上时，任何摆法都躲不开。
        target = np.array(points[0]['controller_pose'][:3]) / 1e6
        valid, detail = motion_validation.check_waypoints(
            checker,
            tool,
            arm,
            points,
            environment=collision_model.Environment(
                other_arm_capsules=(
                    np.array([target - [0, 0, .05]]),
                    np.array([target + [0, 0, .05]]),
                    np.array([.04]),
                ),
                other_arm_margin_m=.03,
            ),
        )
        self.assertFalse(valid)
        self.assertEqual(detail['failure']['kind'], 'blocked_by_obstacle')


if __name__ == '__main__':
    unittest.main()
