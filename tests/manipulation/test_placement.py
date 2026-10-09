"""Placement geometry, persistence and motion sequencing with simulated hardware only."""

import copy
import fcntl
import json
import os
import pathlib
import tempfile
import unittest
import types
from unittest import mock

import numpy as np

from saydo.manipulation import grasp_control
from saydo.geometry import grasp_geometry
from saydo.manipulation import placement_control
from saydo.robot import motion_validation
from saydo.manipulation import placement_geometry


def fixture():
    config = {
        'robot_ip': '127.0.0.1',
        'robot_port': 9,
        'ik_checker': 'checker',
        'workspace_reviewed': True,
        'workspace_min_m': [-1, -1, 0],
        'workspace_max_m': [1, 1, 1],
        'table_z_m': -.02449,
        'min_table_clearance_m': .01,
        'pregrasp_distance_m': .05,
        'gripper': {
            'closed_position': 0,
            'contact_position_margin': 20,
            'open_position': 250,
        },
    }
    settings = {
        **placement_geometry.DEFAULTS,
        'region_xy_m': {'x_min': .3, 'x_max': .6, 'y_min': -.2, 'y_max': .2},
    }
    attachment = np.eye(4)
    attachment[:3, 3] = [.01, 0, .07]
    attachment[:3, :3] = np.diag([1, -1, -1])
    held = {
        'hold_id': 'hold1',
        'object_id': 'cup',
        'object_name': '测试杯',
        'model_sha256': 'mesh1',
        'orientation_version': 0,
        'object_T_tcp': attachment.tolist(),
        'contact_position': 500,
        'tool_response': {},
    }
    vertices = [
        [x, y, z] for x in [-.03, .03] for y in [-.02, .02] for z in [0, .1]
    ]
    arm = {
        'pose': [200000, 0, 220000, 3142, 0, 0], 'joint': [0] * 7, 'err': [0]
    }
    return config, settings, held, vertices, arm


class PlacementGeometryTests(unittest.TestCase):
    def test_height_uses_actual_rotated_mesh_and_tcp_offset(self):
        config, settings, held, vertices, arm = fixture()
        # Keep the upside-down tool's withdrawal above the table.
        config['pregrasp_distance_m'] = .001
        # Permit the lower TCP needed by the horizontal pose.
        config['workspace_min_m'][2] = -.02
        for angles in [[3142, 0, 0], [1571, 0, 300], [0, 0, 0]]:
            with self.subTest(angles=angles):
                arm['pose'][3:] = angles
                plan = placement_geometry.plan_placement(
                    held,
                    vertices,
                    arm,
                    config,
                    settings,
                    mock.Mock(return_value=(True, {})),
                    lambda: None,
                )
                release = next(
                    w for w in plan['waypoints'] if w['name'] == 'lower'
                )
                world = placement_geometry.object_vertices(
                    grasp_geometry.controller_pose(release['controller_pose']),
                    held,
                    vertices,
                )
                self.assertAlmostEqual(
                    world[:, 2].min(), config['table_z_m'] + .003, delta=1.01e-6
                )
                np.testing.assert_allclose(
                    np.asarray(plan['base_T_object_target'])[:3, :3],
                    (
                        grasp_geometry.controller_pose(arm['pose'])
                        @ np.linalg.inv(held['object_T_tcp'])
                    )[:3, :3],
                )
                self.assertEqual(
                    release['controller_pose'][3:], arm['pose'][3:]
                )
                placement_geometry.check_footprint(world, settings)
                self.assertFalse(plan['contact_confirmed'])
                self.assertEqual(plan['release_position'], 1000)
                self.assertEqual(
                    plan['region_constraint_scope'], 'candidate_selection_only'
                )

    def test_search_constrains_entire_object_and_checks_midpoints(self):
        config, settings, held, vertices, arm = fixture()
        calls = []

        def reachable(points, timeout):
            calls.append(points)
            return len(calls) > 1, {'reason': 'first candidate blocked'}

        plan = placement_geometry.plan_placement(
            held, vertices, arm, config, settings, reachable, lambda: None
        )
        self.assertEqual(plan['search_attempt_count'], 2)
        self.assertGreater(len(calls[1]), len(plan['waypoints']))
        self.assertTrue(
            all(
                (
                    w['method']
                    == (
                        'joint_pose'
                        if w['name'] in ('raise', 'transfer')
                        else 'linear'
                    )
                )
                for w in plan['waypoints']
            )
        )
        self.assertFalse(
            any(w['name'].startswith(('raise:', 'transfer:')) for w in calls[1])
        )
        self.assertEqual(plan['motion_commands']['transfer'], 'movej_p')
        self.assertEqual(plan['motion_commands']['lower'], 'movel')
        self.assertEqual(plan['motion_commands']['retreat'], 'movel')
        self.assertTrue(
            all(w['controller_pose'][3:] == arm['pose'][3:] for w in calls[1])
        )
        self.assertEqual(plan['waypoints'][-1]['name'], 'retreat')
        self.assertIn('lower', [w['name'] for w in plan['waypoints']])
        self.assertGreaterEqual(plan['footprint_min_xy_m'][0], .3 - 1e-8)
        self.assertLessEqual(plan['footprint_max_xy_m'][0], .6 + 1e-8)

    def test_input_rectangle_is_not_shrunk_even_with_legacy_margin(self):
        settings = placement_geometry.placement_settings(
            {
                **placement_geometry.DEFAULTS,
                'edge_margin_m': .01,
                'region_xy_m': {
                    'x_min': .3, 'x_max': 1., 'y_min': -1., 'y_max': -.3
                },
            }
        )
        self.assertNotIn('edge_margin_m', settings)
        # Recorded post-transfer footprint: 9.943 mm inside the operator's upper
        # Y boundary.
        world = np.array(
            [[.44989748, -.42517306, .1], [.56222221, -.3099425773, .1]]
        )
        placement_geometry.check_footprint(world, settings)
        world[1, 1] = -.3
        placement_geometry.check_footprint(world, settings)
        world[1, 1] = -.2999
        with self.assertRaisesRegex(ValueError, '超出输入'):
            placement_geometry.check_footprint(world, settings)

    def test_small_region_missing_region_invalid_inputs_and_workspace(self):
        config, settings, held, vertices, arm = fixture()
        checker = mock.Mock(return_value=(True, {}))
        for key, value in [
            ('release_gap_m', -1),
            ('release_gap_m', .0601),
            ('release_gap_m', True),
            ('release_gap_m', float('nan')),
            ('descent_speed_percent', 100),
            ('region_xy_m', None),
        ]:
            with self.subTest(
                key=key, value=value
            ), self.assertRaises(ValueError):
                placement_geometry.placement_settings({**settings, key: value})
        for region in (
            {'x_min': 1, 'x_max': 0, 'y_min': 0, 'y_max': 1},
            {'x_min': True, 'x_max': 1, 'y_min': 0, 'y_max': 1},
        ):
            with self.assertRaises(ValueError):
                placement_geometry.placement_settings(
                    {**settings, 'region_xy_m': region}
                )
        with self.assertRaisesRegex(ValueError, '无法完整'):
            placement_geometry.plan_placement(
                held,
                vertices,
                arm,
                config,
                {
                    **settings,
                    'region_xy_m': {
                        'x_min': .3, 'x_max': .32, 'y_min': 0, 'y_max': .02
                    },
                },
                checker,
                lambda: None,
            )
        with self.assertRaisesRegex(ValueError, '工作区'):
            placement_geometry.plan_placement(
                held,
                vertices,
                arm,
                {**config, 'workspace_max_m': [1, 1, .1]},
                settings,
                checker,
                lambda: None,
            )
        checker.assert_not_called()

    def test_withdrawal_follows_tool_negative_z_and_pregrasp_distance(self):
        config, settings, held, vertices, arm = fixture()
        # Tilted tool: local -Z is not base +Z.
        arm['pose'][3:] = [2500, 200, 300]
        for distance in (.03, .08):
            with self.subTest(distance=distance):
                config['pregrasp_distance_m'] = distance
                plan = placement_geometry.plan_placement(
                    held,
                    vertices,
                    arm,
                    config,
                    settings,
                    mock.Mock(return_value=(True, {})),
                    lambda: None,
                )
                targets = {
                    w['name']: grasp_geometry.controller_pose(
                        w['controller_pose']
                    )
                    for w in plan['waypoints']
                }
                delta = targets['withdraw'][:3, 3] - targets['lower'][:3, 3]
                np.testing.assert_allclose(
                    delta, -targets['lower'][:3, 2] * distance, atol=1e-6
                )
                self.assertGreater(np.linalg.norm(delta[:2]), .01)
                np.testing.assert_allclose(
                    targets['retreat'][:3, 3] - targets['withdraw'][:3, 3],
                    [0, 0, settings['retreat_height_m']],
                    atol=1e-6,
                )
                self.assertEqual(plan['withdraw_distance_m'], distance)
                self.assertEqual(plan['motion_commands']['withdraw'], 'movel')
                self.assertTrue(
                    any(
                        w['name'].startswith('withdraw:')
                        for w in plan['validation_points']
                    )
                )

    def test_invalid_or_unreachable_withdrawal_blocks_entire_plan(self):
        config, settings, held, vertices, arm = fixture()
        checker = mock.Mock(return_value=(True, {}))
        for distance in (None, 0, -.01, True, .31, float('nan')):
            with self.subTest(distance=distance), self.assertRaises(ValueError):
                placement_geometry.plan_placement(
                    held,
                    vertices,
                    arm,
                    {**config, 'pregrasp_distance_m': distance},
                    settings,
                    checker,
                    lambda: None,
                )
        checker.assert_not_called()
        # A tip-up tool would retreat below the configured table at this release
        # height.
        arm['pose'][3:] = [0, 0, 0]
        with self.assertRaisesRegex(ValueError, '未找到'):
            placement_geometry.plan_placement(
                held, vertices, arm, config, settings, checker, lambda: None
            )
        checker.assert_not_called()

    def test_no_reachable_path_cancel_and_deadline_never_return_a_plan(self):
        config, settings, held, vertices, arm = fixture()
        settings['region_xy_m'] = {
            'x_min': .3, 'x_max': .39, 'y_min': -.04, 'y_max': .04
        }
        with self.assertRaisesRegex(ValueError, '未找到'):
            placement_geometry.plan_placement(
                held,
                vertices,
                arm,
                config,
                settings,
                mock.Mock(return_value=(False, {'reason': 'unreachable'})),
                lambda: None,
            )
        checker = mock.Mock()
        with self.assertRaisesRegex(ValueError, 'cancel'):
            placement_geometry.plan_placement(
                held,
                vertices,
                arm,
                config,
                settings,
                checker,
                mock.Mock(side_effect=ValueError('cancel')),
            )
        checker.assert_not_called()
        with mock.patch(
            'saydo.manipulation.placement_geometry.time.monotonic',
            side_effect=[0, 100],
        ), self.assertRaises(TimeoutError):
            placement_geometry.plan_placement(
                held, vertices, arm, config, settings, checker, lambda: None
            )

    def test_small_height_triggers_raise_and_gap_is_adjustable(self):
        config, settings, held, vertices, arm = fixture()
        arm['pose'][2] = 70000
        for gap in [0, .003, .02, .06]:
            plan = placement_geometry.plan_placement(
                held,
                vertices,
                arm,
                config,
                {**settings, 'release_gap_m': gap},
                mock.Mock(return_value=(True, {})),
                lambda: None,
            )
            self.assertEqual(plan['waypoints'][0]['name'], 'raise')
            self.assertAlmostEqual(
                plan['object_bottom_z_m'],
                config['table_z_m'] + gap,
                delta=1.01e-6,
            )

    @unittest.skipUnless(
        os.environ.get('SAYDO_IK_CHECKER'),
        'Set SAYDO_IK_CHECKER for offline native placement verification',
    )
    def test_native_sdk_finds_sampled_placement_from_historical_arm_pose(self):
        config, settings, held, _, _ = fixture()
        arm = {
            'joint': [-43683, -71382, 763, 132968, -2415, 67928, 120559],
            'pose': [135983, -147300, 209720, 2315, -356, 440],
            'err': [0],
        }
        attachment = np.eye(4)
        attachment[:3, :3] = grasp_geometry.controller_pose(arm['pose'])[:3, :3]
        attachment[2, 3] = .22121
        held['object_T_tcp'] = attachment.tolist()
        vertices = [
            [x, y, z]
            for x in [-.02, .02]
            for y in [-.02, .02]
            for z in [0, .24]
        ]
        settings['region_xy_m'] = {
            'x_min': .05, 'x_max': .23, 'y_min': -.24, 'y_max': -.06
        }
        tool = np.eye(4)
        tool[2, 3] = .1905
        plan = placement_geometry.plan_placement(
            held,
            vertices,
            arm,
            config,
            settings,
            lambda points, timeout: motion_validation.check_waypoints(
                os.environ['SAYDO_IK_CHECKER'],
                {'flange_T_grip_tcp': tool.tolist()},
                arm,
                points,
                timeout,
            ),
            lambda: None,
        )
        self.assertTrue(plan['reachability_check']['verified'])
        self.assertGreater(
            len(plan['validation_points']), len(plan['waypoints'])
        )
        self.assertAlmostEqual(
            plan['object_bottom_z_m'], -.02149, delta=1.01e-6
        )


class PlacementControlTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.addCleanup(self.temp.cleanup)
        self.root = pathlib.Path(self.temp.name)
        self.config, self.settings, self.held, self.vertices, self.arm = (
            fixture()
        )
        self.config.setdefault('arm_speed_percent', 25)
        self.settings.update(
            transfer_speed_percent=self.config['arm_speed_percent'],
            descent_speed_percent=self.config['arm_speed_percent'],
        )
        self.path = self.root / 'trial.json'
        self.path.write_text(json.dumps(self.config))
        (self.root / 'config/grasp').mkdir(parents=True)
        (
            self.root / 'config/grasp/placement.json'
        ).write_text(json.dumps(self.settings))
        (self.root / 'checker').touch()
        self.service = grasp_control.GraspController(
            self.root, types.SimpleNamespace(models_root=self.root), self.path
        )
        self.service.held_object = copy.deepcopy(self.held)

    def request(self):
        return {
            **{
                k: self.held[k]
                for k in (
                    'hold_id',
                    'object_id',
                    'model_sha256',
                    'orientation_version',
                )
            },
            'revision': placement_control.settings_revision(self.settings),
        }

    def robot(self, failure=None):
        robot = mock.Mock()
        self.opened = False
        self.commands = []

        def read_arm():
            return copy.deepcopy(self.arm)

        def move(method, pose, speed, cancelled, watchdog):
            self.commands.append((method, pose.copy()))
            if failure == 'motion':
                raise TimeoutError('controller timeout')
            watchdog()
            self.arm['pose'] = pose.copy()
            if failure == 'arrival':
                self.arm['pose'][0] += 20000
            if failure == 'below_table':
                self.arm['pose'][2] = 1000
            if failure == 'cancel':
                self.service.cancelled.set()
            if failure == 'slip':
                self.service.check_reorient_contact.side_effect = ValueError(
                    'slip'
                )
            watchdog()

        def open_gripper(position):
            self.commands.append(('open', position))
            if failure == 'release':
                raise RuntimeError('release rejected')
            self.opened = True
            if failure == 'cancel_after_open':
                self.service.cancelled.set()

        def query(command):
            state = {
                'sys_err': 0,
                'sys_state': 0,
                'dof_err': [0],
                'dof_state': [2],
                'pos': [1000 if self.opened else 500],
                'speed': [0],
                'force': [0],
            }
            if failure == 'release_feedback' and self.opened:
                state['sys_err'] = 1
            return {'state_info': state}

        robot.move_linear.side_effect = lambda *args: move('linear', *args)
        robot.move_joint_pose.side_effect = lambda *args: move('joint', *args)
        robot.move_gripper.side_effect = open_gripper
        robot.query.side_effect = query
        self.service.arm_state = mock.Mock(side_effect=read_arm)
        self.service.check_reorient_contact = mock.Mock()
        self.service.verify_motion_targets = mock.Mock()
        self.service.reorientation_hardware = mock.Mock(
            side_effect=lambda *_: (
                read_arm(), {'flange_T_grip_tcp': np.eye(4).tolist()}
            )
        )
        return robot

    def run_job(self, robot, preview=False):
        self.service.status.update(busy=True, operation='place')
        with mock.patch(
            'saydo.robot.grasp_robot.RobotClient',
            return_value=robot,
        ) as client, mock.patch(
            'saydo.manipulation.reorientation_geometry.model_geometry',
            return_value={'vertices': self.vertices, 'model_sha256': 'mesh1'},
        ), mock.patch(
            'saydo.robot.motion_validation.check_waypoints',
            return_value=(True, {}),
        ):
            self.service.run_placement(
                copy.deepcopy(self.held), self.settings, preview
            )
        return client

    def test_preview_reads_only_and_keeps_hold(self):
        robot = self.robot()
        client = self.run_job(robot, preview=True)
        self.assertEqual(self.service.status['state'], 'placement_preview')
        self.assertFalse(client.call_args.kwargs['writes'])
        self.assertEqual(self.service.held_object['hold_id'], 'hold1')
        robot.move_linear.assert_not_called()
        robot.move_joint_pose.assert_not_called()
        robot.move_gripper.assert_not_called()
        robot.stop.assert_not_called()
        self.service.mode = 'preview'
        robot = self.robot()
        self.run_job(robot)
        robot.move_linear.assert_not_called()
        robot.move_joint_pose.assert_not_called()
        robot.move_gripper.assert_not_called()

    def test_success_uses_joint_transfer_then_linear_lower_and_retreat(self):
        robot = self.robot()
        self.run_job(robot)
        self.assertEqual(
            self.service.status['state'],
            'completed',
            self.service.status.get('error'),
        )
        plan = self.service.status['placement_plan']
        self.assertEqual(
            [x[0] for x in self.commands],
            ['joint', 'joint', 'linear', 'open', 'linear', 'linear'],
        )
        self.assertEqual(self.commands[-3], ('open', 1000))
        self.assertEqual(
            self.commands[-1][1], plan['waypoints'][-1]['controller_pose']
        )
        self.assertIsNone(self.service.held_object)
        self.assertEqual(robot.move_joint_pose.call_count, 2)
        robot.move_joints.assert_not_called()
        robot.set_register.assert_not_called()
        report = json.loads(
            next(
                (self.root / 'work/grasp/placement_jobs').glob('*.json')
            ).read_text()
        )
        self.assertTrue(report['release_confirmed'])
        self.assertFalse(report['contact_confirmed'])
        self.assertTrue(
            all(check['settled'] for check in report['arrival_checks'])
        )
        self.assertFalse(report['result']['busy'])

    def test_rectangle_is_not_rechecked_during_motion_or_before_release(self):
        robot = self.robot()

        def feedback_outside_rectangle(original):
            def move(pose, speed, cancelled, watchdog):
                original(pose, speed, cancelled, watchdog)
                if not self.opened:
                    # 0.5 mm: within arrival tolerance, outside the selection
                    # rectangle.
                    self.arm['pose'][0] -= 500
                    watchdog()

            return move

        robot.move_joint_pose.side_effect = feedback_outside_rectangle(
            robot.move_joint_pose.side_effect
        )
        robot.move_linear.side_effect = feedback_outside_rectangle(
            robot.move_linear.side_effect
        )
        self.run_job(robot)
        self.assertEqual(
            self.service.status['state'],
            'completed',
            self.service.status.get('error'),
        )
        report = json.loads(
            next(
                (self.root / 'work/grasp/placement_jobs').glob('*.json')
            ).read_text()
        )
        target = next(
            w for w in report['plan']['waypoints'] if w['name'] == 'lower'
        )
        placement_geometry.check_footprint(
            placement_geometry.object_vertices(
                grasp_geometry.controller_pose(target['controller_pose']),
                self.held,
                self.vertices,
            ),
            self.settings,
        )
        arrival = report['arrival_checks'][1]['actual_arm_state']
        with self.assertRaisesRegex(ValueError, '超出输入'):
            placement_geometry.check_footprint(
                placement_geometry.object_vertices(
                    grasp_geometry.controller_pose(arrival['pose']),
                    self.held,
                    self.vertices,
                ),
                self.settings,
            )
        self.assertTrue(report['release_confirmed'])
        robot.move_gripper.assert_called_once_with(1000)
        robot.stop.assert_not_called()

    def test_raise_and_transfer_both_use_joint_pose(self):
        self.arm['pose'][2] = 70000
        robot = self.robot()
        self.run_job(robot)
        self.assertEqual(
            self.service.status['state'],
            'completed',
            self.service.status.get('error'),
        )
        self.assertEqual(
            [x[0] for x in self.commands],
            ['joint', 'joint', 'linear', 'open', 'linear', 'linear'],
        )
        self.assertEqual(robot.move_joint_pose.call_count, 2)
        self.assertEqual(robot.move_linear.call_count, 3)
        checks = self.service.verify_motion_targets.call_args_list
        self.assertEqual([w['name'] for w in checks[0].args[-1]], ['raise'])
        self.assertEqual([w['name'] for w in checks[1].args[-1]], ['transfer'])
        self.assertTrue(
            all(w['name'].startswith('lower:') for w in checks[2].args[-1])
        )

    def test_joint_path_can_curve_and_change_orientation_then_arrive_correctly(
        self
    ):
        robot = self.robot()
        original = robot.move_joint_pose.side_effect

        def curve(pose, speed, cancelled, watchdog):
            start = list(self.arm['pose'])
            self.arm['pose'][0] += 30000
            self.arm['pose'][2] += 30000
            self.arm['pose'][5] += 300
            # Intermediate Cartesian curve and rotation are valid for movej_p.
            watchdog()
            self.arm['pose'] = start
            return original(pose, speed, cancelled, watchdog)

        robot.move_joint_pose.side_effect = curve
        self.run_job(robot)
        self.assertEqual(
            self.service.status['state'],
            'completed',
            self.service.status.get('error'),
        )
        robot.move_gripper.assert_called_once_with(1000)

    def test_linear_descent_still_rejects_curves_and_rotation(self):
        for index, offset in [(0, 30000), (5, 300)]:
            with self.subTest(index=index):
                self.arm = fixture()[-1]
                self.service.actuated = False
                self.service.held_object = copy.deepcopy(self.held)
                robot = self.robot()

                def wrong_path(pose, speed, cancelled, watchdog):
                    self.arm['pose'][index] += offset
                    watchdog()

                robot.move_linear.side_effect = wrong_path
                self.run_job(robot)
                self.assertEqual(self.service.status['state'], 'failed')
                robot.move_gripper.assert_not_called()
                robot.stop.assert_called()

    def test_motion_arrival_cancel_and_slip_never_release(self):
        for failure in ['motion', 'arrival', 'below_table', 'cancel', 'slip']:
            with self.subTest(failure=failure):
                self.arm = fixture()[-1]
                self.service.cancelled.clear()
                self.service.actuated = False
                self.service.held_object = copy.deepcopy(self.held)
                robot = self.robot(failure)
                self.run_job(robot)
                self.assertIn(
                    self.service.status['state'], ('failed', 'cancelled')
                )
                robot.move_gripper.assert_not_called()
                robot.stop.assert_called()

    def test_release_failure_or_cancel_prevents_retreat_and_clears_hold(self):
        for failure in ['release', 'release_feedback', 'cancel_after_open']:
            with self.subTest(failure=failure):
                self.arm = fixture()[-1]
                self.service.cancelled.clear()
                self.service.actuated = False
                self.service.held_object = copy.deepcopy(self.held)
                robot = self.robot(failure)
                self.run_job(robot)
                self.assertIn(
                    self.service.status['state'], ('failed', 'cancelled')
                )
                self.assertEqual(self.commands[-1], ('open', 1000))
                self.assertIsNone(self.service.held_object)
                robot.stop.assert_called()

    def test_withdrawal_failure_after_release_prevents_vertical_exit(self):
        robot = self.robot()
        original = robot.move_linear.side_effect

        def fail_after_open(pose, speed, cancelled, watchdog):
            if self.opened:
                self.commands.append(('withdraw_failed', pose.copy()))
                raise TimeoutError('withdraw timeout')
            return original(pose, speed, cancelled, watchdog)

        robot.move_linear.side_effect = fail_after_open
        self.run_job(robot)
        self.assertEqual(self.service.status['state'], 'failed')
        self.assertEqual(
            [x[0] for x in self.commands],
            ['joint', 'joint', 'linear', 'open', 'withdraw_failed'],
        )
        self.assertIsNone(self.service.held_object)
        self.assertIn('grip_tcp', self.service.status['error'])
        robot.stop.assert_called()

    def test_withdrawal_uses_measured_arrival_tool_axis(self):
        # Start inside the region, with room for settling error.
        self.arm['pose'][0] = 450000
        robot = self.robot()
        original = robot.move_linear.side_effect

        def small_arrival_error(pose, speed, cancelled, watchdog):
            original(pose, speed, cancelled, watchdog)
            if not self.opened:
                self.arm['pose'][0] += 200
                self.arm['pose'][3] += 1

        robot.move_linear.side_effect = small_arrival_error
        self.run_job(robot)
        self.assertEqual(
            self.service.status['state'],
            'completed',
            self.service.status.get('error'),
        )
        report = json.loads(
            next(
                (self.root / 'work/grasp/placement_jobs').glob('*.json')
            ).read_text()
        )
        arrival = grasp_geometry.controller_pose(
            next(
                row
                for row in report['arrival_checks']
                if row['stage'] == '保持朝向下降至释放高度'
            )['actual_arm_state']['pose']
        )
        withdrawal = next(
            w for w in report['executed_waypoints'] if w['name'] == 'withdraw'
        )
        actual_target = grasp_geometry.controller_pose(
            withdrawal['controller_pose']
        )
        np.testing.assert_allclose(
            actual_target[:3, 3],
            (
                arrival[:3, 3]
                - arrival[:3, 2] * self.config['pregrasp_distance_m']
            ),
            atol=1e-6,
        )
        np.testing.assert_allclose(
            actual_target[:3, :3], arrival[:3, :3], atol=1e-10
        )

    def test_no_hold_stale_identity_version_settings_and_busy_rejected(self):
        with mock.patch(
            'saydo.manipulation.placement_control.threading.Thread'
        ) as thread:
            for key in (
                'hold_id',
                'object_id',
                'model_sha256',
                'orientation_version',
                'revision',
            ):
                with self.subTest(key=key), self.assertRaises(ValueError):
                    self.service.start_placement(
                        {**self.request(), key: 'stale'}
                    )
            self.service.held_object = None
            with self.assertRaisesRegex(ValueError, '夹持记录'):
                self.service.start_placement(self.request())
            self.service.held_object = self.held
            self.service.status['busy'] = True
            with self.assertRaisesRegex(ValueError, '正在处理'):
                self.service.start_placement(self.request())
            thread.assert_not_called()
            self.service.status['busy'] = False
            self.service.start_placement(self.request(), preview=True)
            self.assertTrue(self.service.status['busy'])
            with self.assertRaises(ValueError):
                self.service.start_reset()

    def test_settings_save_is_atomic_validated_and_never_connects(self):
        original = self.path.read_bytes()
        value = {
            'revision': placement_control.settings_revision(self.settings),
            'region_xy_m': {
                'x_min': .2, 'x_max': .5, 'y_min': -.1, 'y_max': .1
            },
            'release_gap_m': .06,
        }
        with mock.patch('saydo.robot.grasp_robot.RobotClient') as client:
            result = self.service.save_placement_config(value)
            client.assert_not_called()
        self.assertFalse(result['motion_sent'])
        self.assertEqual(self.path.read_bytes(), original)
        saved = json.loads(self.service.placement_path.read_text())
        self.assertEqual(saved['release_gap_m'], .06)
        self.assertEqual(
            saved['descent_speed_percent'],
            self.settings['descent_speed_percent'],
        )
        before = self.service.placement_path.read_bytes()
        with self.assertRaisesRegex(ValueError, '变化'):
            self.service.save_placement_config(value)
        value['revision'] = result['revision']
        with mock.patch.object(
            pathlib.Path, 'replace', side_effect=OSError('disk failure')
        ), self.assertRaises(OSError):
            self.service.save_placement_config(value)
        self.assertEqual(before, self.service.placement_path.read_bytes())
        self.assertFalse(
            list(self.service.placement_path.parent.glob('.placement-*'))
        )
        self.service.status['busy'] = True
        with self.assertRaises(ValueError):
            self.service.save_placement_config(value)

    def test_device_lock_and_model_mismatch_block_before_motion(self):
        lock_path = self.root / 'work/grasp/robot.lock'
        lock_path.parent.mkdir(parents=True)
        with lock_path.open('a') as lock:
            fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
            with mock.patch('saydo.robot.grasp_robot.RobotClient') as client:
                self.service.run_placement(self.held, self.settings)
                client.assert_not_called()
        robot = self.robot()
        with mock.patch(
            'saydo.robot.grasp_robot.RobotClient',
            return_value=robot,
        ), mock.patch(
            'saydo.manipulation.reorientation_geometry.model_geometry',
            return_value={'model_sha256': 'changed'},
        ):
            self.service.run_placement(self.held, self.settings)
        self.assertIn('模型已变化', self.service.status['error'])
        robot.move_linear.assert_not_called()
        robot.move_joint_pose.assert_not_called()
        robot.move_gripper.assert_not_called()

    def test_external_config_change_before_motion_blocks(self):
        robot = self.robot()

        def changed(*_, **unused):
            self.path.write_text(json.dumps({**self.config, 'table_z_m': 0}))
            return True, {}

        with mock.patch(
            'saydo.robot.grasp_robot.RobotClient',
            return_value=robot,
        ), mock.patch(
            'saydo.manipulation.reorientation_geometry.model_geometry',
            return_value={'vertices': self.vertices, 'model_sha256': 'mesh1'},
        ), mock.patch(
            'saydo.robot.motion_validation.check_waypoints',
            side_effect=changed,
        ):
            self.service.run_placement(self.held, self.settings)
        robot.move_linear.assert_not_called()
        robot.move_joint_pose.assert_not_called()
        robot.move_gripper.assert_not_called()
        self.assertIn('配置在规划后变化', self.service.status['error'])


if __name__ == '__main__':
    unittest.main()
