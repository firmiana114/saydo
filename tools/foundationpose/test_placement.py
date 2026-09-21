"""Placement geometry, persistence and motion sequencing with simulated hardware only."""
import copy
import fcntl
import json
import os
from pathlib import Path
import tempfile
import unittest
from types import SimpleNamespace
from unittest.mock import Mock, patch

import numpy as np

from grasp_control import GraspController
from grasp_geometry import controller_pose
from placement_control import settings_revision
from motion_validation import check_waypoints
from placement_geometry import (DEFAULTS, placement_settings, plan_placement, object_vertices,
                                sample_linear_path, check_footprint)


def fixture():
    config = {'robot_ip': '127.0.0.1', 'robot_port': 9, 'ik_checker': 'checker',
              'workspace_reviewed': True, 'workspace_min_m': [-1, -1, 0], 'workspace_max_m': [1, 1, 1],
              'table_z_m': -.02449, 'min_table_clearance_m': .01,
              'gripper': {'closed_position': 0, 'contact_position_margin': 20, 'open_position': 250}}
    settings = {**DEFAULTS, 'region_xy_m': {'x_min': .3, 'x_max': .6, 'y_min': -.2, 'y_max': .2}}
    attachment = np.eye(4);attachment[:3, 3] = [.01, 0, .07];attachment[:3, :3] = np.diag([1, -1, -1])
    held = {'hold_id': 'hold1', 'object_id': 'cup', 'object_name': '测试杯', 'model_sha256': 'mesh1',
            'orientation_version': 0, 'object_T_tcp': attachment.tolist(), 'contact_position': 500,
            'tool_response': {}}
    vertices = [[x, y, z] for x in [-.03, .03] for y in [-.02, .02] for z in [0, .1]]
    arm = {'pose': [200000, 0, 220000, 3142, 0, 0], 'joint': [0] * 7, 'err': [0]}
    return config, settings, held, vertices, arm


class PlacementGeometryTests(unittest.TestCase):
    def test_height_uses_actual_rotated_mesh_and_tcp_offset(self):
        config, settings, held, vertices, arm = fixture()
        config['workspace_min_m'][2] = -.02  # Permit the lower TCP needed by the horizontal pose.
        for angles in [[3142, 0, 0], [1571, 0, 300], [0, 0, 0]]:
            with self.subTest(angles=angles):
                arm['pose'][3:] = angles
                plan = plan_placement(held, vertices, arm, config, settings, Mock(return_value=(True, {})), lambda: None)
                release = next(w for w in plan['waypoints'] if w['name'] == 'lower')
                world = object_vertices(controller_pose(release['controller_pose']), held, vertices)
                self.assertAlmostEqual(world[:, 2].min(), config['table_z_m'] + .003, delta=1.01e-6)
                np.testing.assert_allclose(np.asarray(plan['base_T_object_target'])[:3, :3],
                    (controller_pose(arm['pose']) @ np.linalg.inv(held['object_T_tcp']))[:3, :3])
                self.assertEqual(release['controller_pose'][3:], arm['pose'][3:])
                check_footprint(world, settings)
                self.assertFalse(plan['contact_confirmed'])
                self.assertEqual(plan['release_position'], 1000)

    def test_search_constrains_entire_object_and_checks_midpoints(self):
        config, settings, held, vertices, arm = fixture();calls = []
        def reachable(points, timeout):
            calls.append(points)
            return len(calls) > 1, {'reason': 'first candidate blocked'}
        plan = plan_placement(held, vertices, arm, config, settings, reachable, lambda: None)
        self.assertEqual(plan['search_attempt_count'], 2)
        self.assertGreater(len(calls[1]), len(plan['waypoints']))
        self.assertTrue(all(w['method'] == ('joint_pose' if w['name'] in ('raise', 'transfer') else 'linear') for w in plan['waypoints']))
        self.assertFalse(any(w['name'].startswith(('raise:', 'transfer:')) for w in calls[1]))
        self.assertEqual(plan['motion_commands']['transfer'], 'movej_p')
        self.assertEqual(plan['motion_commands']['lower'], 'movel')
        self.assertEqual(plan['motion_commands']['retreat'], 'movel')
        self.assertTrue(all(w['controller_pose'][3:] == arm['pose'][3:] for w in calls[1]))
        self.assertEqual(plan['waypoints'][-1]['name'], 'retreat')
        self.assertIn('lower', [w['name'] for w in plan['waypoints']])
        self.assertGreaterEqual(plan['footprint_min_xy_m'][0], .3 - 1e-8)
        self.assertLessEqual(plan['footprint_max_xy_m'][0], .6 + 1e-8)

    def test_input_rectangle_is_not_shrunk_even_with_legacy_margin(self):
        settings = placement_settings({**DEFAULTS, 'edge_margin_m': .01,
            'region_xy_m': {'x_min': .3, 'x_max': 1., 'y_min': -1., 'y_max': -.3}})
        self.assertNotIn('edge_margin_m', settings)
        # Recorded post-transfer footprint: 9.943 mm inside the operator's upper Y boundary.
        world = np.array([[.44989748, -.42517306, .1], [.56222221, -.3099425773, .1]])
        check_footprint(world, settings)
        world[1, 1] = -.3
        check_footprint(world, settings)
        world[1, 1] = -.2999
        with self.assertRaisesRegex(ValueError, '超出输入'):
            check_footprint(world, settings)

    def test_small_region_missing_region_invalid_inputs_and_workspace(self):
        config, settings, held, vertices, arm = fixture();checker = Mock(return_value=(True, {}))
        for key, value in [('release_gap_m', -1), ('release_gap_m', True), ('release_gap_m', float('nan')),
                           ('descent_speed_percent', 100), ('region_xy_m', None)]:
            with self.subTest(key=key, value=value), self.assertRaises(ValueError):
                placement_settings({**settings, key: value})
        for region in ({'x_min': 1, 'x_max': 0, 'y_min': 0, 'y_max': 1},
                       {'x_min': True, 'x_max': 1, 'y_min': 0, 'y_max': 1}):
            with self.assertRaises(ValueError):placement_settings({**settings, 'region_xy_m': region})
        with self.assertRaisesRegex(ValueError, '无法完整'):
            plan_placement(held, vertices, arm, config,
                {**settings, 'region_xy_m': {'x_min': .3, 'x_max': .32, 'y_min': 0, 'y_max': .02}}, checker, lambda: None)
        with self.assertRaisesRegex(ValueError, '工作区'):
            plan_placement(held, vertices, arm, {**config, 'workspace_max_m': [1, 1, .1]}, settings, checker, lambda: None)
        checker.assert_not_called()

    def test_no_reachable_path_cancel_and_deadline_never_return_a_plan(self):
        config, settings, held, vertices, arm = fixture()
        settings['region_xy_m'] = {'x_min': .3, 'x_max': .39, 'y_min': -.04, 'y_max': .04}
        with self.assertRaisesRegex(ValueError, '未找到'):
            plan_placement(held, vertices, arm, config, settings, Mock(return_value=(False, {'reason': 'unreachable'})), lambda: None)
        checker = Mock()
        with self.assertRaisesRegex(ValueError, 'cancel'):
            plan_placement(held, vertices, arm, config, settings, checker, Mock(side_effect=ValueError('cancel')))
        checker.assert_not_called()
        with patch('placement_geometry.time.monotonic', side_effect=[0, 100]), self.assertRaises(TimeoutError):
            plan_placement(held, vertices, arm, config, settings, checker, lambda: None)

    def test_small_height_triggers_raise_and_gap_is_adjustable(self):
        config, settings, held, vertices, arm = fixture();arm['pose'][2] = 70000
        for gap in [0, .003, .01]:
            plan = plan_placement(held, vertices, arm, config, {**settings, 'release_gap_m': gap}, Mock(return_value=(True, {})), lambda: None)
            self.assertEqual(plan['waypoints'][0]['name'], 'raise')
            self.assertAlmostEqual(plan['object_bottom_z_m'], config['table_z_m'] + gap, delta=1.01e-6)

    @unittest.skipUnless(os.environ.get('SAYDO_IK_CHECKER'), 'Set SAYDO_IK_CHECKER for offline native placement verification')
    def test_native_sdk_finds_sampled_placement_from_historical_arm_pose(self):
        config, settings, held, _, _ = fixture()
        arm = {'joint': [-43683, -71382, 763, 132968, -2415, 67928, 120559],
               'pose': [135983, -147300, 209720, 2315, -356, 440], 'err': [0]}
        attachment = np.eye(4);attachment[:3, :3] = controller_pose(arm['pose'])[:3, :3]
        attachment[2, 3] = .22121;held['object_T_tcp'] = attachment.tolist()
        vertices = [[x, y, z] for x in [-.02, .02] for y in [-.02, .02] for z in [0, .24]]
        settings['region_xy_m'] = {'x_min': .05, 'x_max': .23, 'y_min': -.24, 'y_max': -.06}
        tool = np.eye(4);tool[2, 3] = .1905
        plan = plan_placement(held, vertices, arm, config, settings,
            lambda points, timeout: check_waypoints(os.environ['SAYDO_IK_CHECKER'],
                {'flange_T_grip_tcp': tool.tolist()}, arm, points, timeout), lambda: None)
        self.assertTrue(plan['reachability_check']['verified'])
        self.assertGreater(len(plan['validation_points']), len(plan['waypoints']))
        self.assertAlmostEqual(plan['object_bottom_z_m'], -.02149, delta=1.01e-6)


class PlacementControlTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory();self.addCleanup(self.temp.cleanup)
        self.root = Path(self.temp.name)
        self.config, self.settings, self.held, self.vertices, self.arm = fixture()
        self.path = self.root / 'trial.json';self.path.write_text(json.dumps(self.config))
        (self.root / 'config/grasp').mkdir(parents=True)
        (self.root / 'config/grasp/placement.json').write_text(json.dumps(self.settings))
        (self.root / 'checker').touch()
        self.service = GraspController(self.root, SimpleNamespace(models_root=self.root), self.path)
        self.service.held_object = copy.deepcopy(self.held)

    def request(self):
        return {**{k: self.held[k] for k in ('hold_id', 'object_id', 'model_sha256', 'orientation_version')},
                'revision': settings_revision(self.settings)}

    def robot(self, failure=None):
        robot = Mock();self.opened = False;self.commands = []
        def read_arm():return copy.deepcopy(self.arm)
        def move(method, pose, speed, cancelled, watchdog):
            self.commands.append((method, pose.copy()))
            if failure == 'motion':raise TimeoutError('controller timeout')
            watchdog()
            self.arm['pose'] = pose.copy()
            if failure == 'arrival':self.arm['pose'][0] += 20000
            if failure == 'below_table':self.arm['pose'][2] = 1000
            if failure == 'cancel':self.service.cancelled.set()
            if failure == 'slip':self.service.check_reorient_contact.side_effect = ValueError('slip')
            watchdog()
        def open_gripper(position):
            self.commands.append(('open', position))
            if failure == 'release':raise RuntimeError('release rejected')
            self.opened = True
            if failure == 'cancel_after_open':self.service.cancelled.set()
        def query(command):
            state = {'sys_err': 0, 'sys_state': 0, 'dof_err': [0], 'dof_state': [2],
                     'pos': [1000 if self.opened else 500], 'speed': [0], 'force': [0]}
            if failure == 'release_feedback' and self.opened:state['sys_err'] = 1
            return {'state_info': state}
        robot.move_linear.side_effect = lambda *args: move('linear', *args)
        robot.move_joint_pose.side_effect = lambda *args: move('joint', *args)
        robot.move_gripper.side_effect = open_gripper;robot.query.side_effect = query
        self.service.arm_state = Mock(side_effect=read_arm)
        self.service.check_reorient_contact = Mock()
        self.service.verify_motion_targets = Mock()
        self.service.reorientation_hardware = Mock(side_effect=lambda *_: (read_arm(), {'flange_T_grip_tcp': np.eye(4).tolist()}))
        return robot

    def run_job(self, robot, preview=False):
        self.service.status.update(busy=True, operation='place')
        with patch('placement_control.RobotClient', return_value=robot) as client, \
             patch('placement_control.model_geometry', return_value={'vertices': self.vertices, 'model_sha256': 'mesh1'}), \
             patch('placement_control.check_waypoints', return_value=(True, {})):
            self.service.run_placement(copy.deepcopy(self.held), self.settings, preview)
        return client

    def test_preview_reads_only_and_keeps_hold(self):
        robot = self.robot();client = self.run_job(robot, preview=True)
        self.assertEqual(self.service.status['state'], 'placement_preview')
        self.assertFalse(client.call_args.kwargs['writes'])
        self.assertEqual(self.service.held_object['hold_id'], 'hold1')
        robot.move_linear.assert_not_called();robot.move_joint_pose.assert_not_called();robot.move_gripper.assert_not_called();robot.stop.assert_not_called()
        self.service.mode = 'preview';robot = self.robot();self.run_job(robot)
        robot.move_linear.assert_not_called();robot.move_joint_pose.assert_not_called();robot.move_gripper.assert_not_called()

    def test_success_uses_joint_transfer_then_linear_lower_and_retreat(self):
        robot = self.robot();self.run_job(robot)
        self.assertEqual(self.service.status['state'], 'completed', self.service.status.get('error'))
        plan = self.service.status['placement_plan']
        self.assertEqual([x[0] for x in self.commands], ['joint', 'linear', 'open', 'linear'])
        self.assertEqual(self.commands[-2], ('open', 1000))
        self.assertEqual(self.commands[-1][1], plan['waypoints'][-1]['controller_pose'])
        self.assertIsNone(self.service.held_object)
        robot.move_joint_pose.assert_called_once();robot.move_joints.assert_not_called();robot.set_register.assert_not_called()
        report = json.loads(next((self.root / 'work/grasp/placement_jobs').glob('*.json')).read_text())
        self.assertTrue(report['release_confirmed']);self.assertFalse(report['contact_confirmed'])
        self.assertTrue(all(check['settled'] for check in report['arrival_checks']))
        self.assertFalse(report['result']['busy'])

    def test_raise_and_transfer_both_use_joint_pose(self):
        self.arm['pose'][2] = 70000
        robot = self.robot();self.run_job(robot)
        self.assertEqual(self.service.status['state'], 'completed', self.service.status.get('error'))
        self.assertEqual([x[0] for x in self.commands], ['joint', 'joint', 'linear', 'open', 'linear'])
        self.assertEqual(robot.move_joint_pose.call_count, 2)
        self.assertEqual(robot.move_linear.call_count, 2)
        checks = self.service.verify_motion_targets.call_args_list
        self.assertEqual([w['name'] for w in checks[0].args[-1]], ['raise'])
        self.assertEqual([w['name'] for w in checks[1].args[-1]], ['transfer'])
        self.assertTrue(all(w['name'].startswith('lower:') for w in checks[2].args[-1]))

    def test_joint_path_can_curve_and_change_orientation_then_arrive_correctly(self):
        robot = self.robot();original = robot.move_joint_pose.side_effect
        def curve(pose, speed, cancelled, watchdog):
            start = list(self.arm['pose'])
            self.arm['pose'][0] += 30000;self.arm['pose'][2] += 30000
            self.arm['pose'][5] += 300
            watchdog()  # Intermediate Cartesian curve and rotation are valid for movej_p.
            self.arm['pose'] = start
            return original(pose, speed, cancelled, watchdog)
        robot.move_joint_pose.side_effect = curve
        self.run_job(robot)
        self.assertEqual(self.service.status['state'], 'completed', self.service.status.get('error'))
        robot.move_gripper.assert_called_once_with(1000)

    def test_linear_descent_still_rejects_curves_and_rotation(self):
        for index, offset in [(0, 30000), (5, 300)]:
            with self.subTest(index=index):
                self.arm = fixture()[-1];self.service.actuated = False
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
                self.arm = fixture()[-1];self.service.cancelled.clear();self.service.actuated = False
                self.service.held_object = copy.deepcopy(self.held)
                robot = self.robot(failure);self.run_job(robot)
                self.assertIn(self.service.status['state'], ('failed', 'cancelled'))
                robot.move_gripper.assert_not_called();robot.stop.assert_called()

    def test_release_failure_or_cancel_prevents_retreat_and_clears_hold(self):
        for failure in ['release', 'release_feedback', 'cancel_after_open']:
            with self.subTest(failure=failure):
                self.arm = fixture()[-1];self.service.cancelled.clear();self.service.actuated = False
                self.service.held_object = copy.deepcopy(self.held)
                robot = self.robot(failure);self.run_job(robot)
                self.assertIn(self.service.status['state'], ('failed', 'cancelled'))
                self.assertEqual(self.commands[-1], ('open', 1000))
                self.assertIsNone(self.service.held_object);robot.stop.assert_called()

    def test_no_hold_stale_identity_version_settings_and_busy_rejected(self):
        with patch('placement_control.threading.Thread') as thread:
            for key in ('hold_id', 'object_id', 'model_sha256', 'orientation_version', 'revision'):
                with self.subTest(key=key), self.assertRaises(ValueError):
                    self.service.start_placement({**self.request(), key: 'stale'})
            self.service.held_object = None
            with self.assertRaisesRegex(ValueError, '夹持记录'):self.service.start_placement(self.request())
            self.service.held_object = self.held;self.service.status['busy'] = True
            with self.assertRaisesRegex(ValueError, '正在处理'):self.service.start_placement(self.request())
            thread.assert_not_called()
            self.service.status['busy'] = False
            self.service.start_placement(self.request(), preview=True)
            self.assertTrue(self.service.status['busy'])
            with self.assertRaises(ValueError):self.service.start_reset()

    def test_settings_save_is_atomic_validated_and_never_connects(self):
        original = self.path.read_bytes();value = {'revision': settings_revision(self.settings),
            'region_xy_m': {'x_min': .2, 'x_max': .5, 'y_min': -.1, 'y_max': .1}, 'release_gap_m': .005}
        with patch('placement_control.RobotClient') as client:
            result = self.service.save_placement_config(value)
            client.assert_not_called()
        self.assertFalse(result['motion_sent']);self.assertEqual(self.path.read_bytes(), original)
        saved = json.loads(self.service.placement_path.read_text())
        self.assertEqual(saved['release_gap_m'], .005)
        self.assertEqual(saved['descent_speed_percent'], self.settings['descent_speed_percent'])
        before = self.service.placement_path.read_bytes()
        with self.assertRaisesRegex(ValueError, '变化'):self.service.save_placement_config(value)
        value['revision'] = result['revision']
        with patch.object(Path, 'replace', side_effect=OSError('disk failure')), self.assertRaises(OSError):
            self.service.save_placement_config(value)
        self.assertEqual(before, self.service.placement_path.read_bytes())
        self.assertFalse(list(self.service.placement_path.parent.glob('.placement-*')))
        self.service.status['busy'] = True
        with self.assertRaises(ValueError):self.service.save_placement_config(value)

    def test_device_lock_and_model_mismatch_block_before_motion(self):
        lock_path = self.root / 'work/grasp/robot.lock';lock_path.parent.mkdir(parents=True)
        with lock_path.open('a') as lock:
            fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
            with patch('placement_control.RobotClient') as client:
                self.service.run_placement(self.held, self.settings)
                client.assert_not_called()
        robot = self.robot()
        with patch('placement_control.RobotClient', return_value=robot), \
             patch('placement_control.model_geometry', return_value={'model_sha256': 'changed'}):
            self.service.run_placement(self.held, self.settings)
        self.assertIn('模型已变化', self.service.status['error'])
        robot.move_linear.assert_not_called();robot.move_joint_pose.assert_not_called();robot.move_gripper.assert_not_called()

    def test_external_config_change_before_motion_blocks(self):
        robot = self.robot()
        def changed(*_):
            self.path.write_text(json.dumps({**self.config, 'table_z_m': 0}))
            return True, {}
        with patch('placement_control.RobotClient', return_value=robot), \
             patch('placement_control.model_geometry', return_value={'vertices': self.vertices, 'model_sha256': 'mesh1'}), \
             patch('placement_control.check_waypoints', side_effect=changed):
            self.service.run_placement(self.held, self.settings)
        robot.move_linear.assert_not_called();robot.move_joint_pose.assert_not_called();robot.move_gripper.assert_not_called()
        self.assertIn('配置在规划后变化', self.service.status['error'])


if __name__ == '__main__':
    unittest.main()
