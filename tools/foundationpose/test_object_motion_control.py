import copy
import hashlib
import json
from pathlib import Path
import tempfile
import time
from types import SimpleNamespace
import unittest
from unittest.mock import Mock, patch
import numpy as np
from grasp_control import GraspController
from grasp_geometry import controller_pose
import test_grasp
from test_object_motion import fixture


class ObjectControlTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.addCleanup(self.temp.cleanup)
        self.root = Path(self.temp.name)
        self.held, self.vertices, self.arm, base = fixture()
        self.config = test_grasp.GraspTests().config()
        self.config.update(
            base,
            tool_file='tool.yaml',
            handeye_file='handeye.yaml',
            ik_checker='checker',
        )
        self.path = self.root / 'trial.json'
        self.path.write_text(json.dumps(self.config))
        (self.root / 'tool.yaml').write_text('{}')
        (self.root / 'handeye.yaml').write_text('{}')
        (self.root / 'checker').touch()
        mesh = self.root / 'mesh.obj'
        mesh.write_text('fixture mesh')
        self.held['model_sha256'] = hashlib.sha256(
            mesh.read_bytes()
        ).hexdigest()
        self.tracker = SimpleNamespace(
            models_root=self.root,
            state={'object_id': 'sample', 'camera_frame_at': time.time()},
            generation=1,
        )
        self.service = GraspController(self.root, self.tracker, self.path)
        self.service.held_object = copy.deepcopy(self.held)
        self.geometry = {
            'object_id': 'sample',
            'model_sha256': self.held['model_sha256'],
            'pose_mode': '6d',
            'vertices': self.vertices.tolist(),
        }
        self.robot = Mock()
        self.opened = False
        self.fail = None
        self.joint_targets = {}
        self.counter = 0
        self.calls = []

        def query(command):
            if command == 'get_current_arm_state':
                return {'arm_state': copy.deepcopy(self.arm)}
            if command == 'get_rm_plus_state_info':
                return {
                    'state_info': {
                        'sys_state': 0,
                        'sys_err': 0,
                        'dof_err': [0],
                        'dof_state': [2 if self.opened else 3],
                        'pos': [1000 if self.opened else 500],
                        'speed': [0],
                        'force': [0],
                    }
                }
            raise AssertionError(command)

        self.robot.query.side_effect = query

        def linear(pose, speed, cancelled, watchdog):
            self.calls.append(('movel', pose, speed))
            watchdog()
            if self.fail == 'move':
                raise RuntimeError('motion failed')
            self.arm['pose'] = list(pose)
            watchdog()

        def joints(values, speed, cancelled, watchdog):
            self.calls.append(('movej', values, speed))
            watchdog()
            if self.fail == 'move':
                raise RuntimeError('motion failed')
            self.arm['joint'] = list(values)
            self.arm['pose'] = self.joint_targets[tuple(values)]
            watchdog()

        def open_gripper(position):
            self.calls.append(('open', position))
            if self.fail == 'open':
                raise RuntimeError('open unconfirmed')
            self.opened = True

        self.robot.move_linear.side_effect = linear
        self.robot.move_joints.side_effect = joints
        self.robot.move_gripper.side_effect = open_gripper

        def context(held, writes=False):
            config = json.loads(self.path.read_text())
            self.service.robot = self.robot
            return (
                config,
                copy.deepcopy(self.arm),
                {'flange_T_grip_tcp': np.eye(4).tolist()},
                self.geometry,
                self.root / 'checker',
            )

        self.service.motion_context = Mock(side_effect=context)

        def verify(plan, config, report, arm, points, timeout=3):
            solutions = []
            for point in points:
                self.counter += 1
                raw = [self.counter * 1000, 0, 0, 0, 0, 0, 0]
                self.joint_targets[tuple(raw)] = point['controller_pose']
                solutions.append([v / 1000 for v in raw])
            return {'verified': True, 'joint_solutions_deg': solutions}

        self.service.verify_motion_targets = Mock(side_effect=verify)
        self.service.retain_reorientation_hold = Mock(return_value=True)
        self.patch1 = patch(
            'object_motion_control.check_waypoints',
            return_value=(True, {'verified': True}),
        )
        self.patch2 = patch(
            'object_motion_control.select_object',
            return_value={'mesh': str(mesh), 'pose_mode': '6d'},
        )
        self.patch1.start()
        self.patch2.start()
        self.addCleanup(self.patch1.stop)
        self.addCleanup(self.patch2.stop)

    def args(self, goal=None):
        held = self.service.held_object
        return {
            'object_id': held['object_id'],
            'hold_id': held['hold_id'],
            'orientation_version': held['orientation_version'],
            'goal': goal or {'target_position_m': [.4, -.1, .3]},
        }

    def planned(self, args=None, release=False):
        self.service.start_object_plan(
            args or self.args(), release=release, request_id='planning-001'
        )
        self.service.thread.join(3)
        self.assertFalse(self.service.thread.is_alive())
        self.assertEqual(
            self.service.status['state'],
            'object_motion_planned',
            self.service.status.get('error'),
        )
        self.assertEqual(self.calls, [])
        return self.service.status['object_motion_plan']

    def execute(self, plan, request='execute-001'):
        self.service.start_execute_object_plan(plan['plan_id'], request)
        self.service.thread.join(4)
        self.assertFalse(self.service.thread.is_alive())
        return self.service.status

    def test_full_motion_then_release_and_retreat(self):
        plan = self.planned(
            self.args(
                {
                    'target_position_m': [.4, -.1, .3],
                    'target_rpy_deg': [0, 0, 30],
                }
            )
        )
        status = self.execute(plan)
        self.assertEqual(status['state'], 'completed', status.get('error'))
        self.assertEqual(self.service.held_object['orientation_version'], 1)
        self.assertEqual(
            [x[0] for x in self.calls], ['movej', 'movej', 'movej', 'movel']
        )
        self.robot.move_joint_pose.assert_not_called()
        self.robot.move_gripper.assert_not_called()
        self.assertTrue(all(x[2] == 25 for x in self.calls))
        self.calls.clear()
        args = self.args()
        args.update(goal=None, retreat_vector_base_m=[0, 0, .05])
        plan = self.planned(args, True)
        before = controller_pose(self.arm['pose'])
        status = self.execute(plan, 'release-001')
        self.assertEqual(status['state'], 'completed', status.get('error'))
        self.assertIsNone(self.service.held_object)
        self.assertEqual(
            [x[0] for x in self.calls], ['movel', 'movel', 'open', 'movel']
        )
        np.testing.assert_allclose(
            controller_pose(self.arm['pose'])[:3, 3],
            before[:3, 3] + [0, 0, .05],
            atol=1e-6,
        )
        result = status['execution_result']
        self.assertTrue(result['release_confirmed'])
        self.assertFalse(result['object_settled_pose_verified'])

    def test_linear_target_release_failure_does_not_retreat(self):
        args = self.args(
            {'target_position_m': [.4, -.1, .3], 'path_mode': 'linear'}
        )
        args['retreat_vector_base_m'] = [0, 0, .05]
        plan = self.planned(args, True)
        self.fail = 'open'
        status = self.execute(plan)
        self.assertEqual(status['state'], 'failed')
        self.assertIsNone(self.service.held_object)
        self.assertEqual([x[0] for x in self.calls], ['movel', 'movel', 'open'])
        self.robot.stop.assert_called_once()
        self.service.retain_reorientation_hold.assert_not_called()
        with self.assertRaises(ValueError):
            self.service.start_execute_object_plan(
                plan['plan_id'], 'another-id'
            )

    def test_descent_failure_never_opens_gripper(self):
        plan = self.planned(
            self.args({'target_position_m': [.4, -.1, .3]}), True
        )
        original = self.robot.move_linear.side_effect

        def fail_descent(*args):
            if self.robot.motion_stage.startswith('release_descent'):
                raise RuntimeError('descent failed')
            return original(*args)

        self.robot.move_linear.side_effect = fail_descent
        status = self.execute(plan)
        self.assertEqual(status['state'], 'failed')
        self.robot.move_gripper.assert_not_called()
        self.robot.stop.assert_called_once()

    def test_remaining_descent_checked_before_any_motion(self):
        args = self.args()
        args['retreat_vector_base_m'] = [0, 0, .05]
        plan = self.planned(args, True)

        def reject_future(plan, config, report, arm, points):
            self.assertTrue(
                any(p['name'].startswith('release_descent') for p in points)
            )
            self.assertTrue(
                any(p['name'].startswith('release_retreat') for p in points)
            )
            raise ValueError('remaining descent has no continuous solution')

        self.service.verify_motion_targets.side_effect = reject_future
        status = self.execute(plan)
        self.assertEqual(status['state'], 'failed')
        self.robot.move_linear.assert_not_called()
        self.robot.move_joints.assert_not_called()
        self.robot.move_gripper.assert_not_called()

    def test_start_config_and_hold_changes_reject_before_motion(self):
        for case in ['start', 'config', 'hold', 'preview', 'recording']:
            with self.subTest(case=case):
                # Fresh controller fixture for each independently valid plan.
                self.setUp()
                plan = self.planned()
                if case == 'start':
                    self.arm['pose'][0] += 10000
                if case == 'config':
                    self.config['arm_speed_percent'] = 20
                    self.path.write_text(json.dumps(self.config))
                if case == 'hold':
                    self.service.held_object['orientation_version'] += 1
                if case == 'preview':
                    self.service.mode = 'preview'
                if case == 'recording':
                    self.service.agent_recording_error = 'disk error'
                if case in ('hold', 'preview'):
                    with self.assertRaises(ValueError):
                        self.service.start_execute_object_plan(
                            plan['plan_id'], 'invalid-exec'
                        )
                else:
                    self.assertEqual(self.execute(plan)['state'], 'failed')
                self.assertEqual(self.calls, [])
                self.robot.stop.assert_not_called()

    def test_move_failure_stops_keeps_device_lock_and_never_opens(self):
        plan = self.planned()
        self.fail = 'move'
        import fcntl

        def stop():
            with (self.root / 'work/grasp/robot.lock').open('a') as lock:
                with self.assertRaises(BlockingIOError):
                    fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)

        self.robot.stop.side_effect = stop
        status = self.execute(plan)
        self.assertEqual(status['state'], 'failed')
        self.robot.stop.assert_called_once()
        self.robot.move_gripper.assert_not_called()
        self.service.retain_reorientation_hold.assert_called_once()
        ref = status['diagnostics_ref']
        report = json.loads(
            (
                self.root / 'work/grasp' / ref['directory'] / ref['file']
            ).read_text()
        )
        self.assertTrue(report['robot_motion_sent'])
        self.assertIn('motion failed', report['error'])

    def test_release_retreat_failure_does_not_restore_or_reclose(self):
        args = self.args()
        args.update(goal=None, retreat_vector_base_m=[0, 0, .05])
        plan = self.planned(args, True)
        original = self.robot.move_linear.side_effect

        def fail_retreat(*args):
            if self.opened:
                raise RuntimeError('retreat failed')
            return original(*args)

        self.robot.move_linear.side_effect = fail_retreat
        status = self.execute(plan)
        self.assertEqual(status['state'], 'failed')
        self.assertIsNone(self.service.held_object)
        self.robot.move_gripper.assert_called_once_with(1000)
        self.service.retain_reorientation_hold.assert_not_called()
        ref = status['diagnostics_ref']
        report = json.loads(
            (
                self.root / 'work/grasp' / ref['directory'] / ref['file']
            ).read_text()
        )
        self.assertTrue(report['release_confirmed'])
        self.assertTrue(report['gripper_command_attempted'])

    def test_tracked_reference_generation_change_rejects_execution(self):
        generation = [1]
        reference = np.eye(4)
        reference[0, 3] = .1
        self.tracker.grasp_observation = Mock(
            side_effect=lambda: {
                'object_id': 'rack',
                'pose_mode': '6d',
                'pose': reference.tolist(),
                'model_sha256': 'rackhash',
                'generation': generation[0],
                'time': time.time(),
                'pose_mode_revision': 'rack-revision',
            }
        )
        self.service.calibration = Mock(
            return_value=(
                {'flange_T_color_optical_camera': np.eye(4).tolist()}, {}
            )
        )
        plan = self.planned(
            self.args(
                {
                    'reference_frame': 'tracked_object',
                    'reference_object_id': 'rack',
                    'target_position_m': [0, 0, .05],
                }
            )
        )
        self.assertEqual(plan['reference_context']['generation'], 1)
        generation[0] = 2
        status = self.execute(plan)
        self.assertEqual(status['state'], 'failed')
        self.assertIn('观测版本', status['error'])
        self.assertEqual(self.calls, [])

    def test_scene_reference_motion_uses_frozen_frame_after_camera_moves(self):
        self.service.held_object.update(scene_id='scene', instance_id='tube')
        frame = np.eye(4)
        frame[:3, 3] = [.4, -.1, .3]
        self.service.scene_reference = Mock(
            return_value=(frame, {'name': 'scene_object', 'scene_id': 'scene'})
        )
        self.service.validate_scene = Mock()
        self.service.update_scene_instance = Mock()
        self.tracker.grasp_observation = Mock(
            side_effect=AssertionError('must not track reference again')
        )
        args = self.args(
            {
                'reference_frame': 'scene_object',
                'scene_id': 'scene',
                'reference_instance_id': 'rack',
                'reference_feature_id': 'entry',
                'target_position_m': [0, 0, 0],
            }
        )
        plan = self.planned(args)
        np.testing.assert_allclose(
            plan['target_reference_position_base_m'], frame[:3, 3]
        )
        self.tracker.state['camera_frame_at'] = 0
        status = self.execute(plan)
        self.assertEqual(status['state'], 'completed', status.get('error'))
        self.assertEqual(self.service.held_object['scene_id'], 'scene')
        self.tracker.grasp_observation.assert_not_called()
        self.calls.clear()
        args = self.args()
        args.update(goal=None, retreat_vector_base_m=[0, 0, 0])
        plan = self.planned(args, True)
        status = self.execute(plan, 'release-scene')
        self.assertEqual(status['state'], 'completed', status.get('error'))
        self.assertEqual(
            [
                c.args[1]
                for c in self.service.update_scene_instance.call_args_list
            ],
            ['release_unknown', 'placed_unverified'],
        )

    def test_reference_frame_resolution_and_5d_rejection(self):
        held = self.held
        config = self.config
        tool = {'flange_T_grip_tcp': np.eye(4).tolist()}
        goal = {'reference_frame': 'tcp'}
        frame, context, arm = self.service.resolve_motion_frame(
            goal, held, config, self.arm, tool
        )
        np.testing.assert_allclose(frame, controller_pose(self.arm['pose']))
        with self.assertRaisesRegex(ValueError, '5D'):
            self.service.resolve_motion_frame(
                {'reference_frame': 'held_object'},
                {**held, 'pose_mode': '5d'},
                config,
                self.arm,
                tool,
            )
        self.service.calibration = Mock(
            return_value=(
                {'flange_T_color_optical_camera': np.eye(4).tolist()}, tool
            )
        )
        frame, _, _ = self.service.resolve_motion_frame(
            {'reference_frame': 'camera'}, held, config, self.arm, tool
        )
        np.testing.assert_allclose(frame, controller_pose(self.arm['pose']))
        self.tracker.state['camera_frame_at'] = 0
        with self.assertRaisesRegex(ValueError, '新鲜'):
            self.service.resolve_motion_frame(
                {'reference_frame': 'camera'}, held, config, self.arm, tool
            )


if __name__ == '__main__':
    unittest.main()
