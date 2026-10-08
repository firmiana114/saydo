"""Persistent opening configuration and joint reset, using simulated hardware only."""

import copy
import fcntl
import json
from pathlib import Path
import tempfile
import unittest
from unittest.mock import Mock, patch

import yaml

from grasp_control import GraspController
from grasp_geometry import return_joint_target
import test_grasp


class OperatorControlsTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.addCleanup(self.temp.cleanup)
        self.root = Path(self.temp.name)
        self.config = test_grasp.GraspTests().config()
        self.config.update(
            tool_file='tool.yaml',
            return_after_lift={'joint_deg': [1, 2, 3, 4, 5, 6, 7]},
        )
        self.config['unrelated'] = {'keep': [1, 2, 3]}
        self.path = self.root / 'config.json'
        self.path.write_text(json.dumps(self.config, indent=2) + '\n')
        self.tool = {'tool_name': 'grip_tcp', 'pose': [0] * 6, 'payload': 1061}
        (self.root / 'tool.yaml').write_text(
            yaml.safe_dump(
                {'controller_readback': {'tool_response_raw': self.tool}}
            )
        )
        self.service = GraspController(self.root, None, self.path)

    def robot(self, failure=None, initial_joints=None):
        robot = Mock()
        arm = {
            'joint': initial_joints if initial_joints is not None else [0] * 7,
            'pose': [0, 0, 200000, 0, 0, 0],
            'err': [0],
        }

        def query(command):
            values = {
                'get_arm_software_info': {'Product_version': 'RM75-BI'},
                'get_rm_plus_mode': {
                    'mode': 0 if failure == 'communication' else 115200
                },
                'get_joint_en_state': {'en_state': [1] * 7},
                'get_joint_err_flag': {'err_flag': [0] * 7},
                'get_current_tool_frame': self.tool,
                'get_current_arm_state': {'arm_state': arm},
                'get_rm_plus_state_info': {
                    'state_info': {
                        'sys_err': 0,
                        'sys_state': 0,
                        'dof_err': [0],
                        'dof_state': [3],
                        'pos': [500],
                        'speed': [0],
                        'force': [10],
                    }
                },
            }
            reply = copy.deepcopy(values[command])
            if command == 'get_joint_en_state' and failure == 'disabled':
                reply['en_state'][0] = 0
            return reply

        robot.query.side_effect = query

        def move(target, speed, cancelled, watchdog):
            if failure == 'cancel':
                self.service.abort()
            watchdog()
            if failure == 'timeout':
                raise TimeoutError('controller timeout')
            if failure != 'arrival':
                arm['joint'] = target

        robot.move_joints.side_effect = move
        return robot

    def test_open_position_persists_only_requested_field_and_never_connects(
        self
    ):
        original = self.path.read_text()
        with patch('grasp_control.RobotClient') as client:
            for value in (300, 0, 1000):
                result = self.service.set_open_position(value)
                expected = copy.deepcopy(self.config)
                expected['gripper']['open_position'] = value
                self.assertEqual(json.loads(self.path.read_text()), expected)
                self.assertEqual(
                    self.path.read_text(),
                    original.replace(
                        '"open_position": 1000', f'"open_position": {value}'
                    ),
                )
                self.assertFalse(result['motion_sent'])
                fresh = GraspController(self.root, None, self.path)
                self.assertEqual(
                    fresh.snapshot()['open_position_config']['value'], value
                )
            client.assert_not_called()

    def test_bad_input_busy_lock_and_failed_replace_preserve_config(self):
        original = self.path.read_bytes()
        for value in (-1, 1001, 1.5, True, '300', None):
            with self.subTest(value=value), self.assertRaises(ValueError):
                self.service.set_open_position(value)
        self.service.status['busy'] = True
        with self.assertRaises(ValueError):
            self.service.set_open_position(300)
        self.service.status['busy'] = False
        directory = self.root / 'work/grasp'
        directory.mkdir(parents=True)
        with (directory / 'robot.lock').open('a') as lock:
            fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
            with self.assertRaises(BlockingIOError):
                self.service.set_open_position(300)
        with patch.object(Path, 'replace', side_effect=OSError('disk failure')):
            with self.assertRaises(OSError):
                self.service.set_open_position(300)
        self.assertEqual(self.path.read_bytes(), original)
        self.assertFalse(list(self.root.glob('.grasp-*')))

    def test_reset_uses_configured_return_without_camera_or_gripper_motion(
        self
    ):
        robot = self.robot()
        with patch(
            'grasp_control.RobotClient', return_value=robot
        ), patch('grasp_control.time.sleep'):
            self.assertTrue(self.service.start_reset()['accepted'])
            self.service.thread.join(3)
        self.assertFalse(self.service.thread.is_alive())
        self.assertEqual(self.service.snapshot()['state'], 'completed')
        self.assertFalse(self.service.snapshot()['busy'])
        self.assertEqual(
            robot.move_joints.call_args.args[:2],
            (return_joint_target(self.config), 2),
        )
        robot.move_gripper.assert_not_called()
        robot.set_register.assert_not_called()
        robot.stop.assert_not_called()
        report = json.loads(
            next(
                (self.root / 'work/grasp/reset_jobs').glob('*.json')
            ).read_text()
        )
        self.assertEqual(
            report['arrival_checks'][0]['target_joint'],
            return_joint_target(self.config),
        )
        self.assertFalse(report['result']['busy'])

    def test_zero_reset_uses_seven_zero_angles_and_preserves_observation_target(
        self
    ):
        original = self.path.read_bytes()
        robot = self.robot(initial_joints=[5000] * 7)
        with patch(
            'grasp_control.RobotClient', return_value=robot
        ), patch('grasp_control.time.sleep'):
            result = self.service.start_reset('zero')
            self.service.thread.join(3)
        self.assertEqual(result['reset_target_kind'], 'zero')
        self.assertEqual(self.service.status['state'], 'completed')
        self.assertIn('七关节零位', self.service.status['message'])
        self.assertEqual(
            robot.move_joints.call_args.args[:2],
            ([0] * 7, self.config['arm_speed_percent']),
        )
        robot.move_gripper.assert_not_called()
        robot.set_register.assert_not_called()
        self.assertEqual(self.path.read_bytes(), original)
        report = json.loads(
            next(
                (self.root / 'work/grasp/reset_jobs').glob('*.json')
            ).read_text()
        )
        self.assertEqual(report['reset_target_kind'], 'zero')
        self.assertEqual(report['arrival_checks'][0]['target_joint'], [0] * 7)

    def test_zero_reset_is_independent_of_observation_target_but_keeps_write_gates(
        self
    ):
        self.path.write_text(
            json.dumps(
                {**self.config, 'return_after_lift': {'joint_deg': [1, 2]}}
            )
        )
        self.assertFalse(self.service.snapshot()['reset_available'])
        self.assertTrue(self.service.snapshot()['zero_reset_available'])
        with patch('grasp_control.threading.Thread') as thread:
            for kind in ('invalid', '', None, True):
                with self.assertRaises(ValueError):
                    self.service.start_reset(kind)
            self.service.mode = 'preview'
            with self.assertRaises(ValueError):
                self.service.start_reset('zero')
            self.service.mode = 'execute'
            self.service.status['busy'] = True
            with self.assertRaises(ValueError):
                self.service.start_reset('zero')
            self.service.status['busy'] = False
            thread.assert_not_called()
            self.service.start_reset('zero')
            self.assertEqual(
                thread.call_args.kwargs['args'][1:], ([0] * 7, 'zero')
            )
            with self.assertRaises(ValueError):
                self.service.start_reset()
            with self.assertRaises(ValueError):
                self.service.open_gripper()

    def test_zero_reset_failure_cancel_and_device_lock(self):
        for failure in (
            'arrival', 'timeout', 'cancel', 'disabled', 'communication'
        ):
            with self.subTest(failure=failure):
                self.service = GraspController(self.root, None, self.path)
                robot = self.robot(failure, initial_joints=[1000] * 7)
                with patch(
                    'grasp_control.RobotClient', return_value=robot
                ), patch('grasp_control.time.sleep'):
                    self.service.start_reset('zero')
                    self.service.thread.join(3)
                self.assertEqual(
                    self.service.status['state'],
                    'cancelled' if failure == 'cancel' else 'failed',
                )
                if failure in ('disabled', 'communication'):
                    robot.move_joints.assert_not_called()
                    robot.stop.assert_not_called()
                else:
                    robot.stop.assert_called()
                robot.move_gripper.assert_not_called()
                robot.set_register.assert_not_called()
        self.service = GraspController(self.root, None, self.path)
        with (
            self.root / 'work/grasp/robot.lock'
        ).open('a') as lock, patch('grasp_control.RobotClient') as client:
            fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
            self.service.start_reset('zero')
            self.service.thread.join(3)
            client.assert_not_called()
        self.assertEqual(self.service.status['state'], 'failed')

    def test_reset_rereads_communication_after_recovery(self):
        for failure, expected in [
            ('communication', 'failed'), (None, 'completed')
        ]:
            robot = self.robot(failure)
            with patch(
                'grasp_control.RobotClient', return_value=robot
            ), patch('grasp_control.time.sleep'):
                self.service.start_reset()
                self.service.thread.join(3)
            self.assertEqual(self.service.status['state'], expected)
            if failure:
                self.assertIn('当前模式 0', self.service.status['error'])
                robot.move_joints.assert_not_called()
            else:
                self.assertIsNone(self.service.status['error'])
                robot.move_joints.assert_called_once()

    def test_reset_requires_execution_mode_target_and_exclusive_task(self):
        with patch('grasp_control.threading.Thread') as thread:
            self.service.mode = 'preview'
            with self.assertRaises(ValueError):
                self.service.start_reset()
            self.service.mode = 'execute'
            self.service.status['busy'] = True
            with self.assertRaises(ValueError):
                self.service.start_reset()
            self.service.status['busy'] = False
            for target in (
                None,
                {'joint_deg': [True] * 7},
                {'joint_deg': [1] * 6},
                {'joint_deg': [float('nan')] * 7},
            ):
                self.path.write_text(
                    json.dumps({**self.config, 'return_after_lift': target})
                )
                with self.subTest(target=target), self.assertRaises(ValueError):
                    self.service.start_reset()
            thread.assert_not_called()
            self.path.write_text(json.dumps(self.config))
            self.service.start_reset()
            with self.assertRaises(ValueError):
                self.service.start_reset()
            with self.assertRaises(ValueError):
                self.service.set_open_position(500)
            with self.assertRaises(ValueError):
                self.service.open_gripper()
            self.assertEqual(thread.call_count, 1)

    def test_reset_errors_cancel_and_device_lock(self):
        for failure in (
            'arrival', 'timeout', 'cancel', 'disabled', 'communication'
        ):
            with self.subTest(failure=failure):
                self.service = GraspController(self.root, None, self.path)
                robot = self.robot(failure)
                with patch(
                    'grasp_control.RobotClient', return_value=robot
                ), patch('grasp_control.time.sleep'):
                    self.service.start_reset()
                    self.service.thread.join(3)
                self.assertEqual(
                    self.service.status['state'],
                    'cancelled' if failure == 'cancel' else 'failed',
                )
                self.assertFalse(self.service.status['busy'])
                if failure in ('disabled', 'communication'):
                    robot.move_joints.assert_not_called()
                    robot.stop.assert_not_called()
                else:
                    robot.stop.assert_called()
                robot.move_gripper.assert_not_called()
                robot.set_register.assert_not_called()
        lock_path = self.root / 'work/grasp/robot.lock'
        self.service = GraspController(self.root, None, self.path)
        with lock_path.open('a') as lock, patch(
            'grasp_control.RobotClient'
        ) as client:
            fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
            self.service.start_reset()
            self.service.thread.join(3)
            client.assert_not_called()
        self.assertEqual(self.service.status['state'], 'failed')


if __name__ == '__main__':
    unittest.main()
