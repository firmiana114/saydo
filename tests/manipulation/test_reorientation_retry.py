"""Retry state survives motion failure only when the stopped hold remains valid."""

import copy
import json
from pathlib import Path
import tempfile
from types import SimpleNamespace
import unittest
from unittest.mock import Mock, patch

import numpy as np
import yaml

from saydo.manipulation.grasp_control import GraspController
from saydo.geometry.grasp_geometry import controller_pose
from tests.manipulation.test_reorientation import fixture


class ReorientationRetryTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.addCleanup(self.temp.cleanup)
        self.root = Path(self.temp.name)
        self.config, self.settings, self.held, self.vertices, self.arm = (
            fixture()
        )
        self.config['tool_file'] = 'tool.yaml'
        self.config['ik_checker'] = 'checker'
        self.path = self.root / 'trial.json'
        self.path.write_text(json.dumps(self.config))
        (self.root / 'tool.yaml').write_text(
            yaml.safe_dump(
                {
                    'controller_readback': {
                        'tool_response_raw': self.held['tool_response']
                    }
                }
            )
        )
        (self.root / 'checker').touch()
        (self.root / 'config/grasp').mkdir(parents=True)
        (
            self.root / 'config/grasp/reorientation.json'
        ).write_text(json.dumps(self.settings))
        tracker = SimpleNamespace(
            models_root=self.root,
            state={'object_id': 'sample', 'model_sha256': 'mesh1'},
        )
        self.service = GraspController(self.root, tracker, self.path)
        self.service.held_object = copy.deepcopy(self.held)
        self.mesh = {
            'model_sha256': 'mesh1', 'vertices': self.vertices.tolist()
        }

    def robot(self, failure=None):
        robot = Mock()
        counter = 0

        def query(command):
            nonlocal counter
            if failure == 'disconnect':
                raise OSError('disconnected')
            if command == 'get_current_arm_state':
                counter += 1
                arm = copy.deepcopy(self.arm)
                if failure == 'moving':
                    arm['joint'][0] += counter * 100
                return {'arm_state': arm}
            if command == 'get_rm_plus_state_info':
                return {
                    'state_info': {
                        'sys_err': 0,
                        'sys_state': 0,
                        'dof_err': [0],
                        'dof_state': [2 if failure == 'open' else 3],
                        'pos': [1000 if failure == 'open' else 500],
                        'speed': [0],
                        'force': [0],
                    }
                }
            return {
                'get_current_tool_frame': (
                    {'changed': True}
                    if failure == 'tool'
                    else self.held['tool_response']
                ),
                'get_current_work_frame': {
                    'frame_name': 'Base', 'pose': [0] * 6
                },
                'get_rm_plus_mode': {'mode': 115200},
                'get_rm_plus_base_info': {
                    'base_info': {'manu': 'ZX', 'id': 1, 'dof': 1, 'type': 1}
                },
            }[command]

        robot.query.side_effect = query
        self.service.robot = robot
        return robot

    def test_stopped_hold_refreshes_from_actual_pose_even_with_controller_error(
        self
    ):
        robot = self.robot()
        self.arm['pose'][3:] = [2800, 200, 300]
        self.arm['err'] = [4099]
        report = {}
        with patch(
            'saydo.manipulation.reorientation_control.model_geometry',
            return_value=self.mesh,
        ), patch('saydo.manipulation.reorientation_control.time.sleep'):
            self.assertTrue(
                self.service.retain_reorientation_hold(
                    self.held, self.config, report
                )
            )
        actual = (
            controller_pose(self.arm['pose'])
            @ np.linalg.inv(self.held['object_T_tcp'])
        )
        np.testing.assert_allclose(
            self.service.held_object['current_object_rotation'], actual[:3, :3]
        )
        self.assertEqual(self.service.held_object['hold_id'], 'hold1')
        self.assertEqual(self.service.held_object['orientation_version'], 1)
        self.assertEqual(
            self.service.held_object['object_T_tcp'], self.held['object_T_tcp']
        )
        self.assertEqual(report['hold_retention']['controller_errors'], [4099])
        with self.assertRaisesRegex(ValueError, '错误状态'):
            self.service.arm_state()
        robot.stop.assert_not_called()
        robot.move_joint_pose.assert_not_called()
        robot.move_gripper.assert_not_called()

    def test_uncertain_or_invalidated_holds_are_not_restored(self):
        for failure in (
            'disconnect',
            'moving',
            'tool',
            'open',
            'model',
            'stop',
            'invalidated',
        ):
            with self.subTest(failure=failure):
                self.service.held_object = copy.deepcopy(self.held)
                robot = self.robot(failure)
                report = (
                    {'stop_error': 'stop failed'} if failure == 'stop' else {}
                )
                if failure == 'invalidated':
                    self.service.held_object = None
                mesh = (
                    {**self.mesh, 'model_sha256': 'changed'}
                    if failure == 'model'
                    else self.mesh
                )
                with patch(
                    'saydo.manipulation.reorientation_control.model_geometry',
                    return_value=mesh,
                ), patch('saydo.manipulation.reorientation_control.time.sleep'):
                    self.assertFalse(
                        self.service.retain_reorientation_hold(
                            self.held, self.config, report
                        )
                    )
                self.assertIsNone(self.service.held_object)
                self.assertFalse(report['hold_retention']['retained'])
                robot.move_gripper.assert_not_called()
                robot.move_joint_pose.assert_not_called()

    def test_motion_failure_and_cancel_allow_new_target_after_revalidation(
        self
    ):
        for cancelled in (False, True):
            with self.subTest(cancelled=cancelled):
                self.service.held_object = copy.deepcopy(self.held)
                self.service.cancelled.clear()
                self.service.actuated = False
                robot = self.robot()
                self.service.reorientation_hardware = Mock(
                    return_value=(
                        copy.deepcopy(self.arm),
                        {'flange_T_grip_tcp': np.eye(4).tolist()},
                    )
                )

                def fail(*_):
                    self.service.actuated = True
                    self.arm['pose'][3:] = [2800, 200, 300]
                    if cancelled:
                        self.service.cancelled.set()
                    raise RuntimeError('motion failed')

                self.service.execute_reorient = Mock(side_effect=fail)
                with patch(
                    'saydo.manipulation.reorientation_control.RobotClient',
                    return_value=robot,
                ), patch(
                    'saydo.manipulation.reorientation_control.model_geometry',
                    return_value=self.mesh,
                ), patch(
                    'saydo.manipulation.reorientation_control.endpoint_reachab'
                    'ility',
                    return_value=(True, {}),
                ), patch('saydo.manipulation.reorientation_control.time.sleep'):
                    self.service.run_reorient(self.held, np.eye(3).tolist())
                self.assertEqual(
                    self.service.status['state'],
                    'cancelled' if cancelled else 'failed',
                )
                self.assertFalse(self.service.status['busy'])
                self.assertTrue(
                    self.service.status['reorientation_retry_available']
                )
                self.assertIsNone(self.service.status['reorientation_plan'])
                self.assertIn('夹持记录已保留', self.service.status['message'])
                self.assertEqual(self.service.held_object['hold_id'], 'hold1')
                robot.stop.assert_called_once()
                robot.move_gripper.assert_not_called()
                request = {
                    'rotation': np.diag([1, -1, -1]).tolist(),
                    'orientation_version': self.service.held_object[
                        'orientation_version'
                    ],
                    **{
                        k: self.held[k]
                        for k in ['hold_id', 'object_id', 'model_sha256']
                    },
                }
                with patch(
                    'saydo.manipulation.reorientation_control.threading.Thread'
                ) as thread:
                    self.service.start_reorient(request)
                    self.assertFalse(self.service.cancelled.is_set())
                    self.assertTrue(self.service.status['busy'])
                    self.assertEqual(
                        thread.call_args.kwargs['args'][0][
                            'orientation_version'
                        ],
                        1,
                    )
                self.service.status['busy'] = False

    def test_stop_failure_clears_hold_and_prevents_retry(self):
        robot = self.robot()
        robot.stop.side_effect = RuntimeError('stop failed')
        self.service.reorientation_hardware = Mock(
            return_value=(self.arm, {'flange_T_grip_tcp': np.eye(4).tolist()})
        )

        def fail(*_):
            self.service.actuated = True
            raise RuntimeError('motion failed')

        self.service.execute_reorient = Mock(side_effect=fail)
        with patch(
            'saydo.manipulation.reorientation_control.RobotClient',
            return_value=robot,
        ), patch(
            'saydo.manipulation.reorientation_control.model_geometry',
            return_value=self.mesh,
        ), patch(
            'saydo.manipulation.reorientation_control.endpoint_reachability',
            return_value=(True, {}),
        ):
            self.service.run_reorient(self.held, np.eye(3).tolist())
        self.assertIsNone(self.service.held_object)
        self.assertFalse(self.service.status['reorientation_retry_available'])

    def failed_report(self):
        jobs = self.root / 'work/grasp/reorient_jobs'
        jobs.mkdir(parents=True, exist_ok=True)
        path = jobs / '12345.json'
        path.write_text(
            json.dumps(
                {
                    'operation': 'reorient',
                    'mode': 'execute',
                    'error': 'motion failed',
                    'held_object': self.held,
                }
            )
        )
        return path

    def test_explicit_startup_recovery_is_read_only_and_one_use(self):
        path = self.failed_report()
        self.service.held_object = None
        robot = self.robot()
        with patch(
            'saydo.manipulation.reorientation_control.RobotClient',
            return_value=robot,
        ) as client, patch(
            'saydo.manipulation.reorientation_control.model_geometry',
            return_value=self.mesh,
        ), patch('saydo.manipulation.reorientation_control.time.sleep'):
            self.assertTrue(self.service.restore_reorientation_report(path))
            self.assertFalse(client.call_args.kwargs['writes'])
            self.assertEqual(self.service.held_object['hold_id'], 'hold1')
            self.service.held_object = None
            client.reset_mock()
            self.assertFalse(self.service.restore_reorientation_report(path))
            client.assert_not_called()
        robot.stop.assert_not_called()
        robot.move_gripper.assert_not_called()
        robot.move_joint_pose.assert_not_called()

    def test_recovery_rejects_wrong_model_or_non_reorientation_report(self):
        path = self.failed_report()
        self.service.held_object = None
        with patch(
            'saydo.manipulation.reorientation_control.RobotClient'
        ) as client:
            self.service.tracker.state['object_id'] = 'different'
            self.assertFalse(self.service.restore_reorientation_report(path))
            self.service.tracker.state['object_id'] = 'sample'
            value = json.loads(path.read_text())
            value['operation'] = 'place'
            path.write_text(json.dumps(value))
            self.assertFalse(self.service.restore_reorientation_report(path))
            client.assert_not_called()


if __name__ == '__main__':
    unittest.main()
