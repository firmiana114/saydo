"""Grasp must stop after lift; manual reset transport is retained."""

import copy
import json
from pathlib import Path
import tempfile
import threading
import unittest
from unittest.mock import Mock, patch

import numpy as np

from grasp_control import GraspController
from grasp_geometry import plan_grasp, return_joint_target
from grasp_robot import RobotClient
import test_grasp


class ReturnGraspTests(unittest.TestCase):
    def test_grasp_stops_after_lift_even_with_a_legacy_return_target(self):
        for failure in (None, 'arrival', 'empty'):
            with self.subTest(
                failure=failure
            ), tempfile.TemporaryDirectory() as directory:
                root = Path(directory)
                config = test_grasp.GraspTests().config()
                config['return_after_lift'] = {
                    'joint_deg': [1, 2, 3, 4, 5, 6, 7]
                }
                cfg = root / 'config.json'
                cfg.write_text(json.dumps(config))
                service = GraspController(root, None, cfg)
                arm = {
                    'joint': [0] * 7,
                    'pose': [0, 0, 200000, 0, 0, 0],
                    'err': [0],
                }
                obj = np.eye(4)
                obj[:3, 3] = [.1, 0, .2]
                plan = plan_grasp(
                    arm['pose'], obj, config['grasp_profile'], config
                )
                self.assertNotIn('return_joint_raw', plan)
                plan.update(
                    current_arm=copy.deepcopy(arm),
                    gripper_base={'force_up': [100]},
                    tool_response={},
                    work_response={},
                    object_id='sample',
                    model_sha256='fixture',
                    return_joint_raw=[1000, 2000, 3000, 4000, 5000, 6000, 7000],
                )
                calls = []
                closed = [False]
                robot = Mock()

                def query(command):
                    if command == 'get_current_arm_state':
                        return {'arm_state': copy.deepcopy(arm)}
                    if (
                        command
                        in ('get_current_tool_frame', 'get_current_work_frame')
                    ):
                        return {}
                    empty = closed[0] and failure == 'empty'
                    return {
                        'state_info': {
                            'sys_err': 0,
                            'sys_state': 0,
                            'dof_err': [0],
                            'dof_state': [2 if empty or not closed[0] else 3],
                            'pos': [0 if empty else 500 if closed[0] else 1000],
                            'speed': [0],
                            'force': [10 if closed[0] else 0],
                        }
                    }

                robot.query.side_effect = query

                def gripper(position):
                    calls.append(('gripper', position))
                    closed[0] = position == 0

                robot.move_gripper.side_effect = gripper

                def linear(pose, speed, cancelled, watchdog):
                    before = robot.query.call_count
                    watchdog()
                    self.assertEqual(robot.query.call_count, before)
                    calls.append(('linear', pose))
                    arm['pose'] = pose.copy()
                    if (
                        failure == 'arrival'
                        and pose == plan['waypoints'][-1]['controller_pose']
                    ):
                        arm['pose'][2] += 15000

                robot.move_linear.side_effect = linear

                def joint_pose(pose, speed, cancelled, watchdog):
                    watchdog()
                    calls.append(('joint_pose', pose))
                    arm['pose'] = pose

                robot.move_joint_pose.side_effect = joint_pose
                robot.move_joints.side_effect = AssertionError(
                    'Automatic return after grasp is removed'
                )
                service.robot = robot
                with patch.object(service, 'verify_motion_targets'):
                    if failure:
                        with self.assertRaises(ValueError):
                            service.execute(plan, config, {})
                    else:
                        service.execute(plan, config, {})
                        self.assertEqual(service.status['state'], 'completed')
                        robot.move_joint_pose.assert_called_once_with(
                            plan['waypoints'][0]['controller_pose'],
                            config['arm_speed_percent'],
                            service.cancelled,
                            unittest.mock.ANY,
                        )
                        self.assertEqual(robot.move_linear.call_count, 2)
                        self.assertEqual(
                            calls[-1],
                            (
                                'linear',
                                plan['waypoints'][-1]['controller_pose'],
                            ),
                        )
                        self.assertEqual(
                            arm['pose'],
                            plan['waypoints'][-1]['controller_pose'],
                        )
                        self.assertEqual(
                            service.held_object['object_id'], 'sample'
                        )
                    robot.move_joints.assert_not_called()
                self.assertEqual(
                    [x for x in calls if x[0] == 'gripper'],
                    [('gripper', 1000), ('gripper', 0)],
                )

    def test_movej_transport_requires_matching_ack_and_completion(self):
        robot = RobotClient('test', writes=True)
        cancel = threading.Event()
        for accepted in (True, False):
            connection = Mock()
            connection.recv.return_value = (
                json.dumps({'command': 'movej', 'receive_state': accepted})
                + '\n'
                + json.dumps(
                    {
                        'state': 'current_trajectory_state',
                        'trajectory_state': True,
                    }
                )
                + '\n'
            ).encode()
            with patch('grasp_robot.socket.create_connection') as connect:
                connect.return_value.__enter__.return_value = connection
                if accepted:
                    robot.move_joints([0] * 7, 10, cancel, lambda: None)
                else:
                    with self.assertRaises(RuntimeError):
                        robot.move_joints([0] * 7, 10, cancel, lambda: None)
                payload = json.loads(connection.sendall.call_args.args[0])
                self.assertEqual(
                    payload,
                    {
                        'command': 'movej',
                        'joint': [0] * 7,
                        'v': 10,
                        'r': 0,
                        'trajectory_connect': 0,
                    },
                )
        with self.assertRaises(ValueError):
            RobotClient('test').move_joints([0] * 7, 10, cancel, lambda: None)
        cancel.set()
        with patch('grasp_robot.socket.create_connection') as connect:
            with self.assertRaisesRegex(RuntimeError, '取消'):
                robot.move_joints([0] * 7, 10, cancel, lambda: None)
            connect.assert_not_called()

    def test_invalid_manual_reset_config_does_not_block_grasp_planning(self):
        config = test_grasp.GraspTests().config()
        obj = np.eye(4)
        obj[2, 3] = .2
        for joints in ([0] * 6, [0] * 6 + [float('nan')], [0] * 6 + [400]):
            config['return_after_lift'] = {'joint_deg': joints}
            with self.assertRaises(ValueError):
                return_joint_target(config)
            plan = plan_grasp(
                [0, 0, 200000, 0, 0, 0], obj, config['grasp_profile'], config
            )
            self.assertNotIn('return_joint_raw', plan)

    def test_movel_controller_rejection_or_failed_completion_is_not_ignored(
        self
    ):
        robot = RobotClient('test', writes=True)
        for accepted, completed in (
            (False, False), (True, False), (True, True)
        ):
            connection = Mock()
            connection.recv.return_value = (
                json.dumps({'command': 'movel', 'receive_state': accepted})
                + '\n'
                + json.dumps(
                    {
                        'state': 'current_trajectory_state',
                        'trajectory_state': completed,
                    }
                )
                + '\n'
            ).encode()
            with patch('grasp_robot.socket.create_connection') as connect:
                connect.return_value.__enter__.return_value = connection
                if accepted and completed:
                    robot.move_linear(
                        [0] * 6, 10, threading.Event(), lambda: None
                    )
                else:
                    with self.assertRaises(RuntimeError):
                        robot.move_linear(
                            [0] * 6, 10, threading.Event(), lambda: None
                        )
                self.assertEqual(
                    json.loads(connection.sendall.call_args.args[0])['command'],
                    'movel',
                )

    def test_movej_p_transport_and_validation(self):
        robot = RobotClient('test', writes=True)
        pose = [100000, 200000, 300000, 400, 500, 600]
        for accepted, completed in (
            (False, False), (True, False), (True, True)
        ):
            with self.subTest(accepted=accepted, completed=completed):
                connection = Mock()
                connection.recv.return_value = (
                    json.dumps(
                        {'command': 'movej_p', 'receive_state': accepted}
                    )
                    + '\n'
                    + json.dumps(
                        {
                            'state': 'current_trajectory_state',
                            'trajectory_state': completed,
                        }
                    )
                    + '\n'
                ).encode()
                with patch('grasp_robot.socket.create_connection') as connect:
                    connect.return_value.__enter__.return_value = connection
                    if accepted and completed:
                        robot.move_joint_pose(
                            pose, 20, threading.Event(), lambda: None
                        )
                    else:
                        with self.assertRaises(RuntimeError):
                            robot.move_joint_pose(
                                pose, 20, threading.Event(), lambda: None
                            )
                    self.assertEqual(
                        json.loads(connection.sendall.call_args.args[0]),
                        {
                            'command': 'movej_p',
                            'pose': pose,
                            'v': 20,
                            'r': 0,
                            'trajectory_connect': 0,
                        },
                    )
        with patch('grasp_robot.socket.create_connection') as connect:
            cancel = threading.Event()
            cancel.set()
            with self.assertRaises(RuntimeError):
                robot.move_joint_pose(pose, 20, cancel, lambda: None)
            for invalid in ([0] * 7, [0] * 5, [0., 0, 0, 0, 0, 0]):
                with self.assertRaises(ValueError):
                    robot.move_joint_pose(
                        invalid, 20, threading.Event(), lambda: None
                    )
            robot.writes = False
            with self.assertRaises(ValueError):
                robot.move_joint_pose(pose, 20, threading.Event(), lambda: None)
            connect.assert_not_called()


if __name__ == '__main__':
    unittest.main()
