"""Controller failure replies must never reach callers as a missing state_info key."""

import unittest
from unittest import mock

from saydo.robot import grasp_robot


class GripperQueryTests(unittest.TestCase):
    def test_failed_missing_or_malformed_payload_is_actionable(self):
        robot = grasp_robot.RobotClient('unused')
        for command, field in [
            ('get_rm_plus_state_info', 'state_info'),
            ('get_rm_plus_base_info', 'base_info'),
        ]:
            for reply in [
                {'command': command, 'get_state': False},
                {'command': command, 'get_state': True},
                {'command': command, field: None},
                {'command': command, field: []},
                {'command': command, 'get_state': False, field: {}},
                None,
            ]:
                with self.subTest(
                    command=command, reply=reply
                ), mock.patch.object(robot, '_request', return_value=reply):
                    with self.assertRaisesRegex(
                        RuntimeError, '夹爪 RM Plus 查询.*供电和连接'
                    ):
                        robot.query(command)

    def test_valid_payload_preserved_without_extra_hardware_commands(self):
        robot = grasp_robot.RobotClient('unused')
        for command, field in [
            ('get_rm_plus_state_info', 'state_info'),
            ('get_rm_plus_base_info', 'base_info'),
        ]:
            reply = {'command': command, field: {'sys_state': 0, 'speed': [0]}}
            with mock.patch.object(
                robot, '_request', return_value=reply
            ) as request:
                self.assertIs(robot.query(command), reply)
                request.assert_called_once_with({'command': command})

    def test_successful_retry_is_not_blocked_by_previous_error(self):
        robot = grasp_robot.RobotClient('unused')
        valid = {'state_info': {'sys_state': 0, 'speed': [0]}}
        with mock.patch.object(
            robot, '_request', side_effect=[{'get_state': False}, valid]
        ):
            with self.assertRaises(RuntimeError):
                robot.query('get_rm_plus_state_info')
            self.assertEqual(robot.query('get_rm_plus_state_info'), valid)


if __name__ == '__main__':
    unittest.main()
