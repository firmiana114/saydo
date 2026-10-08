"""Motion diagnostics use simulated controller responses, never hardware."""

import json
import threading
import unittest
from unittest.mock import Mock, patch

from saydo.robot.grasp_robot import RobotClient


class MotionLogTests(unittest.TestCase):
    def run_motion(self, replies):
        robot = RobotClient('test', writes=True)
        connection = Mock()
        connection.recv.side_effect = replies
        with patch(
            'saydo.robot.grasp_robot.socket.create_connection'
        ) as connect:
            connect.return_value.__enter__.return_value = connection
            try:
                robot.move_joint_pose(
                    [0] * 6, 20, threading.Event(), lambda: None
                )
                error = None
            except Exception as failure:
                error = failure
        return robot.motion_log[0], error

    def test_failure_is_distinct_from_missing_ack(self):
        completion = (
            b'{"state":"current_trajectory_state","trajectory_state":false,"dev'
            b'ice":0}\n'
        )
        for ack in (b'', b'{"command":"movej_p","receive_state":true}\n'):
            record, error = self.run_motion([ack + completion])
            self.assertIn('轨迹未成功完成' if ack else '尚未收到', str(error))
            self.assertEqual(record['accepted'], bool(ack))
            self.assertTrue(record['send_completed'])
            self.assertEqual(record['command']['command'], 'movej_p')
            self.assertEqual(
                record['completion_reply']['raw'], completion.decode().strip()
            )
            self.assertEqual(record['outcome'], 'failed')
            self.assertGreaterEqual(record['elapsed_s'], 0)
            json.dumps(record)

    def test_split_reply_and_success(self):
        record, error = self.run_motion(
            [
                b'{"command":"movej_',
                (
                    b'p","receive_state":true}\n{"state":"current_trajectory_st'
                    b'ate","trajectory_state":true}\n'
                ),
            ]
        )
        self.assertIsNone(error)
        self.assertEqual(len(record['responses']), 2)
        self.assertEqual(record['outcome'], 'completed')

    def test_rejection_invalid_json_disconnect(self):
        for replies, text in [
            ([b'{"command":"movej_p","receive_state":false}\n'], '拒绝'),
            ([b'bad json\n'], 'Expecting value'),
            ([b''], '断开连接'),
        ]:
            record, error = self.run_motion(replies)
            self.assertIn(text, str(error))
            self.assertEqual(record['outcome'], 'failed')
            self.assertEqual(record['error_type'], type(error).__name__)

    def test_timeout_and_cancel_keep_diagnostics(self):
        robot = RobotClient('test', writes=True)
        robot.motion_stage = '前往预抓点'
        with patch(
            'saydo.robot.grasp_robot.socket.create_connection'
        ) as connect:
            connect.return_value.__enter__.return_value = Mock()
            with self.assertRaises(TimeoutError):
                robot.move_joint_pose(
                    [0] * 6, 20, threading.Event(), lambda: None, timeout=0
                )
            self.assertEqual(robot.motion_log[-1]['stage'], '前往预抓点')
            self.assertEqual(robot.motion_log[-1]['error_type'], 'TimeoutError')
            connect.reset_mock()
            cancelled = threading.Event()
            cancelled.set()
            with self.assertRaises(RuntimeError):
                robot.move_joint_pose([0] * 6, 20, cancelled, lambda: None)
            connect.assert_not_called()
            self.assertFalse(robot.motion_log[-1]['send_completed'])

    def test_bounded_responses(self):
        record, error = self.run_motion(
            [
                b'{"state":"other"}\n' * 40
                + b'{"command":"movej_p","receive_state":true}\n'
                + (
                    b'{"state":"current_trajectory_state","trajectory_state":tr'
                    b'ue}\n'
                )
            ]
        )
        self.assertIsNone(error)
        self.assertEqual(len(record['responses']), 32)
        self.assertEqual(record['responses_dropped'], 10)


if __name__ == '__main__':
    unittest.main()
