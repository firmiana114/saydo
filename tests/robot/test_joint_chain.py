"""连续关节运动的发送与完成判定：替身只在控制器连接这一处。"""

import json
import threading
import unittest
from unittest import mock

from saydo.robot import grasp_robot

ACCEPTED = {'command': 'movej', 'receive_state': True}
PASSED = {
    'state': 'current_trajectory_state',
    'trajectory_state': True,
    'trajectory_connect': 1,
}
DONE = {
    'state': 'current_trajectory_state',
    'trajectory_state': True,
    'trajectory_connect': 0,
}
TARGETS = [[1000 * k] + [0] * 6 for k in (1, 2, 3)]


class FakeController:
    """控制器替身：每收到一条指令，把预先安排好的下一批应答放出来。"""

    def __init__(self, replies_per_command):
        self.replies = list(replies_per_command)
        self.sent = []
        self.pending = []

    def sendall(self, data):
        self.sent.append(json.loads(data))
        batch = self.replies.pop(0) if self.replies else []
        if batch:
            self.pending.append(
                ''.join(
                    json.dumps(reply) + '\r\n' for reply in batch
                ).encode()
            )

    def recv(self, size):
        if not self.pending:
            raise grasp_robot.socket.timeout()
        return self.pending.pop(0)

    def settimeout(self, seconds):
        pass


class JointChainTests(unittest.TestCase):
    def run_chain(self, replies, targets=TARGETS, **options):
        controller = FakeController(replies)
        robot = grasp_robot.RobotClient('test', writes=True)
        with mock.patch(
            'saydo.robot.grasp_robot.socket.create_connection'
        ) as connect:
            connect.return_value.__enter__.return_value = controller
            try:
                robot.move_joints_chain(
                    targets, 10, options.pop('cancelled', threading.Event()),
                    options.pop('watchdog', lambda: None), **options,
                )
                error = None
            except Exception as caught:
                error = caught
        return controller, robot.motion_log[-1], error

    def test_all_but_the_last_command_are_connected(self):
        controller, record, error = self.run_chain(
            [[ACCEPTED], [ACCEPTED], [ACCEPTED, PASSED, PASSED, DONE]]
        )
        self.assertIsNone(error)
        self.assertEqual(
            controller.sent,
            [
                {
                    'command': 'movej', 'joint': joint, 'v': 10, 'r': 0,
                    'trajectory_connect': connect,
                }
                for joint, connect in zip(TARGETS, (1, 1, 0))
            ],
        )
        self.assertEqual(record['outcome'], 'completed')
        self.assertEqual(record['accepted_count'], 3)
        self.assertEqual(record['command']['command'], 'movej_chain')
        # 控制器的每条应答都留了下来。
        self.assertEqual(len(record['responses']), 6)
        self.assertIn(
            '"trajectory_connect": 0', record['completion_reply']['raw']
        )

    def test_blend_radius_applies_to_every_point_but_the_last(self):
        controller, record, error = self.run_chain(
            [[ACCEPTED], [ACCEPTED], [ACCEPTED, DONE]], blend_percent=40
        )
        self.assertIsNone(error)
        self.assertEqual([c['r'] for c in controller.sent], [40, 40, 0])
        self.assertEqual(record['command']['r'], 40)
        robot = grasp_robot.RobotClient('test', writes=True)
        for bad in (-1, 101, 2.5):
            with self.assertRaisesRegex(ValueError, '未启用或参数无效'):
                robot.move_joints_chain(
                    TARGETS, 10, threading.Event(), lambda: None,
                    blend_percent=bad,
                )

    def test_next_command_waits_for_the_previous_acceptance(self):
        with mock.patch.object(grasp_robot, 'ACCEPTANCE_TIMEOUT_S', .2):
            controller, record, error = self.run_chain([[]])
        # 第一条没有得到接受确认：不发第二条，超时报错。
        self.assertEqual(len(controller.sent), 1)
        self.assertIsInstance(error, TimeoutError)
        self.assertIn('已接受 0 条', str(error))
        self.assertEqual(record['outcome'], 'failed')

    def test_rejected_command_stops_the_chain(self):
        refused = {'command': 'movej', 'receive_state': False}
        controller, record, error = self.run_chain([[ACCEPTED], [refused]])
        self.assertEqual(len(controller.sent), 2)
        self.assertIsInstance(error, RuntimeError)
        self.assertIn('拒绝第 2 条', str(error))

    def test_failed_trajectory_and_early_completion_are_errors(self):
        failed = {**DONE, 'trajectory_state': False}
        _, _, error = self.run_chain(
            [[ACCEPTED], [ACCEPTED], [ACCEPTED, failed]]
        )
        self.assertIn('没有成功完成', str(error))
        _, _, error = self.run_chain([[ACCEPTED, DONE]])
        self.assertIn('只接受了 1 条指令', str(error))

    def test_single_target_is_an_ordinary_joint_move(self):
        controller, record, error = self.run_chain(
            [[ACCEPTED, DONE]], targets=TARGETS[:1]
        )
        self.assertIsNone(error)
        self.assertEqual(controller.sent[0]['trajectory_connect'], 0)

    def test_cancel_watchdog_and_invalid_arguments(self):
        cancelled = threading.Event()
        cancelled.set()
        controller, _, error = self.run_chain([], cancelled=cancelled)
        self.assertIn('取消', str(error))
        self.assertEqual(controller.sent, [])

        def watchdog():
            raise ValueError('离开监视范围')

        controller, _, error = self.run_chain([], watchdog=watchdog)
        self.assertIn('离开监视范围', str(error))
        self.assertEqual(controller.sent, [])
        robot = grasp_robot.RobotClient('test', writes=True)
        for targets, speed in (
            ([], 10), ([[0] * 6], 10), ([[0.] * 7], 10), (TARGETS, 0)
        ):
            with self.assertRaisesRegex(ValueError, '未启用或参数无效'):
                robot.move_joints_chain(
                    targets, speed, threading.Event(), lambda: None
                )
        with self.assertRaisesRegex(ValueError, '未启用或参数无效'):
            grasp_robot.RobotClient('test').move_joints_chain(
                TARGETS, 10, threading.Event(), lambda: None
            )


if __name__ == '__main__':
    unittest.main()
