"""角度透传的发送循环：替身只在控制器连接和时钟两处。"""

import json
import socket
import threading
import time
import unittest
from unittest import mock

import numpy as np

from saydo.robot import grasp_robot
from saydo.robot import joint_trajectory

START = [0., 10., 0., 30., 0., 0., 0.]
END = [10., 20., 0., 50., 0., 0., 0.]


class FakeClock:
    """时钟替身：只有 sleep 会让时间前进。"""

    def __init__(self):
        self.now = 100.

    def clock(self):
        return self.now

    def sleep(self, seconds):
        self.now += seconds


class Connection:
    """控制器替身：一对真实的本机套接字的一端。

    用真实套接字是为了让“有没有数据可读”“带超时的读取”这些行为与
    实际一致；另一端 `controller` 留给测试扮演控制器发应答。
    """

    def __init__(self, clock, on_frame=None):
        self.clock = clock
        self.on_frame = on_frame
        self.frames = []
        self.times = []
        self.local, self.controller = socket.socketpair()

    def close(self):
        self.local.close()
        self.controller.close()

    def fileno(self):
        return self.local.fileno()

    def settimeout(self, seconds):
        self.local.settimeout(seconds)

    def sendall(self, data):
        self.local.sendall(data)
        self.controller.recv(65536)
        self.frames.append(json.loads(data))
        self.times.append(self.clock.now)
        if self.on_frame is not None:
            self.on_frame(len(self.frames))

    def recv(self, size, flags=0):
        return self.local.recv(size, flags)


class JointStreamTests(unittest.TestCase):
    def setUp(self):
        self.trajectory = joint_trajectory.plan_through([START, END], 20., 60.)
        self.time = FakeClock()
        self.arm_joint_raw = [int(v * 1000) for v in START]

    def run_stream(self, on_frame=None, robot=None, **options):
        connection = Connection(self.time, on_frame)
        self.addCleanup(connection.close)
        robot = robot or grasp_robot.RobotClient('test', writes=True)
        robot._request = mock.Mock(
            return_value={
                'arm_state': {'joint': self.arm_joint_raw, 'err': [0]}
            }
        )
        with mock.patch(
            'saydo.robot.grasp_robot.socket.create_connection'
        ) as connect:
            connect.return_value.__enter__.return_value = connection
            try:
                robot.stream_joints(
                    options.pop('position', self.trajectory.position_deg),
                    options.pop('duration_s', self.trajectory.duration_s),
                    options.pop('cancelled', threading.Event()),
                    options.pop('watchdog', lambda: None),
                    clock=self.time.clock,
                    sleep=self.time.sleep,
                    **options,
                )
                error = None
            except Exception as caught:
                error = caught
        record = robot.motion_log[-1] if robot.motion_log else None
        return connection, record, error

    def joints(self, connection):
        return np.array([f['joint'] for f in connection.frames]) / 1000

    def test_frames_follow_the_trajectory_at_a_fixed_period(self):
        connection, record, error = self.run_stream(
            hold_s=.1, record_frames=True
        )
        self.assertIsNone(error)
        sent = self.joints(connection)
        # 第一帧是起点，最后几帧停在终点上。
        np.testing.assert_allclose(sent[0], START, atol=1e-3)
        np.testing.assert_allclose(sent[-1], END, atol=1e-3)
        np.testing.assert_allclose(sent[-5], END, atol=1e-3)
        # 每 10 毫秒一帧，第 k 帧就是轨迹在 k×10 毫秒时的位置。
        np.testing.assert_allclose(np.diff(connection.times), .01, atol=1e-9)
        for index in (10, 50, len(sent) // 2):
            np.testing.assert_allclose(
                sent[index], self.trajectory.position_deg(index * .01),
                atol=1e-3,
            )
        self.assertEqual(
            {
                (f['command'], f['follow'], f['expand'])
                for f in connection.frames
            },
            {('movej_canfd', False, 0)},
        )
        self.assertTrue(
            all(type(v) is int for f in connection.frames for v in f['joint'])
        )
        # 走完全程再多发 0.1 秒的终点。
        total = self.trajectory.duration_s + .1
        self.assertAlmostEqual(len(sent) * .01, total, delta=.03)
        self.assertEqual(record['outcome'], 'completed')
        self.assertEqual(record['frames_sent'], len(sent))
        self.assertEqual(len(record['frames']), len(sent))
        self.assertAlmostEqual(record['max_gap_s'], .01, places=6)

    def test_sending_never_waits_for_replies(self):
        # 控制器对透传指令不应答。带超时的连接上直接读会等到超时，
        # 每帧白等 0.2 秒；这里用真实时间确认没有这种等待。
        began = time.monotonic()
        connection, record, error = self.run_stream(hold_s=0.)
        self.assertIsNone(error)
        self.assertGreater(len(connection.frames), 100)
        self.assertLess(time.monotonic() - began, 2.)
        self.assertEqual(record['responses'], [])

    def test_controller_replies_are_recorded(self):
        def on_frame(count):
            if count == 5:
                connection_box[0].controller.sendall(b'{"state":"x"}\r\n')

        connection_box = []
        original = Connection.__init__

        def remember(instance, *args, **kwargs):
            original(instance, *args, **kwargs)
            connection_box.append(instance)

        with mock.patch.object(Connection, '__init__', remember):
            connection, record, error = self.run_stream(on_frame, hold_s=0.)
        self.assertIsNone(error)
        self.assertEqual(len(record['responses']), 1)
        self.assertIn('"state":"x"', record['responses'][0]['raw'])

    def test_controller_disconnect_stops_the_stream(self):
        def on_frame(count):
            if count == 5:
                connection_box[0].controller.close()

        connection_box = []
        original = Connection.__init__

        def remember(instance, *args, **kwargs):
            original(instance, *args, **kwargs)
            connection_box.append(instance)

        with mock.patch.object(Connection, '__init__', remember):
            connection, record, error = self.run_stream(on_frame)
        self.assertIsNotNone(error)
        self.assertLess(len(connection.frames), 10)

    def test_high_follow_and_period_are_passed_through(self):
        connection, record, error = self.run_stream(
            period_s=.005, follow=True, hold_s=0.
        )
        self.assertIsNone(error)
        self.assertTrue(all(f['follow'] is True for f in connection.frames))
        np.testing.assert_allclose(np.diff(connection.times), .005, atol=1e-9)
        self.assertNotIn('frames', record)

    def test_start_must_match_the_arm(self):
        self.arm_joint_raw[1] += 600
        connection, record, error = self.run_stream()
        self.assertIsInstance(error, ValueError)
        self.assertIn('相差 0.60 度', str(error))
        self.assertEqual(connection.frames, [])
        self.assertIsNone(record)

    def test_cancel_slows_down_along_the_path_before_stopping(self):
        cancelled = threading.Event()

        def on_frame(count):
            if count == 60:
                cancelled.set()

        connection, record, error = self.run_stream(
            on_frame, cancelled=cancelled, brake_s=.2
        )
        self.assertIn('取消', str(error))
        sent = self.joints(connection)
        steps = np.abs(np.diff(sent, axis=0)).max(axis=1)
        # 取消后不是立刻停发：约 0.2 秒内一帧比一帧走得少，最后停住。
        after = steps[60:]
        self.assertGreater(len(after), 15)
        self.assertLess(len(after), 25)
        self.assertTrue((np.diff(after) <= 1e-9).all())
        self.assertLess(after[-1], after[0] * .1)
        # 停在半路，而且始终在原来的轨迹上。
        self.assertLess(sent[-1][0], END[0] - 1)
        along = [
            np.abs(
                self.trajectory.sample(.001) - point
            ).max(axis=1).min()
            for point in sent[::7]
        ]
        self.assertLess(max(along), .02)
        self.assertEqual(record['outcome'], 'failed')

    def test_watchdog_error_also_brakes_and_is_passed_on(self):
        calls = []

        def watchdog():
            calls.append(self.time.now)
            if len(calls) == 4:
                raise ValueError('离开监视范围')

        connection, _, error = self.run_stream(watchdog=watchdog)
        self.assertIsInstance(error, ValueError)
        self.assertIn('离开监视范围', str(error))
        # 每 0.1 秒检查一次。
        np.testing.assert_allclose(np.diff(calls), .1, atol=.011)
        self.assertLess(self.joints(connection)[-1][0], END[0] - 1)

    def test_arm_that_does_not_follow_stops_the_stream(self):
        connection, _, error = self.run_stream(
            actual_joint_deg=lambda: START, max_lag_deg=3.
        )
        self.assertIn('没有跟上指令', str(error))
        # 指令走出 3 度多一点就开始减速，没有走到终点。
        self.assertLess(self.joints(connection)[-1][3], END[3] - 5)
        # 还没有读数时不判。
        _, _, error = self.run_stream(actual_joint_deg=lambda: None)
        self.assertIsNone(error)

    def test_jump_between_frames_is_refused(self):
        def jumping(time_s):
            position = np.array(self.trajectory.position_deg(time_s))
            if time_s > .5:
                position[2] += 5.
            return position

        connection, _, error = self.run_stream(position=jumping)
        self.assertIn('相邻两帧关节变化', str(error))
        self.assertTrue((self.joints(connection)[:, 2] == 0).all())

    def test_stalled_sender_stops_instead_of_catching_up(self):
        def on_frame(count):
            if count == 40:
                self.time.now += .2

        connection, _, error = self.run_stream(on_frame)
        self.assertIn('超过 50 毫秒', str(error))
        self.assertEqual(len(connection.frames), 40)

    def test_short_delay_keeps_the_planned_speed(self):
        def on_frame(count):
            if count == 40:
                self.time.now += .02

        connection, _, error = self.run_stream(on_frame, hold_s=0.)
        self.assertIsNone(error)
        sent = self.joints(connection)
        # 晚到的那一帧按实际时刻取位置：步子大一些，但不超过三个间隔
        # 的量，之后恢复正常。
        steps = np.abs(np.diff(sent, axis=0)).max(axis=1)
        self.assertGreater(steps[39], steps[38] * 2)
        self.assertLess(steps[39], steps[38] * 3.5)
        self.assertLess(steps[41], steps[38] * 1.5)

    def test_requires_writes_and_valid_arguments(self):
        robot = grasp_robot.RobotClient('test')
        _, record, error = self.run_stream(robot=robot)
        self.assertIn('未启用或参数无效', str(error))
        for options in (
            {'duration_s': 0.}, {'period_s': 0.}, {'period_s': .04},
            {'brake_s': 0.}, {'hold_s': -1.},
        ):
            connection, _, error = self.run_stream(**options)
            self.assertIn('未启用或参数无效', str(error))
            self.assertEqual(connection.frames, [])


if __name__ == '__main__':
    unittest.main()
