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

    def run_stream(self, on_frame=None, robot=None, real_time=False, **options):
        connection = Connection(self.time, on_frame)
        self.addCleanup(connection.close)
        robot = robot or grasp_robot.RobotClient('test', writes=True)

        def request(payload):
            # 机械臂替身：完全跟得上，实际关节角就是最近收到的那一帧。
            joint = (
                connection.frames[-1]['joint']
                if connection.frames
                else self.arm_joint_raw
            )
            return {'arm_state': {'joint': list(joint), 'err': [0]}}

        robot._request = mock.Mock(side_effect=request)
        if real_time:
            # 监视在另一个线程里按真实时间进行，这类测试用真实时钟。
            options.update(clock=time.monotonic, sleep=time.sleep)
        else:
            options.update(clock=self.time.clock, sleep=self.time.sleep)
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
        threads = set()

        def watchdog():
            calls.append(time.monotonic())
            threads.add(threading.get_ident())
            if len(calls) == 4:
                raise ValueError('离开监视范围')

        connection, record, error = self.run_stream(
            watchdog=watchdog, real_time=True
        )
        self.assertIsInstance(error, ValueError)
        self.assertIn('离开监视范围', str(error))
        # 约每 0.1 秒检查一次，而且不在发送循环所在的线程里。
        np.testing.assert_allclose(np.diff(calls), .1, atol=.05)
        self.assertNotIn(threading.get_ident(), threads)
        self.assertLess(self.joints(connection)[-1][0], END[0] - 1)
        # 监视不占用发送循环：节拍没有被打乱。
        self.assertLess(record['max_gap_s'], .03)

    def test_slow_watchdog_does_not_disturb_the_send_period(self):
        def watchdog():
            time.sleep(.04)

        connection, record, error = self.run_stream(
            watchdog=watchdog, real_time=True, hold_s=0.
        )
        self.assertIsNone(error)
        self.assertLess(record['max_gap_s'], .03)

    def test_arm_that_does_not_follow_stops_the_stream(self):
        connection, _, error = self.run_stream(
            actual_joint_deg=lambda: START, max_lag_deg=3., real_time=True
        )
        self.assertIn('没有跟上指令', str(error))
        # 指令走出 3 度多一点就开始减速，没有走到终点。
        self.assertLess(self.joints(connection)[-1][3], END[3] - 5)
        # 还没有读数时不判。
        _, _, error = self.run_stream(
            actual_joint_deg=lambda: None, real_time=True
        )
        self.assertIsNone(error)

    def test_actual_joints_are_read_from_the_controller_by_default(self):
        # 不另给读数来源时向控制器只读查询；替身跟得上，所以走完全程。
        connection, _, error = self.run_stream(real_time=True, hold_s=0.)
        self.assertIsNone(error)
        np.testing.assert_allclose(
            self.joints(connection)[-1], END, atol=1e-3
        )

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


class MoveThroughTests(unittest.TestCase):
    """连贯地经过一串关节目标：规划、限速和退回逐个运动。"""

    def setUp(self):
        self.robot = grasp_robot.RobotClient('test', writes=True)
        self.robot._request = mock.Mock(
            return_value={
                'arm_state': {
                    'joint': [int(v * 1000) for v in START], 'err': [0]
                }
            }
        )
        self.cancelled = threading.Event()
        # 沿一条缓变的路径密集取的 10 个目标。
        self.targets = [
            np.rint(
                (np.array(START) + (np.array(END) - START) * k / 10) * 1000
            ).astype(int).tolist()
            for k in range(1, 11)
        ]

    def move(self, targets, speed=25):
        with mock.patch.object(
            self.robot, 'stream_joints'
        ) as stream, mock.patch.object(self.robot, 'move_joints') as step:

            def streamed(*args, **kwargs):
                self.robot.motion_log.append({})

            stream.side_effect = streamed
            self.robot.move_joints_through(
                targets, speed, self.cancelled, self.watchdog
            )
        return stream, step

    def watchdog(self):
        pass

    def test_dense_targets_become_one_smooth_high_follow_stream(self):
        stream, step = self.move(self.targets)
        step.assert_not_called()
        position, duration_s, cancelled, watchdog = stream.call_args.args
        self.assertTrue(stream.call_args.kwargs['follow'])
        self.assertIs(cancelled, self.cancelled)
        self.assertEqual(watchdog, self.watchdog)
        # 从当前关节角出发，到最后一个目标结束，途中经过每个目标。
        np.testing.assert_allclose(position(0.), START, atol=1e-9)
        np.testing.assert_allclose(position(duration_s), END, atol=1e-3)
        dense = np.array(
            [position(t) for t in np.arange(0., duration_s, .002)]
        )
        for target in self.targets:
            self.assertLess(
                np.abs(dense - np.array(target) / 1000).max(axis=1).min(),
                .05,
            )
        # 25% 对应每秒 40 度、每二次方秒 120 度的上限。
        velocity = np.diff(dense, axis=0) / .002
        self.assertLessEqual(np.abs(velocity).max(), 40.01)
        self.assertLessEqual(
            np.abs(np.diff(velocity, axis=0) / .002).max(), 121.
        )
        record = self.robot.motion_log[-1]
        self.assertEqual(record['through_targets'], 10)
        self.assertLess(record['path_deviation_deg'], .05)

    def test_speed_setting_scales_the_limits_up_to_the_tested_cap(self):
        durations = {}
        for speed in (5, 25, 100):
            stream, _ = self.move(self.targets, speed)
            durations[speed] = stream.call_args.args[1]
        self.assertGreater(durations[5], durations[25] * 1.5)
        # 超过 25% 不再加快：更高的速度没有实测过。
        self.assertEqual(durations[100], durations[25])

    def test_sparse_zigzag_falls_back_to_separate_joint_moves(self):
        # 先只转第 3 关节 20 度，再转回来改转第 5 关节：路径拐了个急弯。
        zigzag = []
        for joint in (2, 4):
            target = np.array(START)
            target[joint] += 20
            zigzag.append(np.rint(target * 1000).astype(int).tolist())
        with self.assertLogs('saydo.robot.grasp_robot', 'WARNING') as logs:
            stream, step = self.move(zigzag)
        stream.assert_not_called()
        self.assertEqual([c.args[0] for c in step.call_args_list], zigzag)
        self.assertIn('改为逐个关节运动', logs.output[0])

    def test_already_there_sends_nothing(self):
        stream, step = self.move([[int(v * 1000) for v in START]])
        stream.assert_not_called()
        step.assert_not_called()

    def test_requires_writes_and_valid_targets(self):
        for targets, speed in (
            ([], 25), ([[0] * 6], 25), ([[0.] * 7], 25), (self.targets, 0)
        ):
            with self.assertRaisesRegex(ValueError, '未启用或参数无效'):
                self.robot.move_joints_through(
                    targets, speed, self.cancelled, self.watchdog
                )
        with self.assertRaisesRegex(ValueError, '未启用或参数无效'):
            grasp_robot.RobotClient('test').move_joints_through(
                self.targets, 25, self.cancelled, self.watchdog
            )


if __name__ == '__main__':
    unittest.main()
