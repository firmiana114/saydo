"""实际轨迹的记录与分析：替身只在控制器连接这一处。"""

import json
import unittest

import numpy as np

from saydo.robot import collision_model
from saydo.robot import trajectory_recorder

TOOL = np.eye(4)
TOOL[2, 3] = .2275


class FakeConnection:
    """控制器替身：每次查询返回下一组关节角，用完后断开。"""

    def __init__(self, joints_raw):
        self.joints_raw = list(joints_raw)
        self.requests = []

    def __enter__(self):
        return self

    def __exit__(self, *_):
        return False

    def settimeout(self, seconds):
        pass

    def sendall(self, data):
        self.requests.append(json.loads(data))

    def makefile(self, mode):
        return self

    def readline(self, limit):
        if not self.joints_raw:
            return b''
        state = {'joint': self.joints_raw.pop(0), 'pose': [1, 2, 3, 0, 0, 0]}
        return (json.dumps({'arm_state': state}) + '\r\n').encode()


def samples_along(times, first_joint_deg):
    return [
        {'t_s': t, 'joint_deg': [value] + [0.] * 6}
        for t, value in zip(times, first_joint_deg)
    ]


class RecorderTests(unittest.TestCase):
    def test_polls_read_only_and_keeps_samples_after_disconnect(self):
        connection = FakeConnection([[1000 * k] + [0] * 6 for k in range(5)])
        ticks = iter(np.arange(0., 100., .25))
        recorder = trajectory_recorder.TrajectoryRecorder(
            ('host', 8080),
            period_s=0.,
            connect=lambda address, timeout: connection,
            clock=lambda: next(ticks),
        )
        recorder.start()
        recorder._thread.join(5)
        samples = recorder.stop()
        self.assertEqual(
            [s['joint_deg'][0] for s in samples], [0., 1., 2., 3., 4.]
        )
        self.assertEqual(samples[0]['pose'], [1, 2, 3, 0, 0, 0])
        self.assertTrue(
            all(a['t_s'] < b['t_s'] for a, b in zip(samples, samples[1:]))
        )
        # 只发只读的状态查询；连接断开后已有的样本保留，原因记下来。
        self.assertEqual(
            {r['command'] for r in connection.requests},
            {'get_current_arm_state'},
        )
        self.assertIn('控制器断开连接', recorder.error)

    def test_connection_failure_is_reported_not_raised(self):
        def refuse(address, timeout):
            raise OSError('No route to host')

        recorder = trajectory_recorder.TrajectoryRecorder(
            ('host', 8080), connect=refuse
        )
        recorder.start()
        self.assertEqual(recorder.stop(), [])
        self.assertIn('No route to host', recorder.error)


class AnalysisTests(unittest.TestCase):
    def test_deviation_from_the_planned_joint_polyline(self):
        path = [[0.] * 7, [10.] + [0.] * 6, [10., 10.] + [0.] * 5]
        actual = [
            [5.] + [0.] * 6,            # 在第一段上
            [5., 2.] + [0.] * 5,        # 离第一段 2 度
            [9., 1.] + [0.] * 5,        # 抄近路拐过第一个路点
            [10., 4.] + [0.] * 5,       # 在第二段上
            [12., 12.] + [0.] * 5,      # 冲过终点 2 度
        ]
        np.testing.assert_allclose(
            trajectory_recorder.deviation_from_polyline(actual, path),
            [0., 2., 1., 0., 2.],
            atol=1e-9,
        )
        # 抄近路时离中间那个路点最近也有 1 度；终点和起点都到过。
        np.testing.assert_allclose(
            trajectory_recorder.closest_approach_deg(actual, path[1:]),
            [1., 2.],
            atol=1e-9,
        )

    def test_tcp_deviation_from_a_straight_line(self):
        start = [0., 30., 0., 60., 0., 0., 0.]
        end = [20., 30., 0., 60., 0., 0., 0.]
        tips = [
            (collision_model.link_frames(q)[7] @ TOOL)[:3, 3]
            for q in (start, end)
        ]
        middle = [10., 30., 0., 60., 0., 0., 0.]
        deviation = trajectory_recorder.tcp_line_deviation_m(
            [start, middle, end], TOOL, tips[0], tips[1]
        )
        # 第 1 关节转动时夹爪走圆弧：两端在线上，中点鼓出去。
        self.assertAlmostEqual(deviation[0], 0., places=9)
        self.assertAlmostEqual(deviation[2], 0., places=9)
        radius = float(np.hypot(*tips[0][:2]))
        self.assertAlmostEqual(
            deviation[1], radius * (1 - np.cos(np.radians(10.))), places=6
        )

    def test_stops_only_count_between_first_and_last_motion(self):
        times = np.arange(0., 2.01, .01)
        position = np.piecewise(
            times,
            [times < .3, (times >= .3) & (times < .8), (times >= .8) & (
                times < 1.), (times >= 1.) & (times < 1.5), times >= 1.5],
            [
                0.,
                lambda t: (t - .3) * 20,
                10.,
                lambda t: 10 + (t - 1.) * 20,
                20.,
            ],
        )
        stops = trajectory_recorder.stops_during_motion(
            samples_along(times, position)
        )
        # 开头没动和结尾停下都不算；中间那 0.2 秒算一次。
        self.assertEqual(len(stops), 1)
        self.assertAlmostEqual(stops[0]['start_s'], .8, delta=.02)
        self.assertAlmostEqual(stops[0]['duration_s'], .2, delta=.03)
        _, speed = trajectory_recorder.speed_deg_s(
            samples_along(times, position)
        )
        self.assertAlmostEqual(float(speed.max()), 20., places=6)

    def test_continuous_motion_has_no_stops(self):
        times = np.arange(0., 1.01, .01)
        self.assertEqual(
            trajectory_recorder.stops_during_motion(
                samples_along(times, times * 15)
            ),
            [],
        )
        self.assertEqual(trajectory_recorder.stops_during_motion([]), [])
        still = samples_along(times, np.zeros(len(times)))
        self.assertEqual(trajectory_recorder.stops_during_motion(still), [])


if __name__ == '__main__':
    unittest.main()
