"""带时间的关节角轨迹：纯计算，没有替身。"""

import unittest

import numpy as np

from saydo.robot import joint_trajectory

WAYPOINTS = [
    [0., 10., 0., 30., 0., 0., 0.],
    [4., 14., 1., 33., 0., -2., 5.],
    [9., 17., 3., 38., 0., -3., 9.],
    [15., 19., 6., 45., 0., -3., 12.],
    [22., 20., 10., 54., 0., -2., 14.],
]


class JointTrajectoryTests(unittest.TestCase):
    def plan(self, speed=20., acceleration=50., **options):
        return joint_trajectory.plan_through(
            WAYPOINTS, speed, acceleration, **options
        )

    def test_starts_and_ends_at_rest_on_the_first_and_last_waypoint(self):
        trajectory = self.plan()
        np.testing.assert_allclose(
            trajectory.position_deg(0.), WAYPOINTS[0], atol=1e-9
        )
        np.testing.assert_allclose(
            trajectory.position_deg(trajectory.duration_s), WAYPOINTS[-1],
            atol=1e-9,
        )
        # 起点之前、终点之后都停在端点上。
        np.testing.assert_allclose(
            trajectory.position_deg(-1.), WAYPOINTS[0], atol=1e-9
        )
        np.testing.assert_allclose(
            trajectory.position_deg(trajectory.duration_s + 5), WAYPOINTS[-1],
            atol=1e-9,
        )
        # 起步和收尾的头 10 毫秒几乎不动：速度、加速度都从零开始。
        early = trajectory.position_deg(.01) - np.array(WAYPOINTS[0])
        late = np.array(WAYPOINTS[-1]) - trajectory.position_deg(
            trajectory.duration_s - .01
        )
        self.assertLess(np.abs(early).max(), 1e-3)
        self.assertLess(np.abs(late).max(), 1e-3)

    def test_passes_exactly_through_every_waypoint_in_order(self):
        trajectory = self.plan()
        dense = trajectory.sample(.0005)
        last = -1
        for point in WAYPOINTS:
            distance = np.abs(dense - point).max(axis=1)
            index = int(np.argmin(distance))
            self.assertLess(distance[index], .01)
            self.assertGreater(index, last)
            last = index

    def test_speed_and_acceleration_stay_within_the_limits(self):
        for speed, acceleration in ((20., 50.), (60., 30.), (5., 500.)):
            trajectory = self.plan(speed, acceleration)
            step = .001
            dense = trajectory.sample(step)[:-1]
            velocity = np.diff(dense, axis=0) / step
            rate = np.diff(velocity, axis=0) / step
            self.assertLessEqual(np.abs(velocity).max(), speed * 1.001)
            self.assertLessEqual(np.abs(rate).max(), acceleration * 1.01)
            # 至少有一项用到了上限附近：没有白白走得更慢。
            self.assertTrue(
                np.abs(velocity).max() > speed * .97
                or np.abs(rate).max() > acceleration * .95
            )
            self.assertAlmostEqual(
                trajectory.peak_speed_deg_s, np.abs(velocity).max(),
                delta=speed * .01,
            )
            self.assertAlmostEqual(
                trajectory.peak_acceleration_deg_s2, np.abs(rate).max(),
                delta=acceleration * .02,
            )

    def test_tighter_limits_take_longer_and_minimum_duration_holds(self):
        self.assertGreater(
            self.plan(10., 50.).duration_s, self.plan(20., 50.).duration_s
        )
        self.assertGreater(
            self.plan(200., 20.).duration_s, self.plan(200., 80.).duration_s
        )
        self.assertEqual(
            self.plan(1e6, 1e9, min_duration_s=.7).duration_s, .7
        )

    def test_samples_end_exactly_on_the_target(self):
        trajectory = self.plan()
        frames = trajectory.sample(.01)
        np.testing.assert_allclose(frames[0], WAYPOINTS[0], atol=1e-9)
        np.testing.assert_allclose(frames[-1], WAYPOINTS[-1], atol=1e-9)
        self.assertEqual(
            len(frames), int(np.ceil(trajectory.duration_s / .01)) + 1
        )
        # 相邻两帧的变化不超过“最高转速 × 间隔”。
        self.assertLessEqual(
            np.abs(np.diff(frames, axis=0)).max(),
            trajectory.peak_speed_deg_s * .01 * 1.001,
        )

    def test_even_knots_spread_the_waypoints_evenly_along_the_path(self):
        knots = np.linspace(0., 1., len(WAYPOINTS))
        trajectory = self.plan(knots=knots)
        # 走到一半时间时正好走完一半路程，也就是中间那个路点。
        np.testing.assert_allclose(
            trajectory.position_deg(trajectory.duration_s / 2), WAYPOINTS[2],
            atol=1e-9,
        )

    def test_invalid_inputs_are_refused(self):
        for bad in (
            WAYPOINTS[:1], [[0.] * 6, [1.] * 6], [[0.] * 7, [np.nan] * 7]
        ):
            with self.assertRaisesRegex(ValueError, '至少需要两个路点'):
                joint_trajectory.plan_through(bad, 20., 50.)
        with self.assertRaisesRegex(ValueError, '重合'):
            joint_trajectory.plan_through(
                [WAYPOINTS[0], WAYPOINTS[0], WAYPOINTS[1]], 20., 50.
            )
        for speed, acceleration in ((0., 50.), (20., -1.)):
            with self.assertRaisesRegex(ValueError, '必须是正数'):
                self.plan(speed, acceleration)
        for knots in ([0., .5, 1.], [0., .2, .2, .7, 1.], [.1, .3, .5, .7, 1.]):
            with self.assertRaisesRegex(ValueError, 'knots'):
                self.plan(knots=knots)

    def test_progress_is_smooth_and_monotonic(self):
        tau = np.linspace(-.2, 1.2, 141)
        value = joint_trajectory.progress(tau)
        self.assertEqual(value[0], 0.)
        self.assertEqual(value[-1], 1.)
        self.assertTrue((np.diff(value) >= 0).all())
        self.assertAlmostEqual(float(joint_trajectory.progress(.5)), .5)


if __name__ == '__main__':
    unittest.main()
