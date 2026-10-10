"""出画检查：按估计位姿把模型表面投影到画面上，统计画面外的面积占比。"""

import unittest

import numpy as np

from saydo.perception import frame_coverage

# 与 1 臂 D435i 在 640×360 下的内参同量级。
CAMERA_MATRIX = np.array([
    [454., 0., 320.],
    [0., 454., 180.],
    [0., 0., 1.],
])
WIDTH_PX = 640
HEIGHT_PX = 360
# 150 毫米试管沿物品 Z 轴的 151 个等距取样点，每点代表相同的表面积。
TUBE_POINTS_OBJECT_M = np.column_stack([
    np.zeros(151), np.zeros(151), np.linspace(0., .15, 151)
])
TUBE_WEIGHTS = np.ones(151)


def tube_pose(row_start_px, depth_m=.4):
    """试管横在画面里、轴线沿画面竖直方向，起点落在给定的像素行。"""
    camera_T_object = np.eye(4)
    # 物品 Z 轴对到相机 Y 轴（画面向下），物品 Y 轴对到相机 -Z。
    camera_T_object[:3, :3] = np.array([
        [1., 0., 0.],
        [0., 0., 1.],
        [0., -1., 0.],
    ])
    camera_T_object[:3, 3] = [
        0., (row_start_px - 180.) * depth_m / 454., depth_m
    ]
    return camera_T_object


def measure(camera_T_object, **options):
    return frame_coverage.measure(
        camera_T_object,
        TUBE_POINTS_OBJECT_M,
        TUBE_WEIGHTS,
        CAMERA_MATRIX,
        WIDTH_PX,
        HEIGHT_PX,
        **options,
    )


class MeasureTests(unittest.TestCase):
    def test_object_fully_in_frame_has_nothing_outside(self):
        coverage = measure(tube_pose(100.))
        self.assertTrue(coverage['good'])
        self.assertEqual(coverage['outside_percent'], 0.)
        self.assertEqual(coverage['outside_percent_by_edge'], {})

    def test_reports_fraction_and_edge_when_one_end_leaves_the_frame(self):
        # 0.4 米处 150 毫米约 170 像素；从第 230 行起，末端 40 像素出画。
        coverage = measure(tube_pose(230.))
        self.assertFalse(coverage['good'])
        self.assertAlmostEqual(coverage['outside_percent'], 23.8, delta=1.)
        self.assertEqual(list(coverage['outside_percent_by_edge']), ['bottom'])
        self.assertAlmostEqual(
            coverage['outside_percent_by_edge']['bottom'],
            coverage['outside_percent'],
        )

    def test_crop_as_small_as_the_incident_is_rejected(self):
        # 2026-10-09 15:35 那次试管有 5.3% 的表面在画面下边之外。
        coverage = measure(tube_pose(360. - .947 * 170.25))
        self.assertAlmostEqual(coverage['outside_percent'], 5.3, delta=.7)
        self.assertFalse(coverage['good'])

    def test_clipped_corner_below_limit_is_accepted(self):
        # 只有末端一个取样点（0.66%）出画，相当于物品的一个角被切到。
        coverage = measure(tube_pose(360. - 170.25 + .5))
        self.assertGreater(coverage['outside_percent'], 0.)
        self.assertLess(coverage['outside_percent'], 1.)
        self.assertTrue(coverage['good'])

    def test_weights_make_result_independent_of_point_density(self):
        # 把出画那一端的取样点加密十倍、权重相应缩小，面积占比不变。
        dense_end = np.column_stack([
            np.zeros(300), np.zeros(300), np.linspace(.1205, .15, 300)
        ])
        points = np.vstack([TUBE_POINTS_OBJECT_M[:121], dense_end])
        weights = np.concatenate([np.ones(121), np.full(300, .1)])
        coverage = frame_coverage.measure(
            tube_pose(230.),
            points,
            weights,
            CAMERA_MATRIX,
            WIDTH_PX,
            HEIGHT_PX,
        )
        self.assertAlmostEqual(
            coverage['outside_percent'],
            measure(tube_pose(230.))['outside_percent'],
            delta=1.,
        )

    def test_corner_overlap_counts_once_in_total(self):
        camera_T_object = tube_pose(230.)
        # 整体右移到同时越过右边和下边。
        camera_T_object[0, 3] = 330. * .4 / 454.
        coverage = measure(camera_T_object)
        self.assertEqual(coverage['outside_percent'], 100.)
        self.assertEqual(coverage['outside_percent_by_edge']['right'], 100.)
        self.assertLess(coverage['outside_percent_by_edge']['bottom'], 100.)

    def test_points_behind_camera_count_as_outside(self):
        coverage = measure(tube_pose(100., depth_m=-.4))
        self.assertEqual(coverage['outside_percent'], 100.)
        self.assertEqual(coverage['outside_percent_by_edge'], {})
        self.assertFalse(coverage['good'])

    def test_limit_is_inclusive_and_recorded(self):
        coverage = measure(tube_pose(230.), limit_percent=50.)
        self.assertTrue(coverage['good'])
        self.assertEqual(coverage['limit_percent'], 50.)

    def test_mismatched_or_empty_weights_are_rejected(self):
        for points, weights in (
            (TUBE_POINTS_OBJECT_M, np.ones(3)),
            (np.zeros((4, 2)), np.ones(4)),
            (TUBE_POINTS_OBJECT_M, np.zeros(151)),
        ):
            with self.assertRaises(ValueError):
                frame_coverage.measure(
                    tube_pose(100.),
                    points,
                    weights,
                    CAMERA_MATRIX,
                    WIDTH_PX,
                    HEIGHT_PX,
                )


class RequireInsideTests(unittest.TestCase):
    def test_passes_silently_when_inside(self):
        frame_coverage.require_inside('tube', measure(tube_pose(100.)))

    def test_error_names_object_amount_edge_and_limit(self):
        with self.assertRaises(ValueError) as raised:
            frame_coverage.require_inside('tube', measure(tube_pose(230.)))
        message = str(raised.exception)
        self.assertIn('tube', message)
        self.assertRegex(message, r'有 2\d\.\d% 的表面在画面之外')
        self.assertIn('下边', message)
        self.assertIn('上限 1%', message)

    def test_error_without_edge_when_object_is_behind_camera(self):
        with self.assertRaises(ValueError) as raised:
            frame_coverage.require_inside(
                'rack', measure(tube_pose(100., depth_m=-.4))
            )
        self.assertIn(
            'rack 有 100.0% 的表面在画面之外，', str(raised.exception)
        )


if __name__ == '__main__':
    unittest.main()
