"""机械臂外形与障碍物检查：纯计算，没有替身。"""

import pathlib
import tempfile
import unittest

import numpy as np
import yaml

from saydo.robot import collision_model
from saydo.robot import motion_validation

# grip_tcp：绕法兰 Z 轴转 2.611 弧度，沿 Z 平移 227.5 毫米。
TOOL = np.array(
    [
        [-.8625073128574445, -.5060445981012249, 0., 0.],
        [.5060445981012249, -.8625073128574445, 0., 0.],
        [0., 0., 1., .2275],
        [0., 0., 0., 1.],
    ]
)
ZERO = [0.] * 7
# 2026-10-09 两台臂的观测姿态（度）。
OBSERVATION_1 = [-115.381, -54.354, 81.509, 57.501, 28.999, 110.543, 195.475]
OBSERVATION_2 = [39.039, 43.688, 1.669, -71.65, -1.337, -115.995, -52.539]
POLE_1 = (-.0782, -.2633)
POLE_2 = (.0740, -.2442)


def environment(pole=None, table=-.0275, margin=.03):
    return collision_model.Environment(
        table_z_m=table,
        pole_xy_m=pole,
        pole_radius_m=.02 * np.sqrt(.5),
        pole_margin_m=margin,
    )


class KinematicsTests(unittest.TestCase):
    def test_zero_pose_stands_straight_up(self):
        frames = collision_model.link_frames(ZERO)
        self.assertEqual(len(frames), 8)
        np.testing.assert_allclose(
            frames[7][:3, 3], [0, 0, .8505], atol=1e-9
        )
        tip = frames[7] @ TOOL
        np.testing.assert_allclose(tip[:3, 3], [0, 0, 1.078], atol=1e-9)

    def test_matches_controller_readback_at_the_observation_poses(self):
        # 控制器当天报告的夹爪参考点位置（毫米），工具长 227.5 毫米。
        for joint, expected_mm in (
            (OBSERVATION_1, [276.4, -7.6, 157.2]),
            (OBSERVATION_2, [-111.1, -85.7, 310.9]),
        ):
            tip = collision_model.link_frames(joint)[7] @ TOOL
            np.testing.assert_allclose(
                tip[:3, 3] * 1000, expected_mm, atol=.5
            )


class ShapeTests(unittest.TestCase):
    def test_gripper_shapes_follow_the_vendor_drawing(self):
        starts, ends, radii, labels, _ = collision_model.capsules_in_base(
            ZERO, TOOL
        )
        gripper = [
            (start, end, radius)
            for start, end, radius, label in zip(starts, ends, radii, labels)
            if label.startswith('夹')
        ]
        top = max(max(s[2], e[2]) + r for s, e, r in gripper)
        # 闭合指尖正好在夹爪参考点，不超出。
        self.assertAlmostEqual(top, 1.078, places=6)
        # 张开总宽 151 毫米：最宽处离夹爪轴 75.5 毫米。
        widest = max(
            max(np.hypot(*s[:2]), np.hypot(*e[:2])) + r for s, e, r in gripper
        )
        self.assertAlmostEqual(widest, .0755, places=6)
        # 本体半径 37.5 毫米（安装座直径 75 毫米）。
        self.assertAlmostEqual(gripper[0][2], .0375)

    def test_every_arm_link_has_a_shape(self):
        _, _, radii, labels, table_checked = collision_model.capsules_in_base(
            ZERO, TOOL
        )
        self.assertEqual(labels[:8], list(collision_model.ARM_LABELS))
        self.assertTrue(all(.03 < radius < .08 for radius in radii[:8]))
        # 底座和肩部固定在桌面上方，不参与桌面检查。
        self.assertEqual(table_checked[:3], [False, False, True])


class TableTests(unittest.TestCase):
    def test_fingertip_may_touch_the_table_but_not_go_below(self):
        # 1 臂观测姿态下夹爪斜向下，指尖离桌面约 18 厘米。
        state = collision_model.check_configuration(
            OBSERVATION_1, TOOL, environment()
        )
        self.assertIsNone(state['violation'])
        self.assertGreater(state['table_clearance_m'], .12)
        # 把桌面抬到比最低点还高 1 毫米，就应报碰桌。
        lowest = -.0275 + state['table_clearance_m']
        raised = collision_model.check_configuration(
            OBSERVATION_1, TOOL, environment(table=lowest + .001)
        )
        self.assertIn(state['table_part'], raised['violation'])
        self.assertIn('会碰到桌面（低于桌面 1 毫米）', raised['violation'])
        # 正好贴着桌面（容差内）不算碰。
        touching = collision_model.check_configuration(
            OBSERVATION_1, TOOL, environment(table=lowest)
        )
        self.assertIsNone(touching['violation'])

    def test_base_and_shoulder_are_not_checked_against_the_table(self):
        # 桌面抬到底座中部：只要求肩部以上的部分高于它。
        state = collision_model.check_configuration(
            ZERO, TOOL, environment(table=.1)
        )
        self.assertIsNone(state['violation'])
        self.assertEqual(state['table_part'], '上臂')

    def test_table_is_not_checked_when_unset(self):
        state = collision_model.check_configuration(
            ZERO, TOOL, environment(table=None)
        )
        self.assertIsNone(state['table_clearance_m'])


class PoleTests(unittest.TestCase):
    def test_observation_poses_are_clear_of_the_pole(self):
        for joint, pole in (
            (OBSERVATION_1, POLE_1), (OBSERVATION_2, POLE_2)
        ):
            state = collision_model.check_configuration(
                joint, TOOL, environment(pole)
            )
            self.assertIsNone(state['violation'])
            self.assertGreater(state['pole_clearance_m'], .1)

    def test_pole_inside_the_margin_is_reported_with_the_part(self):
        # 把立杆放在零位机械臂的上臂旁边：离中心线 9 厘米。
        state = collision_model.check_configuration(
            ZERO, TOOL, environment((.09, 0.))
        )
        self.assertRegex(
            state['violation'], r'离立杆 \d+ 毫米，要求至少 30 毫米'
        )
        self.assertLess(state['pole_clearance_m'], .03)
        # 同样的位置，余量设为 0 时只要没有真的碰上就通过。
        relaxed = collision_model.check_configuration(
            ZERO, TOOL, environment((.09, 0.), margin=0.)
        )
        self.assertIsNone(relaxed['violation'])

    def test_square_bar_uses_its_half_diagonal(self):
        self.assertAlmostEqual(
            environment(POLE_1).pole_radius_m, .014142, places=5
        )

    def test_gripper_clamped_on_the_pole_is_a_violation(self):
        # 2026-10-09 测量立杆位置时 1 臂夹着立杆的姿态。
        clamped = [-61.4, -70.9, -2.2, 112.7, -39.0, 55.4, 140.4]
        state = collision_model.check_configuration(
            clamped, TOOL, environment(POLE_1)
        )
        self.assertIn('夹', state['pole_part'])
        self.assertLess(state['pole_clearance_m'], 0.)
        self.assertIsNotNone(state['violation'])


class PathTests(unittest.TestCase):
    def test_path_is_sampled_between_waypoints(self):
        # 起点和终点都远离立杆，但中途第 1 关节转过去时上臂会扫过它。
        start = [0., 45., 0., 45., 0., 0., 0.]
        end = [180., 45., 0., 45., 0., 0., 0.]
        frames = collision_model.link_frames([90., 45., 0., 45., 0., 0., 0.])
        # 立杆立在转到一半时手腕所在的位置。
        pole = tuple(frames[5][:2, 3])
        for joint in (start, end):
            self.assertIsNone(
                collision_model.check_configuration(
                    joint, TOOL, environment(pole)
                )['violation']
            )
        report = collision_model.check_path(
            [start, end], TOOL, environment(pole)
        )
        self.assertEqual(report['checked_samples'], 90)
        self.assertEqual(report['violation']['waypoint_index'], 1)
        self.assertIn('离立杆', report['violation']['message'])
        self.assertLess(report['min_pole_clearance_m'], .03)

    def test_the_current_pose_itself_is_not_judged(self):
        clamped = [-61.4, -70.9, -2.2, 112.7, -39.0, 55.4, 140.4]
        report = collision_model.check_path(
            [clamped, OBSERVATION_1], TOOL, environment(POLE_1)
        )
        # 起点夹着杆，离开的路上头几个取样点仍在余量之内，所以被拒绝；
        # 但违规记在“走向第 1 个路点”的途中，而不是起点。
        self.assertEqual(report['violation']['waypoint_index'], 1)
        self.assertNotEqual(report['violation']['joint_deg'], clamped)

    def test_clear_path_reports_the_closest_approach(self):
        report = collision_model.check_path(
            [ZERO, OBSERVATION_2], TOOL, environment(POLE_2)
        )
        self.assertIsNone(report['violation'])
        self.assertGreater(report['min_pole_clearance_m'], .03)
        self.assertGreater(report['min_table_clearance_m'], .2)
        self.assertIsNotNone(report['min_pole_part'])

    def test_malformed_path_is_refused(self):
        with self.assertRaisesRegex(ValueError, '路径至少需要'):
            collision_model.check_path([ZERO], TOOL, environment())


class ValidationHookTests(unittest.TestCase):
    """路径校验入口在逆解通过之后调用的那一步。"""

    def waypoints(self):
        return [
            {'name': 'pregrasp', 'controller_pose': [1, 2, 3, 4, 5, 6]},
            {'name': 'grasp', 'controller_pose': [7, 8, 9, 10, 11, 12]},
        ]

    def run_check(self, solutions, pole):
        detail = {'verified': True, 'joint_solutions_deg': solutions}
        return motion_validation.check_environment(
            detail,
            {'flange_T_grip_tcp': TOOL.tolist()},
            {'joint': [0] * 7},
            self.waypoints(),
            environment(pole),
        )

    def test_clear_path_passes_and_records_the_closest_approach(self):
        valid, detail = self.run_check(
            [[10.] + [0.] * 6, OBSERVATION_2], POLE_2
        )
        self.assertTrue(valid)
        self.assertTrue(detail['verified'])
        self.assertIsNone(detail['collision_check']['violation'])
        self.assertGreater(
            detail['collision_check']['min_pole_clearance_m'], .03
        )

    def test_violation_names_the_waypoint_being_approached(self):
        # 第二个路点让手腕转到立杆所在的位置。
        target = [90., 45., 0., 45., 0., 0., 0.]
        pole = tuple(collision_model.link_frames(target)[5][:2, 3])
        valid, detail = self.run_check([[10.] + [0.] * 6, target], pole)
        self.assertFalse(valid)
        self.assertFalse(detail['verified'])
        self.assertRegex(detail['reason'], r'^grasp：.+离立杆 -?\d+ 毫米')
        self.assertEqual(detail['failure']['kind'], 'collision')
        self.assertEqual(detail['failure']['waypoint'], 'grasp')
        self.assertEqual(
            detail['failure']['controller_pose'], [7, 8, 9, 10, 11, 12]
        )
        self.assertEqual(len(detail['failure']['joint_deg']), 7)


class LoadTests(unittest.TestCase):
    def setUp(self):
        directory = tempfile.TemporaryDirectory()
        self.addCleanup(directory.cleanup)
        self.root = pathlib.Path(directory.name)
        self.path = self.root / collision_model.OBSTACLES_FILE
        self.path.parent.mkdir(parents=True)

    def write(self, **pole):
        value = {'schema_version': 1}
        if pole:
            value['pole'] = pole
        self.path.write_text(yaml.safe_dump(value))

    def test_without_the_file_only_the_table_is_checked(self):
        self.path.parent.rmdir()
        loaded = collision_model.load_environment(self.root, 'arm1', -.02)
        self.assertEqual(loaded.table_z_m, -.02)
        self.assertIsNone(loaded.pole_xy_m)

    def test_each_arm_reads_its_own_pole_position(self):
        self.write(
            side_m=.02, margin_m=.03,
            position_m={'arm1': list(POLE_1), 'arm2': list(POLE_2)},
        )
        first = collision_model.load_environment(self.root, 'arm1', -.0275)
        second = collision_model.load_environment(self.root, 'arm2', -.0275)
        self.assertEqual(first.pole_xy_m, POLE_1)
        self.assertEqual(second.pole_xy_m, POLE_2)
        self.assertAlmostEqual(first.pole_radius_m, .014142, places=5)
        self.assertEqual(first.pole_margin_m, .03)

    def test_arm_without_a_position_has_no_pole(self):
        self.write(side_m=.02, margin_m=.03, position_m={'arm1': list(POLE_1)})
        loaded = collision_model.load_environment(self.root, 'arm2', -.0275)
        self.assertIsNone(loaded.pole_xy_m)

    def test_bad_values_are_refused(self):
        self.write(side_m=0., margin_m=.03, position_m={'arm1': list(POLE_1)})
        with self.assertRaisesRegex(ValueError, '立杆的数值缺失或无效'):
            collision_model.load_environment(self.root, 'arm1', -.0275)
        self.path.write_text(yaml.safe_dump({'schema_version': 9}))
        with self.assertRaisesRegex(ValueError, '格式版本不受支持'):
            collision_model.load_environment(self.root, 'arm1', -.0275)


if __name__ == '__main__':
    unittest.main()
