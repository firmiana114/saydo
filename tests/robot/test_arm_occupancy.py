"""两臂占位登记处的测试：只在“向控制器读取关节角”这一处边界打替身。"""

import pathlib
import tempfile
import unittest

import numpy as np
import yaml

from saydo.robot import arm_occupancy
from saydo.robot import arms
from saydo.robot import base_frames
from saydo.robot import collision_model

TOOL = np.eye(4)
TOOL[2, 3] = .2275
# 2 臂在报错和记录里的称呼。
ARM2 = arms.by_id('arm2').label
ZERO = [0.] * 7
# 2 臂底座在 1 臂底座的 −Y 方向 0.52 米处，绕竖直轴转 180 度。
ARM1_BASE_T_ARM2_BASE = np.array(
    [
        [-1., 0., 0., 0.],
        [0., -1., 0., -.52],
        [0., 0., 1., 0.],
        [0., 0., 0., 1.],
    ]
)


class OccupancyTests(unittest.TestCase):
    def setUp(self):
        directory = tempfile.TemporaryDirectory()
        self.addCleanup(directory.cleanup)
        self.root = pathlib.Path(directory.name)
        (self.root / 'config/calibration').mkdir(parents=True)
        (self.root / 'config/grasp').mkdir(parents=True)
        (self.root / base_frames.RELATION_FILE).write_text(
            yaml.safe_dump(
                {
                    'schema_version': 1,
                    'arm1_base_T_arm2_base': ARM1_BASE_T_ARM2_BASE.tolist(),
                }
            )
        )
        arms.by_id('arm2').config_path(self.root).write_text('{}')
        self.joints = {'arm1': list(ZERO), 'arm2': list(ZERO)}
        self.reads = []
        self.now = 100.
        self.failure = None
        self.occupancy = arm_occupancy.Occupancy(
            self.root, self.read_posture, lambda: self.now
        )

    def read_posture(self, root, arm):
        self.reads.append(arm.arm_id)
        if self.failure is not None:
            raise self.failure
        return np.array(self.joints[arm.arm_id]), TOOL

    def write_settings(self, **other_arm):
        (self.root / collision_model.OBSTACLES_FILE).write_text(
            yaml.safe_dump({'schema_version': 1, 'other_arm': other_arm})
        )

    def test_static_other_arm_is_its_current_pose_in_my_base_frame(self):
        other = self.occupancy.other_arm('arm1')
        self.assertFalse(other.moving)
        self.assertEqual(other.label, ARM2)
        self.assertEqual(other.margin_m, .03)
        starts, ends, radii = other.capsules
        self.assertEqual(len(radii), 13)
        # 2 臂竖直向上：最后一个胶囊体（夹指）的末端比夹爪参考点低
        # 11.5 毫米，在 1 臂坐标里位于 (0, -0.52, 1.0665)。
        np.testing.assert_allclose(
            ends[-1], [0., -.52, 1.0665], atol=1e-6
        )
        # 反过来，1 臂在 2 臂坐标里也在 −Y 方向 0.52 米处（两底座互为
        # 转了 180 度）。
        mirrored = self.occupancy.other_arm('arm2')
        self.assertEqual(mirrored.label, arms.by_id('arm1').label)
        np.testing.assert_allclose(
            mirrored.capsules[1][-1], [0., -.52, 1.0665], atol=1e-6
        )

    def test_static_reading_is_reused_briefly_then_refreshed(self):
        self.occupancy.other_arm('arm1')
        self.occupancy.other_arm('arm1')
        self.assertEqual(self.reads, ['arm2'])
        self.now += arm_occupancy.STATIC_MAX_AGE_S + .01
        self.joints['arm2'] = [0., 90., 0., 0., 0., 0., 0.]
        moved = self.occupancy.other_arm('arm1')
        self.assertEqual(self.reads, ['arm2', 'arm2'])
        # 2 臂放平了，最高点不再是 1.078 米。
        self.assertLess(moved.capsules[1][:, 2].max(), .5)

    def test_claim_replaces_the_static_pose_until_released(self):
        path = [ZERO, [0., 90., 0., 0., 0., 0., 0.]]
        version = self.occupancy.version()
        self.assertTrue(
            self.occupancy.claim_if_unchanged('arm2', path, TOOL, version)
        )
        self.assertTrue(self.occupancy.claimed('arm2'))
        other = self.occupancy.other_arm('arm1')
        self.assertTrue(other.moving)
        self.assertEqual(other.label, ARM2 + '正在走的路径')
        # 有占位时不去读控制器。
        self.assertEqual(self.reads, [])
        # 占位既包含起点（竖直，高 1.078 米）也包含终点（放平）。
        ends = other.capsules[1]
        self.assertGreater(ends[:, 2].max(), 1.)
        self.assertGreater(np.abs(ends[:, :2] - [0., -.52]).max(), .7)
        self.occupancy.release('arm2')
        self.assertFalse(self.occupancy.claimed('arm2'))
        self.joints['arm2'] = path[-1]
        after = self.occupancy.other_arm('arm1')
        self.assertFalse(after.moving)
        self.assertEqual(self.reads, ['arm2'])
        self.assertLess(after.capsules[1][:, 2].max(), .5)

    def test_claim_is_refused_when_the_other_arm_claimed_meanwhile(self):
        version = self.occupancy.version()
        # 1 臂检查期间，2 臂登记了占位。
        self.assertTrue(
            self.occupancy.claim_if_unchanged('arm2', [ZERO], TOOL, version)
        )
        self.assertFalse(
            self.occupancy.claim_if_unchanged('arm1', [ZERO], TOOL, version)
        )
        self.assertFalse(self.occupancy.claimed('arm1'))
        # 按新的版本号重新检查后可以登记。
        self.assertTrue(
            self.occupancy.claim_if_unchanged(
                'arm1', [ZERO], TOOL, self.occupancy.version()
            )
        )
        # 撤销占位同样让别的臂手里的旧版本号失效。
        stale = self.occupancy.version()
        self.occupancy.release('arm2')
        self.assertFalse(
            self.occupancy.claim_if_unchanged('arm1', [ZERO], TOOL, stale)
        )
        # 没有占位时撤销不改变版本号。
        current = self.occupancy.version()
        self.occupancy.release('arm2')
        self.assertEqual(self.occupancy.version(), current)

    def test_release_drops_the_cached_static_pose(self):
        self.occupancy.other_arm('arm1')
        self.joints['arm2'] = [0., 90., 0., 0., 0., 0., 0.]
        # 2 臂的任务刚结束：它动过，1 臂不能沿用半秒前的读数。
        self.occupancy.release('arm2')
        self.assertLess(
            self.occupancy.other_arm('arm1').capsules[1][:, 2].max(), .5
        )

    def test_unreadable_other_arm_is_refused_by_default(self):
        self.failure = OSError('No route to host')
        with self.assertRaisesRegex(
            ValueError,
            f'读不到{ARM2}的当前姿态（No route to host）.*未发送运动',
        ) as caught:
            self.occupancy.other_arm('arm1')
        self.assertIn('when_unreadable: ignore', str(caught.exception))
        self.assertIsInstance(caught.exception.__cause__, OSError)

    def test_unreadable_other_arm_can_be_ignored_with_a_warning(self):
        self.failure = RuntimeError('控制器断开连接')
        self.write_settings(when_unreadable='ignore')
        with self.assertLogs('saydo.robot.arm_occupancy', 'WARNING') as logs:
            self.assertIsNone(self.occupancy.other_arm('arm1'))
        self.assertIn('不检查与它的碰撞', logs.output[0])
        # 警告不会每次都记。
        with self.assertNoLogs('saydo.robot.arm_occupancy', 'WARNING'):
            self.occupancy.other_arm('arm1')
        self.now += arm_occupancy.WARNING_INTERVAL_S
        with self.assertLogs('saydo.robot.arm_occupancy', 'WARNING'):
            self.occupancy.other_arm('arm1')
        # 它有占位时不需要读控制器，仍然照常避让。
        self.occupancy.claim_if_unchanged(
            'arm2', [ZERO], TOOL, self.occupancy.version()
        )
        self.assertTrue(self.occupancy.other_arm('arm1').moving)

    def test_margin_comes_from_the_obstacles_file(self):
        self.write_settings(margin_m=.05)
        self.assertEqual(self.occupancy.other_arm('arm1').margin_m, .05)

    def test_nothing_to_avoid_without_relation_or_second_arm(self):
        (self.root / base_frames.RELATION_FILE).unlink()
        self.assertIsNone(self.occupancy.other_arm('arm1'))
        self.assertEqual(self.reads, [])

    def test_single_arm_machine_has_no_other_arm(self):
        arms.by_id('arm2').config_path(self.root).unlink()
        self.assertIsNone(self.occupancy.other_arm('arm1'))

    def test_shared_returns_one_registry_per_root(self):
        first = arm_occupancy.shared(self.root)
        self.assertIs(first, arm_occupancy.shared(self.root / '.'))
        with tempfile.TemporaryDirectory() as other:
            self.assertIsNot(first, arm_occupancy.shared(other))


class TransformTests(unittest.TestCase):
    def test_capsules_follow_the_base_transform(self):
        starts = np.array([[1., 0., 0.]])
        ends = np.array([[1., 0., .5]])
        radii = np.array([.1])
        moved = arm_occupancy.transformed(
            (starts, ends, radii), ARM1_BASE_T_ARM2_BASE
        )
        np.testing.assert_allclose(moved[0], [[-1., -.52, 0.]])
        np.testing.assert_allclose(moved[1], [[-1., -.52, .5]])
        self.assertIs(moved[2], radii)


if __name__ == '__main__':
    unittest.main()
