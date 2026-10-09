"""机械臂登记表：各臂的文件位置互不重叠，设备锁每臂一把。"""

import fcntl
import pathlib
import tempfile
import unittest

from saydo.robot import arms


class RegistryTests(unittest.TestCase):
    def test_first_arm_keeps_the_original_paths(self):
        arm = arms.default()
        self.assertEqual(arm.arm_id, 'arm1')
        self.assertEqual(arm.side, 'left')
        self.assertEqual(arm.config_file, 'config/grasp/trial.json')
        self.assertEqual(arm.placement_file, 'config/grasp/placement.json')
        self.assertEqual(
            arm.reorientation_file, 'config/grasp/reorientation.json'
        )
        self.assertEqual(arm.state_directory, 'work/grasp')

    def test_second_arm_is_the_right_arm(self):
        self.assertEqual(arms.by_id('arm2').side, 'right')

    def test_no_file_or_directory_is_shared_between_arms(self):
        for field in (
            'arm_id',
            'label',
            'config_file',
            'placement_file',
            'reorientation_file',
            'state_directory',
        ):
            values = [getattr(arm, field) for arm in arms.ARMS]
            self.assertEqual(len(set(values)), len(values), field)
        root = pathlib.Path('/project')
        locks = [arm.lock_path(root) for arm in arms.ARMS]
        self.assertEqual(
            locks,
            [
                root / 'work/grasp/robot.lock',
                root / 'work/grasp_arm2/robot.lock',
            ],
        )

    def test_unknown_arm_is_reported_with_the_known_ids(self):
        with self.assertRaisesRegex(ValueError, "'arm3'.*arm1、arm2"):
            arms.by_id('arm3')


class ConfiguredTests(unittest.TestCase):
    def setUp(self):
        directory = tempfile.TemporaryDirectory()
        self.addCleanup(directory.cleanup)
        self.root = pathlib.Path(directory.name)

    def configure(self, arm_id):
        path = arms.by_id(arm_id).config_path(self.root)
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_text('{}')

    def test_only_the_first_arm_without_a_second_config(self):
        self.assertEqual(
            [arm.arm_id for arm in arms.configured(self.root)], ['arm1']
        )

    def test_second_arm_appears_once_its_config_exists(self):
        self.configure('arm2')
        self.assertEqual(
            [arm.arm_id for arm in arms.configured(self.root)],
            ['arm1', 'arm2'],
        )

    def test_all_locked_holds_every_configured_arm_then_releases(self):
        self.configure('arm2')
        paths = [arm.lock_path(self.root) for arm in arms.ARMS]
        with arms.all_locked(self.root):
            for path in paths:
                with path.open('a') as other:
                    with self.assertRaises(BlockingIOError):
                        fcntl.flock(other, fcntl.LOCK_EX | fcntl.LOCK_NB)
        for path in paths:
            with path.open('a') as other:
                fcntl.flock(other, fcntl.LOCK_EX | fcntl.LOCK_NB)

    def test_all_locked_fails_when_any_arm_is_busy_and_keeps_nothing(self):
        self.configure('arm2')
        first, second = [arm.lock_path(self.root) for arm in arms.ARMS]
        second.parent.mkdir(parents=True)
        with second.open('a') as busy:
            fcntl.flock(busy, fcntl.LOCK_EX | fcntl.LOCK_NB)
            with self.assertRaises(BlockingIOError):
                with arms.all_locked(self.root):
                    self.fail('另一台臂正忙时不应进入')
        # 失败后已经拿到的 1 臂的锁必须放掉，否则 1 臂会被永久卡住。
        with first.open('a') as other:
            fcntl.flock(other, fcntl.LOCK_EX | fcntl.LOCK_NB)

    def test_all_locked_ignores_an_arm_that_is_not_configured(self):
        second = arms.by_id('arm2').lock_path(self.root)
        with arms.all_locked(self.root):
            self.assertFalse(second.exists())


if __name__ == '__main__':
    unittest.main()
