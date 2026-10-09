"""两台臂各有一个真实的动作控制器；只有控制器通信是替身。"""

import fcntl
import hashlib
import json
import pathlib
import tempfile
import unittest
from unittest import mock

from saydo.manipulation import grasp_control
from saydo.robot import arms
from saydo.robot import grasp_robot
from tests.robot import fake_controller

ADDRESSES = {'arm1': '10.0.0.1', 'arm2': '10.0.0.2'}
RESET_JOINT_DEG = {'arm1': [1] * 7, 'arm2': [2] * 7}


class DualArmControllerTests(unittest.TestCase):
    def setUp(self):
        directory = tempfile.TemporaryDirectory()
        self.addCleanup(directory.cleanup)
        self.root = pathlib.Path(directory.name)
        self.fleet = fake_controller.Fleet()
        patcher = mock.patch.object(grasp_robot, 'RobotClient', self.fleet)
        patcher.start()
        self.addCleanup(patcher.stop)
        self.services = {}
        for arm in arms.ARMS:
            path = fake_controller.write_arm_config(
                self.root,
                arm,
                ADDRESSES[arm.arm_id],
                RESET_JOINT_DEG[arm.arm_id],
            )
            self.services[arm.arm_id] = grasp_control.GraspController(
                self.root, None, path, 'execute', arm
            )

    def robot(self, arm_id):
        return self.fleet.controller(ADDRESSES[arm_id])

    def hold_motion(self):
        for arm_id in self.services:
            self.robot(arm_id).release.clear()

    def finish_motion(self):
        for arm_id, service in self.services.items():
            self.robot(arm_id).release.set()
            if service.thread is not None:
                service.thread.join(timeout=5)
                self.assertFalse(service.thread.is_alive())

    def records(self, arm_id, kind):
        directory = arms.by_id(arm_id).state_path(self.root) / kind
        return sorted(directory.glob('*.json'))

    def test_both_arms_move_at_the_same_time(self):
        self.hold_motion()
        self.addCleanup(self.finish_motion)
        for service in self.services.values():
            service.start_reset()
        # 两台臂都停在运动半途：如果它们还共用一把设备锁，后开始的那台
        # 会在加锁时失败，永远到不了这里。
        for arm_id in self.services:
            self.assertTrue(self.robot(arm_id).moving.wait(3), arm_id)
        for service in self.services.values():
            self.assertTrue(service.status['busy'])
            self.assertEqual(service.status['state'], 'resetting')
        self.finish_motion()
        for arm_id, service in self.services.items():
            self.assertEqual(service.status['state'], 'completed', arm_id)
            self.assertEqual(service.status['arm'], arm_id)
        self.assertEqual(self.robot('arm1').targets, [[1000] * 7])
        self.assertEqual(self.robot('arm2').targets, [[2000] * 7])

    def test_each_arm_records_under_its_own_directory(self):
        self.services['arm2'].start_reset()
        self.finish_motion()
        self.assertEqual(self.records('arm1', 'reset_jobs'), [])
        (record,) = self.records('arm2', 'reset_jobs')
        report = json.loads(record.read_text())
        self.assertTrue(report['robot_motion_sent'])
        self.assertEqual(report['target_joint_raw'], [2000] * 7)
        self.assertEqual(
            record.parent.parent, self.root / 'work/grasp_arm2'
        )
        self.assertTrue((self.root / 'work/grasp_arm2/robot.lock').exists())
        self.assertFalse((self.root / 'work/grasp/robot.lock').exists())

    def test_occupied_device_lock_blocks_only_that_arm(self):
        lock_path = arms.by_id('arm2').lock_path(self.root)
        lock_path.parent.mkdir(parents=True)
        with lock_path.open('a') as other_program:
            fcntl.flock(other_program, fcntl.LOCK_EX | fcntl.LOCK_NB)
            for service in self.services.values():
                service.start_reset()
            self.finish_motion()
        self.assertEqual(self.services['arm1'].status['state'], 'completed')
        self.assertEqual(self.services['arm2'].status['state'], 'failed')
        self.assertEqual(self.robot('arm2').targets, [])
        (record,) = self.records('arm2', 'reset_jobs')
        self.assertFalse(json.loads(record.read_text())['robot_motion_sent'])

    def test_second_task_on_the_same_arm_is_still_refused(self):
        self.hold_motion()
        self.addCleanup(self.finish_motion)
        self.services['arm1'].start_reset()
        self.assertTrue(self.robot('arm1').moving.wait(3))
        with self.assertRaisesRegex(ValueError, '已有机械臂任务'):
            self.services['arm1'].start_reset()
        self.services['arm2'].start_reset()
        self.assertTrue(self.robot('arm2').moving.wait(3))

    def test_aborting_one_arm_leaves_the_other_running(self):
        self.hold_motion()
        self.addCleanup(self.finish_motion)
        for service in self.services.values():
            service.start_reset()
        for arm_id in self.services:
            self.assertTrue(self.robot(arm_id).moving.wait(3))
        self.services['arm2'].abort()
        self.assertEqual(self.robot('arm2').stops, 1)
        self.assertEqual(self.robot('arm1').stops, 0)
        self.assertFalse(self.services['arm1'].cancelled.is_set())
        self.finish_motion()
        self.assertEqual(self.services['arm1'].status['state'], 'completed')
        self.assertEqual(self.services['arm2'].status['state'], 'cancelled')

    def test_speed_setting_is_saved_to_that_arm_only(self):
        service = self.services['arm2']
        before = {
            arm_id: other.config_path.read_bytes()
            for arm_id, other in self.services.items()
        }
        result = service.set_arm_speed(
            20, hashlib.sha256(before['arm2']).hexdigest()
        )
        self.assertEqual(result['value'], 20)
        self.assertEqual(
            json.loads(service.config_path.read_text())['arm_speed_percent'],
            20,
        )
        self.assertEqual(
            self.services['arm1'].config_path.read_bytes(), before['arm1']
        )
        self.assertEqual(len(self.records('arm2', 'speed_commands')), 1)
        self.assertEqual(self.records('arm1', 'speed_commands'), [])

    def test_settings_scenes_and_plans_are_kept_per_arm(self):
        first, second = self.services['arm1'], self.services['arm2']
        self.assertEqual(
            first.placement_path, self.root / 'config/grasp/placement.json'
        )
        self.assertEqual(
            second.placement_path,
            self.root / 'config/grasp/arm2_placement.json',
        )
        self.assertEqual(first.scenes().root, self.root / 'work/grasp/scenes')
        self.assertEqual(
            second.scenes().root, self.root / 'work/grasp_arm2/scenes'
        )
        self.assertEqual(
            first.object_plan_store().path,
            self.root / 'work/grasp/object_motion_plans.sqlite',
        )
        self.assertEqual(
            second.object_plan_store().path,
            self.root / 'work/grasp_arm2/object_motion_plans.sqlite',
        )

    def test_camera_reuse_policy_defaults_to_the_original_file(self):
        service = self.services['arm2']
        self.assertEqual(
            service.camera_reuse_path({}),
            self.root / 'config/calibration/camera_reuse.yaml',
        )
        self.assertEqual(
            service.camera_reuse_path(
                {'camera_reuse_file': 'config/calibration/arm2_reuse.yaml'}
            ),
            self.root / 'config/calibration/arm2_reuse.yaml',
        )


if __name__ == '__main__':
    unittest.main()
