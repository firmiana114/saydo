"""工具读回脚本：控制器是替身，记录的生成与写盘是真实代码。"""

import hashlib
import json
import pathlib
import tempfile
import unittest
from unittest import mock

import numpy as np
import yaml

from saydo.robot import arms
from saydo.robot import grasp_robot
from scripts import record_arm_tool
from tests.robot import fake_controller


class RecordArmToolTests(unittest.TestCase):
    def setUp(self):
        directory = tempfile.TemporaryDirectory()
        self.addCleanup(directory.cleanup)
        self.root = pathlib.Path(directory.name)
        self.fleet = fake_controller.Fleet()
        patcher = mock.patch.object(grasp_robot, 'RobotClient', self.fleet)
        patcher.start()
        self.addCleanup(patcher.stop)
        self.arm = arms.by_id('arm2')
        self.target = (
            self.root / 'config/calibration/arm2_tool_grip_tcp.yaml'
        )

    def record(self, **kwargs):
        return record_arm_tool.record(
            self.root, self.arm, '10.0.0.2', stamp='20261009T040000Z', **kwargs
        )

    def test_record_matches_what_the_controller_reports(self):
        self.assertEqual(self.record(), self.target)
        value = yaml.safe_load(self.target.read_text())
        self.assertEqual(value['arm'], 'arm2')
        # 控制器原始值 222500（0.001 毫米）即 0.2225 米，2611 即 2.611 弧度。
        self.assertEqual(value['translation_m'], [0, 0, .2225])
        self.assertEqual(value['rpy_rad'], [0, 0, 2.611])
        expected = np.eye(4)
        expected[:2, :2] = [
            [np.cos(2.611), -np.sin(2.611)], [np.sin(2.611), np.cos(2.611)]
        ]
        expected[2, 3] = .2225
        np.testing.assert_allclose(value['flange_T_grip_tcp'], expected)
        readback = value['controller_readback']
        # 执行前核对用的就是这份原始应答，必须与控制器给的一字不差。
        self.assertEqual(
            readback['tool_response_raw'], fake_controller.TOOL_RESPONSE
        )
        raw = (self.root / readback['path']).read_bytes()
        self.assertEqual(readback['sha256'], hashlib.sha256(raw).hexdigest())
        self.assertEqual(
            json.loads(raw)['replies']['get_current_tool_frame'],
            fake_controller.TOOL_RESPONSE,
        )
        self.assertEqual(self.fleet.controller('10.0.0.2').write_flags, [False])

    def test_wrong_active_tool_is_refused_and_nothing_is_written(self):
        with mock.patch.object(
            fake_controller,
            'TOOL_RESPONSE',
            {'tool_name': 'Arm_Tip', 'pose': [0] * 6},
        ), self.assertRaisesRegex(ValueError, '2臂.*Arm_Tip.*grip_tcp'):
            self.record()
        self.assertFalse(self.target.exists())
        self.assertFalse((self.root / 'work').exists())

    def test_existing_record_needs_replace_and_is_backed_up(self):
        self.record()
        original = self.target.read_text()
        with self.assertRaises(FileExistsError):
            record_arm_tool.record(
                self.root, self.arm, '10.0.0.2', stamp='20261009T050000Z'
            )
        self.assertEqual(self.target.read_text(), original)
        record_arm_tool.record(
            self.root, self.arm, '10.0.0.2', replace=True,
            stamp='20261009T050000Z',
        )
        backup = (
            self.root
            / 'work/calibration/arm2_tool_20261009T050000Z'
            / 'arm2_tool_grip_tcp.yaml.before'
        )
        self.assertEqual(backup.read_text(), original)

    def test_command_line_reports_failure_without_a_traceback(self):
        with mock.patch.object(
            record_arm_tool.paths, 'ROOT', self.root
        ), mock.patch.object(
            record_arm_tool.logging, 'basicConfig'
        ), mock.patch.object(
            fake_controller,
            'TOOL_RESPONSE',
            {'tool_name': 'Arm_Tip', 'pose': [0] * 6},
        ), self.assertLogs(record_arm_tool.LOG, 'ERROR') as logs:
            code = record_arm_tool.main(
                ['--arm', 'arm2', '--robot-ip', '10.0.0.2']
            )
        self.assertEqual(code, 1)
        self.assertIn('Arm_Tip', logs.output[0])


if __name__ == '__main__':
    unittest.main()
