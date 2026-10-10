"""两台臂的动作控制器互相避让：占位在什么时候登记、什么时候撤销。

替身只放在两处边界：控制器（`RobotClient`）和逆解子进程
（`motion_validation.check_waypoints`）。
"""

import json
import pathlib
import tempfile
import types
import unittest
from unittest import mock

import numpy as np
import yaml

from saydo.manipulation import grasp_control
from saydo.robot import arm_occupancy
from saydo.robot import arms
from saydo.robot import base_frames
from tests.robot import fake_kinematics

ARM1_BASE_T_ARM2_BASE = [
    [-1., 0., 0., 0.],
    [0., -1., 0., -.52],
    [0., 0., 1., 0.],
    [0., 0., 0., 1.],
]
TOOL = np.eye(4)
TOOL[2, 3] = .2275
# 两台臂在报错和记录里的称呼。
ARM1 = arms.by_id('arm1').label
ARM2 = arms.by_id('arm2').label


class ArmAvoidanceTests(unittest.TestCase):
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
                    'arm1_base_T_arm2_base': ARM1_BASE_T_ARM2_BASE,
                }
            )
        )
        (self.root / 'checker').touch()
        (self.root / 'tool.yaml').write_text(
            yaml.safe_dump({'flange_T_grip_tcp': TOOL.tolist()})
        )
        self.joint_raw = {'arm1-host': [0] * 7, 'arm2-host': [0] * 7}
        self.queries = []
        self.services = {}
        for arm_id in ('arm1', 'arm2'):
            arm = arms.by_id(arm_id)
            self.config = {
                'robot_ip': arm_id + '-host',
                'robot_port': 8080,
                'tool_file': 'tool.yaml',
                'table_z_m': -.0275,
                'ik_checker': 'checker',
            }
            path = arm.config_path(self.root)
            path.write_text(json.dumps(self.config))
            self.services[arm_id] = grasp_control.GraspController(
                self.root, types.SimpleNamespace(), path, 'execute', arm
            )
        patcher = mock.patch(
            'saydo.robot.grasp_robot.RobotClient', side_effect=self.client
        )
        patcher.start()
        self.addCleanup(patcher.stop)
        self.kinematics = fake_kinematics.PairedKinematics()
        self.plan = {'flange_T_grip_tcp': TOOL.tolist()}
        self.arm_state = {'joint': [0] * 7, 'pose': [0] * 6, 'err': [0]}
        self.points = [
            {'name': 'pregrasp', 'controller_pose': [1, 2, 3, 0, 0, 0]}
        ]

    def client(self, ip, port=8080, writes=False):
        """控制器替身：只应答关节角查询，并记下是谁、以什么权限来查。"""
        robot = mock.Mock()

        def query(command):
            self.queries.append((ip, command, writes))
            return {'arm_state': {'joint': list(self.joint_raw[ip])}}

        robot.query.side_effect = query
        return robot

    def verify(self, arm_id, **options):
        with mock.patch(
            'saydo.robot.motion_validation.check_waypoints',
            side_effect=self.kinematics.check_waypoints,
        ) as check:
            self.services[arm_id].verify_motion_targets(
                self.plan, self.config, {}, self.arm_state, self.points,
                **options,
            )
        return check

    def test_idle_other_arm_is_read_from_its_controller_read_only(self):
        environment = self.services['arm1'].collision_environment(self.config)
        self.assertEqual(
            self.queries, [('arm2-host', 'get_current_arm_state', False)]
        )
        self.assertEqual(environment.other_arm_label, ARM2)
        self.assertEqual(environment.other_arm_margin_m, .03)
        self.assertEqual(len(environment.other_arm_capsules[2]), 13)
        # 2 臂竖直向上，它的夹指末端在 1 臂坐标里是 (0, -0.52, 1.0665)。
        np.testing.assert_allclose(
            environment.other_arm_capsules[1][-1], [0., -.52, 1.0665],
            atol=1e-6,
        )
        self.assertEqual(environment.table_z_m, -.0275)

    def test_verified_path_is_claimed_while_the_task_is_running(self):
        first, second = self.services['arm1'], self.services['arm2']
        occupancy = arm_occupancy.shared(self.root)
        first.update(busy=True)
        check = self.verify('arm1')
        self.assertTrue(occupancy.claimed('arm1'))
        # 检查时带着 2 臂的静止姿态。
        self.assertEqual(
            check.call_args.kwargs['environment'].other_arm_label, ARM2
        )
        self.queries.clear()
        environment = second.collision_environment(self.config)
        self.assertEqual(
            environment.other_arm_label, ARM1 + '正在走的路径'
        )
        # 1 臂有占位时，不必再去读它的控制器。
        self.assertEqual(self.queries, [])
        # 任务结束，占位撤销，此后按 1 臂当时的实际姿态对待它。
        first.update(busy=False)
        self.assertFalse(occupancy.claimed('arm1'))
        self.assertEqual(
            second.collision_environment(self.config).other_arm_label, ARM1
        )
        self.assertEqual(
            self.queries, [('arm1-host', 'get_current_arm_state', False)]
        )

    def test_starting_a_task_also_drops_a_stale_claim(self):
        first = self.services['arm1']
        occupancy = arm_occupancy.shared(self.root)
        first.update(busy=True)
        self.verify('arm1')
        first.update(busy=True, state='planning')
        self.assertFalse(occupancy.claimed('arm1'))

    def test_nothing_is_claimed_outside_a_task_or_when_only_comparing(self):
        occupancy = arm_occupancy.shared(self.root)
        self.verify('arm1')
        self.assertFalse(occupancy.claimed('arm1'))
        self.services['arm1'].update(busy=True)
        self.verify('arm1', claim=False)
        self.assertFalse(occupancy.claimed('arm1'))

    def test_path_is_rechecked_when_the_other_arm_claims_during_the_check(self):
        first = self.services['arm1']
        occupancy = arm_occupancy.shared(self.root)
        first.update(busy=True)
        labels = []

        def check(executable, tool, arm, waypoints, timeout, environment):
            labels.append(environment.other_arm_label)
            if len(labels) == 1:
                # 1 臂的这次检查还没结束，2 臂登记了占位。
                occupancy.claim_if_unchanged(
                    'arm2', [[0.] * 7], TOOL, occupancy.version()
                )
            return self.kinematics.check_waypoints(
                executable, tool, arm, waypoints
            )

        report = {}
        with mock.patch(
            'saydo.robot.motion_validation.check_waypoints', side_effect=check
        ):
            first.verify_motion_targets(
                self.plan, self.config, report, self.arm_state, self.points
            )
        # 第二次检查用的是 2 臂新登记的占位。
        self.assertEqual(labels, [ARM2, ARM2 + '正在走的路径'])
        self.assertTrue(occupancy.claimed('arm1'))
        self.assertEqual(len(report['kinematics_checks']), 2)

    def test_task_stops_when_the_other_arm_keeps_changing_its_claim(self):
        first = self.services['arm1']
        occupancy = arm_occupancy.shared(self.root)
        first.update(busy=True)

        def check(executable, tool, arm, waypoints, timeout, environment):
            occupancy.release('arm2')
            occupancy.claim_if_unchanged(
                'arm2', [[0.] * 7], TOOL, occupancy.version()
            )
            return self.kinematics.check_waypoints(
                executable, tool, arm, waypoints
            )

        with mock.patch(
            'saydo.robot.motion_validation.check_waypoints', side_effect=check
        ) as checked:
            with self.assertRaisesRegex(ValueError, '反复变化，未发送运动'):
                first.verify_motion_targets(
                    self.plan, self.config, {}, self.arm_state, self.points
                )
        self.assertEqual(
            checked.call_count, grasp_control.OCCUPANCY_ATTEMPTS
        )
        self.assertFalse(occupancy.claimed('arm1'))

    def test_failed_check_claims_nothing(self):
        first = self.services['arm1']
        first.update(busy=True)
        with mock.patch(
            'saydo.robot.motion_validation.check_waypoints',
            return_value=(False, {'reason': '离2 臂 5 毫米'}),
        ):
            with self.assertRaisesRegex(ValueError, '离2 臂 5 毫米'):
                first.verify_motion_targets(
                    self.plan, self.config, {}, self.arm_state, self.points
                )
        self.assertFalse(arm_occupancy.shared(self.root).claimed('arm1'))

    def test_unreachable_other_controller_blocks_planning(self):
        self.joint_raw.pop('arm2-host')
        with self.assertRaisesRegex(ValueError, f'读不到{ARM2}的当前姿态'):
            self.services['arm1'].collision_environment(self.config)


if __name__ == '__main__':
    unittest.main()
