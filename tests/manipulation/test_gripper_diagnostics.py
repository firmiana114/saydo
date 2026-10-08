import copy
import json
import pathlib
import tempfile
import time
import types
import unittest
from unittest import mock
import numpy as np

from saydo.manipulation import grasp_control
from saydo.manipulation import gripper_diagnostics
from tests.manipulation import test_grasp


def state(**changes):
    return {
        'sys_state': 0,
        'sys_err': 0,
        'dof_state': [2],
        'dof_err': [0],
        'pos': [0],
        'speed': [0],
        'force': [0],
        'current': [2],
        **changes,
    }


class ClosureDiagnosticsTests(unittest.TestCase):
    def run_grasp(
        self,
        responses,
        command_failure=None,
        stop_failure=False,
        timeout=2,
        cancel_after_sample=False,
    ):
        temp = tempfile.TemporaryDirectory()
        self.addCleanup(temp.cleanup)
        root = pathlib.Path(temp.name)
        config = test_grasp.GraspTests().config()
        config['gripper']['close_timeout_s'] = timeout
        cfg = root / 'grasp.json'
        cfg.write_text(json.dumps(config))
        observation = {
            'pose': np.eye(4).tolist(),
            'time': time.time(),
            'generation': 1,
            'object_id': 'never_configured_paper_cup',
            'K': np.diag([.5, .5, 1]).tolist(),
        }
        observation['pose'][0][3] = .1
        arm = {'pose': [0, 0, 200000, 0, 0, 0], 'joint': [0] * 7, 'err': [0]}
        raw = {'tool_name': 'grip_tcp', 'pose': [0] * 6}
        work = {'frame_name': 'Base', 'pose': [0] * 6}
        tool = {
            'controller_readback': {'tool_response_raw': raw},
            'flange_T_grip_tcp': np.eye(4).tolist(),
        }
        handeye = {
            'installation': {'controller_firmware': 'V1.7.0'},
            'flange_T_color_optical_camera': np.eye(4).tolist(),
            'camera': {'K': np.eye(3).tolist()},
        }
        replies = {
            'get_arm_software_info': {
                'Product_version': 'RM75-BI', 'ctrl_info': {'version': 'V1.7.0'}
            },
            'get_current_tool_frame': raw,
            'get_current_work_frame': work,
            'get_joint_en_state': {'en_state': [1] * 7},
            'get_joint_err_flag': {'err_flag': [0] * 7},
            'get_rm_plus_mode': {'mode': 115200},
            'get_rm_plus_base_info': {
                'base_info': {
                    'manu': 'ZX',
                    'id': 1,
                    'dof': 1,
                    'type': 1,
                    'force': True,
                    'force_up': [100],
                }
            },
        }
        service = grasp_control.GraspController(
            root, types.SimpleNamespace(), cfg
        )
        robot = mock.Mock()
        closed = False
        index = 0
        force_calls = 0

        def query(command):
            nonlocal index
            if command == 'get_current_arm_state':
                return {'arm_state': copy.deepcopy(arm)}
            if command == 'get_rm_plus_state_info':
                if not closed:
                    return {'state_info': state(pos=[1000])}
                sample = responses[min(index, len(responses) - 1)]
                index += 1
                if isinstance(sample, Exception):
                    raise sample
                if cancel_after_sample:
                    service.cancelled.set()
                return {'state_info': sample}
            return copy.deepcopy(replies[command])

        def set_register(*args):
            nonlocal force_calls
            force_calls += 1
            if force_calls == 2 and command_failure == 'force':
                raise RuntimeError('force rejected')

        def gripper(position):
            nonlocal closed
            if position == 0 and command_failure == 'close':
                raise RuntimeError('close rejected')
            closed = position == 0

        def move(pose, *args):
            arm['pose'] = list(pose)

        def stop():
            # Simulate feedback changing as a consequence of the failure-stop.
            for sample in responses:
                if isinstance(sample, dict):
                    sample.update(
                        pos=[0],
                        dof_state=[2],
                        sys_err=0,
                        dof_err=[0],
                        current=[0],
                    )
            if stop_failure:
                raise RuntimeError('stop not acknowledged')

        robot.query.side_effect = query
        robot.set_register.side_effect = set_register
        robot.move_gripper.side_effect = gripper
        robot.move_linear.side_effect = move
        robot.move_joint_pose.side_effect = move
        robot.stop.side_effect = stop
        with mock.patch(
            'saydo.robot.grasp_robot.RobotClient', return_value=robot
        ), mock.patch.object(
            service, 'calibration', return_value=(handeye, tool)
        ), mock.patch.object(
            service, 'paired_observation', return_value=(arm, observation)
        ), mock.patch.object(
            service,
            'verify_motion_targets',
            return_value={'joint_travel_deg': 1},
        ), mock.patch('saydo.robot.motion_validation.wait_for_pose'):
            service.run(observation)
        report = json.loads(
            next((root / 'work/grasp/jobs').glob('*.json')).read_text()
        )
        return report, robot

    def test_pre_stop_fault_sample_is_saved_before_stop_changes_feedback(self):
        for failure, label in [
            (state(), '未确认接触'),
            (state(dof_state=[5], pos=[150], current=[900]), '电流保护'),
            (state(dof_state=[6], dof_err=[7]), '故障停止'),
            (state(sys_err=12), '反馈异常'),
        ]:
            original = copy.deepcopy(failure)
            report, robot = self.run_grasp([failure], stop_failure=True)
            diag = report['gripper_close_diagnostics']
            sample = diag['decision']['sample']
            self.assertEqual(sample['state'], original)
            self.assertEqual(
                diag['failure_before_stop']['last_closing_sample']['state'],
                original,
            )
            self.assertLessEqual(
                diag['decision']['at'], report['failure_before_stop']['at']
            )
            self.assertLessEqual(
                report['failure_before_stop']['at'],
                report['failure_stop']['requested_at'],
            )
            self.assertIn(label, report['error'])
            self.assertIn('pos=', report['error'])
            self.assertEqual(report['stop_error'], 'stop not acknowledged')
            self.assertFalse(report['failure_stop']['commands_acknowledged'])
            robot.stop.assert_called_once()
            self.assertEqual(robot.move_linear.call_count, 1)

    def test_success_requires_three_consecutive_samples_and_records_commands(
        self
    ):
        responses = [
            state(dof_state=[3], pos=[200]),
            state(dof_state=[1], pos=[180], speed=[10]),
            state(dof_state=[3], pos=[160]),
            state(dof_state=[3], pos=[160]),
            state(dof_state=[3], pos=[160], force=[2]),
        ]
        report, robot = self.run_grasp(responses)
        diag = report['gripper_close_diagnostics']
        self.assertNotIn('error', report)
        self.assertEqual(
            [x['consecutive_contact_samples'] for x in diag['samples']],
            [1, 0, 1, 2, 3],
        )
        self.assertEqual(diag['decision']['outcome'], 'contact_confirmed')
        self.assertEqual(
            [c['command'] for c in diag['commands']],
            ['set_rm_plus_reg', 'hand_follow_pos'],
        )
        self.assertTrue(all(c['acknowledged'] for c in diag['commands']))
        self.assertLessEqual(
            diag['commands'][-1]['finished']['at'], diag['samples'][0]['at']
        )
        robot.stop.assert_not_called()
        self.assertEqual(robot.move_linear.call_count, 2)

    def test_query_and_command_errors_remain_recorded_and_do_not_lift(self):
        report, robot = self.run_grasp([RuntimeError('query unavailable')])
        sample = report['gripper_close_diagnostics']['last_sample']
        self.assertIsNone(sample['state'])
        self.assertEqual(sample['error'], 'query unavailable')
        self.assertEqual(robot.move_linear.call_count, 1)
        for command in ('force', 'close'):
            report, robot = self.run_grasp([state()], command_failure=command)
            diag = report['gripper_close_diagnostics']
            self.assertEqual(diag['samples_total'], 0)
            self.assertFalse(diag['commands'][-1]['acknowledged'])
            self.assertIn('rejected', diag['commands'][-1]['error'])
            self.assertEqual(robot.move_linear.call_count, 1)

    def test_timeout_reports_last_feedback_without_relaxing_contact_rule(self):
        report, robot = self.run_grasp(
            [state(dof_state=[1], pos=[500], speed=[30])], timeout=.01
        )
        diag = report['gripper_close_diagnostics']
        self.assertEqual(diag['decision']['outcome'], 'timeout')
        self.assertEqual(diag['decision']['sample']['state']['pos'], [500])
        self.assertIn('闭合确认超时', report['error'])
        self.assertEqual(robot.move_linear.call_count, 1)

    def test_sampling_is_bounded_but_terminal_sample_survives(self):
        report = {}
        diag = gripper_diagnostics.CloseDiagnostics(
            report, test_grasp.GraspTests().config()['gripper']
        )
        robot = mock.Mock()
        raw = state(dof_state=[1], pos=[500])
        robot.query.side_effect = lambda *_: {'state_info': raw}
        for _ in range(gripper_diagnostics.MAX_SAMPLES + 4):
            diag.sample(robot)
        raw.update(dof_state=[5], current=[999])
        sample = diag.sample(robot)
        diag.decision('fault', sample)
        raw['current'][0] = 0
        self.assertEqual(
            len(diag.data['samples']), gripper_diagnostics.MAX_SAMPLES
        )
        self.assertEqual(diag.data['samples_dropped'], 5)
        self.assertEqual(
            diag.data['decision']['sample']['state']['current'], [999]
        )
        self.assertEqual(
            diag.data['last_sample']['sequence'],
            gripper_diagnostics.MAX_SAMPLES + 5,
        )
        json.dumps(report, allow_nan=False)

    def test_cancellation_keeps_last_feedback_and_stops_without_lifting(self):
        report, robot = self.run_grasp(
            [state(dof_state=[1], pos=[400])], cancel_after_sample=True
        )
        self.assertEqual(report['result']['state'], 'cancelled')
        self.assertEqual(
            report[
                'failure_before_stop'
            ]['last_closing_sample']['state']['pos'],
            [400],
        )
        robot.stop.assert_called_once()
        self.assertEqual(robot.move_linear.call_count, 1)


if __name__ == '__main__':
    unittest.main()
