"""按臂分发 Agent 请求：真实的接口、分发和动作控制器，只有控制器通信是替身。"""

import pathlib
import tempfile
import threading
import types
import unittest
from unittest import mock

from saydo.manipulation import grasp_control
from saydo.robot import arms
from saydo.robot import grasp_robot
from saydo.workbench import agent_api
from saydo.workbench import agent_router
from saydo.workbench import live_tracker
from saydo.workbench import no_vision
from tests.robot import fake_controller

ADDRESSES = {'arm1': '10.0.0.1', 'arm2': '10.0.0.2'}
RESET = {'target': 'observation'}


class ArmRouterTests(unittest.TestCase):
    def setUp(self):
        directory = tempfile.TemporaryDirectory()
        self.addCleanup(directory.cleanup)
        self.root = pathlib.Path(directory.name)
        self.fleet = fake_controller.Fleet()
        patcher = mock.patch.object(grasp_robot, 'RobotClient', self.fleet)
        patcher.start()
        self.addCleanup(patcher.stop)
        models = self.root / 'models'
        # 1 臂的视觉来源：相机在系统边界上，用一个没有画面的替身。
        self.vision = types.SimpleNamespace(
            lock=threading.RLock(),
            state={'state': 'idle', 'object_id': None},
            models_root=models,
            latest=None,
        )
        self.services = {}
        for arm in arms.ARMS:
            path = fake_controller.write_arm_config(
                self.root, arm, ADDRESSES[arm.arm_id], [1] * 7
            )
            vision = (
                self.vision
                if arm.arm_id == 'arm1'
                else no_vision.NoVision(models, arm)
            )
            service = grasp_control.GraspController(
                self.root, vision, path, 'execute', arm
            )
            vision.grasp = service
            self.services[arm.arm_id] = service
        self.vision.snapshot = lambda: {
            **self.vision.state, 'grasp': self.services['arm1'].snapshot()
        }
        self.vision.arms = dict(self.services)
        self.token = self.root / 'token'
        self.token.write_text('x' * 40)
        self.token.chmod(0o600)
        self.router = agent_router.ArmRouter(self.vision, self.token)

    def robot(self, arm_id):
        return self.fleet.controller(ADDRESSES[arm_id])

    def hold_motion(self):
        for arm_id in self.services:
            self.robot(arm_id).release.clear()
        self.addCleanup(self.finish_motion)

    def finish_motion(self):
        for arm_id, service in self.services.items():
            self.robot(arm_id).release.set()
            if service.thread is not None:
                service.thread.join(timeout=5)
                self.assertFalse(service.thread.is_alive())

    def start(self, task_id, arm=None, operation='reset', arguments=None):
        request = {
            'operation': operation,
            'request_id': task_id,
            'arguments': RESET if arguments is None else arguments,
        }
        if arm is not None:
            request['arm'] = arm
        return self.router.post('/api/agent/tasks', request)

    def task(self, task_id):
        return self.router.get(f'/api/agent/tasks/{task_id}', {})

    def test_both_arms_run_a_task_at_once_and_each_task_names_its_arm(self):
        self.hold_motion()
        self.assertEqual(self.start('left-0001')['arm'], 'arm1')
        self.assertEqual(self.start('right-0001', arm='arm2')['arm'], 'arm2')
        for arm_id in self.services:
            self.assertTrue(self.robot(arm_id).moving.wait(3), arm_id)
        rows = {
            row['arm']: row for row in self.router.get(
                '/api/agent/arms', {}
            )['arms']
        }
        self.assertEqual(
            {arm_id: row['busy'] for arm_id, row in rows.items()},
            {'arm1': True, 'arm2': True},
        )
        self.assertEqual(rows['arm1']['active_task_id'], 'left-0001')
        self.assertEqual(rows['arm2']['active_task_id'], 'right-0001')
        self.assertEqual(
            (rows['arm1']['side'], rows['arm2']['side']), ('left', 'right')
        )
        # 查询任务不用写臂：按任务记录找到它所属的臂。
        self.assertFalse(self.task('right-0001')['terminal'])
        self.finish_motion()
        for task_id in ('left-0001', 'right-0001'):
            task = self.task(task_id)
            self.assertTrue(task['terminal'])
            self.assertEqual(task['state'], 'completed')
        self.assertEqual(self.robot('arm1').targets, [[1000] * 7])
        self.assertEqual(self.robot('arm2').targets, [[1000] * 7])

    def test_busy_arm_refuses_a_second_task_but_the_other_arm_accepts(self):
        self.hold_motion()
        self.start('left-0001')
        self.assertTrue(self.robot('arm1').moving.wait(3))
        with self.assertRaises(agent_api.AgentError) as refused:
            self.start('left-0002')
        self.assertEqual(refused.exception.payload['code'], 'BUSY')
        self.assertEqual(self.start('right-0001', arm='arm2')['arm'], 'arm2')

    def test_request_id_cannot_be_reused_on_the_other_arm(self):
        self.start('shared-0001')
        self.finish_motion()
        with self.assertRaises(agent_api.AgentError) as refused:
            self.start('shared-0001', arm='arm2')
        self.assertEqual(
            refused.exception.payload['code'], 'REQUEST_ID_CONFLICT'
        )
        self.assertEqual(self.robot('arm2').targets, [])
        # 同一台臂上重复提交仍然只返回原任务，不重复执行。
        self.assertTrue(self.start('shared-0001')['deduplicated'])
        self.assertEqual(len(self.robot('arm1').targets), 1)

    def test_unknown_arm_is_refused_before_anything_is_recorded(self):
        for call in (
            lambda: self.start('other-0001', arm='arm3'),
            lambda: self.router.get('/api/agent/status', {'arm': ['arm3']}),
            lambda: self.router.post('/api/agent/stop', {'arm': 'arm3'}),
        ):
            with self.assertRaises(agent_api.AgentError) as refused:
                call()
            self.assertEqual(refused.exception.payload['code'], 'UNKNOWN_ARM')
            self.assertIn('arm1、arm2', refused.exception.payload['message'])
        with self.assertRaises(agent_api.AgentError) as missing:
            self.task('other-0001')
        self.assertEqual(missing.exception.payload['code'], 'TASK_NOT_FOUND')

    def test_stop_without_an_arm_stops_every_arm(self):
        result = self.router.post('/api/agent/stop', {})
        self.assertTrue(result['stop_acknowledged'])
        self.assertTrue(result['stationary_observed'])
        self.assertEqual(set(result['arms']), {'arm1', 'arm2'})
        self.assertEqual(
            (self.robot('arm1').stops, self.robot('arm2').stops), (1, 1)
        )

    def test_stop_with_an_arm_stops_only_that_arm(self):
        result = self.router.post('/api/agent/stop', {'arm': 'arm2'})
        self.assertTrue(result['stop_acknowledged'])
        self.assertNotIn('arms', result)
        self.assertEqual(
            (self.robot('arm1').stops, self.robot('arm2').stops), (0, 1)
        )

    def test_stop_reports_the_arm_that_was_not_confirmed(self):
        self.robot('arm2').stop_error = OSError('controller unreachable')
        with self.assertLogs(agent_router.LOG, 'WARNING') as logs:
            with self.assertRaises(agent_api.AgentError) as failed:
                self.router.post('/api/agent/stop', {})
        payload = failed.exception.payload
        self.assertEqual(payload['code'], 'STOP_UNCONFIRMED')
        self.assertIn('arm2', payload['message'])
        self.assertNotIn('arm1', payload['message'])
        self.assertFalse(payload['stationary_observed'])
        # 一台臂失败不影响另一台臂的停止指令照常发出并得到确认。
        self.assertEqual(self.robot('arm1').stops, 1)
        self.assertTrue(payload['arms']['arm1']['stationary_observed'])
        self.assertFalse(payload['arms']['arm2']['stop_acknowledged'])
        self.assertIn('arm2', logs.output[0])

    def test_arm_without_a_camera_refuses_vision_with_a_clear_message(self):
        second = {'arm': ['arm2']}
        with self.assertRaises(agent_api.AgentError) as pose:
            self.router.get('/api/agent/pose', second)
        self.assertEqual(pose.exception.payload['code'], 'POSE_NOT_READY')
        self.assertRegex(
            pose.exception.payload['message'], '2臂.*尚未接入视觉'
        )
        with self.assertRaises(agent_api.AgentError) as frame:
            self.router.get('/api/agent/frame', second)
        self.assertEqual(
            frame.exception.payload['code'], 'CAMERA_UNAVAILABLE'
        )
        with self.assertRaisesRegex(ValueError, '尚未接入视觉'):
            self.router.post(
                '/api/agent/vision',
                {
                    'arm': 'arm2',
                    'operation': 'select_object',
                    'arguments': {'object_id': 'tube'},
                },
            )
        status = self.router.get('/api/agent/status', second)
        self.assertEqual(status['tracking']['state'], 'vision_unavailable')
        self.assertEqual(status['grasp']['arm'], 'arm2')
        rejected = self.start(
            'right-grasp-1',
            arm='arm2',
            operation='grasp',
            arguments={'object_id': 'tube'},
        )
        self.assertEqual(rejected['state'], 'rejected')
        self.assertIn('尚未接入视觉', rejected['error']['message'])
        self.assertEqual(self.robot('arm2').targets, [])

    def test_robot_state_is_read_from_the_requested_arm(self):
        self.robot('arm2').joint = [5000] * 7
        self.assertEqual(
            self.router.get('/api/agent/robot', {'arm': ['arm2']})['joint_deg'],
            [5.] * 7,
        )
        self.assertEqual(
            self.router.get('/api/agent/robot', {})['joint_deg'], [0.] * 7
        )
        self.assertEqual(self.robot('arm2').write_flags, [False])


class SingleArmCompatibilityTests(unittest.TestCase):
    def test_tracker_with_one_controller_behaves_as_before(self):
        with tempfile.TemporaryDirectory() as directory:
            root = pathlib.Path(directory)
            fleet = fake_controller.Fleet()
            arm = arms.default()
            path = fake_controller.write_arm_config(
                root, arm, '10.0.0.1', [1] * 7
            )
            tracker = types.SimpleNamespace(lock=threading.RLock(), state={})
            tracker.grasp = grasp_control.GraspController(root, tracker, path)
            token = root / 'token'
            token.write_text('x' * 40)
            token.chmod(0o600)
            with mock.patch.object(grasp_robot, 'RobotClient', fleet):
                router = agent_router.ArmRouter(tracker, token)
                self.assertEqual(list(router.apis), ['arm1'])
                result = router.post('/api/agent/stop', {})
            # 只有一台臂时停止的应答保持原来的形状，不多出 arms 字段。
            self.assertNotIn('arms', result)
            self.assertTrue(result['stationary_observed'])
            self.assertEqual(fleet.controller('10.0.0.1').stops, 1)


class ControllerCreationTests(unittest.TestCase):
    def setUp(self):
        directory = tempfile.TemporaryDirectory()
        self.addCleanup(directory.cleanup)
        self.root = pathlib.Path(directory.name)
        self.tracker = live_tracker.Tracker.__new__(live_tracker.Tracker)
        self.tracker.models_root = self.root / 'models'

    def create(self):
        with mock.patch.object(live_tracker, 'ROOT', self.root):
            return self.tracker._create_controllers(types.SimpleNamespace())

    def test_only_the_first_arm_until_the_second_is_configured(self):
        controllers = self.create()
        self.assertEqual(list(controllers), ['arm1'])
        self.assertIs(controllers['arm1'].tracker, self.tracker)
        self.assertEqual(
            controllers['arm1'].config_path,
            self.root / 'config/grasp/trial.json',
        )

    def test_configured_second_arm_gets_its_own_files_and_no_camera(self):
        fake_controller.write_arm_config(
            self.root, arms.by_id('arm2'), '10.0.0.2', [0] * 7
        )
        controllers = self.create()
        self.assertEqual(list(controllers), ['arm1', 'arm2'])
        second = controllers['arm2']
        self.assertEqual(
            second.config_path, self.root / 'config/grasp/arm2.json'
        )
        self.assertEqual(second.state_dir, self.root / 'work/grasp_arm2')
        self.assertIsInstance(second.tracker, no_vision.NoVision)
        self.assertIs(second.tracker.grasp, second)
        with self.assertRaisesRegex(ValueError, '2臂.*尚未接入视觉'):
            second.tracker.grasp_observation()


class AbortAllTests(unittest.TestCase):
    def test_every_arm_is_aborted_even_if_one_fails(self):
        first = mock.Mock()
        first.abort.side_effect = OSError('stop not acknowledged')
        second = mock.Mock()
        tracker = types.SimpleNamespace(arms={'arm1': first, 'arm2': second})
        with self.assertLogs(live_tracker.LOG, 'WARNING') as logs:
            with self.assertRaisesRegex(OSError, 'stop not acknowledged'):
                live_tracker.abort_all(tracker)
        second.abort.assert_called_once_with()
        self.assertIn('arm1', logs.output[0])


if __name__ == '__main__':
    unittest.main()
