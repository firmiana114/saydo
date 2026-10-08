"""Agent operations use existing controller entrypoints; all hardware here is mocked."""

import base64
import json
from pathlib import Path
import tempfile
import threading
import time
from types import SimpleNamespace
import unittest
from unittest.mock import Mock, patch

import cv2
import numpy as np

from agent_api import AgentAPI, AgentError
from grasp_control import GraspController


class AgentAPITests(unittest.TestCase):
    def setUp(self):
        self.directory = tempfile.TemporaryDirectory()
        self.addCleanup(self.directory.cleanup)
        self.root = Path(self.directory.name)
        self.config = self.root / 'trial.json'
        self.config.write_text(
            json.dumps(
                {
                    'robot_ip': '127.0.0.1',
                    'robot_port': 8080,
                    'gripper': {'force_target_raw': 10, 'open_position': 300},
                    'arm_speed_percent': 10,
                    'return_after_lift': {'joint_deg': [0] * 7},
                    'table_z_m': 0.,
                }
            )
        )
        observation = {
            'object_id': 'bottle',
            'time': time.time(),
            'pose': np.eye(4).tolist(),
            'generation': 1,
            'model_sha256': 'model',
        }
        self.tracker = SimpleNamespace(
            lock=threading.RLock(),
            state={'object_id': 'bottle', 'state': 'tracking'},
            models_root=self.root / 'models',
            grasp_observation=Mock(return_value=observation),
            latest={
                'bgr': np.full((24, 32, 3), 100, np.uint8),
                'time': time.time(),
                'sequence': 12,
            },
        )
        self.grasp = GraspController(
            self.root, self.tracker, self.config, 'execute'
        )
        self.tracker.grasp = self.grasp
        self.tracker.snapshot = lambda: {
            **self.tracker.state, 'grasp': self.grasp.snapshot()
        }
        self.token = self.root / 'token'
        self.token.write_text('x' * 40)
        self.token.chmod(0o600)
        self.api = AgentAPI(self.tracker, self.token)
        self.calls = []
        self.grasp.run = lambda obs, preview=False: self.complete(
            'preview' if preview else 'completed', ('grasp', preview)
        )
        self.grasp.run_reset = lambda config, target, kind: self.complete(
            'completed', ('reset', kind)
        )
        self.grasp.run_placement = (
            lambda held, settings, preview: self.complete(
                'placement_preview' if preview else 'completed',
                ('place', preview),
            )
        )

    def complete(self, state, call):
        self.calls.append(call)
        self.grasp.update(state=state, message='test result', busy=False)

    def request(
        self, operation='grasp', task_id='request-0001', arguments=None
    ):
        return self.api.start(
            {
                'operation': operation,
                'request_id': task_id,
                'arguments': arguments or {'object_id': 'bottle'},
            }
        )

    def wait(self, task_id='request-0001'):
        self.grasp.thread.join(timeout=2)
        self.assertFalse(self.grasp.thread.is_alive())
        return self.api.get_task(task_id)

    def test_candidate_frame_matches_frozen_selection_and_expires(self):
        self.tracker.frozen = {
            'bgr': np.full((24, 32, 3), 30, np.uint8),
            'time': time.time() - 5,
            'sequence': 7,
        }
        self.tracker.generation = 4
        self.tracker.state['state'] = 'choose_candidate'
        frame = self.api.frame(selection=True)
        self.assertEqual(frame['sequence'], 7)
        self.assertEqual(frame['selection_id'], 4)
        self.assertEqual(frame['image_kind'], 'frozen_selection_frame')
        self.assertEqual(self.api.frame()['sequence'], 12)
        self.tracker.frozen['time'] -= 31
        with self.assertRaises(AgentError):
            self.api.frame(selection=True)
        self.tracker.state['state'] = 'tracking'
        with self.assertRaises(AgentError):
            self.api.frame(selection=True)

    def test_grasp_completion_duplicate_and_conflicting_retry(self):
        self.request()
        result = self.wait()
        self.assertTrue(result['terminal'])
        self.assertEqual(result['state'], 'completed')
        self.assertEqual(self.request()['task_id'], 'request-0001')
        self.assertEqual(self.calls, [('grasp', False)])
        with self.assertRaises(AgentError) as context:
            self.request(arguments={'object_id': 'cup'})
        self.assertEqual(
            context.exception.payload['code'], 'REQUEST_ID_CONFLICT'
        )

    def test_concurrent_duplicate_only_launches_one_worker(self):
        release = threading.Event()
        started = threading.Event()

        def run(*args, **kwargs):
            started.set()
            release.wait(2)
            self.complete('completed', ('grasp', False))

        self.grasp.run = run
        self.request()
        self.assertTrue(started.wait(1))
        results = []
        threads = [
            threading.Thread(target=lambda: results.append(self.request()))
            for _ in range(6)
        ]
        for t in threads:
            t.start()
        for t in threads:
            t.join(1)
        self.assertEqual(len(results), 6)
        self.assertTrue(all(r['deduplicated'] for r in results))
        release.set()
        self.wait()
        self.assertEqual(len(self.calls), 1)

    def test_old_task_not_replaced_by_later_ui_status(self):
        self.request()
        first = self.wait()
        self.grasp.update(
            state='failed', message='later unrelated UI job', busy=False
        )
        self.assertEqual(self.api.get_task('request-0001'), first)

    def test_wrong_object_and_null_identifier_never_execute(self):
        result = self.request(arguments={'object_id': 'cup'})
        self.assertEqual(result['state'], 'rejected')
        self.assertEqual(self.calls, [])
        with self.assertRaises(AgentError):
            self.api.start(
                {
                    'operation': 'grasp',
                    'request_id': 'request-null',
                    'arguments': {'object_id': None},
                }
            )
        self.assertEqual(self.calls, [])

    def test_preview_mode_blocks_actions_but_allows_preview(self):
        self.grasp.mode = 'preview'
        with self.assertRaises(AgentError) as context:
            self.request()
        self.assertEqual(
            context.exception.payload['code'], 'EXECUTION_DISABLED'
        )
        self.request('preview_grasp')
        self.assertEqual(self.wait()['state'], 'preview')
        self.assertEqual(self.calls, [('grasp', True)])

    def test_reset_reuses_existing_target_and_cannot_run_while_stopping(self):
        self.request('reset', arguments={'target': 'zero'})
        self.wait()
        self.assertEqual(self.calls, [('reset', 'zero')])
        self.grasp.status['stopping'] = True
        with self.assertRaises(AgentError):
            self.request('reset', 'request-0002', {'target': 'observation'})
        with self.assertRaises(ValueError):
            self.grasp.start_reset('observation')
        with self.assertRaises(ValueError):
            self.grasp.start()

    def holding(self):
        self.grasp.held_object = {
            'hold_id': 'hold-1',
            'object_id': 'bottle',
            'model_sha256': 'model',
            'orientation_version': 2,
        }
        settings = {
            'region_xy_m': {
                'x_min': .3, 'x_max': .5, 'y_min': -.5, 'y_max': -.2
            }
        }
        self.grasp.placement_configuration = lambda: {
            'error': None, 'settings': settings, 'revision': 'place-v1'
        }
        return {
            'hold_id': 'hold-1',
            'orientation_version': 2,
            'placement_revision': 'place-v1',
        }

    def test_placement_binds_hold_orientation_and_configuration(self):
        args = self.holding()
        self.request('preview_place', arguments=args)
        self.assertEqual(self.wait()['state'], 'placement_preview')
        self.request('place', 'request-0002', args)
        self.wait('request-0002')
        self.assertEqual(self.calls, [('place', True), ('place', False)])
        for index, changed in enumerate(
            [
                {'hold_id': 'other'},
                {'orientation_version': 1},
                {'placement_revision': 'old'},
            ]
        ):
            result = self.request(
                'place', 'rejected-' + str(index), {**args, **changed}
            )
            self.assertEqual(result['state'], 'rejected')
        self.assertEqual(len(self.calls), 2)

    def test_restart_marks_inflight_unknown_and_never_replays_it(self):
        # Simulate a process dying after durable insertion and before final
        # callback.
        self.grasp.start = Mock(
            side_effect=lambda **kw: self.grasp.status.update(
                state='moving', busy=True
            )
        )
        self.request()
        self.grasp.status['busy'] = False
        replacement = AgentAPI(self.tracker, self.token)
        record = replacement.get_task('request-0001')
        self.assertEqual(record['state'], 'interrupted')
        self.assertTrue(record['outcome_unknown'])
        result = replacement.start(
            {
                'operation': 'grasp',
                'request_id': 'request-0001',
                'arguments': {'object_id': 'bottle'},
            }
        )
        self.assertTrue(result['deduplicated'])
        self.grasp.start.assert_called_once()

    def test_missing_task_and_bad_auth(self):
        with self.assertRaises(AgentError):
            self.api.get_task('unknown')
        self.assertTrue(self.api.authorized('Bearer ' + 'x' * 40))
        for value in [None, 'Bearer wrong', 'Bearer 中文']:
            self.assertFalse(self.api.authorized(value))

    def test_frame_is_latest_not_frozen_and_has_capture_time(self):
        self.tracker.frozen = {
            'bgr': np.zeros((24, 32, 3), np.uint8), 'time': 0, 'sequence': 1
        }
        result = self.api.frame()
        image = cv2.imdecode(
            np.frombuffer(base64.b64decode(result['image_base64']), np.uint8),
            cv2.IMREAD_COLOR,
        )
        self.assertEqual(result['sequence'], 12)
        self.assertEqual(result['captured_at'], self.tracker.latest['time'])
        self.assertGreater(image.mean(), 90)
        self.tracker.latest['time'] = time.time() - 2
        with self.assertRaises(AgentError):
            self.api.frame()

    def test_pose_keeps_camera_frame_and_rejects_stale_observation(self):
        pose = self.api.pose()
        self.assertEqual(pose['coordinate_frame'], 'color_optical_frame')
        self.assertEqual(pose['translation_unit'], 'metre')
        self.tracker.grasp_observation.side_effect = ValueError('视觉位姿过期')
        with self.assertRaises(AgentError) as context:
            self.api.pose()
        self.assertEqual(context.exception.payload['code'], 'POSE_NOT_READY')

    def robot(self, joints=None):
        robot = Mock()
        data = {
            'get_current_arm_state': {
                'arm_state': {
                    'joint': joints or [1000] * 7,
                    'pose': [100000, 200000, 300000, 100, 200, 300],
                    'err': [0],
                }
            },
            'get_rm_plus_state_info': {'state_info': {'speed': [0]}},
            'get_joint_en_state': {'en_state': [1] * 7},
            'get_joint_err_flag': {'err_flag': [0] * 7},
            'get_current_tool_frame': {'frame_name': 'grip_tcp'},
            'get_current_work_frame': {'frame_name': 'Base'},
        }
        robot.query.side_effect = lambda command: data[command]
        return robot

    def test_robot_query_units_and_no_writes(self):
        robot = self.robot()
        with patch('agent_api.RobotClient', return_value=robot) as factory:
            result = self.api.robot_state()
            factory.assert_called_once_with('127.0.0.1', 8080, writes=False)
        self.assertEqual(result['joint_deg'], [1] * 7)
        self.assertEqual(result['tcp_position_m'], [.1, .2, .3])
        robot.stop.assert_not_called()

    def test_stop_without_owned_task_still_stops_hardware_and_confirms_standstill(
        self
    ):
        robot = self.robot()
        with patch('agent_api.RobotClient', return_value=robot) as factory:
            result = self.api.stop_robot()
            factory.assert_called_once_with('127.0.0.1', 8080, writes=True)
        robot.stop.assert_called_once()
        self.assertTrue(result['stationary_observed'])
        self.assertFalse(self.grasp.status['stopping'])
        self.assertTrue(self.grasp.cancelled.is_set())
        robot.move_gripper.assert_not_called()

    def test_stop_failure_latches_and_prevents_new_actions(self):
        robot = self.robot()
        robot.stop.side_effect = OSError('disconnected')
        with patch('agent_api.RobotClient', return_value=robot):
            with self.assertRaises(AgentError) as context:
                self.api.stop_robot()
        self.assertEqual(context.exception.payload['code'], 'STOP_UNCONFIRMED')
        self.assertTrue(self.grasp.status['stopping'])
        with self.assertRaises(AgentError):
            self.request()
        robot.stop.side_effect = None
        with patch('agent_api.RobotClient', return_value=robot):
            self.assertTrue(self.api.stop_robot()['stationary_observed'])

    def test_tracking_requires_matching_model_and_no_motion(self):
        self.tracker.automatic = Mock()
        self.tracker.freeze = Mock(return_value={'selection_id': 8})
        self.tracker.initialize = Mock()
        with self.assertRaises(AgentError):
            self.api.vision(
                'start_tracking', {'object_id': 'other', 'box': None}
            )
        self.api.vision(
            'start_tracking', {'object_id': 'bottle', 'box': [10, 20, 100, 200]}
        )
        self.tracker.initialize.assert_called_once_with(
            {'selection_id': 8, 'box': [10, 20, 100, 200]}
        )
        self.assertEqual(self.calls, [])

    def test_real_http_authentication_task_dispatch_and_image_routes(self):
        import http.client
        from http.server import ThreadingHTTPServer
        import live_tracker
        t = self.tracker
        t.stop = threading.Event()
        t.events = Mock()
        t.event = Mock()
        t.capture = lambda: t.stop.wait()
        t.inference = lambda: t.stop.wait()
        ready = threading.Event()
        servers = []
        errors = []

        def make_server(address, handler):
            server = ThreadingHTTPServer(('127.0.0.1', 0), handler)
            servers.append(server)
            ready.set()
            return server

        def serve():
            try:
                live_tracker.main()
            except Exception as error:
                errors.append(error)
                ready.set()

        with patch.object(live_tracker, 'ROOT', self.root), patch.object(
            live_tracker, 'Tracker', return_value=t
        ), patch.object(
            live_tracker, 'ThreadingHTTPServer', side_effect=make_server
        ), patch.object(live_tracker.signal, 'signal'), patch.object(
            live_tracker.urllib.request,
            'urlopen',
            side_effect=OSError('no camera service'),
        ), patch(
            'sys.argv',
            ['live_tracker.py', '--agent-token-file', str(self.token)],
        ):
            thread = threading.Thread(target=serve, daemon=True)
            thread.start()
            client = None
            try:
                self.assertTrue(ready.wait(3))
                self.assertFalse(errors)
                client = http.client.HTTPConnection(
                    *servers[0].server_address, timeout=4
                )
                body = json.dumps(
                    {
                        'operation': 'grasp',
                        'request_id': 'http-task-1',
                        'arguments': {'object_id': 'bottle'},
                    }
                )
                client.request('POST', '/api/agent/tasks', body)
                response = client.getresponse()
                self.assertEqual(response.status, 401)
                response.read()
                self.assertEqual(self.calls, [])
                headers = {
                    'Authorization': 'Bearer ' + 'x' * 40,
                    'Content-Type': 'application/json',
                }
                client.request('POST', '/api/agent/tasks', body, headers)
                response = client.getresponse()
                self.assertEqual(response.status, 200)
                self.assertEqual(
                    json.loads(response.read())['task_id'], 'http-task-1'
                )
                self.grasp.thread.join(2)
                client.request(
                    'GET', '/api/agent/tasks/http-task-1', headers=headers
                )
                response = client.getresponse()
                self.assertTrue(json.loads(response.read())['terminal'])
                client.request('POST', '/api/agent/tasks', body, headers)
                response = client.getresponse()
                self.assertTrue(json.loads(response.read())['deduplicated'])
                self.assertEqual(len(self.calls), 1)
                client.request('GET', '/api/agent/frame', headers=headers)
                response = client.getresponse()
                self.assertEqual(json.loads(response.read())['sequence'], 12)
            finally:
                if client:
                    client.close()
                t.stop.set()
                if servers:
                    servers[0].shutdown()
                thread.join(5)
            self.assertFalse(thread.is_alive())
            self.assertFalse(errors)


if __name__ == '__main__':
    unittest.main()
