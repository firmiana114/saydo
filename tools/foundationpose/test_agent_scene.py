import copy
import threading
import unittest
from unittest.mock import Mock, patch
import test_scene_snapshot as fixtures
from agent_api import AgentAPI


class SceneAPITests(unittest.TestCase):
    def setUp(self):
        fixtures.SceneTests.setUp(self)
        self.tracker.grasp = self.service
        self.tracker.snapshot = lambda: {
            **self.tracker.state, 'grasp': self.service.snapshot()
        }
        token = self.root / 'token'
        token.write_text('x' * 48)
        token.chmod(0o600)
        self.api = AgentAPI(self.tracker, token)

    def capture(self):
        request = {
            'operation': 'capture_scene_snapshot',
            'request_id': 'capture-scene-001',
            'arguments': {
                'targets': fixtures.TARGETS, 'environment_static': True
            },
        }
        with patch(
            'scene_control.RobotClient', return_value=self.robot
        ), patch('scene_control.check_camera_observation'):
            task = self.api.start(request)
            self.service.thread.join(3)
        task = self.api.get_task(request['request_id'])
        self.assertTrue(task['terminal'])
        self.assertEqual(task['state'], 'scene_ready', task)
        self.assertTrue(self.api.start(request)['deduplicated'])
        return task['result']['scene_id']

    def test_capture_get_and_snapshot_grasp_use_backend_reference(self):
        ident = self.capture()
        scene = self.api.get('/api/agent/scene-snapshot', {'scene_id': [ident]})
        self.assertTrue(scene['valid_for_execution'])
        self.assertEqual(len(scene['instances']), 2)
        self.assertNotIn(
            'scene_snapshot',
            self.api.status()['grasp'],
            'status must not expose obsolete capture-time instance states',
        )
        self.service.run = Mock(
            side_effect=lambda observation, preview=False: self.service.update(
                state='preview', busy=False
            )
        )
        request = {
            'operation': 'preview_grasp',
            'request_id': 'grasp-scene-001',
            'arguments': {
                'object_id': 'source', 'scene_id': ident, 'instance_id': 'tube'
            },
        }
        self.api.start(request)
        self.service.thread.join(3)
        self.assertEqual(
            self.api.get_task(request['request_id'])['state'], 'preview'
        )
        observation = self.service.run.call_args.args[0]
        self.assertEqual(
            observation['base_T_object'], scene['instances'][0]['base_T_object']
        )
        self.robot.move_gripper.assert_not_called()

    def test_reference_failure_rejects_grasp_even_with_legacy_arguments(self):
        self.tracker.localize_frozen_scene.side_effect = ValueError(
            'rack unavailable'
        )
        request = {
            'operation': 'capture_scene_snapshot',
            'request_id': 'capture-failed-001',
            'arguments': {
                'targets': fixtures.TARGETS, 'environment_static': True
            },
        }
        with patch(
            'scene_control.RobotClient', return_value=self.robot
        ), patch('scene_control.check_camera_observation'):
            self.api.start(request)
            self.service.thread.join(3)
        self.assertEqual(
            self.api.get_task(request['request_id'])['state'], 'failed'
        )
        self.service.run = Mock()
        task = self.api.start(
            {
                'operation': 'grasp',
                'request_id': 'legacy-bypass-001',
                'arguments': {'object_id': 'source'},
            }
        )
        self.assertEqual(task['state'], 'rejected')
        self.service.run.assert_not_called()
