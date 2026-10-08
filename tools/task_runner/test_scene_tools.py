import asyncio
import copy
import unittest
from .config import Config
from .robot_adapter import RobotAdapter, Blocked, compact
from .tool_registry import schemas, validate
from .test_adapter import Backend
import json

TARGETS = [
    {
        'instance_id': 'tube',
        'object_id': 'tube_model',
        'role': 'source',
        'box': None,
    },
    {
        'instance_id': 'rack',
        'object_id': 'rack_model',
        'role': 'reference',
        'box': None,
    },
]


class SceneAdapterTests(unittest.IsolatedAsyncioTestCase):
    async def test_capture_then_grasp_preserves_identity_and_does_not_switch_tracking(
        self
    ):
        backend = Backend()
        adapter = RobotAdapter(
            Config(api_key='test', mode='execute', poll_interval=.001), backend
        )
        cancel = asyncio.Event()
        scene = {
            'scene_id': 'f' * 32,
            'state': 'ready',
            'valid_for_execution': True,
            'instances': TARGETS,
        }
        backend.result = {
            'state': 'scene_ready',
            'terminal': True,
            'result': {'scene_snapshot': scene},
        }
        result = await adapter.execute(
            'capture_scene_snapshot',
            {'targets': TARGETS, 'environment_static': True},
            'capture-1',
            cancel,
            lambda _: None,
        )
        self.assertEqual(adapter.scene_ids, {scene['scene_id']})
        held = {
            'object_id': 'tube_model',
            'hold_id': 'h',
            'scene_id': scene['scene_id'],
            'instance_id': 'tube',
        }
        backend.result = {
            'state': 'completed',
            'terminal': True,
            'result': {'held_object': held},
        }
        original = backend.request

        def request(path, value=None):
            if (
                path == '/api/agent/tasks'
                and value
                and value['operation'] == 'grasp'
            ):
                backend.state['grasp']['held_object'] = held
            return original(path, value)

        backend.request = request
        args = {
            'object_id': 'tube_model',
            'scene_id': scene['scene_id'],
            'instance_id': 'tube',
        }
        result = await adapter.execute(
            'grasp', args, 'grasp-1', cancel, lambda _: None
        )
        self.assertTrue(result['verified'])
        self.assertFalse(
            any(p == '/api/agent/vision' for p, _ in backend.requests)
        )
        self.assertEqual(
            [
                v for p, v in backend.requests if p == '/api/agent/tasks'
            ][-1]['arguments'],
            args,
        )

    async def test_uncaptured_and_failed_capture_cannot_use_old_scene(self):
        backend = Backend()
        adapter = RobotAdapter(
            Config(api_key='test', poll_interval=.001), backend
        )
        cancel = asyncio.Event()
        args = {
            'object_id': 'tube_model', 'scene_id': 'old', 'instance_id': 'tube'
        }
        result = await adapter.execute(
            'grasp', args, 'g1', cancel, lambda _: None
        )
        self.assertEqual(result['error']['code'], 'SCENE_CAPTURE_REQUIRED')
        self.assertFalse(result['backend_submitted'])
        adapter.scene_ids.add('old')
        backend.result = {
            'state': 'failed', 'terminal': True, 'error': 'rack not visible'
        }
        result = await adapter.execute(
            'capture_scene_snapshot',
            {'targets': TARGETS, 'environment_static': True},
            'c1',
            cancel,
            lambda _: None,
        )
        self.assertIn('error', result)
        self.assertEqual(adapter.scene_ids, set())
        result = await adapter.execute(
            'grasp', args, 'g2', cancel, lambda _: None
        )
        self.assertTrue(result['precondition_correction_required'])
        self.assertFalse(
            any(
                v and v.get('operation') == 'grasp' for p, v in backend.requests
            )
        )

    def test_models_receive_all_bounded_named_features_and_scene_schema(self):
        data = {
            'features': [
                {'id': str(i), 'object_T_feature': [[1, 0], [0, 1]]}
                for i in range(120)
            ]
        }
        self.assertEqual(len(compact(data)['features']), 120)
        names = {d['name'] for d in schemas('execute')}
        self.assertIn('capture_scene_snapshot', names)
        self.assertNotIn(
            'capture_scene_snapshot', {d['name'] for d in schemas('read_only')}
        )
        args = {
            'targets': TARGETS,
            'environment_static': True,
            'progress_message': '先定位所有目标',
        }
        name, _ = validate(
            {
                'type': 'function_call',
                'name': 'capture_scene_snapshot',
                'call_id': 'c',
                'arguments': json.dumps(args),
            },
            'execute',
        )
        self.assertEqual(name, 'capture_scene_snapshot')
