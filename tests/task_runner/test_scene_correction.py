from tests.task_runner import test_runner
import asyncio
import copy
import json
import tempfile
import unittest
from saydo.task_runner import config as task_runner_config
from saydo.task_runner import robot_adapter
from saydo.task_runner import runner as task_runner_runner
from saydo.task_runner import store
from tests.task_runner import test_adapter
from tests.task_runner import test_scene_tools


class SceneCorrectionTests(unittest.IsolatedAsyncioTestCase):
    async def asyncSetUp(self):
        test_runner.mock_global_camera(self)

    def test_scene_annotations_are_idempotent_and_do_not_mutate_backend(self):
        adapter = robot_adapter.RobotAdapter(
            task_runner_config.Config(api_key='test'), test_adapter.Backend()
        )
        scene = {
            'scene_id': 'old', 'valid_for_execution': True, 'instances': []
        }
        view = adapter.scene_view(
            {'grasp': {'scene_snapshot': scene}}
        )['grasp']['scene_snapshot']
        self.assertTrue(view['backend_valid_for_execution'])
        self.assertFalse(view['valid_for_execution'])
        self.assertTrue(scene['valid_for_execution'])
        self.assertEqual(adapter.scene_view(view), view)
        adapter.held_scene_id = 'old'
        bound = adapter.scene_view(view)
        self.assertTrue(bound['usable_as_held_reference'])
        self.assertFalse(bound['usable_for_grasp_in_this_run'])
        adapter.scene_ids.add('old')
        self.assertTrue(
            adapter.scene_view(view)['usable_for_grasp_in_this_run']
        )

    async def test_repeated_missing_scene_is_bounded_without_submitting_motion(
        self
    ):
        backend = test_adapter.Backend()
        adapter = robot_adapter.RobotAdapter(
            task_runner_config.Config(api_key='test'), backend
        )
        for i in range(3):
            result = await adapter.execute(
                'grasp',
                {'scene_id': 'old'},
                str(i),
                asyncio.Event(),
                lambda _: None,
            )
            self.assertTrue(result['precondition_correction_required'])
        with self.assertRaises(robot_adapter.Blocked):
            await adapter.execute(
                'grasp',
                {'scene_id': 'old'},
                '4',
                asyncio.Event(),
                lambda _: None,
            )
        self.assertFalse(
            any(path == '/api/agent/tasks' for path, _ in backend.requests)
        )

    async def test_old_scene_then_capture_then_grasp_completes_without_duplicate_motion(
        self
    ):
        backend = test_adapter.Backend()
        config = task_runner_config.Config(api_key='test', poll_interval=.001)
        adapter = robot_adapter.RobotAdapter(config, backend)
        original = backend.request
        held = {
            'object_id': 'tube_model',
            'hold_id': 'h',
            'scene_id': 'fresh',
            'instance_id': 'tube',
        }

        def request(path, value=None):
            if path == '/api/agent/tasks':
                if value['operation'] == 'capture_scene_snapshot':
                    backend.result = {
                        'state': 'scene_ready',
                        'terminal': True,
                        'result': {
                            'scene_snapshot': {
                                'scene_id': 'fresh',
                                'state': 'ready',
                                'valid_for_execution': True,
                                'instances': test_scene_tools.TARGETS,
                            }
                        },
                    }
                elif value['operation'] == 'grasp':
                    backend.state['grasp']['held_object'] = held
                    backend.result = {
                        'state': 'completed',
                        'terminal': True,
                        'result': {'held_object': held},
                    }
            return original(path, value)

        backend.request = request
        llm = test_runner.FakeLLM(
            test_runner.call(
                'grasp',
                {
                    'object_id': 'tube_model',
                    'scene_id': 'old',
                    'instance_id': 'tube',
                },
                'old',
            ),
            test_runner.call(
                'capture_scene_snapshot',
                {
                    'targets': test_scene_tools.TARGETS,
                    'environment_static': True,
                },
                'capture',
            ),
            test_runner.call(
                'grasp',
                {
                    'object_id': 'tube_model',
                    'scene_id': 'fresh',
                    'instance_id': 'tube',
                },
                'grasp',
            ),
            test_runner.end('succeeded', ['grasp']),
        )
        with tempfile.TemporaryDirectory() as folder:
            runner = task_runner_runner.Runner(
                config, store.Store(folder), llm, lambda: adapter
            )
            ident = runner.submit('抓起物品')
            await runner.worker
            result = runner.store.get(ident)
            self.assertEqual(
                result['state'], 'succeeded', result.get('summary')
            )
            self.assertEqual(
                [
                    v['operation']
                    for p, v in backend.requests
                    if p == '/api/agent/tasks'
                ],
                ['capture_scene_snapshot', 'grasp'],
            )
            self.assertEqual(result['calls'][0]['state'], 'unavailable')
            self.assertEqual(len(result['verified_actions']), 1)

    def test_grasp_retires_historical_source_pose_but_preserves_reference(self):
        scene = {
            'scene_id': 's',
            'instances': [
                {'instance_id': 'source', 'base_T_object': ['old']},
                {'instance_id': 'reference', 'base_T_object': ['keep']},
            ],
        }
        history = [
            {
                'type': 'function_call_output',
                'call_id': 'capture',
                'output': json.dumps({'scene': scene}),
            }
        ]
        result = {
            'verified': True,
            'task': {
                'state': 'completed',
                'result': {
                    'held_object': {
                        'scene_id': 's',
                        'instance_id': 'source',
                        'base_T_object_at_grasp': ['old'],
                    }
                },
            },
        }
        before = copy.deepcopy(result)
        task_runner_runner.Runner.append_result(
            history, 'grasp', 'grasp', result
        )
        prior = json.loads(history[0]['output'])['scene']['instances']
        self.assertNotIn('base_T_object', prior[0])
        self.assertEqual(prior[1]['base_T_object'], ['keep'])
        self.assertEqual(result, before)
        self.assertNotIn(
            'base_T_object_at_grasp',
            json.loads(history[1]['output'])['task']['result']['held_object'],
        )
