from tests.task_runner import test_runner
import asyncio
import copy
import json
import tempfile
import unittest
import pathlib
from saydo.task_runner import config
from saydo.task_runner import robot_adapter
from saydo.task_runner import store as task_runner_store
from saydo.task_runner import reconcile
from saydo.task_runner import tool_registry
from tests.task_runner import test_adapter
from saydo.task_runner import runner as task_runner_runner

I = [[1, 0, 0], [0, 1, 0], [0, 0, 1]]
Z = [[0, -1, 0], [1, 0, 0], [0, 0, 1]]
ARGS = {
    'object_id': 'b',
    'hold_id': 'h',
    'orientation_version': 0,
    'target_rpy_deg': [0, 0, 90],
    'position_mode': 'adaptive',
}


class ReorientationBackend(test_adapter.Backend):
    def __init__(self):
        super().__init__()
        self.state['grasp']['held_object'] = {
            'object_id': 'b',
            'hold_id': 'h',
            'orientation_version': 0,
            'current_object_rotation': I,
        }
        self.bad_version = False
        self.bad_rotation = False

    def request(self, path, value=None):
        if (
            path == '/api/agent/tasks'
            and value['operation'] in ('reorient', 'preview_reorient')
        ):
            preview = value['operation'] == 'preview_reorient'
            if not preview:
                self.state['grasp']['held_object'].update(
                    orientation_version=0 if self.bad_version else 1,
                    current_object_rotation=I if self.bad_rotation else Z,
                )
            self.result = {
                'state': 'reorient_preview' if preview else 'completed',
                'terminal': True,
                'result': {
                    'held_object': copy.deepcopy(
                        self.state['grasp']['held_object']
                    ),
                    'reorientation_plan': {
                        'base_T_object_target': (
                            [r + [0] for r in Z] + [[0, 0, 0, 1]]
                        ),
                        'orientation_tolerance_deg': 1,
                    },
                },
            }
        if path == '/api/agent/tasks' and value['operation'] == 'place':
            assert value['arguments']['orientation_version'] == 1
            self.state['grasp']['held_object'] = None
            self.result = {
                'state': 'completed',
                'terminal': True,
                'result': {'held_object': None},
            }
        return super().request(path, value)


class ReorientationAdapterTests(unittest.IsolatedAsyncioTestCase):
    async def asyncSetUp(self):
        test_runner.mock_global_camera(self)

    def setUp(self):
        self.backend = ReorientationBackend()
        self.config = config.Config(
            api_key='test', mode='execute', poll_interval=.001
        )
        self.adapter = robot_adapter.RobotAdapter(self.config, self.backend)

    async def test_initial_read_is_unchanged(self):
        self.assertIn('status', await self.adapter.initial())
        self.assertTrue(all(v is None for _, v in self.backend.requests))

    async def test_5d_feedback_ignores_spin_but_rejects_axis_error_or_mode_change(
        self
    ):
        held = {
            'object_id': 'b',
            'hold_id': 'h',
            'orientation_version': 1,
            'current_object_rotation': I,
            'pose_mode': '5d',
        }
        task = {
            'result': {
                'held_object': copy.deepcopy(held),
                'reorientation_plan': {
                    'base_T_object_target': (
                        [row + [0] for row in Z] + [[0, 0, 0, 1]]
                    ),
                    'pose_mode': '5d',
                    'orientation_tolerance_deg': 1,
                },
            }
        }
        status = {'grasp': {'held_object': copy.deepcopy(held)}}
        robot_adapter.RobotAdapter.verify_reorientation(
            'reorient', ARGS, task, status
        )
        status['grasp']['held_object']['current_object_rotation'] = [
            [1, 0, 0], [0, 0, -1], [0, 1, 0]
        ]
        with self.assertRaises(robot_adapter.Blocked):
            robot_adapter.RobotAdapter.verify_reorientation(
                'reorient', ARGS, task, status
            )
        status['grasp']['held_object'].update(
            current_object_rotation=I, pose_mode='6d'
        )
        with self.assertRaises(robot_adapter.Blocked):
            robot_adapter.RobotAdapter.verify_reorientation(
                'reorient', ARGS, task, status
            )

    async def test_execute_waits_verifies_angle_version_and_keeps_parameters(
        self
    ):
        result = await self.adapter.execute(
            'reorient', ARGS, 'reorient-1', asyncio.Event(), lambda _: None
        )
        self.assertTrue(result['verified'])
        sent = next(
            v for p, v in self.backend.requests if p == '/api/agent/tasks'
        )
        self.assertEqual(sent['arguments'], ARGS)
        self.assertEqual(
            result['status']['grasp']['held_object']['orientation_version'], 1
        )

    async def test_preview_does_not_increment_orientation(self):
        self.config.mode = 'preview'
        result = await self.adapter.execute(
            'preview_reorient',
            ARGS,
            'preview-1',
            asyncio.Event(),
            lambda _: None,
        )
        self.assertFalse(result['verified'])
        self.assertEqual(
            result['status']['grasp']['held_object']['orientation_version'], 0
        )

    async def test_stale_version_does_not_submit(self):
        with self.assertRaises(robot_adapter.Blocked):
            await self.adapter.execute(
                'reorient',
                {**ARGS, 'orientation_version': 9},
                'stale-1',
                asyncio.Event(),
                lambda _: None,
            )
        self.assertFalse(any(v is not None for _, v in self.backend.requests))

    async def test_missing_or_wrong_feedback_blocks_success(self):
        for bad in ('bad_version', 'bad_rotation'):
            backend = ReorientationBackend()
            setattr(backend, bad, True)
            with self.assertRaises(robot_adapter.Blocked) as error:
                await robot_adapter.RobotAdapter(self.config, backend).execute(
                    'reorient',
                    ARGS,
                    'bad-feedback',
                    asyncio.Event(),
                    lambda _: None,
                )
            self.assertTrue(error.exception.uncertain)

    async def test_runner_reorientation_then_place_uses_new_version(self):
        llm = test_runner.FakeLLM(
            test_runner.call('reorient', ARGS, 'r'),
            test_runner.call(
                'place',
                {
                    'hold_id': 'h',
                    'orientation_version': 1,
                    'placement_revision': 'v',
                },
                'p',
            ),
            test_runner.end('succeeded', ['r', 'p']),
        )
        with tempfile.TemporaryDirectory() as directory:
            runner = task_runner_runner.Runner(
                self.config,
                task_runner_store.Store(directory),
                llm,
                lambda: self.adapter,
            )
            ident = runner.submit('将物品转至目标朝向，再放下')
            await runner.worker
            result = runner.store.get(ident)
            self.assertEqual(result['state'], 'succeeded')
            self.assertEqual(
                [x['name'] for x in result['verified_actions']],
                ['reorient', 'place'],
            )


class ReorientationContractTests(unittest.TestCase):
    def test_modes_and_validation(self):
        self.assertIn(
            'preview_reorient',
            {s['name'] for s in tool_registry.schemas('preview')},
        )
        self.assertNotIn(
            'reorient', {s['name'] for s in tool_registry.schemas('preview')}
        )
        self.assertNotIn(
            'preview_reorient',
            {s['name'] for s in tool_registry.schemas('read_only')},
        )
        for override in (
            {'target_rpy_deg': [0, 0]},
            {'target_rpy_deg': [0, True, 0]},
            {'target_rpy_deg': [0, 0, 181]},
            {'orientation_version': True},
            {'position_mode': 'free'},
        ):
            with self.assertRaises(ValueError):
                tool_registry.validate(
                    test_runner.call(
                        'reorient', {**ARGS, **override}
                    )['output'][0],
                    'execute',
                )

    def test_interrupted_reorientation_is_unknown_and_reconciled_by_original_id(
        self
    ):
        with tempfile.TemporaryDirectory() as directory:
            store = task_runner_store.Store(directory)
            run = store.create('旋转', 'terra', 'execute')
            record = store.prepare(run['run_id'], 'r', 'reorient', ARGS, 1)
            store = task_runner_store.Store(directory)
            self.assertTrue(store.get(run['run_id'])['outcome_unknown'])
            backend = test_adapter.Backend()
            reconcile.reconcile(store, backend, run['run_id'])
            self.assertIn(
                ('/api/agent/tasks/' + record['request_id'], None),
                backend.requests,
            )
            self.assertFalse(store.has_uncertain_runs())
