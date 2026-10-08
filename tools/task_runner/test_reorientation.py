from .test_runner import mock_global_camera
import asyncio
import copy
import json
import tempfile
import unittest
from pathlib import Path
from .config import Config
from .robot_adapter import RobotAdapter, Blocked
from .store import Store
from .reconcile import reconcile
from .tool_registry import schemas, validate
from .test_adapter import Backend
from .runner import Runner
from .test_runner import FakeLLM, call, end

I = [[1, 0, 0], [0, 1, 0], [0, 0, 1]]
Z = [[0, -1, 0], [1, 0, 0], [0, 0, 1]]
ARGS = {
    'object_id': 'b',
    'hold_id': 'h',
    'orientation_version': 0,
    'target_rpy_deg': [0, 0, 90],
    'position_mode': 'adaptive',
}


class ReorientationBackend(Backend):
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
        mock_global_camera(self)

    def setUp(self):
        self.backend = ReorientationBackend()
        self.config = Config(api_key='test', mode='execute', poll_interval=.001)
        self.adapter = RobotAdapter(self.config, self.backend)

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
        RobotAdapter.verify_reorientation('reorient', ARGS, task, status)
        status['grasp']['held_object']['current_object_rotation'] = [
            [1, 0, 0], [0, 0, -1], [0, 1, 0]
        ]
        with self.assertRaises(Blocked):
            RobotAdapter.verify_reorientation('reorient', ARGS, task, status)
        status['grasp']['held_object'].update(
            current_object_rotation=I, pose_mode='6d'
        )
        with self.assertRaises(Blocked):
            RobotAdapter.verify_reorientation('reorient', ARGS, task, status)

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
        with self.assertRaises(Blocked):
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
            with self.assertRaises(Blocked) as error:
                await RobotAdapter(self.config, backend).execute(
                    'reorient',
                    ARGS,
                    'bad-feedback',
                    asyncio.Event(),
                    lambda _: None,
                )
            self.assertTrue(error.exception.uncertain)

    async def test_runner_reorientation_then_place_uses_new_version(self):
        llm = FakeLLM(
            call('reorient', ARGS, 'r'),
            call(
                'place',
                {
                    'hold_id': 'h',
                    'orientation_version': 1,
                    'placement_revision': 'v',
                },
                'p',
            ),
            end('succeeded', ['r', 'p']),
        )
        with tempfile.TemporaryDirectory() as directory:
            runner = Runner(
                self.config, Store(directory), llm, lambda: self.adapter
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
            'preview_reorient', {s['name'] for s in schemas('preview')}
        )
        self.assertNotIn('reorient', {s['name'] for s in schemas('preview')})
        self.assertNotIn(
            'preview_reorient', {s['name'] for s in schemas('read_only')}
        )
        for override in (
            {'target_rpy_deg': [0, 0]},
            {'target_rpy_deg': [0, True, 0]},
            {'target_rpy_deg': [0, 0, 181]},
            {'orientation_version': True},
            {'position_mode': 'free'},
        ):
            with self.assertRaises(ValueError):
                validate(
                    call('reorient', {**ARGS, **override})['output'][0],
                    'execute',
                )

    def test_interrupted_reorientation_is_unknown_and_reconciled_by_original_id(
        self
    ):
        with tempfile.TemporaryDirectory() as directory:
            store = Store(directory)
            run = store.create('旋转', 'terra', 'execute')
            record = store.prepare(run['run_id'], 'r', 'reorient', ARGS, 1)
            store = Store(directory)
            self.assertTrue(store.get(run['run_id'])['outcome_unknown'])
            backend = Backend()
            reconcile(store, backend, run['run_id'])
            self.assertIn(
                ('/api/agent/tasks/' + record['request_id'], None),
                backend.requests,
            )
            self.assertFalse(store.has_uncertain_runs())
