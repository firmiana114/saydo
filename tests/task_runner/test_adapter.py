import asyncio
import copy
from pathlib import Path
import threading
import unittest
from saydo.agent.client import BackendError
from saydo.task_runner.config import Config
from saydo.task_runner.robot_adapter import RobotAdapter, Blocked


class Backend:
    def __init__(self):
        self.requests = []
        self.submission_error = False
        self.query_error = False
        self.state = {
            'session_id': 's',
            'active_task_id': None,
            'grasp': {'mode': 'execute', 'busy': False, 'held_object': None},
            'tracking': {},
        }
        self.result = {
            'state': 'completed',
            'terminal': True,
            'result': {'held_object': None},
        }

    def request(self, path, value=None):
        self.requests.append((path, copy.deepcopy(value)))
        if path == '/api/agent/status':
            return copy.deepcopy(self.state)
        if path == '/api/agent/tasks':
            if self.submission_error:
                raise BackendError({'code': 'REQUEST_UNCERTAIN'})
            return {'terminal': False}
        if path.startswith('/api/agent/tasks/'):
            if self.query_error:
                raise BackendError({'code': 'TASK_NOT_FOUND'})
            return copy.deepcopy(self.result)
        if path == '/api/agent/stop':
            return {'stop_acknowledged': True, 'stationary_observed': True}
        return {}


class AdapterTests(unittest.IsolatedAsyncioTestCase):
    def setUp(self):
        self.config = Config(
            api_key='test',
            mode='execute',
            poll_interval=.001,
            action_timeout=.05,
        )
        self.backend = Backend()
        self.adapter = RobotAdapter(self.config, self.backend)
        self.cancelled = asyncio.Event()

    async def test_vision_uses_matching_candidate_frame(self):
        self.backend.state['tracking'] = {
            'state': 'choose_candidate', 'selection_id': 3
        }
        original = self.backend.request

        def request(path, value=None):
            if path == '/api/agent/selection-frame':
                return {'selection_id': 3, 'sequence': 7}
            return original(path, value)

        self.backend.request = request
        result = await self.adapter.execute(
            'start_tracking',
            {'object_id': 'cup'},
            'id-12345',
            self.cancelled,
            lambda _: None,
        )
        self.assertEqual(result['candidate_frame']['sequence'], 7)
        self.backend.state['tracking']['selection_id'] = 4
        with self.assertRaises(Blocked):
            await self.adapter.execute(
                'start_tracking',
                {'object_id': 'cup'},
                'id-12345',
                self.cancelled,
                lambda _: None,
            )

    async def test_exhausted_detection_returns_failure_without_waiting_for_timeout(
        self
    ):
        self.backend.state['tracking'] = {'state': 'lost'}
        with self.assertRaisesRegex(Blocked, '初始化失败'):
            await self.adapter.execute(
                'start_tracking',
                {'object_id': 'cup'},
                'id-12345',
                self.cancelled,
                lambda _: None,
            )

    async def test_timeout_submission_queries_original_id_without_resubmit(
        self
    ):
        self.backend.submission_error = True
        result = await self.adapter.execute(
            'reset_arm',
            {'target': 'zero'},
            'original-id',
            self.cancelled,
            lambda _: None,
        )
        self.assertTrue(result['verified'])
        self.assertEqual(
            sum(p == '/api/agent/tasks' for p, _ in self.backend.requests), 1
        )
        self.assertTrue(
            any(
                p == '/api/agent/tasks/original-id'
                for p, _ in self.backend.requests
            )
        )

    async def test_unconfirmed_submission_never_reissued(self):
        self.backend.submission_error = self.backend.query_error = True
        with self.assertRaises(Blocked) as caught:
            await self.adapter.execute(
                'reset_arm',
                {'target': 'zero'},
                'original-id',
                self.cancelled,
                lambda _: None,
            )
        self.assertTrue(caught.exception.uncertain)
        self.assertEqual(
            sum(p == '/api/agent/tasks' for p, _ in self.backend.requests), 1
        )

    async def test_held_object_rejects_repeated_grasp(self):
        self.backend.state['grasp']['held_object'] = {'object_id': 'b'}
        with self.assertRaises(Blocked):
            await self.adapter.execute(
                'grasp',
                {'object_id': 'b'},
                'id-12345',
                self.cancelled,
                lambda _: None,
            )
        self.assertFalse(any(v is not None for _, v in self.backend.requests))

    async def test_place_versions_pass_through_unchanged(self):
        args = {
            'hold_id': 'old',
            'orientation_version': 3,
            'placement_revision': 'old-version',
        }
        await self.adapter.execute(
            'place', args, 'id-12345', self.cancelled, lambda _: None
        )
        submitted = next(
            v for p, v in self.backend.requests if p == '/api/agent/tasks'
        )
        self.assertEqual(submitted['arguments'], args)

    async def test_restart_is_blocked(self):
        await self.adapter.status()
        self.backend.state['session_id'] = 'new'
        with self.assertRaises(Blocked):
            await self.adapter.status()

    async def test_rejected_backend_task_is_not_success(self):
        self.backend.result['state'] = 'rejected'
        with self.assertRaises(Blocked):
            await self.adapter.execute(
                'reset_arm',
                {'target': 'zero'},
                'id-12345',
                self.cancelled,
                lambda _: None,
            )

    async def test_preview_does_not_become_held_object(self):
        self.backend.result['state'] = 'preview'
        self.adapter.scene_ids.add('fixture-scene')
        result = await self.adapter.execute(
            'preview_grasp',
            {
                'object_id': 'b',
                'scene_id': 'fixture-scene',
                'instance_id': 'source',
            },
            'id-12345',
            self.cancelled,
            lambda _: None,
        )
        self.assertFalse(result['verified'])
        self.assertIsNone(result['status']['grasp']['held_object'])

    async def test_stop_is_repeated_after_inflight_submission_settles(self):
        started = threading.Event()
        release = threading.Event()
        sequence = []
        original = self.backend.request

        def delayed(path, value=None):
            if path == '/api/agent/tasks':
                sequence.append('submission-started')
                started.set()
                release.wait(2)
                sequence.append('submission-accepted')
                return {'terminal': False}
            if path == '/api/agent/stop':
                sequence.append('stop')
                release.set()
            return original(path, value)

        self.backend.request = delayed
        pending = asyncio.create_task(
            self.adapter.request('/api/agent/tasks', {})
        )
        await asyncio.to_thread(started.wait, 1)
        result = await self.adapter.stop()
        await pending
        self.assertTrue(result['stationary_observed'])
        self.assertEqual(
            sequence,
            ['submission-started', 'stop', 'submission-accepted', 'stop'],
        )

    async def test_busy_blocks_vision_and_motion_at_backend_gate(self):
        self.backend.state['grasp']['busy'] = True
        with self.assertRaises(Blocked):
            await self.adapter.execute(
                'reset_arm',
                {'target': 'zero'},
                'id-12345',
                self.cancelled,
                lambda _: None,
            )

    async def test_cancel_flag_prevents_dispatch(self):
        self.cancelled.set()
        from saydo.task_runner.robot_adapter import Cancelled
        with self.assertRaises(Cancelled):
            await self.adapter.execute(
                'reset_arm',
                {'target': 'zero'},
                'id-12345',
                self.cancelled,
                lambda _: None,
            )
        self.assertEqual(self.backend.requests, [])
