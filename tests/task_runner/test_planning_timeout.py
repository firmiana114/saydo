import asyncio
from types import SimpleNamespace
import unittest
from unittest.mock import AsyncMock
from saydo.task_runner.runner import Runner
from saydo.task_runner.robot_adapter import RobotAdapter, Cancelled
from saydo.task_runner.config import Config
from tests.task_runner.test_adapter import Backend


class PlanningTimeoutTests(unittest.IsolatedAsyncioTestCase):
    async def test_planning_wait_exceeds_action_timeout_and_can_cancel(self):
        config = Config(api_key='test', action_timeout=.005, poll_interval=.002)
        backend = Backend()
        backend.state['grasp']['held_object'] = {
            'object_id': 'tube', 'hold_id': 'h', 'orientation_version': 0
        }
        original = backend.request
        polls = []
        cancel = asyncio.Event()

        def request(path, value=None):
            if path.startswith('/api/agent/tasks/'):
                polls.append(1)
                if len(polls) < 8:
                    return {
                        'terminal': False, 'state': 'object_motion_planning'
                    }
                return {
                    'terminal': True,
                    'state': 'failed',
                    'result': {'error': 'finite search exhausted'},
                }
            return original(path, value)

        backend.request = request
        adapter = RobotAdapter(config, backend)
        args = {
            'object_id': 'tube',
            'hold_id': 'h',
            'orientation_version': 0,
            'goal': None,
            'retreat_vector_base_m': [0, 0, .05],
        }
        result = await adapter.execute(
            'plan_release', args, 'id', cancel, lambda _: None
        )
        self.assertEqual(result['error']['code'], 'PLAN_FAILED')
        self.assertGreaterEqual(len(polls), 8)
        cancel.set()
        with self.assertRaises(Cancelled):
            await adapter.execute(
                'plan_release', args, 'next', cancel, lambda _: None
            )

    async def test_only_planning_pauses_the_whole_task_deadline(self):
        async def slow(*args):
            await asyncio.sleep(.04)
            return {'ok': True}

        host = SimpleNamespace(
            adapter=SimpleNamespace(execute=AsyncMock(side_effect=slow)),
            cancelled=asyncio.Event(),
            progress=lambda _: None,
        )
        async with asyncio.timeout(.02) as timer:
            host.run_deadline = timer
            self.assertTrue(
                (
                    await Runner.execute_tool(host, 'plan_release', {}, 'id')
                )['ok']
            )
            self.assertGreater(
                timer.when() - asyncio.get_running_loop().time(), 0
            )
        with self.assertRaises(TimeoutError):
            async with asyncio.timeout(.01) as timer:
                host.run_deadline = timer
                await Runner.execute_tool(host, 'execute_plan', {}, 'id')
