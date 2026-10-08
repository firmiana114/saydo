import asyncio
import types
import unittest
from unittest import mock
from saydo.task_runner import runner
from saydo.task_runner import robot_adapter
from saydo.task_runner import config as task_runner_config
from tests.task_runner import test_adapter


class PlanningTimeoutTests(unittest.IsolatedAsyncioTestCase):
    async def test_planning_wait_exceeds_action_timeout_and_can_cancel(self):
        config = task_runner_config.Config(
            api_key='test', action_timeout=.005, poll_interval=.002
        )
        backend = test_adapter.Backend()
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
        adapter = robot_adapter.RobotAdapter(config, backend)
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
        with self.assertRaises(robot_adapter.Cancelled):
            await adapter.execute(
                'plan_release', args, 'next', cancel, lambda _: None
            )

    async def test_only_planning_pauses_the_whole_task_deadline(self):
        async def slow(*args):
            await asyncio.sleep(.04)
            return {'ok': True}

        host = types.SimpleNamespace(
            adapter=types.SimpleNamespace(
                execute=mock.AsyncMock(side_effect=slow)
            ),
            cancelled=asyncio.Event(),
            progress=lambda _: None,
        )
        async with asyncio.timeout(.02) as timer:
            host.run_deadline = timer
            self.assertTrue(
                (
                    await runner.Runner.execute_tool(
                        host, 'plan_release', {}, 'id'
                    )
                )['ok']
            )
            self.assertGreater(
                timer.when() - asyncio.get_running_loop().time(), 0
            )
        with self.assertRaises(TimeoutError):
            async with asyncio.timeout(.01) as timer:
                host.run_deadline = timer
                await runner.Runner.execute_tool(host, 'execute_plan', {}, 'id')
