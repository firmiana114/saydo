from tests.task_runner import test_runner
import asyncio
import copy
import tempfile
import unittest
from saydo.task_runner import config
from saydo.task_runner import robot_adapter
from saydo.task_runner import motion_retry
from saydo.task_runner import runner as task_runner_runner
from saydo.task_runner import store
from tests.task_runner import test_object_motion


class RecoveringBackend(test_object_motion.ObjectBackend):
    def __init__(self):
        super().__init__()
        self.failures = 1
        self.valid_recovery = True
        self.executions = 0

    def request(self, path, value=None):
        if path == '/api/agent/tasks' and value['operation'] == 'grasp':
            self.state['grasp']['held_object'] = {
                'object_id': 'b', 'hold_id': 'h', 'orientation_version': 0
            }
            self.result = {
                'state': 'completed',
                'terminal': True,
                'result': {
                    'held_object': copy.deepcopy(
                        self.state['grasp']['held_object']
                    )
                },
            }
        result = super().request(path, value)
        if (
            path == '/api/agent/tasks'
            and (
                value['operation']
                in ('plan_held_object_motion', 'plan_release')
            )
            and self.result.get('state') == 'object_motion_planned'
        ):
            self.result['result']['object_motion_plan']['goal'] = copy.deepcopy(
                value['arguments']['goal']
            )
        if path == '/api/agent/tasks' and value['operation'] == 'execute_plan':
            self.executions += 1
            if self.executions <= self.failures:
                h = self.state['grasp']['held_object']
                self.result = {
                    'state': 'failed',
                    'terminal': True,
                    'result': {
                        'error': 'trajectory failed',
                        'held_object': copy.deepcopy(h),
                        'recovery': {
                            'can_replan': self.valid_recovery,
                            'motion_sent': True,
                            'release_attempted': False,
                            'stationary_confirmed': True,
                            'grip_confirmed': True,
                            'controller_errors': [0],
                            'hold_id': h['hold_id'],
                            'object_id': h['object_id'],
                            'orientation_version': h['orientation_version'],
                            'goal_constraint': {
                                'base_T_object_target': test_object_motion.POSE,
                                'pose_mode': '6d',
                                'position_mode': 'fixed',
                                'position_required': True,
                                'reference_point_object_m': [0, 0, 0],
                            },
                        },
                    },
                }
        return result


class RecoveryTests(unittest.IsolatedAsyncioTestCase):
    async def asyncSetUp(self):
        test_runner.mock_global_camera(self)

    async def test_failed_execution_returns_to_model_and_next_plan_uses_new_version(
        self
    ):
        backend = RecoveringBackend()
        adapter = robot_adapter.RobotAdapter(
            config.Config(api_key='x', poll_interval=.001), backend
        )
        second = copy.deepcopy(test_object_motion.ARGS)
        second['orientation_version'] = 1
        second['goal']['max_rotation_step_deg'] = 5
        llm = test_runner.FakeLLM(
            test_runner.call(
                'plan_held_object_motion', test_object_motion.ARGS, 'p1'
            ),
            test_runner.call('execute_plan', {'plan_id': 'a' * 32}, 'e1'),
            test_runner.call('get_robot_state', {}, 's'),
            test_runner.call('plan_held_object_motion', second, 'p2'),
            test_runner.call('execute_plan', {'plan_id': 'a' * 32}, 'e2'),
            test_runner.end('succeeded', ['e2']),
        )
        with tempfile.TemporaryDirectory() as d:
            runner = task_runner_runner.Runner(
                adapter.config, store.Store(d), llm, lambda: adapter
            )
            ident = runner.submit('move')
            await runner.worker
            r = runner.store.get(ident)
            self.assertEqual(r['state'], 'succeeded', r.get('summary'))
            self.assertEqual(r['calls'][1]['state'], 'recoverable_failure')
            self.assertEqual(len(r['verified_actions']), 1)
            self.assertTrue(
                any(e['event'] == 'motion_recovery' for e in r['events'])
            )
            history = str(llm.histories[2])
            self.assertIn('MOTION_REPLAN_REQUIRED', history)
            self.assertEqual(backend.executions, 2)
            self.assertFalse(
                any(
                    v and v.get('operation') == 'grasp'
                    for _, v in backend.requests
                )
            )

    async def test_grasp_then_physical_recovery_never_repeats_grasp(self):
        backend = RecoveringBackend()
        backend.state['grasp']['held_object'] = None
        adapter = robot_adapter.RobotAdapter(
            config.Config(api_key='x', poll_interval=.001), backend
        )
        adapter.scene_ids.add('fixture-scene')
        second = copy.deepcopy(test_object_motion.ARGS)
        second['orientation_version'] = 1
        second['goal']['max_rotation_step_deg'] = 10
        llm = test_runner.FakeLLM(
            test_runner.call('grasp', {'object_id': 'b'}, 'g'),
            test_runner.call(
                'plan_held_object_motion', test_object_motion.ARGS, 'p1'
            ),
            test_runner.call('execute_plan', {'plan_id': 'a' * 32}, 'e1'),
            test_runner.call('plan_held_object_motion', second, 'p2'),
            test_runner.call('execute_plan', {'plan_id': 'a' * 32}, 'e2'),
            test_runner.end('succeeded', ['g', 'e2']),
        )
        with tempfile.TemporaryDirectory() as d:
            runner = task_runner_runner.Runner(
                adapter.config, store.Store(d), llm, lambda: adapter
            )
            ident = runner.submit('grasp and orient')
            await runner.worker
            r = runner.store.get(ident)
            self.assertEqual(r['state'], 'succeeded', r.get('summary'))
            self.assertEqual(
                [x['name'] for x in r['verified_actions']],
                ['grasp', 'execute_plan'],
            )
            self.assertEqual(
                sum(
                    v is not None and v.get('operation') == 'grasp'
                    for _, v in backend.requests
                ),
                1,
            )

    async def test_invalid_recovery_is_fatal(self):
        backend = RecoveringBackend()
        backend.valid_recovery = False
        adapter = robot_adapter.RobotAdapter(
            config.Config(api_key='x', poll_interval=.001), backend
        )
        await adapter.execute(
            'plan_held_object_motion',
            test_object_motion.ARGS,
            'p',
            asyncio.Event(),
            lambda _: None,
        )
        with self.assertRaises(robot_adapter.Blocked):
            await adapter.execute(
                'execute_plan',
                {'plan_id': 'a' * 32},
                'e',
                asyncio.Event(),
                lambda _: None,
            )

    async def test_five_plans_and_three_execution_attempts_are_enforced(self):
        backend = RecoveringBackend()
        backend.fail_plan = True
        adapter = robot_adapter.RobotAdapter(
            config.Config(api_key='x', poll_interval=.001), backend
        )
        for n in range(5):
            result = await adapter.execute(
                'plan_held_object_motion',
                test_object_motion.ARGS,
                'p' + str(n),
                asyncio.Event(),
                lambda _: None,
            )
            self.assertIn('error', result)
        with self.assertRaisesRegex(robot_adapter.Blocked, '规划5次'):
            await adapter.execute(
                'preview_reorient',
                {
                    'object_id': 'b',
                    'hold_id': 'h',
                    'orientation_version': 0,
                    'target_rpy_deg': [0, 0, 0],
                    'position_mode': 'fixed',
                },
                'p6',
                asyncio.Event(),
                lambda _: None,
            )
        backend = RecoveringBackend()
        backend.failures = 4
        adapter = robot_adapter.RobotAdapter(
            config.Config(api_key='x', poll_interval=.001), backend
        )
        for n in range(3):
            args = {**test_object_motion.ARGS, 'orientation_version': n}
            await adapter.execute(
                'plan_held_object_motion',
                args,
                'plan' + str(n),
                asyncio.Event(),
                lambda _: None,
            )
            result = await adapter.execute(
                'execute_plan',
                {'plan_id': 'a' * 32},
                'exec' + str(n),
                asyncio.Event(),
                lambda _: None,
            )
            self.assertTrue(result['recoverable_failure'])
        with self.assertRaisesRegex(robot_adapter.Blocked, '执行3次'):
            await adapter.execute(
                'plan_held_object_motion',
                {**test_object_motion.ARGS, 'orientation_version': 3},
                'plan4',
                asyncio.Event(),
                lambda _: None,
            )
        with self.assertRaisesRegex(robot_adapter.Blocked, '执行3次'):
            await adapter.execute(
                'execute_plan',
                {'plan_id': 'a' * 32},
                'exec4',
                asyncio.Event(),
                lambda _: None,
            )
        self.assertEqual(backend.executions, 3)

    def test_switching_tool_cannot_reset_budget_or_change_goal(self):
        retry = motion_retry.MotionRetry()
        held = {'hold_id': 'h'}
        for n in range(5):
            retry.before('preview_reorient', {}, held)
        with self.assertRaises(motion_retry.RetryLimit):
            retry.before('plan_held_object_motion', {}, held)
        retry.reset()
        retry.pending = {
            'position_required': True,
            'base_T_object_target': test_object_motion.POSE,
            'pose_mode': '6d',
            'position_mode': 'fixed',
            'reference_point_object_m': [0, 0, 0],
        }
        with self.assertRaises(motion_retry.RetryLimit):
            retry.before('plan_release', {}, held)
        with self.assertRaises(motion_retry.RetryLimit):
            retry.before('execute_plan', {'plan_id': 'old'}, held)
        with self.assertRaises(motion_retry.RetryLimit):
            retry.before(
                'reorient',
                {'target_rpy_deg': [0, 0, 0], 'position_mode': 'fixed'},
                held,
            )
        changed = copy.deepcopy(test_object_motion.POSE)
        changed[0][3] += .1
        with self.assertRaisesRegex(motion_retry.RetryLimit, '位置'):
            retry.plan(
                {
                    'base_T_object_target': changed,
                    'goal': {'position_mode': 'fixed'},
                    'kind': 'held_object_motion',
                    'plan_id': 'p',
                }
            )

    async def test_unresolved_failure_cannot_be_reported_as_success(self):
        backend = RecoveringBackend()
        adapter = robot_adapter.RobotAdapter(
            config.Config(api_key='x', poll_interval=.001), backend
        )
        llm = test_runner.FakeLLM(
            test_runner.call(
                'plan_held_object_motion', test_object_motion.ARGS, 'p'
            ),
            test_runner.call('execute_plan', {'plan_id': 'a' * 32}, 'e'),
            test_runner.end('succeeded', ['e']),
        )
        with tempfile.TemporaryDirectory() as d:
            r = task_runner_runner.Runner(
                adapter.config, store.Store(d), llm, lambda: adapter
            )
            ident = r.submit('move')
            await r.worker
            self.assertEqual(r.store.get(ident)['state'], 'blocked')
