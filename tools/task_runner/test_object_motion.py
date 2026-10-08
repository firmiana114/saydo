from .test_runner import mock_global_camera
import asyncio
import copy
import tempfile
import unittest
from .config import Config
from .robot_adapter import RobotAdapter, Blocked
from .tool_registry import schemas, validate
from .test_adapter import Backend
from .test_runner import FakeLLM, call, end
from .runner import Runner
from .store import Store
from tools.robot_agent.motion_contract import DEFAULT_GOAL

POSE = [[1, 0, 0, .3], [0, 1, 0, 0], [0, 0, 1, .3], [0, 0, 0, 1]]
ARGS = {
    'object_id': 'b',
    'hold_id': 'h',
    'orientation_version': 0,
    'goal': {**copy.deepcopy(DEFAULT_GOAL), 'target_position_m': [.3, 0, .3]},
}


class ObjectBackend(Backend):
    def __init__(self):
        super().__init__()
        self.plans = {}
        self.bad = False
        self.fail_plan = False
        self.state['grasp']['held_object'] = {
            'object_id': 'b', 'hold_id': 'h', 'orientation_version': 0
        }

    def request(self, path, value=None):
        if path == '/api/agent/tasks':
            operation = value['operation']
            args = value['arguments']
            if operation in ('plan_held_object_motion', 'plan_release'):
                pid = (
                    'a' * 32
                    if operation == 'plan_held_object_motion'
                    else 'b' * 32
                )
                plan = {
                    'plan_id': pid,
                    'kind': (
                        'held_object_motion'
                        if operation == 'plan_held_object_motion'
                        else 'release'
                    ),
                    'hold': copy.deepcopy(self.state['grasp']['held_object']),
                    'pose_mode': '6d',
                    'reference_point_object_m': [0, 0, 0],
                    'base_T_object_target': POSE,
                    'position_tolerance_m': .002,
                    'orientation_tolerance_deg': 2,
                }
                self.plans[pid] = plan
                self.result = {
                    'state': 'object_motion_planned',
                    'terminal': True,
                    'result': {
                        'object_motion_plan': plan,
                        'held_object': copy.deepcopy(
                            self.state['grasp']['held_object']
                        ),
                    },
                }
            if operation == 'execute_plan':
                plan = self.plans[args['plan_id']]
                if plan['kind'] == 'release':
                    self.state['grasp']['held_object'] = None
                else:
                    self.state['grasp']['held_object'][
                        'orientation_version'
                    ] += 1
                pose = copy.deepcopy(POSE)
                if self.bad:
                    pose[0][3] += .05
                self.result = {
                    'state': 'completed',
                    'terminal': True,
                    'result': {
                        'object_motion_plan': plan,
                        'execution_result': {
                            'plan_id': args['plan_id'],
                            'kind': plan['kind'],
                            'target_verified': True,
                            'reference_position_error_m': 0,
                            'orientation_error_deg': 0,
                            'actual_object_pose_before_release': pose,
                            'release_confirmed': plan['kind'] == 'release',
                            'retreat_confirmed': plan['kind'] == 'release',
                        },
                        'held_object': copy.deepcopy(
                            self.state['grasp']['held_object']
                        ),
                    },
                }
            if (
                operation in ('plan_held_object_motion', 'plan_release')
                and self.fail_plan
            ):
                self.result = {
                    'state': 'failed',
                    'terminal': True,
                    'result': {'error': 'candidate unreachable'},
                }
        return super().request(path, value)


class ObjectAdapterTests(unittest.IsolatedAsyncioTestCase):
    async def asyncSetUp(self):
        mock_global_camera(self)

    def setUp(self):
        self.config = Config(api_key='test', mode='execute', poll_interval=.001)
        self.backend = ObjectBackend()
        self.adapter = RobotAdapter(self.config, self.backend)

    async def test_plan_then_execute_then_release_runner_evidence(self):
        release = {
            'object_id': 'b',
            'hold_id': 'h',
            'orientation_version': 1,
            'goal': None,
            'retreat_vector_base_m': [0, 0, .05],
        }
        llm = FakeLLM(
            call('plan_held_object_motion', ARGS, 'p1'),
            call('execute_plan', {'plan_id': 'a' * 32}, 'e1'),
            call('plan_release', release, 'p2'),
            call('execute_plan', {'plan_id': 'b' * 32}, 'e2'),
            end('succeeded', ['e1', 'e2']),
        )
        with tempfile.TemporaryDirectory() as d:
            runner = Runner(self.config, Store(d), llm, lambda: self.adapter)
            ident = runner.submit('持物移动到目标并松爪')
            await runner.worker
            result = runner.store.get(ident)
            self.assertEqual(
                result['state'], 'succeeded', result.get('summary')
            )
            self.assertEqual(
                [x['name'] for x in result['verified_actions']],
                ['execute_plan', 'execute_plan'],
            )
            self.assertIsNone(self.backend.state['grasp']['held_object'])

    async def test_real_pose_mismatch_rejects_even_if_backend_flags_success(
        self
    ):
        await self.adapter.execute(
            'plan_held_object_motion',
            ARGS,
            'plan-001',
            asyncio.Event(),
            lambda _: None,
        )
        self.backend.bad = True
        with self.assertRaises(Blocked):
            await self.adapter.execute(
                'execute_plan',
                {'plan_id': 'a' * 32},
                'exec-001',
                asyncio.Event(),
                lambda _: None,
            )

    async def test_preview_mode_never_submits_execution(self):
        self.config.mode = 'preview'
        planned = await self.adapter.execute(
            'plan_held_object_motion',
            ARGS,
            'plan-001',
            asyncio.Event(),
            lambda _: None,
        )
        self.assertFalse(planned['verified'])
        with self.assertRaises(Blocked):
            await self.adapter.execute(
                'execute_plan',
                {'plan_id': 'a' * 32},
                'exec-001',
                asyncio.Event(),
                lambda _: None,
            )
        self.assertFalse(
            any(
                v and v.get('operation') == 'execute_plan'
                for p, v in self.backend.requests
            )
        )

    async def test_readonly_planning_failure_returns_evidence_for_replanning(
        self
    ):
        self.backend.fail_plan = True
        result = await self.adapter.execute(
            'plan_held_object_motion',
            ARGS,
            'plan-fail',
            asyncio.Event(),
            lambda _: None,
        )
        self.assertEqual(result['error']['code'], 'PLAN_FAILED')
        self.assertFalse(result['motion_sent'])
        self.assertIn('unreachable', result['error']['message'])

    def test_restart_marks_unfinished_execution_unknown_and_reconcile_queries_original_id(
        self
    ):
        from .reconcile import reconcile
        with tempfile.TemporaryDirectory() as d:
            store = Store(d)
            run = store.create('移动后释放', 'terra', 'execute')
            record = store.prepare(
                run['run_id'], 'e', 'execute_plan', {'plan_id': 'a' * 32}, 1
            )
            store = Store(d)
            self.assertTrue(store.get(run['run_id'])['outcome_unknown'])
            self.backend.result = {
                'terminal': True,
                'state': 'interrupted',
                'outcome_unknown': True,
            }
            reconcile(store, self.backend, run['run_id'])
            self.assertIn(
                ('/api/agent/tasks/' + record['request_id'], None),
                self.backend.requests,
            )

    def test_contract_fields_and_mode_visibility(self):
        self.assertNotIn(
            'execute_plan', {x['name'] for x in schemas('preview')}
        )
        self.assertIn('plan_release', {x['name'] for x in schemas('preview')})
        self.assertNotIn(
            'plan_release', {x['name'] for x in schemas('read_only')}
        )
        for goal in [
            {**ARGS['goal'], 'target_position_m': [True, 0, 0]},
            {**ARGS['goal'], 'max_rotation_step_deg': 100},
            {**ARGS['goal'], 'unexpected': 1},
        ]:
            with self.assertRaises(ValueError):
                validate(
                    call(
                        'plan_held_object_motion', {**ARGS, 'goal': goal}
                    )['output'][0],
                    'execute',
                )


if __name__ == '__main__':
    unittest.main()
