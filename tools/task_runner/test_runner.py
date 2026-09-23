import asyncio
import copy
import json
from pathlib import Path
import tempfile
import unittest
from .config import Config
from .llm_client import ModelUnavailable, LLMError
from .runner import Runner
from .store import Store
from .robot_adapter import Blocked


def call(name, args=None, ident='call-1'):
    return {'status': 'completed', 'output': [{'type': 'function_call', 'name': name, 'call_id': ident, 'arguments': json.dumps(args or {})}]}


def end(outcome='answered', evidence=None):
    return call('finish_task', {'outcome': outcome, 'summary': '任务结果', 'evidence_call_ids': evidence or []}, 'finish')


class FakeLLM:
    def __init__(self, *responses):
        self.responses = list(responses)
        self.histories = []
        self.models = []
        self.started = asyncio.Event()

    async def generate(self, history, tools, model, instructions):
        self.histories.append(copy.deepcopy(history))
        self.models.append(model)
        self.started.set()
        result = self.responses.pop(0)
        if isinstance(result, Exception):
            raise result
        if result == 'wait':
            await asyncio.sleep(10)
        return result


class FakeAdapter:
    def __init__(self):
        self.motion_inflight = False
        self.sent = []
        self.stops = 0
        self.state = {'session_id': 's', 'grasp': {'busy': False, 'mode': 'execute', 'held_object': None}}
        self.fail = None

    async def initial(self):
        return {'status': self.state, 'objects': {'objects': []}, 'robot': {}}

    async def status(self):
        return self.state

    def check_idle(self, state):
        if state['grasp'].get('busy'):
            raise Blocked('busy')

    async def execute(self, name, args, request_id, cancelled, progress):
        self.sent.append((name, args, request_id))
        if self.fail:
            raise self.fail
        if name == 'grasp':
            self.state['grasp']['held_object'] = {'hold_id': 'h', 'object_id': args['object_id']}
            return {'verified': True, 'task': {'state': 'completed', 'result': {'held_object': copy.deepcopy(self.state['grasp']['held_object'])}}}
        if name == 'place':
            self.state['grasp']['held_object'] = None
            return {'verified': True, 'task': {'state': 'completed'}}
        if name == 'get_camera_frame':
            return {'image_base64': 'AA==', 'mime_type': 'image/png', 'captured_at': 1}
        return {'status': self.state}

    async def stop(self):
        self.stops += 1
        return {'stop_acknowledged': True, 'stationary_observed': True}


class RunnerTests(unittest.IsolatedAsyncioTestCase):
    async def asyncSetUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.config = Config(api_key='test-secret', mode='execute', state_dir=Path(self.temp.name),
            model='gpt-6-luna', fallback_model='gpt-5.6-luna')
        self.store = Store(self.temp.name, secrets=('test-secret',))
        self.adapter = FakeAdapter()

    async def asyncTearDown(self):
        self.temp.cleanup()

    def make(self, llm):
        return Runner(self.config, self.store, llm, lambda: self.adapter)

    async def execute(self, llm):
        runner = self.make(llm)
        ident = runner.submit('测试任务')
        await runner.worker
        return self.store.get(ident), runner

    async def test_grasp_place_loop_owns_ids_and_evidence(self):
        llm = FakeLLM(call('grasp', {'object_id': 'bottle'}, 'g'),
            call('place', {'hold_id': 'h', 'orientation_version': 0, 'placement_revision': 'v'}, 'p'), end('succeeded', ['g', 'p']))
        result, runner = await self.execute(llm)
        self.assertEqual(result['state'], 'succeeded')
        self.assertEqual(len(result['verified_actions']), 2)
        self.assertEqual(len(llm.histories), 3)
        for name, args, request_id in self.adapter.sent:
            self.assertTrue(request_id.startswith('run-'+result['run_id']))
            self.assertNotIn('request_id', args)
        self.assertTrue(all(c['state'] == 'completed' for c in result['calls']))

    async def test_fallback_only_on_initial_model_unavailable(self):
        llm = FakeLLM(ModelUnavailable('unavailable'), call('get_status'), end('answered', ['call-1']))
        result, _ = await self.execute(llm)
        self.assertEqual(result['model'], 'gpt-5.6-luna')
        self.assertEqual(llm.models, ['gpt-6-luna', 'gpt-5.6-luna', 'gpt-5.6-luna'])

    async def test_no_model_switch_after_task_has_started(self):
        llm = FakeLLM(call('get_status'), ModelUnavailable('unavailable'))
        result, _ = await self.execute(llm)
        self.assertEqual(result['state'], 'failed')
        self.assertEqual(llm.models, ['gpt-6-luna', 'gpt-6-luna'])

    async def test_auth_or_network_failure_does_not_fallback(self):
        llm = FakeLLM(LLMError('error'))
        result, _ = await self.execute(llm)
        self.assertEqual(result['state'], 'failed')
        self.assertEqual(llm.models, ['gpt-6-luna'])

    async def test_cancel_during_llm_discards_late_action(self):
        llm = FakeLLM('wait')
        runner = self.make(llm)
        ident = runner.submit('抓取')
        await llm.started.wait()
        await runner.cancel(ident)
        await runner.worker
        self.assertEqual(self.adapter.sent, [])
        self.assertEqual(self.adapter.stops, 1)
        self.assertEqual(self.store.get(ident)['state'], 'cancelled')

    async def test_failed_stop_blocks_new_run(self):
        async def failed():
            raise Blocked('stop unconfirmed', uncertain=True)
        self.adapter.stop = failed
        llm = FakeLLM('wait')
        runner = self.make(llm)
        ident = runner.submit('抓取')
        await llm.started.wait()
        await runner.cancel(ident)
        await runner.worker
        self.assertEqual(self.store.get(ident)['state'], 'blocked')
        with self.assertRaises(Blocked):
            runner.submit('再试一次')

    async def test_one_active_run(self):
        llm = FakeLLM('wait')
        runner = self.make(llm)
        ident = runner.submit('查询')
        with self.assertRaises(Blocked):
            runner.submit('其他任务')
        await llm.started.wait()
        await runner.cancel(ident)
        await runner.worker

    async def test_multiple_calls_rejected_before_any_dispatch(self):
        response = call('grasp', {'object_id': 'b'})
        response['output'] += call('reset_arm', {'target': 'zero'}, 'c2')['output']
        result, _ = await self.execute(FakeLLM(response))
        self.assertEqual(result['state'], 'blocked')
        self.assertEqual(self.adapter.sent, [])

    async def test_plain_text_cannot_claim_success(self):
        result, _ = await self.execute(FakeLLM({'output': [{'type': 'message', 'content': [{'type': 'output_text', 'text': '完成'}]}]}))
        self.assertEqual(result['state'], 'blocked')

    async def test_success_requires_real_motion_evidence(self):
        result, _ = await self.execute(FakeLLM(call('preview_grasp', {'object_id': 'b'}), end('succeeded', ['call-1'])))
        self.assertEqual(result['state'], 'blocked')

    async def test_fabricated_evidence_rejected(self):
        result, _ = await self.execute(FakeLLM(end('answered', ['invented'])))
        self.assertEqual(result['state'], 'blocked')

    async def test_grasp_failure_never_runs_place(self):
        self.adapter.fail = Blocked('failed grasp')
        llm = FakeLLM(call('grasp', {'object_id': 'b'}), call('place'))
        result, _ = await self.execute(llm)
        self.assertEqual(result['state'], 'blocked')
        self.assertEqual(len(llm.histories), 1)

    async def test_unknown_action_latches_across_restart(self):
        self.adapter.fail = Blocked('unknown', uncertain=True)
        result, _ = await self.execute(FakeLLM(call('grasp', {'object_id': 'b'})))
        self.assertTrue(result['outcome_unknown'])
        restarted = Runner(self.config, Store(self.temp.name), FakeLLM(), lambda: self.adapter)
        with self.assertRaises(Blocked):
            restarted.submit('继续')

    async def test_image_is_image_and_not_base64_text_or_journal(self):
        llm = FakeLLM(call('get_camera_frame'), end('answered', ['call-1']))
        result, _ = await self.execute(llm)
        history = llm.histories[1]
        images = [c for item in history if isinstance(item.get('content'), list) for c in item['content'] if c.get('type') == 'input_image']
        self.assertEqual(len(images), 1)
        self.assertNotIn('image_base64', json.dumps(result))

    async def test_preview_mode_rejects_motion_even_if_model_requests_it(self):
        self.config.mode = 'preview'
        result, _ = await self.execute(FakeLLM(call('grasp', {'object_id': 'b'})))
        self.assertNotEqual(result['state'], 'succeeded')
        self.assertEqual(self.adapter.sent, [])

    async def test_turn_limit(self):
        self.config.max_turns = 1
        result, _ = await self.execute(FakeLLM(call('get_status')))
        self.assertEqual(result['state'], 'blocked')

    async def test_request_is_durable_before_dispatch(self):
        original = self.adapter.execute
        async def checked(name, args, request_id, cancelled, progress):
            saved = self.store.recent()[0]
            calls = self.store.get(saved['run_id'])['calls']
            self.assertEqual(calls[-1]['request_id'], request_id)
            self.assertEqual(calls[-1]['state'], 'dispatching')
            return await original(name, args, request_id, cancelled, progress)
        self.adapter.execute = checked
        result, _ = await self.execute(FakeLLM(call('get_status'), end('answered', ['call-1'])))
        self.assertEqual(result['state'], 'succeeded')

    async def test_duplicate_successful_action_cannot_use_a_new_call_id(self):
        llm = FakeLLM(call('grasp', {'object_id': 'b'}, 'g1'), call('grasp', {'object_id': 'b'}, 'g2'))
        result, _ = await self.execute(llm)
        self.assertEqual(result['state'], 'blocked')
        self.assertEqual(len(self.adapter.sent), 1)

    async def test_llm_timeout_does_not_mark_motion_unknown_or_send_stop(self):
        self.config.run_timeout = .02
        result, runner = await self.execute(FakeLLM('wait'))
        self.assertFalse(runner.uncertain)
        self.assertEqual(self.adapter.stops, 0)
        self.assertEqual(result['state'], 'failed')

    async def test_disabled_fallback_never_switches_from_terra(self):
        self.config.model = 'gpt-5.6-terra'
        self.config.fallback_model = ''
        llm = FakeLLM(ModelUnavailable('unavailable'))
        result, _ = await self.execute(llm)
        self.assertEqual(llm.models, ['gpt-5.6-terra'])
        self.assertEqual(result['state'], 'failed')

    async def test_global_stop_cancels_active_model_decision(self):
        llm = FakeLLM('wait')
        runner = self.make(llm)
        ident = runner.submit('观察')
        await llm.started.wait()
        await runner.stop_current()
        await runner.worker
        self.assertEqual(self.store.get(ident)['state'], 'cancelled')
        self.assertEqual(self.adapter.stops, 1)

    async def test_global_stop_without_run_is_durable_and_no_model_call(self):
        llm = FakeLLM()
        runner = self.make(llm)
        result = await runner.stop_current()
        self.assertTrue(result['stationary_observed'])
        self.assertEqual(llm.histories, [])
        self.assertEqual(self.store.recent()[0]['state'], 'cancelled')

    async def test_unready_pose_is_returned_to_model_without_motion_retry(self):
        from tools.robot_agent.client import BackendError
        self.adapter.fail = BackendError({'code':'POSE_NOT_READY','message':'请等待跟踪'})
        llm = FakeLLM(call('get_object_pose'), end('answered',['call-1']))
        result, _ = await self.execute(llm)
        self.assertEqual(result['state'], 'succeeded')
        self.assertEqual(result['calls'][0]['state'], 'unavailable')
        self.assertIn('POSE_NOT_READY', json.dumps(llm.histories[1]))

    async def test_changed_hold_identity_cannot_claim_success(self):
        original = self.adapter.status
        async def changed():
            state = await original()
            state['grasp']['held_object']['hold_id'] = 'different-hold'
            return state
        self.adapter.status = changed
        result, _ = await self.execute(FakeLLM(call('grasp', {'object_id':'b'}, 'g'), end('succeeded',['g'])))
        self.assertEqual(result['state'], 'blocked')

    async def test_bad_final_evidence_gets_one_non_motion_correction(self):
        llm = FakeLLM(call('grasp', {'object_id':'b'}, 'g'), end('succeeded',['backend-task-id']), end('succeeded',['g']))
        result, _ = await self.execute(llm)
        self.assertEqual(result['state'], 'succeeded')
        self.assertEqual(len(self.adapter.sent), 1)
        self.assertIn('evidence_call_id', json.dumps(llm.histories[1]))
        self.assertIn('required_motion_evidence_call_ids', json.dumps(llm.histories[2]))

    async def test_final_correction_cannot_dispatch_another_motion(self):
        llm = FakeLLM(call('grasp', {'object_id':'b'}, 'g'), end('succeeded',['bad']), call('reset_arm', {'target':'zero'}, 'r'))
        result, _ = await self.execute(llm)
        self.assertEqual(result['state'], 'blocked')
        self.assertEqual(len(self.adapter.sent), 1)


class StoreTests(unittest.TestCase):
    def test_restart_never_replays_and_redacts_secret(self):
        with tempfile.TemporaryDirectory() as directory:
            store = Store(directory, secrets=('secret-value',))
            run = store.create('secret-value', 'gpt-6-luna', 'execute')
            store.prepare(run['run_id'], 'c', 'grasp', {'object_id': 'b'}, 1)
            restarted = Store(directory)
            saved = restarted.get(run['run_id'])
            self.assertEqual(saved['state'], 'interrupted')
            self.assertTrue(saved['outcome_unknown'])
            self.assertEqual(saved['instruction'], '[REDACTED]')
            self.assertEqual(saved['calls'][0]['state'], 'prepared')
    def test_read_only_restart_does_not_latch_motion_uncertainty(self):
        with tempfile.TemporaryDirectory() as directory:
            store = Store(directory)
            run = store.create('观察', 'gpt-6-luna', 'read_only')
            store.prepare(run['run_id'], 'c', 'get_status', {}, 1)
            saved = Store(directory).get(run['run_id'])
            self.assertEqual(saved['state'], 'interrupted')
            self.assertFalse(saved['outcome_unknown'])
