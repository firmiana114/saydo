import asyncio
import copy
import json
from pathlib import Path
import tempfile
import unittest
from unittest.mock import patch
import httpx
from saydo.task_runner.config import Config
from saydo.task_runner.llm_client import LLMClient, LLMError
from saydo.task_runner.llm_diagnostics import RequestDiagnostics
from saydo.task_runner.robot_adapter import model_result
from saydo.task_runner.runner import Runner
from saydo.task_runner.store import Store
from tests.task_runner.test_runner import FakeAdapter

OK = {
    'status': 'completed',
    'output': [
        {
            'type': 'function_call',
            'name': 'finish_task',
            'call_id': 'done',
            'arguments': json.dumps(
                {
                    'outcome': 'answered',
                    'summary': 'ok',
                    'evidence_call_ids': [],
                }
            ),
        }
    ],
    'usage': {
        'input_tokens': 22,
        'output_tokens': 3,
        'total_tokens': 25,
        'input_tokens_details': {'cached_tokens': 7},
        'output_tokens_details': {'reasoning_tokens': 1},
        'attribution': {'private': 'never log'},
    },
}


class DiagnosticsTests(unittest.IsolatedAsyncioTestCase):
    async def invoke(self, handler, timeout=1):
        rows = []
        base = httpx.AsyncClient
        with patch(
            'saydo.task_runner.llm_client.httpx.AsyncClient',
            side_effect=lambda **kw: base(
                transport=httpx.MockTransport(handler), **kw
            ),
        ):
            try:
                value = await LLMClient(
                    Config(api_key='SECRET', llm_timeout=timeout)
                ).generate(
                    [],
                    [],
                    'gpt-6-sol',
                    'private prompt',
                    diagnostics_callback=rows.append,
                )
                return value, rows
            except BaseException as error:
                return error, rows

    async def test_success_metadata_and_no_content_leak(self):
        async def handle(request):
            self.assertNotIn(
                'diagnostics_callback', json.loads(request.content)
            )
            trace = request.extensions['trace']
            await trace(
                'connection.connect_tcp.started', {'host': 'secret-host'}
            )
            await trace('connection.connect_tcp.complete', {})
            await trace(
                'http11.send_request_body.complete', {'request': 'SECRET'}
            )
            await trace('http11.receive_response_headers.started', {})
            return httpx.Response(
                200, json=OK, headers={'x-request-id': 'request-123'}
            )

        result, rows = await self.invoke(handle)
        d = rows[-1]
        self.assertEqual(result, OK)
        self.assertEqual(d['outcome'], 'completed')
        self.assertEqual(d['provider_request_id'], 'request-123')
        self.assertEqual(d['usage']['cached_tokens'], 7)
        self.assertGreater(d['response_bytes'], 0)
        self.assertIn('first_byte_s', d)
        self.assertIn('request_sent_s', d)
        self.assertFalse(
            any(
                v in json.dumps(rows)
                for v in (
                    'SECRET',
                    'private prompt',
                    'secret-host',
                    'attribution',
                    'never log',
                )
            )
        )

    async def test_total_timeout_preserves_waiting_phase_and_original_type(
        self
    ):
        async def handle(request):
            await request.extensions['trace'](
                'http11.send_request_body.complete', {}
            )
            await request.extensions['trace'](
                'http11.receive_response_headers.started', {}
            )
            await asyncio.sleep(.2)

        result, rows = await self.invoke(handle, .02)
        self.assertIsInstance(result, LLMError)
        self.assertIn('等待响应头', str(result))
        self.assertEqual(rows[-1]['exception_type'], 'TimeoutError')
        self.assertEqual(rows[-1]['phase'], 'waiting_headers')
        self.assertNotIn('http_status', rows[-1])
        self.assertEqual(rows[-1]['outcome'], 'failed')

    async def test_timeout_during_body_records_partial_bytes(self):
        class SlowBody(httpx.AsyncByteStream):
            async def __aiter__(self):
                yield b'{'
                await asyncio.sleep(.2)

        result, rows = await self.invoke(
            lambda r: httpx.Response(200, stream=SlowBody()), .02
        )
        self.assertIsInstance(result, LLMError)
        self.assertIn('读取响应正文', str(result))
        self.assertEqual(rows[-1]['response_bytes'], 1)
        self.assertIn('first_byte_s', rows[-1])
        self.assertEqual(rows[-1]['http_status'], 200)

    async def test_transport_failure_and_cancel_are_distinct_without_exception_text(
        self
    ):
        for error in (
            httpx.ConnectError('SECRET url'),
            httpx.ReadTimeout('SECRET url'),
            asyncio.CancelledError(),
        ):
            async def handle(request):
                raise error

            result, rows = await self.invoke(handle)
            self.assertEqual(rows[-1]['exception_type'], type(error).__name__)
            self.assertEqual(
                rows[-1]['outcome'],
                (
                    'cancelled'
                    if isinstance(error, asyncio.CancelledError)
                    else 'failed'
                ),
            )
            self.assertNotIn('SECRET', json.dumps(rows) + str(result))

    async def test_http_parse_and_incomplete_errors_preserve_diagnostics(self):
        for status, body in [
            (429, b'{"error":{"message":"SECRET"}}'),
            (200, b'not-json SECRET'),
            (200, b'{"status":"incomplete","output":[]}'),
            (200, b'[]'),
        ]:
            result, rows = await self.invoke(
                lambda r: httpx.Response(
                    status,
                    content=body,
                    headers={'x-request-id': 'bad token with spaces'},
                )
            )
            self.assertIsInstance(result, LLMError)
            self.assertEqual(rows[-1]['http_status'], status)
            self.assertEqual(rows[-1]['outcome'], 'failed')
            self.assertNotIn('provider_request_id', rows[-1])
            self.assertNotIn('SECRET', json.dumps(rows) + str(result))

    async def test_trace_is_bounded_and_does_not_confuse_proxy_tunnel_with_upload(
        self
    ):
        payload = {
            'model': 'm',
            'reasoning': {'effort': 'low'},
            'stream': False,
            'input': [],
            'tools': [],
        }
        d = RequestDiagnostics(payload, b'{}', 90, True)
        await d.trace(
            'http11.send_request_body.complete', {'authorization': 'SECRET'}
        )
        self.assertNotIn('request_sent_s', d.data)
        await d.trace('proxy.start_tls.complete', {})
        for _ in range(80):
            await d.trace('http11.send_request_body.complete', {})
        self.assertEqual(len(d.data['transport_events']), 48)
        self.assertGreater(d.data['transport_events_dropped'], 0)
        self.assertIn('request_sent_s', d.data)

    async def test_runner_persists_failed_invocation_without_tool_dispatch(
        self
    ):
        with tempfile.TemporaryDirectory() as directory:
            c = Config(api_key='SECRET', mode='read_only', llm_timeout=.02)
            store = Store(directory)
            adapter = FakeAdapter()
            runner = Runner(c, store, adapter_factory=lambda: adapter)
            base = httpx.AsyncClient

            async def handle(request):
                await request.extensions['trace'](
                    'http11.receive_response_headers.started', {}
                )
                await asyncio.sleep(.2)

            with patch(
                'saydo.task_runner.llm_client.httpx.AsyncClient',
                side_effect=lambda **kw: base(
                    transport=httpx.MockTransport(handle), **kw
                ),
            ):
                ident = runner.submit('read')
                await runner.worker
            run = store.get(ident)
            event = next(e for e in run['events'] if e['event'] == 'llm')
            self.assertEqual(run['state'], 'failed')
            self.assertEqual(
                event['diagnostics']['exception_type'], 'TimeoutError'
            )
            self.assertEqual(adapter.sent, [])
            self.assertEqual(run['calls'], [])


class ContextTests(unittest.TestCase):
    def fixture(self):
        held = {
            'hold_id': 'h',
            'orientation_version': 3,
            'object_T_tcp': [[1, 0], [0, 1]],
        }
        plan = {
            'grasp_id': 'g',
            'effective_parameters': {'tcp_offset_m': [0, 0, .03]},
            'waypoints': [{'many': 'numbers'}] * 20,
        }
        return {
            'task': {
                'task_id': 't',
                'state': 'completed',
                'result': {
                    'plan': copy.deepcopy(plan),
                    'held_object': copy.deepcopy(held),
                },
            },
            'status': {
                'grasp': {
                    'state': 'completed',
                    'plan': copy.deepcopy(plan),
                    'held_object': copy.deepcopy(held),
                    'placement_config': {'revision': 'v'},
                    'error': None,
                }
            },
            'verified': True,
            'verification_scope': 'controller only',
            'evidence_call_id': 'c',
        }

    def test_successful_grasp_reduces_model_history_but_preserves_journal_and_parameters(
        self
    ):
        result = self.fixture()
        original = copy.deepcopy(result)
        history = []
        saved = Runner.append_result(history, 'c', 'grasp', result)
        small = json.loads(history[0]['output'])
        self.assertEqual(result, original)
        self.assertEqual(saved, original)
        self.assertNotIn('plan', small['status']['grasp'])
        self.assertNotIn('held_object', small['status']['grasp'])
        self.assertEqual(
            small['task']['result']['held_object'],
            original['task']['result']['held_object'],
        )
        self.assertEqual(
            small['task']['result']['plan']['effective_parameters'],
            original['task']['result']['plan']['effective_parameters'],
        )
        self.assertEqual(
            small['status']['grasp']['placement_config']['revision'], 'v'
        )
        self.assertLess(len(json.dumps(small)), len(json.dumps(original)))

    def test_plans_failures_and_diagnostics_preserve_decision_evidence(self):
        result = self.fixture()
        result['task']['state'] = 'failed'
        result['verified'] = False
        self.assertEqual(model_result('grasp', result)['task'], result['task'])
        self.assertEqual(model_result('get_task_diagnostics', result), result)
        for state in ('preview', 'object_motion_planned', 'failed'):
            status = {
                'grasp': {
                    'state': state, 'object_motion_plan': {'plan_id': 'pending'}
                }
            }
            self.assertEqual(model_result('get_status', status), status)
        result['task']['result']['object_motion_plan'] = {
            'plan_id': 'p',
            'goal': {'target_position_m': [.3, 0, .3]},
            'validation_scope': {'environment_collision_checked': False},
        }
        self.assertEqual(
            model_result('plan_held_object_motion', result)['task'],
            result['task'],
        )

    def test_mismatched_hold_is_not_deduplicated_and_initial_status_keeps_error(
        self
    ):
        result = self.fixture()
        result['status']['grasp']['held_object']['hold_id'] = 'changed'
        self.assertEqual(
            model_result(
                'grasp', result
            )['status']['grasp']['held_object']['hold_id'],
            'changed',
        )
        result['status']['grasp']['error'] = 'blocked'
        initial = model_result(
            'initial',
            {
                'status': result['status'],
                'robot': {'controller': result['status']['grasp']},
            },
        )
        self.assertNotIn('plan', initial['status']['grasp'])
        self.assertEqual(initial['status']['grasp']['error'], 'blocked')
        self.assertEqual(
            initial['robot']['controller']['held_object']['hold_id'], 'changed'
        )
