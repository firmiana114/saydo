import asyncio
import ssl
import tempfile
import unittest
from unittest.mock import patch
import httpx
from .config import Config
from .llm_client import LLMClient, LLMError, ConnectionNotSent, ResponseTimeout, ResponseFailure
from .llm_diagnostics import RequestDiagnostics
from .runner import Runner
from .store import Store
from .test_runner import FakeLLM, FakeAdapter, call, end


class RetryTests(unittest.IsolatedAsyncioTestCase):
    async def test_real_client_retries_only_connect_failure_and_records_attempts(
        self
    ):
        for fail_count in (2, 5):
            with self.subTest(
                fail_count=fail_count
            ), tempfile.TemporaryDirectory() as folder:
                requests = []
                base = httpx.AsyncClient

                async def handle(request):
                    requests.append(request.content)
                    await request.extensions['trace'](
                        'proxy.start_tls.started', {}
                    )
                    if len(requests) <= fail_count:
                        raise httpx.ConnectError('private detail')
                    return httpx.Response(200, json=end())

                c = Config(api_key='secret', mode='read_only')
                a = FakeAdapter()
                runner = Runner(c, Store(folder), adapter_factory=lambda: a)
                with patch(
                    'tools.task_runner.llm_client.httpx.AsyncClient',
                    side_effect=lambda **kw: base(
                        transport=httpx.MockTransport(handle), **kw
                    ),
                ), patch(
                    'tools.task_runner.runner.CONNECTION_RETRY_DELAYS',
                    (0, 0, 0, 0),
                ):
                    ident = runner.submit('read')
                    await runner.worker
                r = runner.store.get(ident)
                self.assertEqual(len(requests), 3 if fail_count == 2 else 5)
                self.assertEqual(
                    len(set(requests)),
                    1,
                    'retry must preserve model payload exactly',
                )
                self.assertEqual(
                    r['state'], 'succeeded' if fail_count == 2 else 'failed'
                )
                self.assertEqual(a.sent, [])
                starts = [e for e in r['events'] if e['event'] == 'llm_started']
                self.assertEqual(
                    [e['connection_attempt'] for e in starts],
                    list(range(1, len(requests) + 1)),
                )
                self.assertEqual(
                    len(
                        [
                            e
                            for e in r['events']
                            if e['event'] == 'llm_connection_retry'
                        ]
                    ),
                    len(requests) - 1,
                )
                self.assertEqual({e['turn'] for e in starts}, {1})
                if fail_count == 5:
                    self.assertIn('已尝试5次', r['summary'])

    async def test_completed_grasp_is_never_replayed(self):
        with tempfile.TemporaryDirectory() as folder:
            llm = FakeLLM(
                call('grasp', {'object_id': 'bottle'}, 'grasp-once'),
                ResponseTimeout('response timeout'),
                ResponseFailure('protocol failure'),
                end('succeeded', ['grasp-once']),
            )
            a = FakeAdapter()
            r = Runner(Config(api_key='x'), Store(folder), llm, lambda: a)
            with patch(
                'tools.task_runner.runner.CONNECTION_RETRY_DELAYS', (0, 0, 0, 0)
            ):
                ident = r.submit('grasp')
                await r.worker
            self.assertEqual(r.store.get(ident)['state'], 'succeeded')
            self.assertEqual([x[0] for x in a.sent], ['grasp'])
            self.assertEqual(llm.histories[1], llm.histories[2])
            self.assertEqual(llm.histories[2], llm.histories[3])

    async def test_cancel_and_run_deadline_interrupt_retry_wait(self):
        for cancel in (True,):
            with self.subTest(
                cancel=cancel
            ), tempfile.TemporaryDirectory() as folder:
                llm = FakeLLM(ConnectionNotSent('connect failed'), end())
                a = FakeAdapter()
                c = Config(
                    api_key='x',
                    mode='read_only',
                    run_timeout=1 if cancel else .03,
                )
                r = Runner(c, Store(folder), llm, lambda: a)
                with patch(
                    'tools.task_runner.runner.CONNECTION_RETRY_DELAYS',
                    (10, 10, 10, 10),
                ):
                    ident = r.submit('read')
                    if cancel:
                        for _ in range(100):
                            if any(
                                e['event'] == 'llm_connection_retry'
                                for e in r.active['events']
                            ):
                                break
                            await asyncio.sleep(.001)
                        r.cancelled.set()
                    await asyncio.wait_for(r.worker, 1)
                self.assertEqual(len(llm.histories), 1)
                self.assertEqual(a.sent, [])
                self.assertEqual(
                    r.store.get(ident)['state'],
                    'cancelled' if cancel else 'failed',
                )

    async def test_post_upload_and_http_errors_do_not_retry(self):
        for scenario in (
            'write', 'protocol_upload', 'read_upload', 'http', 'unknown'
        ):
            with self.subTest(
                scenario=scenario
            ), tempfile.TemporaryDirectory() as folder:
                sent = []
                base = httpx.AsyncClient

                async def handle(request):
                    sent.append(1)
                    trace = request.extensions['trace']
                    if scenario == 'http':
                        return httpx.Response(
                            503, json={'error': {'code': 'service_busy'}}
                        )
                    if scenario == 'unknown':
                        raise httpx.ConnectError('missing trace')
                    await trace('connection.start_tls.complete', {})
                    await trace('http11.send_request_headers.started', {})
                    if scenario == 'protocol_upload':
                        raise httpx.RemoteProtocolError('upload failed')
                    if scenario == 'read_upload':
                        raise httpx.ReadError('upload phase')
                    if scenario == 'write':
                        raise httpx.ConnectError(
                            'connection lost after upload began'
                        )
                    await trace('http11.receive_response_headers.started', {})
                    raise httpx.ReadTimeout('slow response')

                r = Runner(
                    Config(api_key='x', mode='read_only'),
                    Store(folder),
                    adapter_factory=FakeAdapter,
                )
                with patch(
                    'tools.task_runner.llm_client.httpx.AsyncClient',
                    side_effect=lambda **kw: base(
                        transport=httpx.MockTransport(handle), **kw
                    ),
                ):
                    ident = r.submit('read')
                    await r.worker
                self.assertEqual(len(sent), 1)
                self.assertEqual(r.store.get(ident)['state'], 'failed')

    async def test_certificate_error_and_sticky_upload_flag_are_not_retryable(
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
        await d.trace('proxy.start_tls.started', {})
        error = httpx.ConnectError('cert')
        error.__cause__ = ssl.SSLCertVerificationError('private details')
        self.assertFalse(LLMClient.can_retry_connection(error, d))
        self.assertTrue(
            LLMClient.can_retry_connection(httpx.ConnectTimeout('connect'), d)
        )
        await d.trace('proxy.start_tls.complete', {})
        await d.trace('http11.send_request_headers.started', {})
        d.data['phase'] = 'tls'
        self.assertFalse(
            LLMClient.can_retry_connection(httpx.ConnectError('late'), d)
        )


class ResponseRetryTests(unittest.IsolatedAsyncioTestCase):
    async def test_response_timeouts_retry_identical_payload_up_to_five(self):
        for error_type in (
            httpx.ReadTimeout, httpx.RemoteProtocolError, httpx.ReadError
        ):
            for phase in ('headers', 'body'):
                for failures in (2, 5):
                    with self.subTest(
                        phase=phase,
                        failures=failures,
                        error_type=error_type.__name__,
                    ), tempfile.TemporaryDirectory() as folder:
                        sent = []
                        base = httpx.AsyncClient

                        class Body(httpx.AsyncByteStream):
                            async def __aiter__(self):
                                yield b'{"output":['
                                raise error_type('partial response')

                        async def handle(request):
                            sent.append(request.content)
                            await request.extensions['trace'](
                                'http11.receive_response_headers.started', {}
                            )
                            if len(sent) <= failures:
                                if phase == 'headers':
                                    raise error_type('slow')
                                return httpx.Response(200, stream=Body())
                            return httpx.Response(200, json=end())

                        a = FakeAdapter()
                        r = Runner(
                            Config(api_key='x', mode='read_only'),
                            Store(folder),
                            adapter_factory=lambda: a,
                        )
                        with patch(
                            'tools.task_runner.llm_client.httpx.AsyncClient',
                            side_effect=lambda **kw: base(
                                transport=httpx.MockTransport(handle), **kw
                            ),
                        ), patch(
                            'tools.task_runner.runner.CONNECTION_RETRY_DELAYS',
                            (0, 0, 0, 0),
                        ):
                            ident = r.submit('read')
                            await r.worker
                        run = r.store.get(ident)
                        self.assertEqual(len(sent), 3 if failures == 2 else 5)
                        self.assertEqual(len(set(sent)), 1)
                        self.assertEqual(
                            run['state'],
                            'succeeded' if failures == 2 else 'failed',
                        )
                        self.assertEqual(a.sent, [])
                        self.assertEqual(
                            len(
                                [
                                    e
                                    for e in run['events']
                                    if e['event'] == 'llm_response_retry'
                                ]
                            ),
                            len(sent) - 1,
                        )

    async def test_model_wait_does_not_consume_backend_budget(self):
        class Slow(FakeLLM):
            async def generate(self, *args, **kwargs):
                await asyncio.sleep(.05)
                return await super().generate(*args, **kwargs)

        with tempfile.TemporaryDirectory() as folder:
            r = Runner(
                Config(api_key='x', mode='read_only', run_timeout=.03),
                Store(folder),
                Slow(ResponseTimeout('slow'), end()),
                FakeAdapter,
            )
            with patch(
                'tools.task_runner.runner.CONNECTION_RETRY_DELAYS',
                (.05, 0, 0, 0),
            ):
                ident = r.submit('read')
                await r.worker
            self.assertEqual(r.store.get(ident)['state'], 'succeeded')

    async def test_backend_budget_still_applies(self):
        class SlowAdapter(FakeAdapter):
            async def initial(self):
                await asyncio.sleep(.1)

        with tempfile.TemporaryDirectory() as folder:
            llm = FakeLLM(end())
            r = Runner(
                Config(api_key='x', run_timeout=.02),
                Store(folder),
                llm,
                SlowAdapter,
            )
            ident = r.submit('read')
            await r.worker
            self.assertEqual(r.store.get(ident)['state'], 'failed')
            self.assertEqual(llm.histories, [])
