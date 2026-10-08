"""Responses protocol only; never executes model text or follows redirects."""

import asyncio
import json
import ssl
import httpx
from saydo.task_runner.llm_diagnostics import RequestDiagnostics


class LLMError(RuntimeError):
    pass


class ConnectionNotSent(LLMError):
    """Transport trace confirms a failed connection before application HTTP upload."""
    pass


class ResponseFailure(LLMError):
    """Retryable transport failure while awaiting a complete model response."""
    pass


class ResponseTimeout(ResponseFailure):
    """No complete model response; partial output must never be dispatched."""
    pass


class ModelUnavailable(LLMError):
    pass


class LLMClient:
    def __init__(self, config):
        self.config = config

    async def generate(
        self,
        history,
        tools,
        model,
        instructions,
        *,
        reasoning_effort=None,
        diagnostics_callback=None,
        **extra,
    ):
        payload = {
            'model': model,
            'instructions': instructions,
            'input': history,
            'tools': tools,
            'reasoning': {
                'effort': (
                    reasoning_effort
                    if reasoning_effort is not None
                    else self.config.reasoning_effort
                )
            },
            'tool_choice': 'required' if tools else 'none',
            'parallel_tool_calls': False,
            'store': False,
            'stream': False,
            'max_output_tokens': 4096,
            'include': ['reasoning.encrypted_content'],
            **extra,
        }
        encoded = RequestDiagnostics.encode(payload)
        diagnostics = RequestDiagnostics(
            payload,
            encoded,
            self.config.llm_timeout,
            self.config.llm_proxy,
            diagnostics_callback,
        )
        failure = None
        try:
            try:
                async with asyncio.timeout(self.config.llm_timeout):
                    async with httpx.AsyncClient(
                        timeout=self.config.llm_timeout,
                        proxy=self.config.llm_proxy,
                        follow_redirects=False,
                        trust_env=False,
                    ) as client:
                        async with client.stream(
                            'POST',
                            self.config.responses_url,
                            content=encoded,
                            headers={
                                'Authorization': (
                                    'Bearer ' + self.config.api_key
                                ),
                                'User-Agent': 'SaydoTaskRunner/0.1',
                                'Content-Type': 'application/json',
                            },
                            extensions={'trace': diagnostics.trace},
                        ) as response:
                            diagnostics.headers(response)
                            raw = bytearray()
                            async for part in response.aiter_bytes():
                                diagnostics.chunk(part)
                                raw.extend(part)
                                if len(raw) > 4 * 1024 * 1024:
                                    raise LLMError('模型响应超出大小限制')
            except (httpx.HTTPError, TimeoutError) as error:
                failure = error
                if self.can_retry_connection(error, diagnostics):
                    diagnostics.data['retryable_connection_failure'] = True
                    raise ConnectionNotSent(
                        diagnostics.failure_message(
                            isinstance(
                                error, (TimeoutError, httpx.TimeoutException)
                            )
                        ).removesuffix('；未自动重试')
                    ) from error
                if (
                    isinstance(error, (TimeoutError, httpx.ReadTimeout))
                    and (
                        diagnostics.data['phase']
                        in ('waiting_headers', 'reading_body')
                    )
                    and diagnostics.data.get('http_status', 200) == 200
                ):
                    diagnostics.data['retryable_response_timeout'] = True
                    raise ResponseTimeout(
                        diagnostics.failure_message(True).removesuffix(
                            '；未自动重试'
                        )
                    ) from error
                if (
                    isinstance(
                        error, (httpx.RemoteProtocolError, httpx.ReadError)
                    )
                    and (
                        diagnostics.data['phase']
                        in ('waiting_headers', 'reading_body')
                    )
                    and diagnostics.data.get('http_status', 200) == 200
                ):
                    diagnostics.data['retryable_response_failure'] = True
                    raise ResponseFailure(
                        diagnostics.failure_message(False).removesuffix(
                            '；未自动重试'
                        )
                    ) from error
                raise LLMError(
                    diagnostics.failure_message(
                        isinstance(
                            error, (TimeoutError, httpx.TimeoutException)
                        )
                    )
                ) from error
            diagnostics.mark('parsing')
            return self.parse_response(response, raw, diagnostics)
        except BaseException as error:
            failure = failure or error
            raise
        finally:
            diagnostics.finish(
                (
                    'cancelled'
                    if isinstance(failure, asyncio.CancelledError)
                    else 'failed'
                    if failure
                    else 'completed'
                ),
                failure,
            )

    @staticmethod
    def can_retry_connection(error, diagnostics):
        if not isinstance(
            error, (httpx.ConnectError, httpx.ConnectTimeout, TimeoutError)
        ):
            return False
        if (
            diagnostics.application_request_started
            or diagnostics.data['phase'] not in ('connecting', 'tls')
        ):
            return False
        if (
            'http_status' in diagnostics.data
            or diagnostics.data['response_bytes']
        ):
            return False
        # Certificate validation/configuration errors do not improve with
        # repetition.
        cause = error
        seen = set()
        while cause is not None and id(cause) not in seen:
            seen.add(id(cause))
            if isinstance(cause, ssl.SSLCertVerificationError):
                return False
            cause = cause.__cause__
        return True

    @staticmethod
    def parse_response(response, raw, diagnostics):
        if response.status_code != 200:
            try:
                error = json.loads(raw).get('error', {})
                code = error.get('code', '') if isinstance(error, dict) else ''
                message = (
                    str(error.get('message', ''))
                    if isinstance(error, dict)
                    else ''
                )
            except (ValueError, AttributeError):
                code, message = '', ''
            unavailable = (
                code
                in (
                    'model_not_found',
                    'model_not_available',
                    'unsupported_model',
                    'model_not_supported',
                )
            )
            unavailable |= (
                response.status_code in (400, 404, 503)
                and 'model' in message.lower()
                and any(
                    s in message.lower()
                    for s in (
                        'not found',
                        'not available',
                        'does not exist',
                        'not supported',
                        'no available channel',
                    )
                )
            )
            if unavailable:
                raise ModelUnavailable('服务商明确报告模型不可用')
            raise LLMError(
                f'模型服务拒绝请求（HTTP {response.status_code}）；未回退或重试'
            )
        try:
            result = json.loads(raw)
        except ValueError as error:
            raise LLMError('模型返回非 JSON 响应') from error
        diagnostics.response(result)
        if (
            not isinstance(result, dict)
            or result.get('status') != 'completed'
            or not isinstance(result.get('output'), list)
            or not result['output']
        ):
            raise LLMError('模型响应未完整结束，禁止执行其中的调用')
        return result
