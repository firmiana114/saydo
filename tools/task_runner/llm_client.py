"""Responses protocol only; never executes model text or follows redirects."""
import asyncio
import json
import httpx


class LLMError(RuntimeError):
    pass


class ModelUnavailable(LLMError):
    pass


class LLMClient:
    def __init__(self, config):
        self.config = config

    async def generate(self, history, tools, model, instructions, **extra):
        payload = {'model': model, 'instructions': instructions, 'input': history,
            'tools': tools, 'reasoning': {'effort': self.config.reasoning_effort},
            'tool_choice': 'required' if tools else 'none',
            'parallel_tool_calls': False, 'store': False, 'stream': False,
            'max_output_tokens': 4096,
            'include': ['reasoning.encrypted_content'], **extra}
        try:
            async with asyncio.timeout(self.config.llm_timeout):
                async with httpx.AsyncClient(timeout=self.config.llm_timeout, proxy=self.config.llm_proxy, follow_redirects=False, trust_env=False) as client:
                    async with client.stream('POST', self.config.responses_url, json=payload,
                            headers={'Authorization': 'Bearer '+self.config.api_key, 'User-Agent': 'SaydoTaskRunner/0.1'}) as response:
                        raw = bytearray()
                        async for part in response.aiter_bytes():
                            raw.extend(part)
                            if len(raw) > 4*1024*1024:
                                raise LLMError('模型响应超出大小限制')
        except (httpx.HTTPError, TimeoutError) as error:
            raise LLMError('模型网络请求失败或超时；未自动重试') from error
        if response.status_code != 200:
            try:
                error = json.loads(raw).get('error', {})
                code = error.get('code', '') if isinstance(error, dict) else ''
                message = str(error.get('message', '')) if isinstance(error, dict) else ''
            except ValueError:
                code, message = '', ''
            unavailable = code in ('model_not_found', 'model_not_available', 'unsupported_model', 'model_not_supported')
            unavailable |= response.status_code in (400, 404, 503) and 'model' in message.lower() and any(s in message.lower() for s in ('not found', 'not available', 'does not exist', 'not supported', 'no available channel'))
            if unavailable:
                raise ModelUnavailable('服务商明确报告模型不可用')
            raise LLMError(f'模型服务拒绝请求（HTTP {response.status_code}）；未回退或重试')
        try:
            result = json.loads(raw)
        except ValueError as error:
            raise LLMError('模型返回非 JSON 响应') from error
        if result.get('status') != 'completed' or not isinstance(result.get('output'), list) or not result['output']:
            raise LLMError('模型响应未完整结束，禁止执行其中的调用')
        return result
