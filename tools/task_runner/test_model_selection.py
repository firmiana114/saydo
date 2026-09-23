import asyncio
import json
from pathlib import Path
import tempfile
import unittest
from unittest.mock import patch
import httpx
from .config import Config, UI_MODELS, UI_REASONING_EFFORTS
from .llm_client import LLMClient, ModelUnavailable
from .runner import Runner
from .store import Store
from .test_runner import FakeLLM, FakeAdapter, call, end


class SelectionTests(unittest.IsolatedAsyncioTestCase):
    async def test_all_frontend_choices_are_recorded_without_mutating_defaults(self):
        with tempfile.TemporaryDirectory() as directory:
            config=Config(api_key='test', mode='read_only')
            llm=FakeLLM(*[end() for _ in range(len(UI_MODELS)*len(UI_REASONING_EFFORTS))])
            runner=Runner(config,Store(directory),llm,FakeAdapter)
            for model in UI_MODELS:
                for effort in UI_REASONING_EFFORTS:
                    ident=runner.submit('读取状态',model=model,reasoning_effort=effort)
                    await runner.worker
                    record=runner.store.get(ident)
                    self.assertEqual((record['model'],record['reasoning_effort']),(model,effort))
                    self.assertEqual((llm.models[-1],llm.efforts[-1]),(model,effort))
                    self.assertTrue(all(e.get('reasoning_effort')==effort for e in record['events'] if e['event'] in ('llm_started','llm')))
            self.assertEqual((config.model,config.reasoning_effort),('gpt-5.6-terra','medium'))

    async def test_selection_is_frozen_for_all_rounds(self):
        with tempfile.TemporaryDirectory() as directory:
            config=Config(api_key='test',mode='read_only');llm=FakeLLM(call('get_status'),end('answered',['call-1']))
            runner=Runner(config,Store(directory),llm,FakeAdapter)
            ident=runner.submit('读取',model='gpt-6-astra',reasoning_effort='high')
            config.model='gpt-5.5';config.reasoning_effort='low'
            await runner.worker
            self.assertEqual(llm.models,['gpt-6-astra','gpt-6-astra']);self.assertEqual(llm.efforts,['high','high'])
            self.assertEqual(runner.store.get(ident)['reasoning_effort'],'high')

    async def test_explicit_selection_disables_automatic_fallback(self):
        with tempfile.TemporaryDirectory() as directory:
            config=Config(api_key='test',fallback_model='gpt-5.6-luna')
            llm=FakeLLM(ModelUnavailable('no channel'));runner=Runner(config,Store(directory),llm,FakeAdapter)
            ident=runner.submit('查询',model='gpt-5.6-terra',reasoning_effort='low');await runner.worker
            self.assertEqual(llm.models,['gpt-5.6-terra']);self.assertEqual(runner.store.get(ident)['state'],'failed')

    async def test_invalid_values_never_create_run(self):
        with tempfile.TemporaryDirectory() as directory:
            runner=Runner(Config(api_key='test'),Store(directory),FakeLLM(),FakeAdapter)
            for options in ({'model':'gpt-6-luna'},{'model':'untrusted-model'},{'reasoning_effort':'ultra'},{'reasoning_effort':True}):
                with self.assertRaises(ValueError):runner.submit('test',**options)
            self.assertEqual(runner.store.recent(),[])

    async def test_actual_responses_payload_uses_selected_model_and_effort(self):
        sent=[]
        def handle(request):
            sent.append(json.loads(request.content))
            return httpx.Response(200,json={'status':'completed','output':[{'type':'message','content':[{'type':'output_text','text':'ok'}]}]})
        client_type=httpx.AsyncClient
        with patch('tools.task_runner.llm_client.httpx.AsyncClient',side_effect=lambda **kw:client_type(transport=httpx.MockTransport(handle),**kw)):
            await LLMClient(Config(api_key='secret')).generate([],[],'gpt-5.5','test',reasoning_effort='high')
        self.assertEqual(sent[0]['model'],'gpt-5.5');self.assertEqual(sent[0]['reasoning'],{'effort':'high'})
        self.assertNotIn('reasoning_effort',sent[0]);self.assertNotIn('secret',json.dumps(sent[0]))
