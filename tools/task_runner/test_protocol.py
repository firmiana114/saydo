import asyncio
import json
from pathlib import Path
import tempfile
import unittest
from unittest.mock import patch
import httpx
from .config import Config, read_env
from .llm_client import LLMClient, LLMError, ModelUnavailable
from .tool_registry import schemas, validate


class ProtocolTests(unittest.IsolatedAsyncioTestCase):
    async def response(self, status, data):
        transport = httpx.MockTransport(lambda request: httpx.Response(status, json=data))
        client_type = httpx.AsyncClient
        with patch('tools.task_runner.llm_client.httpx.AsyncClient', side_effect=lambda **kw: client_type(transport=transport, **kw)):
            return await LLMClient(Config(api_key='test')).generate([], [], 'gpt-6-luna', 'test')

    async def test_incomplete_output_never_executes(self):
        with self.assertRaises(LLMError):
            await self.response(200, {'status': 'incomplete', 'output': [{'type': 'function_call'}]})

    async def test_model_unavailable_is_distinct(self):
        with self.assertRaises(ModelUnavailable):
            await self.response(503, {'error': {'code': 'model_not_found'}})
        for status in (401, 429, 500):
            with self.assertRaises(LLMError) as error:
                await self.response(status, {'error': {'code': 'generic'}})
            self.assertNotIsInstance(error.exception, ModelUnavailable)

    async def test_redirect_not_followed(self):
        with self.assertRaises(LLMError):
            await self.response(302, {})


class RegistryTests(unittest.TestCase):
    def test_strict_schemas_and_no_request_ids(self):
        all_schemas = schemas('execute')
        self.assertEqual(len(all_schemas), 23)
        for schema in all_schemas:
            self.assertTrue(schema['strict'])
            self.assertFalse(schema['parameters']['additionalProperties'])
            self.assertEqual(set(schema['parameters']['required']), set(schema['parameters']['properties']))
            self.assertNotIn('request_id', schema['parameters']['properties'])

    def test_bad_types_extra_arguments_unknown_tools_and_duplicate_keys(self):
        for name, raw in [('grasp', '{"object_id":"b","force":99}'), ('exec', '{}'),
                          ('choose_tracking_target', '{"selection_id":true,"index":0}'),
                          ('grasp', '{"object_id":"b","object_id":"c"}'),
                          ('start_tracking', '{"object_id":"b","box":[0,0,640,640]}')]:
            with self.assertRaises(ValueError):
                validate({'type': 'function_call', 'name': name, 'call_id': 'c', 'arguments': raw}, 'execute')

    def test_read_only_has_no_writes(self):
        self.assertEqual({s['name'] for s in schemas('read_only')}, {'get_status','get_camera_frame','get_robot_state','list_objects','get_object_pose','get_grasp_profiles','get_object_geometry','get_task_diagnostics','finish_task'})

    def test_env_does_not_execute_or_expand_shell(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory)/'.env'
            path.write_text('SAYDO_TEST="$(touch should-not-exist)"\n');path.chmod(0o600)
            self.assertEqual(read_env(path)['SAYDO_TEST'], '$(touch should-not-exist)')
            path.chmod(0o644)
            with self.assertRaises(ValueError):
                read_env(path)

    def test_fresh_env_defaults_to_terra_without_hidden_fallback(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory)/'.env'
            path.write_text('SAYDO_LLM_API_KEY="test"\n');path.chmod(0o600)
            config = Config.load(path)
            self.assertEqual(config.model, 'gpt-5.6-terra')
            self.assertEqual(config.fallback_model, '')
            self.assertEqual(config.mode, 'execute')

    def test_provider_schema_subset_keeps_local_length_and_uniqueness_checks(self):
        definitions = json.dumps(schemas('execute'))
        self.assertNotIn('uniqueItems', definitions)
        self.assertNotIn('minLength', definitions)
        for args in [{'outcome':'answered','summary':'ok','evidence_call_ids':['same','same']},
                     {'outcome':'answered','summary':'','evidence_call_ids':[]}]:
            with self.assertRaises(ValueError):
                validate({'type':'function_call','name':'finish_task','call_id':'f','arguments':json.dumps(args)},'execute')
