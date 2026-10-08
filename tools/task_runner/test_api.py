import asyncio
from http.server import ThreadingHTTPServer
import http.client
import json
from pathlib import Path
import tempfile
import threading
import time
import unittest
from .api import make_handler
from .config import Config
from .runner import Runner
from .store import Store
from .test_runner import FakeAdapter, FakeLLM, call, end


class APITests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.config = Config(api_key='private-test-key', mode='read_only', state_dir=Path(self.temp.name))
        self.store = Store(self.temp.name, secrets=(self.config.api_key,))
        self.loop = asyncio.new_event_loop()
        self.thread = threading.Thread(target=self.loop.run_forever)
        self.thread.start()
        self.runner = Runner(self.config, self.store, FakeLLM(call('get_status'), end('answered', ['call-1'])), FakeAdapter)
        self.server = ThreadingHTTPServer(('127.0.0.1', 0), make_handler(self.runner, self.store, self.loop, '127.0.0.1', 0))
        self.serving = threading.Thread(target=self.server.serve_forever)
        self.serving.start()

    def tearDown(self):
        self.server.shutdown();self.server.server_close();self.serving.join()
        self.loop.call_soon_threadsafe(self.loop.stop);self.thread.join();self.loop.close()
        self.temp.cleanup()

    def request(self, method, path, value=None, headers=None):
        connection = http.client.HTTPConnection(*self.server.server_address, timeout=3)
        connection.request(method, path, body=json.dumps(value) if value is not None else None,
            headers={'Content-Type': 'application/json', **(headers or {})})
        response = connection.getresponse()
        status, body = response.status, response.read()
        connection.close()
        return status, body

    def test_submit_poll_and_native_page(self):
        status, body = self.request('POST', '/api/task-runs', {'instruction': '<script>observe</script>'})
        self.assertEqual(status, 202)
        ident = json.loads(body)['run_id']
        for _ in range(100):
            _, body = self.request('GET', '/api/task-runs/'+ident)
            run = json.loads(body)
            if run['state'] != 'running' and run.get('events') and run['events'][-1]['event']=='final':
                break
            time.sleep(.01)
        self.assertEqual(run['state'], 'succeeded')
        self.assertEqual(run['events'][-1]['event'], 'final')
        self.assertEqual(len([e for e in run['events'] if e['event']=='llm']), 2)
        _, listing = self.request('GET', '/api/task-runs')
        self.assertNotIn('events', json.loads(listing)['runs'][0])
        status, html = self.request('GET', '/')
        self.assertEqual(status, 200)
        self.assertNotIn(b'private-test-key', html)
        self.assertIn(b'id="timeline"', html)
        _, script = self.request('GET', '/app.js')
        self.assertIn(b'root.SaydoTimeline=', script)
        self.assertEqual(self.request('GET', '/.env')[0], 404)

    def test_cross_site_and_rebinding_rejected(self):
        for headers in [{'Origin': 'https://evil.example'}, {'Host': 'evil.example'}, {'Content-Type': 'text/plain'}]:
            status, _ = self.request('POST', '/api/task-runs', {'instruction': 'move'}, headers)
            self.assertEqual(status, 403)
        self.assertEqual(self.store.recent(), [])

    def test_model_options_and_selected_run_metadata(self):
        from .config import UI_MODELS, UI_REASONING_EFFORTS
        status,body=self.request('GET','/api/task-runs');data=json.loads(body)
        self.assertEqual(data['llm_options']['models'],list(UI_MODELS))
        self.assertEqual(data['llm_options']['reasoning_efforts'],list(UI_REASONING_EFFORTS))
        self.assertNotIn('private-test-key',body.decode())
        status,body=self.request('POST','/api/task-runs',{'instruction':'查询','model':'gpt-6-sol','reasoning_effort':'high'})
        self.assertEqual(status,202);ident=json.loads(body)['run_id']
        for _ in range(100):
            run=json.loads(self.request('GET','/api/task-runs/'+ident)[1])
            if run['state']!='running':break
            time.sleep(.01)
        self.assertEqual((run['model'],run['reasoning_effort']),('gpt-6-sol','high'))
        self.assertEqual(self.runner.llm.efforts,['high','high'])

    def test_model_request_rejects_unknown_values_and_configuration_overrides(self):
        for options in ({'model':'gpt-6-terra'},{'reasoning_effort':'invalid'},{'model':None},
                        {'reasoning_effort':False},{'api_key':'override'},{'mode':'execute'}):
            status,_=self.request('POST','/api/task-runs',{'instruction':'查询',**options})
            self.assertIn(status,(400,404))
        self.assertEqual(self.store.recent(),[])
