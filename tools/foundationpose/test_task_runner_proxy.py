import json
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
import threading
import unittest
from task_runner_proxy import TaskRunnerProxy


class ProxyTests(unittest.TestCase):
    def setUp(self):
        self.calls = []
        calls = self.calls
        class Handler(BaseHTTPRequestHandler):
            def log_message(self, *args):pass
            def do_GET(self):self.respond()
            def do_POST(self):self.respond()
            def respond(self):
                body = self.rfile.read(int(self.headers.get('Content-Length', 0)))
                calls.append((self.command, self.path, body, dict(self.headers)))
                status = 202 if self.command == 'POST' else 200
                self.send_response(status);self.send_header('Content-Type', 'application/json');self.end_headers()
                self.wfile.write(b'{"ok":true}')
        self.server = ThreadingHTTPServer(('127.0.0.1', 0), Handler)
        self.thread = threading.Thread(target=self.server.serve_forever);self.thread.start()
        self.proxy = TaskRunnerProxy(self.server.server_port)

    def tearDown(self):
        self.server.shutdown();self.server.server_close();self.thread.join()

    def test_page_assets_and_task_routes(self):
        for path,target in [('/task-runner/','/'),('/task-runner/app.js','/app.js'),('/api/task-runs','/api/task-runs')]:
            status,body,mime=self.proxy.request('GET',path)
            self.assertEqual(status,200);self.assertEqual(self.calls[-1][1],target)
        value={'instruction':'查看桌面'}
        status,body,mime=self.proxy.request('POST','/api/task-runs',value)
        self.assertEqual(status,202);self.assertEqual(json.loads(self.calls[-1][2]),value)
        self.assertEqual(self.calls[-1][3]['Origin'],f'http://127.0.0.1:{self.server.server_port}')
        self.assertNotIn('Authorization',self.calls[-1][3])

    def test_rejects_arbitrary_paths_and_methods_without_forwarding(self):
        for method,path in [('GET','/task-runner/../../.env'),('GET','/api/task-runs?url=http://evil'),('POST','/task-runner/'),('POST','/api/task-runs/not-a-run/cancel'),('GET','/api/agent/stop')]:
            with self.assertRaises(ValueError):self.proxy.request(method,path,{})
        self.assertEqual(self.calls,[])

    def test_cancel_and_unavailable_server(self):
        path='/api/task-runs/'+'a'*32+'/cancel'
        self.assertEqual(self.proxy.request('POST',path,{})[0],202)
        self.server.shutdown();self.server.server_close()
        self.assertEqual(self.proxy.request('GET','/task-runner/')[0],503)
