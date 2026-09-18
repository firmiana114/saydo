"""Exercise the actual page routes with a local simulated robot, never hardware."""
import http.client
from http.server import ThreadingHTTPServer
import json
from pathlib import Path
import socketserver
import tempfile
import threading
from types import SimpleNamespace
import unittest
from unittest.mock import patch

import live_tracker
from grasp_control import GraspController


class GripperHTTPTests(unittest.TestCase):
    def test_page_force_request_reaches_robot_and_reports_failures(self):
        requests = []
        fail = threading.Event()

        class RobotHandler(socketserver.StreamRequestHandler):
            def handle(self):
                payload = json.loads(self.rfile.readline())
                requests.append(payload)
                command = payload['command']
                values = {
                    'get_rm_plus_mode': {'mode': 115200},
                    'get_rm_plus_base_info': {'base_info': {'manu': 'ZX', 'id': 1,
                        'type': 1, 'dof': 1, 'force': True, 'force_up': [100]}},
                    'get_rm_plus_state_info': {'state_info': {'sys_err': 0, 'sys_state': 0,
                        'dof_err': [0], 'dof_state': [2], 'speed': [0], 'pos': [1000]}},
                    'set_rm_plus_reg': {'set_state': not fail.is_set()},
                    'hand_follow_pos': {'set_state': not fail.is_set()},
                }
                self.wfile.write((json.dumps({'command': command, **values[command]}) + '\r\n').encode())

        with tempfile.TemporaryDirectory() as directory, socketserver.TCPServer(('127.0.0.1', 0), RobotHandler) as robot:
            root = Path(directory)
            (root / 'work').mkdir()
            config = json.loads((live_tracker.ROOT / 'config/grasp/trial.json').read_text())
            config.update(robot_ip='127.0.0.1', robot_port=robot.server_address[1])
            cfg = root / 'trial.json'
            cfg.write_text(json.dumps(config))
            robot_thread = threading.Thread(target=robot.serve_forever, daemon=True)
            robot_thread.start()
            server_ready = threading.Event()
            servers, trackers, errors = [], [], []

            class Tracker:
                def __init__(self, args):
                    self.stop = threading.Event()
                    self.events = SimpleNamespace(close=lambda: None)
                    self.grasp = GraspController(root, self, cfg)
                    trackers.append(self)

                def capture(self):
                    self.stop.wait()

                inference = capture

                def event(self, *args):
                    pass

                def snapshot(self):
                    return {'grasp': self.grasp.snapshot()}

            def make_server(address, handler):
                server = ThreadingHTTPServer(('127.0.0.1', 0), handler)
                servers.append(server)
                server_ready.set()
                return server

            def serve():
                try:
                    live_tracker.main()
                except Exception as error:
                    errors.append(error)
                    server_ready.set()

            with patch.object(live_tracker, 'ROOT', root), patch.object(live_tracker, 'Tracker', Tracker), \
                 patch.object(live_tracker, 'ThreadingHTTPServer', make_server), \
                 patch.object(live_tracker.signal, 'signal'), \
                 patch.object(live_tracker.urllib.request, 'urlopen', side_effect=OSError('preview disabled')), \
                 patch('sys.argv', ['live_tracker.py']):
                thread = threading.Thread(target=serve, daemon=True)
                thread.start()
                try:
                    self.assertTrue(server_ready.wait(5))
                    self.assertFalse(errors)
                    client = http.client.HTTPConnection(*servers[0].server_address, timeout=5)

                    def post(value):
                        client.request('POST', '/api/gripper/force-config', json.dumps({'value': value}),
                                       {'Content-Type': 'application/json'})
                        response = client.getresponse()
                        return response.status, json.loads(response.read())

                    client.request('GET', '/')
                    page = client.getresponse().read().decode()
                    self.assertIn('立即下发力度', page)
                    status, result = post(10)
                    self.assertEqual(status, 200)
                    self.assertTrue(result['write_acknowledged'])
                    self.assertEqual(requests[-1], {'command': 'set_rm_plus_reg', 'addr': 1220,
                                                   'length': 1, 'data': [10]})
                    fail.set()
                    status, result = post(50)
                    self.assertEqual(status, 400)
                    self.assertIn('未确认', result['error'])
                    self.assertFalse(trackers[0].grasp.force_config()['hardware_applied'])
                    self.assertEqual(trackers[0].grasp.force_config()['value'], 10)
                    count = len(requests)
                    trackers[0].grasp.mode = 'preview'
                    self.assertEqual(post(20)[0], 200)
                    self.assertEqual(len(requests), count)
                    self.assertFalse(any(x['command'] in ('hand_follow_pos', 'movel') for x in requests))
                    self.assertIn('id="gripper-open"', page)
                    self.assertIn("post('/api/gripper/open')", page)
                    def open_gripper():
                        client.request('POST', '/api/gripper/open', '{}', {'Content-Type': 'application/json'})
                        response = client.getresponse()
                        return response.status, json.loads(response.read())
                    self.assertEqual(open_gripper()[0], 400)  # Preview must not move.
                    self.assertEqual(len(requests), count)
                    trackers[0].grasp.mode = 'execute'
                    fail.clear()
                    status, result = open_gripper()
                    self.assertEqual(status, 200)
                    self.assertTrue(result['write_acknowledged'])
                    self.assertEqual(requests[-1], {'command': 'hand_follow_pos', 'hand_pos': [1000]})
                    fail.set()
                    self.assertEqual(open_gripper()[0], 400)
                    self.assertEqual(sum(x['command'] == 'hand_follow_pos' for x in requests), 2)
                    self.assertFalse(any(x['command'] == 'movel' for x in requests))
                    self.assertIn('记录当前夹法', page)
                    self.assertIn('预览抓取方案', page)
                    with patch.object(trackers[0].grasp, 'teach', return_value={'motion_sent':False}) as teach:
                        client.request('POST', '/api/grasp/teach', '{"use_current_point":true}', {'Content-Type':'application/json'})
                        response=client.getresponse()
                        self.assertEqual(response.status,200)
                        self.assertFalse(json.loads(response.read())['motion_sent'])
                        teach.assert_called_once_with(True)
                    with patch.object(trackers[0].grasp, 'start') as start:
                        client.request('POST', '/api/grasp/preview', '{}', {'Content-Type':'application/json'})
                        response=client.getresponse()
                        self.assertEqual(response.status,200)
                        self.assertEqual(json.loads(response.read())['mode'],'preview')
                        start.assert_called_once_with(preview=True)
                    client.close()
                finally:
                    for tracker in trackers:
                        tracker.stop.set()
                    for server in servers:
                        server.shutdown()
                    thread.join(5)
                    robot.shutdown()
                    robot_thread.join(5)
            self.assertFalse(thread.is_alive())
            self.assertFalse(errors)


if __name__ == '__main__':
    unittest.main()
