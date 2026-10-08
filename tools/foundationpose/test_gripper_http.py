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
                    'get_rm_plus_base_info': {
                        'base_info': {
                            'manu': 'ZX',
                            'id': 1,
                            'type': 1,
                            'dof': 1,
                            'force': True,
                            'force_up': [100],
                        }
                    },
                    'get_rm_plus_state_info': {
                        'state_info': {
                            'sys_err': 0,
                            'sys_state': 0,
                            'dof_err': [0],
                            'dof_state': [2],
                            'speed': [0],
                            'pos': [1000],
                        }
                    },
                    'set_rm_plus_reg': {'set_state': not fail.is_set()},
                    'hand_follow_pos': {'set_state': not fail.is_set()},
                }
                self.wfile.write(
                    (
                        json.dumps({'command': command, **values[command]})
                        + '\r\n'
                    ).encode()
                )

        with tempfile.TemporaryDirectory() as directory, socketserver.TCPServer(
            ('127.0.0.1', 0), RobotHandler
        ) as robot:
            root = Path(directory)
            (root / 'work').mkdir()
            config = json.loads(
                (live_tracker.ROOT / 'config/grasp/trial.json').read_text()
            )
            config.update(
                robot_ip='127.0.0.1', robot_port=robot.server_address[1]
            )
            cfg = root / 'trial.json'
            cfg.write_text(json.dumps(config))
            robot_thread = threading.Thread(
                target=robot.serve_forever, daemon=True
            )
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

            with patch.object(live_tracker, 'ROOT', root), patch.object(
                live_tracker, 'Tracker', Tracker
            ), patch.object(
                live_tracker, 'ThreadingHTTPServer', make_server
            ), patch.object(live_tracker.signal, 'signal'), patch.object(
                live_tracker.urllib.request,
                'urlopen',
                side_effect=OSError('preview disabled'),
            ), patch('sys.argv', ['live_tracker.py']):
                thread = threading.Thread(target=serve, daemon=True)
                thread.start()
                try:
                    self.assertTrue(server_ready.wait(5))
                    self.assertFalse(errors)
                    client = http.client.HTTPConnection(
                        *servers[0].server_address, timeout=5
                    )

                    def post(value):
                        client.request(
                            'POST',
                            '/api/gripper/force-config',
                            json.dumps({'value': value}),
                            {'Content-Type': 'application/json'},
                        )
                        response = client.getresponse()
                        return response.status, json.loads(response.read())

                    client.request('GET', '/')
                    page = client.getresponse().read().decode()
                    self.assertNotIn('id="gripper-force"', page)
                    self.assertIn('id="teach-force"', page)
                    self.assertIn('id="arm-speed"', page)
                    speed = trackers[0].grasp.motion_speed_configuration()
                    client.request(
                        'POST',
                        '/api/arm/speed',
                        json.dumps(
                            {'value': 20, 'revision': speed['revision']}
                        ),
                        {'Content-Type': 'application/json'},
                    )
                    response = client.getresponse()
                    result = json.loads(response.read())
                    self.assertEqual(response.status, 200)
                    self.assertEqual(result['value'], 20)
                    self.assertFalse(result['motion_sent'])
                    self.assertEqual(
                        requests,
                        [],
                        'saving speed must not connect to hardware',
                    )
                    client.request(
                        'POST',
                        '/api/arm/speed',
                        json.dumps(
                            {'value': 30, 'revision': speed['revision']}
                        ),
                        {'Content-Type': 'application/json'},
                    )
                    response = client.getresponse()
                    response.read()
                    self.assertEqual(response.status, 400)

                    status, result = post(10)
                    self.assertEqual(status, 200)
                    self.assertTrue(result['write_acknowledged'])
                    self.assertEqual(
                        requests[-1],
                        {
                            'command': 'set_rm_plus_reg',
                            'addr': 1220,
                            'length': 1,
                            'data': [10],
                        },
                    )
                    fail.set()
                    status, result = post(50)
                    self.assertEqual(status, 400)
                    self.assertIn('未确认', result['error'])
                    self.assertFalse(
                        trackers[0].grasp.force_config()['hardware_applied']
                    )
                    self.assertEqual(
                        trackers[0].grasp.force_config()['value'], 10
                    )
                    count = len(requests)
                    trackers[0].grasp.mode = 'preview'
                    self.assertEqual(post(20)[0], 200)
                    self.assertEqual(len(requests), count)
                    self.assertFalse(
                        any(
                            x['command'] in ('hand_follow_pos', 'movel')
                            for x in requests
                        )
                    )
                    self.assertIn('id="gripper-open"', page)
                    self.assertIn("post('/api/gripper/open')", page)

                    def open_gripper():
                        client.request(
                            'POST',
                            '/api/gripper/open',
                            '{}',
                            {'Content-Type': 'application/json'},
                        )
                        response = client.getresponse()
                        return response.status, json.loads(response.read())

                    # Preview must not move.
                    self.assertEqual(open_gripper()[0], 400)
                    self.assertEqual(len(requests), count)
                    trackers[0].grasp.mode = 'execute'
                    fail.clear()
                    status, result = open_gripper()
                    self.assertEqual(status, 200)
                    self.assertTrue(result['write_acknowledged'])
                    self.assertEqual(
                        requests[-1],
                        {'command': 'hand_follow_pos', 'hand_pos': [1000]},
                    )
                    fail.set()
                    self.assertEqual(open_gripper()[0], 400)
                    self.assertEqual(
                        sum(
                            x['command'] == 'hand_follow_pos' for x in requests
                        ),
                        2,
                    )
                    self.assertFalse(
                        any(x['command'] == 'movel' for x in requests)
                    )
                    self.assertIn('记录当前夹法', page)
                    self.assertIn('预览抓取方案', page)
                    with patch.object(
                        trackers[0].grasp,
                        'teach',
                        return_value={'motion_sent': False},
                    ) as teach:
                        client.request(
                            'POST',
                            '/api/grasp/teach',
                            '{"use_current_point":true}',
                            {'Content-Type': 'application/json'},
                        )
                        response = client.getresponse()
                        self.assertEqual(response.status, 200)
                        self.assertFalse(
                            json.loads(response.read())['motion_sent']
                        )
                        teach.assert_called_once_with(
                            True,
                            gripper_parameters=None,
                            expected_object_id=None,
                        )
                    with patch.object(trackers[0].grasp, 'start') as start:
                        client.request(
                            'POST',
                            '/api/grasp/preview',
                            '{}',
                            {'Content-Type': 'application/json'},
                        )
                        response = client.getresponse()
                        self.assertEqual(response.status, 200)
                        self.assertEqual(
                            json.loads(response.read())['mode'], 'preview'
                        )
                        start.assert_called_once_with(preview=True)
                    self.assertNotIn('id="gripper-open-position"', page)
                    self.assertIn('id="teach-open-position"', page)
                    self.assertIn('id="arm-reset"', page)
                    before = len(requests)
                    client.request(
                        'POST',
                        '/api/gripper/open-position',
                        '{"value":300}',
                        {'Content-Type': 'application/json'},
                    )
                    response = client.getresponse()
                    self.assertEqual(response.status, 200)
                    self.assertTrue(json.loads(response.read())['saved'])
                    self.assertEqual(
                        json.loads(cfg.read_text())['gripper']['open_position'],
                        300,
                    )
                    self.assertEqual(len(requests), before)
                    with patch.object(
                        trackers[0].grasp,
                        'start_reset',
                        return_value={'accepted': True},
                    ) as reset:
                        client.request(
                            'POST',
                            '/api/arm/reset',
                            '{}',
                            {'Content-Type': 'application/json'},
                        )
                        response = client.getresponse()
                        self.assertEqual(response.status, 200)
                        self.assertTrue(json.loads(response.read())['accepted'])
                        reset.assert_called_once_with()
                    self.assertIn('机械臂复位(观测桌面)', page)
                    self.assertIn('机械臂复位(零位)', page)
                    self.assertIn('id="arm-reset-zero"', page)
                    with patch.object(
                        trackers[0].grasp,
                        'start_reset',
                        return_value={'accepted': True},
                    ) as reset:
                        client.request(
                            'POST',
                            '/api/arm/reset-zero',
                            '{}',
                            {'Content-Type': 'application/json'},
                        )
                        response = client.getresponse()
                        self.assertEqual(response.status, 200)
                        response.read()
                        reset.assert_called_once_with('zero')
                    trackers[0].grasp.mode = 'preview'
                    before = len(requests)
                    client.request(
                        'POST',
                        '/api/arm/reset-zero',
                        '{}',
                        {'Content-Type': 'application/json'},
                    )
                    response = client.getresponse()
                    self.assertEqual(response.status, 400)
                    response.read()
                    self.assertEqual(len(requests), before)
                    trackers[0].grasp.mode = 'execute'
                    self.assertIn('id="reorientation-view"', page)
                    client.request('GET', '/reorientation_viewer.js')
                    response = client.getresponse()
                    self.assertEqual(response.status, 200)
                    self.assertIn(
                        'class ReorientationViewer', response.read().decode()
                    )
                    with patch.object(
                        trackers[0].grasp,
                        'reorientation_model',
                        return_value={'object_id': 'sample'},
                    ) as model:
                        client.request(
                            'GET', '/api/reorient/model?object_id=sample'
                        )
                        response = client.getresponse()
                        self.assertEqual(response.status, 200)
                        self.assertEqual(
                            json.loads(response.read())['object_id'], 'sample'
                        )
                        model.assert_called_once_with('sample')
                    with patch.object(
                        trackers[0].grasp,
                        'start_reorient',
                        return_value={'accepted': True},
                    ) as place:
                        for path in ['/api/reorient/preview', '/api/reorient']:
                            client.request(
                                'POST',
                                path,
                                '{"rotation":[]}',
                                {'Content-Type': 'application/json'},
                            )
                            response = client.getresponse()
                            self.assertEqual(response.status, 200)
                            self.assertTrue(
                                json.loads(response.read())['accepted']
                            )
                        self.assertEqual(
                            place.call_args_list[0].kwargs, {'preview': True}
                        )
                        self.assertEqual(
                            place.call_args_list[1].args, ({'rotation': []},)
                        )
                    client.request('GET', '/grasp_editor.js')
                    response = client.getresponse()
                    self.assertEqual(response.status, 200)
                    self.assertIn(
                        'class GraspEditorPanel', response.read().decode()
                    )
                    self.assertIn('物品夹法库', page)
                    for route, method in [
                        (
                            '/api/grasp/profiles?object_id=sample',
                            'grasp_profiles',
                        ),
                        (
                            '/api/grasp/editor-model?object_id=sample',
                            'grasp_editor_model',
                        ),
                    ]:
                        with patch.object(
                            trackers[0].grasp,
                            method,
                            return_value={'object_id': 'sample'},
                        ) as handler:
                            client.request('GET', route)
                            response = client.getresponse()
                            self.assertEqual(response.status, 200)
                            self.assertEqual(
                                json.loads(response.read())['object_id'],
                                'sample',
                            )
                            handler.assert_called_once_with('sample')
                    with patch.object(
                        trackers[0].grasp,
                        'edit_grasp_profiles',
                        return_value={'motion_sent': False},
                    ) as edit:
                        client.request(
                            'POST',
                            '/api/grasp/profiles',
                            '{"action":"select"}',
                            {'Content-Type': 'application/json'},
                        )
                        response = client.getresponse()
                        self.assertEqual(response.status, 200)
                        self.assertFalse(
                            json.loads(response.read())['motion_sent']
                        )
                        edit.assert_called_once_with({'action': 'select'})
                    client.request('GET', '/placement_panel.js')
                    response = client.getresponse()
                    self.assertEqual(response.status, 200)
                    self.assertIn(
                        'class PlacementPanel', response.read().decode()
                    )
                    self.assertIn('将物品放置在桌面', page)
                    for path in ['/api/place', '/api/place/preview']:
                        client.request(
                            'POST',
                            path,
                            '{}',
                            {'Content-Type': 'application/json'},
                        )
                        response = client.getresponse()
                        self.assertEqual(response.status, 400)
                        self.assertIn(
                            '夹持记录', json.loads(response.read())['error']
                        )
                    count = len(requests)
                    config_status = trackers[0].grasp.placement_configuration()
                    value = {
                        'revision': config_status['revision'],
                        'region_xy_m': {
                            'x_min': .2, 'x_max': .5, 'y_min': -.2, 'y_max': .2
                        },
                        'release_gap_m': .003,
                    }
                    client.request(
                        'POST',
                        '/api/place/config',
                        json.dumps(value),
                        {'Content-Type': 'application/json'},
                    )
                    response = client.getresponse()
                    self.assertEqual(response.status, 200)
                    self.assertTrue(json.loads(response.read())['saved'])
                    self.assertEqual(len(requests), count)
                    client.request(
                        'POST',
                        '/api/place/config',
                        json.dumps(value),
                        {
                            'Content-Type': 'application/json',
                            'Origin': 'http://wrong-host',
                        },
                    )
                    response = client.getresponse()
                    self.assertEqual(response.status, 403)
                    response.read()
                    with patch.object(
                        trackers[0].grasp,
                        'start_placement',
                        return_value={'accepted': True},
                    ) as place:
                        for path in ['/api/place/preview', '/api/place']:
                            client.request(
                                'POST',
                                path,
                                '{"hold_id":"sample"}',
                                {'Content-Type': 'application/json'},
                            )
                            response = client.getresponse()
                            self.assertEqual(response.status, 200)
                            response.read()
                        self.assertEqual(
                            place.call_args_list[0].kwargs, {'preview': True}
                        )
                        self.assertEqual(
                            place.call_args_list[1].args,
                            ({'hold_id': 'sample'},),
                        )
                    for path in [
                        '/api/placement/model', '/placement_viewer.js'
                    ]:
                        client.request('GET', path)
                        response = client.getresponse()
                        self.assertEqual(response.status, 404)
                        response.read()
                    self.assertIn('将物品摆至指定姿态', page)
                    self.assertNotIn('按指定姿态放置物品', page)
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
