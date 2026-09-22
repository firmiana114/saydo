"""Runtime mode switching and actual HTTP routes, with camera/robot workers disabled."""
from collections import deque
import http.client
from http.server import ThreadingHTTPServer
import json
from pathlib import Path
import tempfile
import threading
import unittest
from unittest.mock import Mock, patch

import numpy as np

import live_tracker
from icp_refinement import ICPSettings


def tracker_fixture():
    tracker = live_tracker.Tracker.__new__(live_tracker.Tracker)
    tracker.icp_mode = 'off'; tracker.icp_revision = 0
    tracker.icp_settings = ICPSettings()
    tracker.lock = threading.RLock(); tracker.stop = threading.Event()
    tracker.generation = 7; tracker.pose_history = deque([{'pose': np.eye(4).tolist()}])
    tracker.state = {'state': 'tracking', 'quality': {'good': True}, 'camera_T_object': np.eye(4).tolist(),
                     'pose_frame_at': 100., 'sequence': 17}
    tracker.state['icp_config'] = tracker.icp_configuration()
    tracker.grasp = Mock(); tracker.grasp.snapshot.return_value = {'busy': False}
    tracker.event = Mock(); tracker.events = Mock()
    return tracker


class ICPLiveTests(unittest.TestCase):
    def test_switch_invalidates_old_result_and_stable_history_without_restart(self):
        t = tracker_fixture()
        self.assertTrue(t.icp_result_current(7, 0))
        result = t.set_icp_mode('shadow', 0)
        self.assertEqual(result, {'mode': 'shadow', 'revision': 1, 'budget_ms': 20.})
        self.assertFalse(t.icp_result_current(7, 0))
        self.assertTrue(t.icp_result_current(7, 1))
        self.assertFalse(t.icp_result_current(6, 1))
        self.assertEqual(len(t.pose_history), 0)
        self.assertIsNone(t.state['camera_T_object'])
        self.assertEqual(t.state['quality'], {})
        self.assertEqual(t.state['pose_frame_at'], 100.)
        self.assertEqual(t.state['sequence'], 17)
        self.assertFalse(t.stop.is_set())
        t.grasp.start.assert_not_called();t.grasp.abort.assert_not_called()
        self.assertEqual(t.set_icp_mode('apply', 1)['revision'], 2)
        self.assertEqual(t.set_icp_mode('off', 2)['revision'], 3)

    def test_idempotent_request_preserves_stable_history(self):
        t = tracker_fixture()
        t.set_icp_mode('off', 0)
        self.assertEqual(len(t.pose_history), 1)

    def test_switch_during_initialization_discards_frame_but_allows_next_tracking_frame(self):
        t = tracker_fixture()
        for state in ('initializing', 'catching_up'):
            t.state['state'] = state
            old_revision = t.icp_revision
            t.set_icp_mode('apply' if t.icp_mode == 'off' else 'off', old_revision)
            self.assertFalse(t.icp_result_current(t.generation, old_revision))
            self.assertEqual(t.state['state'], 'tracking')
            self.assertIsNone(t.state['camera_T_object'])
            self.assertTrue(t.icp_result_current(t.generation, t.icp_revision))
        t.state['state'] = 'paused'
        t.generation += 1
        self.assertFalse(t.icp_result_current(t.generation-1, old_revision))
        self.assertEqual(t.state['state'], 'paused')

    def test_invalid_busy_requests_leave_configuration_unchanged(self):
        t = tracker_fixture()
        for mode, revision in [('invalid', 0), ('apply', True), ('apply', -1), ('apply', None)]:
            with self.assertRaises(ValueError):t.set_icp_mode(mode, revision)
        t.grasp.snapshot.return_value = {'busy': True}
        with self.assertRaisesRegex(ValueError, '任务处理中'):t.set_icp_mode('apply', 0)
        self.assertEqual(t.icp_mode, 'off')
        self.assertEqual(len(t.pose_history), 1)

    def test_http_switch_status_and_script_without_hardware(self):
        t = tracker_fixture()
        t.capture = lambda: t.stop.wait()
        t.inference = lambda: t.stop.wait()
        ready = threading.Event();servers=[];errors=[]
        def make_server(address, handler):
            server = ThreadingHTTPServer(('127.0.0.1', 0), handler)
            servers.append(server);ready.set();return server
        def serve():
            try:live_tracker.main()
            except Exception as error:errors.append(error);ready.set()
        with tempfile.TemporaryDirectory() as directory:
            root=Path(directory);(root/'work').mkdir()
            with patch.object(live_tracker,'ROOT',root), patch.object(live_tracker,'Tracker',return_value=t), \
                 patch.object(live_tracker,'ThreadingHTTPServer',side_effect=make_server), \
                 patch.object(live_tracker.signal,'signal'), \
                 patch.object(live_tracker.urllib.request,'urlopen',side_effect=OSError('no camera service')), \
                 patch('sys.argv',['live_tracker.py']):
                thread=threading.Thread(target=serve,daemon=True);thread.start()
                client=None
                try:
                    self.assertTrue(ready.wait(5));self.assertFalse(errors)
                    client=http.client.HTTPConnection(*servers[0].server_address,timeout=5)
                    client.request('GET','/icp_controls.js');response=client.getresponse()
                    self.assertEqual(response.status,200);self.assertIn(b'ICPControls',response.read())
                    for request,status in [({'mode':'shadow','revision':0},200),({'mode':'apply','revision':0},400),({'mode':'apply','revision':1},200)]:
                        client.request('POST','/api/icp',json.dumps(request),{'Content-Type':'application/json'})
                        response=client.getresponse();body=json.loads(response.read())
                        self.assertEqual(response.status,status,body)
                    client.request('GET','/api/status');response=client.getresponse();body=json.loads(response.read())
                    self.assertEqual(body['icp_config']['mode'],'apply')
                    self.assertIsNone(body['camera_T_object'])
                    self.assertEqual(t.icp_revision,2)
                    t.grasp.start.assert_not_called();t.grasp.abort.assert_not_called()
                finally:
                    if client:client.close()
                    t.stop.set()
                    if servers:servers[0].shutdown()
                    thread.join(timeout=5)
                self.assertFalse(thread.is_alive());self.assertFalse(errors)


if __name__ == '__main__':
    unittest.main()
