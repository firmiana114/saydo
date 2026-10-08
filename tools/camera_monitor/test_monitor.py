import base64
import http.client
import json
from pathlib import Path
import tempfile
import threading
import time
import unittest
from unittest.mock import Mock, patch
from http.server import ThreadingHTTPServer

from tools.camera_monitor import server as monitor

JPEG=b'\xff\xd8test-frame\xff\xd9'


def tracker_frame(serial='435123456789', age=0):
    return {'camera':{'serial':serial,'name':'RealSense D435I'},'mime_type':'image/jpeg',
            'image_base64':base64.b64encode(JPEG).decode(),'captured_at':time.time()-age,
            'sequence':42,'width':640,'height':360}


class TestCamera(unittest.TestCase):
    def test_tracker_reuses_raw_frame_without_device_access(self):
        camera=monitor.Camera(1280,720,15,serial='435123456789',tracker=Mock())
        stop=threading.Event()
        data=tracker_frame()
        def publish(*args):
            monitor.Camera.publish(camera,*args);stop.set()
        with patch.object(camera,'publish',side_effect=publish):camera.capture_tracker(data,stop)
        self.assertEqual(camera.source,'tracker');self.assertEqual(camera.requested_size,(1280,720))
        self.assertEqual((camera.width,camera.height),(640,360));self.assertEqual(camera.jpeg,JPEG)
        camera.tracker.request.assert_not_called()

    def test_tracker_rejects_wrong_identity_and_stale_frame(self):
        for data in [tracker_frame('another-camera'),tracker_frame(age=2),
                     {**tracker_frame(),'mime_type':'image/png'}]:
            camera=monitor.Camera(1280,720,15,serial='435123456789',tracker=Mock())
            with self.assertRaises((RuntimeError,ValueError)):camera.capture_tracker(data,threading.Event())
            self.assertIsNone(camera.jpeg)

    def test_tracker_failure_does_not_open_device_or_issue_writes(self):
        from tools.robot_agent.client import BackendError
        tracker=Mock();tracker.request.side_effect=BackendError({'code':'CAMERA_UNAVAILABLE','message':'stale'})
        camera=monitor.Camera(1280,720,15,tracker=tracker)
        camera.capture(1,threading.Event())
        self.assertEqual(camera.state,'error');self.assertIn('8089',camera.error)
        tracker.request.assert_called_once_with('/api/agent/frame')

    def test_subscribers_share_worker_and_limit_is_per_camera(self):
        camera=monitor.Camera(1280,720,15)
        with patch.object(camera,'capture',side_effect=lambda epoch,stop:stop.wait(2)):
            epochs=[camera.subscribe() for _ in range(4)]
            thread=camera.thread
            self.assertEqual(len(set(epochs)),1)
            with self.assertRaises(RuntimeError):camera.subscribe()
            for epoch in epochs:camera.unsubscribe(epoch)
            thread.join(2)
            self.assertTrue(camera.stop_event.is_set())

    def test_old_unsubscribe_cannot_remove_new_subscriber(self):
        camera=monitor.Camera(1280,720,15)
        camera.epoch=2;camera.clients=1
        camera.unsubscribe(1)
        self.assertEqual(camera.clients,1);self.assertFalse(camera.stop_event.is_set())

    def test_fresh_frame_never_starts_or_returns_idle_cached_image(self):
        camera=monitor.Camera(1280,720,15,'d405','405123456789')
        camera.jpeg=JPEG;camera.frame_at=time.time()
        with self.assertRaises(RuntimeError):camera.fresh_frame()
        self.assertIsNone(camera.thread)

    def test_device_binding_and_independent_lock_release_on_error(self):
        camera=monitor.Camera(1280,720,15,'d405','405123456789')
        rs=Mock();device=Mock()
        device.get_info.side_effect=lambda key: '405123456789' if key==rs.camera_info.serial_number else 'RealSense D405'
        rs.context.return_value.query_devices.return_value=[device]
        rs.pipeline.return_value.start.return_value.get_device.return_value=device
        rs.pipeline.return_value.wait_for_frames.side_effect=RuntimeError('fixture ended')
        with tempfile.TemporaryDirectory() as directory,patch.object(monitor,'PROJECT',Path(directory)),patch.object(monitor,'camera_users',return_value=[]) as users,patch.dict('sys.modules',{'pyrealsense2':rs,'cv2':Mock(),'numpy':Mock()}):
            camera.capture(1,threading.Event())
        rs.config.return_value.enable_device.assert_called_once_with('405123456789')
        users.assert_called_once_with('405123456789')
        rs.pipeline.return_value.stop.assert_called_once()
        self.assertEqual(camera.state,'error');self.assertIsNone(camera.jpeg)

    def test_multiple_devices_never_guess_serial(self):
        camera=monitor.Camera(1280,720,15,'d405')
        rs=Mock();device=Mock();device.get_info.return_value='RealSense D405'
        rs.context.return_value.query_devices.return_value=[device,device]
        with patch.dict('sys.modules',{'pyrealsense2':rs,'cv2':Mock(),'numpy':Mock()}):camera.capture(1,threading.Event())
        self.assertEqual(camera.state,'error');rs.pipeline.assert_not_called()

    def test_slow_encoded_device_frame_is_discarded_then_preview_continues(self):
        camera=monitor.Camera(1280,720,15,'d435i','435123456789')
        rs=Mock();device=Mock();color=Mock();clock=[100.]
        device.get_info.side_effect=lambda key:'435123456789' if key==rs.camera_info.serial_number else 'RealSense D435I'
        rs.context.return_value.query_devices.return_value=[device]
        rs.pipeline.return_value.start.return_value.get_device.return_value=device
        rs.pipeline.return_value.wait_for_frames.return_value.get_color_frame.return_value=color
        color.get_frame_number.return_value=2;color.get_width.return_value=1280;color.get_height.return_value=720
        cv=Mock();encoded=Mock();encoded.tobytes.return_value=JPEG;attempts=[0]
        def encode(*args):
            attempts[0]+=1
            if attempts[0]==1:clock[0]+=2
            return True,encoded
        cv.imencode.side_effect=encode;stop=threading.Event()
        def publish(*args):
            self.assertEqual(attempts[0],2);stop.set()
        with patch.dict('sys.modules',{'pyrealsense2':rs,'cv2':cv,'numpy':Mock()}),patch.object(monitor,'camera_users',return_value=[]),patch.object(monitor.time,'time',side_effect=lambda:clock[0]),patch.object(camera,'publish',side_effect=publish) as output:
            camera.capture(1,stop)
        output.assert_called_once();self.assertIsNone(camera.error)


class FixtureCamera(monitor.Camera):
    def capture(self,epoch,stop):
        try:
            while not stop.is_set():
                self.publish(JPEG,time.time(),self.sequence+1,{},'device',1280,720)
                stop.wait(.02)
        finally:
            with self.cv:
                self.clients=0;self.state='idle';self.jpeg=None;self.frame_at=None;self.cv.notify_all()


class TestHTTP(unittest.TestCase):
    def setUp(self):
        self.server=ThreadingHTTPServer(('127.0.0.1',0),monitor.Handler)
        self.server.daemon_threads=True
        self.server.cameras={'d435i':FixtureCamera(1280,720,15,'d435i','435123456789'),
                             'd405':FixtureCamera(1280,720,15,'d405','405123456789')}
        self.worker=threading.Thread(target=self.server.serve_forever,daemon=True);self.worker.start()
        self.addCleanup(self.cleanup)

    def cleanup(self):
        for camera in self.server.cameras.values():camera.stop()
        self.server.shutdown();self.server.server_close();self.worker.join()

    def request(self,path,method='GET',headers=None):
        connection=http.client.HTTPConnection(*self.server.server_address,timeout=3)
        try:
            connection.request(method,path,headers=headers or {})
            response=connection.getresponse();return response.status,response.read()
        finally:connection.close()

    def open_stream(self,path):
        connection=http.client.HTTPConnection(*self.server.server_address,timeout=3)
        connection.request('GET',path);response=connection.getresponse()
        self.assertEqual(response.status,200)
        self.assertIn(b'--frame',response.readline())
        self.addCleanup(connection.close)
        return response

    def test_dual_streams_advance_and_pause_independently(self):
        self.open_stream('/stream/d435i');self.open_stream('/stream/d405')
        code,data=self.request('/api/status');state=json.loads(data)
        self.assertEqual(code,200);self.assertEqual(set(state['cameras']),{'d435i','d405'})
        for row in state['cameras'].values():self.assertEqual(row['state'],'streaming')
        self.request('/api/cameras/d405/pause','POST')
        self.assertEqual(self.server.cameras['d405'].state,'idle')
        self.assertEqual(self.server.cameras['d435i'].state,'streaming')
        self.request('/api/pause','POST')
        self.assertEqual(json.loads(self.request('/api/status')[1])['state'],'idle')

    def test_observation_waits_for_next_frame_and_preserves_identity(self):
        self.open_stream('/stream/d405')
        before=time.time();code,raw=self.request('/api/cameras/d405/frame');data=json.loads(raw)
        self.assertEqual(code,200);self.assertGreaterEqual(data['captured_at'],before)
        self.assertEqual(data['camera_serial'],'405123456789');self.assertEqual(data['camera_role'],'movable_global')
        self.assertEqual(base64.b64decode(data['image_base64']),JPEG)
        self.assertEqual(self.server.cameras['d435i'].state,'idle')

    def test_legacy_routes_and_idle_snapshot(self):
        self.assertEqual(self.request('/snapshot.jpg')[0],409)
        self.open_stream('/stream')
        self.assertEqual(self.request('/snapshot.jpg')[1],JPEG)
        self.assertEqual(self.server.cameras['d405'].state,'idle')
        self.assertEqual(self.request('/api/cameras/d405/frame')[0],409)
        self.assertEqual(self.request('/stream/unknown')[0],404)

    def test_cross_site_stream_and_pause_rejected_without_starting_camera(self):
        self.assertEqual(self.request('/stream/d405',headers={'Sec-Fetch-Site':'cross-site'})[0],403)
        self.assertEqual(self.request('/api/pause','POST',{'Origin':'http://elsewhere'})[0],403)
        for camera in self.server.cameras.values():self.assertIsNone(camera.thread)

    def test_last_viewer_releases_only_its_camera(self):
        connection=http.client.HTTPConnection(*self.server.server_address,timeout=3)
        connection.request('GET','/stream/d405');response=connection.getresponse();response.readline()
        response.close();connection.close()
        deadline=time.monotonic()+2
        while self.server.cameras['d405'].state!='idle' and time.monotonic()<deadline:time.sleep(.02)
        self.assertEqual(self.server.cameras['d405'].state,'idle')


if __name__=='__main__':unittest.main()
