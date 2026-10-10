import base64
import http.client
import json
import pathlib
import tempfile
import threading
import time
import types
import unittest
from unittest import mock
import http.server

import cv2
import numpy as np

from saydo.camera_monitor import server as monitor
from tests.camera_monitor import fake_realsense

JPEG = b'\xff\xd8test-frame\xff\xd9'


def fake_profile(rs, device):
    """让 Mock 出来的驱动像真设备一样给出彩色画面的内参。"""
    profile = rs.pipeline.return_value.start.return_value
    profile.get_device.return_value = device
    video = profile.get_stream.return_value.as_video_stream_profile
    video.return_value.get_intrinsics.return_value = types.SimpleNamespace(
        fx=650., fy=650., ppx=640., ppy=360., model='none', coeffs=[0.] * 5
    )
    device.first_depth_sensor.return_value.get_depth_scale.return_value = (
        .0001
    )


def tracker_frame(serial='435123456789', age=0):
    return {
        'camera': {'serial': serial, 'name': 'RealSense D435I'},
        'mime_type': 'image/jpeg',
        'image_base64': base64.b64encode(JPEG).decode(),
        'captured_at': time.time() - age,
        'sequence': 42,
        'width': 640,
        'height': 360,
    }


class TestCamera(unittest.TestCase):
    def test_tracker_reuses_raw_frame_without_device_access(self):
        camera = monitor.Camera(
            1280, 720, 15, serial='435123456789', tracker=mock.Mock()
        )
        stop = threading.Event()
        data = tracker_frame()

        def publish(*args):
            monitor.Camera.publish(camera, *args)
            stop.set()

        with mock.patch.object(camera, 'publish', side_effect=publish):
            camera.capture_tracker(data, stop)
        self.assertEqual(camera.source, 'tracker')
        self.assertEqual(camera.requested_size, (1280, 720))
        self.assertEqual((camera.width, camera.height), (640, 360))
        self.assertEqual(camera.jpeg, JPEG)
        camera.tracker.request.assert_not_called()

    def test_tracker_rejects_wrong_identity_and_stale_frame(self):
        for data in [
            tracker_frame('another-camera'),
            tracker_frame(age=2),
            {**tracker_frame(), 'mime_type': 'image/png'},
        ]:
            camera = monitor.Camera(
                1280, 720, 15, serial='435123456789', tracker=mock.Mock()
            )
            with self.assertRaises((RuntimeError, ValueError)):
                camera.capture_tracker(data, threading.Event())
            self.assertIsNone(camera.jpeg)

    def test_tracker_failure_does_not_open_device_or_issue_writes(self):
        from saydo.agent import client
        tracker = mock.Mock()
        tracker.request.side_effect = client.BackendError(
            {'code': 'CAMERA_UNAVAILABLE', 'message': 'stale'}
        )
        camera = monitor.Camera(1280, 720, 15, tracker=tracker)
        camera.capture(1, threading.Event())
        self.assertEqual(camera.state, 'error')
        self.assertIn('8089', camera.error)
        tracker.request.assert_called_once_with('/api/agent/frame')

    def test_second_arm_camera_shares_the_frame_8089_has_for_that_arm(self):
        tracker = mock.Mock()
        tracker.request.return_value = tracker_frame('344422070678')
        camera = monitor.Camera(
            1280, 720, 15, monitor.ARM2_CAMERA_KEY, '344422070678',
            tracker, arm='arm2',
        )
        stop = threading.Event()

        def publish(*args):
            monitor.Camera.publish(camera, *args)
            stop.set()

        with mock.patch.object(camera, 'publish', side_effect=publish):
            camera.capture(1, stop)
        tracker.request.assert_called_once_with('/api/agent/frame?arm=arm2')
        self.assertEqual(camera.source, 'tracker')

    def test_second_arm_camera_is_opened_directly_when_8089_lacks_it(self):
        from saydo.agent import client
        for code in ('UNKNOWN_ARM', 'CAMERA_UNAVAILABLE'):
            tracker = mock.Mock()
            tracker.request.side_effect = client.BackendError(
                {'code': code, 'message': 'not served by 8089'}
            )
            camera = monitor.Camera(
                1280, 720, 15, monitor.ARM2_CAMERA_KEY, '344422070678',
                tracker, arm='arm2',
            )
            rs = mock.Mock()
            rs.context.return_value.query_devices.return_value = []
            with self.subTest(code=code), mock.patch.dict(
                'sys.modules',
                {'pyrealsense2': rs, 'cv2': mock.Mock(), 'numpy': mock.Mock()},
            ):
                camera.capture(1, threading.Event())
                # 走到了枚举设备这一步，而不是停在“8089 画面不可用”。
                rs.context.return_value.query_devices.assert_called_once()
                self.assertNotIn('8089', camera.error)

    def test_subscribers_share_worker_and_limit_is_per_camera(self):
        camera = monitor.Camera(1280, 720, 15)
        with mock.patch.object(
            camera, 'capture', side_effect=lambda epoch, stop: stop.wait(2)
        ):
            epochs = [camera.subscribe() for _ in range(4)]
            thread = camera.thread
            self.assertEqual(len(set(epochs)), 1)
            with self.assertRaises(RuntimeError):
                camera.subscribe()
            for epoch in epochs:
                camera.unsubscribe(epoch)
            thread.join(2)
            self.assertTrue(camera.stop_event.is_set())

    def test_old_unsubscribe_cannot_remove_new_subscriber(self):
        camera = monitor.Camera(1280, 720, 15)
        camera.epoch = 2
        camera.clients = 1
        camera.unsubscribe(1)
        self.assertEqual(camera.clients, 1)
        self.assertFalse(camera.stop_event.is_set())

    def test_fresh_frame_never_starts_or_returns_idle_cached_image(self):
        camera = monitor.Camera(1280, 720, 15, 'd405', '405123456789')
        camera.jpeg = JPEG
        camera.frame_at = time.time()
        with self.assertRaises(RuntimeError):
            camera.fresh_frame()
        self.assertIsNone(camera.thread)

    def test_device_binding_and_independent_lock_release_on_error(self):
        camera = monitor.Camera(1280, 720, 15, 'd405', '405123456789')
        rs = mock.Mock()
        device = mock.Mock()
        device.get_info.side_effect = lambda key: '405123456789' if key == rs.camera_info.serial_number else 'RealSense D405'
        rs.context.return_value.query_devices.return_value = [device]
        fake_profile(rs, device)
        rs.pipeline.return_value.wait_for_frames.side_effect = RuntimeError(
            'fixture ended'
        )
        with tempfile.TemporaryDirectory() as directory, mock.patch.object(
            monitor, 'PROJECT', pathlib.Path(directory)
        ), mock.patch.object(
            monitor, 'camera_users', return_value=[]
        ) as users, mock.patch.dict(
            'sys.modules',
            {'pyrealsense2': rs, 'cv2': mock.Mock(), 'numpy': mock.Mock()},
        ):
            camera.capture(1, threading.Event())
        rs.config.return_value.enable_device.assert_called_once_with(
            '405123456789'
        )
        users.assert_called_once_with('405123456789')
        rs.pipeline.return_value.stop.assert_called_once()
        self.assertEqual(camera.state, 'error')
        self.assertIsNone(camera.jpeg)

    def test_multiple_devices_never_guess_serial(self):
        camera = monitor.Camera(1280, 720, 15, 'd405')
        rs = mock.Mock()
        device = mock.Mock()
        device.get_info.return_value = 'RealSense D405'
        rs.context.return_value.query_devices.return_value = [device, device]
        with mock.patch.dict(
            'sys.modules',
            {'pyrealsense2': rs, 'cv2': mock.Mock(), 'numpy': mock.Mock()},
        ):
            camera.capture(1, threading.Event())
        self.assertEqual(camera.state, 'error')
        rs.pipeline.assert_not_called()

    def test_slow_encoded_device_frame_is_discarded_then_preview_continues(
        self
    ):
        camera = monitor.Camera(1280, 720, 15, 'd435i', '435123456789')
        rs = mock.Mock()
        device = mock.Mock()
        color = mock.Mock()
        clock = [100.]
        device.get_info.side_effect = lambda key: '435123456789' if key == rs.camera_info.serial_number else 'RealSense D435I'
        rs.context.return_value.query_devices.return_value = [device]
        fake_profile(rs, device)
        rs.pipeline.return_value.wait_for_frames.return_value.get_color_frame.return_value = color
        color.get_frame_number.return_value = 2
        color.get_timestamp.return_value = 100000.
        color.get_width.return_value = 1280
        color.get_height.return_value = 720
        cv = mock.Mock()
        encoded = mock.Mock()
        encoded.tobytes.return_value = JPEG
        attempts = [0]

        def encode(*args):
            attempts[0] += 1
            if attempts[0] == 1:
                clock[0] += 2
            return True, encoded

        cv.imencode.side_effect = encode
        stop = threading.Event()

        def publish(*args):
            self.assertEqual(attempts[0], 2)
            stop.set()

        with mock.patch.dict(
            'sys.modules', {'pyrealsense2': rs, 'cv2': cv, 'numpy': mock.Mock()}
        ), mock.patch.object(
            monitor, 'camera_users', return_value=[]
        ), mock.patch.object(
            monitor.time, 'time', side_effect=lambda: clock[0]
        ), mock.patch.object(
            # 这个测试把图像库整个换成了替身，画面不是真的数组，写不进
            # 共享内存。
            monitor.camera_frames, 'FrameWriter'
        ), mock.patch.object(camera, 'publish', side_effect=publish) as output:
            camera.capture(1, stop)
        output.assert_called_once()
        self.assertIsNone(camera.error)


class FixtureCamera(monitor.Camera):
    def capture(self, epoch, stop):
        try:
            while not stop.is_set():
                self.publish(
                    JPEG,
                    time.time(),
                    self.sequence + 1,
                    {},
                    'device',
                    1280,
                    720,
                )
                stop.wait(.02)
        finally:
            with self.cv:
                self.clients = 0
                self.state = 'idle'
                self.jpeg = None
                self.frame_at = None
                self.cv.notify_all()


class TestHTTP(unittest.TestCase):
    def setUp(self):
        self.server = http.server.ThreadingHTTPServer(
            ('127.0.0.1', 0), monitor.Handler
        )
        self.server.daemon_threads = True
        self.server.cameras = {
            'd435i': FixtureCamera(1280, 720, 15, 'd435i', '435123456789'),
            'd405': FixtureCamera(1280, 720, 15, 'd405', '405123456789'),
        }
        self.worker = threading.Thread(
            target=self.server.serve_forever, daemon=True
        )
        self.worker.start()
        self.addCleanup(self.cleanup)

    def cleanup(self):
        for camera in self.server.cameras.values():
            camera.stop()
        self.server.shutdown()
        self.server.server_close()
        self.worker.join()

    def request(self, path, method='GET', headers=None):
        connection = http.client.HTTPConnection(
            *self.server.server_address, timeout=3
        )
        try:
            connection.request(method, path, headers=headers or {})
            response = connection.getresponse()
            return response.status, response.read()
        finally:
            connection.close()

    def open_stream(self, path):
        connection = http.client.HTTPConnection(
            *self.server.server_address, timeout=3
        )
        connection.request('GET', path)
        response = connection.getresponse()
        self.assertEqual(response.status, 200)
        self.assertIn(b'--frame', response.readline())
        self.addCleanup(connection.close)
        return response

    def test_dual_streams_advance_and_pause_independently(self):
        self.open_stream('/stream/d435i')
        self.open_stream('/stream/d405')
        code, data = self.request('/api/status')
        state = json.loads(data)
        self.assertEqual(code, 200)
        self.assertEqual(set(state['cameras']), {'d435i', 'd405'})
        for row in state['cameras'].values():
            self.assertEqual(row['state'], 'streaming')
        self.request('/api/cameras/d405/pause', 'POST')
        self.assertEqual(self.server.cameras['d405'].state, 'idle')
        self.assertEqual(self.server.cameras['d435i'].state, 'streaming')
        self.request('/api/pause', 'POST')
        self.assertEqual(
            json.loads(self.request('/api/status')[1])['state'], 'idle'
        )

    def test_observation_waits_for_next_frame_and_preserves_identity(self):
        self.open_stream('/stream/d405')
        before = time.time()
        code, raw = self.request('/api/cameras/d405/frame')
        data = json.loads(raw)
        self.assertEqual(code, 200)
        self.assertGreaterEqual(data['captured_at'], before)
        self.assertEqual(data['camera_serial'], '405123456789')
        self.assertEqual(data['camera_role'], 'movable_global')
        self.assertEqual(base64.b64decode(data['image_base64']), JPEG)
        self.assertEqual(self.server.cameras['d435i'].state, 'idle')

    def test_legacy_routes_and_idle_snapshot(self):
        self.assertEqual(self.request('/snapshot.jpg')[0], 409)
        self.open_stream('/stream')
        self.assertEqual(self.request('/snapshot.jpg')[1], JPEG)
        self.assertEqual(self.server.cameras['d405'].state, 'idle')
        self.assertEqual(self.request('/api/cameras/d405/frame')[0], 409)
        self.assertEqual(self.request('/stream/unknown')[0], 404)

    def test_cross_site_stream_and_pause_rejected_without_starting_camera(self):
        self.assertEqual(
            self.request(
                '/stream/d405', headers={'Sec-Fetch-Site': 'cross-site'}
            )[0],
            403,
        )
        self.assertEqual(
            self.request(
                '/api/pause', 'POST', {'Origin': 'http://elsewhere'}
            )[0],
            403,
        )
        for camera in self.server.cameras.values():
            self.assertIsNone(camera.thread)

    def test_last_viewer_releases_only_its_camera(self):
        connection = http.client.HTTPConnection(
            *self.server.server_address, timeout=3
        )
        connection.request('GET', '/stream/d405')
        response = connection.getresponse()
        response.readline()
        response.close()
        connection.close()
        deadline = time.monotonic() + 2
        while (
            self.server.cameras['d405'].state != 'idle'
            and time.monotonic() < deadline
        ):
            time.sleep(.02)
        self.assertEqual(self.server.cameras['d405'].state, 'idle')



class TestFramesForPrograms(unittest.TestCase):
    """程序（如标定）向本服务要画面：相机由本服务打开，预览照常。"""

    def setUp(self):
        rng = np.random.default_rng(7)
        self.image = rng.integers(0, 255, (72, 128, 3), dtype=np.uint8)
        self.rs = fake_realsense.Module(self.image)
        directory = tempfile.TemporaryDirectory()
        self.addCleanup(directory.cleanup)
        for patch in fake_realsense.patches(
            self.rs, monitor, directory.name
        ):
            patch.start()
            self.addCleanup(patch.stop)
        tracker = mock.Mock()
        sequence = iter(range(1, 100000))
        tracker.request.side_effect = lambda path: {
            **tracker_frame(), 'sequence': next(sequence)
        }
        self.server = http.server.ThreadingHTTPServer(
            ('127.0.0.1', 0), monitor.Handler
        )
        self.server.daemon_threads = True
        self.server.cameras = {
            'd435i': monitor.Camera(
                1280, 720, 15, 'd435i', '435123456789', tracker, arm='arm1'
            ),
            'd405': monitor.Camera(128, 72, 15, 'd405', '405123456789'),
        }
        self.camera = self.server.cameras['d405']
        self.worker = threading.Thread(
            target=self.server.serve_forever, daemon=True
        )
        self.worker.start()
        self.addCleanup(self.cleanup)

    def cleanup(self):
        for camera in self.server.cameras.values():
            camera.stop()
        self.server.shutdown()
        self.server.server_close()
        self.worker.join()

    def request(self, path, method='GET', headers=None, body=None):
        connection = http.client.HTTPConnection(
            *self.server.server_address, timeout=15
        )
        try:
            connection.request(method, path, body=body, headers=headers or {})
            response = connection.getresponse()
            return response.status, json.loads(response.read())
        finally:
            connection.close()

    def wait_idle(self, timeout_s=3.):
        deadline = time.monotonic() + timeout_s
        while self.camera.state != 'idle' and time.monotonic() < deadline:
            time.sleep(.02)
        return self.camera.state

    def test_request_starts_the_camera_and_returns_the_exact_pixels(self):
        code, data = self.request('/api/cameras/d405/frame?keep_s=5&format=png')
        self.assertEqual(code, 200)
        self.assertEqual(data['mime_type'], 'image/png')
        self.assertEqual(data['camera_serial'], '405123456789')
        decoded = cv2.imdecode(
            np.frombuffer(base64.b64decode(data['image_base64']), np.uint8),
            cv2.IMREAD_COLOR,
        )
        # 无损：每个像素都与相机给出的一样。
        self.assertTrue(np.array_equal(decoded, self.image))
        self.assertEqual(
            data['intrinsics'],
            {
                'fx': 650., 'fy': 651., 'cx': 640., 'cy': 360.,
                'distortion_model': 'distortion.inverse_brown_conrady',
                'distortion': [0.] * 5,
            },
        )
        status = self.request('/api/cameras/d405/status')[1]
        self.assertEqual(status['state'], 'streaming')
        self.assertEqual(status['clients'], 0)
        self.assertGreater(status['kept_s'], 3)
        self.assertEqual(status['intrinsics']['fx'], 650.)
        self.assertEqual(self.rs.started, 1)

    def test_preview_and_program_share_one_open_camera(self):
        self.request('/api/cameras/d405/frame?keep_s=5')
        viewer = http.client.HTTPConnection(
            *self.server.server_address, timeout=3
        )
        viewer.request('GET', '/stream/d405')
        stream = viewer.getresponse()
        self.assertEqual(stream.status, 200)
        self.assertIn(b'--frame', stream.readline())
        code, data = self.request('/api/cameras/d405/frame?keep_s=5&format=png')
        self.assertEqual(code, 200)
        # 观看者离开后，程序还在用，相机不释放。
        stream.close()
        viewer.close()
        time.sleep(.3)
        self.assertEqual(self.camera.state, 'streaming')
        self.assertEqual(self.request('/api/cameras/d405/frame')[0], 200)
        self.assertEqual((self.rs.started, self.rs.stopped), (1, 0))

    def test_camera_is_released_once_the_program_stops_asking(self):
        self.assertEqual(
            self.request('/api/cameras/d405/frame?keep_s=0.3')[0], 200
        )
        self.assertEqual(self.wait_idle(), 'idle')
        self.assertEqual((self.rs.started, self.rs.stopped), (1, 1))
        self.assertEqual(
            self.request('/api/cameras/d405/status')[1]['kept_s'], 0
        )
        # 再来取图时重新打开。
        self.assertEqual(
            self.request('/api/cameras/d405/frame?keep_s=0.3')[0], 200
        )
        self.assertEqual(self.rs.started, 2)

    def test_a_viewer_keeps_the_camera_after_the_program_is_gone(self):
        viewer = http.client.HTTPConnection(
            *self.server.server_address, timeout=3
        )
        viewer.request('GET', '/stream/d405')
        stream = viewer.getresponse()
        stream.readline()
        self.request('/api/cameras/d405/frame?keep_s=0.2')
        time.sleep(.5)
        self.assertEqual(self.camera.state, 'streaming')
        stream.close()
        viewer.close()
        self.assertEqual(self.wait_idle(), 'idle')

    def test_plain_request_still_never_starts_the_camera(self):
        code, data = self.request('/api/cameras/d405/frame?format=png')
        self.assertEqual(code, 409)
        self.assertEqual(self.rs.started, 0)

    def test_camera_lent_to_a_capture_program_is_not_started(self):
        self.request(
            '/api/cameras/d405/pause', 'POST', body=b'{"hold_s": 30}'
        )
        code, data = self.request('/api/cameras/d405/frame?keep_s=5')
        self.assertEqual(code, 409)
        self.assertIn('已让给采集程序', data['error'])
        self.assertEqual(self.rs.started, 0)

    def test_relayed_8089_picture_has_no_lossless_image(self):
        code, data = self.request(
            '/api/cameras/d435i/frame?keep_s=5&format=png'
        )
        self.assertEqual(code, 409)
        self.assertIn('没有无损图像', data['error'])
        # 压缩的画面照常给出。
        code, data = self.request('/api/cameras/d435i/frame?keep_s=5')
        self.assertEqual(code, 200)
        self.assertIsNone(data['intrinsics'])
        self.assertEqual(self.rs.started, 0)

    def test_bad_parameters_and_cross_site_requests_are_refused(self):
        for query in ('keep_s=abc', 'keep_s=-1', 'keep_s=61', 'keep_s=nan',
                      'keep_s=5&format=bmp'):
            with self.subTest(query=query):
                self.assertEqual(
                    self.request('/api/cameras/d405/frame?' + query)[0], 400
                )
        self.assertEqual(
            self.request(
                '/api/cameras/d405/frame?keep_s=5',
                headers={'Sec-Fetch-Site': 'cross-site'},
            )[0],
            403,
        )
        self.assertEqual(self.rs.started, 0)

    def test_a_camera_that_cannot_open_reports_why(self):
        self.rs.device.serial = 'another'
        code, data = self.request('/api/cameras/d405/frame?keep_s=5')
        self.assertEqual(code, 409)
        self.assertIn('没有找到唯一的指定相机', data['error'])


class TestSecondArmCameraAndHold(unittest.TestCase):
    """2 臂相机一路，以及把相机让给采集程序再归还。"""

    KEY = monitor.ARM2_CAMERA_KEY

    def setUp(self):
        self.server = http.server.ThreadingHTTPServer(
            ('127.0.0.1', 0), monitor.Handler
        )
        self.server.daemon_threads = True
        self.server.cameras = {
            'd435i': FixtureCamera(
                1280, 720, 15, 'd435i', '405622073572', arm='arm1'
            ),
            'd405': FixtureCamera(1280, 720, 15, 'd405', '218622270784'),
            self.KEY: FixtureCamera(
                1280, 720, 15, self.KEY, '344422070678', arm='arm2'
            ),
        }
        self.worker = threading.Thread(
            target=self.server.serve_forever, daemon=True
        )
        self.worker.start()
        self.addCleanup(self.cleanup)

    def cleanup(self):
        for camera in self.server.cameras.values():
            camera.stop()
        self.server.shutdown()
        self.server.server_close()
        self.worker.join()

    def request(self, path, method='GET', body=None):
        connection = http.client.HTTPConnection(
            *self.server.server_address, timeout=3
        )
        try:
            connection.request(
                method, path, None if body is None else json.dumps(body)
            )
            response = connection.getresponse()
            return response.status, json.loads(response.read() or b'{}')
        finally:
            connection.close()

    def open_stream(self, key):
        connection = http.client.HTTPConnection(
            *self.server.server_address, timeout=3
        )
        connection.request('GET', '/stream/' + key)
        response = connection.getresponse()
        self.addCleanup(connection.close)
        return response

    def test_second_arm_camera_is_listed_as_an_eye_in_hand_d435i(self):
        row = self.request('/api/status')[1]['cameras'][self.KEY]
        self.assertEqual(row['camera'], 'RealSense D435I')
        self.assertEqual(row['camera_role'], 'eye_in_hand')
        self.assertEqual(row['camera_serial'], '344422070678')
        self.assertEqual(row['arm'], 'arm2')
        self.assertEqual(row['held_s'], 0)
        self.assertEqual(self.open_stream(self.KEY).status, 200)

    def test_held_camera_refuses_preview_until_it_is_returned(self):
        self.assertEqual(self.open_stream(self.KEY).status, 200)
        self.assertEqual(self.open_stream('d435i').status, 200)
        code, state = self.request(
            f'/api/cameras/{self.KEY}/pause', 'POST', {'hold_s': 30}
        )
        self.assertEqual(code, 200)
        row = state['cameras'][self.KEY]
        self.assertEqual(row['state'], 'idle')
        self.assertGreater(row['held_s'], 25)
        # 页面此时自动重连也连不上，采集程序可以放心打开相机。
        refused = self.open_stream(self.KEY)
        self.assertEqual(refused.status, 409)
        self.assertIn('让给采集程序', json.loads(refused.read())['error'])
        self.assertEqual(self.server.cameras[self.KEY].state, 'idle')
        # 只让出这一台，另一路预览不受影响。
        self.assertEqual(state['cameras']['d435i']['state'], 'streaming')
        self.assertEqual(state['cameras']['d435i']['held_s'], 0)
        code, state = self.request(
            f'/api/cameras/{self.KEY}/resume', 'POST', {}
        )
        self.assertEqual(code, 200)
        self.assertEqual(state['cameras'][self.KEY]['held_s'], 0)
        self.assertEqual(self.open_stream(self.KEY).status, 200)

    def test_hold_expires_on_its_own_and_is_capped(self):
        camera = self.server.cameras[self.KEY]
        self.request(
            f'/api/cameras/{self.KEY}/pause', 'POST', {'hold_s': 99999}
        )
        self.assertLessEqual(camera.status()['held_s'], monitor.MAX_HOLD_S)
        with mock.patch.object(
            monitor.time, 'monotonic', return_value=camera.hold_until + 1
        ):
            self.assertEqual(camera.status()['held_s'], 0)
            camera.subscribe()

    def test_plain_pause_and_pause_all_do_not_hold(self):
        self.request(f'/api/cameras/{self.KEY}/pause', 'POST')
        self.request('/api/pause', 'POST', {'hold_s': 30})
        for row in self.request('/api/status')[1]['cameras'].values():
            self.assertEqual(row['held_s'], 0)

    def test_invalid_hold_is_rejected_without_touching_the_camera(self):
        self.assertEqual(self.open_stream(self.KEY).status, 200)
        for body in ({'hold_s': -1}, {'hold_s': 'long'}, {'other': 1}, [30]):
            code, reply = self.request(
                f'/api/cameras/{self.KEY}/pause', 'POST', body
            )
            self.assertEqual(code, 400, body)
            self.assertIn('error', reply)
        self.assertEqual(self.server.cameras[self.KEY].state, 'streaming')


class TestCameraSelectionAtStartup(unittest.TestCase):
    def start(self, arguments, settings=None):
        created = []

        class Server:
            def __init__(self, address, handler):
                created.append(self)

            def serve_forever(self):
                pass

            def server_close(self):
                pass

        with tempfile.TemporaryDirectory() as directory:
            root = pathlib.Path(directory)
            if settings is not None:
                path = root / 'config/cameras/global.json'
                path.parent.mkdir(parents=True)
                path.write_text(json.dumps(settings))
            with mock.patch.object(monitor, 'PROJECT', root), mock.patch.object(
                http.server, 'ThreadingHTTPServer', Server
            ), mock.patch.object(monitor.signal, 'signal'), mock.patch.object(
                monitor.logging, 'basicConfig'
            ), mock.patch('sys.argv', ['server.py', *arguments]):
                monitor.main()
        return created[0].cameras

    def test_second_arm_camera_comes_from_the_local_config(self):
        cameras = self.start(
            [],
            {
                'camera_name': 'RealSense D405',
                'serial': '218622270784',
                'eye_in_hand_serial': '405622073572',
                'arm2_eye_in_hand_serial': '344422070678',
            },
        )
        self.assertEqual(
            list(cameras), ['d435i', 'd405', monitor.ARM2_CAMERA_KEY]
        )
        second = cameras[monitor.ARM2_CAMERA_KEY]
        self.assertEqual((second.serial, second.arm), ('344422070678', 'arm2'))
        # 没有给 --tracker-url：两路手上相机都由本服务直接采集。
        self.assertIsNone(second.tracker)
        self.assertEqual(second.frame_path, '/api/agent/frame?arm=arm2')
        self.assertEqual(cameras['d435i'].frame_path, '/api/agent/frame')
        self.assertEqual(cameras['d435i'].arm, 'arm1')

    def test_without_a_second_serial_the_page_keeps_two_cameras(self):
        cameras = self.start([], {'eye_in_hand_serial': '405622073572'})
        self.assertEqual(list(cameras), ['d435i', 'd405'])

    def test_second_arm_camera_requires_the_first_to_be_named(self):
        with self.assertRaises(SystemExit):
            self.start(['--arm2-serial', '344422070678'])
        with self.assertRaises(SystemExit):
            self.start(
                [
                    '--arm2-serial',
                    '344422070678',
                    '--eye-in-hand-serial',
                    '344422070678',
                ]
            )


if __name__ == '__main__':
    unittest.main()
