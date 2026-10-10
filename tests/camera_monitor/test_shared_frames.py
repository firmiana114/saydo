"""相机服务把原始画面放进共享内存，取图程序经 `Feed` 持续读取。

假的只有相机驱动；相机服务的采集与接口、共享内存的读写都是真实代码。
"""

import base64
import http.client
import http.server
import json
import tempfile
import threading
import time
import unittest
from unittest import mock

import numpy as np

from saydo.camera_monitor import server as monitor
from saydo.contracts import camera_frames
from tests.camera_monitor import fake_realsense


class SharedFrameTests(unittest.TestCase):
    def setUp(self):
        rng = np.random.default_rng(11)
        self.rs = fake_realsense.Module(
            rng.integers(0, 255, (72, 128, 3), dtype=np.uint8)
        )
        self.rs.depth = rng.integers(0, 60000, (72, 128), dtype=np.uint16)
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
            'camera': {'serial': '435123456789', 'name': 'RealSense D435I'},
            'mime_type': 'image/jpeg',
            'image_base64': base64.b64encode(b'\xff\xd8relayed').decode(),
            'captured_at': time.time(),
            'sequence': next(sequence),
            'width': 640,
            'height': 360,
        }
        self.server = http.server.ThreadingHTTPServer(
            ('127.0.0.1', 0), monitor.Handler
        )
        self.server.daemon_threads = True
        self.server.cameras = {
            'd435i': monitor.Camera(
                128, 72, 15, 'd435i', '435123456789', tracker, arm='arm1'
            ),
            'd405': monitor.Camera(
                128, 72, 15, 'd405', '405123456789', depth=True
            ),
        }
        self.camera = self.server.cameras['d405']
        self.worker = threading.Thread(
            target=self.server.serve_forever, daemon=True
        )
        self.worker.start()
        self.addCleanup(self.cleanup)
        self.url = 'http://127.0.0.1:%d' % self.server.server_address[1]

    def cleanup(self):
        for camera in self.server.cameras.values():
            camera.stop()
        self.server.shutdown()
        self.server.server_close()
        self.worker.join()

    def feed(self, key='d405', keep_s=15.):
        feed = camera_frames.Feed(self.url, key, keep_s)
        self.addCleanup(feed.close)
        return feed

    def wait_idle(self, timeout_s=3.):
        deadline = time.monotonic() + timeout_s
        while self.camera.state != 'idle' and time.monotonic() < deadline:
            time.sleep(.02)
        return self.camera.state

    def test_feed_starts_the_camera_and_reads_colour_and_depth(self):
        feed = self.feed()
        before = time.time()
        first = feed.frame()
        self.assertTrue(np.array_equal(first['bgr'], self.rs.image))
        self.assertTrue(np.array_equal(first['depth_raw'], self.rs.depth))
        self.assertEqual(first['depth_scale_m'], fake_realsense.DEPTH_SCALE_M)
        self.assertEqual(first['serial'], '405123456789')
        self.assertTrue(
            np.allclose(
                first['K'],
                [[650., 0., 640.], [0., 651., 360.], [0., 0., 1.]],
            )
        )
        # 曝光时刻取相机给出的时间戳，比主机收到的时刻早。
        self.assertAlmostEqual(
            first['received_at'] - first['captured_at'],
            fake_realsense.EXPOSURE_LEAD_S, delta=.005,
        )
        self.assertGreater(first['received_at'], before)
        # 每次取到的都是更新的一帧。
        sequences = [first['sequence']] + [
            feed.frame()['sequence'] for _ in range(5)
        ]
        self.assertEqual(sequences, sorted(set(sequences)))
        self.assertEqual(self.rs.started, 1)
        self.assertEqual(self.camera.status()['clients'], 0)

    def test_frames_arrive_at_the_camera_rate_without_extra_delay(self):
        feed = self.feed()
        feed.frame()
        started = time.monotonic()
        ages = []
        for _ in range(30):
            frame = feed.frame()
            ages.append(time.time() - frame['received_at'])
        elapsed = time.monotonic() - started
        # 假相机每 10 毫秒一帧；取 30 帧不应明显慢于相机出图。
        self.assertLess(elapsed, 30 * fake_realsense.FRAME_PERIOD_S * 3)
        self.assertLess(float(np.median(ages)), .02)

    def test_preview_and_feed_share_one_open_camera(self):
        feed = self.feed()
        feed.frame()
        viewer = http.client.HTTPConnection(
            *self.server.server_address, timeout=3
        )
        self.addCleanup(viewer.close)
        viewer.request('GET', '/stream/d405')
        stream = viewer.getresponse()
        self.assertIn(b'--frame', stream.readline())
        self.assertTrue(np.array_equal(feed.frame()['bgr'], self.rs.image))
        self.assertEqual((self.rs.started, self.rs.stopped), (1, 0))

    def test_camera_is_released_after_the_feed_goes_quiet(self):
        feed = self.feed(keep_s=.3)
        feed.frame()
        path = self.camera.status()['shared_frames']['path']
        self.assertEqual(self.wait_idle(), 'idle')
        self.assertEqual((self.rs.started, self.rs.stopped), (1, 1))
        self.assertIsNone(self.camera.status()['shared_frames'])
        with self.assertRaises(FileNotFoundError):
            open(path, 'rb')
        # 再取图时相机重新打开，取图程序自动接上新的共享内存。
        self.assertTrue(np.array_equal(feed.frame()['bgr'], self.rs.image))
        self.assertEqual(self.rs.started, 2)

    def test_feed_follows_a_camera_that_was_reopened_meanwhile(self):
        feed = self.feed()
        feed.frame()
        # 有人在页面上点了“释放”，随后相机被重新打开，画面也变了。
        self.camera.stop()
        self.rs.image = self.rs.image[::-1].copy()
        frame = feed.frame()
        self.assertTrue(np.array_equal(frame['bgr'], self.rs.image))
        self.assertEqual(self.rs.started, 2)

    def test_reasons_are_given_when_no_raw_picture_can_be_had(self):
        # 这一路转接的是 8089 的画面，没有原始画面可给。
        with self.assertRaisesRegex(
            camera_frames.CameraUnavailable, '不提供 d435i 的原始画面'
        ):
            self.feed('d435i').frame()
        self.rs.device.serial = 'unplugged'
        with self.assertRaisesRegex(
            camera_frames.CameraUnavailable, '没有找到唯一的指定相机'
        ):
            self.feed().frame()
        self.assertEqual(self.rs.started, 0)

    def test_keep_request_is_validated(self):
        def post(body):
            connection = http.client.HTTPConnection(
                *self.server.server_address, timeout=15
            )
            try:
                connection.request(
                    'POST', '/api/cameras/d405/keep', body=body
                )
                response = connection.getresponse()
                return response.status, json.loads(response.read())
            finally:
                connection.close()

        for body in (b'{}', b'{"keep_s": 0}', b'{"keep_s": 61}',
                     b'{"hold_s": 5}', b'{"keep_s": "5"}'):
            with self.subTest(body=body):
                self.assertEqual(post(body)[0], 400)
        self.assertEqual(self.rs.started, 0)
        code, status = post(b'{"keep_s": 5}')
        self.assertEqual(code, 200)
        self.assertEqual(status['state'], 'streaming')
        self.assertTrue(status['shared_frames']['path'].endswith(
            'saydo-camera-d405'
        ))

    def test_stopped_service_is_reported(self):
        feed = self.feed()
        feed.frame()
        self.cleanup()
        self.cleanup = lambda: None
        with self.assertRaisesRegex(
            camera_frames.CameraUnavailable, '连不上相机服务'
        ):
            feed.frame(timeout_s=2.)


if __name__ == '__main__':
    unittest.main()
