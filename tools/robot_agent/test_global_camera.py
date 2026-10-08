import base64
import json
import io
from pathlib import Path
import subprocess
import tempfile
import time
import unittest
from unittest.mock import Mock, patch

import global_camera as camera


def image_result():
    return {
        'camera_serial': '344422070678',
        'camera_role': 'movable_global',
        'camera_name': 'RealSense D435I',
        'captured_at': time.time(),
        'mime_type': 'image/jpeg',
        'image_base64': base64.b64encode(b'\xff\xd8fixture').decode(),
    }


class GlobalCameraTests(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        self.addCleanup(self.tmp.cleanup)
        root = Path(self.tmp.name)
        interpreter = root / 'python'
        interpreter.touch()
        self.config = root / 'global.json'
        self.settings = {
            'serial': '344422070678',
            'eye_in_hand_serial': '405622073572',
            'python': str(interpreter),
            'sdk_path': str(root),
        }
        self.config.write_text(json.dumps(self.settings))

    def test_fresh_image_identity_and_uncalibrated_contract(self):
        with patch.object(
            camera.subprocess,
            'run',
            return_value=Mock(returncode=0, stdout=json.dumps(image_result())),
        ) as run:
            result = camera.observe_global(self.config)
        args = run.call_args.args[0]
        self.assertEqual(
            args[-4:],
            ['--capture', '344422070678', '--model', 'RealSense D435I'],
        )
        self.assertEqual(run.call_args.kwargs['timeout'], 12)
        self.assertNotIn('shell', run.call_args.kwargs)
        self.assertFalse(result['usable_for_robot_target'])
        self.assertFalse(result['calibrated_to_robot_base'])
        self.assertTrue(result['observation_only'])
        self.assertFalse(result['motion_sent'])

    def test_missing_configuration_and_same_camera_never_start_worker(self):
        with patch.object(camera.subprocess, 'run') as run:
            with self.assertRaises(camera.CameraUnavailable):
                camera.observe_global(self.config.with_name('missing'))
            self.config.write_text(
                json.dumps(
                    {
                        **self.settings,
                        'serial': self.settings['eye_in_hand_serial'],
                    }
                )
            )
            with self.assertRaises(camera.CameraUnavailable):
                camera.observe_global(self.config)
            run.assert_not_called()

    def test_d405_binding_and_reported_model_are_preserved(self):
        self.config.write_text(
            json.dumps(
                {
                    **self.settings,
                    'serial': '218622270784',
                    'camera_name': 'RealSense D405',
                }
            )
        )
        frame = {
            **image_result(),
            'camera_serial': '218622270784',
            'camera_name': 'RealSense D405',
        }
        with patch.object(
            camera.subprocess,
            'run',
            return_value=Mock(returncode=0, stdout=json.dumps(frame)),
        ) as run:
            result = camera.observe_global(self.config)
        self.assertEqual(result['camera_name'], 'RealSense D405')
        self.assertEqual(
            run.call_args.args[0][-4:],
            ['--capture', '218622270784', '--model', 'RealSense D405'],
        )

    def test_worker_rejects_wrong_model_before_opening(self):
        rs = Mock()
        device = Mock()
        device.get_info.side_effect = lambda key: '218622270784' if key == rs.camera_info.serial_number else 'RealSense D435I'
        rs.context.return_value.query_devices.return_value = [device]
        with patch.dict(
            'sys.modules', {'pyrealsense2': rs, 'cv2': Mock(), 'numpy': Mock()}
        ), patch.object(camera, 'ROOT', Path(self.tmp.name)), patch.object(
            camera, 'monitor_frame', return_value=None
        ):
            with self.assertRaisesRegex(ValueError, '不会改用'):
                camera.capture('218622270784', 'RealSense D405')
        rs.pipeline.assert_not_called()

    def test_stale_wrong_camera_and_invalid_images_rejected(self):
        for changes in (
            {'captured_at': time.time() - 3},
            {'captured_at': time.time() + 3},
            {'camera_serial': '405622073572'},
            {'camera_role': 'eye_in_hand'},
            {'camera_name': 'RealSense D405'},
            {'image_base64': 'not-base64'},
            {'mime_type': 'image/png'},
        ):
            with self.subTest(changes=changes), patch.object(
                camera.subprocess,
                'run',
                return_value=Mock(
                    returncode=0,
                    stdout=json.dumps({**image_result(), **changes}),
                ),
            ):
                with self.assertRaises(camera.CameraUnavailable):
                    camera.observe_global(self.config)

    def test_timeout_and_device_errors_have_no_retry_or_fallback(self):
        with patch.object(
            camera.subprocess,
            'run',
            side_effect=subprocess.TimeoutExpired('capture', 12),
        ) as run:
            with self.assertRaises(camera.CameraUnavailable):
                camera.observe_global(self.config)
            run.assert_called_once()
        with patch.object(
            camera.subprocess,
            'run',
            return_value=Mock(returncode=1, stderr='device busy'),
        ) as run:
            with self.assertRaisesRegex(
                camera.CameraUnavailable, 'device busy'
            ):
                camera.observe_global(self.config)
            run.assert_called_once()

    def test_worker_selects_only_fixed_color_stream_and_releases_on_error(self):
        rs = Mock()
        device = Mock()
        device.get_info.side_effect = lambda key: '344422070678' if key == rs.camera_info.serial_number else 'RealSense D435I'
        rs.context.return_value.query_devices.return_value = [device]
        rs.pipeline.return_value.start.return_value.get_device.return_value = (
            device
        )
        rs.pipeline.return_value.wait_for_frames.side_effect = RuntimeError(
            'no frame'
        )
        with patch.dict(
            'sys.modules', {'pyrealsense2': rs, 'cv2': Mock(), 'numpy': Mock()}
        ), patch.object(camera, 'ROOT', Path(self.tmp.name)):
            with self.assertRaisesRegex(RuntimeError, 'no frame'):
                camera.capture('344422070678')
        rs.config.return_value.enable_device.assert_called_once_with(
            '344422070678'
        )
        rs.config.return_value.enable_stream.assert_called_once_with(
            rs.stream.color, 1280, 720, rs.format.bgr8, 15
        )
        rs.pipeline.return_value.stop.assert_called_once()

    def test_worker_never_uses_available_eye_in_hand_when_fixed_missing(self):
        rs = Mock()
        device = Mock()
        device.get_info.return_value = '405622073572'
        rs.context.return_value.query_devices.return_value = [device]
        with patch.dict(
            'sys.modules', {'pyrealsense2': rs, 'cv2': Mock(), 'numpy': Mock()}
        ), patch.object(camera, 'ROOT', Path(self.tmp.name)):
            with self.assertRaisesRegex(ValueError, '不会改用'):
                camera.capture('344422070678')
        rs.pipeline.assert_not_called()


class SharedMonitorTests(unittest.TestCase):
    def frame(self, **changes):
        return {
            **image_result(),
            'camera_serial': '218622270784',
            'camera_name': 'RealSense D405',
            **changes,
        }

    def test_shared_frame_is_used_without_opening_hardware(self):
        result = self.frame()
        opener = Mock()
        opener.open.return_value = io.BytesIO(json.dumps(result).encode())
        with patch.object(
            camera.urllib.request, 'build_opener', return_value=opener
        ):
            self.assertEqual(
                camera.capture('218622270784', 'RealSense D405'), result
            )
        opener.open.assert_called_once_with(
            'http://127.0.0.1:8088/api/cameras/d405/frame', timeout=3
        )

    def test_shared_wrong_identity_stale_and_invalid_image_rejected(self):
        for changes in (
            {'camera_serial': 'other'},
            {'camera_role': 'eye_in_hand'},
            {'captured_at': time.time() - 3},
            {'image_base64': 'bad'},
            {'mime_type': 'image/png'},
        ):
            opener = Mock()
            opener.open.return_value = io.BytesIO(
                json.dumps(self.frame(**changes)).encode()
            )
            with self.subTest(changes=changes), patch.object(
                camera.urllib.request, 'build_opener', return_value=opener
            ):
                with self.assertRaises(ValueError):
                    camera.monitor_frame('218622270784', 'RealSense D405')

    def test_idle_or_absent_monitor_allows_original_capture(self):
        for error in [
            camera.urllib.error.URLError('offline'),
            camera.urllib.error.HTTPError(
                'local', 409, 'idle', {}, io.BytesIO(b'')
            ),
        ]:
            opener = Mock()
            opener.open.side_effect = error
            with patch.object(
                camera.urllib.request, 'build_opener', return_value=opener
            ):
                self.assertIsNone(
                    camera.monitor_frame('218622270784', 'RealSense D405')
                )

    def test_other_model_does_not_consult_d405_preview(self):
        with patch.object(camera.urllib.request, 'build_opener') as build:
            self.assertIsNone(
                camera.monitor_frame('344422070678', 'RealSense D435I')
            )
        build.assert_not_called()


if __name__ == '__main__':
    unittest.main()
