"""2 臂手上相机的取图线程：相机库与控制器是替身，其余是真实代码。"""

import hashlib
import json
import pathlib
import tempfile
import threading
import time
import types
import unittest
from unittest import mock

import numpy as np
import yaml

from saydo.geometry import grasp_geometry
from saydo.manipulation import grasp_control
from saydo.objects import object_catalog
from saydo.robot import arms
from saydo.robot import grasp_robot
from saydo.scene import scene_features
from saydo.workbench import arm_camera
from tests.robot import fake_controller
from tests.workbench import fake_realsense

SERIAL = '344422070678'
MODEL = 'Intel RealSense D435I'


class ArmCameraTestCase(unittest.TestCase):
    def setUp(self):
        directory = tempfile.TemporaryDirectory()
        self.addCleanup(directory.cleanup)
        self.root = pathlib.Path(directory.name)
        self.rs = fake_realsense.FakeRealSense({SERIAL: MODEL})
        self.events = []
        # 跟踪器里与本测试有关的部分：定位模型在显卡上，属于系统边界。
        self.tracker = types.SimpleNamespace(
            models_root=self.root / 'models',
            state={'model_ready': True},
            stop=threading.Event(),
            event=lambda name, **values: self.events.append((name, values)),
            realsense_module=lambda: self.rs,
            localize_frozen_scene=mock.Mock(),
        )
        self.arm = arms.by_id('arm2')
        self.camera = arm_camera.ArmCamera(self.tracker, self.arm, SERIAL)
        patcher = mock.patch.object(arm_camera, 'RETRY_INTERVAL_S', .02)
        patcher.start()
        self.addCleanup(patcher.stop)

    def start(self):
        thread = threading.Thread(target=self.camera.run, daemon=True)
        thread.start()

        def stop():
            self.tracker.stop.set()
            thread.join(timeout=5)
            self.assertFalse(thread.is_alive())

        self.addCleanup(stop)
        return thread

    def wait_for(self, condition, message):
        deadline = time.monotonic() + 5
        while time.monotonic() < deadline:
            if condition():
                return
            time.sleep(.01)
        self.fail(message)

    def names(self):
        return [name for name, _ in self.events]


class FrameTests(ArmCameraTestCase):
    def test_frames_are_published_in_the_same_form_as_the_first_arm(self):
        self.start()
        self.wait_for(lambda: self.camera.latest is not None, '没有画面')
        with self.camera.lock:
            frame = dict(self.camera.latest)
            state = dict(self.camera.state)
        self.assertEqual(self.rs.started, [SERIAL])
        self.assertEqual(frame['bgr'].shape, (360, 640, 3))
        self.assertEqual(frame['depth'].shape, (360, 640))
        self.assertEqual(frame['depth'].dtype, np.float32)
        # 原始值 400 乘以每单位 1 毫米，即 0.4 米。
        self.assertAlmostEqual(float(frame['depth'][0, 0]), .4, places=5)
        # 图像缩小一半，内参随之减半。
        np.testing.assert_allclose(
            frame['K'], [[455, 0, 323], [0, 454, 184], [0, 0, 1]]
        )
        self.assertEqual(
            frame['camera'],
            {
                'serial': SERIAL,
                'name': MODEL,
                'native_size': [1280, 720],
                'image_size': [640, 360],
                'frame': 'color_optical_frame',
                'K': frame['K'].tolist(),
                'distortion_coefficients': [0.] * 5,
            },
        )
        self.assertLess(abs(time.time() - frame['time']), 1)
        self.assertEqual(state['camera_state'], 'streaming')
        self.assertEqual(state['camera'], frame['camera'])
        self.assertTrue(state['model_ready'])
        self.assertIsNone(state['error'])
        self.assertEqual(self.names(), ['arm_camera_connected'])
        self.assertEqual(self.events[0][1]['arm'], 'arm2')

    def test_missing_camera_is_reported_once_and_recovers_when_connected(self):
        self.rs.devices = {'405622073572': MODEL}
        self.start()
        self.wait_for(
            lambda: self.camera.state['camera_state'] == 'error', '没有报错'
        )
        self.assertIn(SERIAL, self.camera.state['error'])
        self.assertIn('405622073572', self.camera.state['error'])
        time.sleep(.15)
        # 重试了好几轮，同一条错误只记一次。
        self.assertEqual(self.names(), ['arm_camera_failed'])
        self.assertEqual(self.rs.started, [])
        self.rs.devices[SERIAL] = MODEL
        self.wait_for(lambda: self.camera.latest is not None, '没有恢复')
        self.assertIsNone(self.camera.state['error'])
        self.assertEqual(
            self.names(), ['arm_camera_failed', 'arm_camera_connected']
        )

    def test_dropout_clears_the_frame_but_does_not_stop_the_service(self):
        self.rs.fail_after = arm_camera.WARMUP_FRAMES + 3
        self.start()
        self.wait_for(
            lambda: self.camera.state['camera_state'] == 'error', '没有报错'
        )
        self.assertIsNone(self.camera.latest)
        self.assertIn("didn't arrive", self.camera.state['error'])
        self.assertFalse(self.tracker.stop.is_set())
        self.assertGreaterEqual(self.rs.stopped, 1)
        self.rs.fail_after = None
        self.wait_for(lambda: self.camera.latest is not None, '没有恢复')

    def test_live_tracking_operations_are_refused_with_the_reason(self):
        for name in (
            'grasp_observation',
            'switch_object',
            'automatic',
            'freeze',
            'initialize',
            'choose',
        ):
            with self.subTest(name=name), self.assertRaisesRegex(
                ValueError, '2臂.*场景快照.*不支持实时跟踪'
            ):
                getattr(self.camera, name)('anything')

    def test_scene_localization_uses_the_shared_models(self):
        self.tracker.localize_frozen_scene.return_value = ['rows']
        check, progress = mock.Mock(), mock.Mock()
        rows = self.camera.localize_frozen_scene(
            'frame', 'targets', check, progress
        )
        self.assertEqual(rows, ['rows'])
        self.tracker.localize_frozen_scene.assert_called_once_with(
            'frame', 'targets', check, progress
        )


class SceneCaptureTests(ArmCameraTestCase):
    """2 臂沿用 1 臂的手眼标定，用自己的相机完成一次场景采集。"""

    def setUp(self):
        super().setUp()
        self.fleet = fake_controller.Fleet()
        patcher = mock.patch.object(grasp_robot, 'RobotClient', self.fleet)
        patcher.start()
        self.addCleanup(patcher.stop)
        folder = self.tracker.models_root / 'tube'
        folder.mkdir(parents=True)
        (folder / 'model.obj').write_text('fixture')
        (folder / 'tracking.json').write_text(
            json.dumps({'name': 'tube', 'units': 'metres', 'pose_mode': '6d'})
        )
        self.flange_T_camera = np.eye(4)
        self.flange_T_camera[:3, 3] = [.075, .063, .012]
        self.flange_T_tcp = np.eye(4)
        self.flange_T_tcp[2, 3] = .2225
        source = self.write('work/handeye/result.json', 'arm1 solution')
        self.write(
            'config/calibration/handeye.yaml',
            yaml.safe_dump(
                {
                    'acceptance_status': 'user_accepted_usable',
                    'installation': {
                        'id': 'mount-a', 'controller_firmware': 'V1.7.0'
                    },
                    'camera': {
                        'serial': '135122077750',
                        'name': MODEL,
                        'size': [1280, 720],
                        'frame': 'color_optical_frame',
                    },
                    'flange_T_color_optical_camera': (
                        self.flange_T_camera.tolist()
                    ),
                    'source': {
                        'path': 'work/handeye/result.json', 'sha256': source
                    },
                }
            ),
        )
        readback = self.write('work/calibration/arm2_readback.json', '{}')
        config_path = fake_controller.write_arm_config(
            self.root, self.arm, '10.0.0.2', [0] * 7
        )
        tool_file = json.loads(config_path.read_text())['tool_file']
        self.write(
            tool_file,
            yaml.safe_dump(
                {
                    'flange_T_grip_tcp': self.flange_T_tcp.tolist(),
                    'controller_readback': {
                        'path': 'work/calibration/arm2_readback.json',
                        'sha256': readback,
                        'tool_response_raw': fake_controller.TOOL_RESPONSE,
                    },
                }
            ),
        )
        config = json.loads(config_path.read_text())
        config.update(
            handeye_file='config/calibration/handeye.yaml',
            camera_reuse_file='config/calibration/arm2_camera_reuse.yaml',
            camera_serial=SERIAL,
            nominal_tcp_user_accepted=True,
        )
        config_path.write_text(json.dumps(config))
        self.policy = {
            'enabled': True,
            'mount_unchanged_confirmed': True,
            'camera_model': MODEL,
            'installation_id': 'mount-a',
            'handeye_source_sha256': source,
        }
        self.service = grasp_control.GraspController(
            self.root, self.camera, config_path, 'execute', self.arm
        )
        self.camera.grasp = self.service
        self.camera_T_object = np.eye(4)
        self.camera_T_object[:3, 3] = [.02, -.01, .35]
        self.tracker.localize_frozen_scene.side_effect = self.localize
        self.frames = []

    def write(self, relative, text):
        path = self.root / relative
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_text(text)
        return hashlib.sha256(text.encode()).hexdigest()

    def localize(self, frame, targets, check, progress):
        self.frames.append(frame)
        models = self.tracker.models_root
        rows = []
        for target in targets:
            info = object_catalog.select_object(models, target['object_id'])
            rows.append(
                {
                    **target,
                    'object_name': target['object_id'],
                    'camera_T_object': self.camera_T_object.tolist(),
                    'quality': {'good': True},
                    'model_sha256': hashlib.sha256(
                        pathlib.Path(info['mesh']).read_bytes()
                    ).hexdigest(),
                    'pose_mode': '6d',
                    'pose_mode_revision': info['pose_mode_revision'],
                    **scene_features.object_features(
                        models, target['object_id']
                    ),
                }
            )
        return rows

    def capture_scene(self):
        self.start()
        self.wait_for(lambda: self.camera.latest is not None, '没有画面')
        accepted = self.service.start_scene_capture(
            [
                {
                    'instance_id': 'tube-1',
                    'object_id': 'tube',
                    'role': 'source',
                    'box': [10, 10, 80, 90],
                }
            ],
            True,
            'request-0001',
        )
        self.service.thread.join(timeout=10)
        self.assertFalse(self.service.thread.is_alive())
        return accepted['scene_id']

    def test_scene_is_captured_from_this_arms_camera_into_its_own_store(self):
        self.write(
            'config/calibration/arm2_camera_reuse.yaml',
            yaml.safe_dump(self.policy),
        )
        scene_id = self.capture_scene()
        self.assertEqual(
            self.service.status['state'],
            'scene_ready',
            self.service.status.get('error'),
        )
        scene = self.service.scenes().require(scene_id)
        self.assertEqual(scene['camera']['serial'], SERIAL)
        self.assertEqual(self.frames[0]['bgr'].shape, (360, 640, 3))
        self.assertEqual(self.frames[0]['camera']['serial'], SERIAL)
        # 基座 ← 夹爪参考点（控制器位姿）← 法兰 ← 相机（1 臂的手眼）← 物品。
        arm = self.fleet.controller('10.0.0.2').query(
            'get_current_arm_state'
        )['arm_state']
        expected = (
            grasp_geometry.controller_pose(arm['pose'])
            @ np.linalg.inv(self.flange_T_tcp)
            @ self.flange_T_camera
            @ self.camera_T_object
        )
        np.testing.assert_allclose(
            scene['instances'][0]['base_T_object'], expected, atol=1e-9
        )
        self.assertTrue(
            (self.root / 'work/grasp_arm2/scenes' / scene_id).is_dir()
        )
        self.assertFalse((self.root / 'work/grasp/scenes').exists())
        # 采集只读：没有向控制器发过任何写入类的连接。
        self.assertEqual(
            set(self.fleet.controller('10.0.0.2').write_flags), {False}
        )

    def test_without_the_reuse_policy_another_arms_calibration_is_refused(
        self,
    ):
        self.capture_scene()
        self.assertEqual(self.service.status['state'], 'failed')
        self.assertIn('相机与手眼标定不匹配', self.service.status['error'])
        self.assertEqual(self.frames, [])


if __name__ == '__main__':
    unittest.main()
