"""Replacement-camera validation without camera or robot access."""

import copy
import hashlib
import json
from pathlib import Path
import tempfile
from types import SimpleNamespace
import unittest

import yaml

from saydo.perception.camera_checks import select_camera, camera_binding, check_camera_observation
from saydo.manipulation.grasp_control import GraspController


class CameraChecksTests(unittest.TestCase):
    def setUp(self):
        self.camera = {
            'name': 'RealSense D435I',
            'serial': 'new',
            'native_size': [1280, 720],
            'image_size': [640, 360],
            'frame': 'color_optical_frame',
            'distortion_coefficients': [0.] * 5,
            'K': [[454.44, 0, 321.67], [0, 454.08, 191.60], [0, 0, 1]],
        }
        self.handeye = {
            'camera': {
                **self.camera,
                'serial': 'old',
                'size': [1280, 720],
                'K': [[911.75, 0, 645.93], [0, 910.18, 367.70], [0, 0, 1]],
            },
            'source': {'sha256': 'source'},
            'installation': {'id': 'mount'},
        }
        self.policy = {
            'enabled': True,
            'mount_unchanged_confirmed': True,
            'camera_model': 'RealSense D435I',
            'handeye_source_sha256': 'source',
            'installation_id': 'mount',
        }

    def test_single_camera_selection_and_explicit_selection(self):
        self.assertEqual(select_camera([self.camera]), self.camera)
        other = {**self.camera, 'serial': 'second'}
        with self.assertRaisesRegex(ValueError, '多台'):
            select_camera([self.camera, other])
        self.assertEqual(select_camera([self.camera, other], 'second'), other)
        with self.assertRaises(ValueError):
            select_camera([], 'missing')
        with self.assertRaises(ValueError):
            select_camera([{**self.camera, 'name': 'other model'}])

    def test_reuse_is_bound_to_model_installation_and_original_source(self):
        self.assertEqual(
            camera_binding(self.handeye, self.camera, self.policy)['mode'],
            'same_model_same_mount',
        )
        for change in (
            {'enabled': False},
            {'mount_unchanged_confirmed': False},
            {'installation_id': 'other'},
            {'handeye_source_sha256': 'other'},
        ):
            with self.subTest(change=change), self.assertRaises(ValueError):
                camera_binding(
                    self.handeye, self.camera, {**self.policy, **change}
                )
        with self.assertRaises(ValueError):
            camera_binding(self.handeye, self.camera, {})
        with self.assertRaisesRegex(ValueError, '型号'):
            camera_binding(
                self.handeye,
                {**self.camera, 'name': 'RealSense D455'},
                self.policy,
            )
        with self.assertRaises(ValueError):
            camera_binding(self.handeye, None, self.policy)
        self.assertEqual(
            camera_binding(
                self.handeye, {**self.camera, 'serial': 'old'}, {}
            )['mode'],
            'original_camera',
        )

    def test_replacement_accuracy_is_verified_only_by_explicit_policy_flag(
        self
    ):
        binding = camera_binding(self.handeye, self.camera, self.policy)
        self.assertIs(binding['replacement_accuracy_verified'], False)
        self.assertIsNone(binding['replacement_accuracy_basis'])
        confirmed = {
            **self.policy,
            'replacement_accuracy_verified': True,
            'accuracy_source': 'user_confirmation',
        }
        binding = camera_binding(self.handeye, self.camera, confirmed)
        self.assertIs(binding['replacement_accuracy_verified'], True)
        self.assertEqual(
            binding['replacement_accuracy_basis'], 'user_confirmation'
        )
        for value in ('true', 1, None):
            with self.subTest(value=value):
                binding = camera_binding(
                    self.handeye,
                    self.camera,
                    {**confirmed, 'replacement_accuracy_verified': value},
                )
                self.assertIs(binding['replacement_accuracy_verified'], False)
                self.assertIsNone(binding['replacement_accuracy_basis'])
        original = camera_binding(
            self.handeye, {**self.camera, 'serial': 'old'}, confirmed
        )
        self.assertIsNone(original['replacement_accuracy_verified'])
        self.assertIsNone(original['replacement_accuracy_basis'])

    def test_observation_uses_its_actual_camera_not_old_intrinsics(self):
        self.handeye['runtime_camera'] = self.camera
        observation = {'K': self.camera['K'], 'camera': self.camera}
        check_camera_observation(self.handeye, observation)
        for key, value in [
            ('serial', 'wrong'),
            ('K', [[455, 0, 320], [0, 455, 180], [0, 0, 1]]),
        ]:
            with self.subTest(key=key), self.assertRaises(ValueError):
                check_camera_observation(
                    self.handeye,
                    {**observation, 'camera': {**self.camera, key: value}},
                )
        for K in (
            [[455.875, 0, 322.965], [0, 455.09, 183.85], [0, 0, 1]],
            [[float('nan'), 0, 0], [0, 1, 0], [0, 0, 1]],
            [[-1, 0, 0], [0, 1, 0], [0, 0, 1]],
        ):
            with self.subTest(K=K), self.assertRaises(ValueError):
                check_camera_observation(self.handeye, {**observation, 'K': K})
        for key, value in [
            ('native_size', [640, 480]),
            ('frame', 'depth_frame'),
            ('image_size', [320, 180]),
            ('distortion_coefficients', [1, 0, 0, 0, 0]),
        ]:
            camera = {**self.camera, key: value}
            with self.subTest(key=key), self.assertRaises(ValueError):
                check_camera_observation(
                    {**self.handeye, 'runtime_camera': camera},
                    {'K': camera['K'], 'camera': camera},
                )

    def test_calibration_preserves_source_and_transform_and_requires_policy(
        self
    ):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            config_dir = root / 'config/calibration'
            config_dir.mkdir(parents=True)
            source = root / 'source.json'
            source.write_text('{}')
            digest = hashlib.sha256(source.read_bytes()).hexdigest()
            record = {'path': 'source.json', 'sha256': digest}
            transform = [
                [1, 0, 0, .07], [0, 1, 0, .06], [0, 0, 1, .01], [0, 0, 0, 1]
            ]
            handeye = {
                **self.handeye,
                'source': record,
                'acceptance_status': 'user_accepted_usable',
                'flange_T_color_optical_camera': transform,
            }
            h = config_dir / 'handeye.yaml'
            h.write_text(yaml.safe_dump(handeye))
            t = config_dir / 'tool.yaml'
            t.write_text(yaml.safe_dump({'controller_readback': record}))
            policy = config_dir / 'camera_reuse.yaml'
            policy.write_text(
                yaml.safe_dump({**self.policy, 'handeye_source_sha256': digest})
            )
            config = {
                'handeye_file': str(h.relative_to(root)),
                'tool_file': str(t.relative_to(root)),
                'nominal_tcp_user_accepted': True,
            }
            tracker = SimpleNamespace(state={'camera': self.camera})
            service = GraspController(root, tracker, 'unused')
            original = h.read_bytes()
            loaded, _ = service.calibration(config)
            self.assertEqual(loaded['flange_T_color_optical_camera'], transform)
            self.assertEqual(loaded['camera']['serial'], 'old')
            self.assertEqual(
                loaded['runtime_camera_binding']['active_serial'], 'new'
            )
            check_camera_observation(
                loaded, {'camera': self.camera, 'K': self.camera['K']}
            )
            self.assertEqual(h.read_bytes(), original)
            policy.unlink()
            with self.assertRaisesRegex(ValueError, '未配置'):
                service.calibration(config)
            source.write_text('{"changed":true}')
            with self.assertRaisesRegex(ValueError, '哈希'):
                service.calibration(config)


if __name__ == '__main__':
    unittest.main()
