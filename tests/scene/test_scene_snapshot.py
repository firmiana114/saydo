import copy
import hashlib
import json
from pathlib import Path
import tempfile
import threading
import time
from types import SimpleNamespace
import unittest
from unittest.mock import Mock, patch
import numpy as np
from saydo.manipulation.grasp_control import GraspController
from saydo.geometry.grasp_geometry import controller_pose
from saydo.scene.scene_snapshot import SceneStore, arm_still
from saydo.scene.scene_features import object_features
from saydo.objects.object_catalog import select_object
from saydo.contracts.scene_contract import checked_targets
from saydo.contracts.motion_contract import checked_goal
from tests.manipulation import test_grasp

TARGETS = [
    {
        'instance_id': 'tube',
        'object_id': 'source',
        'role': 'source',
        'box': [10, 10, 80, 90],
    },
    {
        'instance_id': 'rack',
        'object_id': 'reference',
        'role': 'reference',
        'box': None,
    },
]


class SceneTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.addCleanup(self.temp.cleanup)
        self.root = Path(self.temp.name)
        self.models = self.root / 'models'
        for ident in ('source', 'reference'):
            folder = self.models / ident
            folder.mkdir(parents=True)
            (folder / 'model.obj').write_text('fixture')
            (folder / 'tracking.json').write_text(
                json.dumps(
                    {'name': ident, 'units': 'metres', 'pose_mode': '6d'}
                )
            )
        self.config = test_grasp.GraspTests().config()
        self.config.update(tool_file='tool.yaml', handeye_file='handeye.yaml')
        self.path = self.root / 'trial.json'
        self.path.write_text(json.dumps(self.config))
        (self.root / 'tool.yaml').write_text('{}')
        (self.root / 'handeye.yaml').write_text('{}')
        self.arm = {
            'joint': [0] * 7, 'pose': [100000, 0, 300000, 0, 0, 0], 'err': [0]
        }
        camera = {'serial': 'fixture'}
        self.tracker = SimpleNamespace(
            lock=threading.RLock(),
            models_root=self.models,
            generation=1,
            command=None,
            state={'model_ready': True, 'state': 'tracking', 'camera': camera},
            latest={
                'time': time.time(),
                'sequence': 12,
                'camera': camera,
                'K': np.eye(3),
                'bgr': np.zeros((360, 640, 3), dtype=np.uint8),
                'depth': np.ones((360, 640)),
            },
        )
        self.service = GraspController(self.root, self.tracker, self.path)
        self.robot = Mock()
        self.robot.query.side_effect = lambda name: {
            'get_current_arm_state': {'arm_state': copy.deepcopy(self.arm)},
            'get_current_tool_frame': {'frame_name': 'grip_tcp'},
            'get_current_work_frame': {'frame_name': 'Base', 'pose': [0] * 6},
        }[name]
        self.tool = {
            'flange_T_grip_tcp': np.eye(4).tolist(),
            'controller_readback': {
                'tool_response_raw': {'frame_name': 'grip_tcp'}
            },
        }
        self.service.calibration = Mock(
            return_value=(
                {'flange_T_color_optical_camera': np.eye(4).tolist()}, self.tool
            )
        )
        self.service.stationary_scene_arm = Mock(
            side_effect=lambda: copy.deepcopy(self.arm)
        )
        self.service.paired_observation = Mock(
            side_effect=lambda o: (copy.deepcopy(self.arm), o)
        )
        self.frames = []

        def localize(frame, targets, check, progress):
            check()
            self.frames.append(frame)
            result = []
            for i, t in enumerate(targets):
                info = select_object(self.models, t['object_id'])
                pose = np.eye(4)
                pose[0, 3] = i * .1
                pose[2, 3] = .2
                result.append(
                    {
                        **t,
                        'object_name': t['object_id'],
                        'camera_T_object': pose.tolist(),
                        'quality': {'good': True},
                        'model_sha256': hashlib.sha256(
                            Path(info['mesh']).read_bytes()
                        ).hexdigest(),
                        'pose_mode': '6d',
                        'pose_mode_revision': info['pose_mode_revision'],
                        **object_features(self.models, t['object_id']),
                    }
                )
            check()
            return result

        self.tracker.localize_frozen_scene = Mock(side_effect=localize)

    def capture(self):
        with patch(
            'saydo.scene.scene_control.RobotClient', return_value=self.robot
        ) as client, patch(
            'saydo.scene.scene_control.check_camera_observation'
        ):
            self.service.start_scene_capture(TARGETS, True, 'scene-test')
            self.service.thread.join(4)
            self.assertFalse(self.service.thread.is_alive())
            client.assert_called_once_with('test', 8080, writes=False)
        return self.service.scenes().get(self.service.status['scene_id'])

    def test_capture_same_frame_freezes_base_pose_and_never_writes_robot(self):
        scene = self.capture()
        self.assertEqual(scene['state'], 'ready', scene.get('error'))
        self.assertEqual(len(self.frames), 1)
        self.assertEqual(scene['frame_sequence'], 12)
        self.assertEqual(self.service.status['state'], 'scene_ready')
        self.robot.move_gripper.assert_not_called()
        self.robot.move_linear.assert_not_called()
        self.robot.stop.assert_not_called()
        before = scene['instances'][0]['base_T_object']
        self.arm['pose'][0] += 200000
        obs = self.service.scene_observation(
            scene['scene_id'], 'tube', 'source'
        )
        self.assertEqual(obs['base_T_object'], before)
        self.assertEqual(obs['base_T_object'][0][3], .1)
        self.service.run = Mock()
        self.tracker.grasp_observation = Mock(
            side_effect=AssertionError('must not reobserve')
        )
        self.service.start(
            preview=True,
            expected_object_id='source',
            scene_id=scene['scene_id'],
            instance_id='tube',
        )
        self.service.thread.join(2)
        self.assertEqual(
            self.service.run.call_args.args[0]['base_T_object'], before
        )

    def test_incomplete_scene_blocks_grasp_and_legacy_bypass(self):
        self.tracker.localize_frozen_scene.side_effect = ValueError(
            'reference failed'
        )
        scene = self.capture()
        self.assertEqual(scene['state'], 'failed')
        with self.assertRaises(ValueError):
            self.service.scene_observation(scene['scene_id'], 'tube', 'source')
        with self.assertRaisesRegex(ValueError, '场景'):
            self.service.start(expected_object_id='source')
        self.robot.move_gripper.assert_not_called()

    def test_robot_motion_during_capture_invalidates_every_target(self):
        def moved(frame, targets, check, progress):
            self.arm['pose'][0] += 10000
            check()

        self.tracker.localize_frozen_scene.side_effect = moved
        scene = self.capture()
        self.assertEqual(scene['state'], 'failed')
        self.assertIn('移动', scene['error'])

    def test_binding_features_model_restart_expiry_and_supersession(self):
        scene = self.capture()
        ident = scene['scene_id']
        (self.root / 'tool.yaml').write_text('changed')
        with self.assertRaisesRegex(ValueError, '标定'):
            self.service.validate_scene(ident)
        (self.root / 'tool.yaml').write_text('{}')
        (self.models / 'source/model.obj').write_text('changed')
        with self.assertRaisesRegex(ValueError, '模型'):
            self.service.validate_scene(ident)
        with self.assertRaisesRegex(ValueError, '重启'):
            SceneStore(self.root).require(ident)
        scene['expires_at'] = 0
        self.service.scenes().save(scene)
        with self.assertRaisesRegex(ValueError, '30分钟'):
            self.service.scenes().require(ident)
        self.service.scenes().begin(TARGETS)
        with self.assertRaises(ValueError):
            self.service.scenes().require(ident)

    def test_feature_frame_and_moved_instance_invalidation(self):
        pose = np.eye(4)
        pose[2, 3] = .085
        (self.models / 'reference/features.json').write_text(
            json.dumps(
                {
                    'units': 'metres',
                    'features': [
                        {'id': 'entry', 'object_T_feature': pose.tolist()}
                    ],
                }
            )
        )
        scene = self.capture()
        goal = checked_goal(
            {
                'reference_frame': 'scene_object',
                'scene_id': scene['scene_id'],
                'reference_instance_id': 'rack',
                'reference_feature_id': 'entry',
                'target_position_m': [0, 0, 0],
            }
        )
        frame, context = self.service.scene_reference(goal)
        self.assertAlmostEqual(frame[2, 3], .585)
        self.service.update_scene_instance(
            {'scene_id': scene['scene_id'], 'instance_id': 'tube'}, 'held'
        )
        with self.assertRaises(ValueError):
            self.service.scene_observation(scene['scene_id'], 'tube', 'source')
        self.service.update_scene_instance(
            {'scene_id': scene['scene_id'], 'instance_id': 'tube'},
            'placed_unverified',
            np.eye(4),
        )
        self.assertIsNone(self.service.scenes().active)
        self.assertFalse(
            self.service.scene_snapshot(scene['scene_id'])[
                'valid_for_execution'
            ]
        )

    def test_target_validation_and_capture_while_holding(self):
        for rows in ([], TARGETS * 2, [{**TARGETS[0], 'box': [0, 0, 641, 20]}]):
            with self.assertRaises(ValueError):
                checked_targets(rows)
        self.service.held_object = {'hold_id': 'held'}
        with self.assertRaisesRegex(ValueError, '抓取前'):
            self.service.start_scene_capture(TARGETS, True, 'held-test')
        self.robot.query.assert_not_called()

    def test_generic_features_preserve_uncalibrated_geometry(self):
        pose = np.eye(4)
        pose[2, 3] = .085
        data = {
            'units': 'metres',
            'notes': ['orientation ambiguity'],
            'features': [
                {
                    'id': 'target',
                    'object_T_feature': pose.tolist(),
                    'diameter_m': .02,
                }
            ],
        }
        (self.models / 'reference/features.json').write_text(json.dumps(data))
        features = object_features(self.models, 'reference')
        self.assertEqual(len(features['features']), 1)
        self.assertFalse(features['features'][0]['physical_fit_verified'])
        self.assertIn('ambiguity', features['feature_notes'][0])
        self.assertEqual(features['features'][0]['diameter_m'], .02)


if __name__ == '__main__':
    unittest.main()
