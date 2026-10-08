"""Object-relative grasp geometry and read-only teaching; no real hardware."""

import copy
import json
import math
import pathlib
import tempfile
import time
import types
import unittest
from unittest import mock

import numpy as np

from saydo.geometry import grasp_geometry
from saydo.manipulation import grasp_control
from tests.manipulation import test_grasp


class AdaptiveGraspTests(unittest.TestCase):
    def config(self):
        config = test_grasp.GraspTests().config()
        config['grasp_profile'].update(
            orientation='follow_object',
            object_R_tcp=None,
            approach_axis_tcp=[0, 0, 1],
        )
        return config

    def test_object_rotation_rotates_point_orientation_and_approach(self):
        config = self.config()
        # Object turns 90 degrees about Z; taught tool points along object X.
        obj = np.eye(4)
        obj[:3, :3] = [[0, -1, 0], [1, 0, 0], [0, 0, 1]]
        obj[:3, 3] = [.1, 0, .2]
        rotation = [[0, 0, 1], [0, 1, 0], [-1, 0, 0]]
        profile = {
            **config['grasp_profile'],
            'object_R_tcp': rotation,
            'grasp_point_object_m': [.02, 0, 0],
        }
        plan = grasp_geometry.plan_grasp(
            [0, 0, 200000, 0, 0, 0], obj, profile, config
        )
        np.testing.assert_allclose(
            plan['target_position_m'], [.1, .02, .2], atol=1e-9
        )
        np.testing.assert_allclose(
            plan['approach_direction_base'], [0, 1, 0], atol=1e-9
        )
        self.assertEqual(
            [w['name'] for w in plan['waypoints']],
            ['pregrasp', 'grasp', 'lift'],
        )
        np.testing.assert_allclose(
            plan['waypoints'][0]['position_m'], [.1, -.03, .2], atol=1e-9
        )
        np.testing.assert_allclose(
            plan['waypoints'][2]['position_m'], [.1, .02, .3], atol=1e-9
        )
        expected = obj[:3, :3] @ rotation
        for waypoint in plan['waypoints']:
            self.assertLess(
                grasp_geometry.rotation_error(
                    grasp_geometry.controller_pose(
                        waypoint['controller_pose']
                    )[:3, :3],
                    expected,
                ),
                .002,
            )

    def test_missing_or_invalid_orientation_cannot_silently_keep_current(self):
        config = self.config()
        obj = np.eye(4)
        obj[2, 3] = .2
        for rotation in (
            None, [[1, 0], [0, 1]], [[1, 0, 0], [0, 1, 0], [0, 0, -1]]
        ):
            profile = {**config['grasp_profile'], 'object_R_tcp': rotation}
            with self.assertRaises(ValueError):
                grasp_geometry.plan_grasp(
                    [0, 0, 200000, 0, 0, 0], obj, profile, config
                )
        profile = {
            **config['grasp_profile'],
            'object_R_tcp': np.eye(3).tolist(),
            'approach_axis_tcp': [0, 0, 0],
        }
        with self.assertRaises(ValueError):
            grasp_geometry.plan_grasp(
                [0, 0, 200000, 0, 0, 0], obj, profile, config
            )

    def test_pose_conversion_roundtrip_including_gimbal_lock(self):
        rng = np.random.default_rng(9)
        angles = (
            list(rng.uniform(-3000, 3000, (100, 3)))
            + [[400, math.pi * 500, 700], [400, -math.pi * 500, 700]]
        )
        for angles_raw in angles:
            T = grasp_geometry.controller_pose(
                [100000, 200000, 300000, *angles_raw]
            )
            raw = grasp_geometry.pose_values(T) * [1e6, 1e6, 1e6, 1e3, 1e3, 1e3]
            np.testing.assert_allclose(
                grasp_geometry.controller_pose(raw), T, atol=1e-8
            )

    def test_teaching_saves_relative_pose_and_rejects_changed_model_or_tool(
        self
    ):
        with tempfile.TemporaryDirectory() as directory:
            root = pathlib.Path(directory)
            cfg = root / 'config.json'
            config = self.config()
            cfg.write_text(json.dumps(config))
            camera_object = np.eye(4)
            camera_object[:3, 3] = [.1, 0, 0]
            observation = {
                'object_id': 'cup',
                'model_sha256': 'model-a',
                'time': time.time(),
                'pose': camera_object.tolist(),
                'K': np.diag([.5, .5, 1]).tolist(),
            }
            arm = {'joint': [0] * 7, 'pose': [0, 0, 200000, 0, 0, 0]}
            raw = {'tool_name': 'grip_tcp', 'pose': [0] * 6}
            tool = {
                'flange_T_grip_tcp': np.eye(4).tolist(),
                'controller_readback': {'tool_response_raw': raw},
            }
            handeye = {
                'camera': {'K': np.eye(3).tolist()},
                'flange_T_color_optical_camera': np.eye(4).tolist(),
            }
            tracker = types.SimpleNamespace(
                grasp_observation=mock.Mock(return_value=observation)
            )
            service = grasp_control.GraspController(root, tracker, cfg)
            robot = mock.Mock()
            robot.query.side_effect = (
                lambda q: raw if q == 'get_current_tool_frame' else {
                    'frame_name': 'Base', 'pose': [0] * 6
                }
            )
            with mock.patch(
                'saydo.robot.grasp_robot.RobotClient',
                return_value=robot,
            ) as client, mock.patch.object(
                service, 'paired_observation', return_value=(arm, observation)
            ), mock.patch.object(
                service, 'calibration', return_value=(handeye, tool)
            ):
                result = service.teach(
                    True,
                    gripper_parameters={
                        'force_target_raw': 10, 'open_position': 1000
                    },
                    expected_object_id='cup',
                )
                self.assertFalse(result['motion_sent'])
                self.assertEqual(
                    result['gripper_parameters'],
                    {'force_target_raw': 10, 'open_position': 1000},
                )
                self.assertEqual(
                    service.grasp_library().object('cup')['grasps'][0][
                        'overrides'
                    ],
                    result['gripper_parameters'],
                )
                client.assert_called_once_with('test', 8080, writes=False)
                np.testing.assert_allclose(
                    result['profile']['grasp_point_object_m'], [-.1, 0, 0]
                )
                robot.move_linear.assert_not_called()
                robot.move_gripper.assert_not_called()
                robot.set_register.assert_not_called()
                fresh = grasp_control.GraspController(root, tracker, cfg)
                self.assertEqual(
                    fresh.profile_for(config, observation, tool, handeye),
                    result['profile'],
                )
                with self.assertRaisesRegex(ValueError, '模型已变化'):
                    fresh.profile_for(
                        config,
                        {**observation, 'model_sha256': 'model-b'},
                        tool,
                        handeye,
                    )
                changed = copy.deepcopy(tool)
                changed[
                    'controller_readback'
                ]['tool_response_raw']['pose'][2] = 1
                with self.assertRaisesRegex(ValueError, '工具参数已变化'):
                    fresh.profile_for(config, observation, changed, handeye)
                changed_handeye = copy.deepcopy(handeye)
                changed_handeye['flange_T_color_optical_camera'][0][3] = .01
                with self.assertRaisesRegex(ValueError, '手眼标定已变化'):
                    fresh.profile_for(
                        config, observation, tool, changed_handeye
                    )
                saved = (
                    root / 'work/object_models/cup/grasp_profiles.json'
                ).read_bytes()
                with mock.patch.object(
                    service,
                    'check_cancel',
                    side_effect=ValueError('抓取已取消'),
                ):
                    with self.assertRaisesRegex(ValueError, '取消'):
                        service.teach()
                self.assertEqual(
                    (
                        root / 'work/object_models/cup/grasp_profiles.json'
                    ).read_bytes(),
                    saved,
                )
                result = service.teach(False)
                self.assertEqual(
                    result['profile']['grasp_point_object_m'], [0, 0, 0]
                )
                library = json.loads(
                    (
                        root / 'work/object_models/cup/grasp_profiles.json'
                    ).read_text()
                )
                self.assertEqual(len(library['configuration']['grasps']), 2)
                self.assertEqual(
                    library['configuration']['grasps'][0]['profile'][
                        'grasp_point_object_m'
                    ],
                    [-.1, 0, 0],
                )
                service.status['busy'] = True
                with self.assertRaisesRegex(ValueError, '已有抓取'):
                    service.teach()
            self.assertEqual(
                len(
                    list(
                        (
                            root / 'work/object_models/cup/teach_commands'
                        ).glob('*.json')
                    )
                ),
                3,
            )


if __name__ == '__main__':
    unittest.main()
