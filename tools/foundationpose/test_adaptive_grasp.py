"""Object-relative grasp geometry and read-only teaching; no real hardware."""
import copy
import json
import math
from pathlib import Path
import tempfile
import time
from types import SimpleNamespace
import unittest
from unittest.mock import Mock, patch

import numpy as np

from grasp_geometry import controller_pose, pose_values, rotation_error, sample_motion, plan_grasp
from grasp_control import GraspController
import test_grasp


class AdaptiveGraspTests(unittest.TestCase):
    def test_ik_checker_samples_rotation_only_path(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory);(root/'checker').touch()
            service = GraspController(root, None, root/'unused')
            arm = {'pose':[0,0,200000,0,0,0], 'joint':[0]*7}
            plan = {'waypoints':[{'controller_pose':[0,0,200000,0,0,1571]}]}
            tool = {'controller_readback':{'tool_response_raw':{'pose':[0]*6}}}
            def check(*args, **kwargs):
                lines = kwargs['input'].splitlines()
                count = int(lines[2])
                self.assertGreaterEqual(count, 30)
                self.assertEqual(count, len(lines)-3)
                return SimpleNamespace(returncode=0, stdout='FK 0 0 0.2 0 0 0\n' + ('IK 0 0 0 0 0 0 0 0 0\n' * count))
            with patch('grasp_control.subprocess.run', side_effect=check):
                self.assertTrue(service.check_ik(plan, arm, tool, {'ik_checker':'checker'})['ok'])

    def config(self):
        config = test_grasp.GraspTests().config()
        config['grasp_profile'].update(orientation='follow_object', object_R_tcp=None,
                                      approach_axis_tcp=[0, 0, 1])
        return config

    def test_object_rotation_rotates_point_orientation_and_approach(self):
        config = self.config()
        # Object turns 90 degrees about Z; taught tool points along object X.
        obj = np.eye(4);obj[:3, :3] = [[0,-1,0],[1,0,0],[0,0,1]];obj[:3, 3] = [.1,0,.2]
        rotation = [[0,0,1],[0,1,0],[-1,0,0]]
        profile = {**config['grasp_profile'], 'object_R_tcp': rotation, 'grasp_point_object_m': [.02,0,0]}
        plan = plan_grasp([0,0,200000,0,0,0], obj, profile, config)
        np.testing.assert_allclose(plan['target_position_m'], [.1,.02,.2], atol=1e-9)
        np.testing.assert_allclose(plan['approach_direction_base'], [0,1,0], atol=1e-9)
        self.assertEqual([w['name'] for w in plan['waypoints']], ['pregrasp','grasp','lift'])
        np.testing.assert_allclose(plan['waypoints'][0]['position_m'], [.1,-.03,.2], atol=1e-9)
        np.testing.assert_allclose(plan['waypoints'][2]['position_m'], [.1,.02,.3], atol=1e-9)
        expected = obj[:3,:3] @ rotation
        for waypoint in plan['waypoints']:
            self.assertLess(rotation_error(controller_pose(waypoint['controller_pose'])[:3,:3], expected), .002)

    def test_missing_or_invalid_orientation_cannot_silently_keep_current(self):
        config = self.config();obj = np.eye(4);obj[2,3] = .2
        for rotation in (None, [[1,0],[0,1]], [[1,0,0],[0,1,0],[0,0,-1]]):
            profile = {**config['grasp_profile'], 'object_R_tcp': rotation}
            with self.assertRaises(ValueError):plan_grasp([0,0,200000,0,0,0], obj, profile, config)
        profile = {**config['grasp_profile'], 'object_R_tcp': np.eye(3).tolist(), 'approach_axis_tcp': [0,0,0]}
        with self.assertRaises(ValueError):plan_grasp([0,0,200000,0,0,0], obj, profile, config)

    def test_pose_conversion_roundtrip_including_gimbal_lock(self):
        rng = np.random.default_rng(9)
        angles = list(rng.uniform(-3000,3000,(100,3))) + [[400, math.pi*500, 700], [400,-math.pi*500,700]]
        for angles_raw in angles:
            T = controller_pose([100000,200000,300000,*angles_raw])
            raw = pose_values(T) * [1e6,1e6,1e6,1e3,1e3,1e3]
            np.testing.assert_allclose(controller_pose(raw), T, atol=1e-8)

    def test_rotation_only_sampling_and_wrap_take_short_path(self):
        start = [0,0,200000,0,0,0]
        end = [0,0,200000,0,0,3142]
        samples = sample_motion(start,end)
        self.assertGreaterEqual(len(samples),60)
        last = controller_pose(start)
        for row in samples:
            now = controller_pose(np.array(row)*[1e6,1e6,1e6,1e3,1e3,1e3])
            self.assertLessEqual(rotation_error(last[:3,:3],now[:3,:3]), math.radians(3)+1e-7)
            last = now
        self.assertLess(rotation_error(last[:3,:3],controller_pose(end)[:3,:3]),1e-7)
        samples = sample_motion([0,0,0,0,0,3124],[0,0,0,0,0,-3124])
        self.assertEqual(len(samples),1)

    def test_teaching_saves_relative_pose_and_rejects_changed_model_or_tool(self):
        with tempfile.TemporaryDirectory() as directory:
            root=Path(directory);cfg=root/'config.json';config=self.config();cfg.write_text(json.dumps(config))
            camera_object=np.eye(4);camera_object[:3,3]=[.1,0,0]
            observation={'object_id':'cup','model_sha256':'model-a','time':time.time(),
                         'pose':camera_object.tolist(),'K':np.diag([.5,.5,1]).tolist()}
            arm={'joint':[0]*7,'pose':[0,0,200000,0,0,0]}
            raw={'tool_name':'grip_tcp','pose':[0]*6}
            tool={'flange_T_grip_tcp':np.eye(4).tolist(),'controller_readback':{'tool_response_raw':raw}}
            handeye={'camera':{'K':np.eye(3).tolist()},'flange_T_color_optical_camera':np.eye(4).tolist()}
            tracker=SimpleNamespace(grasp_observation=Mock(return_value=observation))
            service=GraspController(root,tracker,cfg)
            robot=Mock()
            robot.query.side_effect=lambda q: raw if q=='get_current_tool_frame' else {'frame_name':'Base','pose':[0]*6}
            with patch('grasp_control.RobotClient',return_value=robot) as client, patch.object(service,'paired_observation',return_value=(arm,observation)), patch.object(service,'calibration',return_value=(handeye,tool)):
                result=service.teach(True)
                self.assertFalse(result['motion_sent'])
                client.assert_called_once_with('test',8080,writes=False)
                np.testing.assert_allclose(result['profile']['grasp_point_object_m'],[-.1,0,0])
                robot.move_linear.assert_not_called();robot.move_gripper.assert_not_called();robot.set_register.assert_not_called()
                fresh=GraspController(root,tracker,cfg)
                self.assertEqual(fresh.profile_for(config,observation,tool,handeye),result['profile'])
                with self.assertRaisesRegex(ValueError,'模型已变化'):
                    fresh.profile_for(config,{**observation,'model_sha256':'model-b'},tool,handeye)
                changed=copy.deepcopy(tool);changed['controller_readback']['tool_response_raw']['pose'][2]=1
                with self.assertRaisesRegex(ValueError,'工具参数已变化'):fresh.profile_for(config,observation,changed,handeye)
                changed_handeye=copy.deepcopy(handeye);changed_handeye['flange_T_color_optical_camera'][0][3]=.01
                with self.assertRaisesRegex(ValueError,'手眼标定已变化'):
                    fresh.profile_for(config,observation,tool,changed_handeye)
                saved=(root/'work/grasp/taught_profiles.json').read_bytes()
                with patch.object(service,'check_cancel',side_effect=ValueError('抓取已取消')):
                    with self.assertRaisesRegex(ValueError,'取消'):service.teach()
                self.assertEqual((root/'work/grasp/taught_profiles.json').read_bytes(), saved)
                result=service.teach(False)
                self.assertEqual(result['profile']['grasp_point_object_m'],[0,0,0])
                service.status['busy']=True
                with self.assertRaisesRegex(ValueError,'已有抓取'):service.teach()
            self.assertEqual(len(list((root/'work/grasp/teach_commands').glob('*.json'))),3)


if __name__ == '__main__':
    unittest.main()
