import copy
import json
from pathlib import Path
import tempfile
import unittest
from types import SimpleNamespace
from unittest.mock import Mock, patch
import numpy as np
from saydo.manipulation.grasp_control import GraspController
from saydo.geometry.grasp_geometry import controller_pose, rotation_error
from saydo.manipulation.object_motion_geometry import plan_object_motion
from saydo.manipulation.reorientation_geometry import reorientation_waypoints
from saydo.geometry.rotation_steps import checked_step
from tests.manipulation import test_grasp
from tests.manipulation.test_object_motion import fixture


class MotionRecoveryTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.addCleanup(self.temp.cleanup)
        root = Path(self.temp.name)
        held, vertices, arm, config = fixture()
        config = {**test_grasp.GraspTests().config(), **config}
        path = root / 'trial.json'
        path.write_text(json.dumps(config))
        self.config = config
        self.held = held
        self.vertices = vertices
        self.arm = arm
        self.c = GraspController(root, SimpleNamespace(models_root=root), path)
        self.c.held_object = {**copy.deepcopy(held), 'orientation_version': 1}
        self.c.actuated = True
        self.c.reorientation_hardware = Mock(return_value=(arm, {}))
        self.c.check_reorient_contact = Mock(return_value={'speed': [0]})
        self.c.motion_binding = Mock(return_value={'binding': 'same'})
        self.c.robot = Mock()
        self.mesh = {
            'model_sha256': held['model_sha256'],
            'pose_mode': '6d',
            'vertices': vertices,
        }
        self.report = {
            'motion_events': [
                {
                    'accepted': True,
                    'completion_reply': {
                        'raw': (
                            '{"state":"current_trajectory_state","trajectory_st'
                            'ate":false}'
                        )
                    },
                }
            ],
            'stop_acknowledged': True,
            'hold_retention': {
                'retained': True,
                'stationary_observed': True,
                'controller_errors': [0],
                'gripper_samples': [{'speed': [0]}] * 3,
            },
            'configuration_binding': {'binding': 'same'},
            'plan': {
                'kind': 'held_object_motion',
                'base_T_object_target': np.eye(4).tolist(),
                'goal': {
                    'position_mode': 'fixed', 'target_position_m': [.3, 0, .3]
                },
            },
            'completed_waypoints': 2,
            'active_waypoint_index': 3,
            'active_stage': 'object_1_3',
        }

    def assess(self):
        with patch(
            'saydo.manipulation.motion_recovery.model_geometry',
            return_value=self.mesh,
        ):
            return self.c.assess_motion_recovery(
                self.held, self.config, self.report
            )

    def test_recovery_is_from_measured_pose_with_new_version_and_no_actuation(
        self
    ):
        r = self.assess()
        self.assertTrue(r['can_replan'], r)
        self.assertEqual(r['orientation_version'], 1)
        np.testing.assert_allclose(
            r['base_T_object_actual'],
            (
                controller_pose(self.arm['pose'])
                @ np.linalg.inv(self.held['object_T_tcp'])
            ),
        )
        self.assertEqual(r['completed_waypoints'], 2)
        self.assertEqual(r['failed_waypoint_index'], 3)
        self.assertTrue(r['requires_new_plan'])
        self.c.robot.stop.assert_not_called()
        self.c.robot.move_joints.assert_not_called()

    def test_cancel_stop_error_contact_controller_and_binding_reject_recovery(
        self
    ):
        for case in (
            'cancel',
            'stop',
            'stationarity',
            'contact',
            'controller',
            'binding',
            'release',
            'recording',
            'unknown_motion',
        ):
            with self.subTest(case=case):
                self.setUp()
                if case == 'cancel':
                    self.c.cancelled.set()
                if case == 'stop':
                    self.report['stop_acknowledged'] = False
                if case == 'stationarity':
                    self.report['hold_retention']['stationary_observed'] = False
                if case == 'contact':
                    self.c.held_object = None
                if case == 'controller':
                    self.report['hold_retention']['controller_errors'] = [4099]
                if case == 'binding':
                    self.c.motion_binding.return_value = {'different': True}
                if case == 'release':
                    self.report['release_attempted'] = True
                if case == 'recording':
                    self.c.agent_recording_error = 'disk'
                if case == 'unknown_motion':
                    self.report['motion_events'] = []
                result = self.assess()
                self.assertFalse(result['can_replan'])
                if case in ('stop', 'stationarity', 'unknown_motion'):
                    self.assertTrue(result['outcome_unknown'])

    def test_new_and_legacy_geometry_have_same_step_cap(self):
        held, vertices, arm, config = fixture()
        target = np.diag([1, -1, -1])
        for step in (15, 10, 5):
            old = reorientation_waypoints(
                held,
                vertices,
                target,
                arm['pose'],
                config,
                {'max_rotation_step_deg': step},
            )
            new = plan_object_motion(
                held,
                vertices,
                arm,
                config,
                {'target_rpy_deg': [180, 0, 0], 'max_rotation_step_deg': step},
                np.eye(4),
                lambda p, t: (True, {}),
                lambda: None,
            )
            for plan in (old, new):
                before = controller_pose(arm['pose'])
                for row in plan['waypoints']:
                    after = controller_pose(row['controller_pose'])
                    self.assertLessEqual(
                        np.degrees(
                            rotation_error(before[:3, :3], after[:3, :3])
                        ),
                        step + 1e-6,
                    )
                    before = after
        for value in (True, 0, 16, 45, float('nan')):
            with self.assertRaises(ValueError):
                checked_step(value)
