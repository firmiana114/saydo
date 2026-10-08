"""Per-grasp binding, observable effective settings and lossless model migration."""

import copy
import json
import pathlib
import tempfile
import types
import unittest
from unittest import mock

from scripts import bind_gripper_parameters
from saydo.manipulation import grasp_control
from saydo.objects import grasp_library


class BoundGripperTests(unittest.TestCase):
    def test_defaults_for_registered_object_types(self):
        for oid, force, opening in [
            ('ob_paper_cup_85mm', 10, 1000),
            ('paper_cup_85mm_origin_wall', 10, 1000),
            ('mug_iphone_20260917', 75, 200),
            ('ob_yibao_350ml', 75, 1000),
            ('transparent_test_tube_150mm', 75, 1000),
            ('new_object', 75, 1000),
        ]:
            self.assertEqual(
                grasp_library.object_gripper_defaults(oid),
                {'force_target_raw': force, 'open_position': opening},
            )

    def test_bound_values_override_global_and_cannot_silently_fall_back(self):
        config = {
            'gripper': {
                'force_target_raw': 99,
                'force_limit_raw': 100,
                'open_position': 333,
            },
            'pregrasp_distance_m': .05,
            'lift_height_m': .2,
            'arm_speed_percent': 25,
        }
        entry = {
            'overrides': {
                'force_target_raw': 10,
                'open_position': 1000,
                'pregrasp_distance_m': .08,
            }
        }
        before = copy.deepcopy(config)
        result = grasp_library.effective_config(config, entry)
        self.assertEqual(
            result['gripper'],
            {
                'force_target_raw': 10,
                'force_limit_raw': 10,
                'open_position': 1000,
            },
        )
        self.assertEqual(result['pregrasp_distance_m'], .08)
        self.assertEqual(config, before)
        params = grasp_library.effective_parameters(config, entry)
        self.assertEqual(params['gripper_parameters_source'], 'grasp_profile')
        self.assertEqual(params['lift_height_m'], .2)
        for values in [
            {},
            {'force_target_raw': 10},
            {'open_position': 200},
            {'force_target_raw': 101, 'open_position': 1000},
            {'force_target_raw': True, 'open_position': 1000},
        ]:
            with self.assertRaises(ValueError):
                grasp_library.effective_config(config, {'overrides': values})

    def library(self, root):
        library = grasp_library.GraspLibrary(root)
        for oid in [
            'ob_paper_cup_85mm',
            'mug_iphone_20260917',
            'ob_yibao_350ml',
            'transparent_test_tube_150mm',
        ]:
            value = {
                'selection_mode': 'manual',
                'selected_id': 'g1',
                'grasps': [
                    {
                        'id': 'g1',
                        'name': '保留名称',
                        'profile': {'original_geometry': [1, 2, 3]},
                        'enabled': True,
                        'priority': 4,
                        'overrides': {'pregrasp_distance_m': .07},
                        'tested_successfully': True,
                        'last_success_at': 123,
                    },
                    {
                        'id': 'g2',
                        'name': '另一个',
                        'profile': {'original_geometry': [4, 5, 6]},
                        'enabled': False,
                        'priority': 0,
                        'overrides': {},
                    },
                ],
            }
            library.write_object(
                oid, value, grasp_library.revision(library.object(oid))
            )
        return library

    def test_bulk_migration_preserves_geometry_selection_and_history(self):
        with tempfile.TemporaryDirectory() as d:
            library = self.library(pathlib.Path(d))
            original = {
                p: p.read_bytes()
                for p in library.models_root.glob('*/grasp_profiles.json')
            }
            preview = bind_gripper_parameters.migrate(library)
            self.assertEqual(preview['files_to_write'], 4)
            for p, data in original.items():
                self.assertEqual(p.read_bytes(), data)
            with mock.patch(
                'saydo.robot.grasp_robot.socket.create_connection',
                side_effect=AssertionError('no hardware'),
            ):
                report = bind_gripper_parameters.migrate(library, True)
            self.assertFalse(report['motion_sent'])
            self.assertEqual(
                bind_gripper_parameters.migrate(library)['files_to_write'], 0
            )
            for p, data in original.items():
                before = json.loads(data)['configuration']
                after = library.object(p.parent.name)
                self.assertEqual(after['selected_id'], before['selected_id'])
                self.assertEqual(
                    after['selection_mode'], before['selection_mode']
                )
                for old, new in zip(before['grasps'], after['grasps']):
                    for key in ['id', 'name', 'profile', 'enabled', 'priority']:
                        self.assertEqual(new[key], old[key])
                    for key, value in grasp_library.object_gripper_defaults(
                        p.parent.name
                    ).items():
                        self.assertEqual(new['overrides'][key], value)
                    self.assertFalse(new['tested_successfully'])
                self.assertEqual(after['grasps'][0]['last_success_at'], 123)
                self.assertEqual(
                    after['grasps'][0]['overrides']['pregrasp_distance_m'], .07
                )
                self.assertIn(
                    data,
                    [
                        b.read_bytes()
                        for b in (
                            p.parent / 'grasp_profile_backups'
                        ).glob('*.json')
                    ],
                )

    def test_migration_preflight_rejects_bad_record_before_any_write(self):
        with tempfile.TemporaryDirectory() as d:
            library = self.library(pathlib.Path(d))
            bad = library.object_path('transparent_test_tube_150mm')
            bad.write_text('{bad json')
            original = {
                p: p.read_bytes()
                for p in library.models_root.glob('*/grasp_profiles.json')
            }
            with self.assertRaises(ValueError):
                bind_gripper_parameters.migrate(library, True)
            for p, data in original.items():
                self.assertEqual(p.read_bytes(), data)

    def test_snapshot_exposes_bound_values_not_global_and_reports_missing_bindings(
        self
    ):
        with tempfile.TemporaryDirectory() as d:
            root = pathlib.Path(d)
            library = self.library(root)
            bind_gripper_parameters.migrate(library, True)
            cfg = root / 'trial.json'
            cfg.write_text(
                json.dumps(
                    {
                        'gripper': {
                            'force_target_raw': 99,
                            'force_limit_raw': 99,
                            'open_position': 111,
                        },
                        'pregrasp_distance_m': .05,
                        'lift_height_m': .2,
                        'arm_speed_percent': 25,
                    }
                )
            )
            tracker = types.SimpleNamespace(
                state={'object_id': 'mug_iphone_20260917'}
            )
            service = grasp_control.GraspController(root, tracker, cfg)
            result = service.snapshot()['grasp_configuration']
            self.assertEqual(
                result['teaching_defaults'],
                {'force_target_raw': 75, 'open_position': 200},
            )
            for g in result['grasps']:
                self.assertEqual(
                    g['effective_parameters']['force_target_raw'], 75
                )
                self.assertEqual(
                    g['effective_parameters']['open_position'], 200
                )
            value = library.object('mug_iphone_20260917')
            before = grasp_library.revision(value)
            value['grasps'][0]['overrides'] = {}
            library.write_object('mug_iphone_20260917', value, before)
            self.assertIn(
                '尚未绑定',
                service.snapshot()[
                    'grasp_configuration'
                ]['grasps'][0]['parameter_error'],
            )

    def test_teaching_rejects_stale_object_and_bad_parameters_before_robot_access(
        self
    ):
        with tempfile.TemporaryDirectory() as d:
            root = pathlib.Path(d)
            cfg = root / 'config.json'
            cfg.write_text('{}')
            tracker = types.SimpleNamespace(
                grasp_observation=mock.Mock(
                    return_value={
                        'object_id': 'ob_paper_cup_85mm', 'model_sha256': 'm'
                    }
                )
            )
            service = grasp_control.GraspController(root, tracker, cfg)
            with mock.patch('saydo.robot.grasp_robot.RobotClient') as robot:
                with self.assertRaisesRegex(ValueError, '物品已变化'):
                    service.teach(expected_object_id='mug')
                for params in [
                    {'force_target_raw': 0, 'open_position': 1000},
                    {'force_target_raw': 10, 'open_position': 1001},
                    {'force_target_raw': 10},
                ]:
                    with self.assertRaises(ValueError):
                        service.teach(gripper_parameters=params)
                robot.assert_not_called()


if __name__ == '__main__':
    unittest.main()
