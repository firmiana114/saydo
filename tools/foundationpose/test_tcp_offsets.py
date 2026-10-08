import copy
import json
from pathlib import Path
import tempfile
import unittest
from unittest.mock import Mock
import numpy as np
from scipy.spatial.transform import Rotation
from grasp_geometry import plan_grasp, checked_tcp_offset
from grasp_library import checked_overrides, effective_config, effective_parameters, GraspLibrary, revision
from rebind_tool_grasps import rebind_grasps, updated_tool_config
import test_grasp_library


class OffsetTests(unittest.TestCase):
    setUp = test_grasp_library.GraspLibraryTests.setUp
    entry = test_grasp_library.GraspLibraryTests.entry
    save = test_grasp_library.GraspLibraryTests.save
    request = test_grasp_library.GraspLibraryTests.request
    candidate_args = test_grasp_library.GraspLibraryTests.candidate_args

    def test_validation_and_absent_value_never_inherit_a_global_offset(self):
        for bad in (
            None,
            0,
            [0, 0],
            [0, 0, 0, 0],
            [False, 0, 0],
            ['0', 0, 0],
            [float('nan'), 0, 0],
            [0, float('inf'), 0],
            [0, 0, .30001],
        ):
            with self.subTest(bad=bad), self.assertRaises(ValueError):
                checked_overrides({'tcp_offset_m': bad})
        self.assertEqual(checked_tcp_offset([-.3, 0, .3]), [-.3, 0, .3])
        self.config['tcp_offset_m'] = [.1, .1, .1]
        entry = {'overrides': {'force_target_raw': 30, 'open_position': 500}}
        self.assertEqual(
            effective_config(self.config, entry)['tcp_offset_m'], [0, 0, 0]
        )
        self.assertEqual(self.config['tcp_offset_m'], [.1, .1, .1])

    def test_offset_rotates_in_tool_axes_and_moves_all_controller_waypoints(
        self
    ):
        config, _, _, _, base, arm, _ = self.candidate_args()
        rotation = Rotation.from_euler(
            'xyz', [25, -35, 110], degrees=True
        ).as_matrix()
        profile = {
            'orientation': 'follow_object',
            'object_R_tcp': rotation.tolist(),
            'grasp_point_object_m': [.01, -.02, .03],
        }
        baseline = plan_grasp(arm['pose'], base, profile, config)
        offset = [.005, -.008, -.03]
        plan = plan_grasp(
            arm['pose'],
            base,
            profile,
            config,
            grasp_reference_offset_tcp_m=offset,
        )
        shift = -rotation @ offset
        np.testing.assert_allclose(
            plan['grasp_reference_position_m'], baseline['target_position_m']
        )
        for old, new in zip(baseline['waypoints'], plan['waypoints']):
            np.testing.assert_allclose(
                np.array(new['position_m']) - old['position_m'],
                shift,
                atol=1e-12,
            )
        np.testing.assert_allclose(
            np.array(plan['target_position_m']) + rotation @ offset,
            plan['grasp_reference_position_m'],
        )

    def test_save_hot_update_reaches_planner_and_copy_without_modifying_geometry(
        self
    ):
        saved = self.save(overrides={'tcp_offset_m': [0, 0, 0]})
        ident = saved['saved_id']
        args = self.candidate_args()
        self.service.verify_motion_targets = Mock(
            return_value={'joint_travel_deg': 1}
        )
        before = self.library.object('cup')['grasps'][0]['profile']
        entry = self.entry()
        entry['overrides']['tcp_offset_m'] = [.005, .006, -.03]
        after = self.service.edit_grasp_profiles(
            self.request(id=ident, grasp=entry)
        )
        self.assertEqual(
            after['grasps'][0]['effective_parameters']['tcp_offset_m'],
            [.005, .006, -.03],
        )
        plan, _, _ = self.service.select_grasp_candidate(*args)
        np.testing.assert_allclose(
            plan['target_position_m'], [.095, -.006, .23]
        )
        self.assertEqual(
            self.library.object('cup')['grasps'][0]['profile'], before
        )
        copied = self.service.edit_grasp_profiles(
            self.request('duplicate', id=ident)
        )
        self.assertEqual(
            copied['grasps'][-1]['overrides']['tcp_offset_m'],
            [.005, .006, -.03],
        )


class RebindTests(unittest.TestCase):
    def test_zero_rebind_preserves_grasps_backups_and_is_idempotent(self):
        old = {'tool_name': 'grip_tcp', 'pose': [0, 0, 190500, 0, 0, 2611]}
        new = {**old, 'pose': [0, 0, 222500, 0, 0, 2611]}
        with tempfile.TemporaryDirectory() as d:
            lib = GraspLibrary(Path(d))
            entry = {
                'id': 'g',
                'name': '保留',
                'enabled': True,
                'priority': 3,
                'profile': {'grasp_point_object_m': [1, 2, 3]},
                'profile_5d': {'keep': 1},
                'tool_response': old,
                'overrides': {'force_target_raw': 30, 'open_position': 500},
                'tested_successfully': True,
                'last_success_at': 123,
            }
            value = {
                'selection_mode': 'manual',
                'selected_id': 'g',
                'grasps': [entry],
            }
            lib.write_object('cup', value, revision(lib.object('cup')))
            before = lib.object_path('cup').read_bytes()
            self.assertEqual(
                rebind_grasps(lib, old, new)['grasps_to_rebind'], 1
            )
            self.assertEqual(lib.object_path('cup').read_bytes(), before)
            rebind_grasps(lib, old, new, True)
            result = lib.object('cup')
            g = result['grasps'][0]
            for key in [
                'id',
                'name',
                'enabled',
                'priority',
                'profile',
                'profile_5d',
                'last_success_at',
            ]:
                self.assertEqual(g[key], entry[key])
            self.assertEqual(g['overrides']['tcp_offset_m'], [0, 0, 0])
            self.assertFalse(g['tested_successfully'])
            self.assertFalse(g['tested_successfully_5d'])
            self.assertEqual(result['selected_id'], 'g')
            self.assertTrue(
                list(
                    (
                        lib.object_directory('cup') / 'grasp_profile_backups'
                    ).glob('*.json')
                )
            )
            g['overrides']['tcp_offset_m'] = [0, 0, -.02]
            lib.write_object('cup', result, revision(lib.object('cup')))
            self.assertEqual(
                rebind_grasps(lib, old, new, True)['files_to_write'], 0
            )
            self.assertEqual(
                lib.object('cup')['grasps'][0]['overrides']['tcp_offset_m'],
                [0, 0, -.02],
            )
            wrong = copy.deepcopy(entry)
            wrong['tool_response'] = {'tool_name': 'other'}
            lib.write_object(
                'wrong',
                {**value, 'grasps': [wrong]},
                revision(lib.object('wrong')),
            )
            with self.assertRaisesRegex(ValueError, '不一致'):
                rebind_grasps(lib, old, new, True)

    def test_tool_snapshot_updates_transform_load_and_provenance_without_mutating_old(
        self
    ):
        old = {
            'tool_name': 'grip_tcp',
            'state': 'current_tool_frame',
            'pose': [0, 0, 190500, 0, 0, 2611],
            'payload': 989.,
            'position': [-12000, 9000, 120000],
        }
        previous = {
            'controller_readback': {'tool_response_raw': old},
            'load': {},
            'geometry': {'status': 'old'},
        }
        before = copy.deepcopy(previous)
        new = {
            **old,
            'pose': [0, 0, 222500, 0, 0, 2611],
            'payload': 1146.8065185546875,
            'position': [-15000, 8000, 55000],
        }
        result = updated_tool_config(
            previous, new, 'work/readback.json', 'hash', 'date'
        )
        self.assertEqual(previous, before)
        self.assertEqual(result['translation_m'], [0, 0, .2225])
        self.assertEqual(
            result['load']['center_of_mass_m'], [-.015, .008, .055]
        )
        self.assertAlmostEqual(result['load']['mass_kg'], new['payload'] / 1000)
        self.assertEqual(
            result['controller_readback']['tool_response_raw'], new
        )
        self.assertEqual(
            result['controller_readback']['path'], 'work/readback.json'
        )
        self.assertFalse(result['controller_written_by_this_change'])
        changed = copy.deepcopy(new)
        changed['pose'][5] += 1
        with self.assertRaises(ValueError):
            updated_tool_config(previous, changed, 'p', 'h', 'date')


if __name__ == '__main__':
    unittest.main()
