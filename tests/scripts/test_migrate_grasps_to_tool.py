"""夹法换算到新工具：真实的夹法库读写，临时目录。"""

import copy
import json
import pathlib
import tempfile
import unittest

import numpy as np

from saydo.geometry import grasp_geometry
from saydo.objects import grasp_library
from scripts import migrate_grasps_to_tool

OLD_TOOL = {
    'state': 'current_tool_frame',
    'tool_name': 'grip_tcp',
    'pose': [0, 0, 222500, 0, 0, 2611],
    'payload': 1146.8,
}
NEW_TOOL = {**OLD_TOOL, 'pose': [0, 0, 227500, 0, 0, 2611], 'payload': 1147}


def grasp(grasp_id, offset=None):
    overrides = {'force_target_raw': 75, 'open_position': 1000}
    if offset is not None:
        overrides['tcp_offset_m'] = offset
    return {
        'id': grasp_id,
        'name': grasp_id,
        'enabled': True,
        'priority': 0,
        'profile': {
            'grasp_point_object_m': [0., 0., 0.],
            'orientation': 'follow_object',
            'object_R_tcp': np.eye(3).tolist(),
            'approach_axis_tcp': [0, 0, 1],
        },
        'overrides': overrides,
        'model_sha256': 'model',
        'tool_response': copy.deepcopy(OLD_TOOL),
        'tested_successfully': True,
        'tested_successfully_5d': True,
    }


class MigrateTests(unittest.TestCase):
    def setUp(self):
        directory = tempfile.TemporaryDirectory()
        self.addCleanup(directory.cleanup)
        self.root = pathlib.Path(directory.name)
        self.models = self.root / 'work/object_models'
        self.library = grasp_library.GraspLibrary(self.root, self.models)
        self.put('tube', [grasp('lying', [0, 0, -.007])])
        self.put('bottle', [grasp('side'), grasp('top', [.001, 0, .002])])

    def put(self, object_id, grasps):
        folder = self.models / object_id
        folder.mkdir(parents=True)
        (folder / 'grasp_profiles.json').write_text(
            json.dumps(
                {
                    'schema_version': 3,
                    'object_id': object_id,
                    'configuration': {
                        'selection_mode': 'auto',
                        'selected_id': None,
                        'grasps': grasps,
                    },
                }
            )
        )

    def offsets(self, object_id):
        return {
            entry['id']: entry['overrides'].get('tcp_offset_m')
            for entry in self.library.object(object_id)['grasps']
        }

    def test_preview_changes_nothing(self):
        before = self.offsets('tube')
        summary = migrate_grasps_to_tool.migrate(
            self.library, NEW_TOOL, {'tube': .0091}
        )
        self.assertFalse(summary['applied'])
        self.assertEqual(summary['grasps_migrated'], 3)
        self.assertEqual(self.offsets('tube'), before)

    def test_flange_target_is_unchanged_by_the_tool_extension(self):
        migrate_grasps_to_tool.migrate(self.library, NEW_TOOL, {}, apply=True)
        self.assertEqual(self.offsets('bottle')['side'], [0., 0., -.005])
        self.assertEqual(self.offsets('bottle')['top'], [.001, 0., -.003])
        # 用真实的规划几何核对：同一个物品位姿下，旧工具配旧偏移与新工具
        # 配新偏移，算出的法兰位置相同。
        base_T_object = np.eye(4)
        base_T_object[:3, 3] = [.3, -.1, .05]
        config = {
            'action': 'grasp_and_lift',
            'pregrasp_distance_m': .05,
            'lift_height_m': .1,
            'max_target_distance_m': 1.,
        }
        flanges = []
        for tool, offset in (
            (OLD_TOOL, [.001, 0., .002]), (NEW_TOOL, [.001, 0., -.003])
        ):
            plan = grasp_geometry.plan_grasp(
                [300000, -100000, 300000, 0, 0, 0],
                base_T_object,
                grasp('top')['profile'],
                config,
                grasp_reference_offset_tcp_m=offset,
            )
            base_T_tcp = np.eye(4)
            base_T_tcp[:3, :3] = plan['target_rotation']
            base_T_tcp[:3, 3] = plan['target_position_m']
            flange_T_tcp = grasp_geometry.controller_pose(tool['pose'])
            flanges.append(base_T_tcp @ np.linalg.inv(flange_T_tcp))
        np.testing.assert_allclose(flanges[0], flanges[1], atol=1e-9)

    def test_height_raise_applies_only_to_the_named_object(self):
        migrate_grasps_to_tool.migrate(
            self.library, NEW_TOOL, {'tube': .0091}, apply=True
        )
        # −7 − 5 + 9.1 = −2.9 毫米。
        self.assertEqual(self.offsets('tube')['lying'], [0., 0., -.0029])
        self.assertEqual(self.offsets('bottle')['side'], [0., 0., -.005])

    def test_migrated_grasp_is_bound_to_the_new_tool_and_marked_untested(self):
        migrate_grasps_to_tool.migrate(self.library, NEW_TOOL, {}, apply=True)
        entry = self.library.object('tube')['grasps'][0]
        self.assertIsNone(
            grasp_library.binding_error(entry, 'model', NEW_TOOL)
        )
        self.assertIsNotNone(
            grasp_library.binding_error(entry, 'model', OLD_TOOL)
        )
        self.assertFalse(entry['tested_successfully'])
        self.assertFalse(entry['tested_successfully_5d'])
        note = entry['tool_migrations'][-1]
        self.assertEqual(note['tcp_offset_before_m'], [0, 0, -.007])
        self.assertAlmostEqual(note['tool_extension_m'], .005)

    def test_running_twice_does_not_shift_the_offsets_again(self):
        migrate_grasps_to_tool.migrate(self.library, NEW_TOOL, {}, apply=True)
        once = self.offsets('bottle')
        summary = migrate_grasps_to_tool.migrate(
            self.library, NEW_TOOL, {}, apply=True
        )
        self.assertEqual(summary['grasps_migrated'], 0)
        self.assertEqual(self.offsets('bottle'), once)

    def test_tool_changed_in_another_way_stops_before_writing_anything(self):
        rotated = {**NEW_TOOL, 'pose': [0, 0, 227500, 0, 0, 0]}
        before = (self.offsets('tube'), self.offsets('bottle'))
        with self.assertRaisesRegex(ValueError, '不只是 Z 不同'):
            migrate_grasps_to_tool.migrate(
                self.library, rotated, {}, apply=True
            )
        self.assertEqual((self.offsets('tube'), self.offsets('bottle')), before)

    def test_raise_for_an_object_without_grasps_is_refused(self):
        with self.assertRaisesRegex(ValueError, 'cup'):
            migrate_grasps_to_tool.migrate(
                self.library, NEW_TOOL, {'cup': .01}, apply=True
            )
        self.assertEqual(self.offsets('tube')['lying'], [0, 0, -.007])

    def test_raise_option_is_parsed_in_millimetres(self):
        self.assertEqual(
            migrate_grasps_to_tool.parse_raise('tube=9.1'), ('tube', .0091)
        )
        for text in ('tube', 'tube=abc', '=3', 'tube=45'):
            with self.subTest(text=text), self.assertRaises(Exception):
                migrate_grasps_to_tool.parse_raise(text)


if __name__ == '__main__':
    unittest.main()
