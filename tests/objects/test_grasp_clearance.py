import copy
import json
import unittest
from unittest.mock import Mock

import numpy as np

from saydo.objects.grasp_library import checked_overrides, effective_config, effective_parameters, held_clearance_config
from tests.objects import test_grasp_library as base


class GraspClearanceTests(unittest.TestCase):
    setUp = base.GraspLibraryTests.setUp
    entry = base.GraspLibraryTests.entry
    request = base.GraspLibraryTests.request
    save = base.GraspLibraryTests.save
    candidate_args = base.GraspLibraryTests.candidate_args

    def test_independent_values_default_and_validation(self):
        self.config['min_table_clearance_m'] = .01
        for value in (0, .003, .12):
            entry = {
                'overrides': {
                    'open_position': 1000,
                    'force_target_raw': 30,
                    'min_table_clearance_m': value,
                }
            }
            config = effective_config(self.config, entry)
            self.assertEqual(config['min_table_clearance_m'], value)
            self.assertEqual(self.config['min_table_clearance_m'], .01)
            self.assertEqual(
                effective_parameters(self.config, entry)[
                    'table_clearance_source'
                ],
                'grasp_profile',
            )
        old = {'overrides': {'open_position': 1000, 'force_target_raw': 30}}
        self.assertEqual(
            effective_parameters(self.config, old)['min_table_clearance_m'], .01
        )
        self.assertEqual(
            effective_parameters(self.config, old)['table_clearance_source'],
            'global_default',
        )
        for value in (-.001, True, '0.01', float('nan'), float('inf'), None):
            with self.assertRaises(ValueError):
                checked_overrides({'min_table_clearance_m': value})

    def test_editor_save_immediately_changes_planning_without_restart(self):
        self.config['min_table_clearance_m'] = .4
        self.cfg.write_text(json.dumps(self.config))
        saved = self.save(
            '可热更新夹法', overrides={'min_table_clearance_m': .4}
        )
        ident = saved['saved_id']
        before = self.cfg.read_bytes()
        self.service.verify_motion_targets = Mock(
            return_value={'joint_travel_deg': 1}
        )
        with self.assertRaisesRegex(ValueError, '距配置桌面'):
            self.service.select_grasp_candidate(*self.candidate_args())
        self.service.verify_motion_targets.assert_not_called()
        edited = self.entry('可热更新夹法')
        edited['overrides']['min_table_clearance_m'] = .003
        result = self.service.edit_grasp_profiles(
            self.request(id=ident, grasp=edited)
        )
        plan, _, config = self.service.select_grasp_candidate(
            *self.candidate_args()
        )
        self.assertEqual(config['min_table_clearance_m'], .003)
        self.assertEqual(
            plan['effective_parameters']['min_table_clearance_m'], .003
        )
        self.assertNotEqual(saved['revision'], result['revision'])
        self.assertEqual(self.cfg.read_bytes(), before)
        self.assertEqual(
            self.library.object('cup')['grasps'][0]['overrides'][
                'min_table_clearance_m'
            ],
            .003,
        )
        # Clearing the override restores the global rule, rather than writing
        # zero.
        self.service.edit_grasp_profiles(
            self.request(id=ident, grasp=self.entry('恢复默认'))
        )
        with self.assertRaisesRegex(ValueError, '距配置桌面'):
            self.service.select_grasp_candidate(*self.candidate_args())

    def test_busy_save_rejected_and_existing_hold_keeps_its_snapshot(self):
        saved = self.save(overrides={'min_table_clearance_m': .025})
        self.service.status['gripper_contact'] = {'pos': [500]}
        self.service.arm_state = Mock(
            return_value={'pose': [0, 0, 200000, 0, 0, 0]}
        )
        plan = {
            'object_id': 'cup',
            'object_name': 'cup',
            'model_sha256': 'mesh',
            'base_T_object': np.eye(4).tolist(),
            'tool_response': self.tool,
            'effective_parameters': {'min_table_clearance_m': .025},
        }
        report = {}
        self.service.capture_attachment(plan, report)
        held = copy.deepcopy(report['held_object'])
        self.assertEqual(held['min_table_clearance_m'], .025)
        self.service.status['busy'] = True
        with self.assertRaisesRegex(ValueError, '任务进行中'):
            self.service.edit_grasp_profiles(
                self.request(id=saved['saved_id'], grasp=self.entry())
            )
        self.service.status['busy'] = False
        edited = self.entry()
        edited['overrides']['min_table_clearance_m'] = .08
        self.service.edit_grasp_profiles(
            self.request(id=saved['saved_id'], grasp=edited)
        )
        config = held_clearance_config(self.config, held)
        self.assertEqual(config['min_table_clearance_m'], .025)
        self.assertEqual(held_clearance_config(self.config, {}), self.config)


if __name__ == '__main__':
    unittest.main()
