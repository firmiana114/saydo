"""Visual/multiple grasp definitions, legacy preservation and pre-motion selection."""

import copy
import fcntl
import json
import os
import pathlib
import tempfile
import types
import unittest
from unittest import mock

import numpy as np
import yaml

from saydo.manipulation import grasp_control
from saydo.geometry import grasp_geometry
from saydo.objects import grasp_library
from tests.manipulation import test_grasp


class GraspLibraryTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.addCleanup(self.temp.cleanup)
        self.root = pathlib.Path(self.temp.name)
        self.config = test_grasp.GraspTests().config()
        self.config.update(tool_file='tool.yaml', handeye_file='handeye.yaml')
        self.config['grasp_profile'].update(
            orientation='follow_object', object_R_tcp=np.eye(3).tolist()
        )
        self.cfg = self.root / 'config.json'
        self.cfg.write_text(json.dumps(self.config))
        self.tool = {'tool_name': 'grip_tcp', 'pose': [0] * 6}
        (self.root / 'tool.yaml').write_text(
            yaml.safe_dump(
                {'controller_readback': {'tool_response_raw': self.tool}}
            )
        )
        (self.root / 'handeye.yaml').write_text(
            yaml.safe_dump(
                {'flange_T_color_optical_camera': np.eye(4).tolist()}
            )
        )
        self.mesh = {
            'object_id': 'cup',
            'object_name': '测试物品',
            'model_sha256': 'mesh',
            'vertices': [
                [x, y, z]
                for x in [-.03, .03]
                for y in [-.03, .03]
                for z in [-.05, .05]
            ],
            'faces': [[0, 1, 2]],
        }
        self.service = grasp_control.GraspController(
            self.root, types.SimpleNamespace(models_root=self.root), self.cfg
        )
        self.library = grasp_library.GraspLibrary(self.root, self.root)
        self.mock = mock.patch(
            'saydo.manipulation.reorientation_geometry.model_geometry',
            side_effect=lambda *args: copy.deepcopy(self.mesh),
        )
        self.mock.start()
        self.addCleanup(self.mock.stop)
        self.profile = {
            'grasp_point_object_m': [0, 0, 0],
            'object_R_tcp': np.eye(3).tolist(),
        }

    def entry(self, name='夹法', priority=0):
        return {
            'name': name,
            'enabled': True,
            'priority': priority,
            'profile': copy.deepcopy(self.profile),
            'overrides': {},
        }

    def request(self, action='save', **extra):
        current = self.service.grasp_profiles('cup')
        return {
            'object_id': 'cup',
            'revision': current['revision'],
            'model_sha256': 'mesh',
            'action': action,
            'id': None,
            **extra,
        }

    def save(self, name='夹法', priority=0, overrides=None):
        entry = self.entry(name, priority)
        if overrides:
            entry['overrides'] = overrides
        return self.service.edit_grasp_profiles(self.request(grasp=entry))

    def test_legacy_import_preserves_original_and_other_objects(self):
        self.library.directory.mkdir(parents=True)
        old = {
            'profile': {**self.config['grasp_profile']},
            'model_sha256': 'mesh',
            'tool_response': self.tool,
            'flange_T_camera': np.eye(4).tolist(),
            'recorded_at': 1,
        }
        self.library.legacy.write_text(
            json.dumps({'cup': old, 'other': old}, indent=2) + '\n'
        )
        original = self.library.legacy.read_bytes()
        loaded = self.service.grasp_profiles('cup')
        self.assertEqual(loaded['grasps'][0]['id'], 'legacy')
        self.assertFalse(
            self.library.path.exists(),
            'reading legacy data must not migrate files',
        )
        result = self.save('新夹法')
        self.assertEqual(len(result['grasps']), 2)
        self.assertEqual(self.library.legacy.read_bytes(), original)
        self.assertFalse(self.library.object_path('other').exists())
        self.assertEqual(
            self.library.object('other')['grasps'][0]['profile'], old['profile']
        )
        self.assertEqual(
            self.library.object('cup')['grasps'][0]['profile'], old['profile']
        )

    def test_save_duplicate_disable_delete_and_selection_never_connect(self):
        with mock.patch(
            'saydo.robot.grasp_robot.socket.create_connection',
            side_effect=AssertionError('editor must never connect'),
        ):
            first = self.save('侧面')
            id1 = first['saved_id']
            copied = self.service.edit_grasp_profiles(
                self.request('duplicate', id=id1)
            )
            id2 = copied['saved_id']
            self.assertNotEqual(id1, id2)
            chosen = self.service.edit_grasp_profiles(
                self.request('select', id=id2, selection_mode='manual')
            )
            self.assertEqual(chosen['selected_id'], id2)
            entry = self.entry('副本停用')
            entry['enabled'] = False
            result = self.service.edit_grasp_profiles(
                self.request(id=id2, grasp=entry)
            )
            self.assertEqual(result['selection_mode'], 'auto')
            with self.assertRaises(ValueError):
                self.service.edit_grasp_profiles(
                    self.request('select', id=id2, selection_mode='manual')
                )
            result = self.service.edit_grasp_profiles(
                self.request('delete', id=id1)
            )
            self.assertEqual(len(result['grasps']), 1)
            result = self.service.edit_grasp_profiles(
                self.request('delete', id=id2)
            )
            self.assertEqual(result['grasps'], [])
        self.assertTrue(
            list(
                (
                    self.library.object_directory('cup')
                    / 'grasp_profile_backups'
                ).glob('*.json')
            )
        )

    def test_stale_revision_bad_geometry_busy_and_disk_failure_preserve_data(
        self
    ):
        stale = self.request(grasp=self.entry())
        self.save()
        original = self.library.object_path('cup').read_bytes()
        with self.assertRaisesRegex(ValueError, '变化'):
            self.service.edit_grasp_profiles(stale)
        for profile in [
            {
                'grasp_point_object_m': [1, 2, 3],
                'object_R_tcp': np.eye(3).tolist(),
            },
            {
                'grasp_point_object_m': [0, 0, 0],
                'object_R_tcp': np.diag([1, 1, -1]).tolist(),
            },
            {
                'grasp_point_object_m': [float('nan'), 0, 0],
                'object_R_tcp': np.eye(3).tolist(),
            },
        ]:
            entry = self.entry()
            entry['profile'] = profile
            with self.assertRaises(ValueError):
                self.service.edit_grasp_profiles(self.request(grasp=entry))
        self.service.status['busy'] = True
        with self.assertRaises(ValueError):
            self.save()
        self.service.status['busy'] = False
        with (self.library.directory / 'robot.lock').open('a') as lock:
            fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
            with self.assertRaises(BlockingIOError):
                self.save()
        with mock.patch.object(
            pathlib.Path, 'replace', side_effect=OSError('disk failure')
        ):
            with self.assertRaises(OSError):
                self.save()
        self.assertEqual(self.library.object_path('cup').read_bytes(), original)
        self.assertFalse(
            list(self.library.object_directory('cup').glob('.grasps-*'))
        )

    def test_nonorigin_center_beyond_old_twenty_cm_limit_and_overrides(self):
        self.mesh['vertices'] = (
            np.asarray(self.mesh['vertices']) + [.4, 0, 0]
        ).tolist()
        entry = self.entry()
        entry['profile']['grasp_point_object_m'] = [.4, 0, 0]
        entry['overrides'] = {
            'pregrasp_distance_m': .08,
            'open_position': 500,
            'force_target_raw': 20,
        }
        result = self.service.edit_grasp_profiles(self.request(grasp=entry))
        saved = result['grasps'][0]
        obj = np.eye(4)
        obj[:3, 3] = [-.2, 0, .2]
        plan = grasp_geometry.plan_grasp(
            [200000, 0, 300000, 0, 0, 0], obj, saved['profile'], self.config
        )
        np.testing.assert_allclose(plan['target_position_m'], [.2, 0, .2])
        self.assertEqual(saved['overrides'], entry['overrides'])

    def candidate_args(self):
        obj = np.eye(4)
        obj[:3, 3] = [.1, 0, .2]
        arm = {'joint': [0] * 7, 'pose': [0, 0, 200000, 0, 0, 0], 'err': [0]}
        observation = {'object_id': 'cup', 'model_sha256': 'mesh'}
        tool = {
            'controller_readback': {'tool_response_raw': self.tool},
            'flange_T_grip_tcp': np.eye(4).tolist(),
        }
        handeye = {'flange_T_color_optical_camera': np.eye(4).tolist()}
        report = {
            'gripper_base': {'force': True, 'force_up': [100]},
            'initial_gripper_state': {
                'sys_err': 0,
                'sys_state': 0,
                'dof_err': [0],
                'dof_state': [2],
                'pos': [1000],
                'speed': [0],
                'force': [0],
            },
        }
        return self.config, observation, tool, handeye, obj, arm, report

    def test_auto_selection_skips_invalid_candidate_and_ranks_priority_and_travel(
        self
    ):
        self.save('不可达')
        self.save('转动多', overrides={'pregrasp_distance_m': .08})
        self.save('转动少', overrides={'force_target_raw': 20})
        args = self.candidate_args()
        report = args[-1]

        def verify(*_, **kw):
            name = report['active_stage']
            if '不可达' in name:
                raise ValueError('inverse failure')
            return {'joint_travel_deg': 10 if '转动少' in name else 90}

        self.service.verify_motion_targets = mock.Mock(side_effect=verify)
        plan, profile, effective = self.service.select_grasp_candidate(*args)
        self.assertEqual(plan['grasp_name'], '转动少')
        self.assertEqual(effective['gripper']['force_target_raw'], 20)
        self.assertEqual(self.config['gripper']['force_target_raw'], 10)
        self.assertEqual(len(report['grasp_candidates']), 3)
        self.save('优先夹法', priority=10)
        plan, _, _ = self.service.select_grasp_candidate(*args)
        self.assertEqual(plan['grasp_name'], '优先夹法')

    def test_manual_selection_does_not_silently_fallback_and_bindings_are_checked(
        self
    ):
        one = self.save('不可达')
        self.save('备用')
        self.service.edit_grasp_profiles(
            self.request('select', id=one['saved_id'], selection_mode='manual')
        )
        self.service.verify_motion_targets = mock.Mock(
            side_effect=ValueError('unreachable')
        )
        with self.assertRaisesRegex(ValueError, '不可达'):
            self.service.select_grasp_candidate(*self.candidate_args())
        self.assertEqual(self.service.verify_motion_targets.call_count, 1)
        args = list(self.candidate_args())
        args[1]['model_sha256'] = 'changed'
        self.service.verify_motion_targets.reset_mock()
        with self.assertRaisesRegex(ValueError, '模型已变化'):
            self.service.select_grasp_candidate(*args)
        self.service.verify_motion_targets.assert_not_called()

    def test_candidate_specific_open_position_and_cancel_are_respected(self):
        self.save('半开夹法', overrides={'open_position': 500})
        args = list(self.candidate_args())
        args[-1]['initial_gripper_state'].update(
            pos=[500], dof_state=[3], force=[10]
        )
        self.service.verify_motion_targets = mock.Mock(
            return_value={'joint_travel_deg': 12}
        )
        plan, _, effective = self.service.select_grasp_candidate(*args)
        self.assertEqual(plan['grasp_name'], '半开夹法')
        self.assertEqual(effective['gripper']['open_position'], 500)
        self.service.cancelled.set()
        self.service.verify_motion_targets.reset_mock()
        with self.assertRaisesRegex(ValueError, '取消'):
            self.service.select_grasp_candidate(*args)
        self.service.verify_motion_targets.assert_not_called()

    def test_success_metadata_only_applies_to_unchanged_grasp_and_delete_stays_deleted(
        self
    ):
        result = self.save()
        id1 = result['saved_id']
        self.library.mark_success('cup', id1, result['revision'])
        self.assertTrue(
            self.library.object('cup')['grasps'][0]['tested_successfully']
        )
        with self.assertRaises(ValueError):
            self.library.mark_success('cup', id1, result['revision'])
        self.service.edit_grasp_profiles(self.request('delete', id=id1))
        with self.assertRaisesRegex(ValueError, '没有已启用'):
            self.service.select_grasp_candidate(*self.candidate_args())

    @unittest.skipUnless(
        os.environ.get('SAYDO_IK_CHECKER'),
        'Set SAYDO_IK_CHECKER for historical native candidate replay',
    )
    def test_native_sdk_rejects_old_grasp_and_selects_new_on_same_object_pose(
        self
    ):
        data = json.loads(
            (
                pathlib.Path(__file__).parent
                / 'fixtures/grasp_candidate_replay.json'
            ).read_text()
        )
        entries = []
        for i, row in enumerate(data['profiles']):
            entries.append(
                {
                    'id': str(i),
                    'name': row['name'],
                    'enabled': True,
                    'priority': 0,
                    'profile': row['profile'],
                    'model_sha256': 'mesh',
                    'tool_response': self.tool,
                    'overrides': {
                        'force_target_raw': 75, 'open_position': 1000
                    },
                }
            )
        self.library.write_object(
            'cup',
            {'selection_mode': 'auto', 'selected_id': None, 'grasps': entries},
            grasp_library.revision(self.library.object('cup')),
        )
        config, observation, tool, handeye, _, _, report = self.candidate_args()
        config.update(
            ik_checker=os.environ['SAYDO_IK_CHECKER'],
            max_target_distance_m=1.,
            workspace_min_m=[-1, -1, 0],
            table_z_m=-.02449,
            pregrasp_distance_m=data['pregrasp_distance_m'],
            lift_height_m=data['lift_height_m'],
        )
        tool['flange_T_grip_tcp'] = data['flange_T_grip_tcp']
        plan, _, _ = self.service.select_grasp_candidate(
            config,
            observation,
            tool,
            handeye,
            np.asarray(data['base_T_object']),
            data['arm'],
            report,
        )
        self.assertEqual(plan['grasp_name'], '重新记录的夹法')
        self.assertFalse(report['grasp_candidates'][0]['passed'])
        self.assertTrue(report['grasp_candidates'][1]['passed'])



class BindingAcrossArmsTests(unittest.TestCase):
    """同一套夹法在装着同样夹爪的两台臂上通用。"""

    FIRST_ARM = {
        'state': 'current_tool_frame',
        'tool_name': 'grip_tcp',
        'pose': [0, 0, 222500, 0, 0, 2611],
        'payload': 1146.8065185546875,
        'position': [-15000, 8000, 55000],
    }

    def error(self, **changes):
        entry = {'model_sha256': 'model', 'tool_response': self.FIRST_ARM}
        return grasp_library.binding_error(
            entry, 'model', {**self.FIRST_ARM, **changes}
        )

    def test_same_tool_geometry_with_a_different_payload_is_accepted(self):
        # 2 臂控制器里登记的负载是 1147 克，1 臂是 1146.8 克。
        self.assertIsNone(self.error(payload=1147))
        self.assertIsNone(self.error(position=[-15000, 8000, 56000]))

    def test_changed_tool_origin_or_name_still_invalidates_the_grasp(self):
        for changes in (
            {'pose': [0, 0, 200500, 0, 0, 2611]},
            {'pose': [0, 0, 222500, 0, 0, 0]},
            {'tool_name': 'Arm_Tip'},
        ):
            with self.subTest(changes=changes):
                self.assertEqual(
                    self.error(**changes), '工具参数已变化，请重新设置夹法'
                )

    def test_grasp_without_a_recorded_tool_is_not_accepted(self):
        self.assertIsNotNone(
            grasp_library.binding_error(
                {'model_sha256': 'model'}, 'model', self.FIRST_ARM
            )
        )


if __name__ == '__main__':
    unittest.main()
