"""Per-model storage, migration, isolation and read-only legacy fallback."""

import copy
import json
from pathlib import Path
import tempfile
import unittest

from saydo.objects.grasp_library import GraspLibrary, revision
from scripts.migrate_model_grasps import migrate


class ModelGraspStorageTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.addCleanup(self.temp.cleanup)
        self.root = Path(self.temp.name)
        self.models = self.root / 'custom-models'
        self.library = GraspLibrary(self.root, self.models)
        self.library.directory.mkdir(parents=True)
        for name in ['bottle', 'cup']:
            (self.models / name).mkdir(parents=True)
        self.value = {
            'selection_mode': 'manual',
            'selected_id': 'one',
            'grasps': [
                {
                    'id': 'one',
                    'name': '轻微下斜0',
                    'enabled': True,
                    'priority': 3,
                    'profile': {
                        'grasp_point_object_m': [.01, .02, .03],
                        'object_R_tcp': [[1, 0, 0], [0, 1, 0], [0, 0, 1]],
                    },
                    'overrides': {'force_target_raw': 20},
                    'tested_successfully': True,
                    'last_success_at': 123,
                    'tool_response': {'pose': [0] * 6},
                    'model_sha256': 'mesh',
                }
            ],
        }
        self.global_data = {
            'schema_version': 2,
            'objects': {'bottle': self.value, 'cup': self.value},
        }
        self.library.path.write_text(json.dumps(self.global_data))

    def test_single_model_write_preserves_other_models_and_legacy(self):
        original = self.library.path.read_bytes()
        before = revision(self.library.object('bottle'))
        changed = copy.deepcopy(self.value)
        changed['grasps'][0]['name'] = '新名字'
        self.library.write_object('bottle', changed, before)
        self.assertEqual(self.library.object('bottle'), changed)
        self.assertEqual(self.library.object('cup'), self.value)
        self.assertFalse(self.library.object_path('cup').exists())
        self.assertEqual(self.library.path.read_bytes(), original)
        self.assertTrue((self.models / 'bottle/grasp_profiles.json').exists())
        self.assertFalse((self.root / 'work/object_models').exists())
        with self.assertRaises(ValueError):
            self.library.write_object('bottle', changed, before)

    def test_empty_local_file_and_success_history_are_authoritative(self):
        self.library.write_object('bottle', self.value, revision(self.value))
        empty = {'selection_mode': 'auto', 'selected_id': None, 'grasps': []}
        self.library.write_object('bottle', empty, revision(self.value))
        self.assertEqual(self.library.object('bottle'), empty)
        self.assertTrue(self.library.has_object('bottle'))
        backups = list(
            (self.models / 'bottle/grasp_profile_backups').glob('*.json')
        )
        self.assertEqual(
            json.loads(backups[0].read_text())['configuration'], self.value
        )
        self.library.path.write_text('invalid central file')
        self.assertEqual(self.library.object('bottle'), empty)

    def test_bad_local_file_does_not_fall_back_to_old_grasp(self):
        path = self.library.object_path('bottle')
        path.write_text('{')
        with self.assertRaises(ValueError):
            self.library.object('bottle')
        path.write_text(
            json.dumps(
                {
                    'schema_version': 3,
                    'object_id': 'cup',
                    'configuration': self.value,
                }
            )
        )
        with self.assertRaises(ValueError):
            self.library.object('bottle')

    def test_path_traversal_and_symlink_rejected(self):
        for name in [
            '../cup', '/tmp/cup', '.', '', 'cup/other', 'cup\\other', None
        ]:
            with self.assertRaises(ValueError):
                self.library.object(name)
        (self.models / 'link').symlink_to(self.root, target_is_directory=True)
        with self.assertRaises(ValueError):
            self.library.object('link')
        self.library.object_path('bottle').symlink_to(self.library.path)
        with self.assertRaises(ValueError):
            self.library.object('bottle')

    def test_migration_preserves_values_histories_sources_and_is_idempotent(
        self
    ):
        self.library.legacy.write_text(
            json.dumps({'cup': {'profile': {'old': True}}})
        )
        backup = self.library.directory / 'grasp_profile_backups'
        backup.mkdir()
        (backup / '123.json').write_text(json.dumps(self.global_data))
        for kind, report in [
            ('teach_commands', {'observation': {'object_id': 'cup'}}),
            ('grasp_profile_commands', {'object_id': 'bottle'}),
        ]:
            directory = self.library.directory / kind
            directory.mkdir()
            (directory / '456.json').write_text(json.dumps(report))
        originals = {
            p: p.read_bytes() for p in self.library.directory.rglob('*.json')
        }
        preview = migrate(self.library)
        self.assertEqual(preview['files_to_write'], 7)
        self.assertFalse(self.library.object_path('bottle').exists())
        migrate(self.library, apply=True)
        for name in ['cup', 'bottle']:
            self.assertEqual(self.library.object(name), self.value)
            self.assertEqual(
                json.loads(
                    (
                        self.models
                        / name
                        / 'grasp_profile_backups/legacy-123.json'
                    ).read_text()
                )['configuration'],
                self.value,
            )
        self.assertTrue((self.models / 'cup/teach_commands/456.json').exists())
        self.assertTrue(
            (self.models / 'bottle/grasp_profile_commands/456.json').exists()
        )
        self.assertEqual({p: p.read_bytes() for p in originals}, originals)
        self.assertEqual(migrate(self.library, apply=True)['files_to_write'], 0)

    def test_existing_local_edits_survive_migration_and_history_conflict_aborts(
        self
    ):
        changed = copy.deepcopy(self.value)
        changed['grasps'][0]['name'] = '本地修改'
        self.library.write_object('bottle', changed, revision(self.value))
        migrate(self.library, apply=True)
        self.assertEqual(self.library.object('bottle'), changed)
        self.library.legacy.write_text(json.dumps({'cup': {'original': True}}))
        (self.models / 'cup/taught_profile_legacy.json').write_text('{}')
        with self.assertRaisesRegex(ValueError, '不同内容'):
            migrate(self.library, apply=True)


if __name__ == '__main__':
    unittest.main()
