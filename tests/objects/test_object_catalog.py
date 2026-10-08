import json
from pathlib import Path
import tempfile
import unittest

from saydo.objects.object_catalog import scan_objects, select_object


class CatalogTests(unittest.TestCase):
    def test_known_models_missing_models_and_metadata(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            for name in [
                'ob_yibao_350ml',
                'mug_iphone',
                'unknown',
                'capture_only',
                '.backup',
            ]:
                (root / name).mkdir()
            for name in ['ob_yibao_350ml', 'mug_iphone', 'unknown', '.backup']:
                (root / name / 'model').mkdir()
                (root / name / 'model/model.obj').write_text('# mesh fixture')
            (root / 'unknown/tracking.json').write_text(
                json.dumps({'name': '遥控器', 'detector_class': 'remote'})
            )
            rows = {row['id']: row for row in scan_objects(root)}
            self.assertNotIn('.backup', rows)
            self.assertEqual(rows['mug_iphone']['detector_class'], 'cup')
            self.assertEqual(rows['unknown']['name'], '遥控器')
            self.assertFalse(rows['capture_only']['available'])
            self.assertEqual(
                select_object(root, 'ob_yibao_350ml')['detector_class'],
                'bottle',
            )
            with self.assertRaises(ValueError):
                select_object(root, '../escape')

    def test_paths_cannot_escape_and_millimetres_are_not_implicitly_selected(
        self
    ):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            (root / 'bad').mkdir()
            (root / 'model_mm.obj').write_text('# fixture')
            (root / 'bad/tracking.json').write_text(
                json.dumps({'mesh': '../model_mm.obj'})
            )
            self.assertFalse(scan_objects(root)[0]['available'])
            (root / 'bad/tracking.json').unlink()
            (root / 'bad/model_mm.obj').write_text('# millimetres')
            self.assertFalse(scan_objects(root)[0]['available'])


if __name__ == '__main__':
    unittest.main()
