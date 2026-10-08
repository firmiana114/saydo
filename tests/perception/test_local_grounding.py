import json
from pathlib import Path
import tempfile
import unittest
from unittest.mock import Mock, patch
import numpy as np
from saydo.perception.local_grounding import read_local_detection, candidates_from_predictions, LocalGroundingDetector
from saydo.objects.object_catalog import scan_objects
from saydo.scene.scene_perception import automatic_candidates


class LocalDetectionTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.addCleanup(self.temp.cleanup)
        self.root = Path(self.temp.name)
        self.settings = {
            'backend': 'grounding_dino',
            'prompt': 'a transparent glass test tube.',
            'box_threshold': .4,
            'text_threshold': .2,
            'max_box_area_fraction': .15,
        }

    def test_model_local_settings_and_catalog_without_changing_coco_label(self):
        model = self.root / 'tube'
        model.mkdir()
        (model / 'model.obj').write_text('mesh fixture')
        self.assertIsNone(read_local_detection(model))
        (model / 'local_detection.json').write_text(json.dumps(self.settings))
        value = read_local_detection(model)
        self.assertEqual(value['backend'], 'grounding_dino')
        self.assertEqual(len(value['revision']), 64)
        row = scan_objects(self.root)[0]
        self.assertTrue(row['available'])
        self.assertIsNone(row['detector_class'])
        self.assertEqual(
            row['local_detection']['prompt'], self.settings['prompt']
        )

    def test_bad_settings_and_symlinks_rejected(self):
        p = self.root / 'local_detection.json'
        for change in [
            {'box_threshold': float('nan')},
            {'text_threshold': True},
            {'backend': 'remote_api'},
            {'prompt': ''},
            {'extra': True},
        ]:
            p.write_text(json.dumps({**self.settings, **change}))
            with self.assertRaises(ValueError):
                read_local_detection(self.root)
        p.unlink()
        other = self.root / 'other.json'
        other.write_text(json.dumps(self.settings))
        p.symlink_to(other)
        with self.assertRaises(ValueError):
            read_local_detection(self.root)

    def test_filtering_nms_keeps_distinct_instances_and_pixel_bounds(self):
        boxes = [
            [10, 20, 40, 90],
            [11, 21, 41, 91],
            [100, 20, 130, 90],
            [-2, 0, 12, 20],
            [200, 20, 220, 70],
            [1, 1, float('nan'), 50],
            [0, 0, 1, 1],
        ]
        scores = [.9, .8, .7, .6, .3, .9, .99]
        found = candidates_from_predictions(boxes, scores, .4, 640, 360)
        self.assertEqual(len(found), 3)
        self.assertEqual(found[0]['raw_box'], [10, 20, 40, 90])
        self.assertEqual(found[-1]['box'][0], 0)
        self.assertTrue(
            all(r['source'] == 'local_grounding_dino' for r in found)
        )
        self.assertEqual(candidates_from_predictions([], [], .4, 640, 360), [])
        self.assertEqual(
            candidates_from_predictions(
                [[0, 0, 640, 360], [94, 144, 303, 333]],
                [.9, .8],
                .4,
                640,
                360,
                .15,
            ),
            [],
        )

    def test_weights_missing_or_wrong_revision_never_import_model_code(self):
        d = self.root / 'work/foundationpose_setup/grounding-dino-tiny'
        d.mkdir(parents=True)
        (d / 'source.json').write_text(json.dumps({'revision': 'wrong'}))
        with self.assertRaisesRegex(ValueError, '版本'):
            LocalGroundingDetector(self.root).ensure_loaded()

    def test_scene_automatic_detection_uses_model_local_settings_without_llm(
        self
    ):
        detector = Mock()
        detector.settings_key = 'local_detection'
        detector.detect.return_value = [{'box': [1, 2, 30, 40]}]
        frame = {'sequence': 10}
        info = {'name': 'tube', 'local_detection': self.settings}
        result = automatic_candidates(frame, info, None, detector, lambda: None)
        self.assertEqual(result, [{'box': [1, 2, 30, 40]}])
        self.assertIs(detector.detect.call_args.args[0], frame)
        self.assertEqual(detector.detect.call_args.args[1], self.settings)
        with self.assertRaises(ValueError):
            automatic_candidates(
                frame, {'name': 'rack'}, None, detector, lambda: None
            )


if __name__ == '__main__':
    unittest.main()
