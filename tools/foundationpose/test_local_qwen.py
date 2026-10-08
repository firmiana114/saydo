import json
from pathlib import Path
import tempfile
import unittest
from unittest.mock import Mock, patch
from local_qwen import parse_boxes, read_qwen_settings, LocalQwenDetector


class QwenTests(unittest.TestCase):
    def test_normalized_boxes_duplicate_flat_and_negative(self):
        r = parse_boxes(
            '{"boxes":[[100,200,200,400],[100,200,200,400]]}', 640, 360, .5
        )
        self.assertEqual(len(r), 1)
        self.assertEqual(r[0]['raw_box'], [64, 72, 128, 144])
        self.assertNotIn('score', r[0])
        self.assertEqual(
            parse_boxes('{"boxes":[100,200,200,400]}', 640, 360, .5), r
        )
        self.assertEqual(
            parse_boxes('```json\n{"boxes":[]}\n```', 640, 360, .5), []
        )

    def test_malformed_out_of_range_reversed_and_huge_boxes(self):
        for text in [
            '{"boxes":[[true,1,10,20]]}',
            '{"boxes":[[-1,1,10,20]]}',
            '{"boxes":[[10,20,1,3]]}',
            '{"boxes":[[0,0,NaN,20]]}',
            '{"boxes":[],"command":"move"}',
            'not json',
        ]:
            with self.assertRaises(ValueError):
                parse_boxes(text, 640, 360, .5)
        self.assertEqual(
            parse_boxes('{"boxes":[[0,0,1000,1000]]}', 640, 360, .5), []
        )

    def test_settings_are_model_local(self):
        with tempfile.TemporaryDirectory() as d:
            self.assertIsNone(read_qwen_settings(d))
            p = Path(d) / 'qwen_detection.json'
            p.write_text(
                json.dumps(
                    {'prompt': 'test tube', 'max_box_area_fraction': .15}
                )
            )
            self.assertEqual(read_qwen_settings(d)['prompt'], 'test tube')
            p.write_text(
                json.dumps(
                    {'prompt': 'test tube', 'max_box_area_fraction': True}
                )
            )
            with self.assertRaises(ValueError):
                read_qwen_settings(d)

    def test_resident_worker_is_reused_and_closed(self):
        d = LocalQwenDetector()
        d.process = Mock()
        d.process.poll.return_value = None
        d.ready = {'event': 'ready'}
        with patch('local_qwen.subprocess.Popen') as spawn:
            d.ensure_loaded()
            spawn.assert_not_called()
        p = d.process
        d.close()
        p.terminate.assert_called_once()
        self.assertIsNone(d.process)
        with self.assertRaises(ValueError):
            d.ensure_loaded()
