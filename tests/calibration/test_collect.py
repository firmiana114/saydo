import copy
import json
from pathlib import Path
import tempfile
import unittest
from unittest.mock import patch

import cv2
import numpy as np
from saydo.calibration import collect


def sample(sample_id, x=0, degrees=0):
    t = np.eye(4)
    t[:3, :3] = cv2.Rodrigues(np.array([0., 0., np.radians(degrees)]))[0]
    t[0, 3] = x
    return {
        'sample_id': sample_id,
        'status': 'candidate',
        'base_T_flange': t.tolist(),
        'board': {'inner_corners': [8, 11], 'square_size_m': .03},
        'camera': {'serial': 'test-only', 'size': [1280, 720]},
        'preflight': {
            'get_current_tool_frame': {'tool_name': 'he_ref', 'pose': [0] * 6},
            'get_current_work_frame': {'frame_name': 'Base', 'pose': [0] * 6},
            'get_arm_software_info': {'Product_version': 'RM75-BI'},
        },
        'corners_px': [[1, 1]] * 88,
        'board_pose_candidates': [{'reprojection_rms_px': .5}],
        'stationarity': {},
    }


class CollectionTests(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        self.addCleanup(self.tmp.cleanup)
        self.root = Path(self.tmp.name)
        self.reference = sample('fit')
        self.write(self.reference)
        self.plan = {
            'training_candidate_ids': ['fit'],
            'validation_candidate_ids': [],
            'validation_target_count': 5,
        }
        collect.atomic_json(self.root / 'dataset_split.json', self.plan)

    def write(self, value):
        path = self.root / value['sample_id']
        path.mkdir(exist_ok=True)
        collect.atomic_json(path / 'sample.json', value)
        return path

    def mock_robot(self, x):
        def query(command):
            if command == 'get_current_arm_state':
                return {
                    'reply': {
                        'arm_state': {
                            'err': [0], 'pose': [int(x * 1e6), 0, 0, 0, 0, 0]
                        }
                    }
                }
            return {'reply': self.reference['preflight'][command]}

        return query

    def test_duplicate_needs_both_translation_and_rotation(self):
        ref = {'fit': self.reference}
        self.assertTrue(
            collect.comparisons(
                sample('a', .005, 2)['base_T_flange'], ref
            )[0]['near_duplicate']
        )
        self.assertFalse(
            collect.comparisons(
                sample('b', .02, 2)['base_T_flange'], ref
            )[0]['near_duplicate']
        )
        self.assertFalse(
            collect.comparisons(
                sample('c', .005, 12)['base_T_flange'], ref
            )[0]['near_duplicate']
        )

    def test_duplicate_skips_camera_and_does_not_append_validation(self):
        with patch.object(
            collect.Robot, 'query', side_effect=self.mock_robot(.001)
        ), patch.object(
            collect, 'pause_preview'
        ) as preview, patch.object(collect, 'run_capture') as capture:
            result = collect.collect_once(self.root, settle_seconds=0)
        self.assertEqual(result['state'], 'duplicate')
        preview.assert_not_called()
        capture.assert_not_called()
        self.assertEqual(
            json.loads((self.root / 'dataset_split.json').read_text()),
            self.plan,
        )

    def test_capture_calls_once_and_freezes_fitting_pool(self):
        new = sample('new', .03)

        def capture(*args):
            return self.write(new), copy.deepcopy(new)

        with patch.object(
            collect.Robot, 'query', side_effect=self.mock_robot(.03)
        ), patch.object(
            collect, 'pause_preview', return_value='idle'
        ), patch.object(collect, 'run_capture', side_effect=capture) as call:
            result = collect.collect_once(self.root, settle_seconds=0)
        call.assert_called_once()
        self.assertEqual(result['state'], 'captured')
        plan = json.loads((self.root / 'dataset_split.json').read_text())
        self.assertEqual(plan['training_candidate_ids'], ['fit'])
        self.assertEqual(plan['validation_candidate_ids'], ['new'])

    def test_post_capture_duplicate_is_rejected(self):
        new = sample('new', .001)

        def capture(*args):
            return self.write(new), copy.deepcopy(new)

        with patch.object(
            collect.Robot, 'query', side_effect=self.mock_robot(.05)
        ), patch.object(
            collect, 'pause_preview', return_value='idle'
        ), patch.object(collect, 'run_capture', side_effect=capture):
            result = collect.collect_once(self.root, settle_seconds=0)
        self.assertEqual(result['state'], 'rejected')
        self.assertEqual(
            json.loads((self.root / 'new/sample.json').read_text())['status'],
            'rejected',
        )
        self.assertEqual(
            json.loads((self.root / 'dataset_split.json').read_text()),
            self.plan,
        )

    def test_bad_camera_parameters_are_rejected(self):
        new = sample('new', .03)
        new['camera']['serial'] = 'different-camera'
        with self.assertRaises(ValueError):
            collect.check_compatibility(self.reference, new)

    def test_quality_rejection_is_not_retried_or_added(self):
        new = sample('new', .03)
        new.update(status='rejected', error='not stationary')

        def capture(*args):
            return self.write(new), copy.deepcopy(new)

        with patch.object(
            collect.Robot, 'query', side_effect=self.mock_robot(.03)
        ), patch.object(
            collect, 'pause_preview', return_value='idle'
        ), patch.object(collect, 'run_capture', side_effect=capture) as call:
            result = collect.collect_once(self.root, settle_seconds=0)
        call.assert_called_once()
        self.assertEqual(result['state'], 'rejected')
        self.assertEqual(
            json.loads((self.root / 'dataset_split.json').read_text()),
            self.plan,
        )

    def test_interrupted_admission_is_recovered_once(self):
        value = sample('pending', .03)
        value['collection_admission'] = {
            'role': 'validation',
            'passed': True,
            'training_candidate_ids': ['fit'],
        }
        self.write(value)
        for _ in range(2):
            plan = collect.load_plan(self.root, collect.load_samples(self.root))
            self.assertEqual(plan['validation_candidate_ids'], ['pending'])
            self.assertEqual(plan['training_candidate_ids'], ['fit'])

    def test_full_validation_set_does_not_touch_hardware(self):
        self.write(sample('v1', .03))
        self.plan.update(
            validation_candidate_ids=['v1'], validation_target_count=1
        )
        collect.atomic_json(self.root / 'dataset_split.json', self.plan)
        with patch.object(collect, 'Robot') as robot, patch.object(
            collect, 'run_capture'
        ) as capture:
            result = collect.collect_once(self.root)
        robot.assert_not_called()
        capture.assert_not_called()
        self.assertEqual(result['state'], 'complete')

    def test_corrupt_split_does_not_get_silently_recreated(self):
        self.plan['validation_candidate_ids'] = ['missing']
        collect.atomic_json(self.root / 'dataset_split.json', self.plan)
        with self.assertRaises(ValueError):
            collect.load_plan(self.root, collect.load_samples(self.root))
        self.assertEqual(
            json.loads((self.root / 'dataset_split.json').read_text()),
            self.plan,
        )


if __name__ == '__main__':
    unittest.main()
