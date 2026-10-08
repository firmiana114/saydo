import copy
import hashlib
import json
from pathlib import Path
import tempfile
import unittest

import cv2
import numpy as np
from saydo.calibration.solve import fit, load_dataset, rigid, solve_dataset


def transform(rv, position):
    t = np.eye(4)
    t[:3, :3] = cv2.Rodrigues(np.asarray(rv, dtype=float))[0]
    t[:3, 3] = position
    return t


def synthetic():
    rng = np.random.default_rng(24)
    handeye = transform([.2, -.3, .4], [.03, -.04, .22])
    board = transform([.15, .05, -.2], [.5, .1, 1.4])
    rows = []
    for i in range(13):
        gripper = transform(
            rng.uniform(-.6, .6, 3), rng.uniform([-.2, -.2, .1], [.3, .3, .5])
        )
        camera = np.linalg.inv(handeye) @ np.linalg.inv(gripper) @ board
        rows.append(
            {
                'sample_id': str(i),
                'base_T_flange': gripper.tolist(),
                'camera_T_board_candidate': camera.tolist(),
            }
        )
    return handeye, board, rows[:8], rows[8:]


@unittest.skipUnless(
    hasattr(cv2, 'calibrateHandEye'),
    'Run solver tests in work/handeye_solver_venv',
)
class SolverChecks(unittest.TestCase):
    def test_known_handeye_is_recovered_in_correct_direction(self):
        expected, _, training, validation = synthetic()
        result = solve_dataset(training, validation)
        np.testing.assert_allclose(
            result['flange_T_color_optical_camera'], expected, atol=1e-8
        )
        np.testing.assert_allclose(
            result['color_optical_camera_T_flange'],
            np.linalg.inv(expected),
            atol=1e-8,
        )
        self.assertLess(
            result['heldout_validation']['summary'][
                'translation_error_mm'
            ]['max'],
            1e-5,
        )
        self.assertFalse(result['validation_used_for_fitting_or_tuning'])
        self.assertFalse(result['deployed'])

    def test_validation_error_cannot_change_fit_or_reference(self):
        handeye, board, training, validation = synthetic()
        original = solve_dataset(training, validation)
        shifted = copy.deepcopy(validation)
        moved_board = board.copy()
        moved_board[0, 3] += .02
        for sample in shifted:
            sample['camera_T_board_candidate'] = (
                np.linalg.inv(handeye)
                @ np.linalg.inv(sample['base_T_flange'])
                @ moved_board
            ).tolist()
        changed = solve_dataset(training, shifted)
        np.testing.assert_allclose(
            changed['flange_T_color_optical_camera'],
            original['flange_T_color_optical_camera'],
            atol=1e-12,
        )
        np.testing.assert_allclose(
            changed['base_T_board_training_reference'],
            original['base_T_board_training_reference'],
            atol=1e-12,
        )
        self.assertAlmostEqual(
            changed['heldout_validation']['summary'][
                'translation_error_mm'
            ]['rms'],
            20,
            places=6,
        )

    def test_one_axis_rotation_is_rejected(self):
        samples = [
            {
                'sample_id': str(i),
                'base_T_flange': transform(
                    [0, 0, i * .2], [i * .1, 0, 0]
                ).tolist(),
                'camera_T_board_candidate': np.eye(4).tolist(),
            }
            for i in range(5)
        ]
        with self.assertRaisesRegex(ValueError, 'Degenerate'):
            fit(samples)

    def test_overlapping_split_is_rejected(self):
        _, _, training, validation = synthetic()
        validation[0]['sample_id'] = training[0]['sample_id']
        with self.assertRaisesRegex(ValueError, 'leakage'):
            solve_dataset(training, validation)

    def test_reflection_is_not_accepted_as_rotation(self):
        value = np.eye(4)
        value[0, 0] = -1
        with self.assertRaises(ValueError):
            rigid(value)

    def test_modified_source_image_is_rejected_before_fit(self):
        with tempfile.TemporaryDirectory() as folder:
            root = Path(folder)
            (root / 'fit').mkdir()
            (root / 'dataset_split.json').write_text(
                json.dumps(
                    {
                        'training_candidate_ids': ['fit'],
                        'validation_candidate_ids': ['val'],
                        'validation_target_count': 1,
                    }
                )
            )
            (root / 'fit/color.png').write_bytes(b'changed image')
            (root / 'fit/sample.json').write_text(
                json.dumps(
                    {
                        'sample_id': 'fit',
                        'status': 'candidate',
                        'image_sha256': hashlib.sha256(
                            b'original image'
                        ).hexdigest(),
                    }
                )
            )
            with self.assertRaisesRegex(ValueError, 'integrity'):
                load_dataset(root)


if __name__ == '__main__':
    unittest.main()
