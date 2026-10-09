"""相机校正的数值换算与文件检查。"""

import pathlib
import tempfile
import unittest

import numpy as np
import yaml

from saydo.perception import vision_refinement


def record(**changes):
    value = {
        'schema_version': 1,
        'arm': 'arm2',
        'acceptance_status': 'user_accepted_usable',
        'correction': {
            'rotation_vector_rad': [0., 0., 0.],
            'translation_m': [0., 0., .012],
        },
        'depth_quadratic_per_m': -.035,
    }
    value.update(changes)
    return value


class LoadTests(unittest.TestCase):
    def setUp(self):
        directory = tempfile.TemporaryDirectory()
        self.addCleanup(directory.cleanup)
        self.path = pathlib.Path(directory.name) / 'refinement.yaml'

    def load(self, value, arm='arm2'):
        self.path.write_text(yaml.safe_dump(value))
        return vision_refinement.load(self.path, arm)

    def test_valid_record_is_returned(self):
        self.assertEqual(self.load(record())['depth_quadratic_per_m'], -.035)

    def test_record_measured_for_another_arm_is_refused(self):
        with self.assertRaisesRegex(ValueError, '为 arm2 测的.*不能用于 arm1'):
            self.load(record(), arm='arm1')

    def test_unaccepted_or_unknown_version_is_refused(self):
        for value in (
            record(acceptance_status='pending'), record(schema_version=2)
        ):
            with self.subTest(value=value), self.assertRaises(ValueError):
                self.load(value)

    def test_implausibly_large_correction_is_refused_with_the_numbers(self):
        with self.assertRaisesRegex(ValueError, '平移 80.0 毫米'):
            self.load(
                record(
                    correction={
                        'rotation_vector_rad': [0, 0, 0],
                        'translation_m': [0, 0, .08],
                    }
                )
            )
        with self.assertRaisesRegex(ValueError, '超出合理范围'):
            self.load(record(depth_quadratic_per_m=.5))

    def test_missing_or_non_finite_numbers_are_refused(self):
        for value in (
            record(correction={'translation_m': [0, 0, 0]}),
            record(depth_quadratic_per_m=None),
            record(
                correction={
                    'rotation_vector_rad': [0, float('nan'), 0],
                    'translation_m': [0, 0, 0],
                }
            ),
        ):
            with self.subTest(value=value), self.assertRaisesRegex(
                ValueError, '缺失或无效'
            ):
                self.load(value)


class GeometryTests(unittest.TestCase):
    def test_rotation_vector_matches_a_known_rotation(self):
        quarter_turn = vision_refinement.rotation_matrix([0, 0, np.pi / 2])
        np.testing.assert_allclose(
            quarter_turn @ [1, 0, 0], [0, 1, 0], atol=1e-12
        )
        np.testing.assert_allclose(
            vision_refinement.rotation_matrix([0, 0, 0]), np.eye(3)
        )

    def test_corrected_point_is_shifted_then_rotated(self):
        value = record(
            correction={
                'rotation_vector_rad': [0, 0, np.pi / 2],
                'translation_m': [.01, 0, .012],
            }
        )
        transform = vision_refinement.camera_T_measured(value)
        # 量到的点 (0.1, 0, 0.5)：先加平移得 (0.11, 0, 0.512)，再绕 Z 转
        # 90 度得 (0, 0.11, 0.512)。
        np.testing.assert_allclose(
            transform @ [.1, 0, .5, 1], [0, .11, .512, 1], atol=1e-12
        )

    def test_refinement_is_applied_on_the_camera_side_of_the_handeye(self):
        flange_T_camera = np.eye(4)
        flange_T_camera[:3, 3] = [.075, .063, .012]
        refined = vision_refinement.refined_flange_T_camera(
            flange_T_camera, record()
        )
        # 相机沿自己的光轴前移 12 毫米。
        np.testing.assert_allclose(refined[:3, 3], [.075, .063, .024])
        np.testing.assert_array_equal(
            vision_refinement.refined_flange_T_camera(flange_T_camera, None),
            flange_T_camera,
        )

    def test_depth_correction_grows_with_the_square_of_distance(self):
        depth = np.array([[0., .2], [.5, 1.]], dtype=np.float32)
        corrected = vision_refinement.corrected_depth(depth, -.035)
        # 0.5 米处读远了 0.035×0.25 米，即 8.75 毫米；没有深度的像素不变。
        np.testing.assert_allclose(
            corrected, [[0., .1986], [.49125, .965]], atol=1e-6
        )
        self.assertEqual(corrected.dtype, np.float32)
        self.assertIs(vision_refinement.corrected_depth(depth, 0.), depth)


if __name__ == '__main__':
    unittest.main()
