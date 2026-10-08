import unittest

import numpy as np

from geometry_checks import camera_pose, depth_millimeters, duplicate, rigid


class GeometryTests(unittest.TestCase):
    def test_rotated_tcp_offset_is_not_world_z_subtraction(self):
        base_T_flange = np.array(
            [
                [0., 0., 1., .4],
                [0., 1., 0., .2],
                [-1., 0., 0., .6],
                [0., 0., 0., 1.],
            ]
        )
        flange_T_tcp = np.eye(4)
        flange_T_tcp[2, 3] = .2225
        flange_T_camera = np.eye(4)
        flange_T_camera[:3, 3] = [.075, .063, .012]
        actual = camera_pose(
            base_T_flange @ flange_T_tcp, flange_T_tcp, flange_T_camera
        )
        np.testing.assert_allclose(
            actual, base_T_flange @ flange_T_camera, atol=1e-12
        )

    def test_depth_scale_and_invalid_zero(self):
        raw = np.array([[0, 1234, 2000]], dtype=np.uint16)
        np.testing.assert_array_equal(
            depth_millimeters(raw, .0001), [[0, 123, 200]]
        )
        with self.assertRaises(ValueError):
            depth_millimeters(raw, 1.)

    def test_duplicate_requires_both_small_translation_and_rotation(self):
        sample = {
            'sample_id': 'one',
            'base_T_color_optical_camera': np.eye(4).tolist(),
        }
        self.assertIsNotNone(duplicate(np.eye(4), [sample]))
        moved = np.eye(4)
        moved[0, 3] = .011
        self.assertIsNone(duplicate(moved, [sample]))
        rotated = np.array(
            [
                [0., -1., 0., 0.],
                [1., 0., 0., 0.],
                [0., 0., 1., 0.],
                [0., 0., 0., 1.],
            ]
        )
        self.assertIsNone(duplicate(rotated, [sample]))

    def test_reflection_and_nonfinite_rejected(self):
        for value in (np.diag([-1, 1, 1, 1]), np.full((4, 4), np.nan)):
            with self.assertRaises(ValueError):
                rigid(value)


if __name__ == '__main__':
    unittest.main()
