"""两底座相对位置的读取与换算。"""

import pathlib
import tempfile
import unittest

import numpy as np
import yaml

from saydo.robot import base_frames


def relation(matrix=None):
    if matrix is None:
        # 2 臂底座在 1 臂坐标里：绕竖直轴转 180 度，位于 (0, -0.5, 0)。
        matrix = np.diag([-1., -1., 1., 1.])
        matrix[1, 3] = -.5
    return {
        'schema_version': 1,
        'arm1_base_T_arm2_base': np.asarray(matrix).tolist(),
    }


class BaseFramesTests(unittest.TestCase):
    def setUp(self):
        directory = tempfile.TemporaryDirectory()
        self.addCleanup(directory.cleanup)
        self.root = pathlib.Path(directory.name)

    def write(self, value):
        path = self.root / base_frames.RELATION_FILE
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_text(yaml.safe_dump(value))

    def test_missing_file_means_no_relation(self):
        self.assertIsNone(base_frames.load(self.root))

    def test_point_seen_by_one_arm_is_expressed_in_the_other(self):
        self.write(relation())
        value = base_frames.load(self.root)
        to_first = base_frames.target_base_T_source_base(value, 'arm1', 'arm2')
        # 2 臂坐标里 (-0.2, -0.3, 0.1) 的点，在 1 臂坐标里是 (0.2, -0.2, 0.1)。
        np.testing.assert_allclose(
            to_first @ [-.2, -.3, .1, 1], [.2, -.2, .1, 1], atol=1e-12
        )
        to_second = base_frames.target_base_T_source_base(
            value, 'arm2', 'arm1'
        )
        np.testing.assert_allclose(to_second @ to_first, np.eye(4), atol=1e-12)
        np.testing.assert_array_equal(
            base_frames.target_base_T_source_base(value, 'arm2', 'arm2'),
            np.eye(4),
        )

    def test_non_rigid_or_far_fetched_relation_is_refused(self):
        scaled = np.eye(4)
        scaled[0, 0] = 1.1
        far = np.eye(4)
        far[0, 3] = 5.
        for matrix, pattern in ((scaled, '刚体变换'), (far, '5.00 米')):
            self.write(relation(matrix))
            with self.subTest(pattern=pattern), self.assertRaisesRegex(
                ValueError, pattern
            ):
                base_frames.load(self.root)

    def test_unknown_arm_is_refused(self):
        with self.assertRaisesRegex(ValueError, 'arm3'):
            base_frames.target_base_T_source_base(relation(), 'arm1', 'arm3')


if __name__ == '__main__':
    unittest.main()
