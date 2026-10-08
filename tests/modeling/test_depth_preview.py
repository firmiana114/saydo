import json
import pathlib
import tempfile
import unittest

import cv2
import numpy as np

from saydo.modeling import depth_preview
from saydo.modeling import reexport_previews


class DepthPreviewTests(unittest.TestCase):
    def test_invalid_depth_is_black_and_scale_is_fixed(self):
        depth = np.array(
            [[0, -1, np.nan, np.inf, .1, .15, .8, 2]], dtype=np.float32
        )
        image = depth_preview.depth_color(depth)
        np.testing.assert_array_equal(
            image[0, :4], np.zeros((4, 3), dtype=np.uint8)
        )
        endpoints = cv2.applyColorMap(
            np.array([[0, 255]], dtype=np.uint8), cv2.COLORMAP_TURBO
        )
        np.testing.assert_array_equal(
            image[0, 4:6], np.repeat(endpoints[:, 0], 2, axis=0)
        )
        np.testing.assert_array_equal(
            image[0, 6:8], np.repeat(endpoints[:, 1], 2, axis=0)
        )

    def make_session(self, root):
        session = root / 'fixture'
        directory = session / 'views' / 'view1'
        directory.mkdir(parents=True)
        bgr = np.full((8, 12, 3), 180, np.uint8)
        raw = np.full((8, 12), 325, np.uint16)
        for name, image in [
            ('color.png', bgr),
            ('preview.jpg', bgr),
            ('depth_raw.png', raw),
            ('depth_mm.png', raw),
        ]:
            self.assertTrue(cv2.imwrite(str(directory / name), image))
        np.save(directory / 'depth_m.npy', raw.astype(np.float32) * .001)
        hashes = {p.name: reexport_previews.sha(p) for p in directory.iterdir()}
        report = {
            'status': 'candidate',
            'files_sha256': hashes,
            'retained_field': {'a': 123},
        }
        (directory / 'sample.json').write_text(json.dumps(report))
        return session, directory, hashes

    def test_reexport_preserves_measurements_and_backs_up_overwritten_files(
        self
    ):
        with tempfile.TemporaryDirectory() as tmp:
            session, directory, hashes = self.make_session(pathlib.Path(tmp))
            old_report = (directory / 'sample.json').read_bytes()
            result = reexport_previews.reexport_session(session)
            backup = pathlib.Path(result['records'][0]['backup'])
            self.assertEqual((backup / 'sample.json').read_bytes(), old_report)
            self.assertEqual(
                reexport_previews.sha(backup / 'preview.jpg'),
                hashes['preview.jpg'],
            )
            for name in [
                'color.png', 'depth_raw.png', 'depth_mm.png', 'depth_m.npy'
            ]:
                self.assertEqual(
                    reexport_previews.sha(directory / name), hashes[name]
                )
            self.assertEqual(
                cv2.imread(str(directory / 'preview.jpg')).shape, (8, 24, 3)
            )
            report = json.loads((directory / 'sample.json').read_text())
            self.assertEqual(report['retained_field'], {'a': 123})
            for name, expected in report['files_sha256'].items():
                self.assertEqual(
                    reexport_previews.sha(directory / name), expected
                )
            first_preview = reexport_previews.sha(directory / 'preview.jpg')
            reexport_previews.reexport_session(session)
            self.assertEqual(
                first_preview, reexport_previews.sha(directory / 'preview.jpg')
            )

    def test_corrupt_source_rejected_before_overwrite(self):
        with tempfile.TemporaryDirectory() as tmp:
            session, directory, hashes = self.make_session(pathlib.Path(tmp))
            (directory / 'depth_m.npy').write_bytes(b'corrupt')
            with self.assertRaisesRegex(ValueError, 'hash mismatch'):
                reexport_previews.reexport_session(session)
            self.assertEqual(
                reexport_previews.sha(directory / 'preview.jpg'),
                hashes['preview.jpg'],
            )
            self.assertFalse((directory / 'depth_color.png').exists())


if __name__ == '__main__':
    unittest.main()
