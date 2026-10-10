"""共享内存里的原始画面：写入、读出，以及边写边读时不会读到半帧。"""

import pathlib
import tempfile
import threading
import time
import unittest

import numpy as np

from saydo.contracts import camera_frames

INTRINSICS = {
    'fx': 653.5, 'fy': 652.9, 'cx': 647., 'cy': 365.3,
    'distortion': [-.055, .059, -.0008, .0005, -.02],
    'distortion_model': 'distortion.inverse_brown_conrady',
}


class FrameTestCase(unittest.TestCase):
    def setUp(self):
        directory = tempfile.TemporaryDirectory()
        self.addCleanup(directory.cleanup)
        self.path = pathlib.Path(directory.name) / 'saydo-camera-test'

    def writer(self, width=64, height=36, depth_scale_m=.0001):
        writer = camera_frames.FrameWriter(
            self.path, width, height, INTRINSICS, '218622270784',
            'RealSense D405', depth_scale_m,
        )
        self.addCleanup(writer.close)
        return writer

    def reader(self):
        reader = camera_frames.FrameReader(self.path)
        self.addCleanup(reader.close)
        return reader


class RoundTripTests(FrameTestCase):
    def test_a_frame_is_read_back_exactly_with_its_description(self):
        rng = np.random.default_rng(3)
        bgr = rng.integers(0, 255, (36, 64, 3), dtype=np.uint8)
        depth = rng.integers(0, 60000, (36, 64), dtype=np.uint16)
        writer = self.writer()
        writer.write(bgr, depth, 7, 1000.25, 1000.28)
        reader = self.reader()
        frame = reader.read()
        self.assertTrue(np.array_equal(frame['bgr'], bgr))
        self.assertTrue(np.array_equal(frame['depth_raw'], depth))
        self.assertEqual(frame['depth_scale_m'], .0001)
        self.assertEqual(frame['sequence'], 7)
        self.assertEqual(
            (frame['captured_at'], frame['received_at']), (1000.25, 1000.28)
        )
        self.assertTrue(
            np.allclose(
                frame['K'],
                [[653.5, 0., 647.], [0., 652.9, 365.3], [0., 0., 1.]],
            )
        )
        self.assertTrue(
            np.allclose(frame['distortion'], INTRINSICS['distortion'])
        )
        self.assertEqual(
            frame['distortion_model'], 'distortion.inverse_brown_conrady'
        )
        self.assertEqual(
            (frame['serial'], frame['name']),
            ('218622270784', 'RealSense D405'),
        )
        # 读出来的是副本：之后再写不会改动已经取走的这一帧。
        writer.write(bgr * 0, depth * 0, 8, 1001., 1001.)
        self.assertTrue(np.array_equal(frame['bgr'], bgr))

    def test_only_newer_frames_are_returned(self):
        writer = self.writer()
        reader = self.reader()
        # 相机服务刚建好共享内存、还没写第一帧。
        self.assertIsNone(reader.read())
        image = np.zeros((36, 64, 3), np.uint8)
        depth = np.zeros((36, 64), np.uint16)
        writer.write(image, depth, 1, 10., 10.)
        self.assertEqual(reader.read()['sequence'], 1)
        self.assertIsNone(reader.read(after_sequence=1))
        writer.write(image, depth, 2, 11., 11.)
        self.assertEqual(reader.read(after_sequence=1)['sequence'], 2)

    def test_a_camera_without_depth_gives_colour_only(self):
        writer = self.writer(depth_scale_m=None)
        image = np.full((36, 64, 3), 9, np.uint8)
        writer.write(image, None, 1, 10., 10.)
        frame = self.reader().read()
        self.assertTrue(np.array_equal(frame['bgr'], image))
        self.assertIsNone(frame['depth_raw'])
        self.assertIsNone(frame['depth_scale_m'])

    def test_the_file_is_private_and_removed_on_close(self):
        writer = self.writer()
        self.assertEqual(self.path.stat().st_mode & 0o777, 0o600)
        self.assertFalse(self.path.with_name(self.path.name + '.tmp').exists())
        writer.close()
        self.assertFalse(self.path.exists())

    def test_missing_or_foreign_files_are_refused_with_a_reason(self):
        with self.assertRaisesRegex(
            camera_frames.CameraUnavailable, '接不上相机服务的共享内存'
        ):
            camera_frames.FrameReader(self.path)
        self.path.write_bytes(b'x' * 100)
        with self.assertRaisesRegex(
            camera_frames.CameraUnavailable, '不完整'
        ):
            camera_frames.FrameReader(self.path)
        self.path.write_bytes(b'x' * 4096)
        with self.assertRaisesRegex(
            camera_frames.CameraUnavailable, '格式与本程序不符'
        ):
            camera_frames.FrameReader(self.path)


class ConcurrencyTests(FrameTestCase):
    def test_a_reader_never_sees_half_a_frame(self):
        # 每一帧的所有像素都等于帧序号（取模），彩色与深度一致；读到
        # 半帧时同一幅图里会出现两种值。
        writer = self.writer(640, 360)
        reader = self.reader()
        stop = threading.Event()

        def write():
            sequence = 0
            while not stop.is_set():
                sequence += 1
                writer.write(
                    np.full((360, 640, 3), sequence % 251, np.uint8),
                    np.full((360, 640), sequence % 251, np.uint16),
                    sequence, float(sequence), float(sequence),
                )
                # 真实的相机每 66 毫秒才写一帧；这里只留 2 毫秒的空隙，
                # 比实际苛刻得多，但不是一刻不停地写。
                stop.wait(.002)

        thread = threading.Thread(target=write)
        thread.start()
        try:
            seen, last = 0, 0
            deadline = time.monotonic() + .6
            while time.monotonic() < deadline:
                frame = reader.read(last)
                if frame is None:
                    time.sleep(.0005)
                    continue
                seen += 1
                last = frame['sequence']
                value = last % 251
                self.assertTrue((frame['bgr'] == value).all())
                self.assertTrue((frame['depth_raw'] == value).all())
                self.assertEqual(frame['captured_at'], float(last))
        finally:
            stop.set()
            thread.join()
        self.assertGreater(seen, 20)


if __name__ == '__main__':
    unittest.main()
