"""选相机与查占用：相机枚举和系统设备目录是替身，判断逻辑是真实代码。"""

import pathlib
import tempfile
import types
import unittest
from unittest import mock

from saydo.calibration import capture


def device(name, serial, asic):
    values = {
        capture.rs.camera_info.name: name,
        capture.rs.camera_info.serial_number: serial,
        capture.rs.camera_info.asic_serial_number: asic,
    }
    return types.SimpleNamespace(
        get_info=values.__getitem__,
        supports=lambda key: values.get(key) is not None,
    )


TWO_ARMS = [
    device('Intel RealSense D435I', '405622073572', '252443060505'),
    device('Intel RealSense D435I', '344422070678', '250343061041'),
    device('Intel RealSense D405', '218622270784', '243223071596'),
]


class SelectCameraTests(unittest.TestCase):
    def connect(self, devices):
        context = types.SimpleNamespace(query_devices=lambda: devices)
        patcher = mock.patch.object(
            capture.rs, 'context', return_value=context
        )
        self.addCleanup(patcher.stop)
        patcher.start()

    def test_serial_picks_that_arm_camera_among_two(self):
        self.connect(TWO_ARMS)
        self.assertEqual(
            capture.select_camera('344422070678'),
            {
                'serial': '344422070678',
                'name': 'Intel RealSense D435I',
                'asic_serial': '250343061041',
            },
        )

    def test_two_cameras_without_a_serial_are_refused_and_listed(self):
        self.connect(TWO_ARMS)
        with self.assertRaisesRegex(
            ValueError, '2 台.*405622073572、344422070678.*--serial'
        ):
            capture.select_camera()

    def test_single_camera_needs_no_serial(self):
        self.connect(TWO_ARMS[1:])
        self.assertEqual(capture.select_camera()['serial'], '344422070678')

    def test_missing_or_non_d435_serial_is_refused(self):
        self.connect(TWO_ARMS)
        for serial in ('000000000000', '218622270784'):
            with self.assertRaisesRegex(
                ValueError, f'没有找到序列号为 {serial}'
            ):
                capture.select_camera(serial)

    def test_camera_without_asic_serial_is_still_selected(self):
        self.connect([device('Intel RealSense D435I', '1', None)])
        self.assertIsNone(capture.select_camera('1')['asic_serial'])


class VideoNodeTests(unittest.TestCase):
    def setUp(self):
        directory = tempfile.TemporaryDirectory()
        self.addCleanup(directory.cleanup)
        root = pathlib.Path(directory.name)
        self.sysfs = root / 'class'
        self.dev = root / 'dev'
        self.sysfs.mkdir()
        self.dev.mkdir()
        self.devices = root / 'devices'
        self.add_camera('2-2.3', '252443060505', ['video2', 'video3'])
        self.add_camera('2-2.2', '250343061041', ['video14', 'video15'])

    def add_camera(self, port, asic, names):
        usb = self.devices / port
        usb.mkdir(parents=True)
        (usb / 'idVendor').write_text('8086\n')
        (usb / 'serial').write_text(asic + '\n')
        for name in names:
            node = usb / f'{port}:1.0' / 'video4linux' / name
            node.mkdir(parents=True)
            (self.sysfs / name).symlink_to(node)
            (self.dev / name).touch()

    def nodes(self, asic):
        return capture.video_nodes(asic, self.sysfs, self.dev)

    def test_only_the_nodes_of_that_camera_are_listed(self):
        self.assertEqual(
            self.nodes('250343061041'),
            [str(self.dev / 'video14'), str(self.dev / 'video15')],
        )

    def test_unidentifiable_camera_falls_back_to_every_node(self):
        everything = sorted(
            str(self.dev / name)
            for name in ('video2', 'video3', 'video14', 'video15')
        )
        self.assertEqual(self.nodes(None), everything)
        self.assertEqual(self.nodes('not-present'), everything)


class CameraUsersTests(unittest.TestCase):
    def run_fuser(self, stdout, returncode=0):
        result = types.SimpleNamespace(stdout=stdout, returncode=returncode)
        return mock.patch.object(
            capture.subprocess, 'run', return_value=result
        )

    def test_reports_other_processes_on_that_camera_only(self):
        camera = {'serial': '344422070678', 'asic_serial': '250343061041'}
        own = capture.os.getpid()
        with mock.patch.object(
            capture, 'video_nodes', return_value=['/dev/video14']
        ) as nodes, self.run_fuser(f' 407263 {own} 407263') as fuser:
            self.assertEqual(capture.camera_users(camera), [407263])
        nodes.assert_called_once_with('250343061041')
        self.assertEqual(fuser.call_args.args[0], ['fuser', '/dev/video14'])

    def test_free_camera_has_no_users(self):
        with mock.patch.object(
            capture, 'video_nodes', return_value=['/dev/video14']
        ), self.run_fuser('', returncode=1):
            self.assertEqual(capture.camera_users({'asic_serial': 'a'}), [])

    def test_fuser_failure_is_not_read_as_free(self):
        with mock.patch.object(
            capture, 'video_nodes', return_value=['/dev/video14']
        ), self.run_fuser('', returncode=127):
            with self.assertRaisesRegex(RuntimeError, '退出码 127'):
                capture.camera_users({'asic_serial': 'a'})


if __name__ == '__main__':
    unittest.main()
