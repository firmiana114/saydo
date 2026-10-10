import pathlib
import tempfile
import time
import types
import unittest
from unittest import mock

from saydo.robot import arms
from saydo.workbench import live_tracker
from tests.robot import fake_controller


class RecoveryLifecycleTests(unittest.TestCase):
    def test_loss_requeues_latest_frame_and_pause_cancels_recovery(self):
        with tempfile.TemporaryDirectory() as directory:
            root = pathlib.Path(directory)
            mesh = root / 'model.obj'
            mesh.write_text('# fixture for state-machine test\n')
            # 跟踪器按项目根目录找抓取配置和配置里引用的标定文件，所以
            # 在临时根目录下自备一份，不读运行所在目录里本机的 config/。
            fake_controller.write_arm_config(
                root, arms.default(), '10.0.0.1', [0] * 7
            )
            args = types.SimpleNamespace(
                mesh=mesh,
                serial='test',
                detector_class='bottle',
                auto_recover=True,
                recovery_interval=2.,
            )
            with mock.patch.object(live_tracker, 'ROOT', root):
                tracker = live_tracker.Tracker(args)
            tracker.state['model_ready'] = True
            tracker.latest = {'time': time.time(), 'sequence': 7}
            tracker.mark_lost('simulated quality failure')
            self.assertEqual(tracker.state['state'], 'recovering')
            self.assertFalse(tracker.recover_if_due())
            tracker.retry_at = time.monotonic() - 1
            self.assertTrue(tracker.recover_if_due())
            self.assertEqual(tracker.state['state'], 'detecting')
            self.assertEqual(tracker.command[1]['sequence'], 7)
            self.assertIsNone(
                tracker.command[2],
                'recovery must run detection rather than reuse old box',
            )
            self.assertEqual(tracker.state['recovery_attempts'], 1)
            tracker.pause()
            self.assertIsNone(tracker.command)
            self.assertFalse(tracker.recover_if_due())
            self.assertEqual(tracker.state['state'], 'paused')

    def test_zero_cooldown_retries_immediately_without_reusing_failed_frame(
        self
    ):
        with tempfile.TemporaryDirectory() as directory:
            root = pathlib.Path(directory)
            mesh = root / 'model.obj'
            mesh.write_text('# fixture\n')
            fake_controller.write_arm_config(
                root, arms.default(), '10.0.0.1', [0] * 7
            )
            args = types.SimpleNamespace(
                mesh=mesh,
                serial='test',
                detector_class='bottle',
                auto_recover=True,
                recovery_interval=0.,
            )
            with mock.patch.object(live_tracker, 'ROOT', root):
                tracker = live_tracker.Tracker(args)
            tracker.state['model_ready'] = True
            tracker.latest = {'time': time.time(), 'sequence': 7}
            tracker.last_attempt_sequence = 7
            tracker.mark_lost('simulated loss')
            self.assertFalse(
                tracker.recover_if_due(),
                'do not repeatedly retry the same failed frame',
            )
            tracker.latest = {'time': time.time(), 'sequence': 8}
            self.assertTrue(
                tracker.recover_if_due(),
                'fresh frame should retry without sleeping',
            )
            self.assertEqual(tracker.command[1]['sequence'], 8)


if __name__ == '__main__':
    unittest.main()
