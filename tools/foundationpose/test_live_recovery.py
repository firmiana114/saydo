from pathlib import Path
import tempfile
import time
from types import SimpleNamespace
import unittest
from unittest.mock import patch

import live_tracker


class RecoveryLifecycleTests(unittest.TestCase):
    def test_loss_requeues_latest_frame_and_pause_cancels_recovery(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            mesh = root / 'model.obj'
            mesh.write_text('# fixture for state-machine test\n')
            args = SimpleNamespace(mesh=mesh, grasp_config=live_tracker.ROOT / 'config/grasp/trial.json', serial='test', detector_class='bottle', auto_recover=True, recovery_interval=2.)
            with patch.object(live_tracker, 'ROOT', root):
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
            self.assertIsNone(tracker.command[2], 'recovery must run detection rather than reuse old box')
            self.assertEqual(tracker.state['recovery_attempts'], 1)
            tracker.pause()
            self.assertIsNone(tracker.command)
            self.assertFalse(tracker.recover_if_due())
            self.assertEqual(tracker.state['state'], 'paused')

    def test_zero_cooldown_retries_immediately_without_reusing_failed_frame(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            mesh = root / 'model.obj'
            mesh.write_text('# fixture\n')
            args = SimpleNamespace(mesh=mesh, grasp_config=live_tracker.ROOT / 'config/grasp/trial.json', serial='test', detector_class='bottle', auto_recover=True, recovery_interval=0.)
            with patch.object(live_tracker, 'ROOT', root):
                tracker = live_tracker.Tracker(args)
            tracker.state['model_ready'] = True
            tracker.latest = {'time': time.time(), 'sequence': 7}
            tracker.last_attempt_sequence = 7
            tracker.mark_lost('simulated loss')
            self.assertFalse(tracker.recover_if_due(), 'do not repeatedly retry the same failed frame')
            tracker.latest = {'time': time.time(), 'sequence': 8}
            self.assertTrue(tracker.recover_if_due(), 'fresh frame should retry without sleeping')
            self.assertEqual(tracker.command[1]['sequence'], 8)


if __name__ == '__main__':
    unittest.main()
