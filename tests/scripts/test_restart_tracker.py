"""Restart helper decisions, without touching any process, port or device."""

import os
from pathlib import Path
import tempfile
import unittest
from unittest.mock import patch

from scripts import restart_tracker as restart

# setUp replaces it so tests never lock the real service directory.
REAL_ACQUIRE_LOCK = restart.acquire_lock


class RestartTrackerTests(unittest.TestCase):
    def setUp(self):
        patcher = patch.object(restart, 'acquire_lock')
        self.lock = patcher.start()
        self.addCleanup(patcher.stop)

    def test_listening_pid_is_parsed_from_ss_output(self):
        line = (
            'LISTEN 0 5 192.168.101.65:8089 0.0.0.0:* '
            'users:(("python",pid=282318,fd=5))\n'
        )
        self.assertEqual(restart.listening_pid(line), 282318)
        self.assertIsNone(restart.listening_pid(''))

    def test_idle_service_has_no_blockers(self):
        status = {'grasp': {'busy': False, 'held_object': None}}
        runs = {
            'stopping': False,
            'runs': [
                {'run_id': 'a', 'state': s} for s in sorted(restart.FINISHED)
            ],
        }
        self.assertEqual(restart.blockers(status, runs), [])
        self.assertEqual(restart.blockers(status, None), [])
        self.assertEqual(restart.blockers({}, {'runs': []}), [])

    def test_busy_hold_and_unfinished_runs_block_restart(self):
        self.assertEqual(
            len(
                restart.blockers(
                    {'grasp': {'busy': True, 'held_object': None}}, None
                )
            ),
            1,
        )
        self.assertEqual(
            len(
                restart.blockers(
                    {'grasp': {'busy': False, 'held_object': {'hold_id': 'h'}}},
                    None,
                )
            ),
            1,
        )
        runs = {
            'stopping': True,
            'runs': [
                {'run_id': 'r1', 'state': 'running'},
                {'run_id': 'r2', 'state': 'succeeded'},
            ],
        }
        reasons = restart.blockers({'grasp': {}}, runs)
        self.assertEqual(len(reasons), 2)
        self.assertIn('r1', reasons[1])
        self.assertNotIn('r2', reasons[1])

    def test_blocked_restart_stops_nothing_and_force_skips_the_check(self):
        with patch.object(
            restart, 'current_pid', return_value=123
        ), patch.object(
            restart, 'check_idle', return_value=['存在夹持记录']
        ) as check, patch.object(restart, 'stop') as stop, patch.object(
            restart, 'start'
        ) as start:
            self.assertEqual(restart.main([]), 1)
            stop.assert_not_called()
            start.assert_not_called()
            start.return_value = (object(), Path('log'))
            with patch.object(
                restart,
                'wait_ready',
                return_value={'model_ready': True, 'error': None},
            ), patch.object(restart, 'ROOT', Path('.')):
                self.assertEqual(
                    restart.main(['--force', '--', '--detector', 'ssdlite']), 0
                )
            check.assert_called_once()
            stop.assert_called_once_with(123)
            start.assert_called_once_with(['--detector', 'ssdlite'])

    def test_unreadable_status_refuses_restart_without_force(self):
        with patch.object(
            restart, 'current_pid', return_value=123
        ), patch.object(
            restart, 'check_idle', side_effect=OSError('refused')
        ), patch.object(restart, 'stop') as stop, patch.object(
            restart, 'start'
        ) as start:
            self.assertEqual(restart.main([]), 1)
            stop.assert_not_called()
            start.assert_not_called()

    def test_concurrent_restart_is_refused_before_touching_the_service(self):
        self.lock.side_effect = RuntimeError('另一次重启正在进行')
        with patch.object(
            restart, 'current_pid'
        ) as pid, patch.object(restart, 'stop') as stop, patch.object(
            restart, 'start'
        ) as start:
            self.assertEqual(restart.main(['--force']), 1)
        pid.assert_not_called()
        stop.assert_not_called()
        start.assert_not_called()

    def test_second_lock_holder_is_rejected(self):
        with tempfile.TemporaryDirectory() as directory, patch.object(
            restart, 'LIVE', Path(directory) / 'live'
        ):
            first = REAL_ACQUIRE_LOCK()
            with self.assertRaisesRegex(RuntimeError, '另一次重启正在进行'):
                REAL_ACQUIRE_LOCK()
            first.close()
            REAL_ACQUIRE_LOCK().close()

    @unittest.skipUnless(
        Path('/proc/self/cmdline').exists(), 'requires Linux /proc'
    )
    def test_stop_refuses_a_process_that_is_not_the_tracker(self):
        with patch.object(restart.os, 'kill') as kill, self.assertRaisesRegex(
            RuntimeError, '不是跟踪服务'
        ):
            restart.stop(os.getpid())
        kill.assert_not_called()


if __name__ == '__main__':
    unittest.main()
