"""Restart helper decisions, without touching any process, port or device."""

import os
import pathlib
import tempfile
import unittest
from unittest import mock

from scripts import restart_tracker as restart

# setUp replaces it so tests never lock the real service directory.
REAL_ACQUIRE_LOCK = restart.acquire_lock


class RestartTrackerTests(unittest.TestCase):
    def setUp(self):
        patcher = mock.patch.object(restart, 'acquire_lock')
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
        with mock.patch.object(
            restart, 'current_pid', return_value=123
        ), mock.patch.object(
            restart, 'check_idle', return_value=['存在夹持记录']
        ) as check, mock.patch.object(
            restart, 'stop'
        ) as stop, mock.patch.object(restart, 'start') as start:
            self.assertEqual(restart.main([]), 1)
            stop.assert_not_called()
            start.assert_not_called()
            start.return_value = (object(), pathlib.Path('log'))
            with mock.patch.object(
                restart,
                'wait_ready',
                return_value={'model_ready': True, 'error': None},
            ), mock.patch.object(restart, 'ROOT', pathlib.Path('.')):
                self.assertEqual(
                    restart.main(['--force', '--', '--detector', 'ssdlite']), 0
                )
            check.assert_called_once()
            stop.assert_called_once_with(123)
            start.assert_called_once_with(['--detector', 'ssdlite'])

    def test_unreadable_status_refuses_restart_without_force(self):
        with mock.patch.object(
            restart, 'current_pid', return_value=123
        ), mock.patch.object(
            restart, 'check_idle', side_effect=OSError('refused')
        ), mock.patch.object(restart, 'stop') as stop, mock.patch.object(
            restart, 'start'
        ) as start:
            self.assertEqual(restart.main([]), 1)
            stop.assert_not_called()
            start.assert_not_called()

    def test_concurrent_restart_is_refused_before_touching_the_service(self):
        self.lock.side_effect = RuntimeError('另一次重启正在进行')
        with mock.patch.object(
            restart, 'current_pid'
        ) as pid, mock.patch.object(restart, 'stop') as stop, mock.patch.object(
            restart, 'start'
        ) as start:
            self.assertEqual(restart.main(['--force']), 1)
        pid.assert_not_called()
        stop.assert_not_called()
        start.assert_not_called()

    def test_second_lock_holder_is_rejected(self):
        with tempfile.TemporaryDirectory() as directory, mock.patch.object(
            restart, 'LIVE', pathlib.Path(directory) / 'live'
        ):
            first = REAL_ACQUIRE_LOCK()
            with self.assertRaisesRegex(RuntimeError, '另一次重启正在进行'):
                REAL_ACQUIRE_LOCK()
            first.close()
            REAL_ACQUIRE_LOCK().close()

    @unittest.skipUnless(
        pathlib.Path('/proc/self/cmdline').exists(), 'requires Linux /proc'
    )
    def test_stop_refuses_a_process_that_is_not_the_tracker(self):
        with mock.patch.object(
            restart.os, 'kill'
        ) as kill, self.assertRaisesRegex(RuntimeError, '不是跟踪服务'):
            restart.stop(os.getpid())
        kill.assert_not_called()


if __name__ == '__main__':
    unittest.main()
