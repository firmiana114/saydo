import json
import pathlib
import tempfile
import unittest

from saydo.workbench import live_events


class EventLogTests(unittest.TestCase):
    def test_flush_unicode_and_bounded_rotation(self):
        with tempfile.TemporaryDirectory() as directory:
            path = pathlib.Path(directory) / 'events.jsonl'
            log = live_events.EventLog(path)
            log.handler.maxBytes = 512
            log.handler.backupCount = 2
            for i in range(20):
                log.write(
                    'attempt_failed',
                    generation=i,
                    error='追帧超时',
                    stage='catchup',
                )
            self.assertEqual(
                json.loads(path.read_text().splitlines()[-1])['generation'], 19
            )
            self.assertLessEqual(
                len(list(pathlib.Path(directory).iterdir())), 3
            )
            log.close()


if __name__ == '__main__':
    unittest.main()
