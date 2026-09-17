import unittest
import numpy as np
from live_checks import checked_box, pose_health, bottle_candidates, recovery_due


class LiveChecks(unittest.TestCase):
    def test_recovery_waits_for_cooldown_fresh_frame_and_explicit_state(self):
        self.assertTrue(recovery_due('recovering', True, 12, 11, .1))
        for state in ['paused', 'selecting', 'choose_candidate', 'tracking', 'camera_error']:
            self.assertFalse(recovery_due(state, True, 12, 11, .1))
        self.assertFalse(recovery_due('recovering', True, 10, 11, .1))
        self.assertFalse(recovery_due('recovering', True, 12, 11, 3))
        self.assertFalse(recovery_due('recovering', False, 12, 11, .1))

    def test_detector_filters_classes_but_preserves_ambiguous_bottles(self):
        boxes = [[10, 10, 60, 160], [200, 10, 250, 160], [400, 20, 410, 50], [1, 1, 640, 360]]
        result = bottle_candidates(boxes, [44, 44, 44, 1], [.8, .9, .7, .99], 44)
        self.assertEqual(len(result), 2)
        self.assertEqual(bottle_candidates(boxes, [44] * 4, [.1] * 4, 44), [])

    def test_box_rejects_invalid_or_clipped_input(self):
        self.assertEqual(checked_box([10, 20, 80, 100], 640, 360), [10, 20, 80, 100])
        for box in ([0, 0, 700, 100], [5, 5, 4, 20], [0, 0, float('nan'), 99], [1, 2]):
            with self.assertRaises(ValueError):
                checked_box(box, 640, 360)

    def test_missing_depth_cannot_pass_health(self):
        pose = np.eye(4)
        pose[2, 3] = .5
        self.assertFalse(pose_health(pose, np.full((20, 20), .5), np.zeros((20, 20)))['good'])

    def test_depth_mismatch_and_large_pose_jump_are_rejected(self):
        pose = np.eye(4)
        pose[2, 3] = .5
        rendered = np.full((20, 20), .5)
        self.assertTrue(pose_health(pose, rendered, rendered.copy())['good'])
        self.assertFalse(pose_health(pose, rendered, np.full((20, 20), .7))['good'])
        previous = pose.copy()
        previous[0, 3] = .2
        self.assertFalse(pose_health(pose, rendered, rendered, previous)['good'])
        pose[0, 0] = np.nan
        self.assertFalse(pose_health(pose, rendered, rendered)['good'])


if __name__ == '__main__':
    unittest.main()
