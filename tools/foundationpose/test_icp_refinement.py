"""Analytic RGB-D scenes and publication checks; no camera or robot connection."""
import time
import unittest
from unittest.mock import patch

import cv2
import numpy as np

from icp_refinement import ICPSettings, centered_pose, refine_pose, refine_observation
from live_checks import pose_health


def scene(width=320, height=240):
    K = np.array([[width*1.3, 0, width/2], [0, width*1.3, height/2], [0, 0, 1.]])
    y, x = np.indices((height, width))
    rays = np.stack(((x-K[0, 2])/K[0, 0], (y-K[1, 2])/K[1, 1], np.ones_like(x)), axis=-1)
    radii = np.array([.09, .055, .07])

    def render(pose):
        direction = (rays @ pose[:3, :3])/radii
        origin = (-pose[:3, 3] @ pose[:3, :3])/radii
        a = np.sum(direction**2, axis=-1)
        b = 2*np.sum(direction*origin, axis=-1)
        c = np.sum(origin**2)-1
        discriminant = b*b-4*a*c
        return np.where(discriminant >= 0, (-b-np.sqrt(np.maximum(discriminant, 0)))/(2*a), 0).astype(np.float32)

    truth = np.eye(4)
    truth[:3, :3] = cv2.Rodrigues(np.array([.2, -.15, .1]))[0]
    truth[:3, 3] = [.015, -.012, .55]
    initial = truth.copy()
    initial[:3, :3] = cv2.Rodrigues(np.array([.008, -.01, .005]))[0] @ truth[:3, :3]
    initial[:3, 3] += [.002, -.001, .003]
    return K, truth, initial, render


class ICPRefinementTests(unittest.TestCase):
    def setUp(self):
        self.K, self.truth, self.initial, self.render = scene()
        self.depth = self.render(self.truth)
        self.rendered = self.render(self.initial)
        self.settings = ICPSettings(budget_ms=50)

    def refine(self, **kwargs):
        return refine_pose(self.initial, self.rendered, self.depth, self.K,
                           settings=self.settings, **kwargs)

    def test_recovers_small_known_rigid_error(self):
        candidate, report = self.refine()
        self.assertTrue(report['accepted'], report)
        self.assertLess(np.linalg.norm(candidate[:3, 3]-self.truth[:3, 3]), .0005)
        angle = np.arccos(np.clip((np.trace(candidate[:3, :3].T @ self.truth[:3, :3])-1)/2, -1, 1))
        self.assertLess(np.rad2deg(angle), .5)
        self.assertLess(report['final_rmse_mm'], report['initial_rmse_mm']*.25)
        self.assertLessEqual(report['sampled_points'], self.settings.max_points)
        self.assertLessEqual(report['iterations'], self.settings.max_iterations)
        np.testing.assert_allclose(self.initial[:3, 3], [.017, -.013, .553])

    def test_noise_and_sparse_outliers_do_not_dominate(self):
        rng = np.random.default_rng(7)
        visible = self.depth > 0
        self.depth[visible] += rng.normal(0, .0003, visible.sum())
        self.depth[::9, ::9] = 1.2
        candidate, report = self.refine()
        self.assertTrue(report['accepted'], report)
        self.assertLess(np.linalg.norm(candidate[:3, 3]-self.truth[:3, 3]), .001)

    def test_missing_transparent_depth_and_occluder_fall_back(self):
        for value in [0, np.nan, .3, 1.5]:
            self.depth[:] = value
            candidate, report = self.refine()
            self.assertFalse(report['accepted'], report)
            np.testing.assert_array_equal(candidate, self.initial)

    def test_plane_is_rejected_as_underconstrained(self):
        self.rendered[:] = .55
        self.depth[:] = .553
        _, report = self.refine()
        self.assertEqual(report['reason'], 'degenerate_geometry')

    def test_large_correction_rejected(self):
        self.settings = ICPSettings(budget_ms=50, max_translation_m=.0001)
        candidate, report = self.refine()
        self.assertEqual(report['reason'], 'correction_limit')
        np.testing.assert_array_equal(candidate, self.initial)

    def test_budget_expiration_discards_partial_result(self):
        counter = iter([0., .051, .052])
        candidate, report = self.refine(clock=lambda: next(counter))
        self.assertEqual(report['reason'], 'budget_exceeded')
        self.assertFalse(report['accepted'])
        np.testing.assert_array_equal(candidate, self.initial)

    def test_invalid_input_and_initial_mask(self):
        for K in [np.eye(2), np.full((3, 3), np.nan)]:
            _, report = refine_pose(self.initial, self.rendered, self.depth, K, settings=self.settings)
            self.assertEqual(report['reason'], 'invalid_input')
        _, report = self.refine(mask=np.zeros_like(self.depth, bool))
        self.assertEqual(report['reason'], 'insufficient_visible_surface')
        for value in [0, -1, 51, np.nan]:
            with self.assertRaises(ValueError):
                ICPSettings(budget_ms=value)

    def test_centered_mesh_feedback_preserves_nonzero_model_origin(self):
        center = np.eye(4);center[:3, 3] = [-.08, .02, -.03]
        internal = centered_pose(self.truth, center)
        np.testing.assert_allclose(internal @ center, self.truth, atol=1e-12)
        self.assertGreater(np.linalg.norm(internal[:3, 3]-self.truth[:3, 3]), .05)

    def observation(self, mode, render=None, age=.02, clock=time.perf_counter):
        frame = {'depth': self.depth, 'K': self.K, 'time': 100.-age, 'sequence': 17}
        result = refine_observation(self.initial, self.rendered, frame, None, None, mode,
                                    self.settings, render or self.render, pose_health,
                                    wall_time=lambda: 100., clock=clock)
        self.assertEqual(frame['time'], 100.-age)
        self.assertEqual(frame['sequence'], 17)
        return result

    def test_shadow_records_candidate_but_preserves_published_pose(self):
        pose, rendered, quality, report = self.observation('shadow')
        self.assertTrue(report['accepted'], report)
        self.assertFalse(report['applied'])
        self.assertTrue(quality['good'])
        np.testing.assert_array_equal(pose, self.initial)
        np.testing.assert_array_equal(rendered, self.rendered)

    def test_apply_publishes_validated_candidate(self):
        pose, _, quality, report = self.observation('apply')
        self.assertTrue(report['applied'], report)
        self.assertTrue(quality['good'])
        self.assertLess(np.linalg.norm(pose[:3, 3]-self.truth[:3, 3]), .0005)

    def test_off_stale_and_bad_quality_skip_solver(self):
        with patch('icp_refinement.refine_pose', side_effect=AssertionError('must not run')):
            self.assertEqual(self.observation('off')[-1]['reason'], 'disabled')
            self.assertEqual(self.observation('apply', age=.41)[-1]['reason'], 'stale_frame')
            self.depth[:] = 0
            self.assertEqual(self.observation('apply')[-1]['reason'], 'raw_quality_failed')

    def test_failed_render_validation_keeps_raw_pose(self):
        pose, _, _, report = self.observation('apply', render=lambda p: np.zeros_like(self.depth))
        self.assertEqual(report['reason'], 'render_validation_failed')
        self.assertFalse(report['applied'])
        np.testing.assert_array_equal(pose, self.initial)

    def test_render_over_budget_never_publishes_candidate(self):
        now = [0.]
        def slow_render(pose):
            now[0] = .051
            return self.render(pose)
        pose, _, _, report = self.observation('apply', render=slow_render, clock=lambda: now[0])
        self.assertEqual(report['reason'], 'budget_exceeded')
        self.assertFalse(report['applied'])
        np.testing.assert_array_equal(pose, self.initial)


if __name__ == '__main__':
    unittest.main()
