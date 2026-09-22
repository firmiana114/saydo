"""Bounded projective point-to-plane ICP on visible rendered RGB-D surfaces.

All depths/translations are metres. No camera, robot, model loading or worker queue.
The budget is cooperative: a NumPy call cannot be interrupted mid-operation.
"""
from dataclasses import dataclass
import time

import cv2
import numpy as np


@dataclass(frozen=True)
class ICPSettings:
    budget_ms: float = 20.
    max_points: int = 1800
    max_iterations: int = 7
    max_distance_m: float = .015
    max_translation_m: float = .01
    max_rotation_deg: float = 5.
    min_points: int = 100
    min_coverage: float = .6

    def __post_init__(self):
        if not np.isfinite(self.budget_ms) or not 0 < self.budget_ms <= 50:
            raise ValueError('ICP budget must be in (0, 50] milliseconds')
        if not 100 <= self.max_points <= 4000 or not 1 <= self.max_iterations <= 12:
            raise ValueError('ICP point/iteration limit outside bounded range')
        if not 6 <= self.min_points <= self.max_points or not 0 < self.min_coverage <= 1:
            raise ValueError('Invalid ICP support limits')
        for v in (self.max_distance_m, self.max_translation_m, self.max_rotation_deg):
            if not np.isfinite(v) or v <= 0:
                raise ValueError('Invalid ICP correction limit')


def unproject(u, v, z, K):
    return np.column_stack(((u-K[0, 2])*z/K[0, 0], (v-K[1, 2])*z/K[1, 1], z))


def centered_pose(pose, tf_to_center):
    """Public original-mesh pose -> FoundationPose's centered-mesh state."""
    return np.asarray(pose) @ np.linalg.inv(np.asarray(tf_to_center))


def refine_pose(pose, rendered, depth, K, mask=None, settings=None, clock=time.perf_counter):
    settings = settings or ICPSettings()
    started = clock()
    original = np.asarray(pose, dtype=float).copy()
    report = {'accepted': False, 'applied': False, 'reason': 'not_started', 'iterations': 0}

    def finish(reason, candidate=None):
        report.update(reason=reason, elapsed_ms=(clock()-started)*1000)
        if report['elapsed_ms'] > settings.budget_ms:
            report.update(reason='budget_exceeded', accepted=False)
            return original, report
        report['accepted'] = candidate is not None
        return (original if candidate is None else candidate), report

    def expired():
        return (clock()-started)*1000 >= settings.budget_ms

    try:
        rendered = np.asarray(rendered)
        depth = np.asarray(depth)
        K = np.asarray(K, dtype=float)
        if (original.shape != (4, 4) or not np.isfinite(original).all()
                or not np.allclose(original[3], [0, 0, 0, 1])
                or not np.allclose(original[:3, :3].T @ original[:3, :3], np.eye(3), atol=1e-4)
                or not np.isclose(np.linalg.det(original[:3, :3]), 1, atol=1e-4)
                or K.shape != (3, 3) or not np.isfinite(K).all() or min(K[0, 0], K[1, 1]) <= 0
                or depth.ndim != 2 or rendered.shape != depth.shape or depth.size > 1280*720):
            return finish('invalid_input')
        height, width = depth.shape
        if min(height, width) < 8 or (mask is not None and np.shape(mask) != depth.shape):
            return finish('invalid_input')
        # Erode silhouettes and reject depth discontinuities (background/occluders).
        visible = np.isfinite(rendered) & (rendered >= .1) & (rendered <= 2)
        if mask is not None:
            visible &= np.asarray(mask, dtype=bool)
        visible = cv2.erode(visible.astype(np.uint8), np.ones((3, 3), np.uint8)).astype(bool)
        visible[:2] = visible[-2:] = False
        visible[:, :2] = visible[:, -2:] = False
        v, u = np.nonzero(visible)
        if len(u) < settings.min_points:
            return finish('insufficient_visible_surface')
        # Deterministic spread across visible pixels; never allocate a full XYZ map.
        indices = np.linspace(0, len(u)-1, min(len(u), settings.max_points), dtype=int)
        u, v = u[indices], v[indices]
        source = unproject(u, v, rendered[v, u], K)
        report['sampled_points'] = len(source)
        pivot = source.mean(axis=0)
        scale = max(float(np.sqrt(np.mean(np.sum((source-pivot)**2, axis=1)))), .01)

        def correspond(points, distance):
            z = points[:, 2]
            uv = np.rint(points[:, :2] / np.maximum(z[:, None], 1e-9)
                         * [K[0, 0], K[1, 1]] + [K[0, 2], K[1, 2]]).astype(int)
            x, y = uv.T
            valid = (z > .1) & (x >= 2) & (x < width-2) & (y >= 2) & (y < height-2)
            ids = np.flatnonzero(valid)
            x, y = x[ids], y[ids]
            d = np.column_stack([depth[y, x], depth[y, x-1], depth[y, x+1], depth[y-1, x], depth[y+1, x]])
            ok = np.isfinite(d).all(axis=1) & (d >= .1).all(axis=1) & (d <= 2).all(axis=1)
            ok &= np.ptp(d, axis=1) < .008
            ok &= np.abs(d[:, 0]-z[ids]) < distance
            if mask is not None:
                ok &= np.asarray(mask, dtype=bool)[y, x]
            ids, x, y, d = ids[ok], x[ok], y[ok], d[ok]
            target = unproject(x, y, d[:, 0], K)
            dx = unproject(x+1, y, d[:, 2], K)-unproject(x-1, y, d[:, 1], K)
            dy = unproject(x, y+1, d[:, 4], K)-unproject(x, y-1, d[:, 3], K)
            normals = np.cross(dx, dy)
            length = np.linalg.norm(normals, axis=1)
            ok = length > 1e-10
            return ids[ok], target[ok], normals[ok]/length[ok, None]

        ids0, target0, normals0 = correspond(source, settings.max_distance_m)
        coverage = len(ids0)/len(source)
        report.update(initial_points=len(ids0), initial_coverage=coverage)
        if len(ids0) < settings.min_points or coverage < settings.min_coverage:
            return finish('insufficient_depth_support')
        initial_error = float(np.sqrt(np.mean(np.sum((source[ids0]-target0)*normals0, axis=1)**2)))
        report['initial_rmse_mm'] = initial_error*1000
        delta = np.eye(4)
        for iteration in range(settings.max_iterations):
            if expired():
                return finish('budget_exceeded')
            points = source @ delta[:3, :3].T + delta[:3, 3]
            # Two correspondence scales, sharing the same bounded source sample.
            distance = settings.max_distance_m if iteration < 3 else settings.max_distance_m*.5
            ids, target, normals = correspond(points, distance)
            if len(ids) < settings.min_points or len(ids)/len(source) < settings.min_coverage:
                return finish('correspondence_loss')
            p = points[ids]
            residual = np.sum((p-target)*normals, axis=1)
            J = np.column_stack((np.cross(p-pivot, normals)/scale, normals))
            weights = np.sqrt(np.minimum(1., .003/np.maximum(np.abs(residual), 1e-12)))
            A, b = J*weights[:, None], -residual*weights
            # Solve only a 6x6 system; normalized rotational columns expose degeneracy.
            U, singular, Vt = np.linalg.svd(A.T @ A)
            if singular[-1] < singular[0]*1e-5:
                return finish('degenerate_geometry')
            step = Vt.T @ ((U.T @ (A.T @ b))/singular)
            rotation = cv2.Rodrigues(step[:3]/scale)[0]
            increment = np.eye(4)
            increment[:3, :3] = rotation
            increment[:3, 3] = pivot + step[3:] - rotation @ pivot
            delta = increment @ delta
            angle = float(np.rad2deg(np.arccos(np.clip((np.trace(delta[:3, :3])-1)/2, -1, 1))))
            shift = float(np.linalg.norm(delta[:3, :3] @ pivot + delta[:3, 3]-pivot))
            report.update(iterations=iteration+1, correction_mm=shift*1000, correction_deg=angle)
            if shift > settings.max_translation_m or angle > settings.max_rotation_deg:
                return finish('correction_limit')
            if np.linalg.norm(step[3:]) < .00002 and np.linalg.norm(step[:3]/scale) < .0002:
                break
        if expired():
            return finish('budget_exceeded')
        points = source @ delta[:3, :3].T + delta[:3, 3]
        ids, target, normals = correspond(points, settings.max_distance_m)
        report['final_coverage'] = len(ids)/len(source)
        if len(ids) < max(settings.min_points, .9*len(ids0)):
            return finish('coverage_regressed')
        # Compare on the SAME original support, avoiding a lower error from dropping points.
        fixed_error = float(np.sqrt(np.mean(np.sum((points[ids0]-target0)*normals0, axis=1)**2)))
        final_error = float(np.sqrt(np.mean(np.sum((points[ids]-target)*normals, axis=1)**2)))
        report.update(fixed_support_rmse_mm=fixed_error*1000, final_rmse_mm=final_error*1000)
        if fixed_error >= initial_error-.00001 or final_error > initial_error:
            return finish('no_improvement')
        candidate = delta @ original
        report['candidate_pose'] = candidate.tolist()
        return finish('accepted', candidate)
    except (ValueError, IndexError, np.linalg.LinAlgError, cv2.error) as error:
        report['error'] = type(error).__name__
        return finish('invalid_input_or_solver_failure')


def refine_observation(pose, rendered, frame, previous, mask, mode, settings, render, health,
                       wall_time=time.time, clock=time.perf_counter):
    """Validate candidates for shadow/apply; preserve raw pose on every rejected path."""
    started = clock()
    raw_quality = health(pose, rendered, frame['depth'], previous, mask)
    report = {'mode': mode, 'accepted': False, 'applied': False, 'reason': 'disabled', 'elapsed_ms': 0.}
    if mode == 'off':
        return pose, rendered, raw_quality, report
    if mode not in ('shadow', 'apply'):
        raise ValueError('Unknown ICP mode')
    report['raw_pose'] = np.asarray(pose).tolist()
    if not raw_quality['good']:
        report['reason'] = 'raw_quality_failed'
        return pose, rendered, raw_quality, report
    if wall_time()-frame['time'] + settings.budget_ms/1000 > .45:
        report['reason'] = 'stale_frame'
        return pose, rendered, raw_quality, report
    candidate, detail = refine_pose(pose, rendered, frame['depth'], frame['K'], mask, settings, clock)
    report.update(detail)
    if report['accepted']:
        try:
            candidate_rendered = render(candidate)
            candidate_quality = health(candidate, candidate_rendered, frame['depth'], previous, mask)
            report['candidate_quality'] = candidate_quality
            # Same pixel support and missing-render penalty: cannot improve by hiding surfaces.
            support = (rendered > 0) & np.isfinite(frame['depth']) & (frame['depth'] >= .1) & (frame['depth'] <= 2)
            if mask is not None:
                support &= mask
            before = np.abs(rendered[support]-frame['depth'][support])
            after = np.abs(candidate_rendered[support]-frame['depth'][support])
            improved = len(before) > 0 and np.mean(after) < np.mean(before)
            if not candidate_quality['good'] or not improved:
                report.update(accepted=False, reason='render_validation_failed')
        except (RuntimeError, ValueError) as error:
            report.update(accepted=False, reason='render_validation_failed', error=type(error).__name__)
    report['elapsed_ms'] = (clock()-started)*1000
    if report['elapsed_ms'] > settings.budget_ms:
        report.update(accepted=False, reason='budget_exceeded')
    if wall_time()-frame['time'] > .5:
        report.update(accepted=False, reason='stale_frame')
    if report['accepted'] and mode == 'apply':
        report['applied'] = True
        return candidate, candidate_rendered, candidate_quality, report
    return pose, rendered, raw_quality, report
