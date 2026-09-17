"""Small input/health checks for the FoundationPose camera adapter."""
import numpy as np


def catchup_frame(frames, previous_sequence, previous_time, stride=.12):
    newer = [frame for frame in frames if frame['sequence'] > previous_sequence]
    if not newer:
        return None
    if newer[0]['time'] - previous_time > .3:
        raise ValueError('初始化期间的连续帧已不足，重新识别最新画面')
    # Follow intermediate observations rather than jumping several seconds.
    # Never bridge a large temporal gap, even when inference is slow.
    eligible = [frame for frame in newer if frame['time'] - previous_time <= .3 + 1e-6]
    return next((frame for frame in eligible if frame['time'] - previous_time >= stride), eligible[-1])


def recovery_due(state, enabled, now, retry_at, frame_age):
    return bool(enabled and state == 'recovering' and now >= retry_at and frame_age is not None and 0 <= frame_age < 2)


def bottle_candidates(boxes, labels, scores, bottle_label, width=640, height=360):
    candidates = []
    for box, label, score in zip(boxes, labels, scores):
        if int(label) != bottle_label or float(score) < .4:
            continue
        x0, y0, x1, y1 = np.asarray(box, dtype=float)
        if not np.isfinite([x0, y0, x1, y1, score]).all():
            continue
        area = (x1 - x0) * (y1 - y0)
        if area < 256 or min(x1 - x0, y1 - y0) < 8:
            continue
        padded = [max(0, int(x0) - 4), max(0, int(y0) - 4), min(width, int(x1) + 5), min(height, int(y1) + 5)]
        try:
            padded = checked_box(padded, width, height)
        except ValueError:
            continue
        candidates.append({'box': padded, 'score': float(score), 'area': float(area)})
    candidates.sort(key=lambda row: row['area'], reverse=True)
    # A much smaller background bottle is not the dominant foreground target.
    if candidates:
        candidates = [row for row in candidates if row['area'] >= .35 * candidates[0]['area']]
    return candidates


def checked_box(box, width, height):
    values = np.asarray(box, dtype=float)
    if values.shape != (4,) or not np.isfinite(values).all():
        raise ValueError('框选坐标无效')
    x0, y0, x1, y1 = values.astype(int)
    if not (0 <= x0 < x1 <= width and 0 <= y0 < y1 <= height) or min(x1 - x0, y1 - y0) < 8:
        raise ValueError('请框选完整目标，框的宽高至少 8 像素')
    return [int(x0), int(y0), int(x1), int(y1)]


def pose_health(pose, rendered, depth, previous=None, mask=None):
    """Heuristic diagnostics, not absolute pose accuracy or robot authorization."""
    p = np.asarray(pose)
    if p.shape != (4, 4) or not np.isfinite(p).all() or p[2, 3] <= 0:
        return {'good': False, 'reason': 'nonfinite_or_behind_camera'}
    visible = rendered > 0
    overlap = visible & np.isfinite(depth) & (depth >= .1) & (depth <= 2)
    residual = float(np.median(np.abs(rendered[overlap] - depth[overlap]))) if overlap.any() else None
    jump = float(np.linalg.norm(p[:3, 3] - previous[:3, 3])) if previous is not None else 0.
    iou = float((visible & mask).sum() / max(1, (visible | mask).sum())) if mask is not None else None
    good = visible.sum() >= 100 and overlap.sum() >= 100 and residual is not None and residual < .04 and jump < .15
    if iou is not None:
        good = good and iou >= .45
    return {'good': bool(good), 'visible_pixels': int(visible.sum()), 'overlap_depth_pixels': int(overlap.sum()),
            'median_depth_residual_mm': None if residual is None else residual * 1000,
            'translation_step_mm': jump * 1000, 'initial_mask_iou': iou,
            'reason': 'ok' if good else 'insufficient_depth_or_pose_mismatch'}
