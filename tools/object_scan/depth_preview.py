"""Fixed-scale depth visualization; numeric depth is never changed."""
from pathlib import Path

import cv2
import numpy as np


PREVIEW_SPEC = {'colormap': 'OpenCV TURBO', 'min_m': .15, 'max_m': .80,
                'invalid': 'nonfinite or <= 0 is black',
                'out_of_range': 'positive depths clipped to color scale endpoints',
                'layout': 'preview.jpg: color left, depth right; no segmentation overlay'}


def depth_color(depth_m):
    depth = np.asarray(depth_m)
    if depth.ndim != 2:
        raise ValueError('Expected a 2D depth image')
    valid = np.isfinite(depth) & (depth > 0)
    normalized = np.clip((np.where(valid, depth, .15) - .15) / .65, 0, 1)
    result = cv2.applyColorMap((normalized * 255).astype(np.uint8), cv2.COLORMAP_TURBO)
    result[~valid] = 0
    return result


def export_previews(directory, bgr, depth_m):
    color = depth_color(depth_m)
    if bgr.shape != color.shape or bgr.dtype != np.uint8:
        raise ValueError('Expected uint8 BGR matching depth dimensions')
    directory = Path(directory)
    for name, data in [('depth_color.png', color), ('preview.jpg', np.hstack([bgr, color]))]:
        temp = directory / ('.export_' + name)
        if not cv2.imwrite(str(temp), data):
            raise OSError('Preview write failed: ' + name)
        temp.replace(directory / name)
    return dict(PREVIEW_SPEC)
