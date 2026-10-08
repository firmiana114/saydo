#!/usr/bin/env python3
"""Segment saved RGB-D views with reviewed SAM prompts; never modify source captures."""

import argparse
import datetime
import hashlib
import json
from pathlib import Path

import cv2
import numpy as np
import torch
from segment_anything import SamPredictor, sam_model_registry

ROOT = Path(__file__).resolve().parents[2]


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        '--prompts',
        type=Path,
        default=ROOT / 'config/foundationpose/tape_20260915_prompts.json',
    )
    parser.add_argument(
        '--checkpoint',
        type=Path,
        default=ROOT / 'work/foundationpose_setup/sam_vit_b_01ec64.pth',
    )
    parser.add_argument('--output', type=Path)
    args = parser.parse_args()
    prompts = json.loads(args.prompts.read_text())
    session = ROOT / 'work/object_scan' / prompts['session']
    out = (
        args.output
        or (
            session
            / (
                'segmentation_'
                + datetime.datetime.now().strftime('%Y%m%d_%H%M%S')
            )
        )
    )
    out.mkdir(parents=True, exist_ok=False)
    (out / 'masks').mkdir()
    model = sam_model_registry['vit_b'](
        checkpoint=str(args.checkpoint)
    ).to('cuda')
    predictor = SamPredictor(model)
    report = {
        'session': prompts['session'],
        'model': 'SAM ViT-B',
        'checkpoint_sha256': hashlib.sha256(
            args.checkpoint.read_bytes()
        ).hexdigest(),
        'prompts_sha256': hashlib.sha256(args.prompts.read_bytes()).hexdigest(),
        'acceptance': 'pending_visual_review',
        'views': [],
    }
    panels = []
    for sid, prompt in prompts['views'].items():
        source = session / 'views' / sid
        color_path = source / 'color.png'
        if (
            hashlib.sha256(color_path.read_bytes()).hexdigest()
            != prompt['color_sha256']
        ):
            raise ValueError('Prompt image hash mismatch: ' + sid)
        bgr = cv2.imread(str(color_path))
        depth = np.load(source / 'depth_m.npy')
        points = prompt['positive_points'] + prompt['negative_points']
        labels = (
            [1] * len(prompt['positive_points'])
            + [0] * len(prompt['negative_points'])
        )
        with torch.inference_mode():
            predictor.set_image(cv2.cvtColor(bgr, cv2.COLOR_BGR2RGB))
            masks, scores, _ = predictor.predict(
                point_coords=np.asarray(points),
                point_labels=np.asarray(labels),
                box=np.asarray(prompt['box_xyxy']),
                multimask_output=True,
            )
        eligible = [
            i
            for i, mask in enumerate(masks)
            if all(
                bool(mask[y, x]) == bool(label)
                for (x, y), label in zip(points, labels)
            )
        ]
        if not eligible:
            raise ValueError(
                'No mask satisfies foreground/background prompts: ' + sid
            )
        chosen = max(eligible, key=lambda i: float(scores[i]))
        mask = masks[chosen]
        if mask.shape != depth.shape or mask.sum() < 100:
            raise ValueError('Invalid target mask: ' + sid)
        target_depth = depth[mask]
        valid = (target_depth >= .1) & (target_depth <= 2.)
        if not cv2.imwrite(
            str(out / 'masks' / (sid + '.png')), mask.astype(np.uint8) * 255
        ):
            raise OSError('Mask save failed')
        row = {
            'sample_id': sid,
            'mask_pixels': int(mask.sum()),
            'sam_predicted_iou': float(scores[chosen]),
            'valid_target_depth_fraction': float(valid.mean()),
            'valid_depth_pixels': int(valid.sum()),
            'target_depth_median_m': (
                float(np.median(target_depth[valid])) if valid.any() else None
            ),
            'mask_sha256': hashlib.sha256(
                (out / 'masks' / (sid + '.png')).read_bytes()
            ).hexdigest(),
        }
        report['views'].append(row)
        overlay = bgr.copy()
        overlay[mask] = (
            .55 * overlay[mask] + .45 * np.array([0, 0, 255])
        ).astype(np.uint8)
        panel = cv2.resize(overlay, (480, 270))
        cv2.putText(
            panel,
            sid,
            (6, 22),
            cv2.FONT_HERSHEY_SIMPLEX,
            .55,
            (255, 255, 255),
            2,
        )
        panels.append(panel)
        print(json.dumps(row), flush=True)
    while len(panels) % 4:
        panels.append(np.zeros_like(panels[0]))
    cv2.imwrite(
        str(out / 'overview.jpg'),
        np.vstack(
            [np.hstack(panels[i:i + 4]) for i in range(0, len(panels), 4)]
        ),
    )
    (out / 'segmentation.json').write_text(json.dumps(report, indent=2) + '\n')
    print(
        json.dumps(
            {
                'output': str(out),
                'views': len(report['views']),
                'acceptance': report['acceptance'],
            }
        ),
        flush=True,
    )


if __name__ == '__main__':
    main()
