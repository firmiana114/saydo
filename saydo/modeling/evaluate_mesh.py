#!/usr/bin/env python3
"""Offline held-out registration and same-frame tracking check; never sends motion."""

import argparse
import json
from pathlib import Path
import sys
import time

import cv2
import numpy as np

from saydo import paths

ROOT = paths.ROOT


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--run', required=True, type=Path)
    parser.add_argument('--segmentation', required=True, type=Path)
    parser.add_argument('--session', default='tape_20260915')
    parser.add_argument('--rerank-topk', type=int, default=0)
    args = parser.parse_args()
    if not 0 <= args.rerank_topk <= 252:
        parser.error('rerank-topk must be between 0 and 252')
    run = json.loads((args.run / 'run.json').read_text())
    if (
        run['status']
        not in (
            'geometry_generated_unvalidated',
            'textured_mesh_generated_unvalidated',
            'geometry_generated_texture_failed',
        )
    ):
        raise ValueError('No completed geometry to evaluate')
    out = (
        args.run
        / (
            'heldout_evaluation'
            if args.rerank_topk == 0
            else f'development_rerank_top{args.rerank_topk}'
        )
    )
    out.mkdir(exist_ok=False)
    runtime = ROOT / 'work/foundationpose_runtime'
    sys.path[:0] = [
        str(runtime),
        str(runtime / 'bundlesdf'),
        str(runtime / 'bundlesdf/mycuda'),
        str(runtime / 'mycpp/build'),
    ]
    import torch
    import trimesh
    import nvdiffrast.torch as dr
    from estimater import FoundationPose
    from learning.training.predict_score import ScorePredictor
    from learning.training.predict_pose_refine import PoseRefinePredictor
    from Utils import draw_posed_3d_box, nvdiffrast_render, set_logging_format
    set_logging_format()
    mesh_path = (
        args.run
        / (
            'model.obj'
            if (args.run / 'model.obj').exists()
            else 'model_geometry.obj'
        )
    )
    mesh = trimesh.load(mesh_path, force='mesh')
    if len(mesh.vertices) < 4 or not np.isfinite(mesh.vertices).all():
        raise ValueError('Invalid reconstruction mesh')
    to_origin, extents = trimesh.bounds.oriented_bounds(mesh)
    bbox = np.stack([-extents / 2, extents / 2])
    glctx = dr.RasterizeCudaContext()
    est = FoundationPose(
        model_pts=mesh.vertices,
        model_normals=mesh.vertex_normals,
        mesh=mesh,
        scorer=ScorePredictor(),
        refiner=PoseRefinePredictor(),
        debug_dir=str(out),
        debug=0,
        glctx=glctx,
    )
    results = []
    for sid in run['heldout_ids']:
        source = ROOT / 'work/object_scan' / args.session / 'views' / sid
        sample = json.loads((source / 'sample.json').read_text())
        bgr = cv2.imread(str(source / 'color.png'))
        depth = np.load(source / 'depth_m.npy')
        mask = (
            cv2.imread(str(args.segmentation / 'masks' / (sid + '.png')), -1)
            > 0
        )
        scale = run['width'] / bgr.shape[1]
        size = (run['width'], round(bgr.shape[0] * scale))
        rgb = cv2.cvtColor(cv2.resize(bgr, size), cv2.COLOR_BGR2RGB)
        depth = cv2.resize(
            depth, size, interpolation=cv2.INTER_NEAREST
        ).astype(np.float32)
        mask = cv2.resize(
            mask.astype(np.uint8), size, interpolation=cv2.INTER_NEAREST
        ).astype(bool)
        K = np.asarray(sample['camera']['K'], dtype=float)
        K[:2] *= scale
        if ((depth > .1) & (depth < 2) & mask).sum() < 100:
            raise ValueError('Insufficient held-out foreground depth: ' + sid)
        torch.cuda.synchronize()
        started = time.perf_counter()
        est.pose_last = None
        pose = est.register(
            K=K, rgb=rgb, depth=depth.copy(), ob_mask=mask, iteration=5
        )
        torch.cuda.synchronize()
        seconds = time.perf_counter() - started
        if not np.isfinite(pose).all() or est.pose_last is None:
            raise ValueError(
                'Registration did not initialize a finite tracked pose'
            )
        candidates = []
        selected_rank = 1
        if args.rerank_topk:
            count = min(args.rerank_topk, len(est.poses))
            mesh_poses = est.poses[:count] @ est.get_tf_to_centered_mesh()
            _, candidate_depths, _ = nvdiffrast_render(
                K=K,
                H=rgb.shape[0],
                W=rgb.shape[1],
                mesh=mesh,
                ob_in_cams=mesh_poses,
                glctx=glctx,
            )
            candidate_depths = candidate_depths.detach().cpu().numpy()
            for rank, rendered in enumerate(candidate_depths):
                visible = rendered > 0
                overlap = visible & mask & (depth > .1) & (depth < 2)
                candidate_iou = float(
                    (visible & mask).sum() / max(1, (visible | mask).sum())
                )
                candidate_error = (
                    float(
                        np.median(np.abs(rendered[overlap] - depth[overlap]))
                        * 1000
                    )
                    if overlap.any()
                    else None
                )
                candidates.append(
                    {
                        'neural_rank': rank + 1,
                        'silhouette_iou': candidate_iou,
                        'overlap_depth_pixels': int(overlap.sum()),
                        'median_depth_residual_mm': candidate_error,
                    }
                )
            eligible = [
                i
                for i, row in enumerate(candidates)
                if row['overlap_depth_pixels'] >= 100
            ]
            if not eligible:
                raise ValueError(
                    'No candidate has enough overlapping target depth'
                )
            chosen = max(
                eligible,
                key=lambda i: (
                    candidates[i]['silhouette_iou'],
                    -candidates[i]['median_depth_residual_mm'],
                ),
            )
            selected_rank = chosen + 1
            pose = mesh_poses[chosen].detach().cpu().numpy()
            est.pose_last = est.poses[chosen].clone()
        _, rendered_depth, _ = nvdiffrast_render(
            K=K,
            H=rgb.shape[0],
            W=rgb.shape[1],
            mesh=mesh,
            ob_in_cams=torch.as_tensor(
                pose[None], dtype=torch.float32, device='cuda'
            ),
            glctx=glctx,
        )
        rd = rendered_depth[0].detach().cpu().numpy()
        silhouette = rd > 0
        iou = (silhouette & mask).sum() / max(1, (silhouette | mask).sum())
        overlap = silhouette & mask & (depth > .1) & (depth < 2)
        residual = np.abs(rd[overlap] - depth[overlap]) * 1000
        reference = (
            np.linalg.inv(np.asarray(sample['base_T_color_optical_camera']))
            @ np.asarray(run['base_T_object_reference'])
        )
        tracking = est.track_one(rgb=rgb, depth=depth.copy(), K=K, iteration=2)
        if not np.isfinite(tracking).all():
            raise ValueError('Nonfinite same-frame tracking pose')
        row = {
            'sample_id': sid,
            'registration_seconds_including_first_use_compilation': seconds,
            'selected_neural_rank': selected_rank,
            'candidate_diagnostics': candidates,
            'silhouette_iou': float(iou),
            'overlap_depth_pixels': int(overlap.sum()),
            'median_depth_residual_mm': (
                float(np.median(residual)) if len(residual) else None
            ),
            'p95_depth_residual_mm': (
                float(np.percentile(residual, 95)) if len(residual) else None
            ),
            'translation_difference_from_handeye_reference_mm': float(
                np.linalg.norm(pose[:3, 3] - reference[:3, 3]) * 1000
            ),
            'bbox_center_difference_from_handeye_reference_mm': float(
                np.linalg.norm(
                    (pose @ np.r_[mesh.bounds.mean(axis=0), 1])[:3]
                    - (reference @ np.r_[mesh.bounds.mean(axis=0), 1])[:3]
                )
                * 1000
            ),
            'same_frame_tracking_translation_change_mm': float(
                np.linalg.norm(tracking[:3, 3] - pose[:3, 3]) * 1000
            ),
            'camera_T_object': pose.tolist(),
            'same_frame_track_camera_T_object': tracking.tolist(),
        }
        results.append(row)
        overlay = draw_posed_3d_box(
            K,
            img=rgb.copy(),
            ob_in_cam=pose @ np.linalg.inv(to_origin),
            bbox=bbox,
        )
        overlay[silhouette] = (
            .65 * overlay[silhouette] + .35 * np.array([255, 0, 0])
        ).astype(np.uint8)
        cv2.imwrite(
            str(out / (sid + '.jpg')), cv2.cvtColor(overlay, cv2.COLOR_RGB2BGR)
        )
        print(json.dumps(row), flush=True)
    report = {
        'results': results,
        'scope': (
            f'{len(results)} held-out still images; track_one reused each '
            f'identical image'
        ),
        'independent_evaluation': args.rerank_topk == 0,
        'development_note': (
            (
                'Reranking was added after inspecting baseline failures; fresh '
                'views are required for independent evaluation'
            )
            if args.rerank_topk
            else None
        ),
        'continuous_motion_tracking_verified': False,
        'absolute_pose_accuracy_verified': False,
        'pose_accepted_for_robot_control': False,
        'mesh_path': str(mesh_path),
    }
    (out / 'evaluation.json').write_text(json.dumps(report, indent=2) + '\n')


if __name__ == '__main__':
    main()
