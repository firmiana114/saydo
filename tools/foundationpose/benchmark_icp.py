#!/usr/bin/env python3
"""Synthetic 640x360 latency check; optional real GPU rendering, never hardware IO."""
import argparse
from collections import Counter
import json
import time

import cv2
import numpy as np

from icp_refinement import ICPSettings, refine_pose, refine_observation
from live_checks import pose_health
from test_icp_refinement import scene


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--gpu', action='store_true', help='Include candidate validation using the deployed FoundationPose GPU renderer')
    parser.add_argument('--samples', type=int, default=100)
    parser.add_argument('--mesh', help='Optional real model mesh for synthetic GPU observations (metre scale)')
    args = parser.parse_args()
    if not 1 <= args.samples <= 10000:
        parser.error('samples must be in [1, 10000]')
    if args.mesh and not args.gpu:
        parser.error('mesh requires --gpu')
    cv2.setNumThreads(1)
    K, truth, initial, render = scene(640, 360)
    if args.gpu:
        import logging
        import torch
        import trimesh
        import nvdiffrast.torch as dr
        from Utils import nvdiffrast_render, make_mesh_tensors
        logging.getLogger().setLevel(logging.WARNING)
        torch.set_num_threads(2)
        if args.mesh:
            mesh = trimesh.load(args.mesh, force='mesh')
            center = (mesh.bounds[0]+mesh.bounds[1])/2
            truth[:3, 3] -= truth[:3, :3] @ center
            initial[:3, 3] -= initial[:3, :3] @ center
        else:
            mesh = trimesh.creation.icosphere(subdivisions=3)
            mesh.vertices *= np.array([.09, .055, .07])
        context = dr.RasterizeCudaContext()
        tensors = make_mesh_tensors(mesh)
        def render(pose):
            with torch.inference_mode():
                _, depth, _ = nvdiffrast_render(K=K, H=360, W=640, mesh_tensors=tensors,
                    ob_in_cams=torch.as_tensor(pose[None], device='cuda', dtype=torch.float32), glctx=context)
                return depth[0].cpu().numpy()
    depth, rendered = render(truth), render(initial)
    times, reasons = [], Counter()
    for index in range(args.samples+10):
        if args.gpu:
            frame = {'depth': depth, 'K': K, 'time': time.time()}
            *_, report = refine_observation(initial, rendered, frame, None, None, 'shadow',
                                            ICPSettings(), render, pose_health)
        else:
            _, report = refine_pose(initial, rendered, depth, K)
        if index >= 10:
            times.append(report['elapsed_ms']);reasons[report['reason']] += 1
    print(json.dumps({'synthetic': True, 'resolution': [640, 360], 'samples': len(times),
        'scope': 'solver_and_gpu_validation' if args.gpu else 'solver_only', 'mesh': args.mesh,
        'budget_ms': ICPSettings().budget_ms, 'p50_ms': float(np.percentile(times, 50)),
        'p95_ms': float(np.percentile(times, 95)), 'max_ms': max(times), 'outcomes': dict(reasons)}, indent=2))


if __name__ == '__main__':
    main()
