#!/usr/bin/env python3
"""Run FoundationPose's neural object field on reviewed, stationary multi-view data."""

import argparse
import datetime
import hashlib
import json
from pathlib import Path
import sys

import cv2
import numpy as np
import yaml

ROOT = Path(__file__).resolve().parents[2]


def load_views(session, segmentation, heldout, width):
    report = json.loads((segmentation / 'segmentation.json').read_text())
    if report['session'] != session.name:
        raise ValueError('Segmentation belongs to a different capture session')
    if not heldout.issubset({row['sample_id'] for row in report['views']}):
        raise ValueError('Held-out sample IDs are not present in this session')
    if report['acceptance'] != 'visually_reviewed':
        raise ValueError(
            'Segmentation must be visually reviewed before reconstruction'
        )
    rows = []
    K_reference = None
    for row in report['views']:
        sid = row['sample_id']
        if sid in heldout:
            continue
        source = session / 'views' / sid
        sample = json.loads((source / 'sample.json').read_text())
        if sample['status'] != 'candidate':
            raise ValueError(
                'View is not an accepted capture candidate: ' + sid
            )
        for filename in ('color.png', 'depth_m.npy', 'base_T_camera.txt'):
            if (
                hashlib.sha256((source / filename).read_bytes()).hexdigest()
                != sample['files_sha256'][filename]
            ):
                raise ValueError(
                    'Capture file hash mismatch: ' + sid + '/' + filename
                )
        mask_path = segmentation / 'masks' / (sid + '.png')
        if (
            hashlib.sha256(mask_path.read_bytes()).hexdigest()
            != row['mask_sha256']
        ):
            raise ValueError('Segmentation hash mismatch: ' + sid)
        rgb = cv2.cvtColor(
            cv2.imread(str(source / 'color.png')), cv2.COLOR_BGR2RGB
        )
        depth = np.load(source / 'depth_m.npy').astype(np.float32)
        mask = cv2.imread(str(mask_path), -1) > 0
        if rgb.shape[:2] != depth.shape or mask.shape != depth.shape:
            raise ValueError('RGB/depth/mask shape mismatch')
        K = np.asarray(sample['camera']['K'], dtype=np.float64)
        scale = width / rgb.shape[1]
        size = (width, round(rgb.shape[0] * scale))
        rgb = cv2.resize(rgb, size, interpolation=cv2.INTER_AREA)
        depth = cv2.resize(depth, size, interpolation=cv2.INTER_NEAREST)
        mask = cv2.resize(
            mask.astype(np.uint8), size, interpolation=cv2.INTER_NEAREST
        ).astype(bool)
        K[:2] *= scale
        if (
            K_reference is not None
            and not np.allclose(K, K_reference, atol=1e-6)
        ):
            raise ValueError('Views have incompatible intrinsics')
        K_reference = K
        pose = np.asarray(
            sample['base_T_color_optical_camera'], dtype=np.float64
        )
        rows.append(
            {'id': sid, 'rgb': rgb, 'depth': depth, 'mask': mask, 'pose': pose}
        )
    if len(rows) < 3:
        raise ValueError('Need at least three fitting views')
    return K_reference, rows


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--session', default='tape_20260915')
    parser.add_argument('--segmentation', required=True, type=Path)
    parser.add_argument('--width', type=int, default=640)
    parser.add_argument('--steps', type=int, default=1000)
    parser.add_argument(
        '--heldout',
        nargs='*',
        default=[],
        help='Explicit held-out IDs from this session',
    )
    parser.add_argument(
        '--reference-view',
        help=(
            'Fitting view with reliable target depth for the object reference '
            'origin'
        ),
    )
    parser.add_argument('--output', type=Path)
    parser.add_argument('--texture', action='store_true')
    parser.add_argument(
        '--include-rgb-only-rays',
        action='store_true',
        help='Use silhouette/color rays even when target depth is invalid',
    )
    args = parser.parse_args()
    if (
        not args.session
        or any(
            (
                c
                not in (
                    'abcdefghijklmnopqrstuvwxyzABCDEFGHIJKLMNOPQRSTUVWXYZ012345'
                    '6789_-'
                )
            )
            for c in args.session
        )
    ):
        parser.error('Invalid session name')
    if args.width < 160 or args.width > 1280 or args.steps < 10:
        parser.error('Invalid image width or training step count')
    session = ROOT / 'work/object_scan' / args.session
    K, rows = load_views(
        session, args.segmentation, set(args.heldout), args.width
    )
    if args.reference_view:
        if args.reference_view not in {row['id'] for row in rows}:
            raise ValueError('Reference view must belong to the fitting views')
        rows.sort(key=lambda row: row['id'] != args.reference_view)
    out = (
        args.output
        or (
            ROOT
            / 'work/foundationpose_runs'
            / args.session
            / datetime.datetime.now().strftime('%Y%m%d_%H%M%S')
        )
    )
    out.mkdir(parents=True, exist_ok=False)
    runtime = ROOT / 'work/foundationpose_runtime'
    sys.path.insert(0, str(runtime))
    sys.path.insert(0, str(runtime / 'bundlesdf'))
    from run_nerf import run_neural_object_field
    from nerf_runner import NerfRunner
    from nerf_helpers import preprocess_data, get_optimized_poses_in_real_world, mesh_to_real_world
    from tool import compute_scene_bounds
    from Utils import glcam_in_cvcam, set_logging_format
    import open3d as o3d
    import torch
    import random
    set_logging_format()
    random.seed(0)
    np.random.seed(0)
    torch.manual_seed(0)
    torch.cuda.manual_seed_all(0)
    # A reviewed reference view's robust valid depth gives a reference
    # center without imposing a circular/cylindrical model on the
    # reconstruction.
    first = rows[0]
    valid = (
        first['mask']
        & np.isfinite(first['depth'])
        & (first['depth'] >= .1)
        & (first['depth'] < 2)
    )
    y, x = np.nonzero(valid)
    if len(x) < 100:
        raise ValueError('Too few valid target depths in first view')
    z = first['depth'][valid]
    camera_points = np.column_stack(
        ((x - K[0, 2]) / K[0, 0] * z, (y - K[1, 2]) / K[1, 1] * z, z)
    )
    base_points = camera_points @ first['pose'][:3, :3].T + first['pose'][:3, 3]
    center = np.median(base_points, axis=0)
    base_T_object = np.eye(4)
    base_T_object[:3, 3] = center
    radius = .15  # loose spatial outlier gate, not a fitted shape or mesh prior
    statistics = []
    for row in rows:
        depth = row['depth']
        y, x = np.indices(depth.shape)
        xyz = np.stack(
            (
                (x - K[0, 2]) / K[0, 0] * depth,
                (y - K[1, 2]) / K[1, 1] * depth,
                depth,
            ),
            -1,
        )
        points = xyz @ row['pose'][:3, :3].T + row['pose'][:3, 3]
        good = (
            np.isfinite(depth)
            & (depth >= .1)
            & (depth <= 2)
            & (np.linalg.norm(points - center, axis=-1) <= radius)
        )
        row['depth'] = np.where(good, depth, 0).astype(np.float32)
        statistics.append(
            {
                'sample_id': row['id'],
                'masked_valid_depth_pixels': int((good & row['mask']).sum()),
                'mask_pixels': int(row['mask'].sum()),
            }
        )
        row['object_T_camera'] = np.linalg.inv(base_T_object) @ row['pose']
    cfg = yaml.safe_load((runtime / 'bundlesdf/config_linemod.yml').read_text())
    cfg.update(
        n_step=args.steps,
        N_rand=512,
        chunk=8192,
        netchunk=131072,
        log2_hashmap_size=19,
        i_print=50,
        i_img=999999,
        i_weights=500,
        i_mesh=999999,
        i_pose=500,
        mesh_resolution=.002,
        save_dir=str(out),
        datadir=str(session),
        rays_valid_depth_only=not args.include_rgb_only_rays,
    )
    (out / 'config.yml').write_text(yaml.safe_dump(cfg))
    run = {
        'status': 'running',
        'fit_ids': [row['id'] for row in rows],
        'heldout_ids': args.heldout,
        'base_T_object_reference': base_T_object.tolist(),
        'object_frame_definition': (
            'axes parallel to robot base; origin is first-view robust surface '
            'median'
        ),
        'depth_statistics': statistics,
        'spatial_outlier_gate_radius_m': radius,
        'width': args.width,
        'steps': args.steps,
        'source': (
            'FoundationPose NerfRunner with its supplied '
            'preprocessing/meshing; no primitive mesh substituted'
        ),
        'metric_accuracy': 'not_validated',
        'notes': [
            'Object reference origin is not a grasp center',
            'Held-out views excluded from fitting',
        ],
    }
    run['include_rgb_only_rays'] = args.include_rgb_only_rays
    run['reference_view'] = rows[0]['id']
    (out / 'run.json').write_text(json.dumps(run, indent=2))
    rgbs = np.asarray([row['rgb'] for row in rows])
    depths = np.asarray([row['depth'] for row in rows])
    masks = np.asarray([row['mask'] for row in rows])
    poses = np.asarray([row['object_T_camera'] for row in rows])
    # Equivalent to upstream run_neural_object_field, but save geometry before
    # optional texture generation so a texturing error cannot erase the mesh.
    try:
        glposes = poses @ glcam_in_cvcam
        scale, translation, _, normalized = compute_scene_bounds(
            None,
            glposes,
            K,
            use_mask=True,
            base_dir=str(out),
            rgbs=rgbs,
            depths=depths,
            masks=masks,
            eps=cfg['dbscan_eps'],
            min_samples=cfg['dbscan_eps_min_samples'],
        )
        cfg.update(sc_factor=scale, translation=translation)
        o3d.io.write_point_cloud(str(out / 'pcd_normalized.ply'), normalized)
        rgb_train, depth_train, mask_train, _, train_poses = preprocess_data(
            rgbs,
            depths,
            masks,
            normal_maps=None,
            poses=glposes,
            sc_factor=scale,
            translation=translation,
        )
        nerf = NerfRunner(
            cfg,
            rgb_train,
            depth_train,
            mask_train,
            normal_maps=None,
            poses=train_poses,
            K=K,
            occ_masks=None,
            build_octree_pcd=normalized,
        )
        nerf.train()
        mesh = nerf.extract_mesh(isolevel=0, voxel_size=cfg['mesh_resolution'])
        optimized, offset = get_optimized_poses_in_real_world(
            train_poses, nerf.models['pose_array'], scale, translation
        )
        real_mesh = mesh_to_real_world(
            mesh.copy(),
            pose_offset=offset,
            translation=translation,
            sc_factor=scale,
        )
        real_mesh.export(out / 'model_geometry.obj')
        np.save(out / 'optimized_camera_poses.npy', optimized)
        run.update(
            status='geometry_generated_unvalidated',
            vertices=len(real_mesh.vertices),
            faces=len(real_mesh.faces),
            extents_m=real_mesh.extents.tolist(),
        )
        if args.texture:
            textured = nerf.mesh_texture_from_train_images(
                mesh, rgbs_raw=rgbs, tex_res=1024
            )
            textured = mesh_to_real_world(
                textured,
                pose_offset=offset,
                translation=translation,
                sc_factor=scale,
            )
            textured.export(out / 'model.obj')
            run['status'] = 'textured_mesh_generated_unvalidated'
    except Exception as error:
        run.update(
            error=repr(error),
            status=(
                'failed'
                if 'vertices' not in run
                else 'geometry_generated_texture_failed'
            ),
        )
        raise
    finally:
        (out / 'run.json').write_text(json.dumps(run, indent=2))
        print(json.dumps({'output': str(out), **run}), flush=True)


if __name__ == '__main__':
    main()
