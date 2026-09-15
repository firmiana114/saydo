#!/usr/bin/env python3
"""Exercise GPU kernels and load weights; no robot/camera access or motion."""
import hashlib
import json
from pathlib import Path
import sys

ROOT = Path(__file__).resolve().parents[2]
runtime = ROOT / 'work/foundationpose_runtime'
sys.path[:0] = [str(runtime), str(runtime / 'bundlesdf'), str(runtime / 'bundlesdf/mycuda'), str(runtime / 'mycpp/build')]

import numpy as np
import torch

assert torch.cuda.is_available()
x = torch.randn(64, 64, device='cuda')
assert torch.isfinite(x @ x).all()
report = {'torch': torch.__version__, 'cuda_runtime': torch.version.cuda, 'gpu': torch.cuda.get_device_name(0), 'matrix_multiply': True}

import mycpp
assert getattr(mycpp, '__file__', '').endswith('.so'), 'mycpp resolved to a namespace instead of the compiled extension'
report['mycpp'] = mycpp.__file__

from pytorch3d.ops import knn_points
points = torch.randn(1, 8, 3, device='cuda')
distances = knn_points(points, points, K=1).dists
assert torch.allclose(distances, torch.zeros_like(distances), atol=1e-6)
report['pytorch3d_cuda_knn'] = True

import nvdiffrast.torch as dr
ctx = dr.RasterizeCudaContext()
vertices = torch.tensor([[[-.5, -.5, 0, 1], [.5, -.5, 0, 1], [0, .5, 0, 1]]], dtype=torch.float32, device='cuda')
triangles = torch.tensor([[0, 1, 2]], dtype=torch.int32, device='cuda')
raster, _ = dr.rasterize(ctx, vertices, triangles, resolution=[32, 32])
assert (raster[..., 3] > 0).any()
report['nvdiffrast_cuda_rasterization'] = True

import kaolin
quantized = kaolin.ops.spc.quantize_points(torch.tensor([[-.5, -.5, -.5], [0., 0., 0.], [.5, .5, .5]], device='cuda'), level=3)
octree = kaolin.ops.spc.unbatched_points_to_octree(quantized, 3, sorted=False)
assert octree.is_cuda and octree.numel() > 0
report['kaolin_cuda_octree'] = True

from bundlesdf.mycuda import common
from mycuda.torch_ngp_grid_encoder.grid import GridEncoder
encoder = GridEncoder(input_dim=3, num_levels=4, level_dim=2, base_resolution=4, log2_hashmap_size=8, desired_resolution=32).cuda()
coords = torch.rand(8, 3, device='cuda', requires_grad=True)
features = encoder(coords)
features.sum().backward()
assert torch.isfinite(features).all() and torch.isfinite(coords.grad).all()
report['gridencoder_cuda_forward_backward'] = True
assert callable(common.sampleRaysUniformOccupiedVoxels)
report['common_extension_loaded'] = True

from learning.training.predict_score import ScorePredictor
from learning.training.predict_pose_refine import PoseRefinePredictor
score = ScorePredictor()
refine = PoseRefinePredictor()
report['scorer_and_refiner_weights_loaded'] = True
report['weights'] = json.loads((runtime / 'weights/SAYDO_WEIGHTS_SOURCE.json').read_text())
for filename, metadata in report['weights']['files'].items():
    assert hashlib.sha256((runtime / 'weights' / filename).read_bytes()).hexdigest() == metadata['sha256']
torch.cuda.synchronize()
report['peak_cuda_memory_bytes'] = torch.cuda.max_memory_allocated()
report['scope'] = 'GPU kernel and weight-loading validation; not tape reconstruction or tracking accuracy'
out = ROOT / 'work/foundationpose_setup/gpu_check.json'
out.write_text(json.dumps(report, indent=2) + '\n')
print(json.dumps(report, indent=2))
