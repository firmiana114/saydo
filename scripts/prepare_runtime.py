#!/usr/bin/env python3
"""Create a separate build tree with explicit compatibility patches; keep vendor pristine."""

import hashlib
import json
from pathlib import Path
import shutil

# 本文件位于 <根目录>/scripts/，按路径直接运行，不依赖 saydo 包。
ROOT = Path(__file__).resolve().parents[1]
source = ROOT / 'third_party/FoundationPose'
runtime = ROOT / 'work/foundationpose_runtime'
manifest = json.loads(
    (ROOT / 'third_party/FoundationPose.upstream.json').read_text()
)
for name, expected in manifest['sha256'].items():
    assert hashlib.sha256(
        (source / name).read_bytes()
    ).hexdigest() == expected, name
if runtime.exists():
    raise SystemExit(
        'Build tree already exists; review it instead of overwriting local '
        'changes'
    )
runtime.mkdir(parents=True)
for name in manifest['sha256']:
    destination = runtime / name
    destination.parent.mkdir(parents=True, exist_ok=True)
    shutil.copy2(source / name, destination)
patches = []
for filename, changes in {
    'bundlesdf/mycuda/setup.py': [
        ('-std=c++14', '-std=c++17'), ("'gcc': c_flags", "'cxx': c_flags")
    ],
    'bundlesdf/mycuda/common.cu': [
        ('z_in_out.type()', 'z_in_out.scalar_type()'),
        ('depth_in_out.type()', 'depth_in_out.scalar_type()'),
        ('V.type()', 'V.scalar_type()'),
    ],
    'Utils.py': [
        ('from mycuda import common', 'from bundlesdf.mycuda import common')
    ],
    'bundlesdf/nerf_helpers.py': [
        (
            'from mycuda.torch_ngp_grid_encoder.grid import GridEncoder',
            (
                'from bundlesdf.mycuda.torch_ngp_grid_encoder.grid import '
                'GridEncoder'
            ),
        )
    ],
}.items():
    p = runtime / filename
    content = p.read_text()
    for old, new in changes:
        if old not in content:
            raise ValueError(
                f'Expected compatibility patch anchor missing: {filename}: '
                f'{old}'
            )
        content = content.replace(old, new)
    p.write_text(content)
    patches.append(
        {
            'file': filename,
            'sha256': hashlib.sha256(p.read_bytes()).hexdigest(),
            'replacements': changes,
        }
    )
weights = source / 'weights'
(runtime / 'weights').symlink_to(weights, target_is_directory=True)
(runtime / 'SAYDO_RUNTIME_PATCHES.json').write_text(
    json.dumps(
        {'upstream_commit': manifest['upstream_commit'], 'patches': patches},
        indent=2,
    )
    + '\n'
)
print(runtime)
