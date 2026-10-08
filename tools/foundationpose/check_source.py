#!/usr/bin/env python3
"""Verify the pinned upstream source snapshot without GPU imports or hardware access."""

import hashlib
import json
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]


def main():
    manifest = json.loads(
        (ROOT / 'third_party/FoundationPose.upstream.json').read_text()
    )
    source = ROOT / 'third_party/FoundationPose'
    failures = []
    for name, expected in manifest['sha256'].items():
        path = source / name
        if not path.is_file():
            failures.append({'path': name, 'error': 'missing'})
        elif hashlib.sha256(path.read_bytes()).hexdigest() != expected:
            failures.append({'path': name, 'error': 'sha256 mismatch'})
    if manifest['file_count'] != len(manifest['sha256']):
        failures.append({'error': 'manifest count mismatch'})
    print(
        json.dumps(
            {
                'source_integrity_passed': not failures,
                'upstream_commit': manifest['upstream_commit'],
                'checked_files': len(manifest['sha256']),
                'failures': failures,
                'runtime_checked': False,
                'weights_checked': False,
                'reconstruction_or_tracking_run': False,
            },
            ensure_ascii=False,
            indent=2,
        )
    )
    return int(bool(failures))


if __name__ == '__main__':
    raise SystemExit(main())
