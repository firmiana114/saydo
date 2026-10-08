#!/usr/bin/env python3
"""Re-export saved RGB-D previews with backups, session locks and updated hashes."""

import argparse
import datetime
import fcntl
import hashlib
import json
from pathlib import Path
import shutil

import cv2
import numpy as np

from depth_preview import export_previews

ROOT = Path(__file__).resolve().parents[2]


def sha(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def reexport_session(session):
    backup = (
        session
        / 'preview_backups'
        / datetime.datetime.now().strftime('%Y%m%d_%H%M%S_%f')
    )
    records = []
    with (session / '.lock').open('a') as lock:
        fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
        selected = []
        for path in sorted((session / 'views').glob('*/sample.json')):
            report = json.loads(path.read_text())
            if report['status'] != 'candidate':
                continue
            for name, expected in report['files_sha256'].items():
                if sha(path.parent / name) != expected:
                    raise ValueError(
                        'Capture file hash mismatch: ' + str(path.parent / name)
                    )
            selected.append((path, report))
        for path, report in selected:
            directory = path.parent
            bgr = cv2.imread(str(directory / 'color.png'))
            depth = np.load(directory / 'depth_m.npy', allow_pickle=False)
            if bgr is None or bgr.shape != (*depth.shape, 3):
                raise ValueError('Invalid RGB-D dimensions: ' + str(directory))
            saved = backup / directory.name
            saved.mkdir(parents=True, exist_ok=False)
            names = ['sample.json', 'preview.jpg', 'depth_color.png']
            for name in names:
                if (directory / name).exists():
                    shutil.copy2(directory / name, saved / name)
            report['depth_visualization'] = export_previews(
                directory, bgr, depth
            )
            for name in ['depth_color.png', 'preview.jpg']:
                report['files_sha256'][name] = sha(directory / name)
            temp = path.with_suffix('.preview.tmp')
            temp.write_text(
                json.dumps(
                    report, indent=2, ensure_ascii=False, allow_nan=False
                )
                + '\n'
            )
            temp.replace(path)
            for name, expected in report['files_sha256'].items():
                if sha(directory / name) != expected:
                    raise ValueError(
                        'Post-export hash mismatch: ' + str(directory / name)
                    )
            records.append(
                {
                    'sample_id': directory.name,
                    'backup': str(saved),
                    'updated': [
                        'preview.jpg', 'depth_color.png', 'sample.json'
                    ],
                }
            )
    result = {
        'session': session.name,
        'views_updated': len(records),
        'records': records,
        'numeric_depth_and_color_unchanged': True,
    }
    if records:
        (backup / 'reexport.json').write_text(
            json.dumps(result, indent=2) + '\n'
        )
    return result


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--root', type=Path, default=ROOT / 'work/object_scan')
    parser.add_argument(
        '--sessions',
        nargs='+',
        help='Default: every capture session under root',
    )
    args = parser.parse_args()
    if (
        args.sessions
        and any(
            Path(name).name != name or name in ('.', '..')
            for name in args.sessions
        )
    ):
        parser.error('Session names must be single directory names')
    sessions = (
        [args.root / name for name in args.sessions]
        if args.sessions
        else sorted(p.parent for p in args.root.glob('*/session.json'))
    )
    for session in sessions:
        print(json.dumps(reexport_session(session)), flush=True)


if __name__ == '__main__':
    main()
