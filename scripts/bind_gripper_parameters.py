#!/usr/bin/env python3
"""Populate per-grasp gripper settings; dry-run by default, no hardware connections."""

import argparse
import copy
import fcntl
import json
import pathlib
import time

from saydo.objects import grasp_library


def migrate(library, apply=False):
    pending = []
    objects = []
    for directory in sorted(library.models_root.iterdir()):
        if (
            not directory.is_dir()
            or directory.is_symlink()
            or directory.name.startswith('.')
        ):
            continue
        object_id = directory.name
        value = library.object(object_id)
        if not value['grasps']:
            continue
        original = copy.deepcopy(value)
        settings = grasp_library.object_gripper_defaults(object_id)
        changed = 0
        for entry in value['grasps']:
            params = grasp_library.checked_bound_overrides(
                {**entry.get('overrides', {}), **settings}
            )
            if entry.get('overrides', {}) != params:
                entry['overrides'] = params
                entry['tested_successfully'] = False
                changed += 1
        objects.append(
            {
                'object_id': object_id,
                'grasp_count': len(value['grasps']),
                'changed': changed,
                **settings,
            }
        )
        if changed:
            pending.append((object_id, original, value))
    # Validate every model before modifying any file. Normal writer keeps
    # per-model backups.
    for object_id, original, value in pending:
        if not apply:
            continue
        library.write_object(object_id, value, grasp_library.revision(original))
        records = library.object_directory(object_id) / 'grasp_profile_commands'
        records.mkdir(exist_ok=True)
        (records / f'{time.time_ns()}.json').write_text(
            json.dumps(
                {
                    'action': 'bind_gripper_parameters',
                    'before_revision': grasp_library.revision(original),
                    'after_revision': grasp_library.revision(value),
                    'values': grasp_library.object_gripper_defaults(object_id),
                    'motion_sent': False,
                },
                ensure_ascii=False,
                indent=2,
            )
        )
    return {
        'objects': objects,
        'files_to_write': len(pending),
        'applied': apply,
        'motion_sent': False,
    }


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        '--root',
        type=pathlib.Path,
        default=pathlib.Path(__file__).resolve().parents[1],
    )
    parser.add_argument('--models-root', type=pathlib.Path)
    parser.add_argument('--apply', action='store_true')
    args = parser.parse_args()
    library = grasp_library.GraspLibrary(args.root, args.models_root)
    directory = args.root / 'work/grasp'
    directory.mkdir(parents=True, exist_ok=True)
    with (directory / 'robot.lock').open('a') as lock:
        fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
        print(
            json.dumps(
                migrate(library, args.apply), ensure_ascii=False, indent=2
            )
        )


if __name__ == '__main__':
    main()
