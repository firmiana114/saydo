"""Move active model data to model directories; retain central sources for rollback.

Run while the tracker is stopped. Dry run is the default; --apply writes files.
This utility never connects to the robot or changes its state.
"""

import argparse
import fcntl
import json
import os
from pathlib import Path
import tempfile

from saydo.objects.grasp_library import GraspLibrary


def migration_plan(library):
    legacy = (
        json.loads(library.legacy.read_text())
        if library.legacy.exists()
        else {}
    )
    ids = set(library.legacy_data()['objects']) | set(legacy)
    pending = {}
    skipped = []

    def include(object_id, relative, data):
        directory = library.object_directory(object_id)
        if not directory.is_dir():
            raise ValueError(f'迁移目标缺少物品目录：{object_id}')
        path = directory / relative
        if not path.resolve().is_relative_to(directory.resolve()):
            raise ValueError('模型数据路径越界')
        content = (
            json.dumps(data, ensure_ascii=False, indent=2, allow_nan=False)
            + '\n'
        ).encode()
        if path.exists():
            if json.loads(path.read_text()) != data:
                raise ValueError(f'迁移目标已存在不同内容：{path}')
        else:
            pending[path] = content

    def document(object_id, value):
        return {
            'schema_version': 3,
            'object_id': object_id,
            'configuration': library.checked_object(value),
        }

    for object_id in sorted(ids):
        # A model-local file is authoritative, including an explicitly empty
        # list.
        if library.object_path(object_id).exists():
            library.object(object_id)
        else:
            include(
                object_id,
                'grasp_profiles.json',
                document(object_id, library.object(object_id)),
            )
        if object_id in legacy:
            include(object_id, 'taught_profile_legacy.json', legacy[object_id])

    for source in sorted(
        (library.directory / 'grasp_profile_backups').glob('*.json')
    ):
        data = json.loads(source.read_text())
        if data.get('schema_version') != 2:
            raise ValueError(f'未知历史备份版本：{source}')
        for object_id, value in data['objects'].items():
            include(
                object_id,
                Path('grasp_profile_backups') / ('legacy-' + source.name),
                document(object_id, value),
            )

    for kind in ['teach_commands', 'grasp_profile_commands']:
        for source in sorted((library.directory / kind).glob('*.json')):
            data = json.loads(source.read_text())
            object_id = (
                data.get('object_id')
                or data.get('observation', {}).get('object_id')
            )
            if object_id is None:
                skipped.append(str(source))
                continue
            include(object_id, Path(kind) / source.name, data)
    return (
        pending,
        {
            'objects': sorted(ids),
            'files_to_write': len(pending),
            'unassigned_history_retained': skipped,
            'central_sources_retained': True,
        },
    )


def migrate(library, apply=False):
    pending, report = migration_plan(library)
    for path, content in pending.items():
        if not apply:
            continue
        path.parent.mkdir(parents=True, exist_ok=True)
        temporary = None
        try:
            with tempfile.NamedTemporaryFile(
                dir=path.parent, prefix='.migration-', delete=False
            ) as out:
                temporary = Path(out.name)
                os.fchmod(out.fileno(), 0o644)
                out.write(content)
                out.flush()
                os.fsync(out.fileno())
            # Exclusive link refuses an unexpected concurrent writer; reruns are
            # safe.
            os.link(temporary, path)
        finally:
            if temporary is not None:
                temporary.unlink(missing_ok=True)
    return {**report, 'applied': apply}


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        '--root', type=Path, default=Path(__file__).resolve().parents[1]
    )
    parser.add_argument('--models-root', type=Path)
    parser.add_argument('--apply', action='store_true')
    args = parser.parse_args()
    library = GraspLibrary(args.root, args.models_root)
    library.directory.mkdir(parents=True, exist_ok=True)
    with (library.directory / 'robot.lock').open('a') as lock:
        fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
        print(
            json.dumps(
                migrate(library, args.apply), ensure_ascii=False, indent=2
            )
        )
