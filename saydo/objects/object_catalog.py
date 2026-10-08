"""Discover object models without selecting millimetre exports or capture files."""

import json
import hashlib
import os
from pathlib import Path
import tempfile
import time

from saydo.geometry.pose_symmetry import checked_mode
from saydo.perception.local_grounding import read_local_detection
from saydo.perception.local_qwen import read_qwen_settings


def tracking_revision(raw):
    return hashlib.sha256(
        raw if raw is not None else b'no-tracking-config'
    ).hexdigest()


def inferred_class(name):
    key = name.lower()
    if any(token in key for token in ('yibao', 'bottle', '怡宝', '水瓶')):
        return 'bottle'
    if any(
        token in key
        for token in ('mug', 'cup', 'makebei', 'zhibei', '马克杯', '纸杯')
    ):
        return 'cup'
    if any(token in key for token in ('yaokongqi', 'remote', '遥控')):
        return 'remote'
    return None


def scan_objects(root):
    root = Path(root).resolve()
    result = []
    if not root.is_dir():
        return result
    for directory in sorted(root.iterdir()):
        if (
            directory.name.startswith('.')
            or not directory.is_dir()
            or directory.is_symlink()
        ):
            continue
        row = {
            'id': directory.name,
            'name': directory.name,
            'mesh': None,
            'detector_class': inferred_class(directory.name),
            'available': False,
            'reason': None,
            'pose_mode': '6d',
        }
        try:
            config_path = directory / 'tracking.json'
            if config_path.is_symlink():
                raise ValueError('tracking.json 不能是符号链接')
            raw = config_path.read_bytes() if config_path.exists() else None
            config = json.loads(raw) if raw is not None else {}
            if not isinstance(config, dict):
                raise ValueError('tracking.json 必须为 JSON 对象')
            row['name'] = str(config.get('name', directory.name))
            row['local_detection'] = read_local_detection(directory)
            row['qwen_detection'] = read_qwen_settings(directory)
            detection_path = directory / 'detection.json'
            if detection_path.is_symlink():
                raise ValueError('detection.json 不能是符号链接')
            detection_raw = (
                detection_path.read_bytes() if detection_path.exists() else None
            )
            detection = (
                json.loads(detection_raw) if detection_raw is not None else {}
            )
            if not isinstance(detection, dict):
                raise ValueError('detection.json 必须为 JSON 对象')
            prompt = checked_detection_prompt(
                detection.get('detection_prompt', '')
            )
            row.update(
                detection_prompt=prompt,
                effective_detection_prompt=prompt or row['name'],
                detection_prompt_revision=tracking_revision(detection_raw),
            )
            row['pose_mode'] = checked_mode(config.get('pose_mode', '6d'))
            row['pose_mode_revision'] = tracking_revision(raw)
            row['detector_class'] = config.get(
                'detector_class', row['detector_class']
            )
            if (
                row['detector_class'] is not None
                and not isinstance(row['detector_class'], str)
            ):
                raise ValueError('detector_class 必须是类别名称或 null')
            if config.get('units', 'metres') not in ('metres', 'meters', 'm'):
                raise ValueError('模型必须采用米制，请先转换单位')
            names = (
                [config['mesh']]
                if 'mesh' in config
                else [
                    'model/model.obj',
                    'model.obj',
                    'model/model.glb',
                    'model.glb',
                ]
            )
            for name in names:
                candidate = (directory / name).resolve()
                if not candidate.is_relative_to(directory.resolve()):
                    raise ValueError('模型路径必须位于该物品目录内')
                if candidate.suffix.lower() not in ('.obj', '.glb', '.ply'):
                    raise ValueError('支持 OBJ、GLB、PLY 模型')
                if candidate.is_file():
                    row.update(mesh=str(candidate), available=True)
                    break
            if not row['available']:
                row['reason'] = '缺少 model/model.obj 或 model.obj 等模型文件'
        except (ValueError, OSError, TypeError) as error:
            row.update(available=False, reason=str(error))
        result.append(row)
    return result


def select_object(root, object_id):
    row = next(
        (row for row in scan_objects(root) if row['id'] == object_id), None
    )
    if row is None:
        raise ValueError('物品不在模型目录列表中')
    if not row['available']:
        raise ValueError(row['reason'])
    return row


def save_pose_mode(root, object_id, mode, expected_revision):
    """Caller holds the device lock. Preserve all other per-model settings."""
    mode = checked_mode(mode)
    row = select_object(root, object_id)
    path = Path(root) / row['id'] / 'tracking.json'
    raw = path.read_bytes() if path.exists() else None
    if expected_revision != tracking_revision(raw):
        raise ValueError('模型设置已变化，请重新打开模型选择')
    if mode == row['pose_mode']:
        return row
    value = json.loads(raw) if raw is not None else {}
    value['pose_mode'] = mode
    temporary = None
    try:
        with tempfile.NamedTemporaryFile(
            mode='w', dir=path.parent, prefix='.tracking-', delete=False
        ) as output:
            temporary = Path(output.name)
            os.fchmod(
                output.fileno(),
                path.stat().st_mode & 0o777 if raw is not None else 0o644,
            )
            output.write(json.dumps(value, ensure_ascii=False, indent=2) + '\n')
            output.flush()
            os.fsync(output.fileno())
        if (path.read_bytes() if path.exists() else None) != raw:
            raise ValueError('模型设置同时被其他程序修改，未保存')
        if raw is not None:
            backup = path.parent / 'tracking_backups'
            backup.mkdir(exist_ok=True)
            (backup / f'{time.time_ns()}.json').write_bytes(raw)
        temporary.replace(path)
    finally:
        if temporary is not None:
            temporary.unlink(missing_ok=True)
    return select_object(root, object_id)


def checked_detection_prompt(prompt):
    if not isinstance(prompt, str) or len(prompt) > 2000:
        raise ValueError('识别提示词必须是最多2000字的文本')
    return prompt.strip()


def save_detection_prompt(root, object_id, prompt, expected_revision):
    prompt = checked_detection_prompt(prompt)
    row = select_object(root, object_id)
    path = Path(root) / row['id'] / 'detection.json'
    raw = path.read_bytes() if path.exists() else None
    if expected_revision != tracking_revision(raw):
        raise ValueError('识别提示词已变化，请重新打开模型选择')
    if prompt == row['detection_prompt']:
        return row
    value = json.loads(raw) if raw is not None else {}
    value['detection_prompt'] = prompt
    temporary = None
    try:
        with tempfile.NamedTemporaryFile(
            mode='w', dir=path.parent, prefix='.detection-', delete=False
        ) as output:
            temporary = Path(output.name)
            output.write(json.dumps(value, ensure_ascii=False, indent=2) + '\n')
            output.flush()
            os.fsync(output.fileno())
        if (
            path.is_symlink()
            or (path.read_bytes() if path.exists() else None) != raw
        ):
            raise ValueError('识别提示词同时被其他程序修改，未保存')
        if raw is not None:
            backup = path.parent / 'detection_backups'
            if backup.is_symlink():
                raise ValueError('识别配置备份目录不能是符号链接')
            backup.mkdir(exist_ok=True)
            (backup / f'{time.time_ns()}.json').write_bytes(raw)
        temporary.replace(path)
    finally:
        if temporary is not None:
            temporary.unlink(missing_ok=True)
    return select_object(root, object_id)
