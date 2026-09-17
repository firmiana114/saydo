"""Discover object models without selecting millimetre exports or capture files."""
import json
from pathlib import Path


def inferred_class(name):
    key = name.lower()
    if any(token in key for token in ('yibao', 'bottle', '怡宝', '水瓶')):
        return 'bottle'
    if any(token in key for token in ('mug', 'cup', 'makebei', 'zhibei', '马克杯', '纸杯')):
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
        if directory.name.startswith('.') or not directory.is_dir() or directory.is_symlink():
            continue
        row = {'id': directory.name, 'name': directory.name, 'mesh': None,
               'detector_class': inferred_class(directory.name), 'available': False, 'reason': None}
        try:
            config_path = directory / 'tracking.json'
            config = json.loads(config_path.read_text()) if config_path.exists() else {}
            if not isinstance(config, dict):
                raise ValueError('tracking.json 必须为 JSON 对象')
            row['name'] = str(config.get('name', directory.name))
            row['detector_class'] = config.get('detector_class', row['detector_class'])
            if row['detector_class'] is not None and not isinstance(row['detector_class'], str):
                raise ValueError('detector_class 必须是类别名称或 null')
            if config.get('units', 'metres') not in ('metres', 'meters', 'm'):
                raise ValueError('模型必须采用米制，请先转换单位')
            names = [config['mesh']] if 'mesh' in config else ['model/model.obj', 'model.obj', 'model/model.glb', 'model.glb']
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
    row = next((row for row in scan_objects(root) if row['id'] == object_id), None)
    if row is None:
        raise ValueError('物品不在模型目录列表中')
    if not row['available']:
        raise ValueError(row['reason'])
    return row
