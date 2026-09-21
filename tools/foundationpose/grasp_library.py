"""Versioned, object-relative grasp library. Legacy teaching records remain untouched."""
import copy
import hashlib
import json
import os
from pathlib import Path
import tempfile
import time
import uuid

import numpy as np

from grasp_geometry import transform

MAX_GRASPS = 32


def revision(value):
    return hashlib.sha256(json.dumps(value, sort_keys=True, allow_nan=False).encode()).hexdigest()


def checked_profile(value):
    if not isinstance(value, dict):
        raise ValueError('夹法必须包含抓取中心和朝向')
    point = np.asarray(value.get('grasp_point_object_m'), dtype=float)
    if point.shape != (3,) or not np.isfinite(point).all():
        raise ValueError('抓取中心必须为三个有限米制坐标')
    rotation = np.asarray(value.get('object_R_tcp'), dtype=float)
    if rotation.shape != (3, 3):
        raise ValueError('夹爪朝向必须为 3×3 旋转矩阵')
    matrix = np.eye(4);matrix[:3, :3] = rotation;transform(matrix)
    return {'orientation': 'follow_object', 'grasp_point_object_m': point.tolist(),
            'object_R_tcp': rotation.tolist(), 'approach_axis_tcp': [0, 0, 1]}


def checked_overrides(value):
    if not isinstance(value, dict) or set(value) - {'pregrasp_distance_m', 'open_position', 'force_target_raw'}:
        raise ValueError('夹法参数覆盖项无效')
    for key, number in value.items():
        if key == 'pregrasp_distance_m':
            if type(number) not in (int, float) or not np.isfinite(number) or not 0 < number <= .3:
                raise ValueError('预抓距离必须大于 0 且不超过 300 毫米')
        elif type(number) is not int or not (0 if key == 'open_position' else 1) <= number <= (1000 if key == 'open_position' else 100):
            raise ValueError('张开位置须为 0–1000、力度须为 1–100 的整数原始值')
    return copy.deepcopy(value)


def point_limits(vertices):
    points = np.asarray(vertices, dtype=float)
    if points.ndim != 2 or points.shape[1] != 3 or not len(points) or not np.isfinite(points).all():
        raise ValueError('模型几何无效')
    padding = max(.05, float(np.ptp(points, axis=0).max()) * .25)
    return points.min(axis=0)-padding, points.max(axis=0)+padding


def binding_error(entry, model_hash, tool_response, handeye=None):
    if entry.get('model_sha256') != model_hash:
        return '物品模型已变化，请重新设置夹法'
    if entry.get('tool_response') != tool_response:
        return '工具参数已变化，请重新设置夹法'
    if entry.get('flange_T_camera') is not None and handeye is not None and entry['flange_T_camera'] != handeye:
        return '手眼标定已变化，请重新记录夹法'
    return None


def effective_config(config, entry):
    result = copy.deepcopy(config);overrides = checked_overrides(entry.get('overrides', {}))
    if 'pregrasp_distance_m' in overrides:
        result['pregrasp_distance_m'] = overrides['pregrasp_distance_m']
    if 'open_position' in overrides:
        result['gripper']['open_position'] = overrides['open_position']
    if 'force_target_raw' in overrides:
        result['gripper']['force_target_raw'] = overrides['force_target_raw']
        result['gripper']['force_limit_raw'] = overrides['force_target_raw']
    return result


class GraspLibrary:
    def __init__(self, root):
        self.directory = Path(root)/'work/grasp'
        self.path = self.directory/'grasp_profiles.json'
        self.legacy = self.directory/'taught_profiles.json'

    def read(self):
        if not self.path.exists():
            return {'schema_version': 2, 'objects': {}}
        data = json.loads(self.path.read_text())
        if not isinstance(data, dict) or data.get('schema_version') != 2 or not isinstance(data.get('objects'), dict):
            raise ValueError('夹法库版本或格式无效')
        return data

    def object(self, object_id, data=None):
        data = self.read() if data is None else data
        if object_id in data['objects']:
            value = copy.deepcopy(data['objects'][object_id])
            if (not isinstance(value, dict) or value.get('selection_mode') not in ('auto', 'manual')
                    or not isinstance(value.get('grasps'), list) or len(value['grasps']) > MAX_GRASPS):
                raise ValueError('物品夹法列表格式无效')
            ids = [g['id'] for g in value['grasps']]
            if len(set(ids)) != len(ids):
                raise ValueError('夹法编号重复')
            return value
        records = json.loads(self.legacy.read_text()) if self.legacy.exists() else {}
        old = records.get(object_id)
        grasps = []
        if old is not None:
            grasps.append({**copy.deepcopy(old), 'id': 'legacy', 'name': '原有示教夹法', 'enabled': True,
                           'priority': 0, 'source': 'taught', 'overrides': {}, 'tested_successfully': False})
        return {'selection_mode': 'auto', 'selected_id': None, 'grasps': grasps}

    def write_object(self, object_id, value, expected_revision):
        """Caller holds the shared task/device lock; also reject external edits."""
        original = self.path.read_bytes() if self.path.exists() else None
        data = self.read()
        if revision(self.object(object_id, data)) != expected_revision:
            raise ValueError('夹法已被其他页面修改，请重新载入')
        # Preserve all legacy objects on first write, without rewriting the source file.
        if self.legacy.exists():
            for key in json.loads(self.legacy.read_text()):
                if key not in data['objects']:
                    data['objects'][key] = self.object(key, data)
        data['objects'][object_id] = copy.deepcopy(value)
        self.directory.mkdir(parents=True, exist_ok=True)
        temporary = None
        try:
            with tempfile.NamedTemporaryFile(mode='w', dir=self.directory, prefix='.grasps-', delete=False) as out:
                temporary = Path(out.name)
                os.fchmod(out.fileno(), self.path.stat().st_mode & 0o777 if original is not None else 0o644)
                out.write(json.dumps(data, ensure_ascii=False, indent=2, allow_nan=False)+'\n');out.flush();os.fsync(out.fileno())
            if (self.path.read_bytes() if self.path.exists() else None) != original:
                raise ValueError('夹法文件同时被其他程序修改，未保存')
            if original is not None:
                backups = self.directory/'grasp_profile_backups';backups.mkdir(exist_ok=True)
                (backups/f'{time.time_ns()}.json').write_bytes(original)
            temporary.replace(self.path)
        finally:
            if temporary is not None:
                temporary.unlink(missing_ok=True)

    def append_taught(self, object_id, record):
        value = self.object(object_id);before = revision(value)
        if len(value['grasps']) >= MAX_GRASPS:
            raise ValueError(f'每个物品最多保存 {MAX_GRASPS} 套夹法，请先删除不需要的记录')
        entry = {**copy.deepcopy(record), 'id': uuid.uuid4().hex, 'name': time.strftime('示教夹法 %m-%d %H:%M:%S'),
                 'enabled': True, 'priority': 0, 'overrides': {}, 'source': 'taught', 'tested_successfully': False}
        value['grasps'].append(entry)
        value.update(selection_mode='manual', selected_id=entry['id'])
        self.write_object(object_id, value, before)
        return entry

    def mark_success(self, object_id, grasp_id, expected_revision):
        value = self.object(object_id)
        if revision(value) != expected_revision:
            raise ValueError('夹法已变化，未将旧任务结果标记到新版本')
        entry = next((g for g in value['grasps'] if g['id']==grasp_id), None)
        if entry is None:
            return
        entry.update(tested_successfully=True, last_success_at=time.time())
        self.write_object(object_id, value, expected_revision)
