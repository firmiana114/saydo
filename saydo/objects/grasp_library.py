"""Versioned, object-relative grasp library. Legacy teaching records remain untouched."""

import copy
import hashlib
import json
import os
import pathlib
import tempfile
import time
import uuid
from typing import Optional

import numpy as np

from saydo.geometry import grasp_geometry

MAX_GRASPS = 32


def revision(value):
    return hashlib.sha256(
        json.dumps(value, sort_keys=True, allow_nan=False).encode()
    ).hexdigest()


def checked_profile(value):
    if not isinstance(value, dict):
        raise ValueError('夹法必须包含抓取中心和朝向')
    point = np.asarray(value.get('grasp_point_object_m'), dtype=float)
    if point.shape != (3,) or not np.isfinite(point).all():
        raise ValueError('抓取中心必须为三个有限米制坐标')
    rotation = np.asarray(value.get('object_R_tcp'), dtype=float)
    if rotation.shape != (3, 3):
        raise ValueError('夹爪朝向必须为 3×3 旋转矩阵')
    matrix = np.eye(4)
    matrix[:3, :3] = rotation
    grasp_geometry.transform(matrix)
    return {
        'orientation': 'follow_object',
        'grasp_point_object_m': point.tolist(),
        'object_R_tcp': rotation.tolist(),
        'approach_axis_tcp': [0, 0, 1],
    }


def checked_overrides(value):
    if (
        not isinstance(value, dict)
        or (
            set(value)
            - {
                'pregrasp_distance_m',
                'open_position',
                'force_target_raw',
                'min_table_clearance_m',
                'tcp_offset_m',
            }
        )
    ):
        raise ValueError('夹法参数覆盖项无效')
    for key, number in value.items():
        if key == 'tcp_offset_m':
            grasp_geometry.checked_tcp_offset(number)
        elif key == 'min_table_clearance_m':
            grasp_geometry.minimum_table_clearance({key: number})
        elif key == 'pregrasp_distance_m':
            if (
                type(number) not in (int, float)
                or not np.isfinite(number)
                or not 0 < number <= .3
            ):
                raise ValueError('预抓距离必须大于 0 且不超过 300 毫米')
        elif (
            type(number) is not int
            or (
                not (0 if key == 'open_position' else 1)
                <= number
                <= (1000 if key == 'open_position' else 100)
            )
        ):
            raise ValueError('张开位置须为 0–1000、力度须为 1–100 的整数原始值')
    return copy.deepcopy(value)


def object_gripper_defaults(object_id):
    """Defaults for newly recorded grasps; execution always uses saved values."""
    name = str(object_id or '').lower().replace('-', '_')
    paper = 'paper_cup' in name or '纸杯' in name
    mug = 'mug' in name or '马克杯' in name or 'makebei' in name
    return {
        'force_target_raw': 10 if paper else 75,
        'open_position': 200 if mug else 1000,
    }


def checked_bound_overrides(value):
    result = checked_overrides(value)
    if not {'force_target_raw', 'open_position'} <= set(result):
        raise ValueError('夹法尚未绑定夹爪力度和抓取张开位置，请更新并保存夹法')
    return result


def point_limits(vertices):
    points = np.asarray(vertices, dtype=float)
    if (
        points.ndim != 2
        or points.shape[1] != 3
        or not len(points)
        or not np.isfinite(points).all()
    ):
        raise ValueError('模型几何无效')
    padding = max(.05, float(np.ptp(points, axis=0).max()) * .25)
    return points.min(axis=0) - padding, points.max(axis=0) + padding


def tool_geometry(tool_response: Optional[dict]) -> Optional[tuple]:
    """取出工具应答里决定夹爪参考点位置的部分：工具名和六维位姿。

    Args:
        tool_response: 控制器对“当前工具”查询的应答；缺失时为 None。

    Returns:
        `(工具名, 位姿)`，位姿是控制器原始单位的六个数；应答缺失时
        为 None。
    """
    if not isinstance(tool_response, dict):
        return None
    return tool_response.get('tool_name'), tool_response.get('pose')


def binding_error(
    entry: dict,
    model_hash: Optional[str],
    tool_response: dict,
    handeye: Optional[list] = None,
) -> Optional[str]:
    """判断一套夹法在当前的模型、工具和手眼标定下是否还能用。

    夹法记录的是夹爪参考点相对物品的位姿。物品模型、工具的参考点
    位置或手眼标定变了，这个相对关系就不再对应实物。

    工具只比较名字和位姿，不比较负载和质心：负载不改变参考点在哪里，
    而两台臂装着同样的夹爪时，控制器里登记的负载会有零点几克的出入
    （1 臂 1146.8 克，2 臂 1147 克）。若连负载一起比，同一套夹法就
    无法在两台臂上通用。执行前另有一步核对控制器当前工具与该臂工具
    记录完全一致，负载的变化在那里把关。

    Args:
        entry: 夹法库里的一条夹法。
        model_hash: 当前物品模型文件的 SHA-256。
        tool_response: 这台臂工具记录里的控制器应答。
        handeye: 这台臂的手眼矩阵；None 表示不检查手眼。

    Returns:
        不能用时返回给操作者看的原因，能用时返回 None。
    """
    if entry.get('model_sha256') != model_hash:
        return '物品模型已变化，请重新设置夹法'
    if tool_geometry(entry.get('tool_response')) != tool_geometry(
        tool_response
    ):
        return '工具参数已变化，请重新设置夹法'
    if (
        entry.get('flange_T_camera') is not None
        and handeye is not None
        and entry['flange_T_camera'] != handeye
    ):
        return '手眼标定已变化，请重新记录夹法'
    return None


def effective_config(config, entry):
    result = copy.deepcopy(config)
    overrides = checked_bound_overrides(entry.get('overrides', {}))
    result['tcp_offset_m'] = grasp_geometry.checked_tcp_offset(
        overrides.get('tcp_offset_m', [0, 0, 0])
    )
    if 'pregrasp_distance_m' in overrides:
        result['pregrasp_distance_m'] = overrides['pregrasp_distance_m']
    if 'min_table_clearance_m' in overrides:
        result['min_table_clearance_m'] = overrides['min_table_clearance_m']
    if 'open_position' in overrides:
        result['gripper']['open_position'] = overrides['open_position']
    if 'force_target_raw' in overrides:
        result['gripper']['force_target_raw'] = overrides['force_target_raw']
        result['gripper']['force_limit_raw'] = overrides['force_target_raw']
    return result


def effective_parameters(config, entry):
    result = effective_config(config, entry)
    return {
        'gripper_parameters_source': 'grasp_profile',
        'force_target_raw': result['gripper']['force_target_raw'],
        'tcp_offset_m': result['tcp_offset_m'],
        'tcp_offset_frame': 'grip_tcp',
        'force_limit_raw': result['gripper']['force_limit_raw'],
        'open_position': result['gripper']['open_position'],
        'force_units': result['gripper'].get(
            'force_units', 'native_units_not_newtons'
        ),
        'open_position_units': 'native_position_0_to_1000',
        'pregrasp_distance_m': result.get('pregrasp_distance_m'),
        'min_table_clearance_m': grasp_geometry.minimum_table_clearance(result),
        'table_clearance_source': (
            'grasp_profile'
            if 'min_table_clearance_m' in entry.get('overrides', {})
            else 'global_default'
        ),
        'lift_height_m': result.get('lift_height_m'),
        'arm_speed_percent': result.get('arm_speed_percent'),
    }


def held_clearance_config(config, held):
    """Keep the clearance of the actual grasp, independent of later editor saves."""
    result = copy.deepcopy(config)
    if 'min_table_clearance_m' in held:
        result['min_table_clearance_m'] = (
            grasp_geometry.minimum_table_clearance(held)
        )
    return result


class GraspLibrary:
    def __init__(self, root, models_root=None):
        self.directory = pathlib.Path(root) / 'work/grasp'
        self.models_root = (
            pathlib.Path(models_root)
            if models_root is not None
            else pathlib.Path(root) / 'work/object_models'
        )
        # Read-only compatibility sources. New writes always belong to one
        # model.
        self.path = self.directory / 'grasp_profiles.json'
        self.legacy = self.directory / 'taught_profiles.json'

    def object_directory(self, object_id):
        if (
            not isinstance(object_id, str)
            or not object_id
            or object_id.startswith('.')
            or '/' in object_id
            or '\\' in object_id
            or '\x00' in object_id
        ):
            raise ValueError('物品 ID 必须是模型目录名称')
        directory = self.models_root / object_id
        if (
            directory.is_symlink()
            or not directory.resolve().is_relative_to(
                self.models_root.resolve()
            )
        ):
            raise ValueError('物品数据路径必须位于模型目录内')
        return directory

    def object_path(self, object_id):
        path = self.object_directory(object_id) / 'grasp_profiles.json'
        if path.is_symlink():
            raise ValueError('夹法文件不能是符号链接')
        return path

    def legacy_data(self):
        if not self.path.exists():
            return {'schema_version': 2, 'objects': {}}
        data = json.loads(self.path.read_text())
        if (
            not isinstance(data, dict)
            or data.get('schema_version') != 2
            or not isinstance(data.get('objects'), dict)
        ):
            raise ValueError('夹法库版本或格式无效')
        return data

    def has_object(self, object_id):
        return (
            self.object_path(object_id).exists()
            or object_id in self.legacy_data()['objects']
        )

    @staticmethod
    def checked_object(value):
        if (
            not isinstance(value, dict)
            or value.get('selection_mode') not in ('auto', 'manual')
            or not isinstance(value.get('grasps'), list)
            or len(value['grasps']) > MAX_GRASPS
        ):
            raise ValueError('物品夹法列表格式无效')
        ids = [g['id'] for g in value['grasps']]
        if len(set(ids)) != len(ids):
            raise ValueError('夹法编号重复')
        return copy.deepcopy(value)

    def object(self, object_id):
        path = self.object_path(object_id)
        if path.exists():
            data = json.loads(path.read_text())
            if (
                not isinstance(data, dict)
                or data.get('schema_version') != 3
                or data.get('object_id') != object_id
            ):
                raise ValueError('模型夹法文件版本或物品 ID 不匹配')
            return self.checked_object(data['configuration'])
        data = self.legacy_data()
        if object_id in data['objects']:
            return self.checked_object(data['objects'][object_id])
        records = (
            json.loads(self.legacy.read_text()) if self.legacy.exists() else {}
        )
        old = records.get(object_id)
        grasps = []
        if old is not None:
            grasps.append(
                {
                    **copy.deepcopy(old),
                    'id': 'legacy',
                    'name': '原有示教夹法',
                    'enabled': True,
                    'priority': 0,
                    'source': 'taught',
                    'overrides': {},
                    'tested_successfully': False,
                }
            )
        return {'selection_mode': 'auto', 'selected_id': None, 'grasps': grasps}

    def write_object(self, object_id, value, expected_revision):
        """Caller holds the shared task/device lock; also reject external edits."""
        path = self.object_path(object_id)
        directory = path.parent
        original = path.read_bytes() if path.exists() else None
        if revision(self.object(object_id)) != expected_revision:
            raise ValueError('夹法已被其他页面修改，请重新载入')
        data = {
            'schema_version': 3,
            'object_id': object_id,
            'configuration': self.checked_object(value),
        }
        directory.mkdir(parents=True, exist_ok=True)
        temporary = None
        try:
            with tempfile.NamedTemporaryFile(
                mode='w', dir=directory, prefix='.grasps-', delete=False
            ) as out:
                temporary = pathlib.Path(out.name)
                os.fchmod(
                    out.fileno(),
                    (
                        path.stat().st_mode & 0o777
                        if original is not None
                        else 0o644
                    ),
                )
                out.write(
                    json.dumps(
                        data, ensure_ascii=False, indent=2, allow_nan=False
                    )
                    + '\n'
                )
                out.flush()
                os.fsync(out.fileno())
            if (
                (path.read_bytes() if path.exists() else None) != original
                or revision(self.object(object_id)) != expected_revision
            ):
                raise ValueError('夹法文件同时被其他程序修改，未保存')
            if original is not None:
                backups = directory / 'grasp_profile_backups'
                backups.mkdir(exist_ok=True)
                (backups / f'{time.time_ns()}.json').write_bytes(original)
            temporary.replace(path)
        finally:
            if temporary is not None:
                temporary.unlink(missing_ok=True)

    def append_taught(self, object_id, record):
        value = self.object(object_id)
        before = revision(value)
        if len(value['grasps']) >= MAX_GRASPS:
            raise ValueError(
                f'每个物品最多保存 {MAX_GRASPS} 套夹法，请先删除不需要的记录'
            )
        entry = {
            **copy.deepcopy(record),
            'id': uuid.uuid4().hex,
            'name': time.strftime('示教夹法 %m-%d %H:%M:%S'),
            'enabled': True,
            'priority': 0,
            'overrides': checked_bound_overrides(
                {
                    **object_gripper_defaults(object_id),
                    **record.get('overrides', {}),
                }
            ),
            'source': 'taught',
            'tested_successfully': False,
        }
        value['grasps'].append(entry)
        value.update(selection_mode='manual', selected_id=entry['id'])
        self.write_object(object_id, value, before)
        return entry

    def mark_success(
        self, object_id, grasp_id, expected_revision, pose_mode='6d'
    ):
        value = self.object(object_id)
        if revision(value) != expected_revision:
            raise ValueError('夹法已变化，未将旧任务结果标记到新版本')
        entry = next((g for g in value['grasps'] if g['id'] == grasp_id), None)
        if entry is None:
            return
        if pose_mode == '5d':
            entry.update(
                tested_successfully_5d=True, last_success_at_5d=time.time()
            )
        else:
            entry.update(tested_successfully=True, last_success_at=time.time())
        self.write_object(object_id, value, expected_revision)
