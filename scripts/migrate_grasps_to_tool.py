"""工具原点沿 Z 改动之后，把夹法库里的夹法换算到新工具；默认只预览。

背景：夹法记录的是“夹爪参考点相对物品”的位姿。在示教器上把工具
`grip_tcp` 的 Z 改长之后（2026-10-09 由 222.5 毫米改为 227.5 毫米，使
参考点落在真正的指尖上），参考点沿工具 Z 往前挪了一段。为了让夹爪在
实物上的位置保持不变，每套夹法的参考点偏移要沿工具 Z 往回退同样的
距离。

另外可以给指定物品的夹法再加一个高度修正（`--raise`），用于相机校正
之后物品高度读数变低的情况：原来靠偏移量抵消的高度偏差不再存在，偏移
要相应减小。

本脚本只改夹法库文件，不连接控制器。夹法库各臂共用，写入期间占住全部
臂的设备锁。

用法（在项目根目录）：

    python -m scripts.migrate_grasps_to_tool \
        --tool-file config/calibration/arm1_tool_grip_tcp.yaml          预览
    python -m scripts.migrate_grasps_to_tool --tool-file ... --apply    写入
"""

from __future__ import annotations

import argparse
import copy
import json
import logging
import pathlib
import time
from typing import Dict, Optional, Sequence

import yaml

from saydo import paths
from saydo.objects import grasp_library
from saydo.robot import arms

LOG = logging.getLogger('migrate_grasps_to_tool')
# 原始单位：0.001 毫米。
RAW_PER_M = 1e6


def migrated_entry(
    entry: dict, new_tool: dict, raise_m: float, at: float
) -> Optional[dict]:
    """把一套夹法换算到新工具。

    Args:
        entry: 夹法库里的一条夹法。
        new_tool: 控制器对新工具的读回。
        raise_m: 额外的高度修正，单位米；正值让夹爪参考点相对物品沿
            接近方向后退（竖直向下抓取时即抬高）。
        at: 记录到迁移说明里的时间（Unix 秒）。

    Returns:
        换算后的夹法；这套夹法已经绑定在新工具上时返回 None。

    Raises:
        ValueError: 夹法绑定的旧工具与新工具除 Z 之外还有别的差别
            （名字、X、Y 或朝向），无法只靠沿 Z 的偏移来换算。
    """
    old_tool = entry.get('tool_response') or {}
    if grasp_library.tool_geometry(old_tool) == grasp_library.tool_geometry(
        new_tool
    ):
        return None
    old_pose, new_pose = old_tool.get('pose'), new_tool['pose']
    if (
        old_tool.get('tool_name') != new_tool.get('tool_name')
        or not isinstance(old_pose, list)
        or len(old_pose) != 6
        or old_pose[:2] + old_pose[3:] != new_pose[:2] + new_pose[3:]
    ):
        raise ValueError(
            f"夹法 {entry.get('id')} 绑定的工具（{old_tool.get('tool_name')} "
            f'{old_pose}）与新工具（{new_pose}）不只是 Z 不同，未换算'
        )
    extension_m = (new_pose[2] - old_pose[2]) / RAW_PER_M
    result = copy.deepcopy(entry)
    overrides = dict(result.get('overrides', {}))
    before = list(overrides.get('tcp_offset_m', [0., 0., 0.]))
    # 偏移的约定：参考点目标 = 夹持点 − R·偏移，所以偏移的 Z 变大表示
    # 参考点沿接近方向后退。工具变长多少，参考点就要多前进多少才能让
    # 法兰留在原处；额外的高度修正则让它后退。
    after = [before[0], before[1], before[2] - extension_m + raise_m]
    overrides['tcp_offset_m'] = [round(value, 6) for value in after]
    result['overrides'] = grasp_library.checked_bound_overrides(overrides)
    result.update(
        tool_response=copy.deepcopy(new_tool),
        tested_successfully=False,
        tested_successfully_5d=False,
    )
    result.setdefault('tool_migrations', []).append(
        {
            'at': at,
            'previous_tool_pose_raw': old_pose,
            'new_tool_pose_raw': new_pose,
            'tool_extension_m': extension_m,
            'height_raise_m': raise_m,
            'tcp_offset_before_m': before,
            'tcp_offset_after_m': result['overrides']['tcp_offset_m'],
            'basis': 'keep_flange_target_then_apply_height_raise',
        }
    )
    return result


def migrate(
    library: grasp_library.GraspLibrary,
    new_tool: dict,
    raises_m: Dict[str, float],
    apply: bool = False,
    at: Optional[float] = None,
) -> dict:
    """换算夹法库里全部物品的夹法；`apply` 为 False 时只预览不写文件。

    先把所有物品检查一遍，全部可以换算才开始写第一个文件，避免只改了
    一半。

    Args:
        library: 夹法库。
        new_tool: 控制器对新工具的读回。
        raises_m: 物品编号到额外高度修正（米）的映射。
        apply: 是否写入。
        at: 记录到迁移说明里的时间；缺省取当前时间。

    Returns:
        摘要：每个物品各换算了哪些夹法、偏移从多少变为多少。

    Raises:
        ValueError: 有夹法无法换算，或 `raises_m` 里的物品不存在。
    """
    at = time.time() if at is None else at
    changes = []
    rows = []
    seen = set()
    for path in sorted(library.models_root.glob('*/grasp_profiles.json')):
        object_id = path.parent.name
        seen.add(object_id)
        before = library.object(object_id)
        after = copy.deepcopy(before)
        for index, entry in enumerate(before['grasps']):
            updated = migrated_entry(
                entry, new_tool, raises_m.get(object_id, 0.), at
            )
            if updated is None:
                continue
            after['grasps'][index] = updated
            rows.append(
                {
                    'object_id': object_id,
                    'grasp_id': entry['id'],
                    'name': entry.get('name'),
                    **updated['tool_migrations'][-1],
                }
            )
        if before != after:
            changes.append((object_id, grasp_library.revision(before), after))
    unknown = sorted(set(raises_m) - seen)
    if unknown:
        raise ValueError(
            '--raise 指定的物品没有夹法文件：' + '、'.join(unknown)
        )
    if apply:
        for object_id, expected, value in changes:
            library.write_object(object_id, value, expected)
            LOG.info(
                '已写入 %s 的夹法（%d 套）', object_id, len(value['grasps'])
            )
    return {
        'applied': apply,
        'objects': [row[0] for row in changes],
        'grasps_migrated': len(rows),
        'grasps': rows,
        'motion_sent': False,
    }


def parse_raise(text: str) -> tuple:
    """解析 `--raise 物品编号=毫米`。

    Args:
        text: 命令行里的一项。

    Returns:
        `(物品编号, 高度修正米数)`。

    Raises:
        argparse.ArgumentTypeError: 格式不对，或数值超出 ±30 毫米。
    """
    object_id, separator, value = text.partition('=')
    try:
        raise_mm = float(value)
    except ValueError as error:
        raise argparse.ArgumentTypeError(
            '格式为 物品编号=毫米，例如 tube=9.1'
        ) from error
    if not separator or not object_id or not abs(raise_mm) <= 30:
        raise argparse.ArgumentTypeError('格式为 物品编号=毫米，且不超过 30')
    return object_id, raise_mm / 1000


def main(argv: Optional[Sequence[str]] = None) -> int:
    """解析命令行并换算夹法；不加 `--apply` 时只预览。

    Args:
        argv: 命令行参数；None 表示取进程的参数。

    Returns:
        进程退出码：0 表示成功（预览或写入），1 表示未执行。
    """
    parser = argparse.ArgumentParser(description=__doc__.split('\n')[0])
    parser.add_argument(
        '--tool-file', required=True, help='新工具的记录文件，相对项目根目录'
    )
    parser.add_argument('--models-root', type=pathlib.Path)
    parser.add_argument(
        '--raise',
        dest='raises',
        type=parse_raise,
        action='append',
        default=[],
        metavar='物品编号=毫米',
        help='给该物品的夹法额外加的高度修正，可重复',
    )
    parser.add_argument('--apply', action='store_true', help='写入夹法库')
    arguments = parser.parse_args(argv)
    logging.basicConfig(level=logging.INFO, format='%(levelname)s %(message)s')
    root = paths.ROOT
    try:
        tool = yaml.safe_load((root / arguments.tool_file).read_text())
        new_tool = tool['controller_readback']['tool_response_raw']
        library = grasp_library.GraspLibrary(root, arguments.models_root)
        with arms.all_locked(root):
            summary = migrate(
                library, new_tool, dict(arguments.raises), arguments.apply
            )
    except (ValueError, OSError, KeyError) as error:
        LOG.error('未换算：%s', error)
        return 1
    print(json.dumps(summary, ensure_ascii=False, indent=2))
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
