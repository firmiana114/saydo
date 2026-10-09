"""把一台机械臂当前选中的工具读回并写成工具记录文件；只读查询，不发指令。

抓取程序在每次动作前都会核对“控制器当前的工具”与“工具记录文件”完全
一致，不一致就拒绝动作。所以在示教器上新建或修改工具之后，要用本脚本
重新读回一次。本脚本不向控制器写入任何内容，也不让机械臂运动。

用法（在项目根目录）：

    python -m scripts.record_arm_tool --arm arm2 --robot-ip 192.168.1.18

读回的原始应答保存在 `work/calibration/` 下，工具记录写到
`config/calibration/<臂编号>_tool_grip_tcp.yaml`。已有同名记录时要加
`--replace`，旧记录会先备份。
"""

from __future__ import annotations

import argparse
import datetime
import hashlib
import json
import logging
import pathlib
import shutil
from typing import Optional, Sequence

import yaml

from saydo import paths
from saydo.geometry import grasp_geometry
from saydo.robot import arms
from saydo.robot import grasp_robot

LOG = logging.getLogger('record_arm_tool')
READ_ONLY_QUERIES = (
    'get_arm_software_info',
    'get_current_tool_frame',
    'get_current_work_frame',
)


def tool_record(
    arm: arms.Arm,
    replies: dict,
    readback_file: str,
    readback_sha256: str,
    tool_name: str,
) -> dict:
    """由控制器的应答生成一份工具记录。

    Args:
        arm: 这份记录属于哪台臂。
        replies: 查询名到控制器应答的映射。
        readback_file: 保存原始应答的文件，相对项目根目录。
        readback_sha256: 该文件内容的 SHA-256。
        tool_name: 要求控制器当前选中的工具名。

    Returns:
        工具记录，含夹爪参考点相对法兰的变换 `flange_T_grip_tcp`（平移
        单位米）和供执行前核对用的控制器原始应答。

    Raises:
        ValueError: 当前选中的不是要求的工具，或工作坐标系不是零偏移的
            `Base`。报错里写明当前的名字。
    """
    tool = replies['get_current_tool_frame']
    work = replies['get_current_work_frame']
    if tool.get('tool_name') != tool_name:
        raise ValueError(
            f"{arm.label}当前选中的工具是 {tool.get('tool_name')}，"
            f'需要先在示教器上切换到 {tool_name} 再读回'
        )
    if work.get('frame_name') != 'Base' or work.get('pose') != [0] * 6:
        raise ValueError(
            f"{arm.label}当前工作坐标系是 {work.get('frame_name')}"
            f"（偏移 {work.get('pose')}），需要零偏移的 Base"
        )
    pose = tool['pose']
    return {
        'schema_version': 1,
        'arm': arm.arm_id,
        'tool_name': tool_name,
        'frame_convention': 'A_T_B maps coordinates in B into A',
        'units': {'translation': 'm', 'rotation': 'rad'},
        # 控制器原始单位：平移 0.001 毫米，转角 0.001 弧度。
        'translation_m': [value / 1e6 for value in pose[:3]],
        'rpy_rad': [value / 1e3 for value in pose[3:]],
        'rpy_convention': 'Rz(yaw) @ Ry(pitch) @ Rx(roll)',
        'flange_T_grip_tcp': grasp_geometry.controller_pose(pose).tolist(),
        'load': {
            'mass_kg': tool.get('payload', 0) / 1000,
            'center_of_mass_m': [
                value / 1e6 for value in tool.get('position', [0, 0, 0])
            ],
        },
        'controller_software': replies['get_arm_software_info'],
        'controller_readback': {
            'path': readback_file,
            'path_base': 'repository_root',
            'sha256': readback_sha256,
            'tool_was_active': True,
            'tool_response_raw': tool,
            'work_frame': work,
        },
        'geometry_status': 'controller_readback_not_geometrically_validated',
        'controller_written_by_this_change': False,
    }


def record(
    root: pathlib.Path,
    arm: arms.Arm,
    robot_ip: str,
    tool_name: str = 'grip_tcp',
    replace: bool = False,
    stamp: Optional[str] = None,
) -> pathlib.Path:
    """读回一台臂当前的工具并写出工具记录；只读查询，不发送任何指令。

    Args:
        root: 项目根目录。
        arm: 要读回的臂。
        robot_ip: 该臂控制器的地址。
        tool_name: 要求控制器当前选中的工具名。
        replace: 已有记录时是否替换（旧记录会备份到原始应答的目录）。
        stamp: 用在目录名里的时间标记；缺省取当前的协调世界时。

    Returns:
        写出的工具记录文件路径。

    Raises:
        ValueError: 工具或工作坐标系不符合要求。
        FileExistsError: 已有记录且没有要求替换。
    """
    target = root / f'config/calibration/{arm.arm_id}_tool_{tool_name}.yaml'
    if target.exists() and not replace:
        raise FileExistsError(
            f'{target.relative_to(root)} 已存在；确认要替换时加 --replace'
        )
    robot = grasp_robot.RobotClient(robot_ip, 8080, writes=False)
    replies = {name: robot.query(name) for name in READ_ONLY_QUERIES}
    stamp = stamp or datetime.datetime.now(datetime.timezone.utc).strftime(
        '%Y%m%dT%H%M%SZ'
    )
    readback_file = (
        f'work/calibration/{arm.arm_id}_tool_{stamp}/controller_readback.json'
    )
    readback_text = json.dumps(
        {'robot_ip': robot_ip, 'read_at': stamp, 'replies': replies},
        ensure_ascii=False,
        indent=2,
    )
    value = tool_record(
        arm,
        replies,
        readback_file,
        hashlib.sha256(readback_text.encode()).hexdigest(),
        tool_name,
    )
    readback_path = root / readback_file
    readback_path.parent.mkdir(parents=True, exist_ok=False)
    readback_path.write_text(readback_text)
    if target.exists():
        shutil.copy2(target, readback_path.parent / (target.name + '.before'))
    target.parent.mkdir(parents=True, exist_ok=True)
    target.write_text(
        yaml.safe_dump(value, allow_unicode=True, sort_keys=False)
    )
    LOG.info(
        '已写出 %s 的工具记录 %s（工具 %s，原点 %s 米，负载 %.4f 千克）',
        arm.arm_id,
        target.relative_to(root),
        tool_name,
        value['translation_m'],
        value['load']['mass_kg'],
    )
    return target


def main(argv: Optional[Sequence[str]] = None) -> int:
    """解析命令行并读回工具；只读查询，不发送任何指令。

    Args:
        argv: 命令行参数；None 表示取进程的参数。

    Returns:
        进程退出码：0 表示已写出记录，1 表示未写出（原因已记录）。
    """
    parser = argparse.ArgumentParser(description=__doc__.split('\n')[0])
    parser.add_argument('--arm', required=True, help='臂编号，例如 arm2')
    parser.add_argument('--robot-ip', required=True, help='该臂控制器的地址')
    parser.add_argument('--tool-name', default='grip_tcp')
    parser.add_argument(
        '--replace', action='store_true', help='替换已有记录（旧记录会备份）'
    )
    arguments = parser.parse_args(argv)
    logging.basicConfig(level=logging.INFO, format='%(levelname)s %(message)s')
    try:
        record(
            paths.ROOT,
            arms.by_id(arguments.arm),
            arguments.robot_ip,
            arguments.tool_name,
            arguments.replace,
        )
    except (ValueError, OSError, RuntimeError) as error:
        LOG.error('未写出工具记录：%s', error)
        return 1
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
