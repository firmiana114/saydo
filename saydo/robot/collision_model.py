"""机械臂和夹爪的几何外形，以及它们与桌面、立杆之间的距离检查。

为什么需要它：逆解和路径检查原先只看夹爪参考点（指尖）这一个点，不知道
手肘、前臂、手腕和夹爪本体占多大地方，也不知道桌面上立着一根装全局相机
的杆子。本模块给每一段一个简单的外形，算出它们离障碍物有多近。

外形用胶囊体（capsule）表示：一条线段加一个半径，即两头是半球的圆柱。
它算距离很简单，又能把手臂的一段包住。

- 机械臂 8 段（底座加 7 个连杆）：按厂家模型文件
  （`rm_description/meshes/rm_75_arm/*.STL`）逐段拟合，半径已加 1 毫米。
  运动学参数与厂家模型逐连杆核对过，位姿相差小于 0.02 毫米；与控制器
  读数相差约 0.2 毫米。
- 夹爪：按知行 CTAG2F90D 的产品外形图。图上闭合总长 222.5 毫米、张到
  最大时总宽 151 毫米（此时指尖缩回到 205 毫米）、本体厚 45 毫米、
  指垫宽 23 毫米、安装座直径 75 毫米。外形取“任何开度下都包得住”的
  范围，所以不随开度变化。夹爪上的相机和支架不在模型里（用户决定）。

没有建模的东西：夹爪上的相机、被夹着的物品、桌上的物品、全局相机本体、
另一台机械臂。

本模块只做几何计算，不连接控制器。
"""

from __future__ import annotations

import dataclasses
import pathlib
from typing import List, Optional, Sequence, Tuple

import numpy as np
import yaml

OBSTACLES_FILE = 'config/calibration/workspace_obstacles.yaml'
SCHEMA_VERSION = 1

# RM75-B 的改进型 DH 参数：每行是 (alpha 度, a 米, d 米)。
DH_PARAMETERS = (
    (0., 0., .2405),
    (-90., 0., 0.),
    (90., 0., .256),
    (-90., 0., 0.),
    (90., 0., .210),
    (-90., 0., 0.),
    (90., 0., .144),
)

# 各段的胶囊体：(所在连杆的序号, 线段起点, 线段终点, 半径)，坐标在该连杆
# 自己的坐标系里，单位米。序号 0 是底座，1～7 是随第 1～7 关节转动的连杆。
ARM_CAPSULES = (
    (0, (-.0167, .0016, .0140), (-.0273, .0089, .1175), .0740),
    (1, (-.0004, -.0214, -.0845), (-.0004, -.0214, .0230), .0658),
    (2, (.0008, .0010, .0388), (-.0010, -.1251, -.0081), .0521),
    (3, (-.0003, -.0189, -.0885), (-.0003, -.0189, .0190), .0570),
    (4, (-.0007, -.1078, -.0057), (.0003, .0020, .0351), .0414),
    (5, (-.0001, -.0146, -.0690), (-.0001, -.0146, .0160), .0463),
    (6, (.0002, -.0074, .0279), (-.0008, -.0889, -.0066), .0448),
    (7, (.0035, -.0005, -.0101), (.0065, -.0005, -.0101), .0387),
)
ARM_LABELS = (
    '底座', '肩部', '上臂', '肘部', '前臂', '腕部', '手腕', '法兰'
)
# 底座和肩部固定在桌面上方，不参与桌面检查。
TABLE_EXEMPT_LINKS = (0, 1)

# 夹爪外形图上的闭合总长：安装面到闭合指尖。夹爪参考点（grip_tcp）就在
# 闭合指尖，所以夹爪各部分的位置按“距指尖多远”来写，与安装垫片厚度无关。
GRIPPER_CLOSED_LENGTH_M = .2225
# 夹爪的胶囊体：(名称, 起点, 终点, 半径)，坐标在夹爪参考点坐标系里
# （Z 沿夹爪轴指向指尖外，X 是两指开合的方向），单位米。
# 图上的高度 h（从安装面量起）换成这里的 Z 是 h − 0.2225。
GRIPPER_CAPSULES = (
    # 本体：安装座直径 75 毫米，一直到连杆根部（图上约 105 毫米处）。
    ('夹爪本体', (0., 0., -.2275), (0., 0., -.1550), .0375),
    # 连杆与张开的手指：宽 151 毫米、厚 45 毫米，占图上 105～217 毫米。
    ('夹爪连杆', (-.053, 0., -.0975), (.053, 0., -.0975), .0225),
    ('夹爪连杆', (-.053, 0., -.0625), (.053, 0., -.0625), .0225),
    ('夹爪连杆', (-.053, 0., -.0275), (.053, 0., -.0275), .0225),
    # 闭合的指尖：指垫宽 23 毫米，末端球面正好到夹爪参考点。
    ('夹指', (0., 0., -.0275), (0., 0., -.0115), .0115),
)
# 相邻路点之间按关节角直线取样的间隔，单位度。
PATH_STEP_DEG = 2.


@dataclasses.dataclass(frozen=True)
class Environment:
    """一台臂周围要避开的东西，坐标都在这台臂的基座坐标系里。

    Attributes:
        table_z_m: 桌面高度；None 表示不检查桌面。
        pole_xy_m: 立杆中心的水平位置；None 表示没有立杆。
        pole_radius_m: 立杆截面的外接圆半径（方杆取半对角线）。
        pole_margin_m: 手臂各部分与立杆表面之间至少要留的距离。
    """

    table_z_m: Optional[float] = None
    pole_xy_m: Optional[Tuple[float, float]] = None
    pole_radius_m: float = 0.
    pole_margin_m: float = 0.


def load_environment(
    root: pathlib.Path, arm_id: str, table_z_m: Optional[float]
) -> Environment:
    """读取一台臂的障碍物设置。

    Args:
        root: 项目根目录。
        arm_id: 臂编号。
        table_z_m: 这台臂抓取配置里的桌面高度。

    Returns:
        障碍物设置。本机没有障碍物文件，或文件里没有这台臂的立杆位置
        时，只检查桌面。

    Raises:
        ValueError: 障碍物文件的版本不认识，或数值缺失、无效。
    """
    path = pathlib.Path(root) / OBSTACLES_FILE
    if not path.is_file():
        return Environment(table_z_m=table_z_m)
    value = yaml.safe_load(path.read_text())
    if not isinstance(value, dict) or value.get('schema_version') != (
        SCHEMA_VERSION
    ):
        raise ValueError(f'障碍物文件 {path} 的格式版本不受支持')
    pole = value.get('pole')
    position = (pole or {}).get('position_m', {}).get(arm_id)
    if position is None:
        return Environment(table_z_m=table_z_m)
    numbers = np.array(
        [*position, pole.get('side_m'), pole.get('margin_m')], dtype=float
    )
    if numbers.shape != (4,) or not np.isfinite(numbers).all() or (
        numbers[2] <= 0 or numbers[3] < 0
    ):
        raise ValueError(f'障碍物文件 {path} 里立杆的数值缺失或无效')
    return Environment(
        table_z_m=table_z_m,
        pole_xy_m=(float(numbers[0]), float(numbers[1])),
        # 方杆的外接圆半径是边长的 √2/2。
        pole_radius_m=float(numbers[2]) * np.sqrt(.5),
        pole_margin_m=float(numbers[3]),
    )


def link_frames(joint_deg: Sequence[float]) -> List[np.ndarray]:
    """计算底座和 7 个连杆的坐标系在基座坐标系里的位姿。

    Args:
        joint_deg: 七个关节角，单位度。

    Returns:
        8 个 4×4 齐次变换矩阵：第 0 个是底座，第 i 个随第 i 关节转动，
        第 7 个就是法兰。
    """
    transform = np.eye(4)
    frames = [transform.copy()]
    for (alpha_deg, a_m, d_m), theta_deg in zip(DH_PARAMETERS, joint_deg):
        alpha, theta = np.radians(alpha_deg), np.radians(theta_deg)
        cos_a, sin_a = np.cos(alpha), np.sin(alpha)
        cos_t, sin_t = np.cos(theta), np.sin(theta)
        transform = transform @ np.array(
            [
                [cos_t, -sin_t, 0., a_m],
                [sin_t * cos_a, cos_t * cos_a, -sin_a, -sin_a * d_m],
                [sin_t * sin_a, cos_t * sin_a, cos_a, cos_a * d_m],
                [0., 0., 0., 1.],
            ]
        )
        frames.append(transform.copy())
    return frames


def capsules_in_base(
    joint_deg: Sequence[float], flange_T_tool: np.ndarray
) -> Tuple[np.ndarray, np.ndarray, np.ndarray, List[str], List[bool]]:
    """把机械臂和夹爪的全部胶囊体换到基座坐标系。

    Args:
        joint_deg: 七个关节角，单位度。
        flange_T_tool: 夹爪参考点在法兰坐标系里的位姿（工具记录里的
            `flange_T_grip_tcp`）。

    Returns:
        `(starts, ends, radii, labels, table_checked)`：线段起点和终点
        （各 N×3，米）、半径（N，米）、每个胶囊体的名称，以及它是否
        参与桌面检查。
    """
    frames = link_frames(joint_deg)
    starts, ends, radii, labels, table_checked = [], [], [], [], []
    for link, start, end, radius in ARM_CAPSULES:
        frame = frames[link]
        starts.append(frame[:3, :3] @ start + frame[:3, 3])
        ends.append(frame[:3, :3] @ end + frame[:3, 3])
        radii.append(radius)
        labels.append(ARM_LABELS[link])
        table_checked.append(link not in TABLE_EXEMPT_LINKS)
    tool = frames[7] @ np.asarray(flange_T_tool, dtype=float)
    for label, start, end, radius in GRIPPER_CAPSULES:
        starts.append(tool[:3, :3] @ start + tool[:3, 3])
        ends.append(tool[:3, :3] @ end + tool[:3, 3])
        radii.append(radius)
        labels.append(label)
        table_checked.append(True)
    return (
        np.array(starts), np.array(ends), np.array(radii), labels,
        table_checked,
    )


def horizontal_distance_to_point(
    starts: np.ndarray, ends: np.ndarray, point_xy: Sequence[float]
) -> np.ndarray:
    """计算若干线段到一条竖直线的距离。

    竖直线在水平面上是一个点，所以只需算线段的水平投影到这个点的距离。

    Args:
        starts: 线段起点，N×3，米。
        ends: 线段终点，N×3，米。
        point_xy: 竖直线的水平位置，米。

    Returns:
        每条线段到竖直线的最小距离，长度 N，米。
    """
    a = starts[:, :2]
    direction = ends[:, :2] - a
    offset = np.asarray(point_xy, dtype=float) - a
    length_squared = np.einsum('ij,ij->i', direction, direction)
    fraction = np.clip(
        np.einsum('ij,ij->i', offset, direction)
        / np.where(length_squared > 0, length_squared, 1.),
        0., 1.,
    )
    return np.linalg.norm(offset - fraction[:, None] * direction, axis=1)


def check_configuration(
    joint_deg: Sequence[float],
    flange_T_tool: np.ndarray,
    environment: Environment,
) -> dict:
    """检查一个姿态下机械臂和夹爪离桌面、立杆有多近。

    Args:
        joint_deg: 七个关节角，单位度。
        flange_T_tool: 夹爪参考点在法兰坐标系里的位姿。
        environment: 这台臂的障碍物设置。

    Returns:
        含 `table_clearance_m` 与 `table_part`（离桌面最近的部分及其
        间距，负值表示低于桌面）、`pole_clearance_m` 与 `pole_part`
        （离立杆表面最近的部分及其间距）；不检查的一项为 None。
        `violation` 是给操作者看的违规说明，没有违规时为 None。
    """
    starts, ends, radii, labels, table_checked = capsules_in_base(
        joint_deg, flange_T_tool
    )
    result = {
        'table_clearance_m': None,
        'table_part': None,
        'pole_clearance_m': None,
        'pole_part': None,
        'violation': None,
    }
    if environment.pole_xy_m is not None:
        clearance = (
            horizontal_distance_to_point(starts, ends, environment.pole_xy_m)
            - radii
            - environment.pole_radius_m
        )
        index = int(np.argmin(clearance))
        result.update(
            pole_clearance_m=float(clearance[index]), pole_part=labels[index]
        )
        if clearance[index] < environment.pole_margin_m:
            result['violation'] = (
                f'{labels[index]}离立杆 {clearance[index] * 1000:.0f} 毫米，'
                f'要求至少 {environment.pole_margin_m * 1000:.0f} 毫米'
            )
    if environment.table_z_m is not None:
        lowest = np.minimum(starts[:, 2], ends[:, 2]) - radii
        clearance = np.where(
            table_checked, lowest - environment.table_z_m, np.inf
        )
        index = int(np.argmin(clearance))
        result.update(
            table_clearance_m=float(clearance[index]),
            table_part=labels[index],
        )
        # 留 0.5 毫米的计算容差：夹指末端的球面本来就贴着夹爪参考点。
        if clearance[index] < -.0005 and result['violation'] is None:
            result['violation'] = (
                f'{labels[index]}会碰到桌面'
                f'（低于桌面 {-clearance[index] * 1000:.0f} 毫米）'
            )
    return result


def check_path(
    joint_path_deg: Sequence[Sequence[float]],
    flange_T_tool: np.ndarray,
    environment: Environment,
    step_deg: float = PATH_STEP_DEG,
) -> dict:
    """沿一串关节角路点检查全程离桌面、立杆有多近。

    相邻路点之间按关节角直线取样（关节运动 `movej` 各关节同步转动，
    走的就是这条直线）。第一个路点是机械臂当前所在的位置，不是计划的
    一部分，所以不检查它本身。

    Args:
        joint_path_deg: 路点序列，每个是七个关节角（度）；第一个是
            当前关节角。
        flange_T_tool: 夹爪参考点在法兰坐标系里的位姿。
        environment: 这台臂的障碍物设置。
        step_deg: 取样间隔：任一关节每转这么多度取一个点。

    Returns:
        含 `checked_samples`（检查了多少个姿态）、全程的
        `min_table_clearance_m`、`min_pole_clearance_m` 及各自最近的
        部分，以及 `violation`：第一处违规的说明、所在的路点序号
        `waypoint_index`（从 1 起，指正在走向的那个路点）和该处的
        关节角；没有违规时为 None。
    """
    path = np.asarray(joint_path_deg, dtype=float)
    if path.ndim != 2 or path.shape[1] != 7 or len(path) < 2:
        raise ValueError('路径至少需要当前关节角和一个路点，每个 7 个关节角')
    report = {
        'checked_samples': 0,
        'min_table_clearance_m': None,
        'min_table_part': None,
        'min_pole_clearance_m': None,
        'min_pole_part': None,
        'violation': None,
    }
    for index in range(1, len(path)):
        previous, target = path[index - 1], path[index]
        steps = max(
            1, int(np.ceil(np.abs(target - previous).max() / step_deg))
        )
        for step in range(1, steps + 1):
            joint = previous + (target - previous) * step / steps
            state = check_configuration(joint, flange_T_tool, environment)
            report['checked_samples'] += 1
            for kind in ('table', 'pole'):
                clearance = state[f'{kind}_clearance_m']
                best = report[f'min_{kind}_clearance_m']
                if clearance is not None and (
                    best is None or clearance < best
                ):
                    report[f'min_{kind}_clearance_m'] = clearance
                    report[f'min_{kind}_part'] = state[f'{kind}_part']
            if state['violation'] and report['violation'] is None:
                report['violation'] = {
                    'message': state['violation'],
                    'waypoint_index': index,
                    'joint_deg': joint.tolist(),
                }
    return report
