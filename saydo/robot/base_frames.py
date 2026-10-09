"""两台机械臂底座之间的相对位置。

每台臂报告的位置都以自己的底座为原点。要让两台臂在同一个任务里配合
（一台看、另一台抓，或者判断两臂会不会相撞），必须知道一台臂的底座在
另一台臂的坐标里处于什么位置和朝向。

这个关系是实测的（2026-10-09：两台臂的指尖对准同一块棋盘上的交叉点，
并与两台相机各自看到的棋盘相互印证），保存在各机器本地的
`config/calibration/base_relation.yaml`。底座没有被挪动，它就不会变。

本模块只负责读取和换算，不连接控制器。
"""

from __future__ import annotations

import pathlib
from typing import Optional

import numpy as np
import yaml

RELATION_FILE = 'config/calibration/base_relation.yaml'
SCHEMA_VERSION = 1
# 两底座实际相距约 0.52 米。读到的距离超出这个范围，说明文件写错了。
MAX_BASE_DISTANCE_M = 2.


def load(root: pathlib.Path) -> Optional[dict]:
    """读取两底座的相对位置。

    Args:
        root: 项目根目录。

    Returns:
        记录，其中 `arm1_base_T_arm2_base` 是 2 臂底座在 1 臂底座坐标里
        的位姿（4×4，平移单位米）；文件不存在时为 None。

    Raises:
        ValueError: 文件版本不认识，或矩阵不是有效的刚体变换、距离
            不合理。
    """
    path = pathlib.Path(root) / RELATION_FILE
    if not path.is_file():
        return None
    value = yaml.safe_load(path.read_text())
    if not isinstance(value, dict) or value.get('schema_version') != (
        SCHEMA_VERSION
    ):
        raise ValueError(f'底座关系文件 {path} 的格式版本不受支持')
    matrix = np.asarray(value.get('arm1_base_T_arm2_base'), dtype=float)
    if (
        matrix.shape != (4, 4)
        or not np.isfinite(matrix).all()
        or not np.allclose(matrix[3], [0, 0, 0, 1])
        or not np.allclose(
            matrix[:3, :3].T @ matrix[:3, :3], np.eye(3), atol=1e-6
        )
        or not np.isclose(np.linalg.det(matrix[:3, :3]), 1, atol=1e-6)
    ):
        raise ValueError(f'底座关系文件 {path} 里的矩阵不是有效的刚体变换')
    distance_m = float(np.linalg.norm(matrix[:3, 3]))
    if distance_m > MAX_BASE_DISTANCE_M:
        raise ValueError(
            f'底座关系文件 {path} 给出的两底座距离 {distance_m:.2f} 米'
            '不合理'
        )
    return value


def target_base_T_source_base(
    relation: dict, target_arm_id: str, source_arm_id: str
) -> np.ndarray:
    """返回把 source 臂底座坐标里的位姿换到 target 臂底座坐标的变换。

    Args:
        relation: `load` 返回的记录。
        target_arm_id: 结果所在坐标系对应的臂，`arm1` 或 `arm2`。
        source_arm_id: 输入所在坐标系对应的臂，`arm1` 或 `arm2`。

    Returns:
        4×4 齐次变换矩阵，平移单位米；两个编号相同时是单位矩阵。

    Raises:
        ValueError: 编号不是 `arm1` 或 `arm2`。
    """
    known = ('arm1', 'arm2')
    if target_arm_id not in known or source_arm_id not in known:
        raise ValueError(
            f'底座关系只登记了 arm1 与 arm2，收到 {target_arm_id!r} 和 '
            f'{source_arm_id!r}'
        )
    if target_arm_id == source_arm_id:
        return np.eye(4)
    arm1_base_T_arm2_base = np.asarray(
        relation['arm1_base_T_arm2_base'], dtype=float
    )
    if target_arm_id == 'arm1':
        return arm1_base_T_arm2_base
    return np.linalg.inv(arm1_base_T_arm2_base)
