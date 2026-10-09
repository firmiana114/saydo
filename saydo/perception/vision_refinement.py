"""相机校正：把手眼标定给出的相机位姿修正到与触碰实测一致。

为什么需要它：棋盘格手眼标定对“相机沿自己视线方向的位置”定不准，
实测比真实位置靠后约 1 厘米多，朝向也有约 1 度的偏差；个别相机的深度
读数还有随距离增大的偏差。这些误差让相机看到的物品比实际高出几毫米到
一厘米多。

校正量不是标定出来的，而是用两类触碰实测反推的：指尖触桌得到真实的
桌面高度，指尖对准棋盘交叉点得到真实的水平位置。每台臂一份，保存在
各机器本地的 `config/calibration/<臂编号>_vision_refinement.yaml`。

校正只在测量它的那个姿态（观测姿态）附近准确：机械臂在不同姿态之间的
绝对定位本身有几毫米误差，相机校正无法消除。因此场景快照应当在观测
姿态下采集。

本模块只做数值换算，不读相机，不连接控制器。
"""

from __future__ import annotations

import pathlib
from typing import Optional

import numpy as np
import yaml

SCHEMA_VERSION = 1
# 校正量的合理上限。超过它说明文件写错了或者安装已经变动，应当重新测量
# 而不是照用：平移 5 厘米、转角 5 度、深度二次项相当于 1 米处偏 10 厘米。
MAX_TRANSLATION_M = .05
MAX_ROTATION_RAD = np.radians(5.)
MAX_DEPTH_QUADRATIC_PER_M = .1


def rotation_matrix(rotation_vector_rad: np.ndarray) -> np.ndarray:
    """把旋转向量换成 3×3 旋转矩阵（罗德里格斯公式）。

    Args:
        rotation_vector_rad: 旋转向量，方向是转轴，长度是转角，单位弧度。

    Returns:
        3×3 旋转矩阵。
    """
    vector = np.asarray(rotation_vector_rad, dtype=float)
    angle_rad = float(np.linalg.norm(vector))
    if angle_rad < 1e-12:
        return np.eye(3)
    x, y, z = vector / angle_rad
    skew = np.array([[0, -z, y], [z, 0, -x], [-y, x, 0]])
    return (
        np.eye(3)
        + np.sin(angle_rad) * skew
        + (1 - np.cos(angle_rad)) * skew @ skew
    )


def load(path: pathlib.Path, arm_id: str) -> dict:
    """读取并检查一台臂的相机校正文件。

    Args:
        path: 校正文件。
        arm_id: 使用这份校正的臂编号；文件必须是为这台臂测的。

    Returns:
        校正记录，其中 `correction` 含 `rotation_vector_rad`、
        `translation_m`，另有 `depth_quadratic_per_m`。

    Raises:
        ValueError: 文件是别的臂的、版本不认识、未被用户接受，或数值
            缺失、非有限、超出合理范围。
    """
    value = yaml.safe_load(pathlib.Path(path).read_text())
    if not isinstance(value, dict) or value.get('schema_version') != (
        SCHEMA_VERSION
    ):
        raise ValueError(f'相机校正文件 {path} 的格式版本不受支持')
    if value.get('arm') != arm_id:
        raise ValueError(
            f"相机校正文件 {path} 是为 {value.get('arm')} 测的，"
            f'不能用于 {arm_id}'
        )
    if value.get('acceptance_status') != 'user_accepted_usable':
        raise ValueError(f'相机校正文件 {path} 尚未被接受使用')
    correction = value.get('correction') or {}
    rotation = np.asarray(correction.get('rotation_vector_rad'), dtype=float)
    translation = np.asarray(correction.get('translation_m'), dtype=float)
    quadratic = value.get('depth_quadratic_per_m')
    if (
        rotation.shape != (3,)
        or translation.shape != (3,)
        or type(quadratic) not in (int, float)
        or not np.isfinite(np.r_[rotation, translation, quadratic]).all()
    ):
        raise ValueError(f'相机校正文件 {path} 的数值缺失或无效')
    if (
        np.linalg.norm(translation) > MAX_TRANSLATION_M
        or np.linalg.norm(rotation) > MAX_ROTATION_RAD
        or abs(quadratic) > MAX_DEPTH_QUADRATIC_PER_M
    ):
        raise ValueError(
            f'相机校正文件 {path} 的校正量超出合理范围（平移 '
            f'{np.linalg.norm(translation) * 1000:.1f} 毫米，转角 '
            f'{np.degrees(np.linalg.norm(rotation)):.2f} 度，深度二次项 '
            f'{quadratic:.4f} 每米）；请重新测量'
        )
    return value


def camera_T_measured(refinement: dict) -> np.ndarray:
    """返回把“相机量到的点”换成“校正后的点”的变换。

    两者都用相机光学坐标系表示。约定：校正后的点等于先加平移、再
    旋转，即 `R @ (量到的点 + 平移)`。

    Args:
        refinement: `load` 返回的校正记录。

    Returns:
        4×4 齐次变换矩阵，平移单位米。
    """
    correction = refinement['correction']
    rotation = rotation_matrix(correction['rotation_vector_rad'])
    transform = np.eye(4)
    transform[:3, :3] = rotation
    transform[:3, 3] = rotation @ np.asarray(
        correction['translation_m'], dtype=float
    )
    return transform


def refined_flange_T_camera(
    flange_T_camera: np.ndarray, refinement: Optional[dict]
) -> np.ndarray:
    """把相机校正合进手眼矩阵。

    Args:
        flange_T_camera: 手眼标定给出的相机在法兰坐标系里的位姿。
        refinement: 校正记录；None 表示这台臂没有校正，原样返回。

    Returns:
        校正后的相机位姿：`flange_T_camera @ camera_T_measured`。
    """
    matrix = np.asarray(flange_T_camera, dtype=float)
    if refinement is None:
        return matrix
    return matrix @ camera_T_measured(refinement)


def corrected_depth(depth_m: np.ndarray, quadratic_per_m: float) -> np.ndarray:
    """修正深度图里随距离平方增大的读数偏差。

    双目深度相机的视差若有固定偏差，深度误差就与距离的平方成正比：
    真实深度约为 `Z + k·Z²`。没有深度的像素（值为 0）保持为 0。

    Args:
        depth_m: 深度图，单位米。
        quadratic_per_m: 系数 k，单位每米；负值表示相机读得偏远。

    Returns:
        修正后的深度图，单位米，数据类型与输入相同。
    """
    if not quadratic_per_m:
        return depth_m
    depth = np.asarray(depth_m)
    return (depth * (1 + quadratic_per_m * depth)).astype(depth.dtype)
