"""测试用：按给定的相机位姿合成一幅“相机看到棋盘格标定板”的画面。

真实的角点检测要在像素图上运行。这里用针孔模型把标定板的图案投影成
画面，代替真实相机这一处系统边界；检测、位姿求解和标定流程都是真实
代码。合成的画面没有镜头畸变。
"""

from __future__ import annotations

import cv2
import numpy as np

# 图案的分辨率：每米多少像素。30 毫米的方格是 90 像素。
TEXTURE_PX_PER_M = 3000.
BACKGROUND = 96


def board_texture(columns: int, rows: int, square_m: float) -> np.ndarray:
    """生成标定板的图案：内角点 `columns × rows`，四周留一格白边。

    Args:
        columns: 每行的内角点数。
        rows: 内角点的行数。
        square_m: 方格边长，单位米。

    Returns:
        灰度图。第一个内角点在离左上角两个方格处。
    """
    size_px = int(round(square_m * TEXTURE_PX_PER_M))
    texture = np.full(
        ((rows + 3) * size_px, (columns + 3) * size_px), 255, np.uint8
    )
    for row in range(rows + 1):
        for column in range(columns + 1):
            if (row + column) % 2 == 0:
                top, left = (row + 1) * size_px, (column + 1) * size_px
                texture[top:top + size_px, left:left + size_px] = 0
    return texture


def render(
    camera_matrix: np.ndarray,
    width_px: int,
    height_px: int,
    camera_T_board: np.ndarray,
    columns: int = 8,
    rows: int = 11,
    square_m: float = .03,
) -> np.ndarray:
    """合成相机看到的画面。

    Args:
        camera_matrix: 相机内参矩阵。
        width_px: 画面宽度。
        height_px: 画面高度。
        camera_T_board: 标定板在相机坐标系里的位姿；标定板坐标系的原点
            在第一个内角点，X 沿一行、Y 沿一列。
        columns: 每行的内角点数。
        rows: 内角点的行数。
        square_m: 方格边长，单位米。

    Returns:
        彩色图（蓝绿红），标定板以外是均匀的灰色。
    """
    texture = board_texture(columns, rows, square_m)
    # 图案像素 → 标定板坐标（米）：第一个内角点在图案的 (2 格, 2 格) 处。
    texture_to_board = np.array(
        [
            [1 / TEXTURE_PX_PER_M, 0., -2 * square_m],
            [0., 1 / TEXTURE_PX_PER_M, -2 * square_m],
            [0., 0., 1.],
        ]
    )
    # 标定板平面上的点 (x, y, 0) → 画面像素。
    board_to_image = camera_matrix @ camera_T_board[:3, [0, 1, 3]]
    image = cv2.warpPerspective(
        texture,
        board_to_image @ texture_to_board,
        (width_px, height_px),
        flags=cv2.INTER_AREA,
        borderMode=cv2.BORDER_CONSTANT,
        borderValue=BACKGROUND,
    )
    return cv2.cvtColor(image, cv2.COLOR_GRAY2BGR)


def looking_at(
    position_m: np.ndarray, target_m: np.ndarray, up: np.ndarray
) -> np.ndarray:
    """构造一个位于某处、光轴指向目标点的相机位姿。

    Args:
        position_m: 相机位置。
        target_m: 光轴指向的点。
        up: 画面“上方”大致对应的空间方向。

    Returns:
        `世界_T_相机`，4×4；相机坐标系的 Z 沿光轴，Y 指向画面下方。
    """
    forward = np.asarray(target_m, float) - np.asarray(position_m, float)
    forward /= np.linalg.norm(forward)
    right = np.cross(forward, np.asarray(up, float))
    right /= np.linalg.norm(right)
    down = np.cross(forward, right)
    pose = np.eye(4)
    pose[:3, :3] = np.column_stack([right, down, forward])
    pose[:3, 3] = position_m
    return pose
