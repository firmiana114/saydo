"""判断物品是否有一部分落在相机画面之外，落在外面太多就拒绝定位结果。

为什么需要它：位姿估计是拿模型去对物品在画面里露出的那部分。物品被画面
边缘裁掉一截时，估计照样会给出一个各项质量指标都通过的结果，但被裁掉的
那一段对结果没有任何约束。对试管这类两头不一样的细长物品，分辨“哪头是
口、哪头是底”靠的正是两端的形状，一端出画就可能认反。

2026-10-09 15:35 的 1 臂抓放就是这样出的事：试管下端有约 5% 的表面在
画面之外，定位把口和底认反，后续规划选了手肘甩向立杆的翻转方式，把立杆
碰倒了。当时的质量检查（深度残差、轮廓重合度）全部通过。

做法：把模型表面上的取样点按估计出的位姿投影到画面上，按面积统计落在
画面外的比例。它只用估计出的位姿做几何计算，不看图像内容。

局限：如果位姿估计为了贴合露出的部分，把模型整体“滑”进了画面里，本检查
发现不了。翻查到 2026-10-09 为止的 100 多次场景记录，没有见到这种情况
（被裁掉的物品，估计出的位姿都如实伸出画面）。
"""

from __future__ import annotations

from typing import Dict, List, TypedDict

import numpy as np

# 画面外表面积占比的上限。取值依据：统计 MSI 上 2026-09-28 至 10-09 的
# 全部场景记录（约 100 次采集），这个比例要么不超过 0.9%（物品完整入画，
# 只有一个角被画面切到），要么不低于 3.2%（确实被裁掉一截），中间没有
# 数据；出事那次试管是 5.3%。1% 落在空档里并靠近“完整”一侧：误拒的代价
# 只是把物品挪一下重新采集，误放的代价是口底认反。
MAX_OUTSIDE_FRAME_PERCENT = 1.

# 按画面的四条边分别统计，报错时告诉操作者物品是从哪一边出去的。
_EDGE_LABELS = {
    'left': '左边',
    'top': '上边',
    'right': '右边',
    'bottom': '下边',
}


class FrameCoverage(TypedDict):
    """物品相对画面范围的检查结果，随场景记录一起保存。

    Attributes:
        good: 画面外的占比是否不超过上限。
        outside_percent: 落在画面外的模型表面积占比，单位百分比。
        outside_percent_by_edge: 越过各条边的表面积占比，单位百分比；键为
            `left`、`top`、`right`、`bottom`，只列出大于 0 的边。越过画面
            角的部分会同时记在两条边上，所以各边之和可能大于总占比。
        limit_percent: 判定时使用的上限，单位百分比。
    """

    good: bool
    outside_percent: float
    outside_percent_by_edge: Dict[str, float]
    limit_percent: float


def measure(
    camera_T_object: np.ndarray,
    surface_points_object_m: np.ndarray,
    point_weights: np.ndarray,
    camera_matrix: np.ndarray,
    width_px: int,
    height_px: int,
    limit_percent: float = MAX_OUTSIDE_FRAME_PERCENT,
) -> FrameCoverage:
    """统计模型表面有多少比例投影到了画面范围之外。

    Args:
        camera_T_object: 估计出的物品位姿，4×4，相机光学坐标系。
        surface_points_object_m: 模型表面的取样点，N×3，物品坐标系，单位米。
            调用方一般传三角面的中心。
        point_weights: 每个取样点代表的表面积，长度 N，只用到相对大小。
            调用方一般传三角面的面积，这样结果与模型网格的疏密无关。
        camera_matrix: 相机内参矩阵，3×3，单位像素。
        width_px: 画面宽度，单位像素。
        height_px: 画面高度，单位像素。
        limit_percent: 画面外占比的上限，单位百分比。

    Returns:
        检查结果，见 `FrameCoverage`。落在相机背后的点一律算作画面外。

    Raises:
        ValueError: 取样点与权重的形状不匹配，或权重之和不是正数。
    """
    points_object_m = np.asarray(surface_points_object_m, dtype=float)
    weights = np.asarray(point_weights, dtype=float)
    if (
        points_object_m.ndim != 2
        or points_object_m.shape[1] != 3
        or weights.shape != (len(points_object_m),)
    ):
        raise ValueError(
            f'模型表面取样点形状 {points_object_m.shape} 与权重形状 '
            f'{weights.shape} 不匹配'
        )
    total_weight = float(weights.sum())
    if not np.isfinite(total_weight) or total_weight <= 0:
        raise ValueError(f'模型表面取样点的权重之和无效：{total_weight}')
    pose = np.asarray(camera_T_object, dtype=float)
    points_camera_m = points_object_m @ pose[:3, :3].T + pose[:3, 3]
    in_front = points_camera_m[:, 2] > 1e-6
    # 相机背后的点没有有意义的投影，深度先换成 1 以免除零，随后整体记为
    # 画面外。
    depth_m = np.where(in_front, points_camera_m[:, 2], 1.)
    projected = points_camera_m @ np.asarray(camera_matrix, dtype=float).T
    column_px = projected[:, 0] / depth_m
    row_px = projected[:, 1] / depth_m
    beyond_edge = {
        'left': in_front & (column_px < 0),
        'top': in_front & (row_px < 0),
        'right': in_front & (column_px >= width_px),
        'bottom': in_front & (row_px >= height_px),
    }
    outside = ~in_front
    for beyond in beyond_edge.values():
        outside = outside | beyond

    def percent(selected: np.ndarray) -> float:
        return float(100. * weights[selected].sum() / total_weight)

    outside_percent = percent(outside)
    return {
        'good': bool(outside_percent <= limit_percent),
        'outside_percent': outside_percent,
        'outside_percent_by_edge': {
            edge: percent(beyond)
            for edge, beyond in beyond_edge.items()
            if beyond.any()
        },
        'limit_percent': float(limit_percent),
    }


def require_inside(name: str, coverage: FrameCoverage) -> None:
    """物品出画超过上限时报错，报错写明出画比例和是从哪条边出去的。

    Args:
        name: 报错里用来称呼这个物品的名字，例如场景里的实例名。
        coverage: `measure` 的返回值。

    Raises:
        ValueError: 画面外的占比超过上限。
    """
    if coverage['good']:
        return
    edges: List[str] = [
        f'{_EDGE_LABELS[edge]} {percent:.1f}%'
        for edge, percent in coverage['outside_percent_by_edge'].items()
    ]
    where = f'（{"、".join(edges)}）' if edges else ''
    raise ValueError(
        f'{name} 有 {coverage["outside_percent"]:.1f}% 的表面在画面之外'
        f'{where}，上限 {coverage["limit_percent"]:g}%。出画的部分对定位'
        '没有约束，位置和方向都可能认错，请把物品完整放进画面后重新识别'
    )
