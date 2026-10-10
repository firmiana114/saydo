"""全局相机的快速外参标定：借已经标好的手上相机，把位置传给它。

全局相机是不装在机械臂上的那台（D405，装在桌边的立杆上）。它会被取下
再装回，每次位置都可能略有不同，所以需要一个一两分钟就能做完的标定。

原理：把棋盘格标定板平放在桌上，让全局相机和手上相机同时看到它。手上
相机已经标好（手眼标定加相机校正），由它可以算出标定板每个角点在基座
坐标系里的位置；全局相机看到的是同一批角点在自己画面里的像素位置。
“空间位置 ↔ 像素位置”的对应关系足够多时，就能反推出全局相机相对基座
的位置和朝向。整个过程机械臂不动，只需要人把标定板在桌上挪几个位置。

脚本每秒取一次三台相机的画面，逐项检查后自动记录合格的一组，凑够
数量和覆盖面后自动求解。各项检查见 `screen_view` 和 `Session.step`。

使用前提：机械臂停在观测姿态。手上相机的校正只在那个姿态下准到 1～2
毫米，标定板的位置是靠它算出来的。

相机画面都从服务那里取，本程序不直接打开任何相机：全局相机经 8088
相机服务，手上相机经 8089。所以采集期间 8088 页面上三路预览照常，
操作者可以看着画面摆标定板。

用法（在项目根目录，MSI 上）：

    .venv/bin/python -m saydo.calibration.global_camera            标定
    .venv/bin/python -m saydo.calibration.global_camera --verify   核对

核对只拍一组，报告现有标定与当前实际相差多少，不改任何文件。

本模块不向机械臂或夹爪发送任何指令。
"""

from __future__ import annotations

import argparse
import base64
import datetime
import hashlib
import json
import logging
import os
import pathlib
import sys
import time
import urllib.error
import urllib.parse
import urllib.request
from typing import Any, Callable, Dict, List, Optional, Tuple

import cv2
import numpy as np
import yaml

from saydo.calibration import collect
from saydo.calibration import geometry
from saydo.geometry import grasp_geometry
from saydo.perception import vision_refinement
from saydo.robot import arms
from saydo.robot import base_frames
from saydo.robot import collision_model

LOG = logging.getLogger(__name__)

RESULT_FILE = 'config/calibration/global_camera_extrinsic.yaml'
CAMERA_CONFIG_FILE = 'config/cameras/global.json'
SCHEMA_VERSION = 1
# 结果以哪台臂的基座为准；另一台臂经两底座的相对位置换算。
REFERENCE_ARM_ID = 'arm1'

# 标定板：内角点 8 列 × 11 行，方格边长 30 毫米。方格数是 9 × 12，一奇
# 一偶，转 180 度后黑白相反，所以角点顺序不会认反。
BOARD_COLUMNS = 8
BOARD_ROWS = 11
SQUARE_M = .03

# 以下是一组画面合格的条件。
# 角点离画面边缘至少这么远：边缘处镜头畸变大，角点也容易被裁掉半个。
EDGE_MARGIN_FRACTION = .01
# 方格在画面里至少这么大（像素），太小时角点定不准。全局相机在 1 米外
# 斜着看时，方格较短的一边只有 9 像素上下，所以门限不能再高。
MIN_SQUARE_PX = 6.
# 视线与标定板法线的夹角上限（度），太斜时角点定位和位姿都不稳。
MAX_TILT_DEG = 72.
# 角点反推回画面的均方根误差上限（像素）。
MAX_REPROJECTION_PX = .8
# 平面目标有两个可能的位姿（朝两边倾斜）。两个解的朝向相差超过下面的
# 角度时，次优解的误差至少要是最优解的这么多倍，否则这一帧分不清标定板
# 朝哪边倾斜。正对着看时两个解几乎重合，不存在分不清的问题。
MIN_POSE_AMBIGUITY_RATIO = 1.5
POSE_AMBIGUITY_ANGLE_DEG = 4.
# 与上一轮相比，角点移动超过这么多像素就认为标定板还在动。三台相机不是
# 同一瞬间曝光的，标定板在动会直接变成误差。
STILL_PX = .7
# 与已记录的各组相比，标定板至少挪开这么远或转过这么大角度才算新的一组。
NEW_PLACEMENT_SHIFT_M = .04
NEW_PLACEMENT_TURN_DEG = 12.
# 两台臂同时看到标定板时，两者算出的角点位置相差的上限（均方根，米）。
ARM_AGREEMENT_M = .006
# 机械臂各关节与观测姿态相差的上限（度），与场景快照的判定一致。
OBSERVATION_TOLERANCE_DEG = 2.

# 以下是采够与验收的条件。
DEFAULT_SAMPLES = 8
# 各组标定板中心在两个水平方向上至少铺开这么宽（米）。全挤在一处时，
# 求出的朝向不准，离这一处越远误差越大。
MIN_SPREAD_M = .12
# 验收：全局相机画面里的均方根误差上限（像素），以及“手上相机算出的角点
# 位置”与“全局相机按结果算出的角点位置”之间的均方根差上限（米）。
ACCEPT_REPROJECTION_PX = 1.5
ACCEPT_DISAGREEMENT_M = .004
# 一组的偏差超过中位数这么多倍时剔除后重算。
OUTLIER_FACTOR = 3.

DEFAULT_BACKEND_URL = 'http://192.168.101.65:8089'
DEFAULT_TOKEN_FILE = '~/.local/share/saydo-mcp/robot-backend-token'
LOOP_PERIOD_S = 1.
# 每次取图让相机服务再保持采集多久，单位秒。比每轮的间隔长得多，
# 所以采集中途不会断；程序退出后相机最多再开这么久。
CAMERA_KEEP_S = 20.


def board_points() -> np.ndarray:
    """返回标定板各内角点在标定板坐标系里的位置，N×3，单位米。"""
    return geometry.object_points(BOARD_COLUMNS, BOARD_ROWS, SQUARE_M)


def detect_corners(image_bgr: np.ndarray) -> Optional[np.ndarray]:
    """在一幅彩色图里找标定板的全部内角点。

    Args:
        image_bgr: 彩色图，蓝绿红顺序。

    Returns:
        角点的像素坐标，N×2；没有找到完整的标定板时为 None。
    """
    found, corners = cv2.findChessboardCornersSB(
        cv2.cvtColor(image_bgr, cv2.COLOR_BGR2GRAY),
        (BOARD_COLUMNS, BOARD_ROWS),
        cv2.CALIB_CB_NORMALIZE_IMAGE,
    )
    if not found:
        return None
    return corners.reshape(-1, 2).astype(np.float64)


def screen_view(
    corners_px: np.ndarray,
    width_px: int,
    height_px: int,
    camera_matrix: np.ndarray,
    distortion: np.ndarray,
) -> Tuple[Optional[np.ndarray], str]:
    """检查一台相机看到的标定板是否合格，并算出标定板相对相机的位姿。

    依次检查：角点不贴画面边缘、方格成像够大、位姿拟合误差够小、平面
    位姿的两个候选解分得开、视线不过分倾斜。

    Args:
        corners_px: 角点的像素坐标，N×2。
        width_px: 画面宽度。
        height_px: 画面高度。
        camera_matrix: 相机内参矩阵，3×3。
        distortion: 镜头畸变参数。

    Returns:
        `(camera_T_board, 原因)`。合格时第一项是标定板在相机坐标系里的
        位姿（4×4），原因为空字符串；不合格时第一项为 None，原因是给
        操作者看的一句话。
    """
    margin = EDGE_MARGIN_FRACTION * width_px
    if (
        corners_px[:, 0].min() < margin
        or corners_px[:, 1].min() < margin
        or corners_px[:, 0].max() > width_px - margin
        or corners_px[:, 1].max() > height_px - margin
    ):
        return None, '标定板离画面边缘太近'
    grid = corners_px.reshape(BOARD_ROWS, BOARD_COLUMNS, 2)
    square_px = float(
        min(
            np.linalg.norm(np.diff(grid, axis=0), axis=2).min(),
            np.linalg.norm(np.diff(grid, axis=1), axis=2).min(),
        )
    )
    if square_px < MIN_SQUARE_PX:
        return None, f'标定板成像太小（方格 {square_px:.0f} 像素）'
    try:
        candidates = geometry.board_pose(
            corners_px, camera_matrix, distortion,
            BOARD_COLUMNS, BOARD_ROWS, SQUARE_M,
        )
    except ValueError:
        return None, '标定板位姿求解失败'
    best = candidates[0]
    if best['reprojection_rms_px'] > MAX_REPROJECTION_PX:
        return None, (
            f"角点拟合误差过大（{best['reprojection_rms_px']:.2f} 像素），"
            '标定板可能不平或画面模糊'
        )
    pose = np.array(best['camera_T_board'])
    if len(candidates) > 1:
        other = np.array(candidates[1]['camera_T_board'])
        apart_deg = np.degrees(
            np.arccos(np.clip(pose[:3, 2] @ other[:3, 2], -1., 1.))
        )
        if apart_deg > POSE_AMBIGUITY_ANGLE_DEG and (
            candidates[1]['reprojection_rms_px']
            < MIN_POSE_AMBIGUITY_RATIO * max(best['reprojection_rms_px'], .05)
        ):
            return None, '分不清标定板朝哪边倾斜，请换个位置'
    center = pose[:3, :3] @ board_points().mean(axis=0) + pose[:3, 3]
    tilt_deg = float(
        np.degrees(
            np.arccos(
                min(1., abs(pose[:3, 2] @ center) / np.linalg.norm(center))
            )
        )
    )
    if tilt_deg > MAX_TILT_DEG:
        return None, f'标定板相对视线太斜（{tilt_deg:.0f} 度）'
    return pose, ''


def transformed(transform: np.ndarray, points: np.ndarray) -> np.ndarray:
    """把一组点从一个坐标系换到另一个坐标系。

    Args:
        transform: 4×4 齐次变换矩阵。
        points: N×3。

    Returns:
        变换后的点，N×3。
    """
    return points @ transform[:3, :3].T + transform[:3, 3]


def hand_camera_pose(
    root: pathlib.Path,
    arm: arms.Arm,
    joint_deg: List[float],
    relation: Optional[dict],
) -> np.ndarray:
    """算出一台臂的手上相机此刻在参考基座坐标系里的位姿。

    由关节角算出法兰位姿，再乘上手眼标定与相机校正；不是参考臂时再经
    两底座的相对位置换算。

    Args:
        root: 项目根目录。
        arm: 哪台臂。
        joint_deg: 七个关节角，单位度。
        relation: `base_frames.load` 读出的两底座关系；只有参考臂时
            可以为 None。

    Returns:
        `参考基座_T_相机`，4×4。

    Raises:
        ValueError: 这台臂没有配置相机校正，或需要两底座关系而没有。
    """
    config = json.loads(arm.config_path(root).read_text())
    handeye = yaml.safe_load((root / config['handeye_file']).read_text())
    name = config.get('vision_refinement_file')
    if not name:
        raise ValueError(
            f'{arm.label}没有配置相机校正，手上相机不够准，不能用来标定'
        )
    flange_T_camera = vision_refinement.refined_flange_T_camera(
        handeye['flange_T_color_optical_camera'],
        vision_refinement.load(root / name, arm.arm_id),
    )
    base_T_camera = collision_model.link_frames(joint_deg)[7] @ flange_T_camera
    if arm.arm_id == REFERENCE_ARM_ID:
        return base_T_camera
    if relation is None:
        raise ValueError('缺少两底座的相对位置，不能使用' + arm.label)
    return (
        base_frames.target_base_T_source_base(
            relation, REFERENCE_ARM_ID, arm.arm_id
        )
        @ base_T_camera
    )


def at_observation_pose(
    root: pathlib.Path, arm: arms.Arm, joint_deg: List[float]
) -> bool:
    """判断一台臂是否停在观测姿态（各关节相差不超过 2 度）。"""
    config = json.loads(arm.config_path(root).read_text())
    target_raw = grasp_geometry.return_joint_target(config)
    if target_raw is None:
        return False
    difference_deg = (
        np.abs(np.asarray(joint_deg) * 1000 - np.asarray(target_raw)) / 1000
    )
    return bool(difference_deg.max() <= OBSERVATION_TOLERANCE_DEG)


def placement_index(
    points_base_m: np.ndarray, recorded: List[dict]
) -> Optional[int]:
    """找出与当前标定板摆位重复的已记录的那一组。

    Args:
        points_base_m: 当前标定板各角点在基座坐标系里的位置，N×3。
        recorded: 已记录的各组。

    Returns:
        重复的那一组的序号（从 1 起）；与每一组都挪开了足够的距离或
        转过了足够的角度时为 None。
    """
    center = points_base_m.mean(axis=0)
    direction = points_base_m[BOARD_COLUMNS - 1] - points_base_m[0]
    for index, sample in enumerate(recorded, 1):
        other = np.asarray(sample['points_base_m'])
        other_direction = other[BOARD_COLUMNS - 1] - other[0]
        cosine = (direction @ other_direction) / (
            np.linalg.norm(direction) * np.linalg.norm(other_direction)
        )
        turn_deg = np.degrees(np.arccos(np.clip(cosine, -1., 1.)))
        if (
            np.linalg.norm(center - other.mean(axis=0))
            < NEW_PLACEMENT_SHIFT_M
            and turn_deg < NEW_PLACEMENT_TURN_DEG
        ):
            return index
    return None


def spread_m(recorded: List[dict]) -> Tuple[float, float]:
    """返回各组标定板中心在水平面里铺开的范围（长轴、短轴），单位米。"""
    if len(recorded) < 2:
        return 0., 0.
    centers = np.array(
        [np.mean(s['points_base_m'], axis=0)[:2] for s in recorded]
    )
    centered = centers - centers.mean(axis=0)
    _, _, axes = np.linalg.svd(centered, full_matrices=False)
    extent = np.ptp(centered @ axes.T, axis=0)
    return float(extent[0]), float(extent[1])


def camera_from_points(
    points_base_m: np.ndarray,
    pixels: np.ndarray,
    camera_matrix: np.ndarray,
    distortion: np.ndarray,
    initial_base_T_camera: np.ndarray,
) -> np.ndarray:
    """由“基座坐标里的点 ↔ 画面里的像素”求相机在基座坐标系里的位姿。

    Args:
        points_base_m: 空间点，M×3。
        pixels: 对应的像素坐标，M×2。
        camera_matrix: 相机内参矩阵。
        distortion: 镜头畸变参数。
        initial_base_T_camera: 迭代的初值。各组标定板都在桌面附近，
            接近共面，必须给初值才能稳定收敛。

    Returns:
        `基座_T_相机`，4×4。

    Raises:
        ValueError: 求解失败。
    """
    start = np.linalg.inv(initial_base_T_camera)
    found, rotation_vector, translation = cv2.solvePnP(
        points_base_m.astype(np.float64),
        pixels.astype(np.float64),
        camera_matrix,
        distortion,
        cv2.Rodrigues(start[:3, :3])[0],
        start[:3, 3].reshape(3, 1).copy(),
        useExtrinsicGuess=True,
        flags=cv2.SOLVEPNP_ITERATIVE,
    )
    if not found:
        raise ValueError('全局相机位姿求解失败')
    camera_T_base = np.eye(4)
    camera_T_base[:3, :3] = cv2.Rodrigues(rotation_vector)[0]
    camera_T_base[:3, 3] = translation.ravel()
    return np.linalg.inv(camera_T_base)


def sample_errors(
    base_T_camera: np.ndarray,
    sample: dict,
    camera_matrix: np.ndarray,
    distortion: np.ndarray,
) -> Tuple[float, float]:
    """算一组数据相对某个标定结果的两种偏差。

    Args:
        base_T_camera: 待评价的标定结果。
        sample: 一组数据。
        camera_matrix: 全局相机内参矩阵。
        distortion: 全局相机畸变参数。

    Returns:
        `(像素偏差, 空间偏差)`：把手上相机算出的角点投到全局相机画面，
        与实际检测到的角点相差多少（均方根，像素）；把全局相机自己看到
        的标定板按标定结果换到基座坐标，与手上相机算出的位置相差多少
        （均方根，米）。
    """
    points = np.asarray(sample['points_base_m'])
    camera_T_base = np.linalg.inv(base_T_camera)
    projected = cv2.projectPoints(
        points,
        cv2.Rodrigues(camera_T_base[:3, :3])[0],
        camera_T_base[:3, 3],
        camera_matrix,
        distortion,
    )[0].reshape(-1, 2)
    pixel_error = np.asarray(sample['corners_px']) - projected
    seen = transformed(
        base_T_camera @ np.asarray(sample['camera_T_board']), board_points()
    )
    return (
        float(np.sqrt(np.mean(np.sum(pixel_error ** 2, axis=1)))),
        float(np.sqrt(np.mean(np.sum((seen - points) ** 2, axis=1)))),
    )


def solve(
    samples: List[dict], camera_matrix: np.ndarray, distortion: np.ndarray
) -> dict:
    """由全部记录的数据求全局相机的位姿，并给出质量评价。

    先用每一组各自的结果取一个初值，再把全部角点放在一起求一次；偏差
    明显偏大的组剔除后重算一遍。

    Args:
        samples: 记录的各组数据。
        camera_matrix: 全局相机内参矩阵。
        distortion: 全局相机畸变参数。

    Returns:
        含 `base_T_camera`、所用的组 `used`、被剔除的组 `rejected`（序号
        从 1 起）、`reprojection_rms_px`、`disagreement_rms_m`、各组的
        偏差 `per_sample`、标定板铺开的范围 `spread_m`，以及
        `accepted`（是否达到验收条件）和不达标的原因 `problems`。

    Raises:
        ValueError: 没有数据，或求解失败。
    """
    if not samples:
        raise ValueError('没有可用的数据')
    first = samples[0]
    initial = (
        np.asarray(first['base_T_board'])
        @ np.linalg.inv(np.asarray(first['camera_T_board']))
    )
    used = list(range(len(samples)))
    rejected: List[int] = []
    for _ in range(2):
        chosen = [samples[i] for i in used]
        base_T_camera = camera_from_points(
            np.vstack([s['points_base_m'] for s in chosen]),
            np.vstack([s['corners_px'] for s in chosen]),
            camera_matrix,
            distortion,
            initial,
        )
        errors = [
            sample_errors(base_T_camera, s, camera_matrix, distortion)
            for s in samples
        ]
        limit = OUTLIER_FACTOR * float(
            np.median([errors[i][1] for i in used])
        )
        outliers = [
            i for i in used
            if errors[i][1] > max(limit, ACCEPT_DISAGREEMENT_M)
        ]
        if not outliers or len(used) - len(outliers) < 3:
            break
        rejected += outliers
        used = [i for i in used if i not in outliers]
        initial = base_T_camera
    chosen = [samples[i] for i in used]
    pixel = float(np.sqrt(np.mean([errors[i][0] ** 2 for i in used])))
    space = float(np.sqrt(np.mean([errors[i][1] ** 2 for i in used])))
    extent = spread_m(chosen)
    problems = []
    if pixel > ACCEPT_REPROJECTION_PX:
        problems.append(
            f'画面里的偏差 {pixel:.2f} 像素，超过 {ACCEPT_REPROJECTION_PX}'
        )
    if space > ACCEPT_DISAGREEMENT_M:
        problems.append(
            f'空间位置的偏差 {space * 1000:.1f} 毫米，超过 '
            f'{ACCEPT_DISAGREEMENT_M * 1000:.0f}'
        )
    if extent[1] < MIN_SPREAD_M:
        problems.append(
            f'标定板摆得太集中（短的方向只铺开 {extent[1] * 100:.0f} 厘米，'
            f'要求至少 {MIN_SPREAD_M * 100:.0f}）'
        )
    return {
        'base_T_camera': base_T_camera,
        'used': [i + 1 for i in used],
        'rejected': sorted(i + 1 for i in rejected),
        'reprojection_rms_px': pixel,
        'disagreement_rms_m': space,
        'per_sample': [
            {'sample': i + 1, 'pixel_rms': e[0], 'space_rms_m': e[1]}
            for i, e in enumerate(errors)
        ],
        'spread_m': list(extent),
        'accepted': not problems,
        'problems': problems,
    }


class Session:
    """一次标定的采集过程：逐轮检查三台相机的画面，记录合格的各组。

    Attributes:
        samples: 已记录的各组数据。
    """

    def __init__(
        self,
        root: pathlib.Path,
        arm_ids: List[str],
        relation: Optional[dict],
        directory: Optional[pathlib.Path] = None,
    ) -> None:
        """创建采集过程；不打开任何设备。

        Args:
            root: 项目根目录。
            arm_ids: 用哪几台臂的手上相机。
            relation: 两底座的相对位置；只用参考臂时可以为 None。
            directory: 保存各组原始画面与数据的目录；None 表示不保存。
        """
        self.root = pathlib.Path(root)
        self.arms = [arms.by_id(arm_id) for arm_id in arm_ids]
        self.relation = relation
        self.directory = directory
        self.samples: List[dict] = []
        self._previous_corners: Optional[np.ndarray] = None

    def hand_views(
        self, hand_frames: Dict[str, dict]
    ) -> Tuple[Dict[str, np.ndarray], List[str]]:
        """从各台手上相机的画面里算出标定板角点在基座坐标系里的位置。

        Args:
            hand_frames: 臂编号到一帧画面的映射，每帧含 `bgr`、`K` 和
                此刻的关节角 `joint_deg`。

        Returns:
            `(各臂算出的角点位置, 没看到或不合格的原因)`。
        """
        points: Dict[str, np.ndarray] = {}
        notes: List[str] = []
        for arm in self.arms:
            frame = hand_frames.get(arm.arm_id)
            if frame is None:
                notes.append(f'{arm.label}的相机没有画面')
                continue
            if not at_observation_pose(self.root, arm, frame['joint_deg']):
                notes.append(f'{arm.label}不在观测姿态')
                continue
            corners = detect_corners(frame['bgr'])
            if corners is None:
                notes.append(f'{arm.label}的相机没看到完整的标定板')
                continue
            height_px, width_px = frame['bgr'].shape[:2]
            pose, reason = screen_view(
                corners, width_px, height_px, np.asarray(frame['K']),
                np.zeros(5),
            )
            if pose is None:
                notes.append(f'{arm.label}的相机：{reason}')
                continue
            points[arm.arm_id] = transformed(
                hand_camera_pose(
                    self.root, arm, frame['joint_deg'], self.relation
                )
                @ pose,
                board_points(),
            )
        return points, notes

    def observe(
        self, fixed_frame: dict, hand_frames: Dict[str, dict]
    ) -> Tuple[Optional[dict], str]:
        """检查一轮画面；合格时返回一组数据，但不记录。

        Args:
            fixed_frame: 全局相机的一帧，含 `bgr`、`K`、`distortion`。
            hand_frames: 各台手上相机的画面，见 `hand_views`。

        Returns:
            `(一组数据或 None, 给操作者看的一句话)`。
        """
        corners = detect_corners(fixed_frame['bgr'])
        previous, self._previous_corners = self._previous_corners, corners
        if corners is None:
            return None, '全局相机没看到完整的标定板'
        height_px, width_px = fixed_frame['bgr'].shape[:2]
        camera_T_board, reason = screen_view(
            corners, width_px, height_px, np.asarray(fixed_frame['K']),
            np.asarray(fixed_frame['distortion']),
        )
        if camera_T_board is None:
            return None, '全局相机：' + reason
        if previous is None or (
            np.linalg.norm(corners - previous, axis=1).max() > STILL_PX
        ):
            return None, '标定板刚放下或还在动，请保持不动'
        points, notes = self.hand_views(hand_frames)
        if not points:
            return None, '；'.join(notes) or '没有可用的手上相机'
        seen_by = sorted(points)
        if len(seen_by) > 1:
            gap = float(
                np.sqrt(
                    np.mean(
                        np.sum(
                            (points[seen_by[0]] - points[seen_by[1]]) ** 2,
                            axis=1,
                        )
                    )
                )
            )
            if gap > ARM_AGREEMENT_M:
                return None, (
                    f'两台臂算出的标定板位置相差 {gap * 1000:.1f} 毫米，'
                    '超过上限；请确认两台臂都停稳在观测姿态'
                )
        points_base = np.mean([points[arm_id] for arm_id in seen_by], axis=0)
        base_T_board = np.eye(4)
        base_T_board[:3, :3], base_T_board[:3, 3] = rigid_fit(
            board_points(), points_base
        )
        return (
            {
                'corners_px': corners.tolist(),
                'camera_T_board': camera_T_board.tolist(),
                'points_base_m': points_base.tolist(),
                'base_T_board': base_T_board.tolist(),
                'seen_by': seen_by,
                'joint_deg': {
                    arm_id: list(hand_frames[arm_id]['joint_deg'])
                    for arm_id in seen_by
                },
            },
            '',
        )

    def step(
        self, fixed_frame: dict, hand_frames: Dict[str, dict]
    ) -> Tuple[bool, str]:
        """处理一轮画面：合格且不重复时记录为新的一组。

        Args:
            fixed_frame: 全局相机的一帧。
            hand_frames: 各台手上相机的画面。

        Returns:
            `(是否记录了新的一组, 给操作者看的一句话)`。
        """
        sample, reason = self.observe(fixed_frame, hand_frames)
        if sample is None:
            return False, reason
        repeated = placement_index(
            np.asarray(sample['points_base_m']), self.samples
        )
        if repeated is not None:
            return False, f'与第 {repeated} 组重复，请把标定板挪到新位置'
        self.samples.append(sample)
        self._save(len(self.samples), sample, fixed_frame, hand_frames)
        seen = '、'.join(arms.by_id(a).label for a in sample['seen_by'])
        return True, f'已记录第 {len(self.samples)} 组（{seen}看到）'

    def _save(
        self,
        number: int,
        sample: dict,
        fixed_frame: dict,
        hand_frames: Dict[str, dict],
    ) -> None:
        """把一组的原始画面和数据存到采集目录，供事后复查。"""
        if self.directory is None:
            return
        target = self.directory / f'sample_{number:02d}'
        target.mkdir(parents=True, exist_ok=True)
        cv2.imwrite(str(target / 'global.png'), fixed_frame['bgr'])
        for arm_id in sample['seen_by']:
            cv2.imwrite(
                str(target / f'{arm_id}.png'), hand_frames[arm_id]['bgr']
            )
        collect.atomic_json(target / 'sample.json', sample)

    def enough(self, wanted: int) -> Tuple[bool, str]:
        """判断是否已经采够。

        Args:
            wanted: 要求的组数。

        Returns:
            `(是否采够, 还差什么)`。
        """
        if len(self.samples) < wanted:
            return False, f'还需要 {wanted - len(self.samples)} 组'
        extent = spread_m(self.samples)
        if extent[1] < MIN_SPREAD_M:
            return False, (
                f'标定板摆得太集中（短的方向只铺开 {extent[1] * 100:.0f} '
                f'厘米），请往垂直于已摆过的方向再挪几处'
            )
        return True, ''


def rigid_fit(
    source: np.ndarray, target: np.ndarray
) -> Tuple[np.ndarray, np.ndarray]:
    """求把一组点对齐到另一组点的刚体变换（最小二乘）。

    Args:
        source: 原始点，N×3。
        target: 目标点，N×3，与原始点一一对应。

    Returns:
        `(旋转矩阵, 平移)`，使 `旋转 @ 原始点 + 平移` 最接近目标点。
    """
    source_center, target_center = source.mean(axis=0), target.mean(axis=0)
    u, _, vt = np.linalg.svd(
        (target - target_center).T @ (source - source_center)
    )
    sign = np.sign(np.linalg.det(u @ vt))
    rotation = u @ np.diag([1., 1., sign]) @ vt
    return rotation, target_center - rotation @ source_center


class Backend:
    """8089 后端的只读访问：取手上相机的画面和机械臂的关节角。"""

    def __init__(self, base_url: str, token_file: pathlib.Path) -> None:
        """记下后端地址和访问令牌文件；不发起连接。

        Args:
            base_url: 后端地址。
            token_file: 访问令牌文件。
        """
        self.base_url = base_url.rstrip('/')
        self.token_file = pathlib.Path(token_file).expanduser()
        self.opener = urllib.request.build_opener(
            urllib.request.ProxyHandler({})
        )

    def get(self, path: str) -> dict:
        """发一次只读查询。

        Args:
            path: 以 `/api/agent/` 开头的路径，可带查询串。

        Returns:
            应答的 JSON。

        Raises:
            OSError: 后端连不上或返回错误。
        """
        request = urllib.request.Request(
            self.base_url + path,
            headers={
                'Authorization': 'Bearer '
                + self.token_file.read_text().strip()
            },
        )
        with self.opener.open(request, timeout=10) as response:
            return json.load(response)

    def hand_frame(self, arm_id: str) -> Optional[dict]:
        """取一台臂的手上相机画面和此刻的关节角。

        Args:
            arm_id: 臂编号。

        Returns:
            含 `bgr`、`K`、`joint_deg`；这台臂暂时取不到画面或状态时
            为 None（原因记入日志）。
        """
        try:
            frame = self.get(f'/api/agent/frame?arm={arm_id}')
            robot = self.get(f'/api/agent/robot?arm={arm_id}')
        except (OSError, ValueError) as error:
            LOG.warning('取不到 %s 的画面或状态：%r', arm_id, error)
            return None
        image = cv2.imdecode(
            np.frombuffer(base64.b64decode(frame['image_base64']), np.uint8),
            cv2.IMREAD_COLOR,
        )
        return {
            'bgr': image,
            'K': frame['camera']['K'],
            'joint_deg': robot['joint_deg'],
        }


class FrameUnavailable(RuntimeError):
    """全局相机这一轮取不到画面；原因写在异常信息里，给操作者看。"""


class ServedCamera:
    """经 8088 相机服务读取全局相机的彩色画面；本程序不打开相机。

    每次取图都让 8088 再保持采集一小段时间（`CAMERA_KEEP_S`）。程序
    退出后不用归还：期限一到，没有人看预览的相机由 8088 自行释放。

    Attributes:
        serial: 相机序列号。
        camera_matrix: 彩色画面的内参矩阵。
        distortion: 畸变系数。
        distortion_model: 畸变模型的名字。
    """

    def __init__(self, base_url: str, serial: str) -> None:
        """在相机服务里找到这台相机并取第一帧，必要时由服务启动相机。

        Args:
            base_url: 相机服务的地址。
            serial: 相机序列号。

        Raises:
            FrameUnavailable: 相机服务没有运行、不认识这台相机，或
                相机打不开。
        """
        self.base_url = base_url.rstrip('/')
        self.serial = serial
        self.opener = urllib.request.build_opener(
            urllib.request.ProxyHandler({})
        )
        cameras = self.get('/api/status').get('cameras', {})
        self.key = next(
            (
                key
                for key, row in cameras.items()
                if row.get('camera_serial') == serial
            ),
            None,
        )
        if self.key is None:
            raise FrameUnavailable(
                f'相机服务里没有序列号为 {serial} 的相机，请检查 '
                f'{CAMERA_CONFIG_FILE} 后重启相机服务'
            )
        self.camera_matrix = np.eye(3)
        self.distortion = np.zeros(5)
        self.distortion_model = ''
        self.frame()

    def get(self, path: str) -> dict:
        """向相机服务发一次只读请求。

        Args:
            path: 请求路径，可带查询串。

        Returns:
            应答的 JSON。

        Raises:
            FrameUnavailable: 相机服务连不上，或拒绝了请求。
        """
        try:
            with self.opener.open(self.base_url + path, timeout=15) as reply:
                return json.load(reply)
        except urllib.error.HTTPError as error:
            try:
                reason = json.load(error).get('error')
            except ValueError:
                reason = None
            raise FrameUnavailable(
                reason or f'相机服务返回 {error.code}'
            ) from error
        except (OSError, ValueError) as error:
            raise FrameUnavailable(
                f'连不上相机服务 {self.base_url}（{error}）；请先启动它：'
                './scripts/run_camera_monitor.sh'
            ) from error

    def frame(self) -> dict:
        """取一帧本次请求之后拍到的无损画面。

        Returns:
            含 `bgr`、`K`、`distortion`。

        Raises:
            FrameUnavailable: 这一轮取不到画面。
        """
        query = urllib.parse.urlencode(
            {'keep_s': CAMERA_KEEP_S, 'format': 'png'}
        )
        data = self.get(f'/api/cameras/{self.key}/frame?{query}')
        intrinsics = data.get('intrinsics')
        if data.get('camera_serial') != self.serial or not intrinsics:
            raise FrameUnavailable('相机服务给出的画面不是这台相机直接拍的')
        image = cv2.imdecode(
            np.frombuffer(base64.b64decode(data['image_base64']), np.uint8),
            cv2.IMREAD_COLOR,
        )
        if image is None:
            raise FrameUnavailable('相机服务给出的图像无法解码')
        self.camera_matrix = np.array(
            [
                [intrinsics['fx'], 0., intrinsics['cx']],
                [0., intrinsics['fy'], intrinsics['cy']],
                [0., 0., 1.],
            ]
        )
        self.distortion = np.array(intrinsics['distortion'], dtype=float)
        self.distortion_model = intrinsics['distortion_model']
        return {
            'bgr': image,
            'K': self.camera_matrix,
            'distortion': self.distortion,
        }


def load_previous(root: pathlib.Path) -> Optional[np.ndarray]:
    """读取上一次的标定结果；没有或读不出时为 None。"""
    path = pathlib.Path(root) / RESULT_FILE
    if not path.is_file():
        return None
    try:
        return np.array(
            yaml.safe_load(path.read_text())['arm1_base_T_camera'],
            dtype=float,
        )
    except (yaml.YAMLError, KeyError, TypeError, ValueError):
        LOG.warning('上一次的标定结果 %s 读不出来，按没有处理', path)
        return None


def change_from(
    previous: np.ndarray, current: np.ndarray
) -> Tuple[float, float]:
    """返回两次标定之间相机挪动的距离（米）和转过的角度（度）。"""
    rotation = previous[:3, :3].T @ current[:3, :3]
    angle = np.arccos(np.clip((np.trace(rotation) - 1) / 2, -1., 1.))
    return (
        float(np.linalg.norm(current[:3, 3] - previous[:3, 3])),
        float(np.degrees(angle)),
    )


def write_result(
    root: pathlib.Path,
    result: dict,
    camera: Any,
    session_directory: pathlib.Path,
    arm_ids: List[str],
) -> pathlib.Path:
    """把验收通过的标定结果写入本机配置；已有的结果先备份。

    Args:
        root: 项目根目录。
        result: `solve` 的返回值。
        camera: 全局相机，取序列号和内参。
        session_directory: 本次采集的目录。
        arm_ids: 用了哪几台臂的手上相机。

    Returns:
        写出的配置文件路径。
    """
    path = pathlib.Path(root) / RESULT_FILE
    now = datetime.datetime.now().astimezone()
    if path.is_file():
        backup = path.with_name(
            path.name + '.before-' + now.strftime('%Y%m%dT%H%M%S')
        )
        backup.write_bytes(path.read_bytes())
        LOG.info('原有的标定结果已备份为 %s', backup.name)
    summary = session_directory / 'result.json'
    record = {
        'schema_version': SCHEMA_VERSION,
        'camera_serial': camera.serial,
        'frame': 'color_optical_frame',
        'frame_convention': 'A_T_B maps coordinates in B into A',
        'arm1_base_T_camera': np.asarray(result['base_T_camera']).tolist(),
        'measured_at': now.isoformat(timespec='seconds'),
        'method': 'board_seen_by_refined_hand_cameras_and_global_camera',
        'hand_camera_arms': arm_ids,
        'samples_used': len(result['used']),
        'reprojection_rms_px': round(result['reprojection_rms_px'], 3),
        'disagreement_rms_m': round(result['disagreement_rms_m'], 5),
        'board_spread_m': [round(v, 3) for v in result['spread_m']],
        'intrinsics': {
            'K': camera.camera_matrix.tolist(),
            'distortion_model': camera.distortion_model,
            'distortion_coefficients': camera.distortion.tolist(),
            'size_px': [1280, 720],
        },
        'acceptance_status': 'meets_script_thresholds',
        'source': {
            'path': str(summary.relative_to(root)),
            'sha256': hashlib.sha256(summary.read_bytes()).hexdigest(),
        },
    }
    path.write_text(
        '# 全局相机（D405）相对 1 臂基座的位置和朝向。由\n'
        '# saydo.calibration.global_camera 生成；相机被取下重装后要重新标定。\n'
        + yaml.safe_dump(record, allow_unicode=True, sort_keys=False)
    )
    return path


def gather(backend: Backend, arm_ids: List[str]) -> Dict[str, dict]:
    """取各台手上相机的画面与关节角；取不到的臂不在结果里。"""
    frames = {}
    for arm_id in arm_ids:
        frame = backend.hand_frame(arm_id)
        if frame is not None:
            frames[arm_id] = frame
    return frames


def calibrate(
    session: Session,
    camera: Any,
    backend: Any,
    arm_ids: List[str],
    wanted: int,
    timeout_s: float,
    clock: Callable[[], float] = time.monotonic,
    sleep: Callable[[float], None] = time.sleep,
) -> bool:
    """采集循环：每秒一轮，直到采够或超时。

    Args:
        session: 采集过程。
        camera: 全局相机，提供 `frame()`。
        backend: 8089 后端，提供 `hand_frame(arm_id)`。
        arm_ids: 用哪几台臂的手上相机。
        wanted: 要求的组数。
        timeout_s: 最长采集多久，单位秒。
        clock: 计时函数，测试时替换。
        sleep: 等待函数，测试时替换。

    Returns:
        是否采够。
    """
    started = clock()
    last_message = ''
    while clock() - started < timeout_s:
        round_started = clock()
        try:
            recorded, message = session.step(
                camera.frame(), gather(backend, arm_ids)
            )
        except FrameUnavailable as error:
            recorded, message = False, f'全局相机暂时取不到画面：{error}'
        done, missing = session.enough(wanted)
        if recorded:
            LOG.info('%s。%s', message, '已采够' if done else missing)
        elif message != last_message:
            # 同一句提示只打印一次，避免每秒刷屏。
            LOG.info('%s', message)
        last_message = message
        if done:
            return True
        sleep(max(0., LOOP_PERIOD_S - (clock() - round_started)))
    return False


def report(result: dict, previous: Optional[np.ndarray]) -> None:
    """把求解结果和质量评价打印给操作者。"""
    position = np.asarray(result['base_T_camera'])[:3, 3] * 1000
    LOG.info(
        '全局相机在 1 臂基座坐标里的位置：(%.0f, %.0f, %.0f) 毫米',
        *position,
    )
    LOG.info(
        '用了 %d 组，剔除 %s；画面偏差 %.2f 像素，空间偏差 %.1f 毫米；'
        '标定板铺开 %.0f × %.0f 厘米',
        len(result['used']),
        result['rejected'] or '无',
        result['reprojection_rms_px'],
        result['disagreement_rms_m'] * 1000,
        result['spread_m'][0] * 100,
        result['spread_m'][1] * 100,
    )
    if previous is not None:
        shift_m, turn_deg = change_from(previous, result['base_T_camera'])
        LOG.info(
            '与上一次标定相比：位置相差 %.1f 毫米，朝向相差 %.2f 度',
            shift_m * 1000, turn_deg,
        )
    for problem in result['problems']:
        LOG.warning('未达标：%s', problem)


def verify(
    session: Session,
    camera: Any,
    backend: Any,
    arm_ids: List[str],
    previous: np.ndarray,
    timeout_s: float,
    clock: Callable[[], float] = time.monotonic,
    sleep: Callable[[float], None] = time.sleep,
) -> Optional[Tuple[float, float]]:
    """核对现有标定：拍到合格的一组后报告偏差，不改任何文件。

    Args:
        session: 采集过程（只用它的检查，不记录）。
        camera: 全局相机。
        backend: 8089 后端。
        arm_ids: 用哪几台臂的手上相机。
        previous: 现有的标定结果。
        timeout_s: 最长等多久，单位秒。
        clock: 计时函数。
        sleep: 等待函数。

    Returns:
        `(像素偏差, 空间偏差米)`；超时仍没有拍到合格的一组时为 None。
    """
    started = clock()
    last_message = ''
    while clock() - started < timeout_s:
        try:
            frame = camera.frame()
            sample, message = session.observe(
                frame, gather(backend, arm_ids)
            )
        except FrameUnavailable as error:
            sample, message = None, f'全局相机暂时取不到画面：{error}'
        if sample is not None:
            return sample_errors(
                previous, sample, np.asarray(frame['K']),
                np.asarray(frame['distortion']),
            )
        if message != last_message:
            LOG.info('%s', message)
        last_message = message
        sleep(LOOP_PERIOD_S)
    return None


def main() -> int:
    """命令行入口：标定或核对全局相机；不发送任何机械臂指令。

    Returns:
        进程退出码：0 表示标定达标并已写入（或核对在限值之内），1 表示
        没有采够、未达标或核对超出限值。
    """
    parser = argparse.ArgumentParser(description='全局相机快速外参标定')
    parser.add_argument('--verify', action='store_true', help='只核对现有标定')
    parser.add_argument('--samples', type=int, default=DEFAULT_SAMPLES)
    parser.add_argument('--timeout-s', type=float, default=300.)
    parser.add_argument(
        '--arms',
        default=None,
        help='用哪几台臂的手上相机，逗号分隔；缺省为全部',
    )
    parser.add_argument('--backend-url', default=DEFAULT_BACKEND_URL)
    parser.add_argument(
        '--camera-url',
        default=collect.PREVIEW_URL,
        help='8088 相机服务的地址，全局相机的画面从它那里取',
    )
    parser.add_argument('--token-file', default=DEFAULT_TOKEN_FILE)
    args = parser.parse_args()
    logging.basicConfig(
        level=logging.INFO, format='%(asctime)s %(message)s', datefmt='%H:%M:%S'
    )
    root = pathlib.Path.cwd()
    arm_ids = (
        args.arms.split(',')
        if args.arms
        else [arm.arm_id for arm in arms.configured(root)]
    )
    relation = base_frames.load(root) if arm_ids != [REFERENCE_ARM_ID] else None
    serial = json.loads((root / CAMERA_CONFIG_FILE).read_text())['serial']
    previous = load_previous(root)
    if args.verify and previous is None:
        LOG.error('还没有标定结果，无法核对')
        return 1
    directory = None
    if not args.verify:
        directory = root / 'work/calibration' / (
            'global_camera_'
            + datetime.datetime.now().strftime('%Y%m%dT%H%M%S')
        )
    backend = Backend(args.backend_url, args.token_file)
    session = Session(root, arm_ids, relation, directory)
    try:
        camera = ServedCamera(args.camera_url, serial)
    except FrameUnavailable as error:
        LOG.error('全局相机不可用：%s', error)
        return 1
    LOG.info(
        '全局相机的画面取自相机服务 %s；采集期间可以在它的页面上看着'
        '画面摆标定板', args.camera_url,
    )
    if args.verify:
        LOG.info('请把标定板平放在机械臂前方、三台相机都能看到的位置')
        errors = verify(
            session, camera, backend, arm_ids, previous, args.timeout_s
        )
        if errors is None:
            LOG.error('限时内没有拍到合格的一组')
            return 1
        good = (
            errors[0] <= ACCEPT_REPROJECTION_PX
            and errors[1] <= ACCEPT_DISAGREEMENT_M
        )
        LOG.info(
            '现有标定与当前实际：画面偏差 %.2f 像素，空间偏差 %.1f 毫米'
            '——%s',
            errors[0], errors[1] * 1000,
            '仍然可用' if good else '已经不准，请重新标定',
        )
        return 0 if good else 1
    LOG.info(
        '开始采集，需要 %d 组。请把标定板平放在桌上，每记录一组后挪到'
        '新的位置', args.samples,
    )
    if not calibrate(
        session, camera, backend, arm_ids, args.samples, args.timeout_s
    ):
        LOG.error(
            '限时内没有采够（已有 %d 组），未改动标定结果',
            len(session.samples),
        )
        return 1
    result = solve(session.samples, camera.camera_matrix, camera.distortion)
    directory.mkdir(parents=True, exist_ok=True)
    collect.atomic_json(
        directory / 'result.json',
        {
            **result,
            'base_T_camera': np.asarray(result['base_T_camera']).tolist(),
        },
    )
    report(result, previous)
    if not result['accepted']:
        LOG.error('标定未达标，未改动标定结果；数据在 %s', directory)
        return 1
    path = write_result(root, result, camera, directory, arm_ids)
    LOG.info('标定达标，已写入 %s', os.path.relpath(path, root))
    return 0


if __name__ == '__main__':
    sys.exit(main())
