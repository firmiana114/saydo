"""Run all instance localization on one immutable RGB-D frame, on the GPU owner thread."""

import hashlib
import pathlib
import time
from typing import Any, Callable, Dict, List

import numpy as np
from saydo.perception import frame_coverage
from saydo.perception import live_checks
from saydo.geometry import pose_symmetry
from saydo.scene import scene_features


def automatic_candidates(frame, info, detector, local_detector, check):
    check()
    if detector is None:
        key = getattr(local_detector, 'settings_key', 'local_detection')
        if local_detector is None or not info.get(key):
            raise ValueError('目标没有本地自动检测配置，请提供目标框')
        return local_detector.detect(frame, info[key], lambda: cancelled(check))
    while detector.inflight.locked():
        check()
        time.sleep(.05)
    check()
    return detector.detect(
        frame,
        info.get('effective_detection_prompt', info['name']),
        lambda: cancelled(check),
        frozen_scene=True,
    )


def localize_scene(
    frame: Dict[str, Any],
    targets: List[Dict[str, Any]],
    models_root: pathlib.Path,
    estimator: Any,
    sam: Any,
    context: Any,
    detector: Any,
    check: Callable[[], None],
    progress: Callable[[str], None],
    local_detector: Any = None,
) -> List[Dict[str, Any]]:
    """在同一帧冻结的彩色加深度画面上，依次定位场景里的全部目标。

    每个目标依次经过：找框（未给框时自动检测）、分割、位姿估计、出画检查、
    深度与轮廓质量检查、同帧细化的稳定性检查。任何一项不通过都会中止整次
    采集，因为后续动作要求所有目标来自同一时刻。只读，不连接机械臂。

    Args:
        frame: 冻结的画面，含 `bgr`（640×360 彩色图）、`depth`（同尺寸
            深度图，单位米）、`K`（相机内参矩阵）。
        targets: 要定位的目标，每项含 `instance_id`、`object_id`、`role`
            和 `box`（像素框，`None` 表示自动检测）。
        models_root: 物品模型的根目录。
        estimator: 已加载的 FoundationPose 估计器，借用它的打分与细化网络。
        sam: 已加载的分割模型。
        context: 渲染用的显卡上下文。
        detector: 大模型找框器；为 `None` 时用 `local_detector`。
        check: 取消与超时检查，需要中止时由它抛出异常。
        progress: 进度提示的回调，参数是给操作者看的文字。
        local_detector: 本地找框器。

    Returns:
        与 `targets` 一一对应的定位结果，在原字段之外加上 `box`、
        `camera_T_object`（物品在相机光学坐标系里的位姿）、`quality`、
        `frame_coverage`（出画检查结果）和模型的特征信息。

    Raises:
        ValueError: 找不到唯一候选、框无效、同一模型的实例框重叠、有效
            深度不足、物品出画超过上限、质量或稳定性检查不通过。
    """
    import cv2
    import trimesh
    import torch
    from estimater import FoundationPose
    from Utils import nvdiffrast_render
    from saydo.objects import object_catalog
    rgb = cv2.cvtColor(frame['bgr'], cv2.COLOR_BGR2RGB)
    rows = []
    used = []
    for target in targets:
        check()
        info = object_catalog.select_object(models_root, target['object_id'])
        progress('定位场景实例：' + target['instance_id'])
        box = target['box']
        if box is None:
            candidates = automatic_candidates(
                frame, info, detector, local_detector, check
            )
            if len(candidates) != 1:
                raise ValueError(
                    f'{target["instance_id"]}未得到唯一可靠候选（'
                    f'{len(candidates)}个），请结合画面指定box后重新采集'
                )
            box = candidates[0]['box']
        box = live_checks.checked_box(box, 640, 360)
        for model, previous in used:
            if model == target['object_id']:
                a = np.asarray(box)
                b = np.asarray(previous)
                overlap = np.maximum(
                    0, np.minimum(a[2:], b[2:]) - np.maximum(a[:2], b[:2])
                ).prod()
                union = (
                    (a[2:] - a[:2]).prod() + (b[2:] - b[:2]).prod() - overlap
                )
                if overlap / max(1, union) > .5:
                    raise ValueError(
                        '同一模型的实例框重叠，不能重复登记同一物品'
                    )
        used.append((target['object_id'], box))
        check()
        mesh_sha256 = hashlib.sha256(
            pathlib.Path(info['mesh']).read_bytes()
        ).hexdigest()
        mesh = trimesh.load(info['mesh'], force='mesh')
        if (
            not np.isfinite(mesh.vertices).all()
            or not .01 < mesh.extents.max() < 1
        ):
            raise ValueError('场景模型尺寸异常')
        with torch.inference_mode():
            sam.set_image(rgb)
            masks, scores, _ = sam.predict(
                box=np.asarray(box), multimask_output=True
            )
            mask = masks[int(np.argmax(scores))]
            if (
                mask.sum() < 100
                or (
                    (
                        (frame['depth'] >= .1) & (frame['depth'] <= 2) & mask
                    ).sum()
                    < 100
                )
            ):
                raise ValueError('场景目标有效深度不足')
            check()
            instance = FoundationPose(
                model_pts=mesh.vertices,
                model_normals=mesh.vertex_normals,
                mesh=mesh,
                scorer=estimator.scorer,
                refiner=estimator.refiner,
                glctx=context,
                debug=0,
                debug_dir=estimator.debug_dir,
            )
            pose = instance.register(
                K=frame['K'],
                rgb=rgb,
                depth=frame['depth'].copy(),
                ob_mask=mask,
                iteration=5,
            )
            check()
            # 排在质量检查之前：被裁掉的物品往往各项质量指标都通过，
            # 只有这一项能发现；先做也能给出更直接的失败原因。
            coverage = frame_coverage.measure(
                pose,
                mesh.triangles_center,
                mesh.area_faces,
                frame['K'],
                width_px=640,
                height_px=360,
            )
            frame_coverage.require_inside(target['instance_id'], coverage)
            quality = None
            for k in range(2):
                _, rd, _ = nvdiffrast_render(
                    K=frame['K'],
                    H=360,
                    W=640,
                    mesh=mesh,
                    ob_in_cams=torch.as_tensor(
                        pose[None], device='cuda', dtype=torch.float32
                    ),
                    glctx=context,
                )
                quality = live_checks.pose_health(
                    pose,
                    rd[0].detach().cpu().numpy(),
                    frame['depth'],
                    None,
                    mask,
                )
                if not quality['good']:
                    raise ValueError(
                        f'{target["instance_id"]}位姿/深度质量检查不通过：'
                        f'{quality}'
                    )
                if k == 0:
                    prior = pose.copy()
                    pose = instance.track_one(
                        rgb=rgb,
                        depth=frame['depth'].copy(),
                        K=frame['K'],
                        iteration=2,
                    )
                    shift = np.linalg.norm(pose[:3, 3] - prior[:3, 3])
                    turn = pose_symmetry.object_rotation_error(
                        pose[:3, :3], prior[:3, :3], info.get('pose_mode', '6d')
                    )
                    if shift > .005 or turn > np.deg2rad(15):
                        # Name the instance and the measured change; either
                        # object can be the unstable one.
                        raise ValueError(
                            f'{target["instance_id"]}同帧细化位姿变化超过5毫米/'
                            f'15度（实际平移{shift*1000:.1f}毫米、'
                            f'旋转{np.rad2deg(turn):.1f}度）'
                        )
                check()
            if info.get('pose_mode') == '5d':
                pose = pose_symmetry.canonical_pose(pose, '5d')
            rows.append(
                {
                    **target,
                    'box': box,
                    'object_name': info['name'],
                    'camera_T_object': np.asarray(pose).tolist(),
                    'model_sha256': mesh_sha256,
                    'pose_mode': info.get('pose_mode', '6d'),
                    'pose_mode_revision': info.get('pose_mode_revision'),
                    'quality': quality,
                    'frame_coverage': coverage,
                    **scene_features.object_features(
                        models_root, target['object_id']
                    ),
                }
            )
            del instance
    return rows


def cancelled(check):
    try:
        check()
        return False
    except Exception:
        return True
