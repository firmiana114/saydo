"""Run all instance localization on one immutable RGB-D frame, on the GPU owner thread."""
import hashlib
from pathlib import Path
import time
import numpy as np
from live_checks import checked_box,pose_health
from pose_symmetry import canonical_pose,object_rotation_error
from scene_features import object_features


def automatic_candidates(frame,info,detector,local_detector,check):
    check()
    if detector is None:
        if local_detector is None or not info.get('local_detection'):raise ValueError('目标没有本地自动检测配置，请提供目标框')
        return local_detector.detect(frame,info['local_detection'],lambda:cancelled(check))
    while detector.inflight.locked():check();time.sleep(.05)
    check()
    return detector.detect(frame,info.get('effective_detection_prompt',info['name']),
                           lambda:cancelled(check),frozen_scene=True)


def localize_scene(frame,targets,models_root,estimator,sam,context,detector,check,progress,local_detector=None):
    import cv2
    import trimesh
    import torch
    from estimater import FoundationPose
    from Utils import nvdiffrast_render
    from object_catalog import select_object
    rgb=cv2.cvtColor(frame['bgr'],cv2.COLOR_BGR2RGB);rows=[];used=[]
    for target in targets:
        check();info=select_object(models_root,target['object_id']);progress('定位场景实例：'+target['instance_id'])
        box=target['box']
        if box is None:
            candidates=automatic_candidates(frame,info,detector,local_detector,check)
            if len(candidates)!=1:raise ValueError(f'{target["instance_id"]}未得到唯一可靠候选（{len(candidates)}个），请结合画面指定box后重新采集')
            box=candidates[0]['box']
        box=checked_box(box,640,360)
        for model,previous in used:
            if model==target['object_id']:
                a=np.asarray(box);b=np.asarray(previous);overlap=np.maximum(0,np.minimum(a[2:],b[2:])-np.maximum(a[:2],b[:2])).prod()
                union=(a[2:]-a[:2]).prod()+(b[2:]-b[:2]).prod()-overlap
                if overlap/max(1,union)>.5:raise ValueError('同一模型的实例框重叠，不能重复登记同一物品')
        used.append((target['object_id'],box));check()
        mesh_sha256=hashlib.sha256(Path(info['mesh']).read_bytes()).hexdigest()
        mesh=trimesh.load(info['mesh'],force='mesh')
        if not np.isfinite(mesh.vertices).all() or not .01<mesh.extents.max()<1:raise ValueError('场景模型尺寸异常')
        with torch.inference_mode():
            sam.set_image(rgb);masks,scores,_=sam.predict(box=np.asarray(box),multimask_output=True);mask=masks[int(np.argmax(scores))]
            if mask.sum()<100 or (((frame['depth']>=.1)&(frame['depth']<=2)&mask).sum()<100):raise ValueError('场景目标有效深度不足')
            check()
            instance=FoundationPose(model_pts=mesh.vertices,model_normals=mesh.vertex_normals,mesh=mesh,
                scorer=estimator.scorer,refiner=estimator.refiner,glctx=context,debug=0,debug_dir=estimator.debug_dir)
            pose=instance.register(K=frame['K'],rgb=rgb,depth=frame['depth'].copy(),ob_mask=mask,iteration=5)
            check();quality=None
            for k in range(2):
                _,rd,_=nvdiffrast_render(K=frame['K'],H=360,W=640,mesh=mesh,
                    ob_in_cams=torch.as_tensor(pose[None],device='cuda',dtype=torch.float32),glctx=context)
                quality=pose_health(pose,rd[0].detach().cpu().numpy(),frame['depth'],None,mask)
                if not quality['good']:raise ValueError(f'{target["instance_id"]}位姿/深度质量检查不通过：{quality}')
                if k==0:
                    prior=pose.copy();pose=instance.track_one(rgb=rgb,depth=frame['depth'].copy(),K=frame['K'],iteration=2)
                    if np.linalg.norm(pose[:3,3]-prior[:3,3])>.005 or object_rotation_error(pose[:3,:3],prior[:3,:3],info.get('pose_mode','6d'))>np.deg2rad(15):raise ValueError('同帧细化位姿变化超过5毫米/15度')
                check()
            if info.get('pose_mode')=='5d':pose=canonical_pose(pose,'5d')
            rows.append({**target,'box':box,'object_name':info['name'],'camera_T_object':np.asarray(pose).tolist(),
                'model_sha256':mesh_sha256,
                'pose_mode':info.get('pose_mode','6d'),'pose_mode_revision':info.get('pose_mode_revision'),
                'quality':quality,**object_features(models_root,target['object_id'])})
            del instance
    return rows


def cancelled(check):
    try:check();return False
    except Exception:return True
