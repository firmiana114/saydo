"""Model-local named feature frames, including legacy rack geometry metadata."""
import hashlib
import json
from pathlib import Path
import numpy as np
from grasp_geometry import transform
from object_catalog import select_object


def object_features(models_root, object_id):
    select_object(models_root,object_id)
    root=Path(models_root)/object_id;features=[];hashes={};notes=[]
    for name in ('features.json','hole_positions.json'):
        path=root/name
        if not path.exists():continue
        if path.is_symlink() or path.stat().st_size>256*1024:raise ValueError('模型特征文件无效或过大')
        raw=path.read_bytes();data=json.loads(raw);hashes[name]=hashlib.sha256(raw).hexdigest()
        if data.get('units')!='metres':raise ValueError('特征文件必须为米制')
        if name=='features.json':
            for row in data['features']:
                pose=transform(row['object_T_feature']);features.append({'id':row['id'],'kind':row.get('kind','reference'),
                    'object_T_feature':pose.tolist(),'physical_fit_verified':False})
        else:
            notes.append(data.get('registration_note',''))
            for slot in data['slots']:
                for field,suffix in [('upper_entry_center_m','entry'),('middle_entry_center_m','middle'),('bottom_support_point_m','support')]:
                    point=np.asarray(slot[field],dtype=float)
                    if point.shape!=(3,) or not np.isfinite(point).all():raise ValueError('孔位特征坐标无效')
                    pose=np.eye(4);pose[:3,3]=point
                    features.append({'id':f'{slot["id"]}.{suffix}','kind':'hole_'+suffix,'object_T_feature':pose.tolist(),
                        'diameter_m':slot['hole_diameter_m'],'physical_fit_verified':False,
                        'identity_scope':'estimated_model_frame_only_not_physical_slot_label'})
    if len(features)>256:raise ValueError('模型特征数量过多')
    names=set()
    for row in features:
        name=row['id']
        if not isinstance(name,str) or not name or len(name)>100 or name in names:raise ValueError('模型特征ID无效或重复')
        names.add(name)
    return {'features':features,'feature_revision':hashlib.sha256(json.dumps(hashes,sort_keys=True).encode()).hexdigest(),
            'feature_notes':notes,'feature_scope':'model_geometry_not_scene_occupancy_or_contact_verification'}
