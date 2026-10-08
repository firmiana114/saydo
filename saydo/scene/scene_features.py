"""Model-local generic named feature frames."""

import hashlib
import json
import pathlib
from saydo.geometry import grasp_geometry
from saydo.objects import object_catalog


def object_features(models_root, object_id):
    object_catalog.select_object(models_root, object_id)
    root = pathlib.Path(models_root) / object_id
    features = []
    hashes = {}
    notes = []
    for name in ('features.json',):
        path = root / name
        if not path.exists():
            continue
        if path.is_symlink() or path.stat().st_size > 256 * 1024:
            raise ValueError('模型特征文件无效或过大')
        raw = path.read_bytes()
        data = json.loads(raw)
        hashes[name] = hashlib.sha256(raw).hexdigest()
        if data.get('units') != 'metres':
            raise ValueError('特征文件必须为米制')
        notes.extend(data.get('notes', []))
        for row in data['features']:
            pose = grasp_geometry.transform(row['object_T_feature'])
            features.append(
                {
                    'id': row['id'],
                    'kind': row.get('kind', 'reference'),
                    'object_T_feature': pose.tolist(),
                    'physical_fit_verified': False,
                    **{
                        k: row[k]
                        for k in ('diameter_m', 'identity_scope')
                        if k in row
                    },
                }
            )
    if len(features) > 256:
        raise ValueError('模型特征数量过多')
    names = set()
    for row in features:
        name = row['id']
        if (
            not isinstance(name, str)
            or not name
            or len(name) > 100
            or name in names
        ):
            raise ValueError('模型特征ID无效或重复')
        names.add(name)
    return {
        'features': features,
        'feature_revision': hashlib.sha256(
            json.dumps(hashes, sort_keys=True).encode()
        ).hexdigest(),
        'feature_notes': notes,
        'feature_scope': (
            'model_geometry_not_scene_occupancy_or_contact_verification'
        ),
    }
