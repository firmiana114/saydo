"""Bounded display-only material payload; never used for motion geometry."""

import base64
from functools import lru_cache
from io import BytesIO
from pathlib import Path

import numpy as np

from object_catalog import select_object


def mesh_appearance(mesh):
    """Keep UV/face indexing identical to the process=False reorientation mesh."""
    visual = mesh.visual
    result = {'materials': [], 'status': 'plain'}
    uv = getattr(visual, 'uv', None)
    if uv is not None:
        uv = np.asarray(uv, dtype=float)
        if uv.shape != (len(mesh.vertices), 2) or not np.isfinite(uv).all():
            raise ValueError('模型纹理坐标无效')
        result['uv'] = uv.tolist()
    material = getattr(visual, 'material', None)
    materials = (
        list(material.materials)
        if hasattr(material, 'materials')
        else [material]
    )
    if len(materials) > 32:
        raise ValueError('预览模型材质数量超过 32')
    for material in materials:
        image = getattr(material, 'baseColorTexture', None)
        if image is None:
            image = getattr(material, 'image', None)
        color = getattr(material, 'baseColorFactor', None)
        if color is None:
            color = getattr(material, 'diffuse', [164, 201, 222, 255])
        color = np.asarray(color, dtype=float).reshape(-1)
        if len(color) not in (3, 4) or not np.isfinite(color).all():
            color = np.array([164., 201., 222., 255.])
        # trimesh Simple/PBR materials expose byte RGBA values.
        rgba = np.clip(color / 255, 0, 1).tolist()
        if len(rgba) == 3:
            rgba.append(1.)
        entry = {'color': rgba}
        if image is not None and uv is not None:
            image = image.convert('RGBA')
            image.thumbnail((1024, 1024))
            buffer = BytesIO()
            image.save(buffer, format='PNG', optimize=True)
            entry['texture'] = (
                'data:image/png;base64,'
                + base64.b64encode(buffer.getvalue()).decode('ascii')
            )
            entry['size'] = list(image.size)
            # OBJ Kd is often an exporter default; the diffuse map contains the
            # color.
            if getattr(material, 'baseColorTexture', None) is None:
                entry['color'] = [1., 1., 1., 1.]
            result['status'] = 'textured'
        result['materials'].append(entry)
    indices = getattr(visual, 'face_materials', None)
    if indices is not None:
        indices = np.asarray(indices)
        if (
            indices.shape != (len(mesh.faces),)
            or not np.isfinite(indices).all()
            or np.any(indices != np.floor(indices))
            or np.any(indices < 0)
            or np.any(indices >= len(materials))
        ):
            raise ValueError('模型材质索引无效')
        result['face_materials'] = indices.astype(int).tolist()
    if visual.kind in ('vertex', 'face'):
        colors = np.asarray(
            visual.vertex_colors
            if visual.kind == 'vertex'
            else visual.face_colors
        )
        result[visual.kind + '_colors'] = (colors.astype(float) / 255).tolist()
        result['status'] = 'colored'
        result['materials'] = [{'color': [1., 1., 1., 1.]}]
    return result


@lru_cache(maxsize=8)
def _appearance(path, dependencies):
    import trimesh
    return mesh_appearance(trimesh.load(path, force='mesh', process=False))


def model_appearance(models_root, object_id):
    path = Path(select_object(models_root, object_id)['mesh'])
    # Include sidecars so changing a texture/MTL refreshes the cached payload.
    extensions = {
        '.obj', '.glb', '.ply', '.mtl', '.png', '.jpg', '.jpeg', '.webp', '.bmp'
    }
    dependencies = tuple(
        sorted(
            (str(p), p.stat().st_mtime_ns, p.stat().st_size)
            for p in path.parent.rglob('*')
            if p.is_file() and p.suffix.lower() in extensions
        )
    )
    return _appearance(str(path), dependencies)
