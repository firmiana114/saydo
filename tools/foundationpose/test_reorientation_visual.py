"""Appearance preserves mesh indexing and never enters reorientation motion geometry."""
import base64
from io import BytesIO
from pathlib import Path
from tempfile import TemporaryDirectory
import unittest
from unittest.mock import patch

import numpy as np
from PIL import Image
import trimesh
from trimesh.visual.material import SimpleMaterial, MultiMaterial
from trimesh.visual.texture import TextureVisuals

from reorientation_visual import mesh_appearance, model_appearance


class AppearanceTests(unittest.TestCase):
    def mesh(self):
        mesh = trimesh.Trimesh(vertices=[[0,0,0],[.1,0,0],[0,.1,0]], faces=[[0,1,2]], process=False)
        mesh.visual = TextureVisuals(uv=[[0,0],[1,0],[0,1]], material=SimpleMaterial(image=Image.new('RGB',(2048,1024),'red')))
        return mesh

    def test_texture_payload_preserves_uv_and_geometry(self):
        mesh = self.mesh();vertices=mesh.vertices.copy();faces=mesh.faces.copy()
        result=mesh_appearance(mesh)
        self.assertEqual(result['status'],'textured')
        self.assertEqual(result['uv'],[[0,0],[1,0],[0,1]])
        image=Image.open(BytesIO(base64.b64decode(result['materials'][0]['texture'].split(',')[1])))
        self.assertEqual(image.size,(1024,512));self.assertEqual(image.getpixel((0,0)),(255,0,0,255))
        np.testing.assert_array_equal(mesh.vertices,vertices);np.testing.assert_array_equal(mesh.faces,faces)

    def test_multi_materials_and_face_selection(self):
        mesh=self.mesh();mesh.visual.material=MultiMaterial([SimpleMaterial(image=Image.new('RGB',(2,2),'red')),SimpleMaterial(image=Image.new('RGB',(2,2),'blue'))])
        mesh.visual.face_materials=np.array([1])
        result=mesh_appearance(mesh)
        self.assertEqual(len(result['materials']),2);self.assertEqual(result['face_materials'],[1])
        mesh.visual.face_materials=np.array([2])
        with self.assertRaisesRegex(ValueError,'索引'):mesh_appearance(mesh)

    def test_color_only_mesh(self):
        mesh=self.mesh();mesh.visual=trimesh.visual.ColorVisuals(mesh,vertex_colors=[[255,0,0,255]]*3)
        result=mesh_appearance(mesh)
        self.assertEqual(result['status'],'colored');self.assertEqual(result['vertex_colors'][0],[1,0,0,1])

    def test_invalid_uv_refused(self):
        mesh=self.mesh();mesh.visual.uv=np.array([[float('nan'),0],[0,0],[0,0]])
        with self.assertRaisesRegex(ValueError,'纹理坐标'):mesh_appearance(mesh)

    def test_sidecar_update_invalidates_cache(self):
        with TemporaryDirectory() as directory:
            root=Path(directory);mesh=root/'model.obj';mesh.write_text('placeholder');texture=root/'texture.png';texture.write_bytes(b'a')
            with patch('reorientation_visual.select_object',return_value={'mesh':str(mesh)}),patch('trimesh.load',return_value=self.mesh()) as load:
                model_appearance(root,'item');model_appearance(root,'item');self.assertEqual(load.call_count,1)
                texture.write_bytes(b'changed');model_appearance(root,'item');self.assertEqual(load.call_count,2)


if __name__=='__main__':unittest.main()
