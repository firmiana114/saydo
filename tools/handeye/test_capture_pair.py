import argparse
import base64
import copy
import json
from pathlib import Path
import tempfile
import threading
import time
import unittest
from unittest.mock import Mock,patch

import cv2
import numpy as np
import yaml

import capture_pair as capture
from geometry import object_points,pose_matrix

K=np.array([[900.,0,640],[0,900.,360],[0,0,1]])
SERIALS={'d435i':'435123456789','d405':'405123456789'}


def frame(key, sequence=1, **changes):
    size=(1280,720)
    ok, encoded=cv2.imencode('.png',np.zeros((size[1],size[0],3),np.uint8))
    assert ok
    camera={'serial':SERIALS[key],'name':'RealSense D435I' if key=='d435i' else 'RealSense D405',
            'frame':'color_optical_frame','image_size':list(size),'native_size':list(size),'K':K.tolist(),'distortion_coefficients':[0.]*5}
    return {'camera':camera,'camera_serial':camera['serial'],'camera_name':camera['name'],
            'camera_role':'eye_in_hand' if key=='d435i' else 'fixed_global','captured_at':time.time(),
            'sequence':sequence,'image_size':list(size),'mime_type':'image/png',
            'image_base64':base64.b64encode(encoded).decode(),'lossless_source':True,**changes}


def corners():
    return cv2.projectPoints(object_points(),np.array([.2,-.1,.1]),np.array([-.105,-.15,1.]),K,np.zeros(5))[0]


class GeometryTests(unittest.TestCase):
    def test_nonzero_tcp_offset_is_removed_with_full_rotation(self):
        state={'pose':[200000,-100000,300000,300,-200,1500]}
        tool={'tool_name':'grip_tcp','pose':[1000,2000,222500,0,0,2611]}
        result=capture.flange_pose(state,tool,{'frame_name':'Base','pose':[0]*6})
        np.testing.assert_allclose(result@pose_matrix(tool['pose']),pose_matrix(state['pose']),atol=1e-12)
        self.assertGreater(np.linalg.norm(result-pose_matrix(state['pose'])),.1)

    def test_nonbase_and_missing_tool_rejected(self):
        for tool,work in [({'tool_name':'tool','pose':[0]*6},{'frame_name':'other','pose':[0]*6}),
                          ({},{'frame_name':'Base','pose':[0]*6})]:
            with self.assertRaises(ValueError):capture.flange_pose({'pose':[0]*6},tool,work)

    def test_frame_identity_age_intrinsics_and_encoding_checked(self):
        for changes in [{'captured_at':time.time()-2},{'camera_serial':'other'},
                        {'camera_role':'fixed_global'},{'image_size':[640,360]},
                        {'camera':{**frame('d435i')['camera'],'distortion_coefficients':[.1]*5}},
                        {'lossless_source':False},
                        {'image_base64':'bad'}]:
            with self.subTest(changes=changes),self.assertRaises(ValueError):
                capture.decode_frame(frame('d435i',**changes),'d435i',SERIALS['d435i'])
        self.assertEqual(capture.decode_frame(frame('d405'),'d405',SERIALS['d405'])[1],'.png')

    def test_pose_candidates_preserved_and_reverse_is_explicit(self):
        image=np.zeros((720,1280,3),np.uint8);cam=frame('d405')['camera'];board={'inner_corners':[8,11],'square_size_m':.03}
        with patch.object(capture.cv2,'findChessboardCornersSB',return_value=(True,corners())):
            normal,_=capture.analyze_board(image,cam,board)
            reversed_result,_=capture.analyze_board(image,cam,board,True)
        self.assertGreaterEqual(len(normal['board_pose_candidates']),2)
        self.assertLess(normal['reprojection_rms_px'],1e-4)
        np.testing.assert_allclose(reversed_result['corners_px'],normal['corners_px'][::-1])

    def test_missing_corners_and_wrong_transport_url_rejected(self):
        with patch.object(capture.cv2,'findChessboardCornersSB',return_value=(False,None)),self.assertRaises(ValueError):
            capture.analyze_board(np.zeros((720,1280,3),np.uint8),frame('d405')['camera'],{'inner_corners':[8,11],'square_size_m':.03})
        for url in ['http://user:pass@host','http://host/path','http://host?query=1']:
            with self.assertRaises(ValueError):capture.Monitor(url)
        with self.assertRaises(ValueError):capture.Monitor('http://localhost:8088').get('/api/pause')
        with patch('capture.socket.create_connection') as connect,self.assertRaises(ValueError):
            capture.Robot('localhost').query('movel')
        connect.assert_not_called()


class WorkflowTests(unittest.TestCase):
    def setUp(self):
        self.tmp=tempfile.TemporaryDirectory();self.addCleanup(self.tmp.cleanup);self.root=Path(self.tmp.name)
        (self.root/'config/cameras').mkdir(parents=True);(self.root/'config/calibration').mkdir()
        original=self.root/'result.json';original.write_text('{}')
        settings={'serial':SERIALS['d405'],'eye_in_hand_serial':SERIALS['d435i'],'camera_name':'RealSense D405'}
        (self.root/'config/cameras/global.json').write_text(json.dumps(settings))
        handeye={'acceptance_status':'user_accepted_usable','source':{'path':'result.json','sha256':capture.digest(original)},
                 'camera':frame('d435i')['camera'],'flange_T_color_optical_camera':np.eye(4).tolist()}
        (self.root/'config/calibration/handeye.yaml').write_text(yaml.safe_dump(handeye))
        self.args=argparse.Namespace(output_root=self.root/'work/dual_camera_calibration',session='test',role='training',
                                     columns=8,rows=11,square_mm=30.,robot_ip='localhost',monitor_url='http://localhost:8088',d405_reverse_corners=False)
        self.leases=[Mock(),Mock()];self.monitor=Mock();self.monitor.leases.return_value=self.leases
        self.monitor.get.return_value={'cameras':{key:{'state':'streaming'} for key in capture.IDS}}
        self.sequence=0
        def pair():
            self.sequence+=1
            return {key:frame(key,self.sequence) for key in capture.IDS}
        self.monitor.pair.side_effect=pair
        self.robot=Mock();self.robot.query.side_effect=self.query
        self.state={'err':[0],'joint':[0]*7,'pose':[200000,0,300000,0,0,0]}
        self.tool={'tool_name':'grip_tcp','pose':[0,0,222500,0,0,2611]}
        self.work={'frame_name':'Base','pose':[0]*6}
        self.sleeper=patch.object(capture.time,'sleep');self.sleeper.start();self.addCleanup(self.sleeper.stop)
        self.finder=patch.object(capture.cv2,'findChessboardCornersSB',return_value=(True,corners()));self.finder.start();self.addCleanup(self.finder.stop)

    def query(self,command):
        replies={'get_current_arm_state':{'arm_state':copy.deepcopy(self.state)},'get_current_tool_frame':self.tool,
                 'get_current_work_frame':self.work,'get_arm_software_info':{'Product_version':'RM75-BI'},
                 'get_joint_en_state':{'en_state':[1]*7},'get_joint_err_flag':{'err_flag':[0]*7}}
        return {'request_monotonic':time.monotonic(),'received_monotonic':time.monotonic(),
                'received_unix':time.time(),'reply':copy.deepcopy(replies[command])}

    def run_capture(self):return capture.capture_once(self.args,self.root,self.robot,self.monitor)

    def test_one_invocation_saves_one_group_and_releases_only_its_viewers(self):
        out,report=self.run_capture()
        self.assertEqual(report['status'],'candidate',report.get('error'))
        self.assertEqual(len(report['images']),4)
        for name in ['d435i.png','d435i_check.png','d405.png','d405_check.png','d435i_corners.jpg','d405_corners.jpg','sample.json']:
            self.assertTrue((out/name).is_file())
        self.assertFalse(report['motion_sent']);self.assertFalse(report['calibration_solved']);self.assertFalse(report['usable_for_execution'])
        np.testing.assert_allclose(np.asarray(report['base_T_flange'])@np.asarray(report['flange_T_tcp']),report['base_T_tcp'],atol=1e-12)
        self.monitor.pair.assert_has_calls([unittest.mock.call(),unittest.mock.call()])
        for lease in self.leases:lease.start.assert_called_once();lease.close.assert_called_once()
        self.assertEqual({call.args[0] for call in self.robot.query.call_args_list},{
            'get_current_arm_state','get_current_tool_frame','get_current_work_frame','get_arm_software_info','get_joint_en_state','get_joint_err_flag'})
        self.assertEqual(self.monitor.get.call_args.args[0],'/api/status')

    def test_arm_can_change_between_groups_and_files_are_not_overwritten(self):
        first,first_report=self.run_capture();self.state['pose'][0]+=50000;self.state['joint'][0]+=10000
        self.args.role='validation'
        second,second_report=self.run_capture()
        self.assertNotEqual(first,second);self.assertEqual(second_report['status'],'candidate',second_report.get('error'))
        self.assertEqual(second_report['role'],'validation')
        self.assertAlmostEqual(second_report['base_T_flange'][0][3]-first_report['base_T_flange'][0][3],.05)

    def test_board_movement_rejects_but_preserves_images(self):
        self.finder.stop()
        with patch.object(capture.cv2,'findChessboardCornersSB',side_effect=[(True,corners()),(True,corners()),(True,corners()+2),(True,corners()+2)]):
            out,report=self.run_capture()
        self.assertEqual(report['status'],'rejected');self.assertIn('角点移动',report['error'])
        self.assertTrue((out/'d405_check.png').exists());self.assertTrue((out/'sample.json').exists())

    def test_motion_and_tool_change_rejected(self):
        self.monitor.pair.side_effect=lambda: (setattr(self,'state',{**self.state,'joint':[100]+[0]*6}) or {key:frame(key,1) for key in capture.IDS})
        _,report=self.run_capture();self.assertIn('not stationary',report['error'])
        for lease in self.leases:lease.close.assert_called_once()

    def test_duplicate_frame_and_pair_skew_rejected(self):
        for scenario in ['duplicate','skew']:
            self.monitor.pair.side_effect=lambda:{key:frame(key,1,captured_at=time.time()-(.3 if scenario=='skew' and key=='d435i' else 0)) for key in capture.IDS}
            _,report=self.run_capture()
            self.assertEqual(report['status'],'rejected');self.assertIn('重复帧' if scenario=='duplicate' else '时间差',report['error'])

    def test_manifest_mismatch_and_missing_corners_do_not_overwrite_success(self):
        first,report=self.run_capture();self.assertEqual(report['status'],'candidate')
        manifest=self.root/'work/dual_camera_calibration/test/session.json';original=manifest.read_bytes()
        self.args.d405_reverse_corners=True
        _,report=self.run_capture();self.assertIn('已有不同',report['error']);self.assertEqual(manifest.read_bytes(),original)
        self.finder.stop()
        with patch.object(capture.cv2,'findChessboardCornersSB',return_value=(False,None)):
            out,report=self.run_capture()
        self.assertIn('未找到',report['error']);self.assertTrue((out/'d405.png').exists());self.assertTrue(first.exists())

    def test_bad_handeye_fails_before_camera_start(self):
        (self.root/'result.json').write_text('changed')
        _,report=self.run_capture();self.assertIn('摘要',report['error']);self.monitor.leases.assert_not_called()

    def test_tool_configuration_change_during_group_is_rejected(self):
        calls=0
        def query(command):
            nonlocal calls
            result=self.query(command)
            if command=='get_current_tool_frame':
                calls+=1
                if calls>1:result['reply']['pose'][2]+=1000
            return result
        self.robot.query.side_effect=query
        _,report=self.run_capture()
        self.assertEqual(report['status'],'rejected');self.assertIn('工具或工作坐标系发生变化',report['error'])


class NativeMonitorTests(unittest.TestCase):
    def test_png_preserves_pixels_intrinsics_and_capture_timestamp(self):
        from tools.camera_monitor.server import Camera
        camera=Camera(1280,720,15,'d405',SERIALS['d405'])
        raw=np.random.default_rng(3).integers(0,255,size=(720,1280,3),dtype=np.uint8)
        ok,jpeg=cv2.imencode('.jpg',raw);self.assertTrue(ok)
        meta={'camera':frame('d405')['camera'],'sensor_timestamp_ms':123.}
        camera.publish(jpeg.tobytes(),time.time(),1,meta,'device',1280,720,raw)
        def next_frame():
            time.sleep(.03);camera.publish(jpeg.tobytes(),time.time(),2,meta,'device',1280,720,raw)
        worker=threading.Thread(target=next_frame);worker.start()
        result=camera.fresh_frame(calibration=True);worker.join()
        decoded=cv2.imdecode(np.frombuffer(base64.b64decode(result['image_base64']),np.uint8),1)
        np.testing.assert_array_equal(decoded,raw)
        self.assertEqual(result['camera']['K'],K.tolist());self.assertTrue(result['lossless_source'])
        self.assertEqual(result['sequence'],2);self.assertEqual(result['sensor_timestamp_ms'],123.)


if __name__=='__main__':unittest.main()
