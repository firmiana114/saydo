"""Fixed-TCP rotation geometry and control, without connecting to hardware."""
import copy
import json
from pathlib import Path
import tempfile
from types import SimpleNamespace
import unittest
from unittest.mock import Mock, patch
import numpy as np
from scipy.spatial.transform import Rotation
from grasp_control import GraspController
from grasp_geometry import controller_pose
from reorientation_geometry import checked_rotation, reorientation_settings, reorientation_waypoints, plan_reorientation, endpoint_reachability
import test_grasp
ROOT=Path(__file__).resolve().parents[2]


def fixture():
    config=test_grasp.GraspTests().config()
    config.update(workspace_reviewed=True,workspace_min_m=[-1,-1,0],workspace_max_m=[1,1,1],table_z_m=0,min_table_clearance_m=.005)
    settings=json.loads((ROOT/'config/grasp/reorientation.json').read_text())
    original=np.eye(4);original[:3,3]=[.3,.2,.03]
    attachment=np.eye(4);attachment[:3,3]=[.02,0,.04];attachment[:3,:3]=np.diag([1,-1,-1])
    held={'hold_id':'hold1','object_id':'sample','object_name':'测试物品','model_sha256':'mesh1',
          'base_T_object_at_grasp':original.tolist(),'object_T_tcp':attachment.tolist(),'tool_response':{},'contact_position':500}
    vertices=np.array([[x,y,z] for x in [-.02,.08] for y in [-.02,.02] for z in [-.03,.05]])
    arm={'pose':[300001,200003,400007,3142,0,0],'joint':[0]*7,'err':[0]}
    return config,settings,held,vertices,arm


class ReorientationTests(unittest.TestCase):
    def test_invalid_rotation_and_settings(self):
        for r in ([1,0,0],np.zeros((3,3)),np.diag([1,1,-1]),np.full((3,3),np.nan)):
            with self.assertRaises(ValueError):checked_rotation(r)
        _,settings,_,_,_=fixture()
        for key,value in [('arm_speed_percent',True),('position_tolerance_m',float('nan')),('planning_timeout_s',0)]:
            with self.assertRaises(ValueError):reorientation_settings({**settings,key:value})

    def test_every_sample_and_target_keep_exact_raw_xyz(self):
        config,settings,held,vertices,arm=fixture();config['max_target_distance_m']=.000001
        for degrees in [[0,0,0],[25,65,120],[0,180,0]]:
            rotation=Rotation.from_euler('xyz',degrees,degrees=True).as_matrix()
            plan=reorientation_waypoints(held,vertices,rotation,arm['pose'],config,settings)
            for w in plan['waypoints']:self.assertEqual(w['controller_pose'][:3],arm['pose'][:3])
            self.assertEqual(len(plan['waypoints']),1)
            object_pose=np.array(plan['base_T_object_target'])
            np.testing.assert_allclose(object_pose[:3,:3],rotation,atol=1e-12)
            tcp=object_pose@held['object_T_tcp'];np.testing.assert_allclose(tcp[:3,3],np.array(arm['pose'][:3])/1e6,atol=1e-12)
            self.assertGreater(np.linalg.norm(object_pose[:3,3]-tcp[:3,3]),.02)
            self.assertEqual(plan['gripper_action'],'keep_holding')
            self.assertNotIn('target_xy_m',plan)

    def test_controller_joint_path_is_not_falsely_checked_as_cartesian_rotation(self):
        config,settings,held,_,arm=fixture();held['object_T_tcp']=np.eye(4).tolist();arm['pose']=[300000,200000,80000,0,0,0]
        vertices=np.array([[0,0,0],[.12,0,0],[0,.01,0]])
        target=Rotation.from_euler('y',180,degrees=True).as_matrix();checker=Mock(return_value=(True,{}))
        plan=plan_reorientation(held,vertices,target,arm,config,settings,checker,lambda:None)
        self.assertFalse(plan['intermediate_path_checked'])
        self.assertEqual(len(checker.call_args.args[0]),1)
        self.assertEqual(plan['waypoints'][0]['method'],'joint_pose')

    def test_reachability_and_cancellation(self):
        config,settings,held,vertices,arm=fixture();checker=Mock(return_value=(True,{'ok':True}))
        plan=plan_reorientation(held,vertices,np.eye(3),arm,config,settings,checker,lambda:None)
        self.assertEqual(checker.call_args.args[0],plan['validation_points'])
        checker.return_value=(False,{})
        with self.assertRaisesRegex(ValueError,'不可达'):plan_reorientation(held,vertices,np.eye(3),arm,config,settings,checker,lambda:None)
        checker.reset_mock()
        with self.assertRaisesRegex(RuntimeError,'cancel'):plan_reorientation(held,vertices,np.eye(3),arm,config,settings,checker,Mock(side_effect=RuntimeError('cancel')))
        checker.assert_not_called()

    def test_workspace_and_bad_config(self):
        config,settings,held,vertices,arm=fixture();config['workspace_max_m'][2]=.3
        with self.assertRaisesRegex(ValueError,'工作区'):plan_reorientation(held,vertices,np.eye(3),arm,config,settings,Mock(),lambda:None)
        config['workspace_reviewed']=False
        with self.assertRaisesRegex(ValueError,'核对'):plan_reorientation(held,vertices,np.eye(3),arm,config,settings,Mock(),lambda:None)

    def test_checker_requires_all_samples_successful(self):
        _,_,_,_,arm=fixture();tool={'flange_T_grip_tcp':np.eye(4).tolist()};points=[{'controller_pose':arm['pose']}]*2
        for output,code,valid in [('IK 0 0 0 0 0 0 0 0 0\n'*2,0,False),('IK 0 0 0 0 0 0 0 0 0\n',0,False),('IK 0 1 0 0 0 0 0 0 0\n'*2,0,False),('IK 0 0 nan 0 0 0 0 0 0\n'*2,0,False),('',1,False)]:
            with patch('reorientation_geometry.subprocess.run',return_value=SimpleNamespace(returncode=code,stdout=output,stderr='')):
                self.assertEqual(endpoint_reachability('offline',tool,arm,points,1)[0],valid)

    def test_no_hold_stale_request_and_busy_rejected(self):
        config,_,held,_,_=fixture()
        with tempfile.TemporaryDirectory() as d:
            root=Path(d);cfg=root/'trial.json';cfg.write_text(json.dumps(config));service=GraspController(root,None,cfg)
            request={'rotation':np.eye(3).tolist(),**{k:held[k] for k in ['object_id','model_sha256','hold_id']}}
            with patch('reorientation_control.threading.Thread') as thread:
                with self.assertRaisesRegex(ValueError,'尚无'):service.start_reorient(request)
                service.held_object=held
                with self.assertRaisesRegex(ValueError,'刷新页面'):service.start_reorient({**request,'max_shift_m':.1})
                for key in ['object_id','model_sha256','hold_id']:
                    with self.assertRaisesRegex(ValueError,'变化'):service.start_reorient({**request,key:'old'})
                service.status['busy']=True
                with self.assertRaisesRegex(ValueError,'正在处理'):service.start_reorient(request)
                thread.assert_not_called()

    def test_attachment_and_current_rotation_are_kept_after_grasp_return(self):
        config,_,held,_,arm=fixture()
        with tempfile.TemporaryDirectory() as d:
            root=Path(d);cfg=root/'trial.json';cfg.write_text(json.dumps(config));service=GraspController(root,None,cfg)
            service.status['gripper_contact']={'pos':[500]};service.arm_state=Mock(return_value=arm)
            report={};service.capture_attachment({**held,'base_T_object':held['base_T_object_at_grasp']},report)
            self.assertIsNone(service.held_object)
            arm['pose'][3:]=[0,0,0];service.finish_attachment(report)
            expected=controller_pose(arm['pose'])@np.linalg.inv(report['held_object']['object_T_tcp'])
            np.testing.assert_allclose(service.held_snapshot()['current_object_rotation'],expected[:3,:3])
            self.assertIn('object_T_tcp',service.held_snapshot())

    def test_model_preview_uses_held_tcp_pivot_and_current_object_rotation(self):
        config,settings,held,_,_=fixture();held['current_object_rotation']=np.eye(3).tolist()
        with tempfile.TemporaryDirectory() as d:
            root=Path(d);cfg=root/'trial.json';cfg.write_text(json.dumps(config));service=GraspController(root,SimpleNamespace(models_root=root),cfg)
            service.held_object=held
            (root/'config/grasp').mkdir(parents=True);(root/'config/grasp/reorientation.json').write_text(json.dumps(settings))
            with patch('reorientation_control.model_geometry',return_value={}),patch('reorientation_control.model_appearance',return_value={}):
                model=service.reorientation_model('sample')
            self.assertEqual(model['pivot_object_m'],[.02,0,.04])
            self.assertEqual(model['initial_rotation'],held['current_object_rotation'])
            self.assertEqual(model['preview_mode'],'adaptive_or_fixed_orientation');self.assertEqual(model['minimum_final_clearance_m'],.10);self.assertNotIn('max_shift_m',model)
            self.assertNotIn('release_clearance_m',model);self.assertNotIn('target_xy_m',model)

    def test_only_cartesian_rotation_no_gripper_writes_and_hold_retained(self):
        for failure in [None,'motion','drift','orientation','cancel','contact']:
            with self.subTest(failure=failure),tempfile.TemporaryDirectory() as d:
                config,settings,held,vertices,arm=fixture();root=Path(d);cfg=root/'trial.json';cfg.write_text(json.dumps(config))
                service=GraspController(root,None,cfg);service.held_object=held
                plan=reorientation_waypoints(held,vertices,np.eye(3),arm['pose'],config,settings)
                robot=service.robot=Mock();service.check_reorient_contact=Mock();service.verify_motion_targets=Mock()
                service.arm_state=Mock(side_effect=lambda:copy.deepcopy(arm))
                def move(pose,speed,cancelled,watchdog):
                    if failure=='motion':raise RuntimeError('motion')
                    arm['pose']=pose.copy()
                    if failure=='drift':arm['pose'][0]+=10000
                    if failure=='orientation':arm['pose'][5]+=300
                    if failure=='cancel':service.cancelled.set()
                    if failure=='contact':service.check_reorient_contact.side_effect=ValueError('contact')
                    watchdog()
                robot.move_joint_pose.side_effect=move;report={}
                if failure:
                    with self.assertRaises((ValueError,RuntimeError)):service.execute_reorient(plan,held,config,settings,report,vertices)
                else:
                    service.execute_reorient(plan,held,config,settings,report,vertices)
                    robot.move_joint_pose.assert_called_once()
                    self.assertEqual(service.held_object['hold_id'],held['hold_id'])
                    self.assertEqual(service.held_object['object_T_tcp'],held['object_T_tcp'])
                    self.assertEqual(service.held_object['orientation_version'],1)
                    self.assertEqual(service.status['state'],'completed')
                robot.move_gripper.assert_not_called();robot.set_register.assert_not_called();robot.move_linear.assert_not_called();robot.move_joints.assert_not_called()

    def test_preview_never_moves_and_motion_failure_stops(self):
        for mode in ['preview','execute']:
            with self.subTest(mode=mode),tempfile.TemporaryDirectory() as d:
                config,settings,held,vertices,arm=fixture();root=Path(d);(root/'config/grasp').mkdir(parents=True)
                config.update(robot_ip='simulation',robot_port=1,ik_checker='checker')
                cfg=root/'trial.json';cfg.write_text(json.dumps(config));(root/'config/grasp/reorientation.json').write_text(json.dumps(settings));(root/'checker').touch()
                service=GraspController(root,SimpleNamespace(models_root=root),cfg);service.held_object=held
                service.reorientation_hardware=Mock(return_value=(arm,{'flange_T_grip_tcp':np.eye(4).tolist()}));robot=Mock();service.arm_state=Mock(return_value=arm);service.check_reorient_contact=Mock()
                def fail(*args):service.actuated=True;raise RuntimeError('simulated rejection')
                service.execute_reorient=Mock(side_effect=fail)
                with patch('reorientation_control.RobotClient',return_value=robot) as client,patch('reorientation_control.model_geometry',return_value={'model_sha256':'mesh1','vertices':vertices}),patch('reorientation_control.endpoint_reachability',return_value=(True,{})):
                    service.run_reorient(held,np.eye(3).tolist(),preview=mode=='preview')
                    self.assertEqual(client.call_args.kwargs['writes'],mode=='execute')
                self.assertFalse(service.status['busy'])
                if mode=='preview':
                    service.execute_reorient.assert_not_called();robot.stop.assert_not_called();self.assertEqual(service.held_object,held);self.assertEqual(service.status['state'],'reorient_preview')
                else:robot.stop.assert_called_once();self.assertIsNone(service.held_object);self.assertEqual(service.status['state'],'failed')
                report=json.loads(next((root/'work/grasp/reorient_jobs').glob('*.json')).read_text())
                self.assertEqual(report['operation'],'reorient');self.assertNotIn('release_attempted',report)


if __name__=='__main__':unittest.main()
