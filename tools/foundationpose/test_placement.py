"""Placement geometry and fault ordering use synthetic meshes and simulated hardware."""
import copy
import json
from pathlib import Path
import tempfile
from types import SimpleNamespace
import unittest
from unittest.mock import Mock, patch

import numpy as np

from grasp_control import GraspController
from grasp_geometry import controller_pose
from placement_geometry import checked_rotation, placement_settings, placement_waypoints, plan_placement, endpoint_reachability
import test_grasp

ROOT=Path(__file__).resolve().parents[2]


def fixture():
    config=test_grasp.GraspTests().config()
    config.update(workspace_reviewed=True,workspace_min_m=[-1,-1,0],workspace_max_m=[1,1,1],
                  max_target_distance_m=2,table_z_m=0,min_table_clearance_m=.005)
    settings=json.loads((ROOT/'config/grasp/placement.json').read_text())
    original=np.eye(4);original[:3,3]=[.3,.2,.03]
    # Deliberately use an offset model origin and a nontrivial tool orientation.
    attachment=np.eye(4);attachment[:3,3]=[.02,0,.04];attachment[:3,:3]=np.diag([1,-1,-1])
    held={'hold_id':'hold1','object_id':'sample','object_name':'测试物品','model_sha256':'mesh1',
          'base_T_object_at_grasp':original.tolist(),'object_T_tcp':attachment.tolist(),
          'tool_response':{},'contact_position':500}
    vertices=np.array([[x,y,z] for x in [-.02,.08] for y in [-.02,.02] for z in [-.03,.05]])
    arm={'pose':[300000,200000,400000,3142,0,0],'joint':[0]*7,'err':[0]}
    return config,settings,held,vertices,arm


class PlacementTests(unittest.TestCase):
    def test_rotation_and_settings_reject_invalid_inputs(self):
        for rotation in ([1,0,0],np.zeros((3,3)),np.diag([1,1,-1]),np.full((3,3),np.nan)):
            with self.subTest(rotation=rotation),self.assertRaises(ValueError):checked_rotation(rotation)
        _,settings,_,_,_=fixture()
        for key,value in [('target_xy_m',[.3]),('target_xy_m',[.3,float('nan')]),('target_xy_m',[True,-.3]),('arm_speed_percent',True),('release_clearance_m',float('nan'))]:
            with self.assertRaises(ValueError):placement_settings({**settings,key:value})

    def test_fixed_location_does_not_depend_on_original_grasp_position(self):
        config,settings,held,vertices,arm=fixture()
        for original_xy in [[.1,.2],[-.5,.4],[.8,-.1]]:
            original=np.array(held['base_T_object_at_grasp']);original[:2,3]=original_xy
            held['base_T_object_at_grasp']=original.tolist()
            plan=placement_waypoints(held,vertices,np.eye(3),arm['pose'],config,settings)
            np.testing.assert_allclose(np.array(plan['base_T_object_target'])[:2,3],[.3,-.3])
            self.assertEqual(plan['target_xy_m'],[.3,-.3])
            self.assertNotIn('offset_m',plan)

    def test_rotated_mesh_bottom_not_origin_sets_height_and_attachment(self):
        config,settings,held,vertices,arm=fixture()
        # Rotate around vertical; attachment includes translation AND rotation.
        rotation=np.array([[0,-1,0],[1,0,0],[0,0,1]])
        plan=placement_waypoints(held,vertices,rotation,arm['pose'],config,settings)
        target=np.array(plan['base_T_object_target'])
        np.testing.assert_allclose(target[:3,:3],rotation)
        np.testing.assert_allclose(target[:2,3],[.3,-.3])
        world=vertices@target[:3,:3].T+target[:3,3]
        self.assertAlmostEqual(world[:,2].min(),settings['release_clearance_m'])
        expected=target@held['object_T_tcp']
        actual=controller_pose(plan['waypoints'][2]['controller_pose'])
        np.testing.assert_allclose(actual[:3,3],expected[:3,3],atol=1e-6)
        np.testing.assert_allclose(actual[:3,:3],expected[:3,:3],atol=.001)
        self.assertGreater(plan['waypoints'][-1]['position_m'][2],plan['waypoints'][2]['position_m'][2])

    def test_fixed_location_preserves_requested_rotation(self):
        config,settings,held,vertices,arm=fixture();reachable=Mock(return_value=(True,{'checked':4}))
        rotation=np.array([[0,-1,0],[1,0,0],[0,0,1]])
        plan=plan_placement(held,vertices,rotation,arm,config,settings,reachable,lambda:None)
        reachable.assert_called_once()
        self.assertEqual(plan['reachability_check'],{'checked':4})
        np.testing.assert_allclose(np.array(plan['base_T_object_target'])[:3,:3],rotation)
        np.testing.assert_allclose(plan['target_xy_m'],[.3,-.3])

    def test_unreachable_fixed_location_never_retries_or_sends_motion(self):
        config,settings,held,vertices,arm=fixture();reachable=Mock(return_value=(False,{'reason':'no solution'}))
        with self.assertRaisesRegex(ValueError,'固定放置点不可达'):
            plan_placement(held,vertices,np.eye(3),arm,config,settings,reachable,lambda:None)
        reachable.assert_called_once()
        reachable.reset_mock()
        with self.assertRaisesRegex(RuntimeError,'cancel'):
            plan_placement(held,vertices,np.eye(3),arm,config,settings,reachable,Mock(side_effect=RuntimeError('cancel')))
        reachable.assert_not_called()
        reachable.return_value=(True,{})
        with self.assertRaisesRegex(RuntimeError,'cancel'):
            plan_placement(held,vertices,np.eye(3),arm,config,settings,reachable,Mock(side_effect=[None,RuntimeError('cancel')]))

    def test_workspace_and_table_reject_before_checker(self):
        config,settings,held,vertices,arm=fixture();reachable=Mock(return_value=(True,{}))
        config['workspace_max_m'][2]=.3
        with self.assertRaisesRegex(ValueError,'工作区'):plan_placement(held,vertices,np.eye(3),arm,config,settings,reachable,lambda:None)
        reachable.assert_not_called()
        config['workspace_reviewed']=False
        with self.assertRaisesRegex(ValueError,'核对'):plan_placement(held,vertices,np.eye(3),arm,config,settings,reachable,lambda:None)

    def test_checker_requires_every_endpoint_no_collision_and_success(self):
        _,_,_,_,arm=fixture();tool={'flange_T_grip_tcp':np.eye(4).tolist()};points=[{'controller_pose':arm['pose']}]*2
        for output,code,valid in [('IK 0 0 0 0 0 0 0 0 0\n'*2,0,True),('IK 0 0 0 0 0 0 0 0 0\n',0,False),('IK 0 1 0 0 0 0 0 0 0\n'*2,0,False),('IK 0 0 nan 0 0 0 0 0 0\n'*2,0,False),('',1,False)]:
            with patch('placement_geometry.subprocess.run',return_value=SimpleNamespace(returncode=code,stdout=output,stderr='')) as run:
                self.assertEqual(endpoint_reachability('offline',tool,arm,points,1)[0],valid)
                self.assertEqual(run.call_args.args[0],['offline'])

    def test_old_or_absent_hold_and_busy_are_rejected_before_thread(self):
        config,_,held,_,_=fixture()
        with tempfile.TemporaryDirectory() as d:
            root=Path(d);cfg=root/'trial.json';cfg.write_text(json.dumps(config));service=GraspController(root,None,cfg)
            request={'rotation':np.eye(3).tolist(),**{k:held[k] for k in ['object_id','model_sha256','hold_id']}}
            with patch('placement_control.threading.Thread') as thread:
                with self.assertRaisesRegex(ValueError,'尚无'):service.start_place(request)
                service.held_object=held
                for key in ['object_id','model_sha256','hold_id']:
                    with self.assertRaisesRegex(ValueError,'变化'):service.start_place({**request,key:'old'})
                service.status['busy']=True
                with self.assertRaisesRegex(ValueError,'正在处理'):service.start_place(request)
                thread.assert_not_called()

    def test_attachment_uses_actual_contact_pose_and_only_publishes_on_success(self):
        config,_,held,_,arm=fixture()
        with tempfile.TemporaryDirectory() as d:
            root=Path(d);cfg=root/'trial.json';cfg.write_text(json.dumps(config));service=GraspController(root,None,cfg)
            service.status['gripper_contact']={'pos':[500]};service.arm_state=Mock(return_value=arm)
            report={};plan={**held,'base_T_object':held['base_T_object_at_grasp']}
            service.capture_attachment(plan,report);self.assertIsNone(service.held_object)
            expected=np.linalg.inv(plan['base_T_object'])@controller_pose(arm['pose'])
            np.testing.assert_allclose(report['held_object']['object_T_tcp'],expected)
            service.finish_attachment(report);self.assertEqual(service.held_snapshot()['object_id'],'sample')
            report['held_object']['object_id']='mutated';self.assertEqual(service.held_object['object_id'],'sample')

    def test_release_only_after_arrival_and_retreat_only_after_open_completion(self):
        for failure in [None,'motion','arrival','open','open_timeout','cancel','contact']:
            with self.subTest(failure=failure),tempfile.TemporaryDirectory() as d:
                config,settings,held,vertices,arm=fixture();root=Path(d);cfg=root/'trial.json';cfg.write_text(json.dumps(config))
                service=GraspController(root,None,cfg);service.held_object=held
                plan=placement_waypoints(held,vertices,np.eye(3),arm['pose'],config,settings)
                robot=service.robot=Mock();events=[];released=[False]
                contact={'sys_err':0,'sys_state':0,'dof_err':[0],'dof_state':[3],'pos':[500],'speed':[0],'force':[10]}
                def query(command):
                    state=copy.deepcopy(contact)
                    if released[0] and failure!='open_timeout':state.update(pos=[1000],force=[0],dof_state=[2])
                    if failure=='contact':state['pos']=[0]
                    return {'state_info':state}
                robot.query.side_effect=query
                def move(pose,*args):
                    events.append('move')
                    if len(events)==3 and failure=='motion':raise RuntimeError('movement failed')
                    arm['pose']=pose.copy()
                    if len(events)==3 and failure=='arrival':arm['pose'][0]+=10000
                    if len(events)==3 and failure=='cancel':service.cancelled.set()
                robot.move_linear.side_effect=robot.move_joint_pose.side_effect=move
                def release(position):
                    events.append('open')
                    if failure=='open':raise RuntimeError('release failed')
                    released[0]=True
                robot.move_gripper.side_effect=release;service.arm_state=Mock(side_effect=lambda:copy.deepcopy(arm))
                report={}
                if failure:
                    with patch('placement_control.time.monotonic',side_effect=[0,100]),self.assertRaises((ValueError,RuntimeError,TimeoutError)):
                        service.execute_place(plan,held,config,settings,report)
                    if failure not in ('open','open_timeout'):robot.move_gripper.assert_not_called()
                else:
                    service.execute_place(plan,held,config,settings,report)
                    self.assertEqual(events,['move','move','move','open','move'])
                    self.assertTrue(report['release_confirmed']);self.assertIsNone(service.held_object)
                    self.assertEqual(service.status['state'],'completed')
                if failure in ('open','open_timeout'):self.assertEqual(events,['move','move','move','open']);self.assertIsNone(service.held_object)

    def test_preview_has_no_writes_and_execution_failure_clears_hold_and_stops(self):
        for mode in ['preview','execute']:
            with self.subTest(mode=mode),tempfile.TemporaryDirectory() as d:
                config,settings,held,vertices,arm=fixture();root=Path(d);(root/'config/grasp').mkdir(parents=True)
                config.update(robot_ip='simulation',robot_port=1,ik_checker='checker')
                cfg=root/'trial.json';cfg.write_text(json.dumps(config));(root/'config/grasp/placement.json').write_text(json.dumps(settings));(root/'checker').touch()
                service=GraspController(root,SimpleNamespace(models_root=root),cfg);service.held_object=held
                service.placement_hardware=Mock(return_value=(arm,{}));robot=Mock();service.arm_state=Mock(return_value=arm);service.check_place_contact=Mock()
                def fail(*args):service.actuated=True;raise RuntimeError('simulated controller rejection')
                service.execute_place=Mock(side_effect=fail)
                with patch('placement_control.RobotClient',return_value=robot) as client,patch('placement_control.model_geometry',return_value={'model_sha256':'mesh1','vertices':vertices}),patch('placement_control.endpoint_reachability',return_value=(True,{})):
                    service.run_place(held,np.eye(3).tolist(),preview=mode=='preview')
                    self.assertEqual(client.call_args.kwargs['writes'],mode=='execute')
                self.assertFalse(service.status['busy'])
                if mode=='preview':
                    service.execute_place.assert_not_called();robot.stop.assert_not_called();self.assertEqual(service.held_object,held);self.assertEqual(service.status['state'],'place_preview')
                else:
                    robot.stop.assert_called_once();self.assertIsNone(service.held_object);self.assertEqual(service.status['state'],'failed')
                report=json.loads(next((root/'work/grasp/place_jobs').glob('*.json')).read_text())
                self.assertFalse(report['robot_motion_sent'])
                if mode=='execute':self.assertIn('traceback',report)


if __name__=='__main__':unittest.main()
