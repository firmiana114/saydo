"""Expanded adaptive position search with only a final TCP height constraint."""
import copy
import itertools
import json
from pathlib import Path
import tempfile
import subprocess
from unittest.mock import Mock,patch
import unittest
import numpy as np
from scipy.spatial.transform import Rotation
from test_reorientation import fixture
from grasp_control import GraspController
from reorientation_geometry import (reorientation_settings, checked_position_request, position_offsets,
    plan_reorientation, adaptive_candidate, segment_distance, endpoint_reachability)

class AdaptiveReorientationTests(unittest.TestCase):
    def test_settings_and_request_mode(self):
        _,settings,_,_,_=fixture();settings=reorientation_settings({**settings,'adaptive_max_shift_m':.001})
        self.assertEqual(checked_position_request('adaptive'),'adaptive');self.assertNotIn('adaptive_max_shift_m',settings)
        for mode in ['other',None,True]:
            with self.assertRaises(ValueError):checked_position_request(mode)
        for k,v in [('search_step_m',0),('max_joint_step_deg',True)]:
            with self.assertRaises(ValueError):reorientation_settings({**settings,k:v})

    def test_candidates_expand_beyond_old_radius_and_allow_downward(self):
        current=np.array([.3,.2,.4]);offsets=list(itertools.islice(position_offsets(current,.102001,.025),1600))
        np.testing.assert_allclose(offsets[0],0)
        self.assertTrue(any(np.linalg.norm(x)>.5 for x in offsets))
        self.assertTrue(any(x[2]<0 for x in offsets))
        self.assertTrue(all(current[2]+x[2]>=.102001-1e-9 for x in offsets))

    def test_fixed_solution_preferred_when_final_height_sufficient(self):
        c,s,h,v,a=fixture();check=Mock(return_value=(True,{'max_joint_step_deg':1}))
        p=plan_reorientation(h,v,np.eye(3),a,c,s,check,lambda:None,'adaptive')
        check.assert_called_once();self.assertEqual(p['offset_distance_m'],0);self.assertEqual(len(p['waypoints']),1)
        self.assertNotIn('max_shift_m',p)

    def test_low_start_is_allowed_and_final_height_has_feedback_margin(self):
        c,s,h,_,a=fixture();a['pose'][2]=50000;h['object_T_tcp']=np.eye(4).tolist()
        v=np.array([[-.001,-.001,-.001],[.001,.001,.001]])
        c.update(workspace_reviewed=False,workspace_min_m=None,workspace_max_m=None,min_table_clearance_m=.20)
        p=plan_reorientation(h,v,np.eye(3),a,c,s,Mock(return_value=(True,{})),lambda:None,'adaptive')
        self.assertGreater(p['target_tcp_position_m'][2],c['table_z_m']+.10+s['position_tolerance_m'])
        self.assertAlmostEqual(p['start_tcp_position_m'][2],.05)
        self.assertEqual([w['name'] for w in p['waypoints']],['translate','reorient'])
        self.assertEqual(p['minimum_final_clearance_m'],.10)

    def test_search_can_exceed_20cm_and_ignore_workspace_and_grasp_distance(self):
        c,s,h,v,a=fixture();c.update(workspace_reviewed=False,workspace_min_m=[0,0,0],workspace_max_m=[.01,.01,.01],max_target_distance_m=.001)
        rotation=Rotation.from_euler('xyz',[40,20,70],degrees=True).as_matrix()
        def check(points,timeout):return (points[-1]['position_m'][0]>.54,{'max_joint_step_deg':2})
        p=plan_reorientation(h,v,rotation,a,c,s,check,lambda:None,'adaptive')
        self.assertGreater(p['offset_distance_m'],.20)
        np.testing.assert_allclose(np.asarray(p['base_T_object_target'])[:3,:3],rotation,atol=1e-12)
        self.assertEqual(p['path_samples'][0]['controller_pose'][:3],a['pose'][:3])

    def test_fixed_mode_still_uses_existing_workspace(self):
        c,s,h,v,a=fixture();c['workspace_reviewed']=False
        with self.assertRaisesRegex(ValueError,'工作区'):plan_reorientation(h,v,np.eye(3),a,c,s,Mock(),lambda:None,'fixed')

    def test_discontinuous_solution_rejected_then_search_continues(self):
        c,s,h,v,a=fixture();check=Mock(side_effect=[(True,{'max_joint_step_deg':45}),(True,{'max_joint_step_deg':3})])
        p=plan_reorientation(h,v,np.eye(3),a,c,s,check,lambda:None,'adaptive')
        self.assertGreater(p['offset_distance_m'],0);self.assertIn('关节变化',p['search_attempts'][0]['reason'])

    def test_cancel_and_time_budget(self):
        c,s,h,v,a=fixture();check=Mock(return_value=(False,{}))
        with self.assertRaisesRegex(RuntimeError,'cancel'):plan_reorientation(h,v,np.eye(3),a,c,s,check,Mock(side_effect=RuntimeError('cancel')),'adaptive')
        check.assert_not_called()
        with patch('reorientation_geometry.time.monotonic',side_effect=[0,21]),self.assertRaisesRegex(TimeoutError,'时间预算'):
            plan_reorientation(h,v,np.eye(3),a,c,s,check,lambda:None,'adaptive')
        check.assert_not_called()

    def test_candidate_timeout_does_not_cancel_remaining_search(self):
        c,s,h,v,a=fixture();check=Mock(side_effect=[subprocess.TimeoutExpired('ik',3),(True,{'max_joint_step_deg':1})])
        p=plan_reorientation(h,v,np.eye(3),a,c,s,check,lambda:None,'adaptive')
        self.assertGreater(p['offset_distance_m'],0);self.assertIn('超时',p['search_attempts'][0]['reason'])

    def test_final_10cm_boundary_and_object_collision(self):
        c,s,h,_,a=fixture();s=reorientation_settings(s);h['object_T_tcp']=np.eye(4).tolist();a['pose'][2]=100000
        v=np.array([[-.001,-.001,-.001],[.001,.001,.001]])
        with self.assertRaisesRegex(ValueError,'高于桌面'):adaptive_candidate(h,v,np.eye(3),a,c,s,np.zeros(3),adaptive=True)
        a['pose'][2]=400000;v=np.array([[0,0,-.5],[.01,.01,0]])
        with self.assertRaisesRegex(ValueError,'桌面'):adaptive_candidate(h,v,np.eye(3),a,c,s,np.zeros(3),adaptive=True)

    def test_diagnostics_remain_bounded_during_expanding_search(self):
        c,s,h,v,a=fixture();counter=[0];attempts=[]
        def check(*args):counter[0]+=1;return counter[0]>=201,{}
        p=plan_reorientation(h,v,np.eye(3),a,c,s,check,lambda:None,'adaptive',attempts)
        self.assertEqual(len(attempts),128);self.assertTrue(p['search_attempts_truncated'])
        self.assertEqual(attempts[0]['candidate_index'],1);self.assertEqual(attempts[-1]['candidate_index'],p['search_attempt_count'])

    def test_final_actual_height_is_enforced_even_within_pose_tolerance(self):
        with tempfile.TemporaryDirectory() as d:
            c,s,h,_,a=fixture();s=reorientation_settings(s);root=Path(d);cfg=root/'trial.json';cfg.write_text(json.dumps(c))
            h['object_T_tcp']=np.eye(4).tolist();v=np.array([[-.001,-.001,-.001],[.001,.001,.001]])
            plan=adaptive_candidate(h,v,np.eye(3),a,c,s,np.array([0,0,.101-a['pose'][2]/1e6]),adaptive=True)
            plan['position_mode']='adaptive'
            service=GraspController(root,None,cfg);service.held_object=h;service.robot=Mock();service.verify_motion_targets=Mock();service.check_reorient_contact=Mock()
            service.arm_state=Mock(side_effect=lambda:copy.deepcopy(a));count=[0]
            def move(pose,*args):
                count[0]+=1;a['pose']=pose.copy()
                if count[0]==2:a['pose'][2]=99500  # Within 2 mm of target, but fails the independent >10 cm rule.
            service.robot.move_linear.side_effect=move
            with self.assertRaisesRegex(ValueError,'实际末端最终高度'):service.execute_reorient(plan,h,c,s,{})
            service.robot.move_gripper.assert_not_called()

    def test_segment_corridor_rejects_lateral_and_endpoint_overshoot(self):
        self.assertEqual(segment_distance([.05,0,0],[0,0,0],[.1,0,0]),0)
        self.assertAlmostEqual(segment_distance([.05,.01,0],[0,0,0],[.1,0,0]),.01)
        self.assertAlmostEqual(segment_distance([.12,0,0],[0,0,0],[.1,0,0]),.02)
        self.assertAlmostEqual(segment_distance([0,.01,0],[0,0,0],[0,0,0]),.01)

    def test_execution_translates_then_rotates_without_gripper_write(self):
        for fault in [None,'deviation','early_arrival','cancel','contact']:
            with self.subTest(fault=fault),tempfile.TemporaryDirectory() as d:
                c,s,h,v,a=fixture();s=reorientation_settings(s);root=Path(d);cfg=root/'trial.json';cfg.write_text(json.dumps(c))
                service=GraspController(root,None,cfg);service.held_object=h;robot=service.robot=Mock()
                plan=adaptive_candidate(h,v,np.eye(3),a,c,s,np.array([0,-.05,0]))
                calls=[];service.verify_motion_targets=Mock();service.arm_state=Mock(side_effect=lambda:copy.deepcopy(a));service.check_reorient_contact=Mock()
                def move(pose,speed,cancelled,watchdog):
                    calls.append(pose)
                    if len(calls)==1:
                        a['pose'][:3]=np.rint((np.array(a['pose'][:3])+pose[:3])/2).astype(int).tolist()
                        if fault=='deviation':a['pose'][0]+=10000
                        if fault=='cancel':service.cancelled.set()
                        if fault=='contact':service.check_reorient_contact.side_effect=ValueError('contact')
                        watchdog()
                        if fault=='early_arrival':return
                    a['pose']=pose.copy()
                robot.move_linear.side_effect=move;report={}
                if fault:
                    with self.assertRaises(ValueError):service.execute_reorient(plan,h,c,s,report)
                    self.assertEqual(len(calls),1)
                else:
                    service.execute_reorient(plan,h,c,s,report)
                    self.assertEqual(len(calls),2);self.assertEqual(calls[0][:3],calls[1][:3]);self.assertEqual(service.status['state'],'completed')
                    self.assertEqual(service.held_object['hold_id'],h['hold_id']);self.assertEqual(len(report['arrival_checks']),2)
                robot.move_gripper.assert_not_called();robot.set_register.assert_not_called();robot.move_joint_pose.assert_not_called()

if __name__=='__main__':unittest.main()
