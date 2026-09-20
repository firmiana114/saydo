"""Bounded position search and translate-then-rotate execution without hardware."""
import copy
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
    def test_settings_and_request_bounds(self):
        _,settings,_,_,_=fixture();settings=reorientation_settings(settings)
        self.assertEqual(checked_position_request('adaptive',None,settings),('adaptive',.10))
        for mode,radius in [('other',.05),('adaptive',-.01),('adaptive',True),('adaptive',float('nan')),('adaptive',.101)]:
            with self.assertRaises(ValueError):checked_position_request(mode,radius,settings)
        for k,v in [('search_step_m',0),('max_joint_step_deg',True),('adaptive_max_shift_m',1)]:
            with self.assertRaises(ValueError):reorientation_settings({**settings,k:v})

    def test_candidates_are_bounded_nearest_first_and_not_lower(self):
        offsets=list(position_offsets(.067,.025));lengths=[np.linalg.norm(x) for x in offsets]
        self.assertEqual(lengths[0],0)
        self.assertTrue(all(0<=x[2] and np.linalg.norm(x)<=.067+1e-9 for x in offsets))
        self.assertTrue(all(a<=b+1e-9 for a,b in zip(lengths,lengths[1:])))
        self.assertLessEqual(len(offsets),1+17*3)

    def test_fixed_solution_is_preferred_and_requests_one_check(self):
        c,s,h,v,a=fixture();check=Mock(return_value=(True,{'max_joint_step_deg':1}))
        p=plan_reorientation(h,v,np.eye(3),a,c,s,check,lambda:None,'adaptive',.1)
        check.assert_called_once();self.assertEqual(p['offset_distance_m'],0);self.assertEqual(len(p['waypoints']),1)

    def test_search_preserves_rotation_and_includes_translation(self):
        c,s,h,v,a=fixture();rotation=Rotation.from_euler('xyz',[40,20,70],degrees=True).as_matrix()
        def check(points,timeout):
            shift=np.array(points[-1]['position_m'])-np.array(a['pose'][:3])/1e6
            return (np.linalg.norm(shift)>1e-5,{'max_joint_step_deg':2})
        p=plan_reorientation(h,v,rotation,a,c,s,check,lambda:None,'adaptive',.05)
        self.assertEqual([w['name'] for w in p['waypoints']],['translate','reorient'])
        self.assertGreater(p['offset_distance_m'],0);self.assertLessEqual(p['offset_distance_m'],.05)
        np.testing.assert_allclose(np.asarray(p['base_T_object_target'])[:3,:3],rotation,atol=1e-12)
        self.assertGreater(len(p['path_samples']),len(p['rotation_samples']))
        self.assertEqual(p['path_samples'][0]['controller_pose'][:3],a['pose'][:3])

    def test_discontinuous_solution_rejected_then_search_continues(self):
        c,s,h,v,a=fixture();check=Mock(side_effect=[(True,{'max_joint_step_deg':45}),(True,{'max_joint_step_deg':3})])
        p=plan_reorientation(h,v,np.eye(3),a,c,s,check,lambda:None,'adaptive',.025)
        self.assertGreater(p['offset_distance_m'],0);self.assertIn('关节变化',p['search_attempts'][0]['reason'])

    def test_exhaustion_cancel_and_time_budget(self):
        c,s,h,v,a=fixture();check=Mock(return_value=(False,{'returncode':1}))
        with self.assertRaisesRegex(ValueError,'限定范围'):plan_reorientation(h,v,np.eye(3),a,c,s,check,lambda:None,'adaptive',.005)
        check.reset_mock()
        with self.assertRaisesRegex(RuntimeError,'cancel'):plan_reorientation(h,v,np.eye(3),a,c,s,check,Mock(side_effect=RuntimeError('cancel')),'adaptive',.05)
        check.assert_not_called()
        with patch('reorientation_geometry.time.monotonic',side_effect=[0,21]),self.assertRaises(TimeoutError):
            plan_reorientation(h,v,np.eye(3),a,c,s,check,lambda:None,'adaptive',.05)
        check.assert_not_called()

    def test_candidate_timeout_does_not_cancel_remaining_search(self):
        c,s,h,v,a=fixture();check=Mock(side_effect=[subprocess.TimeoutExpired('ik',3),(True,{'max_joint_step_deg':1})])
        p=plan_reorientation(h,v,np.eye(3),a,c,s,check,lambda:None,'adaptive',.025)
        self.assertGreater(p['offset_distance_m'],0);self.assertIn('超时',p['search_attempts'][0]['reason'])

    def test_all_translation_candidates_outside_workspace(self):
        c,s,h,v,a=fixture();c['workspace_min_m']=[.279,.179,.329];c['workspace_max_m']=[.361,.221,.431]
        # Keep a small symmetric mesh around the held TCP in the narrow workspace.
        h['object_T_tcp']=np.eye(4).tolist();v=np.array([[-.001,-.001,-.001],[.001,.001,.001]])
        check=Mock(return_value=(False,{}));attempts=[]
        with self.assertRaises(ValueError):plan_reorientation(h,v,np.eye(3),a,c,s,check,lambda:None,'adaptive',.1,attempts)
        self.assertTrue(any('工作区' in str(row.get('reason','')) for row in attempts))

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
