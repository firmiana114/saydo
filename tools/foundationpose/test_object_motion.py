import copy
import json
from pathlib import Path
import tempfile
import unittest
from unittest.mock import Mock,patch
import numpy as np
from scipy.spatial.transform import Rotation
from grasp_geometry import controller_pose,rotation_error
from object_motion_geometry import plan_object_motion,object_error
from object_motion_store import ObjectPlanStore
from tools.robot_agent.motion_contract import checked_goal,DEFAULT_GOAL


def fixture():
    held={'object_id':'sample','hold_id':'hold1','orientation_version':0,'model_sha256':'mesh1','pose_mode':'6d','tool_response':{},'contact_position':500}
    attachment=np.eye(4);attachment[:3,3]=[0,.02,.04];held['object_T_tcp']=attachment.tolist()
    vertices=np.array([[x,y,z] for x in [-.015,.015] for y in [-.015,.015] for z in [-.06,.06]])
    arm={'pose':[300000,-100000,250000,0,0,0],'joint':[0]*7,'err':[0]}
    config={'workspace_reviewed':True,'workspace_min_m':[-1,-1,0],'workspace_max_m':[1,1,1],
            'table_z_m':0,'min_table_clearance_m':.01,'arm_speed_percent':25}
    return held,vertices,arm,config


class ObjectGeometryTests(unittest.TestCase):
    def plan(self,goal,**kw):
        held,vertices,arm,config=fixture()
        return plan_object_motion(held,vertices,arm,config,goal,np.eye(4),Mock(return_value=(True,{'verified':True})),lambda:None,**kw)

    def test_translation_rotation_full_pose_and_custom_reference(self):
        held,vertices,arm,config=fixture();start=controller_pose(arm['pose'])@np.linalg.inv(held['object_T_tcp'])
        plan=self.plan({'target_position_m':[.4,-.2,.3]})
        np.testing.assert_allclose(np.asarray(plan['base_T_object_target'])[:3,3],[.4,-.2,.3])
        np.testing.assert_allclose(controller_pose(plan['waypoints'][-1]['controller_pose'])[:3,3],[.4,-.18,.34])
        plan=self.plan({'target_rpy_deg':[90,0,0],'reference_point_object_m':[0,0,.04]})
        reference=start[:3,3]+start[:3,:3]@np.array([0,0,.04])
        self.assertEqual(len(plan['waypoints']),7)
        last=start[:3,:3]
        for waypoint in plan['waypoints']:
            obj=np.array(waypoint['object_pose']);np.testing.assert_allclose(obj[:3,3]+obj[:3,:3]@[0,0,.04],reference,atol=1e-12)
            self.assertLessEqual(np.rad2deg(rotation_error(last,obj[:3,:3])),15.00001);last=obj[:3,:3]
        plan=self.plan({'target_position_m':[.4,0,.35],'target_rpy_deg':[90,0,0]})
        np.testing.assert_allclose(np.asarray(plan['base_T_object_target'])[:3,3],[.4,0,.35])

    def test_frozen_frame_and_waypoint_component_preservation(self):
        h,v,a,c=fixture();frame=np.eye(4);frame[:3,:3]=Rotation.from_euler('z',90,degrees=True).as_matrix();frame[:3,3]=[.2,.2,0]
        goal={'reference_frame':'camera','target_position_m':[.1,0,.3],'target_rpy_deg':[0,0,0],
              'waypoints':[{'position_m':[0,0,.3],'rpy_deg':None}]}
        p=plan_object_motion(h,v,a,c,goal,frame,Mock(return_value=(True,{})),lambda:None)
        np.testing.assert_allclose(p['target_reference_position_base_m'],[.2,.3,.3])
        np.testing.assert_allclose(np.asarray(p['base_T_object_target'])[:3,:3],frame[:3,:3])

    def test_linear_samples_and_rejections(self):
        h,v,a,c=fixture();reachable=Mock(return_value=(True,{}))
        p=plan_object_motion(h,v,a,c,{'target_position_m':[.4,-.1,.3],'path_mode':'linear'},np.eye(4),reachable,lambda:None)
        self.assertGreater(len(reachable.call_args.args[0]),p['waypoint_count'])
        for goal in [{'target_position_m':[0,0,-.1]}, {'target_position_m':[1.5,0,.2]},
                     {'target_position_m':[.3,0,.2],'reference_point_object_m':[2,0,0]}]:
            with self.assertRaises(ValueError):self.plan(goal)
        for goal in [{'target_position_m':[True,0,0]},{'target_rpy_deg':[0,0,181]}, {'target_position_m':[0,0,.2],'position_mode':'adaptive'}, {'reference_frame':'unknown'}, {'target_rpy_deg':[0,0,0],'max_rotation_step_deg':0}]:
            with self.assertRaises(ValueError):checked_goal(goal)

    def test_5d_free_spin_and_zero_offset_are_not_reapplied(self):
        h,v,a,c=fixture();h['pose_mode']='5d';reachable=Mock(side_effect=[(False,{'reason':'first blocked'}),(True,{})])
        p=plan_object_motion(h,v,a,c,{'target_position_m':[.4,0,.3],'target_rpy_deg':[0,90,0]},np.eye(4),reachable,lambda:None)
        self.assertEqual(reachable.call_count,2)
        np.testing.assert_allclose(np.asarray(p['base_T_object_target'])[:3,2],[1,0,0],atol=1e-12)
        expected=np.asarray(p['base_T_object_target'])@h['object_T_tcp']
        np.testing.assert_allclose(controller_pose(p['waypoints'][-1]['controller_pose'])[:3,3],expected[:3,3],atol=1e-6)
        with self.assertRaises(ValueError):plan_object_motion(h,v,a,c,{'target_position_m':[.4,0,.3],'reference_point_object_m':[.01,0,0]},np.eye(4),reachable,lambda:None)

    def test_generic_release_uses_base_vertical_and_same_reference(self):
        h,v,a,c=fixture();frame=np.eye(4);frame[:3,:3]=Rotation.from_euler('x',30,degrees=True).as_matrix()
        for point in ([0,0,0],[0,0,-.06]):
            goal={'target_position_m':[.3,0,.3],'target_rpy_deg':[0,20,0],'reference_point_object_m':point}
            p=plan_object_motion(h,v,a,c,goal,frame,Mock(return_value=(True,{'verified':True})),lambda:None,release=True)
            pre,last=p['waypoints'][-2:]
            self.assertTrue(pre['name'].startswith('release_preplacement'));self.assertEqual(last['method'],'linear')
            np.testing.assert_allclose(np.array(pre['reference_position_base_m'])-last['reference_position_base_m'],[0,0,.1],atol=1e-12)
            np.testing.assert_allclose(np.array(pre['object_pose'])[:3,:3],np.array(last['object_pose'])[:3,:3],atol=1e-12)
            np.testing.assert_allclose(last['reference_position_base_m'],(frame@np.r_[goal['target_position_m'],1])[:3])
            self.assertEqual(p['reference_point_object_m'],point)
        c['release_preplacement_height_m']=float('nan')
        with self.assertRaises(ValueError):plan_object_motion(h,v,a,c,goal,frame,Mock(),lambda:None,release=True)

    def test_preplacement_workspace_checked_and_height_configurable(self):
        h,v,a,c=fixture();c['release_preplacement_height_m']=.12
        p=plan_object_motion(h,v,a,c,{'target_position_m':[.4,0,.3]},np.eye(4),Mock(return_value=(True,{'verified':True})),lambda:None,release=True)
        np.testing.assert_allclose(np.array(p['waypoints'][-2]['reference_position_base_m'])-p['waypoints'][-1]['reference_position_base_m'],[0,0,.12])
        with self.assertRaises(ValueError):self.plan({'target_position_m':[.4,0,.95]},release=True)

    def test_combined_goal_always_orients_before_translation(self):
        for release in (False,True):
            p=self.plan({'target_position_m':[.4,-.1,.3],'target_rpy_deg':[0,60,0],
                         'waypoints':[{'position_m':[.35,-.1,.35],'rpy_deg':None}]},release=release)
            point=np.array(p['reference_point_object_m']);start=np.array(p['base_T_object_start'])
            start_ref=start[:3,3]+start[:3,:3]@point;target=np.array(p['base_T_object_target'])[:3,:3]
            translating=False;orientation_steps=0
            for w in p['waypoints']:
                obj=np.array(w['object_pose']);ref=obj[:3,3]+obj[:3,:3]@point
                if w['name'].startswith(('orient_','release_orientation')):
                    self.assertFalse(translating)
                    np.testing.assert_allclose(ref,start_ref,atol=1e-9);orientation_steps+=1
                else:
                    translating=True;self.assertEqual(w['method'],'linear')
                    np.testing.assert_allclose(obj[:3,:3],target,atol=1e-9)
            self.assertGreater(orientation_steps,1)
            self.assertTrue(translating)
        with self.assertRaisesRegex(ValueError,'中间路点不得'):
            self.plan({'target_rpy_deg':[0,60,0],'waypoints':[{'position_m':[.3,0,.3],'rpy_deg':[0,20,0]}]})

    def test_detour_search_preserves_goal_orientation_and_final_descent(self):
        h,v,a,c=fixture();proof={'verified':True}
        reachable=Mock(side_effect=[(False,{'reason':'blocked direct','failure':{'kind':'ik_failed','waypoint':'release_preplacement_1_sample_2'}}),(True,proof)])
        p=plan_object_motion(h,v,a,c,{'target_position_m':[.4,-.1,.3]},np.eye(4),reachable,lambda:None,release=True)
        self.assertEqual(p['route_offset_m'],[0.,.08])
        self.assertTrue(any('detour' in w['name'] for w in p['waypoints']))
        pre,last=p['waypoints'][-2:]
        self.assertEqual(last['name'],'release_descent_1')
        np.testing.assert_allclose(np.array(pre['reference_position_base_m'])-last['reference_position_base_m'],[0,0,.1])
        final=np.array(p['base_T_object_target'])
        for w in p['waypoints']:
            self.assertEqual(w['method'],'linear')
            np.testing.assert_allclose(np.array(w['object_pose'])[:3,:3],final[:3,:3])
        self.assertEqual(p['search_attempts'][0]['failure']['kind'],'ik_failed')

    def test_release_current_pose_and_retreat(self):
        p=self.plan(None,release=True,retreat=[0,0,.05]);self.assertEqual(p['kind'],'release');self.assertEqual(p['waypoint_count'],2)
        self.assertAlmostEqual(p['retreat_waypoint']['position_m'][2],.30)
        self.assertFalse(p['validation_scope']['object_support_checked'])
        with self.assertRaises(ValueError):self.plan(None,release=True,retreat=[.3,.3,0])
        with self.assertRaises(ValueError):self.plan({'position_mode':'adaptive'},release=True)


class PlanStoreTests(unittest.TestCase):
    def test_expiry_claim_restart_and_tampering(self):
        with tempfile.TemporaryDirectory() as d:
            store=ObjectPlanStore(d);plan=store.save({'kind':'held_object_motion'})
            self.assertEqual(store.get(plan['plan_id']),plan)
            with patch('object_motion_store.time.time',return_value=plan['expires_at']+1):
                with self.assertRaisesRegex(ValueError,'有效期'):store.get(plan['plan_id'])
            store.claim(plan['plan_id'],'request1')
            with self.assertRaisesRegex(ValueError,'领取'):store.claim(plan['plan_id'],'request2')
            fresh=store.save({'kind':'release'})
            with self.assertRaisesRegex(ValueError,'重启'):ObjectPlanStore(d).get(fresh['plan_id'])
            with store.connect() as db:db.execute('UPDATE plans SET body=? WHERE id=?',(json.dumps({'bad':True}),fresh['plan_id']))
            with self.assertRaisesRegex(ValueError,'校验'):store.get(fresh['plan_id'])

if __name__=='__main__':unittest.main()
