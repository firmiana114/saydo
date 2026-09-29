import copy
import json
import unittest
from unittest.mock import Mock, patch
import numpy as np
from fixed_pick_place import checked_request, placement_goals, joint_targets
import test_object_motion_control as control
import test_agent_api as api_base
from agent_api import AgentError


def fixtures():
    frame=np.eye(4);frame[:3,3]=[.35,-.1,.02]
    obj=np.eye(4);obj[:3,3]=[.3,-.1,.1]
    source={'instance_id':'tube','object_id':'sample','role':'source','state':'static','pose_mode':'5d','base_T_object':obj.tolist()}
    features=[]
    for slot,x in [('A01',0),('A02',.025)]:
        for part,z in [('entry',.085),('support',.012)]:
            f=np.eye(4);f[:3,3]=[x,0,z]
            features.append({'id':slot+'.'+part,'object_T_feature':f.tolist(),'diameter_m':.02})
    rack={'instance_id':'rack','object_id':'rack_model','role':'reference','state':'static','pose_mode':'6d','base_T_object':frame.tolist(),'features':features}
    scene={'scene_id':'a'*32,'instances':[source,rack]}
    request={'scene_id':scene['scene_id'],'source_instance_id':'tube','reference_instance_id':'rack','free_slots':['A01','A02'],'release_gap_m':.003,'seed':7}
    vertices=np.array([[x,y,z] for x in [-.007,.007] for y in [0] for z in [-.075,.075]])
    return scene,request,{'vertices':vertices}


class GeometryTests(unittest.TestCase):
    def test_random_order_reproducible_bottom_reference_and_support_height(self):
        scene,req,geometry=fixtures();checked_request(req)
        _,_,goals=placement_goals(scene,req,geometry)
        self.assertEqual([x[0] for x in goals],[x[0] for x in placement_goals(scene,req,geometry)[2]])
        for _,goal,_ in goals:
            self.assertEqual(goal['reference_point_object_m'],[0,0,-.075])
            self.assertAlmostEqual(goal['target_position_m'][2],.015)
            self.assertEqual(goal['target_rpy_deg'],[0,0,0])

    def test_inverted_rack_occupied_source_missing_slots_and_bad_fit_rejected(self):
        scene,req,geometry=fixtures()
        variants=[]
        inverted=copy.deepcopy(scene);inverted['instances'][1]['base_T_object'][1][1]=-1;inverted['instances'][1]['base_T_object'][2][2]=-1;variants.append((inverted,req))
        moved=copy.deepcopy(scene);moved['instances'][0]['state']='held';variants.append((moved,req))
        variants.append((scene,{**req,'free_slots':['unknown']}))
        tilted=copy.deepcopy(scene);angle=np.deg2rad(3)
        tilted['instances'][1]['base_T_object'][0][:3]=[np.cos(angle),0,np.sin(angle)]
        tilted['instances'][1]['base_T_object'][2][:3]=[-np.sin(angle),0,np.cos(angle)]
        variants.append((tilted,req))
        small=copy.deepcopy(scene)
        for f in small['instances'][1]['features']:f['diameter_m']=.005
        variants.append((small,req))
        for value,request in variants:
            with self.assertRaises(ValueError):placement_goals(value,request,geometry)
        for change in [{'free_slots':[]},{'free_slots':[{}]},{'seed':True},{'release_gap_m':float('nan')},{'release_gap_m':.01}]:
            with self.assertRaises(ValueError):checked_request({**req,**change})

    def test_waypoint_joint_mapping_uses_own_endpoint_not_retreat(self):
        points=[{'name':'grasp_sample_1'},{'name':'lift_sample_1'},{'name':'lift_sample_2'},{'name':'release_retreat_sample_1'}]
        q=[[i]*7 for i in range(4)]
        result=joint_targets(points,q,[{'name':'grasp'},{'name':'lift'}])
        self.assertEqual(result['grasp'],[0]*7);self.assertEqual(result['lift'],[2000]*7)


class PlanningTests(unittest.TestCase):
    def setUp(self):
        control.ObjectControlTests.setUp(self)
        self.service.held_object=None;self.scene,self.request,self.geom=fixtures()
        self.service.validate_scene=Mock(return_value=self.scene);self.service.fixed_binding_check=Mock()
        self.service.scene_observation=Mock(return_value={})
        self.service.stationary_scene_arm=Mock(return_value=copy.deepcopy(self.arm));self.service.arm_state=Mock(return_value=copy.deepcopy(self.arm))
        source=self.scene['instances'][0]
        self.grasp={'object_id':'sample','object_name':'tube','model_sha256':'mesh','pose_mode':'5d','pose_mode_revision':'v',
            'tool_response':{},'scene_id':self.scene['scene_id'],'instance_id':'tube','base_T_object':source['base_T_object'],
            'flange_T_grip_tcp':np.eye(4).tolist(),'grasp_library_revision':'v','execution_blockers':[],
            'effective_parameters':{'min_table_clearance_m':.01},'current_arm':copy.deepcopy(self.arm),
            'waypoints':[{'name':'pregrasp','controller_pose':[300000,-100000,200000,0,0,0]},
                         {'name':'grasp','controller_pose':[300000,-100000,100000,0,0,0]},
                         {'name':'lift','controller_pose':[300000,-100000,250000,0,0,0]}]}
        self.service.prepare_grasp_plan=Mock(return_value=(self.grasp,self.config))

    def check(self,checker,tool,arm,points,timeout=3):
        self.assertEqual(arm,self.arm)
        self.assertTrue(points[0]['name'].startswith('pregrasp_'))
        return True,{'verified':True,'joint_solutions_deg':[[0]*7 for _ in points]}

    def test_full_plan_includes_grasp_and_release_before_any_write(self):
        with patch('fixed_pick_place.model_geometry',return_value=self.geom),patch('fixed_pick_place.RobotClient') as robot,patch('fixed_pick_place.check_waypoints',side_effect=self.check) as check:
            plan=self.service.plan_fixed_pick_place(self.request,{})
        self.assertTrue(plan['complete_path_prechecked']);self.assertIn(plan['selected_slot'],self.request['free_slots'])
        self.assertEqual(set(plan['grasp']['fixed_joint_targets']),{'pregrasp','grasp','lift'})
        points=check.call_args.args[3]
        self.assertTrue(any(p['name'].startswith('release_descent') for p in points))
        self.assertTrue(any(p['name'].startswith('release_retreat') for p in points))
        robot.assert_called_once_with(self.config['robot_ip'],self.config['robot_port'],writes=False)
        self.assertEqual(self.calls,[])

    def test_failure_in_descent_never_saves_or_executes_a_prefix(self):
        def reject(checker,tool,arm,points,timeout=3):
            if any(p['name'].startswith('release_descent') for p in points):return False,{'reason':'descent failure'}
            return self.check(checker,tool,arm,points,timeout)
        store=Mock();self.service.object_plan_store=Mock(return_value=store)
        with patch('fixed_pick_place.model_geometry',return_value=self.geom),patch('fixed_pick_place.RobotClient'),patch('fixed_pick_place.check_waypoints',side_effect=reject):
            with self.assertRaisesRegex(ValueError,'全程'):self.service.plan_fixed_pick_place(self.request,{})
        store.save.assert_not_called();self.assertEqual(self.calls,[])

    def test_start_shift_blocks_before_gripper_or_replanning(self):
        self.service.stationary_scene_arm.return_value={**self.arm,'joint':[1000]*7}
        self.service.execute=Mock()
        plan={'effective_config':self.config,'grasp':self.grasp,'current_arm':self.arm}
        with patch('fixed_pick_place.RobotClient'):
            with self.assertRaisesRegex(ValueError,'起点'):self.service.execute_fixed_pick_place(plan,{},'id')
        self.service.execute.assert_not_called();self.service.prepare_grasp_plan.assert_not_called()

    def test_execution_uses_frozen_targets_and_rejects_changed_attachment(self):
        with patch('fixed_pick_place.model_geometry',return_value=self.geom),patch('fixed_pick_place.RobotClient'),patch('fixed_pick_place.check_waypoints',side_effect=self.check):
            plan=self.service.plan_fixed_pick_place(self.request,{})
        self.service.verify_motion_targets=Mock(side_effect=lambda p,c,r,a,points:{'joint_solutions_deg':[[0]*7 for _ in points]})
        self.service.update_scene_instance=Mock();order=[]
        def grasp(p,c,r):
            order.append('grasp');self.assertEqual(p['waypoints'],plan['grasp']['waypoints'])
            self.service.held_object={**copy.deepcopy(plan['predicted_hold']),'hold_id':'actual'}
        def release(p,request_id,nested=False):
            order.append('release');self.assertTrue(nested)
            self.assertEqual(p['waypoints'],plan['release']['waypoints'])
            self.assertEqual(p['hold']['hold_id'],'actual');self.service.status['state']='completed'
        self.service.execute=Mock(side_effect=grasp);self.service.run_object_execution=Mock(side_effect=release)
        with patch('fixed_pick_place.model_geometry',return_value=self.geom),patch('fixed_pick_place.RobotClient'):
            self.service.execute_fixed_pick_place(plan,{},'test')
        self.assertEqual(order,['grasp','release'])
        def changed(p,c,r):
            grasp(p,c,r);self.service.held_object['object_T_tcp'][0][3]+=.003
        self.service.execute.side_effect=changed;self.service.run_object_execution.reset_mock()
        with patch('fixed_pick_place.model_geometry',return_value=self.geom),patch('fixed_pick_place.RobotClient'):
            with self.assertRaisesRegex(ValueError,'夹持关系'):self.service.execute_fixed_pick_place(plan,{},'test')
        self.service.run_object_execution.assert_not_called()


class ApiTests(unittest.TestCase):
    request=api_base.AgentAPITests.request
    wait=api_base.AgentAPITests.wait
    def setUp(self):
        api_base.AgentAPITests.setUp(self)
        self.grasp.validate_scene=Mock()
        def run(args,request_id,plan):
            if plan is None:
                plan=self.grasp.object_plan_store().save({'kind':'fixed_pick_place','selected_slot':'A01'})
                self.grasp.update(state='fixed_pick_place_planned',fixed_pick_place_plan=plan,busy=False)
            else:
                self.calls.append('execute');self.grasp.object_plan_store().finish(plan['plan_id'],'completed')
                self.grasp.update(state='completed',busy=False)
        self.grasp.run_fixed_pick_place=run

    def test_fixed_execution_is_mode_gated_single_use_and_deduplicated(self):
        req=fixtures()[1];self.request('plan_fixed_pick_place',arguments=req)
        plan=self.wait()['result']['fixed_pick_place_plan'];args={'plan_id':plan['plan_id']}
        self.grasp.mode='preview'
        with self.assertRaises(AgentError):self.request('execute_fixed_pick_place',task_id='exec-001',arguments=args)
        self.grasp.mode='execute';self.request('execute_fixed_pick_place',task_id='exec-001',arguments=args);self.wait('exec-001')
        self.assertTrue(self.request('execute_fixed_pick_place',task_id='exec-001',arguments=args)['deduplicated'])
        self.assertEqual(self.request('execute_fixed_pick_place',task_id='exec-002',arguments=args)['state'],'rejected')
        self.assertEqual(self.calls,['execute'])
