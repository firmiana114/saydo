import copy
import json
from pathlib import Path
import tempfile
import time
from types import SimpleNamespace
import unittest
from unittest.mock import patch

import numpy as np

from grasp_geometry import controller_pose, object_in_base, plan_grasp, workspace_blockers, contact_result
from grasp_robot import RobotClient
from grasp_control import GraspController


class GraspTests(unittest.TestCase):
    def config(self):
        return {'robot_ip': 'test', 'robot_port': 8080, 'handeye_file': 'unused', 'tool_file': 'unused',
                'nominal_tcp_user_accepted': True, 'action': 'close_only_no_lift', 'pregrasp_distance_m': .05,
                'clearance_height_m': .08, 'max_target_distance_m': .3, 'max_plan_age_s': 5,
                'arm_speed_percent': 2, 'workspace_reviewed': True, 'workspace_min_m': [-1,-1,.05],
                'workspace_max_m': [1,1,1], 'table_z_m': 0,
                'gripper': {'vendor': 'ZX', 'device_id': 1, 'force_control_verified': True,
                    'contact_state_verified': True, 'force_limit_raw': 10, 'force_target_raw': 10,
                    'force_units': 'native', 'speed_limit_raw': 100, 'open_position': 900,
                    'closed_position': 0, 'contact_position_margin': 20, 'close_timeout_s': 2},
                'objects': {'bottle': {'grasp_point_object_m': [0,0,0], 'orientation': 'keep_current_tcp'}}}

    def test_rotated_tcp_offset_is_inverted_as_transform(self):
        base_flange = controller_pose([100000,200000,300000,1571,0,0])
        flange_tcp = np.eye(4);flange_tcp[2,3] = .2225
        flange_camera = np.eye(4);flange_camera[0,3] = .07
        camera_object = np.eye(4);camera_object[2,3] = .4
        got = object_in_base(base_flange @ flange_tcp, flange_tcp, flange_camera, camera_object)
        np.testing.assert_allclose(got, base_flange @ flange_camera @ camera_object, atol=1e-10)

    def test_plan_preserves_orientation_and_never_lifts_after_grasp(self):
        target = np.eye(4);target[:3,3] = [.1,0,.2]
        plan = plan_grasp([0,0,200000,0,0,0], target, self.config()['objects']['bottle'], self.config())
        self.assertFalse(plan['lift_after_grasp'])
        self.assertEqual(plan['waypoints'][-1]['name'], 'grasp')
        self.assertEqual(plan['waypoints'][-1]['controller_pose'], [100000,0,200000,0,0,0])
        self.assertFalse(workspace_blockers(plan, self.config()))
        config = self.config();config['workspace_reviewed'] = False
        self.assertTrue(workspace_blockers(plan, config))

    def test_force_contact_is_distinct_from_empty_close_or_current_protection(self):
        state = {'sys_err':0,'sys_state':0,'dof_err':[0],'dof_state':[3],'pos':[500],'speed':[0],'force':[10]}
        grip = self.config()['gripper']
        self.assertEqual(contact_result(state, grip), 'force_contact')
        self.assertEqual(contact_result({**state,'dof_state':[2],'pos':[0]},grip),'empty_close')
        self.assertEqual(contact_result({**state,'dof_state':[5]},grip),'fault')
        self.assertEqual(contact_result({**state,'force':[100]},grip),'fault')

    def test_acknowledgement_without_register_data_is_rejected(self):
        robot = RobotClient('test')
        with patch.object(robot,'query',return_value={'get_state':True}):
            with self.assertRaisesRegex(ValueError,'实际数值'):
                robot.register(1220)
        with self.assertRaises(ValueError):
            robot.open_gripper(900)
        with self.assertRaises(ValueError):
            robot.query('movel')

    def test_preview_never_executes_and_unverified_force_blocks_execute_mode(self):
        for mode, verified in [('preview',True),('execute',False)]:
            with self.subTest(mode=mode), tempfile.TemporaryDirectory() as directory:
                root=Path(directory);config=self.config();config['gripper']['force_control_verified']=verified
                cfg=root/'grasp.json';cfg.write_text(json.dumps(config))
                pose=np.eye(4);pose[0,3]=.1
                observation={'pose':pose.tolist(),'time':time.time(),'generation':1,'object_id':'bottle'}
                tracker=SimpleNamespace(grasp_observation=lambda:observation)
                service=GraspController(root,tracker,cfg,mode)
                arm={'pose':[0,0,200000,0,0,0],'joint':[0]*7,'err':[0]}
                tool_raw={'tool_name':'grip_tcp','pose':[0]*6}
                tool={'controller_readback':{'tool_response_raw':tool_raw},'flange_T_grip_tcp':np.eye(4).tolist()}
                handeye={'installation':{'controller_firmware':'V1.7.0'},'flange_T_color_optical_camera':np.eye(4).tolist()}
                replies={'get_arm_software_info':{'Product_version':'RM75-BI','ctrl_info':{'version':'V1.7.0'}},
                    'get_current_tool_frame':tool_raw,'get_current_work_frame':{'frame_name':'Base','pose':[0]*6},
                    'get_joint_en_state':{'en_state':[1]*7},'get_joint_err_flag':{'err_flag':[0]*7},
                    'get_rm_plus_mode':{'mode':115200},
                    'get_rm_plus_base_info':{'base_info':{'manu':'ZX','id':1,'dof':1,'type':1,'force':True}},
                    'get_rm_plus_state_info':{'state_info':{'sys_err':0,'sys_state':0,'dof_err':[0],'dof_state':[2],'speed':[0],'force':[0],'pos':[500]}}}
                fake=SimpleNamespace(query=lambda command:copy.deepcopy(replies[command]),register=lambda address:100)
                with patch('grasp_control.RobotClient',return_value=fake), patch.object(service,'calibration',return_value=(handeye,tool)), patch.object(service,'paired_observation',return_value=(arm,observation)), patch.object(service,'check_ik',return_value={'ok':True}), patch.object(service,'execute') as execute:
                    service.run()
                    execute.assert_not_called()
                    self.assertFalse(service.status['plan']['can_execute'])

    def test_executor_orders_open_path_and_contact_without_lift(self):
        with tempfile.TemporaryDirectory() as directory:
            root=Path(directory);config=self.config();cfg=root/'config.json';cfg.write_text(json.dumps(config))
            observation={'pose':np.eye(4).tolist(),'time':time.time(),'generation':1,'object_id':'bottle'}
            service=GraspController(root,SimpleNamespace(grasp_observation=lambda:observation),cfg,'execute')
            arm={'pose':[0,0,200000,0,0,0],'joint':[0]*7,'err':[0]}
            target=np.eye(4);target[:3,3]=[.1,0,.2]
            plan=plan_grasp(arm['pose'],target,config['objects']['bottle'],config)
            plan.update(current_arm=copy.deepcopy(arm),gripper_base={'force_up':[100],'speed_up':[500]},tool_response={},work_response={})
            class FakeRobot:
                def __init__(self):self.calls=[];self.registers={};self.contact=False
                def query(self,command):
                    if command=='get_current_arm_state':return {'arm_state':copy.deepcopy(arm)}
                    if command in ('get_current_tool_frame','get_current_work_frame'):return {}
                    return {'state_info':{'sys_err':0,'sys_state':0,'dof_err':[0], 'dof_state':[3 if self.contact else 2], 'pos':[500 if self.contact else 900], 'force':[10 if self.contact else 0],'speed':[0]}}
                def set_register(self,address,value):
                    self.registers[address]=value;self.calls.append(('register',address,value))
                    if address==1470:self.contact=True
                def register(self,address):return self.registers[address]
                def open_gripper(self,position):self.calls.append(('open',position))
                def move_linear(self,pose,speed,cancelled,watchdog):
                    watchdog();self.calls.append(('move',pose));arm['pose']=list(pose)
            robot=FakeRobot();service.robot=robot;report={}
            service.execute(plan,observation,config,report)
            self.assertEqual(service.status['state'],'contact')
            moves=[x for x in robot.calls if x[0]=='move']
            self.assertEqual(len(moves),4)
            self.assertEqual(moves[-1][1],plan['waypoints'][-1]['controller_pose'])
            self.assertEqual(robot.calls[-1],('register',1470,10))
            self.assertLess(next(i for i,x in enumerate(robot.calls) if x[0]=='open'),next(i for i,x in enumerate(robot.calls) if x[0]=='move'))


if __name__ == '__main__':
    unittest.main()
