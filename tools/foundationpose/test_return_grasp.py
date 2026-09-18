"""Post-lift joint return uses simulated hardware only."""
import copy
import json
from pathlib import Path
import tempfile
import threading
from types import SimpleNamespace
import unittest
from unittest.mock import Mock, patch

import numpy as np

from grasp_control import GraspController
from grasp_geometry import plan_grasp
from grasp_robot import RobotClient
import test_grasp


class ReturnGraspTests(unittest.TestCase):
    def test_return_checker_rejects_old_output_and_failed_path(self):
        service=GraspController('.',None,'unused')
        config=test_grasp.GraspTests().config();config['ik_checker']='unused'
        for result in (SimpleNamespace(returncode=0,stderr='',stdout='FK 0 0 0 0 0 0\n'),
                       SimpleNamespace(returncode=1,stderr='',stdout='RETURN_FAIL 5 1\n')):
            with patch('grasp_control.subprocess.run',return_value=result):
                self.assertFalse(service.check_return([0]*7,[1]*7,{'pose':[0]*6},config)['ok'])

    def test_return_is_after_lift_and_requires_valid_path_contact_and_arrival(self):
        for failure in (None, 'path', 'arrival', 'empty'):
            with self.subTest(failure=failure), tempfile.TemporaryDirectory() as directory:
                root=Path(directory);config=test_grasp.GraspTests().config()
                config['return_after_lift']={'joint_deg':[1,2,3,4,5,6,7]}
                cfg=root/'config.json';cfg.write_text(json.dumps(config))
                service=GraspController(root,None,cfg)
                arm={'joint':[0]*7,'pose':[0,0,200000,0,0,0],'err':[0]}
                obj=np.eye(4);obj[:3,3]=[.1,0,.2]
                plan=plan_grasp(arm['pose'],obj,config['grasp_profile'],config)
                plan.update(current_arm=copy.deepcopy(arm),gripper_base={'force_up':[100]},tool_response={},work_response={})
                calls=[];closed=[False]
                robot=Mock()
                def query(command):
                    if command=='get_current_arm_state':return {'arm_state':copy.deepcopy(arm)}
                    if command in ('get_current_tool_frame','get_current_work_frame'):return {}
                    empty=closed[0] and failure=='empty'
                    return {'state_info':{'sys_err':0,'sys_state':0,'dof_err':[0],
                        'dof_state':[2 if empty or not closed[0] else 3],
                        'pos':[0 if empty else 500 if closed[0] else 1000], 'speed':[0], 'force':[10 if closed[0] else 0]}}
                robot.query.side_effect=query
                def gripper(position):calls.append(('gripper',position));closed[0]=position==0
                robot.move_gripper.side_effect=gripper
                def linear(pose,speed,cancelled,watchdog):
                    watchdog();calls.append(('linear',pose));arm['pose']=pose
                robot.move_linear.side_effect=linear
                def joints(target,speed,cancelled,watchdog):
                    watchdog();calls.append(('return',target))
                    if failure!='arrival':arm['joint']=target
                robot.move_joints.side_effect=joints;service.robot=robot
                with patch.object(service,'check_return',return_value={'ok':failure!='path','reason':'return blocked'}):
                    if failure:
                        with self.assertRaises(ValueError):service.execute(plan,config,{})
                    else:
                        service.execute(plan,config,{})
                        self.assertEqual(service.status['state'],'completed')
                        self.assertEqual(calls[-1],('return',[1000,2000,3000,4000,5000,6000,7000]))
                        self.assertEqual(calls[-2],('linear',plan['waypoints'][-1]['controller_pose']))
                    if failure in ('path','empty'):robot.move_joints.assert_not_called()
                self.assertEqual([x for x in calls if x[0]=='gripper'],[('gripper',1000),('gripper',0)])

    def test_movej_transport_requires_matching_ack_and_completion(self):
        robot=RobotClient('test',writes=True);cancel=threading.Event()
        for accepted in (True,False):
            connection=Mock()
            connection.recv.return_value=(json.dumps({'command':'movej','receive_state':accepted})+'\n'+
                json.dumps({'state':'current_trajectory_state','trajectory_state':True})+'\n').encode()
            with patch('grasp_robot.socket.create_connection') as connect:
                connect.return_value.__enter__.return_value=connection
                if accepted:robot.move_joints([0]*7,10,cancel,lambda:None)
                else:
                    with self.assertRaises(RuntimeError):robot.move_joints([0]*7,10,cancel,lambda:None)
                payload=json.loads(connection.sendall.call_args.args[0])
                self.assertEqual(payload,{'command':'movej','joint':[0]*7,'v':10,'r':0,'trajectory_connect':0})
        with self.assertRaises(ValueError):RobotClient('test').move_joints([0]*7,10,cancel,lambda:None)
        cancel.set()
        with patch('grasp_robot.socket.create_connection') as connect:
            with self.assertRaisesRegex(RuntimeError,'取消'):robot.move_joints([0]*7,10,cancel,lambda:None)
            connect.assert_not_called()

    def test_invalid_return_config_rejected_before_motion(self):
        config=test_grasp.GraspTests().config();obj=np.eye(4);obj[2,3]=.2
        for joints in ([0]*6,[0]*6+[float('nan')],[0]*6+[400]):
            config['return_after_lift']={'joint_deg':joints}
            with self.assertRaises(ValueError):plan_grasp([0,0,200000,0,0,0],obj,config['grasp_profile'],config)


if __name__=='__main__':unittest.main()
