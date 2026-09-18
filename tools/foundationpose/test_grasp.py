import copy
import json
from pathlib import Path
import tempfile
import time
from types import SimpleNamespace
import unittest
from unittest.mock import Mock, patch

import numpy as np

from grasp_geometry import controller_pose, object_in_base, plan_grasp, workspace_blockers, contact_result
from grasp_robot import RobotClient
from grasp_control import GraspController


class GraspTests(unittest.TestCase):
    def test_manual_open_releases_contact_and_rejects_conflicts(self):
        import fcntl
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory);cfg = root/'config.json';cfg.write_text(json.dumps(self.config()))
            service = GraspController(root, None, cfg)
            robot = self.force_robot()
            with patch('grasp_control.RobotClient', return_value=robot) as client:
                service.status['busy'] = True
                with self.assertRaisesRegex(ValueError, '进行中'):service.open_gripper()
                service.status['busy'] = False
                service.mode = 'preview'
                with self.assertRaisesRegex(ValueError, '预览'):service.open_gripper()
                client.assert_not_called()
                service.mode = 'execute'
                lock_path = root/'work/grasp';lock_path.mkdir(parents=True)
                with (lock_path/'robot.lock').open('a') as lock:
                    fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
                    with self.assertRaises(ValueError):service.open_gripper()
                client.assert_not_called()
                original = robot.query.side_effect
                def holding(command):
                    reply = original(command)
                    if command == 'get_rm_plus_state_info':
                        reply['state_info'].update(dof_state=[3], pos=[500], force=[10])
                    return reply
                robot.query.side_effect = holding
                self.assertTrue(service.open_gripper()['write_acknowledged'])
                robot.move_gripper.assert_called_once_with(1000)
                robot.move_linear.assert_not_called()
                robot.set_register.assert_not_called()
                robot.move_gripper.side_effect = TimeoutError('no reply')
                with self.assertRaisesRegex(ValueError, 'no reply'):service.open_gripper()
                self.assertEqual(robot.move_gripper.call_count, 2)
                self.assertFalse(json.loads(sorted((lock_path/'open_commands').glob('*.json'))[-1].read_text())['write_acknowledged'])
                def moving(command):
                    reply = original(command)
                    if command == 'get_rm_plus_state_info':reply['state_info']['speed'] = [1]
                    return reply
                robot.query.side_effect = moving
                with self.assertRaisesRegex(ValueError, '正在运动'):service.open_gripper()
                self.assertEqual(robot.move_gripper.call_count, 2)

    def test_manual_open_allows_retained_force_stop_but_not_closed_contact(self):
        state = {'sys_err': 0, 'sys_state': 0, 'dof_err': [0],
                 'dof_state': [3], 'pos': [1000], 'speed': [0], 'force': [0]}
        grip = self.config()['gripper']
        for force in (0, 10):
            for position in (997, 1000):
                GraspController.check_gripper_start({**state, 'pos': [position], 'force': [force]}, grip)
        for changes in ({'pos': [996]}, {'pos': [500]},
                        {'pos': [500], 'dof_state': [2], 'force': [10]}):
            with self.subTest(changes=changes), self.assertRaisesRegex(ValueError, '人工张开复位'):
                GraspController.check_gripper_start({**state, **changes}, grip)
        for changes in ({'speed': [1]}, {'sys_err': 1}, {'dof_state': [5]},
                        {'pos': []}, {'pos': [None]}, {'pos': [float('nan')]}, {'pos': [1001]}):
            with self.subTest(changes=changes), self.assertRaises(ValueError):
                GraspController.check_gripper_start({**state, **changes}, grip)

    def test_start_freezes_one_observation_before_background_work(self):
        observation = {'object_id': 'any_tracked_object', 'time': time.time(),
                       'pose': np.eye(4).tolist(), 'generation': 1}
        tracker = SimpleNamespace(grasp_observation=Mock(return_value=observation))
        service = GraspController('.', tracker, 'unused')
        with patch('grasp_control.threading.Thread') as thread:
            service.start()
            locked = thread.call_args.kwargs['args'][0]
            observation['pose'][0][3] = 10
            observation['object_id'] = 'another_object'
            self.assertEqual(locked['pose'][0][3], 0)
            self.assertEqual(locked['object_id'], 'any_tracked_object')
            self.assertTrue(service.status['busy'])
            tracker.grasp_observation.assert_called_once_with()
            thread.return_value.start.assert_called_once_with()

    def test_missing_initial_tracking_never_starts_job(self):
        tracker = SimpleNamespace(grasp_observation=Mock(side_effect=ValueError('没有可靠跟踪位姿')))
        service = GraspController('.', tracker, 'unused')
        with patch('grasp_control.threading.Thread') as thread:
            with self.assertRaisesRegex(ValueError, '没有可靠跟踪位姿'):
                service.start()
            thread.assert_not_called()
            self.assertFalse(service.status['busy'])

    def test_initial_snapshot_requires_fresh_image_and_stationary_arm(self):
        service = GraspController('.', None, 'unused')
        arm = {'joint': [0]*7, 'pose': [0, 0, 200000, 0, 0, 0]}
        with patch.object(service, 'arm_state', return_value=arm) as read, patch('grasp_control.time.sleep'):
            with self.assertRaisesRegex(ValueError, '过期'):
                service.paired_observation({'time': time.time() - 1})
            read.assert_not_called()
            observation = {'time': time.time()}
            self.assertIs(service.paired_observation(observation)[1], observation)
            moved = copy.deepcopy(arm); moved['pose'][0] = 1000
            read.side_effect = [arm, arm, moved]
            with self.assertRaisesRegex(ValueError, '未停稳'):
                service.paired_observation({'time': time.time()})

    def config(self):
        return {'robot_ip': 'test', 'robot_port': 8080, 'handeye_file': 'unused', 'tool_file': 'unused',
                'nominal_tcp_user_accepted': True, 'action': 'grasp_and_lift', 'lift_height_m': .10, 'pregrasp_distance_m': .05,
                'clearance_height_m': .08, 'max_target_distance_m': .3,
                'arm_speed_percent': 2, 'workspace_reviewed': True, 'workspace_min_m': [-1,-1,.05],
                'workspace_max_m': [1,1,1], 'table_z_m': 0,
                'gripper': {'vendor': 'ZX', 'device_id': 1, 'force_control_verified': True,
                    'force_limit_raw': 10, 'force_target_raw': 10,
                    'force_units': 'native', 'open_position': 1000,
                    'closed_position': 0, 'contact_position_margin': 20, 'close_timeout_s': 2},
                'grasp_profile': {'grasp_point_object_m': [0,0,0], 'orientation': 'keep_current_tcp'}}

    def test_rotated_tcp_offset_is_inverted_as_transform(self):
        base_flange = controller_pose([100000,200000,300000,1571,0,0])
        flange_tcp = np.eye(4);flange_tcp[2,3] = .2225
        flange_camera = np.eye(4);flange_camera[0,3] = .07
        camera_object = np.eye(4);camera_object[2,3] = .4
        got = object_in_base(base_flange @ flange_tcp, flange_tcp, flange_camera, camera_object)
        np.testing.assert_allclose(got, base_flange @ flange_camera @ camera_object, atol=1e-10)

    def test_plan_starts_at_directional_pregrasp_and_lifts_ten_cm(self):
        target = np.eye(4);target[:3,3] = [.1,0,.2]
        plan = plan_grasp([0,0,200000,0,0,0], target, self.config()['grasp_profile'], self.config())
        self.assertTrue(plan['lift_after_grasp'])
        self.assertEqual([w['name'] for w in plan['waypoints']], ['pregrasp', 'grasp', 'lift'])
        self.assertEqual(plan['waypoints'][-1]['name'], 'lift')
        self.assertEqual(plan['waypoints'][-1]['controller_pose'], [100000,0,300000,0,0,0])
        self.assertFalse(workspace_blockers(plan, self.config()))
        config = self.config();config['workspace_max_m'][2] = .29
        self.assertTrue(workspace_blockers(plan, config))  # Only the lift exceeds this ceiling.
        for height in (0, -0.1, float('nan')):
            config = self.config();config['lift_height_m'] = height
            with self.assertRaises(ValueError):
                plan_grasp([0,0,200000,0,0,0], target, config['grasp_profile'], config)
        config = self.config();config['workspace_reviewed'] = False
        self.assertTrue(workspace_blockers(plan, config))

    def test_table_clearance_ten_mm_for_start_and_waypoints(self):
        config = self.config()
        config['workspace_min_m'] = [-1, -1, 0]
        for height, blocked in [(0.009, True), (0.010, False), (0.011, False)]:
            for at_start in (True, False):
                with self.subTest(height=height, at_start=at_start):
                    plan = {'current_position_m': [0, 0, height if at_start else .2],
                            'waypoints': [{'position_m': [0, 0, .2 if at_start else height]}]}
                    self.assertEqual(bool(workspace_blockers(plan, config)), blocked)

    def test_force_contact_is_distinct_from_empty_close_or_current_protection(self):
        state = {'sys_err':0,'sys_state':0,'dof_err':[0],'dof_state':[3],'pos':[500],'speed':[0],'force':[10]}
        grip = self.config()['gripper']
        self.assertEqual(contact_result(state, grip), 'force_contact')
        self.assertEqual(contact_result({**state,'dof_state':[2],'pos':[0]},grip),'empty_close')
        self.assertEqual(contact_result({**state,'dof_state':[5]},grip),'fault')
        self.assertEqual(contact_result({**state,'force':[100]},grip),'force_contact')
        self.assertEqual(contact_result({**state,'force':[0]},grip),'force_stop')

    def test_acknowledgement_without_register_data_is_rejected(self):
        robot = RobotClient('test')
        with patch.object(robot,'query',return_value={'get_state':True}):
            with self.assertRaisesRegex(ValueError,'实际数值'):
                robot.register(1220)
        with self.assertRaises(ValueError):
            robot.move_gripper(1000)
        with self.assertRaises(ValueError):
            robot.query('movel')

    def test_verified_legacy_device_executes_without_readback_and_preview_does_not(self):
        for mode, preview in [('preview',False),('execute',False),('execute',True)]:
            with self.subTest(mode=mode, preview=preview), tempfile.TemporaryDirectory() as directory:
                root=Path(directory);config=self.config();config['gripper']['force_control_verified']=True
                config['grasp_profile'].update(orientation='follow_object',object_R_tcp=np.eye(3).tolist())
                cfg=root/'grasp.json';cfg.write_text(json.dumps(config))
                pose=np.eye(4);pose[0,3]=.1
                observation={'pose':pose.tolist(),'time':time.time(),'generation':1,'object_id':'never_configured_paper_cup','K':np.diag([.5,.5,1]).tolist()}
                tracker=SimpleNamespace(grasp_observation=Mock(side_effect=AssertionError('must not reread tracking')))
                service=GraspController(root,tracker,cfg,mode)
                service.force_override = 17
                arm={'pose':[0,0,200000,0,0,0],'joint':[0]*7,'err':[0]}
                tool_raw={'tool_name':'grip_tcp','pose':[0]*6}
                tool={'controller_readback':{'tool_response_raw':tool_raw},'flange_T_grip_tcp':np.eye(4).tolist()}
                handeye={'installation':{'controller_firmware':'V1.7.0'},'flange_T_color_optical_camera':np.eye(4).tolist(),'camera':{'K':np.eye(3).tolist()}}
                replies={'get_arm_software_info':{'Product_version':'RM75-BI','ctrl_info':{'version':'V1.7.0'}},
                    'get_current_tool_frame':tool_raw,'get_current_work_frame':{'frame_name':'Base','pose':[0]*6},
                    'get_joint_en_state':{'en_state':[1]*7},'get_joint_err_flag':{'err_flag':[0]*7},
                    'get_rm_plus_mode':{'mode':115200},
                    'get_rm_plus_base_info':{'base_info':{'manu':'ZX','id':1,'dof':1,'type':1,'force':True}},
                    'get_rm_plus_state_info':{'state_info':{'sys_err':0,'sys_state':0,'dof_err':[0],'dof_state':[3],'speed':[0],'force':[0],'pos':[1000]}}}
                fake=Mock()
                fake.query.side_effect=lambda command:copy.deepcopy(replies[command])
                fake.register.side_effect=AssertionError('numeric readback must not be required')
                with patch('grasp_control.RobotClient',return_value=fake) as client, patch.object(service,'calibration',return_value=(handeye,tool)), patch.object(service,'paired_observation',return_value=(arm,observation)), patch('subprocess.run',side_effect=AssertionError('must not sample or solve the movel path offline')), patch.object(service,'execute') as execute:
                    service.run(observation, preview=preview)
                    self.assertEqual(client.call_args.kwargs['writes'], mode == 'execute' and not preview)
                    self.assertEqual(execute.call_count, int(mode == 'execute' and not preview))
                    self.assertEqual(service.status['plan']['can_execute'], mode == 'execute' and not preview)
                    self.assertEqual(service.status['plan']['force_limit_raw'], 17)
                    self.assertFalse(service.status['plan']['motion_validation']['offline_path_checked'])
                    self.assertNotIn('ik_check', service.status['plan'])

    def force_robot(self):
        robot = Mock()
        replies = {
            'get_rm_plus_mode': {'mode': 115200},
            'get_rm_plus_base_info': {'base_info': {'manu':'ZX', 'id':1, 'type':1,
                'dof':1, 'force':True, 'force_up':[100]}},
            'get_rm_plus_state_info': {'state_info': {'sys_err':0, 'sys_state':0,
                'dof_err':[0], 'dof_state':[2], 'speed':[0], 'pos':[1000], 'force':[0]}},
        }
        robot.query.side_effect = lambda command: copy.deepcopy(replies[command])
        return robot

    def test_force_input_writes_hardware_and_preserves_session_value(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory); cfg = root / 'grasp.json'
            original = json.dumps(self.config()); cfg.write_text(original)
            service = GraspController(root, None, cfg)
            robot = self.force_robot()
            with patch('grasp_control.RobotClient', return_value=robot) as client:
                for invalid in (0, 101, -1, True, 1.5, '10', None):
                    with self.assertRaises(ValueError):
                        service.set_force_config(invalid)
                client.assert_not_called()
                for value in (1, 100, 12):
                    result = service.set_force_config(value)
                    self.assertEqual(result['value'], value)
                    self.assertTrue(result['write_acknowledged'])
                    self.assertFalse(result['readback_verified'])
                    robot.set_register.assert_called_with(1220, value)
                self.assertEqual(robot.set_register.call_count, 3)
                robot.move_gripper.assert_not_called()
                robot.move_linear.assert_not_called()
                robot.register.assert_not_called()
                service.update(busy=True)
                with self.assertRaises(ValueError):
                    service.set_force_config(20)
                self.assertEqual(service.snapshot()['force_config']['value'], 12)
            self.assertEqual(cfg.read_text(), original)
            self.assertFalse(GraspController(root, None, cfg).force_config()['hardware_applied'])
            self.assertEqual(len(list((root/'work/grasp/force_commands').glob('*.json'))), 3)

    def test_force_failure_does_not_commit_value_or_retry(self):
        with tempfile.TemporaryDirectory() as directory:
            root=Path(directory);cfg=root/'config.json';cfg.write_text(json.dumps(self.config()))
            service=GraspController(root,None,cfg)
            robot=self.force_robot()
            with patch('grasp_control.RobotClient',return_value=robot):
                service.set_force_config(10)
                robot.set_register.side_effect=TimeoutError('no reply')
                with self.assertRaisesRegex(ValueError,'no reply'):
                    service.set_force_config(50)
            self.assertEqual(robot.set_register.call_count,2)
            self.assertEqual(service.force_config()['value'],10)
            self.assertFalse(service.force_config()['hardware_applied'])
            self.assertIn('no reply',service.force_config()['error'])
            robot.move_gripper.assert_not_called()

    def test_preview_force_update_never_connects(self):
        with tempfile.TemporaryDirectory() as directory:
            root=Path(directory);cfg=root/'config.json';cfg.write_text(json.dumps(self.config()))
            service=GraspController(root,None,cfg,'preview')
            with patch('grasp_control.RobotClient') as client:
                self.assertEqual(service.set_force_config(50)['value'],50)
                client.assert_not_called()
            self.assertFalse(service.force_config()['hardware_applied'])

    def test_force_write_is_blocked_by_another_grasp_process(self):
        import fcntl
        with tempfile.TemporaryDirectory() as directory:
            root=Path(directory);cfg=root/'config.json';cfg.write_text(json.dumps(self.config()))
            path=root/'work/grasp';path.mkdir(parents=True)
            service=GraspController(root,None,cfg)
            with (path/'robot.lock').open('a') as lock, patch('grasp_control.RobotClient') as client:
                fcntl.flock(lock,fcntl.LOCK_EX|fcntl.LOCK_NB)
                with self.assertRaises(ValueError):service.set_force_config(10)
                client.assert_not_called()

    def test_gripper_position_transport_and_force_rejection(self):
        robot=RobotClient('test',writes=True)
        with patch.object(robot,'_request',return_value={'set_state':True}) as request:
            robot.move_gripper(1000)
            request.assert_called_with({'command':'hand_follow_pos','hand_pos':[1000]})
            robot.move_gripper(0)
            request.assert_called_with({'command':'hand_follow_pos','hand_pos':[0]})
        with patch.object(robot,'_request',return_value={'set_state':False}):
            with self.assertRaises(RuntimeError):robot.set_register(1220,10)

    def test_default_execute_requires_explicit_start_and_preview_remains_available(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory); cfg = root / 'grasp.json'
            cfg.write_text(json.dumps(self.config()))
            with patch('grasp_control.RobotClient') as client:
                service = GraspController(root, None, cfg)
                self.assertEqual(service.snapshot()['mode'], 'execute')
                self.assertTrue(service.snapshot()['motion_enabled'])
                self.assertFalse(service.snapshot()['busy'])
                self.assertIsNone(service.thread)
                preview = GraspController(root, None, cfg, 'preview')
                self.assertFalse(preview.snapshot()['motion_enabled'])
                client.assert_not_called()

    def test_executor_lifts_only_after_contact(self):
        with tempfile.TemporaryDirectory() as directory:
            root=Path(directory);config=self.config();cfg=root/'config.json';cfg.write_text(json.dumps(config))
            observation={'pose':np.eye(4).tolist(),'time':time.time(),'generation':1,'object_id':'never_configured_paper_cup'}
            service=GraspController(root,SimpleNamespace(grasp_observation=Mock(side_effect=AssertionError('tracking lost'))),cfg,'execute')
            arm={'pose':[0,0,200000,0,0,0],'joint':[0]*7,'err':[0]}
            target=np.eye(4);target[:3,3]=[.1,0,.2]
            plan=plan_grasp(arm['pose'],target,config['grasp_profile'],config)
            plan.update(current_arm=copy.deepcopy(arm),gripper_base={'force_up':[100],'speed_up':[500]},tool_response={},work_response={})
            class FakeRobot:
                def __init__(self):self.calls=[];self.registers={};self.contact=False
                def query(self,command):
                    if command=='get_current_arm_state':return {'arm_state':copy.deepcopy(arm)}
                    if command in ('get_current_tool_frame','get_current_work_frame'):return {}
                    return {'state_info':{'sys_err':0,'sys_state':0,'dof_err':[0], 'dof_state':[3 if self.contact else 2], 'pos':[500 if self.contact else 1000], 'force':[10 if self.contact else 0],'speed':[0]}}
                def set_register(self,address,value):
                    self.registers[address]=value;self.calls.append(('register',address,value))
                def register(self,address):return self.registers[address]
                def move_gripper(self,position):
                    self.calls.append(('position',position))
                    self.contact=position==0
                def move_linear(self,pose,speed,cancelled,watchdog):
                    watchdog();self.calls.append(('move',pose));arm['pose']=list(pose)
            robot=FakeRobot();service.robot=robot;report={}
            service.execute(plan,config,report)
            self.assertEqual(service.status['state'],'completed')
            moves=[x for x in robot.calls if x[0]=='move']
            self.assertEqual(len(moves),3)
            self.assertEqual(moves[-1][1],plan['waypoints'][-1]['controller_pose'])
            self.assertEqual(robot.calls[:2],[('register',1220,10),('position',1000)])
            self.assertEqual(robot.calls[-3:-1],[('register',1220,10),('position',0)])
            self.assertEqual(moves[0][1], plan['waypoints'][0]['controller_pose'])
            self.assertEqual(len([x for x in robot.calls[:robot.calls.index(('position',0))] if x[0]=='move']),2)
            self.assertLess(next(i for i,x in enumerate(robot.calls) if x==('position',1000)),next(i for i,x in enumerate(robot.calls) if x[0]=='move'))
            robot.calls.clear()
            arm['pose'] = [0,0,200000,0,0,0]
            with patch.object(robot, 'set_register', side_effect=RuntimeError('rejected')):
                with self.assertRaisesRegex(RuntimeError, 'rejected'):
                    service.execute(plan, config, {})
            self.assertEqual(robot.calls, [])
            original_query = robot.query
            for stop_state in (2, 5):
                robot.calls.clear()
                arm['pose'] = [0,0,200000,0,0,0]
                def failed_close(command):
                    reply = original_query(command)
                    if command == 'get_rm_plus_state_info' and robot.contact:
                        reply['state_info'].update(dof_state=[stop_state], pos=[0])
                    return reply
                with patch.object(robot, 'query', side_effect=failed_close):
                    with self.assertRaisesRegex(ValueError, '异常或空夹'):
                        service.execute(plan, config, {})
                self.assertEqual(len([c for c in robot.calls if c[0] == 'move']), 2)
            robot.calls.clear()
            arm['pose'] = [0,0,200000,0,0,0]
            move = robot.move_linear
            def wrong_orientation(*args):
                move(*args)
                arm['pose'][3] += 100  # About 5.7 degrees, above the 2-degree tolerance.
            with patch.object(robot, 'move_linear', side_effect=wrong_orientation):
                with self.assertRaisesRegex(ValueError, '朝向'):
                    service.execute(plan, config, {})
            self.assertNotIn(('position', 0), robot.calls)


if __name__ == '__main__':
    unittest.main()
