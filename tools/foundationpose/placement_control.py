"""Placement shares the grasp task/robot locks and never infers an old held object."""
import copy
import fcntl
import json
import threading
import time
import traceback
import uuid

import numpy as np
import yaml

from grasp_geometry import controller_pose, transform, rotation_error, contact_result
from grasp_robot import RobotClient
from placement_visual import model_appearance
from placement_geometry import model_geometry, checked_rotation, placement_settings, plan_placement, endpoint_reachability


class PlacementMixin:
    def held_snapshot(self):
        if self.held_object is None:
            return None
        held = self.held_object
        return {k: copy.deepcopy(held[k]) for k in ['hold_id','object_id','object_name','model_sha256',
                                                   'base_T_object_at_grasp']}

    def capture_attachment(self, plan, report):
        if not plan.get('object_id') or not plan.get('model_sha256'):
            return
        reached = self.arm_state()
        base_tcp = controller_pose(reached['pose'])
        held = {'hold_id':uuid.uuid4().hex, 'object_id':plan['object_id'], 'object_name':plan.get('object_name'),
                'model_sha256':plan['model_sha256'], 'base_T_object_at_grasp':plan['base_T_object'],
                'object_T_tcp':(np.linalg.inv(transform(plan['base_T_object'])) @ base_tcp).tolist(),
                'tool_response':plan['tool_response'],
                'contact_position':self.status['gripper_contact']['pos'][0],
                'recorded_at':time.time(), 'evidence':'frozen_visual_pose_and_gripper_feedback_not_slip_measurement'}
        report['held_object'] = held

    def finish_attachment(self, report):
        with self.lock:
            self.held_object = copy.deepcopy(report.get('held_object'))

    def placement_model(self, object_id=None):
        with self.lock:
            held = copy.deepcopy(self.held_object)
        if not object_id:
            object_id = held['object_id'] if held else self.tracker.state.get('object_id')
        geometry = model_geometry(self.tracker.models_root, object_id)
        try:
            geometry['appearance'] = model_appearance(self.tracker.models_root, object_id)
        except (ValueError, OSError, TypeError) as error:
            geometry['appearance'] = {'status': 'unavailable', 'error': str(error)}
        geometry['initial_rotation'] = (np.asarray(held['base_T_object_at_grasp'])[:3,:3].tolist()
                                        if held and held['object_id']==object_id else np.eye(3).tolist())
        geometry['holding'] = bool(held and held['object_id']==object_id)
        settings = placement_settings(json.loads((self.root/'config/grasp/placement.json').read_text()))
        geometry['release_clearance_m'] = settings['release_clearance_m']
        geometry['target_xy_m'] = settings['target_xy_m']
        return geometry

    def start_place(self, value, preview=False):
        rotation = checked_rotation(value.get('rotation'))
        with self.lock:
            if self.status['busy']:
                raise ValueError('已有机械臂任务正在处理')
            held = copy.deepcopy(self.held_object)
            if held is None:
                raise ValueError('本次服务尚无成功抓取记录，请先抓取物品后再放置')
            if value.get('hold_id') != held['hold_id'] or value.get('object_id') != held['object_id'] or value.get('model_sha256') != held['model_sha256']:
                raise ValueError('预览模型或夹持记录已变化，请重新加载放置模型')
            self.cancelled.clear();self.actuated=False;self.robot=None
            self.status.update(state='place_planning',operation='place',busy=True,error=None,plan=None,
                               placement_plan=None,target_object_id=held['object_id'],
                               message='正在按指定朝向计算桌面落点并检查可达性…')
            self.thread = threading.Thread(target=self.run_place,args=(held,rotation.tolist(),preview),daemon=True)
            self.thread.start()
        return {'accepted':True,'mode':'preview' if preview or self.mode=='preview' else 'execute'}

    def placement_hardware(self, held, config):
        if self.robot.query('get_arm_software_info').get('Product_version') != 'RM75-BI':
            raise ValueError('机械臂型号不匹配')
        if self.robot.query('get_joint_en_state').get('en_state') != [1]*7 or self.robot.query('get_joint_err_flag').get('err_flag') != [0]*7:
            raise ValueError('关节使能或错误状态异常')
        tool = yaml.safe_load((self.root/config['tool_file']).read_text())
        if self.robot.query('get_current_tool_frame') != held['tool_response'] or tool['controller_readback']['tool_response_raw'] != held['tool_response']:
            raise ValueError('工具/负载已变化，不能沿用抓取关系放置')
        work = self.robot.query('get_current_work_frame')
        if work.get('frame_name') != 'Base' or work.get('pose') != [0]*6:
            raise ValueError('放置要求零偏移 Base 工作坐标系')
        if self.robot.query('get_rm_plus_mode').get('mode') != 115200:
            raise ValueError('夹爪通信模式不匹配')
        base = self.robot.query('get_rm_plus_base_info')['base_info'];grip=config['gripper']
        if (base.get('manu'),base.get('id'),base.get('dof'),base.get('type')) != (grip['vendor'],grip['device_id'],1,1):
            raise ValueError('夹爪身份不匹配')
        self.check_place_contact(held,config)
        first=self.arm_state();time.sleep(.15);last=self.arm_state()
        a,b=np.asarray(first['joint'],dtype=float),np.asarray(last['joint'],dtype=float)
        if a.shape!=(7,) or b.shape!=(7,) or not np.isfinite([a,b]).all() or np.max(np.abs(a-b))>30:
            raise ValueError('机械臂未停稳，不能规划放置')
        return last,tool

    def check_place_contact(self, held, config):
        state=self.robot.query('get_rm_plus_state_info')['state_info']
        if contact_result(state,config['gripper']) not in ('force_contact','force_stop') or abs(state['pos'][0]-held['contact_position'])>20:
            with self.lock:self.held_object=None
            raise ValueError('夹持反馈已变化或未保持夹持，停止放置')
        return state

    def run_place(self, held, rotation, preview=False):
        mode='preview' if preview else self.mode
        report={'started_at':time.time(),'operation':'place','mode':mode,'held_object':held,
                'requested_rotation':rotation,'robot_motion_sent':False,'release_attempted':False,'active_stage':'放置前检查'}
        directory=self.root/'work/grasp/place_jobs';robot_lock=None
        try:
            directory.mkdir(parents=True,exist_ok=True)
            robot_lock=(directory.parent/'robot.lock').open('a')
            fcntl.flock(robot_lock,fcntl.LOCK_EX|fcntl.LOCK_NB)
            config=json.loads(self.config_path.read_text())
            settings=placement_settings(json.loads((self.root/'config/grasp/placement.json').read_text()))
            if settings['release_position'] <= held['contact_position']+20:
                raise ValueError('放置张开位置必须大于当前夹持位置至少 20 个原始单位')
            report['settings']=settings
            self.robot=RobotClient(config['robot_ip'],config['robot_port'],writes=mode=='execute')
            self.robot.motion_log=report.setdefault('motion_events',[])
            self.check_cancel()
            arm,tool=self.placement_hardware(held,config)
            mesh=model_geometry(self.tracker.models_root,held['object_id'])
            if mesh['model_sha256']!=held['model_sha256']:
                raise ValueError('物品模型已变化，不能沿用抓取关系放置')
            checker=(self.root/config['ik_checker']).resolve()
            if not checker.is_file():raise ValueError('缺少放置航点可达性检查程序')
            report['active_stage']='检查固定放置点'
            plan=plan_placement(held,mesh['vertices'],rotation,arm,config,settings,
                lambda waypoints,timeout:endpoint_reachability(checker,tool,arm,waypoints,timeout),self.check_cancel)
            plan.update(object_id=held['object_id'],object_name=held['object_name'],hold_id=held['hold_id'],
                        release_position=settings['release_position'],can_execute=mode=='execute')
            report['plan']=plan
            self.update(placement_plan=plan)
            self.check_cancel()
            if mode!='execute':
                self.update(state='place_preview',message=f"固定放置方案已生成：基座 X＝{plan['target_xy_m'][0]*100:g} 厘米，Y＝{plan['target_xy_m'][1]*100:g} 厘米；未发送运动。")
                return
            if np.max(np.abs(np.asarray(self.arm_state()['joint'])-arm['joint']))>30:
                raise ValueError('机械臂在放置规划后发生移动')
            self.check_place_contact(held,config)
            self.execute_place(plan,held,config,settings,report)
        except Exception as error:
            if self.actuated:
                with self.lock:self.held_object=None
                try:self.robot.stop()
                except Exception as stop_error:report['stop_error']=str(stop_error)
            detail=f"{report['active_stage']}：{error}"
            report.update(error=detail,error_type=type(error).__name__,traceback=traceback.format_exc())
            self.update(state='cancelled' if self.cancelled.is_set() else 'failed',message=detail,error=detail)
        finally:
            report['result']={**self.status,'busy':False}
            try:
                directory.mkdir(parents=True,exist_ok=True)
                (directory/f'{time.time_ns()}.json').write_text(json.dumps(report,ensure_ascii=False,indent=2))
            finally:
                if robot_lock is not None:robot_lock.close()
                self.update(busy=False)

    def execute_place(self, plan, held, config, settings, report):
        names={'raise':'抬高已夹物品','above':'调整朝向并移动到落点上方','place':'按指定朝向下降到桌面','retreat':'松开后沿夹爪后方退出'}
        for waypoint in plan['waypoints']:
            self.check_cancel()
            if waypoint['name']!='retreat':self.check_place_contact(held,config)
            report['active_stage']=names[waypoint['name']]
            self.update(state='placing',message=report['active_stage'])
            self.actuated=True;report['robot_motion_sent']=True
            self.robot.motion_stage=report['active_stage']
            move=self.robot.move_joint_pose if waypoint['method']=='joint_pose' else self.robot.move_linear
            move(waypoint['controller_pose'],settings['arm_speed_percent'],self.cancelled,self.check_cancel)
            self.check_cancel();actual=self.arm_state()
            report.setdefault('arrival_checks',[]).append({'stage':report['active_stage'],'target_pose':waypoint['controller_pose'],'actual_arm_state':actual})
            expected=controller_pose(waypoint['controller_pose']);reached=controller_pose(actual['pose'])
            if np.linalg.norm(expected[:3,3]-reached[:3,3])>.002 or rotation_error(expected[:3,:3],reached[:3,:3])>np.deg2rad(2):
                raise ValueError('放置航点实际位置或朝向未到位')
            if waypoint['name']=='place':
                self.check_place_contact(held,config);self.check_cancel()
                report['active_stage']='张开夹爪释放物品'
                self.update(state='releasing',message='物品已到桌面放置位置，正在张开夹爪…')
                report['release_attempted']=True
                with self.lock:self.held_object=None
                self.robot.move_gripper(settings['release_position'])
                deadline=time.monotonic()+settings['release_timeout_s']
                while time.monotonic()<deadline:
                    self.check_cancel();state=self.robot.query('get_rm_plus_state_info')['state_info']
                    if contact_result(state,config['gripper'])=='fault':raise ValueError('释放物品时夹爪异常')
                    if state.get('speed')==[0] and abs(state['pos'][0]-settings['release_position'])<=3:
                        report['release_confirmed']=True;break
                    time.sleep(.1)
                else:raise TimeoutError('夹爪张开未到位，未执行退出动作')
        self.update(state='completed',message='已按指定朝向执行放置、松开并退出；请观察物品是否稳定放置。')
