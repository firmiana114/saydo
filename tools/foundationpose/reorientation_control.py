"""Fixed-TCP object orientation shares task locks and preserves successful holds."""
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
from reorientation_visual import model_appearance
from reorientation_geometry import model_geometry, checked_rotation, reorientation_settings, plan_reorientation, endpoint_reachability, checked_position_request, segment_distance


class ReorientationMixin:
    def held_snapshot(self):
        if self.held_object is None:
            return None
        held = self.held_object
        return {k: copy.deepcopy(held[k]) for k in ['hold_id','object_id','object_name','model_sha256',
                'base_T_object_at_grasp','object_T_tcp','current_object_rotation','orientation_version'] if k in held}

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
        held = copy.deepcopy(report.get('held_object'))
        if held is not None:
            checks = report.get('arrival_checks', [])
            arm = checks[-1].get('actual_arm_state') if checks else None
            if arm is None:
                arm = self.arm_state()
            current = controller_pose(arm['pose']) @ np.linalg.inv(transform(held['object_T_tcp']))
            held.update(current_object_rotation=current[:3,:3].tolist(), orientation_version=0)
        with self.lock:
            self.held_object = held

    def reorientation_model(self, object_id=None):
        with self.lock:
            held = copy.deepcopy(self.held_object)
        if not object_id:
            object_id = held['object_id'] if held else self.tracker.state.get('object_id')
        geometry = model_geometry(self.tracker.models_root, object_id)
        try:
            geometry['appearance'] = model_appearance(self.tracker.models_root, object_id)
        except (ValueError, OSError, TypeError) as error:
            geometry['appearance'] = {'status': 'unavailable', 'error': str(error)}
        geometry['initial_rotation'] = (copy.deepcopy(held.get('current_object_rotation', np.asarray(held['base_T_object_at_grasp'])[:3,:3].tolist()))
                                        if held and held['object_id']==object_id else np.eye(3).tolist())
        geometry['holding'] = bool(held and held['object_id']==object_id)
        geometry['pivot_object_m'] = (np.asarray(held['object_T_tcp'])[:3,3].tolist()
                                      if geometry['holding'] else None)
        geometry['preview_mode'] = 'adaptive_or_fixed_orientation'
        settings = reorientation_settings(json.loads((self.root/'config/grasp/reorientation.json').read_text()))
        geometry['max_shift_m'] = settings['adaptive_max_shift_m']
        return geometry

    def start_reorient(self, value, preview=False):
        rotation = checked_rotation(value.get('rotation'))
        with self.lock:
            if self.status['busy']:
                raise ValueError('已有机械臂任务正在处理')
            held = copy.deepcopy(self.held_object)
            if held is None:
                raise ValueError('本次服务尚无成功抓取记录，请先抓取物品后再调整姿态')
            if value.get('hold_id') != held['hold_id'] or value.get('object_id') != held['object_id'] or value.get('model_sha256') != held['model_sha256']:
                raise ValueError('预览模型或夹持记录已变化，请重新加载物品模型')
            settings = reorientation_settings(json.loads((self.root/'config/grasp/reorientation.json').read_text()))
            position_mode,max_shift_m = checked_position_request(value.get('position_mode','fixed'),value.get('max_shift_m'),settings)
            self.cancelled.clear();self.actuated=False;self.robot=None
            self.status.update(state='reorient_planning',operation='reorient',busy=True,error=None,plan=None,
                               reorientation_plan=None,target_object_id=held['object_id'],
                               message='正在检查目标朝向，优先尝试原地旋转…')
            self.thread = threading.Thread(target=self.run_reorient,args=(held,rotation.tolist(),preview,position_mode,max_shift_m),daemon=True)
            self.thread.start()
        return {'accepted':True,'mode':'preview' if preview or self.mode=='preview' else 'execute'}

    def reorientation_hardware(self, held, config):
        if self.robot.query('get_arm_software_info').get('Product_version') != 'RM75-BI':
            raise ValueError('机械臂型号不匹配')
        if self.robot.query('get_joint_en_state').get('en_state') != [1]*7 or self.robot.query('get_joint_err_flag').get('err_flag') != [0]*7:
            raise ValueError('关节使能或错误状态异常')
        tool = yaml.safe_load((self.root/config['tool_file']).read_text())
        if self.robot.query('get_current_tool_frame') != held['tool_response'] or tool['controller_readback']['tool_response_raw'] != held['tool_response']:
            raise ValueError('工具/负载已变化，不能沿用抓取关系调整姿态')
        work = self.robot.query('get_current_work_frame')
        if work.get('frame_name') != 'Base' or work.get('pose') != [0]*6:
            raise ValueError('姿态调整要求零偏移 Base 工作坐标系')
        if self.robot.query('get_rm_plus_mode').get('mode') != 115200:
            raise ValueError('夹爪通信模式不匹配')
        base = self.robot.query('get_rm_plus_base_info')['base_info'];grip=config['gripper']
        if (base.get('manu'),base.get('id'),base.get('dof'),base.get('type')) != (grip['vendor'],grip['device_id'],1,1):
            raise ValueError('夹爪身份不匹配')
        self.check_reorient_contact(held,config)
        first=self.arm_state();time.sleep(.15);last=self.arm_state()
        a,b=np.asarray(first['joint'],dtype=float),np.asarray(last['joint'],dtype=float)
        if a.shape!=(7,) or b.shape!=(7,) or not np.isfinite([a,b]).all() or np.max(np.abs(a-b))>30:
            raise ValueError('机械臂未停稳，不能规划姿态调整')
        return last,tool

    def check_reorient_contact(self, held, config):
        state=self.robot.query('get_rm_plus_state_info')['state_info']
        if contact_result(state,config['gripper']) not in ('force_contact','force_stop') or abs(state['pos'][0]-held['contact_position'])>20:
            with self.lock:self.held_object=None
            raise ValueError('夹持反馈已变化或未保持夹持，停止姿态调整')
        return state

    def run_reorient(self, held, rotation, preview=False, position_mode='fixed', max_shift_m=None):
        mode='preview' if preview else self.mode
        report={'started_at':time.time(),'operation':'reorient','mode':mode,'held_object':held,
                'requested_rotation':rotation,'position_mode':position_mode,'max_shift_m':max_shift_m,'robot_motion_sent':False,'active_stage':'姿态调整前检查'}
        directory=self.root/'work/grasp/reorient_jobs';robot_lock=None
        try:
            directory.mkdir(parents=True,exist_ok=True)
            robot_lock=(directory.parent/'robot.lock').open('a')
            fcntl.flock(robot_lock,fcntl.LOCK_EX|fcntl.LOCK_NB)
            config=json.loads(self.config_path.read_text())
            settings=reorientation_settings(json.loads((self.root/'config/grasp/reorientation.json').read_text()))
            report['settings']=settings
            self.robot=RobotClient(config['robot_ip'],config['robot_port'],writes=mode=='execute')
            self.robot.motion_log=report.setdefault('motion_events',[])
            self.check_cancel()
            arm,tool=self.reorientation_hardware(held,config)
            report['initial_arm']=copy.deepcopy(arm)
            report['tool']=copy.deepcopy(tool)
            mesh=model_geometry(self.tracker.models_root,held['object_id'])
            if mesh['model_sha256']!=held['model_sha256']:
                raise ValueError('物品模型已变化，不能沿用抓取关系调整姿态')
            checker=(self.root/config['ik_checker']).resolve()
            if not checker.is_file():raise ValueError('缺少姿态调整航点可达性检查程序')
            report['active_stage']='检查姿态路径及允许的位置范围'
            plan=plan_reorientation(held,mesh['vertices'],rotation,arm,config,settings,
                lambda waypoints,timeout:endpoint_reachability(checker,tool,arm,waypoints,timeout),self.check_cancel,position_mode,max_shift_m,report.setdefault('search_attempts',[]))
            plan.update(object_id=held['object_id'],object_name=held['object_name'],hold_id=held['hold_id'],
                        can_execute=mode=='execute')
            report['plan']=plan
            self.update(reorientation_plan=plan)
            self.check_cancel()
            if mode!='execute':
                self.update(state='reorient_preview',message=f"方案已生成：末端移位 {plan['offset_distance_m']*100:.1f} 厘米后调整至指定姿态，继续夹持；未发送运动。")
                return
            if np.max(np.abs(np.asarray(self.arm_state()['joint'])-arm['joint']))>30:
                raise ValueError('机械臂在姿态调整规划后发生移动')
            self.check_reorient_contact(held,config)
            self.execute_reorient(plan,held,config,settings,report)
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

    def execute_reorient(self, plan, held, config, settings, report):
        start = np.asarray(plan.get('start_tcp_position_m',plan.get('fixed_tcp_position_m')))
        # Keep tests and archived fixed-position plans usable without any translation fallback.
        if start.shape != (3,):
            raise ValueError('姿态计划缺少有效的起始位置')
        tolerance = settings['position_tolerance_m']
        initial_rotation = controller_pose(plan['current_arm']['pose'])[:3,:3]
        reached = None
        for target in plan['waypoints']:
            end = np.asarray(target['position_m'])
            translating = target['name']=='translate'
            def watchdog():
                self.check_cancel()
                actual = self.arm_state()
                pose = controller_pose(actual['pose'])
                deviation = segment_distance(pose[:3,3],start,end)
                report['max_observed_path_deviation_m'] = max(report.get('max_observed_path_deviation_m',0.),deviation)
                if deviation>tolerance:
                    raise ValueError(f'末端偏离规划路径 {deviation*1000:.2f} 毫米，超过允许值 {tolerance*1000:g} 毫米')
                if translating and rotation_error(initial_rotation,pose[:3,:3])>np.deg2rad(settings['orientation_tolerance_deg']):
                    raise ValueError('移位阶段的末端朝向发生异常变化')
                self.check_reorient_contact(held,config)
                return pose
            report['active_stage'] = '保持夹持移至可旋转位置' if translating else '在选定位置调整物品姿态'
            self.update(state='reorienting',message=report['active_stage']+'…')
            # Reject a stale plan before its first command, not just a point anywhere along its path.
            reached=watchdog()
            if np.linalg.norm(reached[:3,3]-start)>tolerance:
                raise ValueError('末端已离开本段规划起点')
            self.actuated = True
            report['robot_motion_sent'] = True
            self.robot.motion_stage = report['active_stage']
            self.robot.move_linear(target['controller_pose'],settings['arm_speed_percent'],self.cancelled,watchdog)
            reached = watchdog()
            expected = controller_pose(target['controller_pose'])
            report.setdefault('arrival_checks',[]).append({'stage':report['active_stage'],'target_pose':target['controller_pose'],'actual_tcp':reached.tolist()})
            if np.linalg.norm(expected[:3,3]-reached[:3,3])>tolerance:
                raise ValueError('末端目标位置未到位')
            if rotation_error(expected[:3,:3],reached[:3,:3])>np.deg2rad(settings['orientation_tolerance_deg']):
                raise ValueError('末端目标朝向未到位')
            start=end
        actual_object = reached @ np.linalg.inv(transform(held['object_T_tcp']))
        updated = copy.deepcopy(held)
        updated['current_object_rotation'] = actual_object[:3,:3].tolist()
        updated['orientation_version'] = held.get('orientation_version',0)+1
        with self.lock:
            self.held_object = updated
        shifted=plan.get('offset_distance_m',0.)
        self.update(state='completed',message=f'已将物品摆至指定姿态，末端移位 {shifted*100:.1f} 厘米，夹爪继续夹持。')
