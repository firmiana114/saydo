"""Plan and execute general held-object movements and explicit release tasks."""
import copy
import fcntl
import hashlib
import json
from pathlib import Path
import threading
import time
import traceback
import uuid

import numpy as np
from grasp_geometry import controller_pose, transform, rotation_error, object_in_base
from grasp_library import held_clearance_config
from grasp_robot import RobotClient
from motion_validation import check_waypoints, wait_for_pose, note_joint_change
from object_catalog import select_object
from scene_features import object_features
from object_motion_geometry import plan_object_motion, segment_samples, validate_tcp, object_error
from object_motion_store import ObjectPlanStore
from placement_geometry import check_tcp_workspace
from reorientation_geometry import model_geometry
from tools.robot_agent.motion_contract import checked_goal, identity, vector


def public_object_plan(plan):
    if plan is None:return None
    result={key:copy.deepcopy(value) for key,value in plan.items() if key not in ('search_attempts','reachability_check')}
    if 'hold' in result:result['hold']={k:result['hold'][k] for k in ('object_id','hold_id','orientation_version','model_sha256') if k in result['hold']}
    result['reachability_check']={k:plan.get('reachability_check',{}).get(k) for k in ('verified','max_joint_step_deg','joint_travel_deg')}
    result['search_attempt_count']=len(plan.get('search_attempts',[]))
    return result


class ObjectMotionMixin:
    def object_plan_store(self):
        if not hasattr(self,'_object_plan_store'):self._object_plan_store=ObjectPlanStore(self.root)
        return self._object_plan_store

    def require_motion_hold(self, args):
        identity(args);held=copy.deepcopy(self.held_object)
        if held is None or any(args[k]!=held.get(k,0 if k=='orientation_version' else None) for k in ('object_id','hold_id','orientation_version')):
            raise ValueError('物品、夹持关系或版本已变化，请读取状态后重新规划')
        transform(held['object_T_tcp'])
        return held

    def object_geometry_info(self, object_id):
        geometry=model_geometry(self.tracker.models_root,object_id);v=np.asarray(geometry['vertices']);lo=v.min(axis=0);hi=v.max(axis=0)
        metadata=None
        path=Path(self.tracker.models_root)/geometry['object_id']/'dimensions.json'
        if path.is_file() and not path.is_symlink() and path.stat().st_size<=65536:
            value=json.loads(path.read_text())
            if isinstance(value,dict):metadata={k:value[k] for k in ('origin','axis','axes','units','size_m','length_m','height_m','outer_diameter_m') if k in value}
        return {k:geometry[k] for k in ('object_id','object_name','model_sha256','pose_mode','pose_mode_revision','units')}|{
            **object_features(self.tracker.models_root,object_id),'coordinate_frame':'object_model','bounds_object_m':[lo.tolist(),hi.tolist()],
            'declared_model_dimensions':metadata,
            'reference_points':{'model_origin':[0,0,0],'bounding_box_center':((lo+hi)/2).tolist(),
                'axis_z_min':[0,0,float(lo[2])],'axis_z_max':[0,0,float(hi[2])]},
            'note':'参考点为模型坐标/包围盒特征，不是已标定的接触点；5D参考点须在模型Z轴上。'}

    def start_object_plan(self, args, release=False, request_id=None):
        goal=checked_goal(args.get('goal'),release);retreat=vector(args.get('retreat_vector_base_m',[0,0,0]),'retreat_vector_base_m',.3)
        if np.linalg.norm(retreat)>.3:raise ValueError('退出位移长度不得超过300毫米')
        with self.lock:
            if self.status['busy'] or self.status.get('stopping'):raise ValueError('机械臂任务忙碌或停止未确认')
            held=self.require_motion_hold(args)
            self.cancelled.clear();self.actuated=False;self.robot=None
            self.object_plan_store()
            self.status.update(state='object_motion_planning',operation='plan_release' if release else 'plan_held_object_motion',
                busy=True,error=None,plan=None,object_motion_plan=None,execution_result=None,diagnostics_ref=None,recovery=None,
                target_object_id=held['object_id'],
                message='正在规划物品目标位姿和释放退出' if release else '正在规划持物目标位置与朝向')
            self.thread=threading.Thread(target=self.run_object_plan,args=(held,goal,release,retreat,request_id),daemon=True);self.thread.start()
        return {'accepted':True,'motion_sent':False}

    def start_execute_object_plan(self, plan_id, request_id):
        with self.lock:
            if self.status['busy'] or self.status.get('stopping'):raise ValueError('机械臂任务忙碌或停止未确认')
            if self.mode!='execute':raise ValueError('当前后端模式不允许执行动作')
            plan=self.object_plan_store().get(plan_id);self.require_motion_hold(plan['hold'])
            plan=self.object_plan_store().claim(plan_id,request_id)
            self.cancelled.clear();self.actuated=False;self.robot=None
            self.status.update(state='object_motion_checking',operation='execute_plan',busy=True,error=None,plan=None,
                object_motion_plan=plan,target_object_id=plan['hold']['object_id'],recovery=None,execution_result=None,diagnostics_ref=None,message='检查方案起点、夹持及配置是否仍一致')
            self.thread=threading.Thread(target=self.run_object_execution,args=(plan,request_id),daemon=True);self.thread.start()
        return {'accepted':True,'plan_id':plan_id}

    def motion_binding(self, config):
        paths={'trial':self.config_path,'tool':self.root/config['tool_file'],'handeye':self.root/config['handeye_file'],
               'camera_reuse':self.root/'config/calibration/camera_reuse.yaml'}
        return {key:hashlib.sha256(p.read_bytes()).hexdigest() if p.exists() else None for key,p in paths.items()}

    def motion_context(self, held, writes=False):
        config=json.loads(self.config_path.read_text());config=held_clearance_config(config,held)
        if held.get('scene_id'):self.validate_scene(held['scene_id'],config)
        self.robot=RobotClient(config['robot_ip'],config['robot_port'],writes=writes)
        arm,tool=self.reorientation_hardware(held,config)
        source=tool['controller_readback']
        if hashlib.sha256((self.root/source['path']).read_bytes()).hexdigest()!=source['sha256']:raise ValueError('工具原始读回记录哈希变化')
        geometry=model_geometry(self.tracker.models_root,held['object_id'])
        if geometry['model_sha256']!=held['model_sha256'] or geometry.get('pose_mode','6d')!=held.get('pose_mode','6d'):
            raise ValueError('持物模型或位姿模式已变化')
        checker=(self.root/config['ik_checker']).resolve()
        if not checker.is_file():raise ValueError('缺少离线运动学检查器')
        return config,arm,tool,geometry,checker

    def resolve_motion_frame(self, goal, held, config, arm, tool):
        name=goal['reference_frame'];context={'name':name,'captured_at':time.time()};frame=np.eye(4)
        if held.get('scene_id') and name in ('camera','tracked_object'):raise ValueError('快照任务不使用实时相机/跟踪目标，请引用开始时采集的scene_object')
        if name=='scene_object':
            if held.get('scene_id')!=goal['scene_id']:raise ValueError('参考快照必须与当前夹持来源一致')
            frame,context=self.scene_reference(goal)
        elif name=='tcp':frame=controller_pose(arm['pose'])
        elif name=='held_object':
            if held.get('pose_mode')=='5d':raise ValueError('5D物品横向轴不可观测，请使用base/camera/tcp坐标系')
            frame=controller_pose(arm['pose'])@np.linalg.inv(transform(held['object_T_tcp']))
        elif name in ('camera','tracked_object'):
            handeye,_=self.calibration(config)
            if name=='tracked_object':
                observation=copy.deepcopy(self.tracker.grasp_observation())
                if observation['object_id']!=goal['reference_object_id']:raise ValueError('参考物品不是当前被稳定跟踪的物品')
                if observation.get('pose_mode','6d')=='5d':raise ValueError('参考物品为5D，不能用其不确定的横向坐标轴作为完整参考系')
                if observation['object_id']==held['object_id']:raise ValueError('被夹物品请使用held_object坐标系或base，不能重复作为外部跟踪参考')
                arm,observation=self.paired_observation(observation)
                frame=object_in_base(controller_pose(arm['pose']),tool['flange_T_grip_tcp'],handeye['flange_T_color_optical_camera'],observation['pose'])
                context.update(object_id=observation['object_id'],generation=observation['generation'],model_sha256=observation['model_sha256'],pose_mode_revision=observation.get('pose_mode_revision'),captured_at=observation['time'])
            else:
                if time.time()-self.tracker.state.get('camera_frame_at',0)>.5:raise ValueError('没有新鲜相机坐标观测')
                frame=controller_pose(arm['pose'])@np.linalg.inv(transform(tool['flange_T_grip_tcp']))@transform(handeye['flange_T_color_optical_camera'])
                context['captured_at']=self.tracker.state['camera_frame_at']
        context['base_T_reference']=frame.tolist()
        return frame,context,arm

    def save_object_job(self, report):
        directory=self.root/'work/grasp/object_motion_jobs';directory.mkdir(parents=True,exist_ok=True)
        name=f'{time.time_ns()}_{uuid.uuid4().hex[:6]}.json';path=directory/name
        report['finished_at']=time.time();report['result']={**self.status,'busy':False}
        path.write_text(json.dumps(report,ensure_ascii=False,indent=2,allow_nan=False)+'\n')
        return {'directory':'object_motion_jobs','file':name}

    def run_object_plan(self, held, goal, release, retreat, request_id):
        report={'started_at':time.time(),'request_id':request_id,'operation':'plan_release' if release else 'plan_held_object_motion','robot_motion_sent':False}
        try:
            directory=self.root/'work/grasp';directory.mkdir(parents=True,exist_ok=True)
            with (directory/'robot.lock').open('a') as lock:
                fcntl.flock(lock,fcntl.LOCK_EX|fcntl.LOCK_NB)
                config,arm,tool,geometry,checker=self.motion_context(held)
                binding=self.motion_binding(config)
                frame,context,arm=self.resolve_motion_frame(goal,held,config,arm,tool)
                report.update(goal=goal,held_object=held,initial_arm=arm,reference_context=context,
                              configuration_binding=binding,flange_T_grip_tcp=tool['flange_T_grip_tcp'])
                plan=plan_object_motion(held,geometry['vertices'],arm,config,goal,frame,
                    lambda points,timeout:check_waypoints(checker,tool,arm,points,timeout),self.check_cancel,release,retreat,
                    report.setdefault('search_attempts',[]))
                self.check_cancel()
                if self.motion_binding(config)!=binding:raise ValueError('规划期间配置已变化')
                plan.update(hold=held,configuration_binding=binding,reference_context=context,
                            flange_T_grip_tcp=tool['flange_T_grip_tcp'],model_sha256=held['model_sha256'],pose_mode_revision=geometry.get('pose_mode_revision'))
                plan=self.object_plan_store().save(plan);report['plan']=plan
                self.update(state='object_motion_planned',object_motion_plan=plan,message='方案已保存，未发送运动；使用plan_id执行，有效期5分钟')
        except Exception as error:
            report.update(error=str(error),traceback=traceback.format_exc())
            self.update(state='cancelled' if self.cancelled.is_set() else 'failed',error=str(error),message='持物规划失败：'+str(error))
        finally:
            try:self.update(diagnostics_ref=self.save_object_job(report))
            except Exception as error:self.update(state='failed',error='诊断记录失败：'+str(error),object_motion_plan=None)
            self.update(busy=False)

    def check_object_plan_binding(self, plan, config):
        self.check_cancel()
        if plan['hold'].get('scene_id'):self.validate_scene(plan['hold']['scene_id'],config)
        if plan['goal']['reference_frame']=='scene_object':
            frame,_=self.scene_reference(plan['goal'])
            if not np.allclose(frame,plan['reference_frame_pose'],atol=1e-9):raise ValueError('快照参考位姿变化')
        with self.lock:current_hold=self.require_motion_hold(plan['hold'])
        if current_hold['object_T_tcp']!=plan['hold']['object_T_tcp'] or current_hold.get('tool_response')!=plan['hold'].get('tool_response'):raise ValueError('夹持变换或工具绑定已变化')
        if self.motion_binding(config)!=plan['configuration_binding']:raise ValueError('工具、标定或运动配置已变化，请重新规划')
        current=select_object(self.tracker.models_root,plan['hold']['object_id'])
        if current['pose_mode']!=plan['pose_mode'] or current.get('pose_mode_revision')!=plan.get('pose_mode_revision'):raise ValueError('物品位姿模式或模型设置版本已变化')
        if hashlib.sha256(Path(current['mesh']).read_bytes()).hexdigest()!=plan['model_sha256']:raise ValueError('物品模型已变化')

    def verify_object_target(self, actual, plan):
        error=object_error(actual,plan['base_T_object_target'],plan['reference_point_object_m'],plan['pose_mode'])
        if error['reference_position_error_m']>plan['position_tolerance_m'] or error['orientation_error_deg']>plan['orientation_tolerance_deg']:
            raise ValueError('实际物品参考点/朝向未到达目标：'+str(error))
        return error

    def run_object_execution(self, plan, request_id, nested=False):
        held=copy.deepcopy(plan['hold']);report={'started_at':time.time(),'request_id':request_id,'operation':'execute_plan',
            'plan_id':plan['plan_id'],'plan':plan,'held_object':held,'robot_motion_sent':False,'gripper_command_attempted':False,'release_attempted':False,'release_confirmed':False}
        lock=None
        try:
            if not nested:
                lock=(self.root/'work/grasp/robot.lock').open('a')
                fcntl.flock(lock,fcntl.LOCK_EX|fcntl.LOCK_NB)
            config,arm,tool,geometry,checker=self.motion_context(held,True);self.robot.motion_log=report.setdefault('motion_events',[])
            self.check_object_plan_binding(plan,config)
            expected=controller_pose(plan['current_arm']['pose']);actual=controller_pose(arm['pose'])
            if np.max(np.abs(np.asarray(arm['joint'])-plan['current_arm']['joint']))>500 or np.linalg.norm(actual[:3,3]-expected[:3,3])>.002 or rotation_error(actual[:3,:3],expected[:3,:3])>np.deg2rad(2):
                raise ValueError('机械臂已离开方案起点，请重新规划')
            if plan['goal']['reference_frame'] in ('camera','tracked_object'):
                frame,context,arm=self.resolve_motion_frame(plan['goal'],held,config,arm,tool)
                previous=transform(plan['reference_frame_pose'])
                if np.linalg.norm(frame[:3,3]-previous[:3,3])>.002 or rotation_error(frame[:3,:3],previous[:3,:3])>np.deg2rad(2):raise ValueError('参考坐标物品/相机已移动，请重新规划')
                if any(context.get(k)!=plan['reference_context'].get(k) for k in ('model_sha256','generation','pose_mode_revision')):raise ValueError('参考物品模型或观测版本已变化')
            actual=controller_pose(arm['pose'])
            if np.max(np.abs(np.asarray(arm['joint'])-plan['current_arm']['joint']))>500 or np.linalg.norm(actual[:3,3]-expected[:3,3])>.002 or rotation_error(actual[:3,:3],expected[:3,:3])>np.deg2rad(2):raise ValueError('参考坐标查询期间机械臂离开方案起点')
            previous_feedback=copy.deepcopy(arm)
            attachment_inv=np.linalg.inv(transform(held['object_T_tcp']));deadline=time.monotonic()+plan['max_execution_duration_s']
            def read_state(released=False):
                self.check_cancel()
                if getattr(self,'agent_recording_error',None):raise ValueError('任务状态记录失败，停止后续动作')
                if time.monotonic()>deadline:raise TimeoutError('方案执行超过%d秒预算'%plan['max_execution_duration_s'])
                if self.motion_binding(config)!=plan['configuration_binding']:raise ValueError('执行期间配置变化，停止后续动作')
                a=self.arm_state();tcp=controller_pose(a['pose']);check_tcp_workspace(tcp,config)
                if released:self.check_placement_open(config,require_open=True)
                else:
                    self.check_reorient_contact(held,config)
                    # Workspace and model/table checks apply to observed motion too.
                    validate_tcp(tcp,held,geometry['vertices'],config)
                return a
            def move(target,released=False):
                nonlocal previous_feedback
                report['active_stage']=target['name']
                start=read_state(released)
                before=controller_pose(start['pose']);previous=controller_pose(previous_feedback['pose'])
                if np.max(np.abs(np.asarray(start['joint'])-previous_feedback['joint']))>500 or np.linalg.norm(before[:3,3]-previous[:3,3])>.002 or rotation_error(before[:3,:3],previous[:3,:3])>np.deg2rad(2):raise ValueError('机械臂离开本段规划起点')
                end=controller_pose(target['controller_pose'])
                points=segment_samples(controller_pose(start['pose']),end,None if released else held,
                    geometry['vertices'],config,target['method'],target['name'])
                current_endpoint_index=len(points)-1
                # Revalidate the remaining suffix from real feedback. movel is
                # solved by the controller; do not assume it chose the offline branch.
                if not released:
                    last=end
                    for future in plan['waypoints'][report['active_waypoint_index']:]:
                        future_pose=controller_pose(future['controller_pose'])
                        points.extend(segment_samples(last,future_pose,held,geometry['vertices'],config,future['method'],future['name']))
                        last=future_pose
                    if plan.get('retreat_waypoint'):
                        future=plan['retreat_waypoint'];future_pose=controller_pose(future['controller_pose'])
                        points.extend(segment_samples(last,future_pose,None,None,config,'linear',future['name']))
                detail=self.verify_motion_targets(plan,config,report,start,points)
                fixed=plan.get('fixed_joint_targets',{}).get(target['name'])
                # A fixed plan follows the remaining path as re-solved from real feedback above and
                # drives straight segments through their sampled joint sets instead of movel, whose
                # redundancy the controller resolves on its own; the planned set is a diagnostic reference.
                if fixed is not None:note_joint_change(report,target['name'],'resolved_remaining_path_vs_plan',fixed,np.asarray(detail['joint_solutions_deg'][current_endpoint_index])*1000)
                self.check_cancel()
                if not self.actuated and time.time()>plan['expires_at']:raise ValueError('方案已过期，请重新规划')
                report['active_stage']=target['name'];self.robot.motion_stage=target['name']
                self.update(state='object_motion_executing',message='执行方案：'+target['name'])
                start_pose=controller_pose(start['pose'])
                def observed():
                    feedback=read_state(released);pose=controller_pose(feedback['pose'])
                    if target['method']=='linear':
                        delta=end[:3,3]-start_pose[:3,3]
                        fraction=np.clip(np.dot(pose[:3,3]-start_pose[:3,3],delta)/max(np.dot(delta,delta),1e-12),0,1)
                        if np.linalg.norm(pose[:3,3]-start_pose[:3,3]-fraction*delta)>.01:raise ValueError('实际反馈偏离规划直线超过10毫米')
                        if (target['name'].startswith(('transfer_','release_preplacement','release_descent')) or rotation_error(start_pose[:3,:3],end[:3,:3])<1e-6) and rotation_error(pose[:3,:3],end[:3,:3])>np.deg2rad(plan['orientation_tolerance_deg']):raise ValueError('直线段实际朝向偏离目标')
                    return feedback
                joint_target=None
                if target['method']=='linear' and fixed is not None:
                    self.actuated=True;report['robot_motion_sent']=True
                    joint_target=self.follow_joint_samples(detail['joint_solutions_deg'][:current_endpoint_index+1],plan['speed_percent'],observed,report,target['name'])
                elif target['method']=='linear':
                    self.actuated=True;report['robot_motion_sent']=True
                    self.robot.move_linear(target['controller_pose'],plan['speed_percent'],self.cancelled,observed)
                else:
                    solutions=detail.get('joint_solutions_deg') if isinstance(detail,dict) else None
                    if not solutions or len(solutions)<=current_endpoint_index or len(solutions[current_endpoint_index])!=7 or not np.isfinite(solutions[current_endpoint_index]).all():raise ValueError('缺少已验证的关节解，不能执行关节方案')
                    joint_target=np.rint(np.asarray(solutions[current_endpoint_index])*1000).astype(int).tolist()
                    report.setdefault('commanded_joint_targets',[]).append({'stage':target['name'],'joint_raw':joint_target})
                    self.actuated=True;report['robot_motion_sent']=True
                    self.robot.move_joints(joint_target,plan['speed_percent'],self.cancelled,observed)
                record={'stage':target['name']};report.setdefault('arrival_checks',[]).append(record)
                reached=wait_for_pose(observed,target['controller_pose'],self.check_cancel,record,
                    plan['position_tolerance_m'],np.deg2rad(plan['orientation_tolerance_deg']))
                if joint_target is not None and np.max(np.abs(np.asarray(reached['joint'])-joint_target))>500:raise ValueError('关节方案未到达所选关节解')
                previous_feedback=copy.deepcopy(reached)
                report['completed_waypoints']=report.get('completed_waypoints',0)+1
                return reached
            for index,target in enumerate(plan['waypoints'],1):
                report['active_waypoint_index']=index;arm=move(target)
            from reorientation_geometry import pose_waypoint
            final_tcp=transform(plan['base_T_object_target'])@transform(held['object_T_tcp'])
            final_check={'stage':'verify_object_target'};report.setdefault('arrival_checks',[]).append(final_check)
            arm=wait_for_pose(read_state,pose_waypoint('target',final_tcp)['controller_pose'],self.check_cancel,final_check,plan['position_tolerance_m'],np.deg2rad(plan['orientation_tolerance_deg']))
            previous_feedback=copy.deepcopy(arm)
            actual_object=controller_pose(arm['pose'])@attachment_inv
            errors=self.verify_object_target(actual_object,plan)
            if plan['goal']['position_mode']=='adaptive' and controller_pose(arm['pose'])[2,3]<=config['table_z_m']+.10:raise ValueError('实际自适应终点TCP未高于桌面10厘米')
            result={'plan_id':plan['plan_id'],'kind':plan['kind'],'object_id':held['object_id'],'hold_id':held['hold_id'],
                'target_verified':True,'actual_object_pose_before_release':actual_object.tolist(),**errors,
                'release_confirmed':False,'object_settled_pose_verified':False,'retreat_confirmed':False}
            if plan['kind']=='release':
                self.check_cancel()
                if not self.actuated and time.time()>plan['expires_at']:raise ValueError('释放方案已过期')
                report['active_stage']='open_gripper';self.actuated=True;report['release_attempted']=True;report['gripper_command_attempted']=True
                report['release_requested_at']=time.time()
                self.update_scene_instance(held,'release_unknown',actual_object)
                with self.lock:self.held_object=None
                self.robot.move_gripper(1000);report['release_command_acknowledged']=True
                stop=time.monotonic()+8;stable=0
                report['release_samples']=[]
                while time.monotonic()<stop:
                    self.check_cancel();grip=self.check_placement_open(config)
                    report['release_samples'].append({'at':time.time(),'state':copy.deepcopy(grip)})
                    stable=stable+1 if grip['speed']==[0] and abs(grip['pos'][0]-1000)<=3 else 0
                    if stable>=3:report['release_confirmed']=True;break
                    time.sleep(.1)
                if not report['release_confirmed']:raise TimeoutError('未确认夹爪完全张开，不执行退出')
                result['release_confirmed']=True
                if plan['retreat_waypoint'] is not None:
                    # Correct only the explicitly planned relative retreat, then revalidate.
                    from reorientation_geometry import pose_waypoint
                    if plan.get('fixed_joint_targets'):
                        target=plan['retreat_waypoint']
                    else:
                        retreat=controller_pose(arm['pose']);retreat[:3,3]+=plan['retreat_vector_base_m']
                        target=pose_waypoint('release_retreat',retreat);target['method']='linear'
                    move(target,True)
                result['retreat_confirmed']=True
                self.update_scene_instance(held,'placed_unverified',actual_object)
            else:
                updated=copy.deepcopy(held);updated.update(current_object_rotation=actual_object[:3,:3].tolist(),orientation_version=held['orientation_version']+1)
                with self.lock:self.held_object=updated
                result['orientation_version']=updated['orientation_version']
            self.object_plan_store().finish(plan['plan_id'],'completed')
            report['execution_result']=result;self.update(state='completed',execution_result=result,
                message='已到达物品目标位姿并确认松爪/退出；松爪后的物品位姿尚未独立观测' if plan['kind']=='release' else '已到达物品目标位姿，保持夹持')
        except Exception as error:
            report.update(error=str(error),traceback=traceback.format_exc())
            if self.actuated:
                try:self.robot.stop();report['stop_acknowledged']=True
                except Exception as stop_error:report['stop_error']=str(stop_error)
                # A nested release can fail before its own context exists while the grasp has already
                # actuated; the hold is then left as recorded instead of masking the real error.
                if not report['release_attempted'] and 'config' in locals():
                    self.retain_reorientation_hold(held,config,report)
            try:self.object_plan_store().finish(plan['plan_id'],'failed')
            except Exception as store_error:report['plan_recording_error']=str(store_error)
            recovery=self.assess_motion_recovery(held,config,report) if 'config' in locals() else {'can_replan':False}
            self.update(state='cancelled' if self.cancelled.is_set() else 'failed',error=str(error),message='方案执行失败：'+str(error),execution_result=None,recovery=recovery)
        finally:
            if lock is not None:lock.close()
            try:self.update(diagnostics_ref=self.save_object_job(report))
            except Exception as error:self.update(state='failed',error='诊断记录失败：'+str(error),execution_result=None,recovery={'can_replan':False,'outcome_unknown':True})
            if not nested:self.update(busy=False)
