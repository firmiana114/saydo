"""Plan a complete scene-based pick/place before opening or moving the gripper."""
import copy
import fcntl
import json
import random
import threading
import time
import uuid

import numpy as np

from grasp_geometry import controller_pose, transform, rotation_error
from grasp_robot import RobotClient
from grasp_library import revision
from motion_validation import check_waypoints
from object_motion_geometry import segment_samples, plan_object_motion
from reorientation_geometry import model_geometry
from tools.robot_agent.motion_contract import checked_goal


# Straight segments run as one joint move per sample (about 10 mm apart), so a fixed release
# takes far longer than the 90 s allowed for controller-interpolated plans.
FIXED_RELEASE_BUDGET_S=600


def checked_request(value):
    keys={'scene_id','source_instance_id','reference_instance_id','free_slots','release_gap_m','seed'}
    if not isinstance(value,dict) or set(value)!=keys:raise ValueError('固定流程参数不完整')
    for key in ('scene_id','source_instance_id','reference_instance_id'):
        if not isinstance(value[key],str) or not 1<=len(value[key])<=100:raise ValueError('实例/场景编号无效')
    slots=value['free_slots']
    if not isinstance(slots,list) or not 1<=len(slots)<=80 or any(not isinstance(s,str) or not 1<=len(s)<=90 for s in slots) or len(set(slots))!=len(slots):
        raise ValueError('须提供已确认空闲的孔位列表，不自动推断占用')
    gap=value['release_gap_m']
    if type(gap) not in (float,int) or not np.isfinite(gap) or not 0<=gap<=.005:raise ValueError('支撑面释放间隙须为0至5毫米')
    if type(value['seed']) is not int or not 0<=value['seed']<2**32:raise ValueError('随机种子无效')
    return copy.deepcopy(value)


def placement_goals(scene, request, geometry):
    """Slots use the existing per-model entry/support features; no central model data."""
    rows={r['instance_id']:r for r in scene['instances']}
    source=rows[request['source_instance_id']];rack=rows[request['reference_instance_id']]
    if source['role']!='source' or rack['role']!='reference' or source['state']!='static' or rack['state']!='static':
        raise ValueError('源物品和参照物须是静态场景中的不同实例')
    if rack.get('pose_mode')!='6d':raise ValueError('试管架必须提供完整6D参考坐标系')
    frame=transform(rack['base_T_object'])
    if frame[2,2]<np.cos(np.deg2rad(5)):raise ValueError('试管架Z轴未朝上或倾斜超过5度，不能使用该定位结果')
    points=np.asarray(geometry['vertices']);low=points.min(axis=0);high=points.max(axis=0)
    if source.get('pose_mode')!='5d':raise ValueError('此固定流程要求试管为有方向Z轴的5D模型')
    diameter=2*float(np.linalg.norm(points[:,:2],axis=1).max())
    features={f['id']:f for f in rack['features']};slots=list(request['free_slots']);random.Random(request['seed']).shuffle(slots)
    goals=[]
    for slot in slots:
        entry=features.get(slot+'.entry');support=features.get(slot+'.support')
        if entry is None or support is None:raise ValueError('孔位缺少entry/support模型特征：'+slot)
        e=transform(entry['object_T_feature']);s=transform(support['object_T_feature'])
        hole_diameter=entry.get('diameter_m')
        if type(hole_diameter) not in (int,float) or not np.isfinite(hole_diameter) or hole_diameter<=diameter:
            continue
        if np.linalg.norm(e[:2,3]-s[:2,3])>1e-6 or not 0<e[2,3]-s[2,3]<high[2]-low[2]:
            raise ValueError('孔位轴线/支撑深度与试管模型不一致：'+slot)
        target=s[:3,3]+[0,0,request['release_gap_m']]
        if target[2]>=e[2,3]:raise ValueError('释放位置没有进入孔内')
        # The commanded descent is along Base Z, not a tilted hole axis. The
        # tube's lateral offset while entering must fit the available radius.
        lateral=(e[2,3]-target[2])*np.sqrt(max(0.,1-frame[2,2]**2))/frame[2,2]
        if lateral>(hole_diameter-diameter)/2:continue
        goal=checked_goal({'reference_frame':'scene_object','scene_id':scene['scene_id'],
            'reference_instance_id':rack['instance_id'],'reference_point_object_m':[0,0,float(low[2])],
            'target_position_m':target.tolist(),'target_rpy_deg':[0,0,0],'path_mode':'joint','position_mode':'fixed'},True)
        goals.append((slot,goal,frame))
    if not goals:raise ValueError('没有通过模型直径与竖直下降几何检查的孔位')
    return source,rack,goals


def path_samples(arm, waypoints, held, vertices, config, grasp=False):
    previous=controller_pose(arm['pose']);points=[];ends={}
    for w in waypoints:
        target=controller_pose(w['controller_pose'])
        method=('joint_pose' if w['name']=='pregrasp' else 'linear') if grasp else w['method']
        attachment=held if not grasp or w['name']=='lift' else None
        points.extend(segment_samples(previous,target,attachment,vertices,config,method,w['name']))
        ends[w['name']]=len(points)-1;previous=target
    return points,ends


def joint_targets(points, solutions, waypoints):
    result={}
    for w in waypoints:
        indices=[i for i,p in enumerate(points) if p['name'].startswith(w['name']+'_sample_')]
        if not indices:raise ValueError('固定路径缺少航点采样：'+w['name'])
        result[w['name']]=np.rint(np.asarray(solutions[indices[-1]])*1000).astype(int).tolist()
    return result


class FixedPickPlaceMixin:
    def start_fixed_pick_place(self, args, request_id, execute=False):
        with self.tracker.lock,self.lock:
            if self.status['busy'] or self.status.get('stopping') or self.held_object is not None:
                raise ValueError('固定流程须从空闲且无夹持记录开始')
            if execute:
                if self.mode!='execute':raise ValueError('后端未允许执行')
                plan=self.object_plan_store().get(args['plan_id'])
                if plan.get('kind')!='fixed_pick_place':raise ValueError('不是固定抓放方案')
                plan=self.object_plan_store().claim(args['plan_id'],request_id)
            else:
                args=checked_request(args);self.validate_scene(args['scene_id']);plan=None
            self.cancelled.clear();self.actuated=False;self.robot=None
            self.status.update(state='fixed_pick_place_executing' if execute else 'fixed_pick_place_planning',busy=True,
                operation='execute_fixed_pick_place' if execute else 'plan_fixed_pick_place',error=None,
                fixed_pick_place_plan=None,execution_result=None,recovery=None,diagnostics_ref=None)
            self.thread=threading.Thread(target=self.run_fixed_pick_place,args=(args,request_id,plan),daemon=True);self.thread.start()
        return {'accepted':True,'motion_sent':False}

    def fixed_binding_check(self, plan):
        if self.motion_binding(plan['effective_config'])!=plan['configuration_binding']:raise ValueError('固定流程配置已变化')
        self.validate_scene(plan['scene_id'])
        if revision(self.grasp_library().object(plan['grasp']['object_id']))!=plan['grasp']['grasp_library_revision']:
            raise ValueError('夹法库已变化')

    def plan_fixed_pick_place(self, request, report):
        config=json.loads(self.config_path.read_text());self.robot=RobotClient(config['robot_ip'],config['robot_port'],writes=False)
        binding=self.motion_binding(config);scene=self.validate_scene(request['scene_id'],config)
        source=next(r for r in scene['instances'] if r['instance_id']==request['source_instance_id'])
        geometry=model_geometry(self.tracker.models_root,source['object_id'])
        source,rack,goals=placement_goals(scene,request,geometry)
        observation=self.scene_observation(scene['scene_id'],source['instance_id'],source['object_id'])
        arm=self.stationary_scene_arm();grasp,effective=self.prepare_grasp_plan(observation,config,arm,report)
        if grasp['execution_blockers']:raise ValueError('；'.join(grasp['execution_blockers']))
        grasp_tcp=controller_pose(next(w for w in grasp['waypoints'] if w['name']=='grasp')['controller_pose'])
        held={k:grasp[k] for k in ('object_id','model_sha256','pose_mode','pose_mode_revision','tool_response','scene_id','instance_id')}
        held.update(hold_id='predicted-before-grasp',orientation_version=0,object_T_tcp=(np.linalg.inv(transform(grasp['base_T_object']))@grasp_tcp).tolist(),
                    base_T_object_at_grasp=grasp['base_T_object'],min_table_clearance_m=grasp['effective_parameters']['min_table_clearance_m'])
        prefix,ends=path_samples(arm,grasp['waypoints'],held,geometry['vertices'],effective,True)
        tool={'flange_T_grip_tcp':grasp['flange_T_grip_tcp']};checker=self.root/effective['ik_checker']
        ok,detail=check_waypoints(checker,tool,arm,prefix)
        if not ok:raise ValueError('抓取完整采样路径失败：'+detail['reason'])
        lift=next(w for w in grasp['waypoints'] if w['name']=='lift')
        predicted_arm=copy.deepcopy(arm);predicted_arm.update(pose=lift['controller_pose'],joint=np.rint(np.asarray(detail['joint_solutions_deg'][-1])*1000).astype(int).tolist())
        attempts=report['slot_attempts']=[];full_points=[]
        def reachable(points,timeout):
            nonlocal full_points
            self.check_cancel()
            full_points=prefix+points
            if len(full_points)>480:raise ValueError('全程超过480个采样点')
            return check_waypoints(checker,tool,arm,full_points,timeout)
        for slot,goal,frame in goals:
            self.check_cancel()
            row={'slot':slot};attempts.append(row)
            try:
                release=plan_object_motion(held,geometry['vertices'],predicted_arm,effective,goal,frame,reachable,
                    self.check_cancel,True,[0,0,.05],row.setdefault('search',[]))
                if len(grasp['waypoints'])+len(release['waypoints'])+bool(release['retreat_waypoint'])>64:
                    raise ValueError('固定全程超过64个执行航点')
                solutions=release['reachability_check']['joint_solutions_deg']
                grasp['fixed_joint_targets']=joint_targets(full_points,solutions,grasp['waypoints'])
                release['fixed_joint_targets']=joint_targets(full_points,solutions,release['waypoints']+([release['retreat_waypoint']] if release['retreat_waypoint'] else []))
                release['current_arm']['joint']=grasp['fixed_joint_targets']['lift']
                release.update(model_sha256=held['model_sha256'],pose_mode_revision=held['pose_mode_revision'],flange_T_grip_tcp=tool['flange_T_grip_tcp'])
                plan={'kind':'fixed_pick_place','scene_id':scene['scene_id'],'request':request,'selected_slot':slot,
                    'source_model_id':source['object_id'],'reference_model_id':rack['object_id'],
                    'current_arm':arm,'grasp':grasp,'release':release,'effective_config':effective,
                    'predicted_hold':held,'configuration_binding':binding,
                    'occupancy_basis':'operator_declared_free_slots_not_automatic_detection',
                    'complete_path_prechecked':True,'path_semantics':'fixed_command_waypoints_not_controller_servo_trajectory',
                    'validation_scope':release['validation_scope'], 'motion_sent':False}
                self.fixed_binding_check(plan);self.check_cancel()
                self.stationary_scene_arm()
                if np.max(np.abs(np.asarray(self.arm_state()['joint'])-arm['joint']))>30:raise ValueError('规划期间机械臂移动')
                return self.object_plan_store().save(plan)
            except (ValueError,TimeoutError) as error:row['error']=str(error)
        raise ValueError('有限候选搜索结束，未找到全程通过的孔位；未发送运动')

    def execute_fixed_pick_place(self, plan, report, request_id):
        self.fixed_binding_check(plan);config=plan['effective_config'];grasp=plan['grasp']
        # Repeat hardware checks read-only before enabling any write; the new
        # candidate is validation evidence, never a replacement execution path.
        self.robot=RobotClient(config['robot_ip'],config['robot_port'],writes=False)
        arm=self.stationary_scene_arm()
        if np.max(np.abs(np.asarray(arm['joint'])-plan['current_arm']['joint']))>30:raise ValueError('机械臂已离开固定方案起点')
        observation=self.scene_observation(plan['scene_id'],grasp['instance_id'],grasp['object_id'])
        self.prepare_grasp_plan(observation,json.loads(self.config_path.read_text()),arm,{})
        geometry=model_geometry(self.tracker.models_root,grasp['object_id'])
        prefix,_=path_samples(arm,grasp['waypoints'],plan['predicted_hold'],geometry['vertices'],config,True)
        release=plan['release'];suffix,_=path_samples(release['current_arm'],release['waypoints'],plan['predicted_hold'],geometry['vertices'],config)
        if release['retreat_waypoint']:
            suffix.extend(segment_samples(controller_pose(release['waypoints'][-1]['controller_pose']),
                controller_pose(release['retreat_waypoint']['controller_pose']),None,None,config,'linear','release_retreat'))
        points=prefix+suffix
        detail=self.verify_motion_targets(grasp,config,report,arm,points)
        expected={**grasp['fixed_joint_targets'],**release['fixed_joint_targets']}
        targets=joint_targets(points,detail['joint_solutions_deg'],grasp['waypoints']+release['waypoints']+([release['retreat_waypoint']] if release['retreat_waypoint'] else []))
        if any(np.max(np.abs(np.asarray(targets[k])-q))>500 for k,q in expected.items()):raise ValueError('全程关节构型复核不一致，未发送动作')
        if time.time()>plan['expires_at']:raise ValueError('固定方案已过期')
        self._fixed_guard=lambda:self.fixed_binding_check(plan)
        self._fixed_path_points=points  # Each grasp segment re-solves the remaining part of this same path from real feedback.
        self.robot=RobotClient(config['robot_ip'],config['robot_port'],writes=True);self.robot.motion_log=report.setdefault('motion_events',[])
        self.update_scene_instance(grasp,'moving')
        self.execute(grasp,config,report)
        self.update_scene_instance(grasp,'held')
        actual=self.held_snapshot()
        if actual is None:raise ValueError('抓取后没有有效夹持')
        a=transform(actual['object_T_tcp']);b=transform(plan['predicted_hold']['object_T_tcp'])
        if np.linalg.norm(a[:3,3]-b[:3,3])>.002 or rotation_error(a[:3,:3],b[:3,:3])>np.deg2rad(2):
            raise ValueError('实际夹持关系偏离预先规划，不重新规划或继续下降')
        release=copy.deepcopy(plan['release'])
        # Keep the planned lift pose as the required start; the joint set comes from real feedback,
        # from which every release segment re-solves the remaining path.
        release['current_arm']['joint']=list(self.arm_state()['joint'])
        release.update(max_execution_duration_s=FIXED_RELEASE_BUDGET_S,hold=actual,configuration_binding=self.motion_binding(config),plan_id=plan['plan_id'],expires_at=plan['expires_at'])
        self.run_object_execution(release,request_id,nested=True)
        report['release_diagnostics_ref']=copy.deepcopy(self.status.get('diagnostics_ref'))
        if self.status['state']!='completed':raise ValueError(self.status.get('error') or '固定放置失败')

    def run_fixed_pick_place(self, args, request_id, plan):
        report={'started_at':time.time(),'request_id':request_id,'operation':'fixed_pick_place','robot_motion_sent':False}
        try:
            directory=self.root/'work/grasp';directory.mkdir(exist_ok=True)
            with (directory/'robot.lock').open('a') as lock:
                fcntl.flock(lock,fcntl.LOCK_EX|fcntl.LOCK_NB)
                if plan is None:
                    plan=self.plan_fixed_pick_place(args,report);report['fixed_plan_id']=plan['plan_id']
                    self.update(state='fixed_pick_place_planned',fixed_pick_place_plan=plan,message='完整抓放路线已固定；尚未发送动作')
                else:
                    report['fixed_plan_id']=plan['plan_id'];self.execute_fixed_pick_place(plan,report,request_id)
                    self.object_plan_store().finish(plan['plan_id'],'completed')
        except Exception as error:
            report['error']=str(error)
            if self.actuated and self.robot is not None:
                try:self.robot.stop();report['stop_acknowledged']=True
                except Exception as stop_error:report['stop_error']=str(stop_error)
                try:self.update_scene_instance(plan['grasp'],'unknown')
                except Exception:pass
            if plan is not None:
                try:self.object_plan_store().finish(plan['plan_id'],'failed')
                except Exception:pass
            self.update(state='cancelled' if self.cancelled.is_set() else 'failed',error=str(error),message=str(error),recovery={'can_replan':False})
        finally:
            self._fixed_guard=None;self._fixed_path_points=None
            try:self.update(diagnostics_ref=self.save_object_job(report))
            except Exception as error:self.update(state='failed',error='固定流程日志保存失败：'+str(error))
            self.update(busy=False)
