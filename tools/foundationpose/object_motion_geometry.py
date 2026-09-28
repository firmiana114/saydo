"""General held-object goals. Object references, not TCP origins, define targets."""
import subprocess
import time
import numpy as np
from rotation_steps import interpolate,fractions
from grasp_geometry import transform, controller_pose, rotation_error
from grasp_library import point_limits
from motion_speed import checked_speed
from placement_geometry import check_tcp_workspace
from pose_symmetry import object_bounds, orientation_variants, object_rotation_error
from reorientation_geometry import rotation_from_rpy_deg, pose_waypoint, validate_object_pose, position_offsets
from tools.robot_agent.motion_contract import checked_goal, vector

MAX_WAYPOINTS=64
MAX_VALIDATION_POINTS=480
POSITION_TOLERANCE=.002
ANGLE_TOLERANCE_DEG=2.


def object_at_reference(reference, rotation, point):
    result=np.eye(4);result[:3,:3]=rotation;result[:3,3]=np.asarray(reference)-rotation@point
    return result


def validate_tcp(tcp, held, vertices, config):
    check_tcp_workspace(tcp,config)
    validate_object_pose(tcp,held,vertices,config,False)


def segment_samples(start, end, held, vertices, config, method, name):
    distance=float(np.linalg.norm(end[:3,3]-start[:3,3]))
    angle=rotation_error(start[:3,:3],end[:3,:3])
    steps=max(1,int(np.ceil(distance/.01)),int(np.ceil(angle/np.deg2rad(5)))) if method=='linear' else 1
    if steps>MAX_VALIDATION_POINTS:raise ValueError('单段采样数量过多')
    points=[]
    for k in range(1,steps+1):
        fraction=k/steps;tcp=np.eye(4);tcp[:3,3]=start[:3,3]+fraction*(end[:3,3]-start[:3,3])
        tcp[:3,:3]=interpolate(start[:3,:3],end[:3,:3],fraction)
        if held is not None:validate_tcp(tcp,held,vertices,config)
        else:check_tcp_workspace(tcp,config)
        w=pose_waypoint(f'{name}_sample_{k}',tcp);w['method']=method;points.append(w)
    return points


def object_error(actual, target, point, pose_mode):
    actual,target=transform(actual),transform(target);point=np.asarray(point)
    return {'reference_position_error_m':float(np.linalg.norm((actual[:3,3]+actual[:3,:3]@point)-(target[:3,3]+target[:3,:3]@point))),
            'orientation_error_deg':float(np.rad2deg(object_rotation_error(actual[:3,:3],target[:3,:3],pose_mode)))}


def build_candidate(held, vertices, arm, config, goal, frame, spin, shift, release, retreat):
    current=controller_pose(arm['pose']);attachment=transform(held['object_T_tcp']);obj=current@np.linalg.inv(attachment)
    point=np.asarray(goal['reference_point_object_m']);low,high=point_limits(vertices)
    if np.any(point<low) or np.any(point>high):raise ValueError('物品参考点超出模型附近允许范围')
    if held.get('pose_mode')=='5d' and np.any(np.abs(point[:2])>1e-9):raise ValueError('5D物品参考点必须位于模型Z轴（X/Y为0）')
    validate_tcp(current,held,vertices,config)
    start_obj=obj.copy();start_ref=obj[:3,3]+obj[:3,:3]@point
    rows=list(goal['waypoints'])
    if goal['target_position_m'] is not None or goal['target_rpy_deg'] is not None or not rows:
        rows.append({'position_m':goal['target_position_m'],'rpy_deg':goal['target_rpy_deg']})
    # One final orientation for the whole operation; intermediate points may
    # change position only. Resolve it before any translation can be planned.
    rows=[dict(row) for row in rows]
    orientations=[row['rpy_deg'] for row in rows if row['rpy_deg'] is not None]
    if orientations:
        final_rpy=orientations[-1]
        if any(rotation_error(rotation_from_rpy_deg(r),rotation_from_rpy_deg(final_rpy))>1e-6 for r in orientations):
            raise ValueError('中间路点不得改变最终目标朝向；先调姿，再保持朝向搬运')
        rows[0]['rpy_deg']=final_rpy
    chosen_rotation=None
    waypoints=[];validation=[];previous_tcp=current;target_requests=[]
    for segment,row in enumerate(rows):
        previous_ref=obj[:3,3]+obj[:3,:3]@point
        reference=(frame@np.r_[row['position_m'],1])[:3] if row['position_m'] is not None else start_ref+shift
        if goal['position_mode']=='fixed' and row['position_m'] is None:reference=previous_ref
        requested_rotation=frame[:3,:3]@rotation_from_rpy_deg(row['rpy_deg']) if row['rpy_deg'] is not None else obj[:3,:3]
        # Only explicit orientation goals permit free-spin search. Position-only
        # moves keep the current representative/TCP orientation unchanged.
        if chosen_rotation is not None:rotation=chosen_rotation
        elif row['rpy_deg'] is not None and held.get('pose_mode')=='5d':
            rotations=orientation_variants(requested_rotation,held,previous_tcp);rotation=rotations[spin%len(rotations)]
        else:rotation=requested_rotation
        chosen_rotation=rotation.copy()
        target_requests.append({'reference_position_base_m':reference.tolist(),'requested_rotation_base':requested_rotation.tolist()})
        method='linear' if goal['path_mode']=='linear' else 'joint_pose'
        stages=[(previous_ref,method,f'orient_{segment+1}'),(reference,'linear',f'transfer_{segment+1}')]
        if release and segment==len(rows)-1:
            height=float(config.get('release_preplacement_height_m',.1))
            if not np.isfinite(height) or not .01<=height<=.3:raise ValueError('预放置高度须为10至300毫米')
            stages=[(previous_ref,method,'release_orientation'),
                    (reference+np.array([0,0,height]),'linear','release_preplacement'),
                    (reference,'linear','release_descent')]
        for stage_ref,stage_method,label in stages:
            previous_ref=obj[:3,3]+obj[:3,:3]@point
            steps=fractions(obj[:3,:3],rotation,goal['max_rotation_step_deg'])
            origin_obj=obj.copy()
            for k,f in enumerate(steps,1):
                rot=interpolate(origin_obj[:3,:3],rotation,f)
                reference_k=previous_ref+f*(stage_ref-previous_ref)
                target_obj=object_at_reference(reference_k,rot,point);tcp=target_obj@attachment
                w=pose_waypoint(f'{label}_{k}',tcp);w['method']=stage_method
                quantized=controller_pose(w['controller_pose']);validate_tcp(quantized,held,vertices,config)
                w['object_pose']=target_obj.tolist();w['reference_position_base_m']=reference_k.tolist()
                validation.extend(segment_samples(previous_tcp,quantized,held,vertices,config,w['method'],w['name']))
                if np.linalg.norm(quantized[:3,3]-previous_tcp[:3,3])>1e-7 or rotation_error(quantized[:3,:3],previous_tcp[:3,:3])>1e-6:
                    waypoints.append(w)
                previous_tcp=quantized
            obj=object_at_reference(stage_ref,rotation,point)
    if goal['position_mode']=='adaptive' and previous_tcp[2,3]<=config['table_z_m']+.10:
        raise ValueError('自适应姿态最终TCP须高于桌面10厘米')
    if len(waypoints)>MAX_WAYPOINTS:raise ValueError('方案超过64个运动路点，请拆分任务')
    retreat_target=None
    if release and np.linalg.norm(retreat)>0:
        endpoint=previous_tcp.copy();endpoint[:3,3]+=retreat;check_tcp_workspace(endpoint,config)
        retreat_target=pose_waypoint('release_retreat',endpoint);retreat_target['method']='linear'
        validation.extend(segment_samples(previous_tcp,endpoint,None,None,config,'linear','release_retreat'))
    if len(validation)>MAX_VALIDATION_POINTS:raise ValueError('方案超过480个检查采样点，请拆分任务')
    if not validation:validation=[pose_waypoint('stationary',current)]
    final_ref=obj[:3,3]+obj[:3,:3]@point
    return {'kind':'release' if release else 'held_object_motion','goal':goal,'pose_mode':held.get('pose_mode','6d'),
            'current_arm':arm,'base_T_object_start':start_obj.tolist(),'base_T_object_target':obj.tolist(),
            'reference_point_object_m':point.tolist(),'target_reference_position_base_m':final_ref.tolist(),
            'reference_frame_pose':frame.tolist(),'target_requests':target_requests,'waypoints':waypoints,
            'waypoint_count':len(waypoints),'position_constraint_scope':'commanded_waypoint_endpoints','stage_policy':'orient_then_translate_then_release','validation_points':validation,'retreat_waypoint':retreat_target,
            'retreat_vector_base_m':retreat.tolist(),'speed_percent':checked_speed(config['arm_speed_percent']),
            'position_tolerance_m':POSITION_TOLERANCE,'orientation_tolerance_deg':ANGLE_TOLERANCE_DEG,
            'gripper_action':'open_after_arrival' if release else 'keep_holding','max_execution_duration_s':90,
            'validation_scope':{'ik_fk':True,'self_collision_at_samples':True,'tcp_workspace':True,'object_table_clearance':True,
                'linear_path_sampled':True,'joint_intermediate_path_checked':False,
                'gripper_collision_checked':False,'environment_collision_checked':False,'object_support_checked':False},
            'release_preplacement_height_m':float(config.get('release_preplacement_height_m',.1)) if release else None,
            'release_semantics':'target pose before opening; settled pose after release not verified' if release else None}


def plan_object_motion(held,vertices,arm,config,goal,frame,reachable,cancelled,release=False,retreat=None,diagnostics=None):
    goal=checked_goal(goal,release);frame=transform(frame);retreat=np.asarray(vector(retreat or [0,0,0],'retreat_vector_base_m',.3))
    if np.linalg.norm(retreat)>.3:raise ValueError('退出位移长度不得超过300毫米')
    attempts=diagnostics if diagnostics is not None else [];deadline=time.monotonic()+20
    current=controller_pose(arm['pose']);spins=range(24) if held.get('pose_mode')=='5d' and (goal['target_rpy_deg'] is not None or any(r['rpy_deg'] is not None for r in goal['waypoints'])) else range(1)
    shifts=position_offsets(current[:3,3],config['table_z_m']+.103,.025) if goal['position_mode']=='adaptive' else [np.zeros(3)]
    for shift in shifts:
        for spin in spins:
            cancelled()
            if time.monotonic()>=deadline:raise TimeoutError('持物运动规划达到20秒预算，未发送运动')
            attempt={'spin_candidate':spin,'shift_m':shift.tolist()};attempts.append(attempt)
            if len(attempts)>64:del attempts[32]
            try:
                plan=build_candidate(held,vertices,arm,config,goal,frame,spin,shift,release,retreat)
                valid,detail=reachable(plan['validation_points'],min(3.,max(.05,deadline-time.monotonic())))
                if not valid:raise ValueError(detail.get('reason','逆解/正解或自身碰撞检查失败'))
                cancelled()
                if time.monotonic()>deadline:raise TimeoutError('规划超时')
                plan.update(reachability_check=detail,search_attempts=attempts)
                plan.pop('validation_points')
                return plan
            except (ValueError,TimeoutError,subprocess.TimeoutExpired) as error:
                attempt['reason']=str(error)[:1200]
        if goal['position_mode']!='adaptive':break
    raise ValueError('没有通过检查的持物运动方案：'+(attempts[-1].get('reason','无候选') if attempts else '无候选'))
