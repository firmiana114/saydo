"""Reuse Saydo's authenticated HTTP client; own the fixed state polling."""
import asyncio
import copy
import math
import time
from urllib.parse import quote
from tools.robot_agent.client import RobotBackend, BackendError
from tools.robot_agent.global_camera import observe_global, CameraUnavailable
from .tool_registry import READS, ACTIONS, MOTIONS, VISION
from .motion_retry import MotionRetry,RetryLimit
from .planning_validation import READONLY_PLANNING,validate_planning_arguments,goal_error_detail


class Blocked(RuntimeError):
    def __init__(self, message, uncertain=False):
        self.uncertain = uncertain
        super().__init__(message)


class Cancelled(RuntimeError):
    pass


def compact(value):
    # Full motion reports remain in the backend journal. Do not resend sample arrays.
    excluded = {'image_base64', 'kinematics_checks', 'arrival_checks', 'samples', 'joint_samples',
                'controller_readback', 'model_path', 'mesh', 'output', 'log_path'}
    if isinstance(value, dict):
        return {k: ([compact(row) for row in v[:256]] if k=='features' and isinstance(v,list) else compact(v)) for k, v in value.items() if k not in excluded}
    if isinstance(value, list):
        return [compact(v) for v in value[:32]]
    return value


class RobotAdapter:
    def __init__(self, config, backend=None):
        self.config = config
        self.backend = backend or RobotBackend(config.robot_url, config.token_file)
        self.pending = set()
        self.session_id = None
        self.motion_inflight = False
        self.active_request = None
        self.scene_ids=set()
        self.scene_corrections=0
        self.held_scene_id=None
        self.motion_retry=MotionRetry()

    async def request(self, path, value=None, cancelled=None):
        if cancelled is not None and cancelled.is_set():
            raise Cancelled()
        task = asyncio.create_task(asyncio.to_thread(self.backend.request, path, value))
        if value is not None:
            self.pending.add(task)
            task.add_done_callback(self.pending.discard)
        return await asyncio.shield(task)

    def scene_view(self, value):
        """Expose backend validity and this run's provenance as separate facts."""
        if isinstance(value,list):return [self.scene_view(v) for v in value]
        if not isinstance(value,dict):return value
        result={k:self.scene_view(v) for k,v in value.items()}
        if 'median_depth_residual_mm' in result:
            result['metric_semantics']='模型表面与观测深度的拟合残差，不是目标横向定位误差或位姿误差上界，不能直接与目标间隙比较判定可达/碰撞。good只表示拟合质量检查通过。'
        if 'scene_id' in result and isinstance(result.get('instances'),list):
            backend_valid=result.get('backend_valid_for_execution',result.get('valid_for_execution',False))
            current=result['scene_id'] in self.scene_ids
            reference_usable=bool(backend_valid and (current or result['scene_id']==self.held_scene_id))
            result.update(backend_valid_for_execution=backend_valid,captured_in_this_run=current,
                          usable_as_held_reference=reference_usable,
                          usable_for_grasp_in_this_run=bool(current and backend_valid),
                          valid_for_execution=reference_usable)
            result['scene_usage_note']=('本任务已采集；仍须核对实例未移动及后端有效性。' if current else
                '历史场景不可用于本任务新抓取；仅usable_as_held_reference=true时可供已有夹持继续引用。无夹持时先capture_scene_snapshot采集全部目标，读取不等于采集。')
        return result

    async def status(self):
        state = await self.request('/api/agent/status')
        if self.session_id is None:
            self.session_id = state['session_id']
        elif self.session_id != state['session_id']:
            raise Blocked('后端已重启，需核对现场；不自动续跑', uncertain=True)
        self.held_scene_id=(state.get('grasp',{}).get('held_object') or {}).get('scene_id')
        return self.scene_view(state)

    @staticmethod
    def check_idle(state):
        grasp = state.get('grasp', {})
        if grasp.get('busy') or grasp.get('stopping') or state.get('active_task_id') or state.get('task_recording_error'):
            raise Blocked('后端忙碌、停止未确认或任务记录故障')

    async def initial(self):
        state = await self.status()
        self.check_idle(state)
        robot, objects = await asyncio.gather(self.request('/api/agent/robot'), self.request('/api/agent/objects'))
        return self.scene_view({'status': state, 'robot': robot, 'objects': objects})

    def argument_error_result(self,name,detail,state,source):
        return {'error':{**detail,'validation_source':source},'status':state,'verified':False,
                'argument_correction_required':True,'motion_sent':False,
                'backend_submitted':source=='backend','retry_budget':self.motion_retry.snapshot(),
                'note':'只读规划参数被拒绝，尚未发送运动。纠正字段后重新调用规划工具，计入最多5次规划预算；保持原目标和夹持身份，不重做已成功动作。'}

    async def planning_argument_error(self,name,args,detail,cancelled):
        if name not in READONLY_PLANNING:raise Blocked('该工具不允许自动纠正后重试')
        if cancelled.is_set():raise Cancelled()
        state=await self.status();self.check_idle(state)
        if cancelled.is_set():raise Cancelled()
        try:self.motion_retry.before(name,args,state['grasp'].get('held_object'))
        except RetryLimit as e:raise Blocked(str(e)) from e
        return self.argument_error_result(name,detail,state,'local_schema')

    async def execute(self, name, args, request_id, cancelled, progress):
        if cancelled.is_set():
            raise Cancelled()
        if name == 'observe_global':
            try:
                result = await asyncio.to_thread(observe_global)
            except CameraUnavailable as error:
                raise BackendError({'code':'CAMERA_UNAVAILABLE','message':str(error),
                                    'camera_role':'fixed_global','motion_sent':False}) from error
            if cancelled.is_set():
                raise Cancelled()
            return result
        if name == 'get_task_context':
            queries=[]
            for object_id in args['object_ids']:
                queries.append(('geometry:'+object_id,'get_object_geometry',{'object_id':object_id}))
                if args['include_profiles']:queries.append(('profiles:'+object_id,'get_grasp_profiles',{'object_id':object_id}))
            if args['include_robot']:queries.append(('robot','get_robot_state',{}))
            if args['include_frame']:queries.append(('candidate_frame','get_camera_frame',{}))
            if args['scene_id'] is not None:queries.append(('scene','get_scene_snapshot',{'scene_id':args['scene_id']}))
            values=await asyncio.gather(*(self.execute(tool,params,request_id,cancelled,progress) for _,tool,params in queries))
            return dict(zip((key for key,_,_ in queries),values))
        if name in READS:
            path = '/api/agent/'+READS[name]
            if name in ('get_grasp_profiles','get_object_geometry'):
                path += '?object_id='+quote(args['object_id'], safe='')
            if name=='get_scene_snapshot':path+='?scene_id='+quote(args['scene_id'],safe='')
            if name=='get_task_diagnostics':path+='?task_id='+quote(args['task_id'],safe='')
            return self.scene_view(await self.request(path, cancelled=cancelled))
        if name in VISION:
            await self.request('/api/agent/vision', {'operation': VISION[name], 'arguments': args}, cancelled)
            deadline = time.monotonic()+self.config.vision_timeout
            while time.monotonic() < deadline:
                if cancelled.is_set():
                    raise Cancelled()
                state = await self.status()
                tracking = state['tracking']
                progress('跟踪：'+str(tracking.get('state')))
                if name == 'select_object' and tracking.get('model_ready') and tracking.get('object_id') == args['object_id']:
                    return state
                if name != 'select_object':
                    if tracking.get('state') == 'choose_candidate':
                        frame = await self.request('/api/agent/selection-frame')
                        if frame.get('selection_id') != tracking.get('selection_id'):
                            raise Blocked('候选画面已变化，请重新获取识别结果')
                        return {**state, 'candidate_frame': frame}
                    if tracking.get('state') == 'tracking':
                        try:
                            pose = await self.request('/api/agent/pose')
                            return {**state, 'pose': pose}
                        except BackendError as error:
                            if error.payload.get('code') != 'POSE_NOT_READY':
                                raise
                    if tracking.get('state') in ('model_error', 'camera_error', 'failed', 'lost'):
                        raise Blocked('跟踪初始化失败')
                await asyncio.sleep(self.config.poll_interval)
            raise Blocked('等待模型或跟踪初始化超时；未发送机械臂动作')
        if name not in ACTIONS:
            raise ValueError('不支持的工具')
        state = await self.status()
        self.check_idle(state)
        if cancelled.is_set():raise Cancelled()
        if name in READONLY_PLANNING:
            try:self.motion_retry.before(name,args,state['grasp'].get('held_object'))
            except RetryLimit as error:raise Blocked(str(error)) from error
            try:validate_planning_arguments(name,args)
            except ValueError as error:return self.argument_error_result(name,goal_error_detail(error),state,'local_semantics')
        if name in ('preview_reorient', 'reorient','plan_held_object_motion','plan_release'):
            held = state.get('grasp', {}).get('held_object')
            if not held or any(held.get(k) != args[k] for k in ('object_id', 'hold_id', 'orientation_version')):
                raise Blocked('物品、夹持关系或朝向版本已变化；先读取状态并重新规划')
        if name in MOTIONS:
            if self.config.mode != 'execute' or state['grasp'].get('mode') != 'execute':
                raise Blocked('当前模式不允许实际动作')
            if name == 'grasp' and state['grasp'].get('held_object'):
                raise Blocked('已有夹持记录，不能再次抓取')
        if name in ('grasp','preview_grasp') and args.get('scene_id') not in self.scene_ids:
            if state['grasp'].get('held_object'):raise Blocked('已有夹持记录，不能重新采集抓取场景')
            self.scene_corrections+=1
            if self.scene_corrections>3:raise Blocked('本任务场景前置条件连续纠正3次仍未满足，停止；未派发抓取')
            return {'error':{'code':'SCENE_CAPTURE_REQUIRED','message':'必须先在本任务采集全部目标，再引用返回的scene_id抓取。请调用capture_scene_snapshot，包含所有source/reference实例。'},
                    'precondition_correction_required':True,'required_tool':'capture_scene_snapshot',
                    'motion_sent':False,'backend_submitted':False,'verified':False,
                    'correction_attempt':self.scene_corrections,'correction_limit':3,'status':state}
        if cancelled.is_set():
            raise Cancelled()
        if name not in READONLY_PLANNING:
            try:self.motion_retry.before(name,args,state['grasp'].get('held_object'))
            except RetryLimit as error:raise Blocked(str(error)) from error
        if name=='capture_scene_snapshot':self.scene_ids.clear()
        self.active_request = request_id
        self.motion_inflight = name in MOTIONS
        value = {'operation': ACTIONS[name], 'request_id': request_id, 'arguments': args}
        try:
            result = await self.request('/api/agent/tasks', value, cancelled)
        except BackendError as error:
            if error.payload.get('outcome_unknown'):raise Blocked('后端报告请求结果未知，禁止自动纠错重试',uncertain=True) from error
            if name in READONLY_PLANNING and error.payload.get('code')=='INVALID_ARGUMENTS' and not error.payload.get('outcome_unknown'):
                self.motion_inflight=False
                if cancelled.is_set():raise Cancelled()
                latest=await self.status();self.check_idle(latest)
                if cancelled.is_set():raise Cancelled()
                self.active_request=None  # Rejected planner submission is not an unresolved motion.
                detail={'code':'INVALID_ARGUMENTS','message':str(error.payload.get('message') or '后端规划参数校验失败')[:1200]}
                return self.argument_error_result(name,detail,latest,'backend')
            if error.payload.get('code') != 'REQUEST_UNCERTAIN':
                self.motion_inflight = False
                raise
            result = {'terminal': False}
        deadline = time.monotonic()+(200 if name=='capture_scene_snapshot' else self.config.action_timeout)
        while not result.get('terminal'):
            if cancelled.is_set():
                raise Cancelled()
            if time.monotonic() >= deadline:
                raise Blocked('动作结果未在期限内确认', uncertain=True)
            await asyncio.sleep(self.config.poll_interval)
            try:
                result = await self.request('/api/agent/tasks/'+request_id)
            except BackendError as error:
                raise Blocked('提交结果未知，按原编号查询仍失败；禁止换编号重试', uncertain=True) from error
            progress('后端：'+str(result.get('state')))
        self.motion_inflight = False
        if result.get('state') == 'interrupted' or result.get('outcome_unknown'):
            raise Blocked('后端报告结果未知，需人工核对', uncertain=True)
        expected = 'completed' if name in MOTIONS else {'capture_scene_snapshot':'scene_ready','preview_grasp':'preview', 'preview_place':'placement_preview', 'preview_reorient':'reorient_preview','plan_held_object_motion':'object_motion_planned','plan_release':'object_motion_planned'}[name]
        if result.get('state') != expected:
            if name in ('preview_reorient','plan_held_object_motion','plan_release','capture_scene_snapshot') and result.get('state') in ('failed','rejected'):
                latest=await self.status()
                return {'task':result,'status':latest,'verified':False,
                        'error':{'code':'PLAN_FAILED','message':str(result.get('error') or result.get('result',{}).get('error') or result.get('state'))},
                        'motion_sent':False,'retry_budget':self.motion_retry.snapshot(),'note':'只读规划失败，可查看诊断并在用户目标范围内重新规划；没有执行运动。'}
            recovery=result.get('result',{}).get('recovery') or {}
            if name in ('reorient','execute_plan') and result.get('state')=='failed' and recovery.get('can_replan') is True:
                latest=await self.status();self.check_idle(latest);current=latest['grasp'].get('held_object')
                if not current or any(current.get(k)!=recovery.get(k) for k in ('hold_id','object_id','orientation_version')):raise Blocked('恢复检查后夹持版本已变化',uncertain=True)
                if recovery.get('release_attempted') or recovery.get('stationary_confirmed') is not True or recovery.get('grip_confirmed') is not True or any(recovery.get('controller_errors') or [1]):raise Blocked('恢复条件未通过',uncertain=True)
                try:self.motion_retry.recover(recovery)
                except RetryLimit as error:raise Blocked(str(error)) from error
                return {'task':result,'status':latest,'verified':False,'recoverable_failure':True,'retry_budget':self.motion_retry.snapshot(),
                    'error':{'code':'MOTION_REPLAN_REQUIRED','message':result['result'].get('error')},
                    'note':'原动作失败；已确认停稳、夹持和设备状态。按新orientation_version重新规划原目标；可调整步长/路点，不重做抓取或重放旧方案。'}
            raise Blocked('后端任务未成功：'+str(result.get('state')))
        latest = await self.status()
        if name=='capture_scene_snapshot':
            scene=result.get('result',{}).get('scene_snapshot') or {}
            if not scene.get('valid_for_execution') or scene.get('state')!='ready':raise Blocked('场景未全部准备完成')
            self.scene_ids={scene['scene_id']}
        if name == 'grasp':
            held = result.get('result', {}).get('held_object')
            current = latest.get('grasp', {}).get('held_object')
            if not held or not current or held.get('object_id') != args['object_id'] or held.get('hold_id') != current.get('hold_id'):
                raise Blocked('抓取结束但预期夹持关系未确认', uncertain=True)
        if name == 'place' and (result.get('result', {}).get('held_object') is not None or latest['grasp'].get('held_object') is not None):
            raise Blocked('放置结束但夹持记录未清除', uncertain=True)
        if name in ('preview_reorient', 'reorient'):
            self.verify_reorientation(name, args, result, latest)
        if name=='execute_plan':
            self.verify_object_execution(args,result,latest)
        plan=result.get('result',{}).get('object_motion_plan') or result.get('result',{}).get('reorientation_plan')
        if name in ('plan_held_object_motion','plan_release','preview_reorient') and plan:
            try:self.motion_retry.plan(plan)
            except RetryLimit as error:return {'task':result,'status':latest,'verified':False,'error':{'code':'GOAL_CONSTRAINT_CHANGED','message':str(error)},'retry_budget':self.motion_retry.snapshot()}
        budget=self.motion_retry.snapshot()
        if name in ('grasp','place','reorient','execute_plan') and result.get('state')=='completed':self.motion_retry.reset()
        return self.scene_view({'task': result, 'status': latest,'retry_budget':budget, 'verified': name in MOTIONS,
                'verification_scope': 'controller_feedback_only; not independent contact/slip/visual verification'})

    @staticmethod
    def verify_object_execution(args,task,status):
        try:
            payload=task['result'];plan=payload['object_motion_plan'];result=payload['execution_result']
            if result['plan_id']!=args['plan_id'] or plan['plan_id']!=args['plan_id'] or not result['target_verified']:raise ValueError('plan mismatch')
            if result['kind']!=plan['kind']:raise ValueError('kind mismatch')
            if (not math.isfinite(result['reference_position_error_m']) or not 0<=result['reference_position_error_m']<=plan['position_tolerance_m']
                    or not math.isfinite(result['orientation_error_deg']) or not 0<=result['orientation_error_deg']<=plan['orientation_tolerance_deg']):raise ValueError('target error')
            def pose(value):
                if (not isinstance(value,list) or len(value)!=4 or any(not isinstance(row,list) or len(row)!=4 for row in value)
                        or any(type(x) not in (int,float) or not math.isfinite(x) for row in value for x in row)
                        or any(abs(value[3][i]-(1 if i==3 else 0))>1e-6 for i in range(4))):raise ValueError('invalid pose')
                if any(abs(sum(value[i][k]*value[j][k] for k in range(3))-(i==j))>1e-3 for i in range(3) for j in range(3)):raise ValueError('invalid rotation')
                det=sum(value[0][i]*(value[1][(i+1)%3]*value[2][(i+2)%3]-value[1][(i+2)%3]*value[2][(i+1)%3]) for i in range(3))
                if abs(det-1)>1e-3:raise ValueError('improper rotation')
                return value
            actual=pose(result['actual_object_pose_before_release']);target=pose(plan['base_T_object_target']);point=plan['reference_point_object_m']
            if not isinstance(point,list) or len(point)!=3 or any(type(x) not in (int,float) or not math.isfinite(x) for x in point):raise ValueError('invalid reference point')
            if not .0001<=plan['position_tolerance_m']<=.002 or not .1<=plan['orientation_tolerance_deg']<=2:raise ValueError('invalid tolerance')
            position_error=math.sqrt(sum((actual[i][3]+sum(actual[i][k]*point[k] for k in range(3))-target[i][3]-sum(target[i][k]*point[k] for k in range(3)))**2 for i in range(3)))
            if plan.get('pose_mode') not in ('5d','6d'):raise ValueError('invalid pose mode')
            cosine=sum(actual[i][2]*target[i][2] for i in range(3)) if plan['pose_mode']=='5d' else (sum(actual[i][j]*target[i][j] for i in range(3) for j in range(3))-1)/2
            angle_error=math.degrees(math.acos(max(-1.,min(1.,cosine))))
            if position_error>plan['position_tolerance_m'] or angle_error>plan['orientation_tolerance_deg']:raise ValueError('measured object pose not at target')
            held=payload.get('held_object');current=status['grasp'].get('held_object')
            if plan['kind']=='release':
                if not result['release_confirmed'] or not result['retreat_confirmed'] or held is not None or current is not None:raise ValueError('release unconfirmed')
            elif plan['kind']=='held_object_motion':
                for value in (held,current):
                    if not value or value['hold_id']!=plan['hold']['hold_id'] or value['object_id']!=plan['hold']['object_id'] or value['orientation_version']!=plan['hold']['orientation_version']+1:raise ValueError('hold/version mismatch')
            else:raise ValueError('unknown plan kind')
        except (KeyError,TypeError,ValueError,IndexError) as error:
            raise Blocked('通用方案的目标到位、夹持版本或释放结果未核验通过',uncertain=True) from error

    @staticmethod
    def verify_reorientation(name, args, task, status):
        expected = args['orientation_version'] + (name == 'reorient')
        held = task.get('result', {}).get('held_object')
        current = status.get('grasp', {}).get('held_object')
        for value in (held, current):
            if (not value or value.get('hold_id') != args['hold_id'] or value.get('object_id') != args['object_id']
                    or type(value.get('orientation_version')) is not int or value['orientation_version'] != expected):
                raise Blocked('姿态任务结束但夹持身份或朝向版本未确认', uncertain=name == 'reorient')
        if name != 'reorient':
            return
        try:
            plan = task['result']['reorientation_plan']
            pose_mode = held.get('pose_mode', '6d')
            if pose_mode not in ('5d','6d') or pose_mode != current.get('pose_mode','6d') or pose_mode != plan.get('pose_mode','6d'):
                raise ValueError('pose mode mismatch')
            target = [row[:3] for row in plan['base_T_object_target'][:3]]
            tolerance = plan['orientation_tolerance_deg']
            if type(tolerance) not in (int, float) or not math.isfinite(tolerance) or not .1 <= tolerance <= 3:
                raise ValueError('invalid tolerance')
            for matrix in (target, held['current_object_rotation'], current['current_object_rotation']):
                if (len(matrix) != 3 or any(len(row) != 3 for row in matrix)
                        or any(type(v) not in (int, float) or not math.isfinite(v) for row in matrix for v in row)):
                    raise ValueError('invalid rotation')
                if any(abs(sum(matrix[i][k]*matrix[j][k] for k in range(3))-(i==j)) > 1e-3 for i in range(3) for j in range(3)):
                    raise ValueError('non-orthogonal rotation')
                determinant = sum(matrix[0][i]*(matrix[1][(i+1)%3]*matrix[2][(i+2)%3]-matrix[1][(i+2)%3]*matrix[2][(i+1)%3]) for i in range(3))
                if abs(determinant-1) > 1e-3:
                    raise ValueError('improper rotation')
            for actual in (held['current_object_rotation'], current['current_object_rotation']):
                cosine = (sum(actual[i][2]*target[i][2] for i in range(3)) if pose_mode=='5d'
                          else (sum(actual[i][j]*target[i][j] for i in range(3) for j in range(3))-1)/2)
                if math.degrees(math.acos(max(-1., min(1., cosine)))) > tolerance+1e-6:
                    raise ValueError('target orientation not reached')
        except (KeyError, TypeError, ValueError, IndexError) as error:
            raise Blocked('姿态任务的目标朝向与实际反馈未核验通过', uncertain=True) from error

    async def stop(self):
        # A submission can arrive after the first stop. Do not report cancellation
        # until all dispatched HTTP writes settle and a final stop is confirmed.
        pending = list(self.pending)
        first = None
        try:
            first = await self.request('/api/agent/stop', {})
        except Exception:
            pass
        if pending:
            _, unfinished = await asyncio.wait(pending, timeout=20)
            if unfinished:
                raise Blocked('停止后仍有未确认的请求在途', uncertain=True)
            for task in pending:
                if not task.cancelled():
                    task.exception()  # Consume any failed submission; query below.
            first = await self.request('/api/agent/stop', {})
        if not first or not first.get('stop_acknowledged') or not first.get('stationary_observed'):
            raise Blocked('停止未确认，需现场核对', uncertain=True)
        if self.active_request:
            try:
                task = await self.request('/api/agent/tasks/'+self.active_request)
            except BackendError as error:
                raise Blocked('停止已应答，但此前提交结果仍未知', uncertain=True) from error
            if not task.get('terminal'):
                raise Blocked('停止后原任务仍未结束', uncertain=True)
        self.motion_inflight = False
        return first


def model_result(name, value):
    """Reduce only model context, after verification; journal keeps compact full result."""
    result=copy.deepcopy(value)
    plan_keys=('plan','placement_plan','reorientation_plan','object_motion_plan')
    def strip_history(grasp):
        if isinstance(grasp,dict) and grasp.get('state') in ('completed','idle'):
            for key in plan_keys:grasp.pop(key,None)
    def status_view(status):
        if not isinstance(status,dict):return
        grasp=status.get('grasp')
        # Pending plans and failed-task evidence remain available to a new run.
        strip_history(grasp)
    if name=='get_task_diagnostics':return result
    if name=='get_status':status_view(result)
    status_view(result.get('status'))
    controller=result.get('controller')
    strip_history(controller)
    if name=='initial':
        robot=result.get('robot',{})
        controller=robot.get('controller',{}) if isinstance(robot,dict) else {}
        strip_history(controller)
    task=result.get('task')
    if (name in MOTIONS and isinstance(task,dict) and task.get('state')=='completed'
            and result.get('verified') is True and isinstance(task.get('result'),dict)):
        payload=task['result'];plan=payload.get('plan')
        if name=='grasp' and isinstance(plan,dict):
            # Preserve actual selected grasp semantics/settings, not candidate matrices.
            keep=('object_id','object_name','pose_mode','grasp_id','grasp_name','orientation_policy',
                  'approach_policy','effective_parameters','controller_tcp_frame','lift_height_m')
            payload['plan']={k:plan[k] for k in keep if k in plan}
        if name!='grasp':
            for key in plan_keys:payload.pop(key,None)
        status=result.get('status',{});current=status.get('grasp',{}) if isinstance(status,dict) else {}
        if 'held_object' in payload and isinstance(current,dict) and payload.get('held_object')==current.get('held_object') and 'held_object' in current:
            current.pop('held_object');current['held_object_source']='task.result.held_object'
        result['context_note']='成功动作已核验；历史路径/候选省略，完整报告按task.task_id查询；不得重放成功动作。'
    def trim(value):
        if isinstance(value,list):return [trim(v) for v in value]
        if not isinstance(value,dict):return value
        if value.get('instance_id') and value.get('static_pose_usable') is False:
            value={k:v for k,v in value.items() if k not in ('base_T_object','camera_T_object')}
            value['pose_usage_note']='实例已移动；原快照位姿不再提供给模型。当前持物位姿查询get_robot_state.held_object_pose。'
        if 'base_T_object_at_grasp' in value:
            value={k:v for k,v in value.items() if k!='base_T_object_at_grasp'}
            value['current_pose_source']='get_robot_state.held_object_pose；不能使用抓取时位姿代替抬升后位姿'
        reduced={k:trim(v) for k,v in value.items() if k not in
            ('configuration_binding','flange_T_grip_tcp','target_requests','search_attempts','controller_readback')}
        # Inspection API and durable journal retain detailed trajectories.
        if 'plan_id' in reduced and 'waypoints' in reduced:
            reduced['waypoint_count']=value.get('waypoint_count',len(value['waypoints']))
            reduced.pop('waypoints',None)
            reduced['details_available_via']='get_task_diagnostics'
        features=reduced.get('features')
        if isinstance(features,list):
            for feature in features:
                matrix=feature.get('object_T_feature')
                if matrix and all(abs(matrix[i][j]-(i==j))<1e-9 for i in range(3) for j in range(3)):
                    feature['position_object_m']=[matrix[i][3] for i in range(3)]
                    feature['rotation_object']='identity'
                    feature.pop('object_T_feature')
        return reduced
    result=trim(result)
    # Same snapshot/plan can be echoed by both task result and status.
    task=result.get('task',{});payload=task.get('result',{}) if isinstance(task,dict) else {}
    grasp=result.get('status',{}).get('grasp',{})
    for key in ('scene_snapshot','object_motion_plan','reorientation_plan','held_object'):
        if key in payload and key in grasp and payload[key]==grasp[key]:
            grasp.pop(key);grasp[key+'_source']='task.result.'+key
    return result
