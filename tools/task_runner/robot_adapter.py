"""Reuse Saydo's authenticated HTTP client; own the fixed state polling."""
import asyncio
import math
import time
from urllib.parse import quote
from tools.robot_agent.client import RobotBackend, BackendError
from .tool_registry import READS, ACTIONS, MOTIONS, VISION


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
        return {k: compact(v) for k, v in value.items() if k not in excluded}
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

    async def request(self, path, value=None, cancelled=None):
        if cancelled is not None and cancelled.is_set():
            raise Cancelled()
        task = asyncio.create_task(asyncio.to_thread(self.backend.request, path, value))
        if value is not None:
            self.pending.add(task)
            task.add_done_callback(self.pending.discard)
        return await asyncio.shield(task)

    async def status(self):
        state = await self.request('/api/agent/status')
        if self.session_id is None:
            self.session_id = state['session_id']
        elif self.session_id != state['session_id']:
            raise Blocked('后端已重启，需核对现场；不自动续跑', uncertain=True)
        return state

    @staticmethod
    def check_idle(state):
        grasp = state.get('grasp', {})
        if grasp.get('busy') or grasp.get('stopping') or state.get('active_task_id') or state.get('task_recording_error'):
            raise Blocked('后端忙碌、停止未确认或任务记录故障')

    async def initial(self):
        state = await self.status()
        self.check_idle(state)
        robot, objects = await asyncio.gather(self.request('/api/agent/robot'), self.request('/api/agent/objects'))
        return {'status': state, 'robot': robot, 'objects': objects}

    async def execute(self, name, args, request_id, cancelled, progress):
        if cancelled.is_set():
            raise Cancelled()
        if name in READS:
            path = '/api/agent/'+READS[name]
            if name == 'get_grasp_profiles':
                path += '?object_id='+quote(args['object_id'], safe='')
            return await self.request(path, cancelled=cancelled)
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
                        return {**state, 'candidate_frame': await self.request('/api/agent/frame')}
                    if tracking.get('state') == 'tracking':
                        try:
                            pose = await self.request('/api/agent/pose')
                            return {**state, 'pose': pose}
                        except BackendError as error:
                            if error.payload.get('code') != 'POSE_NOT_READY':
                                raise
                    if tracking.get('state') in ('model_error', 'camera_error', 'failed'):
                        raise Blocked('跟踪初始化失败')
                await asyncio.sleep(self.config.poll_interval)
            raise Blocked('等待模型或跟踪初始化超时；未发送机械臂动作')
        if name not in ACTIONS:
            raise ValueError('不支持的工具')
        state = await self.status()
        self.check_idle(state)
        if name in ('preview_reorient', 'reorient'):
            held = state.get('grasp', {}).get('held_object')
            if not held or any(held.get(k) != args[k] for k in ('object_id', 'hold_id', 'orientation_version')):
                raise Blocked('物品、夹持关系或朝向版本已变化；先读取状态并重新规划')
        if name in MOTIONS:
            if self.config.mode != 'execute' or state['grasp'].get('mode') != 'execute':
                raise Blocked('当前模式不允许实际动作')
            if name == 'grasp' and state['grasp'].get('held_object'):
                raise Blocked('已有夹持记录，不能再次抓取')
        if cancelled.is_set():
            raise Cancelled()
        self.active_request = request_id
        self.motion_inflight = name in MOTIONS
        value = {'operation': ACTIONS[name], 'request_id': request_id, 'arguments': args}
        try:
            result = await self.request('/api/agent/tasks', value, cancelled)
        except BackendError as error:
            if error.payload.get('code') != 'REQUEST_UNCERTAIN':
                self.motion_inflight = False
                raise
            result = {'terminal': False}
        deadline = time.monotonic()+self.config.action_timeout
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
        expected = 'completed' if name in MOTIONS else {'preview_grasp':'preview', 'preview_place':'placement_preview', 'preview_reorient':'reorient_preview'}[name]
        if result.get('state') != expected:
            raise Blocked('后端任务未成功：'+str(result.get('state')))
        latest = await self.status()
        if name == 'grasp':
            held = result.get('result', {}).get('held_object')
            current = latest.get('grasp', {}).get('held_object')
            if not held or not current or held.get('object_id') != args['object_id'] or held.get('hold_id') != current.get('hold_id'):
                raise Blocked('抓取结束但预期夹持关系未确认', uncertain=True)
        if name == 'place' and (result.get('result', {}).get('held_object') is not None or latest['grasp'].get('held_object') is not None):
            raise Blocked('放置结束但夹持记录未清除', uncertain=True)
        if name in ('preview_reorient', 'reorient'):
            self.verify_reorientation(name, args, result, latest)
        return {'task': result, 'status': latest, 'verified': name in MOTIONS,
                'verification_scope': 'controller_feedback_only; not independent contact/slip/visual verification'}

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
