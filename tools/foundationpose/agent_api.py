"""Authenticated, bounded Agent access to the existing tracker/controller.

Motion stays in GraspController. Durable request IDs prevent retries from repeating it.
"""

import base64
import copy
from contextlib import closing
import hashlib
import hmac
import json
from pathlib import Path
import re
import sqlite3
import threading
import time
import uuid

import cv2
import numpy as np

from grasp_robot import RobotClient
from object_catalog import scan_objects
from reorientation_geometry import rotation_from_rpy_deg, checked_position_request
from grasp_geometry import controller_pose, transform
from tools.robot_agent.motion_contract import checked_goal, identity, vector
from object_motion_control import public_object_plan
from tools.robot_agent.scene_contract import checked_targets
from rotation_steps import checked_step
from fixed_pick_place import checked_request as checked_fixed_request


class AgentError(ValueError):
    def __init__(self, code, message, **details):
        super().__init__(message)
        self.payload = {'code': code, 'message': message, **details}


def json_value(value):
    return json.loads(
        json.dumps(value, default=lambda x: x.tolist(), allow_nan=False)
    )


class AgentAPI:
    def __init__(self, tracker, token_path):
        self.tracker, self.grasp = tracker, tracker.grasp
        path = Path(token_path)
        if path.stat().st_mode & 0o077:
            raise ValueError(
                'Agent token file must have owner-only permissions'
            )
        self.token = path.read_text().strip()
        if len(self.token) < 32:
            raise ValueError('Agent token must contain at least 32 characters')
        self.session_id = uuid.uuid4().hex
        self.directory = self.grasp.root / 'work/agent'
        self.directory.mkdir(parents=True, mode=0o700, exist_ok=True)
        self.database = self.directory / 'tasks.sqlite'
        self.active = None
        self.recording_error = None
        self.grasp.agent_recording_error = None
        self.stop_lock = threading.Lock()
        with closing(self.connect()) as db, db:
            db.execute(
                'CREATE TABLE IF NOT EXISTS tasks (id TEXT PRIMARY KEY, '
                'fingerprint TEXT NOT NULL, body TEXT NOT NULL)'
            )
            for task_id, body in db.execute(
                'SELECT id,body FROM tasks'
            ).fetchall():
                value = json.loads(body)
                if not value['terminal']:
                    value.update(
                        state='interrupted',
                        terminal=True,
                        outcome_unknown=True,
                        error=(
                            '服务已重启；执行结果未知。先观察现场，'
                            '不要用新编号自动重试。'
                        ),
                        updated_at=time.time(),
                    )
                    db.execute(
                        'UPDATE tasks SET body=? WHERE id=?',
                        (json.dumps(value, ensure_ascii=False), task_id),
                    )
        self.database.chmod(0o600)
        self.grasp.agent_observer = self.observe_job

    def connect(self):
        return sqlite3.connect(self.database, timeout=3)

    def authorized(self, header):
        return (
            isinstance(header, str)
            and hmac.compare_digest(
                header.encode(), ('Bearer ' + self.token).encode()
            )
        )

    def configuration(self):
        return json.loads(self.grasp.config_path.read_text())

    def robot(self, writes=False):
        config = self.configuration()
        return RobotClient(
            config['robot_ip'], config['robot_port'], writes=writes
        )

    def get_task(self, task_id):
        with self.grasp.lock:
            if self.recording_error:
                raise AgentError(
                    'TASK_STATE_UNCERTAIN',
                    '任务状态记录失败，请查询设备并人工核对，'
                    '不要重新发送动作。',
                )
            with closing(self.connect()) as db:
                row = db.execute(
                    'SELECT body FROM tasks WHERE id=?', (task_id,)
                ).fetchone()
            if row is None:
                raise AgentError('TASK_NOT_FOUND', '找不到该任务编号。')
            return json.loads(row[0])

    def save_task(self, task):
        with closing(self.connect()) as db, db:
            db.execute(
                'UPDATE tasks SET body=? WHERE id=?',
                (json.dumps(task, ensure_ascii=False), task['task_id']),
            )

    def observe_job(self, status):
        # GraspController invokes this under its lock, before another job can
        # start.
        if self.active is None:
            return
        try:
            task = self.get_task(self.active)
            terminal = not status.get('busy', False)
            result = {
                k: status.get(k)
                for k in [
                    'state',
                    'busy',
                    'operation',
                    'message',
                    'error',
                    'plan',
                    'placement_plan',
                    'gripper_contact',
                    'reset_target_kind',
                ]
            }
            if task['operation'] == 'capture_scene_snapshot':
                result.update(
                    scene_id=status.get('scene_id'),
                    scene_snapshot=status.get('scene_snapshot'),
                )
            if (
                task['operation']
                in ('plan_held_object_motion', 'plan_release', 'execute_plan')
            ):
                result.update(
                    {
                        k: status.get(k)
                        for k in (
                            'execution_result', 'diagnostics_ref', 'recovery'
                        )
                    }
                )
                result['object_motion_plan'] = public_object_plan(
                    status.get('object_motion_plan')
                )
            if (
                task['operation']
                in ('plan_fixed_pick_place', 'execute_fixed_pick_place')
            ):
                result.update(
                    {
                        k: status.get(k)
                        for k in (
                            'fixed_pick_place_plan',
                            'execution_result',
                            'diagnostics_ref',
                            'recovery',
                        )
                    }
                )
            if task['operation'] in ('reorient', 'preview_reorient'):
                result['reorientation_plan'] = status.get('reorientation_plan')
                result.update(
                    {k: status.get(k) for k in ('diagnostics_ref', 'recovery')}
                )
            result['held_object'] = self.grasp.held_snapshot()
            task.update(
                state=status.get('state', 'running'),
                terminal=terminal,
                updated_at=time.time(),
                result=json_value(result),
                verification=(
                    'controller_feedback_only; object placement/contact/slip '
                    'not independently verified'
                ),
            )
            if (
                terminal
                and (result.get('recovery') or {}).get('outcome_unknown')
            ):
                task['outcome_unknown'] = True
            self.save_task(task)
            if terminal:
                self.active = None
        except Exception as error:
            self.recording_error = type(error).__name__
            self.grasp.agent_recording_error = self.recording_error

    def status(self):
        result = self.tracker.snapshot()
        return {
            'session_id': self.session_id,
            'tracking': {
                k: result.get(k)
                for k in [
                    'state',
                    'error',
                    'object_id',
                    'object_name',
                    'model_ready',
                    'auto_detection_supported',
                    'pose_mode',
                    'pose_mode_revision',
                    'pose_semantics',
                    'detector_backend',
                    'local_detector_name',
                    'detection_model',
                    'detection_effort',
                    'effective_detection_prompt',
                    'detection_seconds',
                    'camera_age_s',
                    'pose_age_s',
                    'quality',
                    'bottle_candidates',
                    'selection_id',
                ]
            },
            'grasp': self.agent_controller_state(result['grasp']),
            'active_task_id': self.active,
            'task_recording_error': self.recording_error,
        }

    @staticmethod
    def agent_controller_state(state):
        # Old UI compatibility fields are not the parameters used by bound
        # grasps.
        result = {
            key: value
            for key, value in state.items()
            if key not in (
                'force_config', 'open_position_config', 'scene_snapshot'
            )
        }
        if 'object_motion_plan' in result:
            result['object_motion_plan'] = public_object_plan(
                result['object_motion_plan']
            )
        return result

    def frame(self, selection=False):
        with self.tracker.lock:
            frame = self.tracker.frozen if selection else self.tracker.latest
            if selection and self.tracker.state['state'] != 'choose_candidate':
                raise AgentError(
                    'SELECTION_EXPIRED', '候选画面已失效，请重新识别。'
                )
            if (
                frame is None
                or time.time() - frame['time'] > (30. if selection else 1.)
            ):
                raise AgentError(
                    'CAMERA_UNAVAILABLE',
                    (
                        '候选画面超过30秒，请重新识别。'
                        if selection
                        else '没有一秒内的新鲜相机画面。'
                    ),
                )
            image = frame['bgr'].copy()
            stamp, sequence = frame['time'], frame['sequence']
            camera = copy.deepcopy(frame.get('camera'))
            selection_id = self.tracker.generation if selection else None
        ok, encoded = cv2.imencode(
            '.jpg', image, [cv2.IMWRITE_JPEG_QUALITY, 80]
        )
        if not ok:
            raise AgentError('IMAGE_ENCODING_FAILED', '相机画面编码失败。')
        return {
            'captured_at': stamp,
            'frame_age_s': time.time() - stamp,
            'sequence': sequence,
            'width': image.shape[1],
            'height': image.shape[0],
            'camera': camera,
            'image_base64': base64.b64encode(encoded).decode(),
            'mime_type': 'image/jpeg',
            'selection_id': selection_id,
            'image_kind': (
                'frozen_selection_frame'
                if selection
                else 'latest_raw_color_frame_not_frozen_selection'
            ),
        }

    def pose(self):
        with self.tracker.lock:
            try:
                observation = self.tracker.grasp_observation()
            except ValueError as error:
                raise AgentError('POSE_NOT_READY', str(error)) from error
            return {
                'object_id': observation['object_id'],
                'object_name': observation.get('object_name'),
                'pose_mode': observation.get('pose_mode', '6d'),
                'axis_z_camera': [row[2] for row in observation['pose'][:3]],
                'spin_observable': observation.get('pose_mode', '6d') != '5d',
                'camera_T_object': observation['pose'],
                'coordinate_frame': 'color_optical_frame',
                'translation_unit': 'metre',
                'captured_at': observation['time'],
                'age_s': time.time() - observation['time'],
                'generation': observation['generation'],
                'model_sha256': observation.get('model_sha256'),
                'grasp_observation_ready': True,
                'note': (
                    '不是基座坐标；执行抓取时后端重新获取新鲜观测并完成坐标换算'
                    '。5D模式下矩阵仅代表位置和Z轴，绕物品Z轴角度不可观测、'
                    '不应作为约束。'
                ),
            }

    def robot_state(self):
        robot = self.robot()
        arm = robot.query('get_current_arm_state')['arm_state']
        raw = np.asarray(arm['pose'], dtype=float)
        held = self.grasp.held_snapshot()
        held_pose = None
        if held is not None and held.get('object_T_tcp') is not None:
            pose = (
                controller_pose(arm['pose'])
                @ np.linalg.inv(transform(held['object_T_tcp']))
            )
            held_pose = {
                'object_id': held['object_id'],
                'hold_id': held['hold_id'],
                'orientation_version': held.get('orientation_version', 0),
                'base_T_object': pose.tolist(),
                'pose_mode': held.get('pose_mode', '6d'),
                'source': (
                    'robot_feedback_and_saved_rigid_attachment_not_visual_verif'
                    'ication'
                ),
            }
        return {
            'observed_at': time.time(),
            'motion_sent': False,
            'held_object_pose': held_pose,
            'joint_deg': (
                np.asarray(arm['joint'], dtype=float) / 1000
            ).tolist(),
            'tcp_position_m': (raw[:3] / 1e6).tolist(),
            'tcp_rpy_rad': (raw[3:] / 1000).tolist(),
            'arm_errors': arm.get('err'),
            'joint_enabled': robot.query('get_joint_en_state'),
            'joint_errors': robot.query('get_joint_err_flag'),
            'tool_frame': robot.query('get_current_tool_frame'),
            'work_frame': robot.query('get_current_work_frame'),
            'gripper': robot.query('get_rm_plus_state_info')['state_info'],
            'controller': self.agent_controller_state(self.grasp.snapshot()),
            'active_task_id': self.active,
        }

    def vision(self, operation, args):
        with self.tracker.lock, self.grasp.lock:
            if self.grasp.status['busy'] or self.grasp.status.get('stopping'):
                raise AgentError('BUSY', '机械臂任务进行中，不能修改跟踪目标。')
            if (
                self.grasp.held_object
                and self.grasp.held_object.get('scene_id')
            ):
                raise AgentError(
                    'SCENE_FROZEN',
                    '快照持物期间不重新定位环境，请引用开始时的场景',
                )
            if operation == 'select_object' and set(args) == {'object_id'}:
                self.tracker.switch_object(args['object_id'])
            elif (
                operation == 'start_tracking'
                and set(args) == {'object_id', 'box'}
            ):
                if args['object_id'] != self.tracker.state.get('object_id'):
                    raise AgentError(
                        'OBJECT_CHANGED', '当前物品与请求不一致，先选择物品。'
                    )
                if args['box'] is None:
                    self.tracker.automatic()
                else:
                    frozen = self.tracker.freeze()
                    self.tracker.initialize(
                        {
                            'selection_id': frozen['selection_id'],
                            'box': args['box'],
                        }
                    )
            elif (
                operation == 'choose_target'
                and set(args) == {'selection_id', 'index'}
            ):
                self.tracker.choose(args)
            else:
                raise AgentError('INVALID_ARGUMENTS', '未知的跟踪操作或参数。')
            return {
                'accepted': True, 'motion_sent': False, 'status': self.status()
            }

    def start(self, request):
        if (
            not isinstance(request, dict)
            or set(request) != {'operation', 'request_id', 'arguments'}
        ):
            raise AgentError(
                'INVALID_ARGUMENTS', '需要 operation、request_id 和 arguments。'
            )
        task_id, operation, args = (
            request['request_id'], request['operation'], request['arguments']
        )
        if (
            not isinstance(task_id, str)
            or not re.fullmatch(r'[A-Za-z0-9_-]{8,96}', task_id)
        ):
            raise AgentError(
                'INVALID_REQUEST_ID',
                'request_id 必须是 8–96 个字母、数字、连字符或下划线；'
                '重试时复用原编号。',
            )
        schemas = {
            'capture_scene_snapshot': {'targets', 'environment_static'},
            'grasp': {'object_id'},
            'preview_grasp': {'object_id'},
            'place': {'hold_id', 'orientation_version', 'placement_revision'},
            'preview_place': {
                'hold_id', 'orientation_version', 'placement_revision'
            },
            'reorient': {
                'object_id',
                'hold_id',
                'orientation_version',
                'target_rpy_deg',
                'position_mode',
            },
            'preview_reorient': {
                'object_id',
                'hold_id',
                'orientation_version',
                'target_rpy_deg',
                'position_mode',
            },
            'reset': {'target'},
            'plan_held_object_motion': {
                'object_id', 'hold_id', 'orientation_version', 'goal'
            },
            'plan_release': {
                'object_id',
                'hold_id',
                'orientation_version',
                'goal',
                'retreat_vector_base_m',
            },
            'execute_plan': {'plan_id'},
            'execute_fixed_pick_place': {'plan_id'},
            'plan_fixed_pick_place': {
                'scene_id',
                'source_instance_id',
                'reference_instance_id',
                'free_slots',
                'release_gap_m',
            },
        }
        allowed = schemas.get(operation)
        valid = (
            isinstance(args, dict)
            and allowed is not None
            and (
                set(args) == allowed
                or (
                    operation in ('grasp', 'preview_grasp')
                    and set(args) == {'object_id', 'scene_id', 'instance_id'}
                )
                or (
                    operation in ('reorient', 'preview_reorient')
                    and set(args) == allowed | {'max_rotation_step_deg'}
                )
            )
        )
        if not valid:
            raise AgentError('INVALID_ARGUMENTS', '未知的动作或参数。')
        if operation == 'plan_fixed_pick_place':
            try:
                checked_fixed_request(args)
            except (ValueError, TypeError) as error:
                raise AgentError('INVALID_ARGUMENTS', str(error)) from error
        if (
            operation == 'execute_fixed_pick_place'
            and (
                not isinstance(args['plan_id'], str)
                or not re.fullmatch('[a-f0-9]{32}', args['plan_id'])
            )
        ):
            raise AgentError('INVALID_ARGUMENTS', '无效固定方案编号')
        if (
            operation in ('grasp', 'preview_grasp')
            and (
                not isinstance(args['object_id'], str)
                or not args['object_id'].strip()
            )
        ):
            raise AgentError('INVALID_ARGUMENTS', '必须指定非空的物品编号。')
        if (
            operation in ('place', 'preview_place')
            and (
                not isinstance(args['hold_id'], str)
                or not args['hold_id']
                or type(args['orientation_version']) is not int
                or args['orientation_version'] < 0
                or not isinstance(args['placement_revision'], str)
            )
        ):
            raise AgentError(
                'INVALID_ARGUMENTS', '夹持编号、朝向版本或放置版本无效。'
            )
        if operation in ('reorient', 'preview_reorient'):
            if (
                any(
                    not isinstance(args[k], str) or not args[k].strip()
                    for k in ('object_id', 'hold_id')
                )
                or type(args['orientation_version']) is not int
                or args['orientation_version'] < 0
            ):
                raise AgentError(
                    'INVALID_ARGUMENTS', '物品、夹持编号及朝向版本无效。'
                )
            try:
                rotation = rotation_from_rpy_deg(
                    args['target_rpy_deg']
                ).tolist()
                checked_position_request(args['position_mode'])
                checked_step(args.get('max_rotation_step_deg', 15.))
            except ValueError as error:
                raise AgentError('INVALID_ARGUMENTS', str(error)) from error
        if operation in ('plan_held_object_motion', 'plan_release'):
            try:
                identity(args)
                checked_goal(args['goal'], operation == 'plan_release')
                if operation == 'plan_release':
                    vector(
                        args['retreat_vector_base_m'],
                        'retreat_vector_base_m',
                        .3,
                    )
            except ValueError as error:
                raise AgentError('INVALID_ARGUMENTS', str(error)) from error
        if (
            operation == 'execute_plan'
            and (
                not isinstance(args['plan_id'], str)
                or not re.fullmatch('[a-f0-9]{32}', args['plan_id'])
            )
        ):
            raise AgentError(
                'INVALID_ARGUMENTS', 'plan_id必须来自本后端的规划结果'
            )
        if operation == 'capture_scene_snapshot':
            try:
                checked_targets(args['targets'])
            except ValueError as e:
                raise AgentError('INVALID_ARGUMENTS', str(e)) from e
            if args['environment_static'] is not True:
                raise AgentError('INVALID_ARGUMENTS', '当前只支持静态场景快照')
        if operation in ('grasp', 'preview_grasp') and 'scene_id' in args:
            if any(
                (
                    not isinstance(args[k], str)
                    or not args[k]
                    or len(args[k]) > 100
                )
                for k in ('scene_id', 'instance_id')
            ):
                raise AgentError(
                    'INVALID_ARGUMENTS', '请先采集场景并提供有效实例编号'
                )
        fingerprint = hashlib.sha256(
            json.dumps(request, sort_keys=True, allow_nan=False).encode()
        ).hexdigest()
        with self.tracker.lock, self.grasp.lock:
            with closing(self.connect()) as db:
                existing = db.execute(
                    'SELECT fingerprint,body FROM tasks WHERE id=?', (task_id,)
                ).fetchone()
            if existing:
                if existing[0] != fingerprint:
                    raise AgentError(
                        'REQUEST_ID_CONFLICT',
                        '同一 request_id 不能对应不同动作或参数。',
                    )
                return {**self.get_task(task_id), 'deduplicated': True}
            if self.recording_error:
                raise AgentError(
                    'TASK_STATE_UNCERTAIN', '先处理任务记录故障，再开始动作。'
                )
            if (
                self.grasp.status['busy']
                or self.grasp.status.get('stopping')
                or self.active
            ):
                raise AgentError('BUSY', '已有任务正在处理或停止状态尚未确认。')
            if (
                (
                    operation
                    in (
                        'grasp',
                        'place',
                        'reset',
                        'reorient',
                        'execute_plan',
                        'execute_fixed_pick_place',
                    )
                )
                and self.grasp.mode != 'execute'
            ):
                raise AgentError(
                    'EXECUTION_DISABLED',
                    '服务处于预览模式，执行工具不可发送动作。',
                )
            task = {
                'task_id': task_id,
                'operation': operation,
                'arguments': args,
                'session_id': self.session_id,
                'state': 'starting',
                'terminal': False,
                'created_at': time.time(),
                'updated_at': time.time(),
            }
            with closing(self.connect()) as db, db:
                db.execute(
                    'INSERT INTO tasks VALUES (?,?,?)',
                    (
                        task_id,
                        fingerprint,
                        json.dumps(task, ensure_ascii=False),
                    ),
                )
            self.active = task_id
            try:
                if (
                    operation
                    in ('plan_fixed_pick_place', 'execute_fixed_pick_place')
                ):
                    self.grasp.start_fixed_pick_place(
                        args,
                        task_id,
                        execute=operation == 'execute_fixed_pick_place',
                    )
                elif operation in ('grasp', 'preview_grasp'):
                    extra = {
                        k: args[k]
                        for k in ('scene_id', 'instance_id')
                        if k in args
                    }
                    self.grasp.start(
                        preview=operation == 'preview_grasp',
                        expected_object_id=args['object_id'],
                        **extra,
                    )
                elif operation == 'capture_scene_snapshot':
                    self.grasp.start_scene_capture(
                        args['targets'], args['environment_static'], task_id
                    )
                elif operation == 'reset':
                    self.grasp.start_reset(args['target'])
                elif operation in ('plan_held_object_motion', 'plan_release'):
                    self.grasp.start_object_plan(
                        args,
                        release=operation == 'plan_release',
                        request_id=task_id,
                    )
                elif operation == 'execute_plan':
                    self.grasp.start_execute_object_plan(
                        args['plan_id'], task_id
                    )
                elif operation in ('reorient', 'preview_reorient'):
                    held = self.grasp.held_snapshot()
                    if (
                        held is None
                        or args['hold_id'] != held['hold_id']
                        or args['object_id'] != held['object_id']
                        or (
                            args['orientation_version']
                            != held['orientation_version']
                        )
                    ):
                        raise AgentError(
                            'HOLD_CHANGED',
                            '物品、夹持关系或朝向已变化，'
                            '先读取状态并重新规划。',
                        )
                    value = {
                        k: held[k]
                        for k in [
                            'object_id',
                            'hold_id',
                            'model_sha256',
                            'orientation_version',
                        ]
                    }
                    value.update(
                        rotation=rotation,
                        position_mode=args['position_mode'],
                        max_rotation_step_deg=args.get(
                            'max_rotation_step_deg', 15.
                        ),
                    )
                    self.grasp.start_reorient(
                        value, preview=operation == 'preview_reorient'
                    )
                else:
                    held = self.grasp.held_snapshot()
                    if (
                        held is None
                        or args['hold_id'] != held['hold_id']
                        or (
                            args['orientation_version']
                            != held['orientation_version']
                        )
                    ):
                        raise AgentError(
                            'HOLD_CHANGED',
                            '夹持关系或朝向已变化，先查询状态并重新规划。',
                        )
                    value = {
                        k: held[k]
                        for k in [
                            'hold_id',
                            'object_id',
                            'model_sha256',
                            'orientation_version',
                        ]
                    }
                    value['revision'] = args['placement_revision']
                    self.grasp.start_placement(
                        value, preview=operation == 'preview_place'
                    )
                self.observe_job(self.grasp.status)
            except Exception as error:
                # Start methods validate before launching their worker. This
                # record is never replayed.
                task.update(
                    state='rejected',
                    terminal=True,
                    updated_at=time.time(),
                    error={
                        'code': getattr(error, 'payload', {}).get(
                            'code', 'PRECONDITION_FAILED'
                        ),
                        'message': str(error),
                    },
                )
                self.save_task(task)
                self.active = None
            return self.get_task(task_id)

    def stop_robot(self):
        if not self.stop_lock.acquire(blocking=False):
            raise AgentError(
                'STOP_IN_PROGRESS', '已有停止请求正在处理；请查询状态。'
            )
        record = {
            'requested_at': time.time(),
            'stop_acknowledged': False,
            'stationary_observed': False,
        }
        try:
            with self.grasp.lock:
                self.grasp.status['stopping'] = True
                self.grasp.cancelled.set()
            robot = self.robot(writes=True)
            # Also stops the gripper, without opening or releasing it.
            robot.stop()
            record['stop_acknowledged'] = True
            samples = []
            deadline = time.monotonic() + 2.
            while time.monotonic() < deadline:
                arm = robot.query('get_current_arm_state')['arm_state']
                grip = robot.query('get_rm_plus_state_info')['state_info']
                joints = np.asarray(arm['joint'], dtype=float)
                if joints.shape != (7,) or not np.isfinite(joints).all():
                    raise ValueError('无效的停止后关节反馈')
                samples.append((time.monotonic(), joints))
                samples = samples[-3:]
                with self.grasp.lock:
                    busy = self.grasp.status['busy']
                if (
                    len(samples) == 3
                    and samples[-1][0] - samples[0][0] >= .1
                    and (
                        np.ptp(np.stack([v for _, v in samples]), axis=0).max()
                        <= 30
                    )
                    and grip.get('speed') == [0]
                    and not busy
                ):
                    record['stationary_observed'] = True
                    break
                time.sleep(.06)
            if not record['stationary_observed']:
                raise ValueError(
                    '停止已应答，但未确认机械臂、夹爪和任务均已停止；'
                    '请现场核对。'
                )
            with self.grasp.lock:
                self.grasp.status['stopping'] = False
            return {**record, 'note': '软件停止，不替代现场急停；未松爪。'}
        except Exception as error:
            record['error'] = str(error)
            raise AgentError(
                'STOP_UNCONFIRMED',
                str(error),
                stop_acknowledged=record['stop_acknowledged'],
                stationary_observed=False,
                action_required='使用现场急停并核对；不要开始新动作。',
            ) from error
        finally:
            try:
                directory = self.directory / 'stops'
                directory.mkdir(exist_ok=True)
                (directory / f'{time.time_ns()}.json').write_text(
                    json.dumps(record, ensure_ascii=False)
                )
            finally:
                self.stop_lock.release()

    def get(self, path, params):
        if path == '/api/agent/status':
            return self.status()
        if path == '/api/agent/frame':
            return self.frame()
        if path == '/api/agent/selection-frame':
            return self.frame(selection=True)
        if path == '/api/agent/pose':
            return self.pose()
        if path == '/api/agent/robot':
            return self.robot_state()
        if path == '/api/agent/objects':
            return {'objects': scan_objects(self.tracker.models_root)}
        if path == '/api/agent/profiles':
            return self.grasp.grasp_profiles(params.get('object_id', [''])[0])
        if path == '/api/agent/scene-snapshot':
            return self.grasp.scene_snapshot(params.get('scene_id', [''])[0])
        if path == '/api/agent/object-geometry':
            return self.grasp.object_geometry_info(
                params.get('object_id', [''])[0]
            )
        if path == '/api/agent/task-diagnostics':
            task = self.get_task(params.get('task_id', [''])[0])
            ref = (task.get('result') or {}).get('diagnostics_ref')
            if not ref:
                return {
                    'task': task,
                    'diagnostics_available': False,
                    'note': (
                        '该任务没有关联的新诊断文件；已有结果及错误见task。'
                    ),
                }
            patterns = {
                'object_motion_jobs': r'[0-9]+_[a-f0-9]{6}\.json',
                'reorient_jobs': r'[0-9]+\.json',
            }
            if (
                ref.get('directory') not in patterns
                or not re.fullmatch(
                    patterns[ref['directory']], ref.get('file', '')
                )
            ):
                raise AgentError('DIAGNOSTICS_INVALID', '诊断引用无效')
            path = (
                self.grasp.root / 'work/grasp' / ref['directory'] / ref['file']
            )
            if path.is_symlink() or path.stat().st_size > 2 * 1024 * 1024:
                raise AgentError('DIAGNOSTICS_INVALID', '诊断文件无效或过大')
            return {
                'task_id': task['task_id'],
                'diagnostics_available': True,
                'diagnostics': json.loads(path.read_text()),
            }
        if path.startswith('/api/agent/tasks/'):
            return self.get_task(path.rsplit('/', 1)[1])
        raise AgentError('NOT_FOUND', '未知的 Agent 接口。')

    def post(self, path, value):
        if path == '/api/agent/tasks':
            return self.start(value)
        if (
            path == '/api/agent/cancel-planning'
            and isinstance(value, dict)
            and set(value) == {'task_id'}
            and isinstance(value['task_id'], str)
        ):
            with self.grasp.lock:
                if self.active != value['task_id']:
                    return {'cancel_requested': False, 'motion_sent': False}
                task = self.get_task(self.active)
                if (
                    (
                        task['operation']
                        not in (
                            'plan_fixed_pick_place',
                            'plan_release',
                            'plan_held_object_motion',
                        )
                    )
                    or self.grasp.actuated
                ):
                    raise AgentError(
                        'NOT_READONLY_PLANNING', '此接口只能取消只读规划'
                    )
                self.grasp.cancelled.set()
                return {'cancel_requested': True, 'motion_sent': False}
        if path == '/api/agent/stop' and value == {}:
            return self.stop_robot()
        if (
            path == '/api/agent/vision'
            and isinstance(value, dict)
            and set(value) == {'operation', 'arguments'}
        ):
            if not isinstance(value['arguments'], dict):
                raise AgentError('INVALID_ARGUMENTS', 'arguments 必须为对象。')
            return self.vision(value['operation'], value['arguments'])
        raise AgentError('NOT_FOUND', '未知的 Agent 接口或参数。')
