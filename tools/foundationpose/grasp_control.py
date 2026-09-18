"""First-trial grasp coordinator. Button-triggered execution with robot write gates."""
from pathlib import Path
import copy
import fcntl
import hashlib
import json
import subprocess
import threading
import time

import numpy as np
import yaml

from grasp_geometry import controller_pose, object_in_base, plan_grasp, workspace_blockers, contact_result, rotation_error, sample_motion
from grasp_robot import RobotClient


class GraspController:
    def __init__(self, root, tracker, config_path, mode='execute'):
        self.root, self.tracker, self.config_path, self.mode = Path(root), tracker, Path(config_path), mode
        self.lock = threading.RLock()
        self.cancelled = threading.Event()
        self.thread = None
        self.actuated = False
        self.robot = None
        self.force_override = None
        self.force_acknowledged = False
        self.force_error = None
        self.status = {'state': 'idle', 'busy': False, 'mode': mode, 'motion_enabled': mode == 'execute',
                       'message': '抓取后按配置竖直抬升。' if mode == 'execute' else '方案预览模式，不发送运动或夹爪写入命令。'}

    def snapshot(self):
        with self.lock:
            return {**self.status, 'force_config': self.force_config()}

    def force_config(self):
        grip = json.loads(self.config_path.read_text())['gripper']
        value = self.force_override if self.force_override is not None else grip['force_target_raw']
        message = ('力度指令已下发并收到成功回复；下一次抓取也使用此值。'
                   if self.force_acknowledged else '尚未从本次页面下发；下一次抓取将使用此值。')
        if self.mode == 'preview':
            message = '预览模式：仅更新下一次方案的配置，不下发硬件。'
        if self.force_error:
            message = '下发未确认：' + self.force_error + '；设备实际值未知，未自动重试。'
        return {'value': value, 'minimum': 1, 'maximum': 100,
                'hardware_applied': self.force_acknowledged,
                'write_acknowledged': self.force_acknowledged, 'readback_verified': False,
                'error': self.force_error, 'message': message}

    def set_force_config(self, value):
        with self.lock:
            if self.status['busy']:
                raise ValueError('抓取任务进行中，不能更改力度')
            if type(value) is not int or not 1 <= value <= 100:
                raise ValueError('力度必须为 1–100 的整数原始值，不能按牛顿输入')
            if self.mode == 'preview':
                self.force_override = value
                self.status.update(plan=None)
                return self.force_config()
            config = json.loads(self.config_path.read_text())
            grip = config['gripper']
            directory = self.root / 'work/grasp'
            directory.mkdir(parents=True, exist_ok=True)
            report = {'started_at': time.time(), 'requested_force_raw': value,
                      'write_attempted': False, 'write_acknowledged': False}
            self.status.update(plan=None)
            try:
                # Share the grasp lock so another service cannot write during a grasp.
                with (directory / 'robot.lock').open('a') as robot_lock:
                    fcntl.flock(robot_lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
                    robot = RobotClient(config['robot_ip'], config['robot_port'], writes=True)
                    if robot.query('get_rm_plus_mode').get('mode') != 115200:
                        raise ValueError('RM Plus 通信模式不匹配')
                    base = robot.query('get_rm_plus_base_info')['base_info']
                    if (base.get('manu'), base.get('id'), base.get('dof'), base.get('type')) != (grip['vendor'], grip['device_id'], 1, 1):
                        raise ValueError('夹爪身份不匹配')
                    state = robot.query('get_rm_plus_state_info')['state_info']
                    if contact_result(state, grip) == 'fault' or state.get('speed') != [0]:
                        raise ValueError('夹爪异常或正在运动，不能更改力度')
                    if not base.get('force') or not 1 <= value <= base['force_up'][0]:
                        raise ValueError('力度超出设备支持范围')
                    self.force_acknowledged = False
                    report['write_attempted'] = True
                    robot.set_register(1220, value)
                    self.force_override = value
                    self.force_acknowledged = True
                    self.force_error = None
                    report['write_acknowledged'] = True
            except Exception as error:
                self.force_acknowledged = False
                self.force_error = str(error)
                report['error'] = str(error)
                raise ValueError('力度下发未确认：' + str(error)) from error
            finally:
                records = directory / 'force_commands'
                records.mkdir(exist_ok=True)
                (records / f'{time.time_ns()}.json').write_text(json.dumps(report, ensure_ascii=False, indent=2))
            return self.force_config()

    def open_gripper(self):
        with self.lock:
            if self.status['busy']:
                raise ValueError('抓取任务进行中，不能打开夹爪')
            if self.mode != 'execute':
                raise ValueError('预览模式不发送夹爪运动指令')
            config = json.loads(self.config_path.read_text())
            grip = config['gripper']
            directory = self.root / 'work/grasp'
            directory.mkdir(parents=True, exist_ok=True)
            report = {'started_at': time.time(), 'target_position': 1000,
                      'write_attempted': False, 'write_acknowledged': False}
            try:
                with (directory / 'robot.lock').open('a') as robot_lock:
                    fcntl.flock(robot_lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
                    robot = RobotClient(config['robot_ip'], config['robot_port'], writes=True)
                    if robot.query('get_rm_plus_mode').get('mode') != 115200:
                        raise ValueError('RM Plus 通信模式不匹配')
                    base = robot.query('get_rm_plus_base_info')['base_info']
                    if (base.get('manu'), base.get('id'), base.get('dof'), base.get('type')) != (grip['vendor'], grip['device_id'], 1, 1):
                        raise ValueError('夹爪身份不匹配')
                    state = robot.query('get_rm_plus_state_info')['state_info']
                    report['initial_gripper_state'] = state
                    if contact_result(state, grip) == 'fault' or state.get('speed') != [0]:
                        raise ValueError('夹爪异常或正在运动')
                    report['write_attempted'] = True
                    robot.move_gripper(1000)
                    report['write_acknowledged'] = True
                    self.status.update(plan=None)
            except Exception as error:
                report['error'] = str(error)
                raise ValueError('打开夹爪未确认：' + str(error)) from error
            finally:
                records = directory / 'open_commands'
                records.mkdir(exist_ok=True)
                (records / f'{time.time_ns()}.json').write_text(json.dumps(report, ensure_ascii=False, indent=2))
            return {'target_position': 1000, 'write_acknowledged': True,
                    'message': '打开指令已接受，目标位置 1000；实际到位可通过“读取夹爪状态”确认。'}

    def update(self, **values):
        with self.lock:
            self.status.update(values)

    def hardware_status(self):
        config = json.loads(self.config_path.read_text())
        robot = RobotClient(config['robot_ip'], config['robot_port'], writes=False)
        try:
            software = robot.query('get_arm_software_info')
            mode = robot.query('get_rm_plus_mode')['mode']
            base = robot.query('get_rm_plus_base_info')['base_info']
            state = robot.query('get_rm_plus_state_info')['state_info']
            names = {0:'正在松开',1:'正在闭合',2:'位置到位停止',3:'力控到位停止',4:'触觉到位停止',5:'电流保护停止',6:'发生故障'}
            return {'mode': mode, 'base': base, 'state': state, 'state_text': names.get(state.get('dof_state', [-1])[0], '未知'),
                    'force_units': config['gripper']['force_units'], 'motion_sent': False,
                    'controller_version': software.get('ctrl_info', {}).get('version'),
                    'force_control_verified': config['gripper']['force_control_verified'],
                    'register_readback_available': False}
        except Exception as error:
            raise ValueError('读取夹爪失败：' + str(error)) from error

    def profile_for(self, config, observation, tool, handeye):
        profile = copy.deepcopy(config['grasp_profile'])
        if profile['orientation'] != 'follow_object':
            return profile
        path = self.root / 'work/grasp/taught_profiles.json'
        records = json.loads(path.read_text()) if path.exists() else {}
        record = records.get(observation['object_id'])
        if record is not None:
            if record['model_sha256'] != observation.get('model_sha256'):
                raise ValueError('物品模型已变化，请重新记录夹法')
            if record['tool_response'] != tool['controller_readback']['tool_response_raw']:
                raise ValueError('工具参数已变化，请重新记录夹法')
            if record.get('flange_T_camera') != handeye['flange_T_color_optical_camera']:
                raise ValueError('手眼标定已变化，请重新记录夹法')
            profile.update(record['profile'])
        return profile

    def teach(self, use_current_point=False):
        if type(use_current_point) is not bool:
            raise ValueError('记录位置选项必须为布尔值')
        observation = copy.deepcopy(self.tracker.grasp_observation())
        if not observation.get('object_id') or not observation.get('model_sha256'):
            raise ValueError('记录夹法需要已登记的物品模型')
        with self.lock:
            if self.status['busy']:
                raise ValueError('已有抓取任务正在处理')
            self.status.update(busy=True, state='teaching', message='正在读取当前夹法，不发送运动…', error=None)
            self.actuated = False
            self.cancelled.clear()
        directory = self.root / 'work/grasp'
        report = {'at': time.time(), 'observation': observation, 'use_current_point': use_current_point, 'motion_sent': False}
        try:
            directory.mkdir(parents=True, exist_ok=True)
            with (directory / 'robot.lock').open('a') as lock:
                fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
                config = json.loads(self.config_path.read_text())
                self.robot = RobotClient(config['robot_ip'], config['robot_port'], writes=False)
                arm, observation = self.paired_observation(observation)
                handeye, tool = self.calibration(config)
                if self.robot.query('get_current_tool_frame') != tool['controller_readback']['tool_response_raw']:
                    raise ValueError('当前工具参数与记录不一致')
                work = self.robot.query('get_current_work_frame')
                if work.get('frame_name') != 'Base' or work.get('pose') != [0]*6:
                    raise ValueError('当前工作坐标系必须是零偏移 Base')
                expected = np.asarray(handeye['camera']['K'], dtype=float).copy();expected[:2] *= .5
                observed = np.asarray(observation.get('K'), dtype=float)
                if observed.shape != (3, 3) or not np.allclose(observed, expected, atol=.05):
                    raise ValueError('实际视觉内参与手眼标定不匹配')
                base_tcp = controller_pose(arm['pose'])
                base_object = object_in_base(base_tcp, tool['flange_T_grip_tcp'], handeye['flange_T_color_optical_camera'], observation['pose'])
                relative = np.linalg.inv(base_object) @ base_tcp
                profile = copy.deepcopy(config['grasp_profile'])
                profile.update(orientation='follow_object', object_R_tcp=relative[:3, :3].tolist())
                profile['approach_axis_tcp'] = [0, 0, 1]  # User-confirmed tool forward axis.
                if use_current_point:
                    profile['grasp_point_object_m'] = relative[:3, 3].tolist()
                # Validate rotation, point, approach and distances before saving.
                plan_grasp(arm['pose'], base_object, profile, config)
                self.check_cancel()
                record = {'profile': profile, 'model_sha256': observation['model_sha256'],
                          'tool_response': tool['controller_readback']['tool_response_raw'],
                          'flange_T_camera': handeye['flange_T_color_optical_camera'], 'recorded_at': time.time()}
                path = directory / 'taught_profiles.json'
                records = json.loads(path.read_text()) if path.exists() else {}
                records[observation['object_id']] = record
                temporary = path.with_suffix('.tmp')
                temporary.write_text(json.dumps(records, ensure_ascii=False, indent=2))
                temporary.replace(path)
                report['record'] = record
                message = '已记录当前物品的朝向' + ('及夹持位置' if use_current_point else '；夹持位置采用记录时的文件配置')
                self.update(state='taught', message=message, plan=None)
                return {'message': message, 'profile': profile, 'motion_sent': False}
        except Exception as error:
            report['error'] = str(error)
            self.update(state='failed', message='记录夹法失败：' + str(error), error=str(error))
            raise ValueError('记录夹法失败：' + str(error)) from error
        finally:
            self.update(busy=False)
            records = directory / 'teach_commands';records.mkdir(exist_ok=True)
            (records / f'{time.time_ns()}.json').write_text(json.dumps(report, ensure_ascii=False, indent=2))

    def start(self, preview=False):
        # Take the tracker lock before the grasp lock (snapshot uses the same order).
        observation = copy.deepcopy(self.tracker.grasp_observation())
        with self.lock:
            if self.status['busy']:
                raise ValueError('已有抓取任务正在处理')
            self.cancelled.clear()
            self.actuated = False
            self.status.update(state='checking', busy=True, message='已锁定当前物品，检查机械臂、夹爪与抓取路径…', error=None, plan=None,
                               target_object_id=observation['object_id'], target_captured_at=observation['time'])
            self.thread = threading.Thread(target=self.run, args=(observation,), kwargs={'preview': preview}, daemon=True)
            self.thread.start()

    def abort(self):
        self.cancelled.set()
        with self.lock:
            active, robot = self.status['busy'] and self.actuated, self.robot
        if active and robot is not None:
            try:
                robot.stop()
            except Exception as error:
                self.update(error=str(error), message='停止指令未全部确认，请使用现场急停')
                raise

    def check_cancel(self):
        if self.cancelled.is_set():
            raise ValueError('抓取已取消')

    def calibration(self, config):
        handeye = yaml.safe_load((self.root / config['handeye_file']).read_text())
        tool = yaml.safe_load((self.root / config['tool_file']).read_text())
        if handeye['acceptance_status'] != 'user_accepted_usable' or not config['nominal_tcp_user_accepted']:
            raise ValueError('手眼标定或当前名义 TCP 未被接受')
        for record, source in [(handeye, handeye['source']), (tool, tool['controller_readback'])]:
            if hashlib.sha256((self.root / source['path']).read_bytes()).hexdigest() != source['sha256']:
                raise ValueError('标定原始记录哈希变化')
        if self.tracker.args.serial != handeye['camera']['serial']:
            raise ValueError('相机与手眼标定不匹配')
        return handeye, tool

    def arm_state(self):
        state = self.robot.query('get_current_arm_state')['arm_state']
        if any(state.get('err', [1])):
            raise ValueError('机械臂存在错误状态')
        return state

    def paired_observation(self, observation):
        # Bind the click-time image to the stationary arm immediately at job start.
        # Do not replace the frozen image with a later tracker result.
        if time.time() - observation['time'] > .6:
            raise ValueError('开始时的视觉快照已过期，请重新点击抓取')
        first = self.arm_state()
        middle = self.arm_state()
        time.sleep(.15)
        last = self.arm_state()
        joints = np.array([x['joint'] for x in [first, middle, last]], dtype=float) / 1000
        poses = np.array([x['pose'] for x in [first, middle, last]], dtype=float)
        if np.ptp(joints, axis=0).max() > .03 or np.ptp(poses[:, :3], axis=0).max() > 200 or np.ptp(poses[:, 3:], axis=0).max() > 3:
            raise ValueError('机械臂未停稳，不能绑定手眼位姿')
        if time.time() - observation['time'] > .6:
            raise ValueError('抓取视觉快照已过期')
        return last, observation

    def check_ik(self, plan, arm, tool, config):
        executable = self.root / config['ik_checker']
        if not executable.is_file():
            return {'ok': False, 'reason': '离线逆解检查器尚未编译'}
        start = np.array(arm['pose'], dtype=float) / np.array([1e6]*3 + [1e3]*3)
        tool_pose = np.array(tool['controller_readback']['tool_response_raw']['pose']) / np.array([1e6]*3 + [1e3]*3)
        points = []
        stages = []
        last = arm['pose']
        for waypoint in plan['waypoints']:
            samples = sample_motion(last, waypoint['controller_pose'])
            points.extend(samples)
            stages.extend([(waypoint.get('name', '未知阶段'), n+1, len(samples)) for n in range(len(samples))])
            last = waypoint['controller_pose']
        if len(points) > 500:
            return {'ok': False, 'reason': '路径检查采样数超过上限'}
        text = ' '.join(map(str, tool_pose)) + '\n' + ' '.join(str(v / 1000) for v in arm['joint']) + '\n' + str(len(points)) + '\n'
        text += '\n'.join(' '.join(map(str, row)) for row in points) + '\n'
        result = subprocess.run([str(executable)], input=text, capture_output=True, text=True, timeout=10)
        fk = [line for line in result.stdout.splitlines() if line.startswith('FK ')]
        ik = [line for line in result.stdout.splitlines() if line.startswith('IK ')]
        if not fk:
            return {'ok': False, 'reason': f'离线检查器没有返回起始位置计算结果，退出码 {result.returncode}；错误输出：{result.stderr.strip()[:300] or "无"}'}
        forward = np.array(list(map(float, fk[0].split()[1:])))
        error = float(np.linalg.norm(forward[:3] - start[:3]))
        angle_error = rotation_error(controller_pose(arm['pose'])[:3, :3],
                                     controller_pose(forward * np.array([1e6]*3 + [1e3]*3))[:3, :3])
        if error > .002 or angle_error > .01:
            return {'ok': False, 'reason': f'离线计算与控制器起始姿态不一致：位置误差 {error*1000:.3f} 毫米（上限 2 毫米），朝向误差 {np.rad2deg(angle_error):.3f} 度（上限 {np.rad2deg(.01):.3f} 度）', 'position_error_m': error}
        previous = np.array(arm['joint']) / 1000
        for sample_index, line in enumerate(ik, 1):
            row = list(map(float, line.split()[1:]))
            if len(row) != 9 or not np.isfinite(row).all() or sample_index > len(points):
                return {'ok':False, 'reason':f'离线检查器第 {sample_index} 条结果格式无效'}
            stage, local_index, count = stages[sample_index-1]
            names = {'pregrasp':'前往预抓点', 'grasp':'接近夹持点', 'lift':'抬升'}
            point = points[sample_index-1]
            context = f'{names.get(stage, stage)}：第 {local_index}/{count} 个采样点（全程第 {sample_index} 个），位置 [' + ', '.join(f'{v*1000:.3f}' for v in point[:3]) + '] 毫米'
            detail = {'ok':False, 'failed_sample':sample_index, 'stage':stage, 'sample_pose':point,
                      'ik_return_code':int(row[0]), 'self_collision_or_limit':int(row[1])}
            if row[0] != 0:
                codes = {1:'未求得关节解', -1:'输入关节角度为空或超出关节限位', -2:'目标朝向的四元数无效'}
                detail['reason'] = context + f'：逆解失败（由目标位置和朝向求关节角度），返回码 {int(row[0])}：' + codes.get(int(row[0]), '未定义错误，需查看设备算法说明') + '；未进行自碰撞或关节跳变判断'
                return detail
            if row[1] != 0:
                detail['reason'] = context + f'：自身碰撞/关节限位检查失败，返回标志 {int(row[1])}（算法把这两种情况合并返回，无法仅凭此标志区分）'
                return detail
            deltas = np.abs(np.array(row[2:]) - previous)
            if deltas.max() > 10:
                joint = int(deltas.argmax())
                detail['max_joint_delta_deg'] = float(deltas[joint])
                detail['reason'] = context + f'：关节 {joint+1} 跳变，从 {previous[joint]:.3f} 度到 {row[joint+2]:.3f} 度，变化 {deltas[joint]:.3f} 度，超过每采样点 10 度上限'
                return detail
            previous = np.array(row[2:])
        if result.returncode == 0 and len(ik) == len(points) and plan.get('return_joint_raw') is not None:
            check = self.check_return(np.rint(previous*1000).astype(int).tolist(), plan['return_joint_raw'],
                                      tool['controller_readback']['tool_response_raw'], config)
            plan['return_check'] = check
            if not check['ok']:
                return check
        if result.returncode != 0 or len(ik) != len(points):
            return {'ok':False, 'reason':f'离线路径检查未完成：退出码 {result.returncode}，返回 {len(ik)}/{len(points)} 个采样点结果；错误输出：{result.stderr.strip()[:300] or "无"}'}
        return {'ok': result.returncode == 0 and len(ik) == len(points), 'sampled_poses': len(points),
                'fk_position_error_m': error, 'scope': 'SDK kinematics and self-collision only; not full environment collision checking'}

    def check_return(self, start_joints_raw, target_joints_raw, tool_raw, config):
        executable = self.root / config['ik_checker']
        tool_pose = np.asarray(tool_raw['pose']) / np.array([1e6]*3 + [1e3]*3)
        rows = [tool_pose, np.asarray(start_joints_raw)/1000, np.asarray(target_joints_raw)/1000,
                config['workspace_min_m'], config['workspace_max_m'], [config['table_z_m']]]
        data = '\n'.join(' '.join(map(str, row)) for row in rows) + '\n'
        result = subprocess.run([str(executable), '--joint-path'], input=data, capture_output=True, text=True, timeout=10)
        lines = result.stdout.splitlines()
        ok = result.returncode == 0 and len(lines) == 1 and lines[0].startswith('RETURN_OK ')
        reason = None
        if not ok:
            reason = f'返回路径检查器未完成：退出码 {result.returncode}；输出：{result.stdout.strip()[:300] or "无"}；错误输出：{result.stderr.strip()[:300] or "无"}'
            if len(lines) == 1 and lines[0].startswith('RETURN_FAIL '):
                fields = lines[0].split()
                if len(fields) >= 3:
                    reasons = []
                    if int(fields[2]) != 0:
                        reasons.append(f'自身碰撞/关节限位检查标志 {fields[2]}（算法合并返回，无法仅凭标志区分）')
                    if len(fields) == 6:
                        point = [float(v) for v in fields[3:]]
                        reasons.extend(workspace_blockers({'current_position_m':point,'waypoints':[]}, config))
                    reason = f'返回路径第 {fields[1]} 个采样点（0 为起点）：' + '；'.join(reasons or ['检查失败；旧检查器未输出坐标，请重新编译以获取边界详情'])
        return {'ok': ok, 'reason': reason,
                'detail': result.stdout.strip()[:500], 'scope': 'joint interpolation approximation; no environment collision model'}

    @staticmethod
    def check_gripper_start(state, grip):
        position = state.get('pos')
        if (not isinstance(position, list) or len(position) != 1
                or type(position[0]) not in (int, float) or not np.isfinite(position[0])
                or not 0 <= position[0] <= 1000):
            raise ValueError('夹爪位置反馈无效，无法确认复位状态')
        if contact_result(state, grip) == 'fault' or state.get('speed') != [0]:
            raise ValueError('夹爪异常或正在运动')
        # The device can retain force-stop status after manual opening (pos=1000).
        # Use the same open-position tolerance as the opening stage.
        opened = abs(position[0] - grip['open_position']) <= 3
        if not opened and (state.get('dof_state') == [3] or any(state.get('force', []))):
            raise ValueError('夹爪可能已夹持物体，请先人工张开复位')

    def run(self, observation, preview=False):
        mode = 'preview' if preview else self.mode
        report = {'started_at': time.time(), 'mode': mode, 'robot_motion_sent': False,
                  'locked_observation': copy.deepcopy(observation)}
        robot_lock = None
        try:
            with self.lock:
                config = json.loads(self.config_path.read_text())
                if self.force_override is not None:
                    config['gripper']['force_target_raw'] = self.force_override
                    config['gripper']['force_limit_raw'] = self.force_override
            report['effective_gripper_config'] = dict(config['gripper'])
            report['config_sha256'] = hashlib.sha256(self.config_path.read_bytes()).hexdigest()
            if mode == 'execute':
                lock_path = self.root / 'work/grasp/robot.lock'
                lock_path.parent.mkdir(parents=True, exist_ok=True)
                robot_lock = lock_path.open('a')
                fcntl.flock(robot_lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
            self.robot = RobotClient(config['robot_ip'], config['robot_port'], writes=mode == 'execute')
            arm, observation = self.paired_observation(observation)
            handeye, tool = self.calibration(config)
            software = self.robot.query('get_arm_software_info')
            report['controller_software'] = software
            if software.get('Product_version') != 'RM75-BI' or software['ctrl_info']['version'] != handeye['installation']['controller_firmware']:
                raise ValueError('机械臂型号/固件与标定记录不匹配')
            if self.robot.query('get_current_tool_frame') != tool['controller_readback']['tool_response_raw']:
                raise ValueError('当前工具/TCP/负载与 grip_tcp 记录不一致')
            work = self.robot.query('get_current_work_frame')
            if work.get('frame_name') != 'Base' or work.get('pose') != [0]*6:
                raise ValueError('当前工作坐标系必须是零偏移 Base')
            if self.robot.query('get_joint_en_state').get('en_state') != [1]*7 or self.robot.query('get_joint_err_flag').get('err_flag') != [0]*7:
                raise ValueError('关节使能或错误状态异常')
            if self.robot.query('get_rm_plus_mode').get('mode') != 115200:
                raise ValueError('RM Plus 通信模式不匹配')
            base = self.robot.query('get_rm_plus_base_info')['base_info']
            gripper_state = self.robot.query('get_rm_plus_state_info')['state_info']
            report['initial_gripper_state'] = copy.deepcopy(gripper_state)
            grip = config['gripper']
            if (base.get('manu'), base.get('id'), base.get('dof'), base.get('type')) != (grip['vendor'], grip['device_id'], 1, 1):
                raise ValueError('夹爪身份不匹配')
            self.check_gripper_start(gripper_state, grip)
            expected_K = np.array(handeye['camera']['K'], dtype=float)
            expected_K[:2] *= .5  # The live adapter uses 640x360 from calibrated 1280x720 color.
            observed_K = np.asarray(observation.get('K'), dtype=float)
            if observed_K.shape != (3, 3) or not np.allclose(observed_K, expected_K, atol=.05):
                raise ValueError('实际视觉内参与手眼标定使用的彩色流不匹配')
            profile = self.profile_for(config, observation, tool, handeye)
            base_object = object_in_base(controller_pose(arm['pose']), tool['flange_T_grip_tcp'], handeye['flange_T_color_optical_camera'], observation['pose'])
            plan = plan_grasp(arm['pose'], base_object, profile, config)
            plan.update(object_id=observation['object_id'], generation=observation['generation'], captured_at=observation['time'], target_locked=True,
                        object_name=observation.get('object_name'),
                        current_arm=arm, gripper_base=base, gripper_state=gripper_state,
                        tool_response=tool['controller_readback']['tool_response_raw'], work_response=work,
                        force_limit_raw=grip['force_limit_raw'], force_units=grip['force_units'])
            blockers = workspace_blockers(plan, config)
            if not base.get('force'):
                blockers.append('当前夹爪未声明支持力度控制')
            plan['ik_check'] = self.check_ik(plan, arm, tool, config)
            if not plan['ik_check']['ok']:
                blockers.append(plan['ik_check'].get('reason', '路径逆解检查失败'))
            plan['execution_blockers'] = blockers
            plan['can_execute'] = mode == 'execute' and not blockers
            report['grasp_profile'] = profile
            report['plan'] = plan
            self.update(plan=plan)
            self.check_cancel()
            if not plan['can_execute']:
                self.update(state='blocked' if blockers else 'preview', message='已生成方案；未发送运动。' + ('\n阻塞原因：\n' + '\n'.join(f'{i+1}. {reason}' for i, reason in enumerate(blockers)) if blockers else '当前为预览模式，所有检查已通过。'))
                return
            self.execute(plan, config, report)
        except Exception as error:
            if self.actuated:
                try:
                    self.robot.stop()
                except Exception as stop_error:
                    report['stop_error'] = str(stop_error)
            self.update(state='cancelled' if self.cancelled.is_set() else 'failed', error=str(error), message=str(error))
            report['error'] = str(error)
        finally:
            self.update(busy=False)
            report['result'] = self.snapshot()
            if robot_lock is not None:
                robot_lock.close()
            directory = self.root / 'work/grasp/jobs'
            directory.mkdir(parents=True, exist_ok=True)
            (directory / f'{time.time_ns()}.json').write_text(json.dumps(report, indent=2, ensure_ascii=False))

    def execute(self, plan, config, report):
        grip = config['gripper']
        if any(type(grip[key]) is not int for key in ['force_target_raw', 'force_limit_raw', 'open_position', 'closed_position']):
            raise ValueError('夹爪参数必须为整数原始值')
        # The base-frame target is fixed for the job; tracking loss or target motion
        # must not change it. Only robot state is rechecked before actuation.
        current = self.arm_state()
        if np.max(np.abs(np.asarray(current['joint']) - plan['current_arm']['joint'])) > 30:
            raise ValueError('机械臂在规划后发生移动')
        if not (0 < grip['force_target_raw'] <= grip['force_limit_raw'] <= plan['gripper_base']['force_up'][0]):
            raise ValueError('夹爪力参数越界')
        self.check_cancel()
        self.actuated = True
        # User verified register 1220 + position following on the installed firmware.
        # Numeric register readback is unavailable; reject failed write acknowledgements.
        self.robot.set_register(1220, grip['force_limit_raw'])
        self.update(state='opening', message='张开夹爪')
        self.robot.move_gripper(grip['open_position'])
        deadline = time.monotonic() + 8
        while time.monotonic() < deadline:
            self.check_cancel()
            state = self.robot.query('get_rm_plus_state_info')['state_info']
            if contact_result(state, grip) == 'fault':
                raise ValueError('夹爪张开过程中异常')
            if state['speed'] == [0] and abs(state['pos'][0] - grip['open_position']) <= 3:
                break
            time.sleep(.1)
        else:
            raise TimeoutError('夹爪未到达张开位置')
        lifting = False
        def watchdog():
            self.check_cancel()
            if self.robot.query('get_current_tool_frame') != plan['tool_response'] or self.robot.query('get_current_work_frame') != plan['work_response']:
                raise ValueError('运动期间工具或工作坐标系发生变化')
            if lifting:
                grip_state = self.robot.query('get_rm_plus_state_info')['state_info']
                if contact_result(grip_state, grip) not in ('force_contact', 'force_stop'):
                    raise ValueError('抬升或返回期间夹持状态丢失或夹爪异常')
            state = self.arm_state()
            point = controller_pose(state['pose'])[:3, 3]
            if np.any(point < np.asarray(config['workspace_min_m'])) or np.any(point > np.asarray(config['workspace_max_m'])) or point[2] < config['table_z_m'] + .01:
                raise ValueError('机械臂离开约定工作区')
        def move_waypoint(waypoint):
            watchdog()
            names = {'pregrasp':'调整朝向并到达预抓取点', 'grasp':'接近夹持点', 'lift':'夹持后竖直抬升'}
            self.update(state='lifting' if lifting else 'moving', message='执行路径：' + names[waypoint['name']])
            report['robot_motion_sent'] = True
            self.robot.move_linear(waypoint['controller_pose'], config['arm_speed_percent'], self.cancelled, watchdog)
            reached = self.arm_state()
            if np.linalg.norm((np.array(reached['pose'][:3]) - waypoint['controller_pose'][:3]) / 1e6) > .002:
                raise ValueError('机械臂实际位置与目标不符')
            if rotation_error(controller_pose(reached['pose'])[:3, :3], controller_pose(waypoint['controller_pose'])[:3, :3]) > np.deg2rad(2):
                raise ValueError('机械臂实际朝向与目标相差超过 2 度')
        for waypoint in plan['waypoints']:
            if waypoint['name'] != 'lift':
                move_waypoint(waypoint)
        self.update(state='closing', message='按指定力度闭合，等待力控接触反馈')
        self.robot.set_register(1220, grip['force_limit_raw'])
        self.robot.move_gripper(grip['closed_position'])
        deadline = time.monotonic() + grip['close_timeout_s']
        stable = 0
        while time.monotonic() < deadline:
            self.check_cancel()
            state = self.robot.query('get_rm_plus_state_info')['state_info']
            result = contact_result(state, grip)
            if result in ('fault', 'empty_close'):
                raise ValueError('夹爪异常或空夹到位')
            stable = stable + 1 if result in ('force_contact', 'force_stop') else 0
            if stable >= 3:
                self.update(gripper_contact=state)
                lifting = True
                move_waypoint(next(w for w in plan['waypoints'] if w['name'] == 'lift'))
                watchdog()
                if plan.get('return_joint_raw') is not None:
                    current = self.arm_state()
                    report['return_check_actual'] = self.check_return(current['joint'], plan['return_joint_raw'], plan['tool_response'], config)
                    if not report['return_check_actual']['ok']:
                        raise ValueError(report['return_check_actual']['reason'])
                    watchdog()
                    self.update(state='returning', message='抬升完成，保持夹持并返回记录的关节角度')
                    self.robot.move_joints(plan['return_joint_raw'], config['arm_speed_percent'], self.cancelled, watchdog)
                    watchdog()
                    reached = self.arm_state()
                    if np.max(np.abs(np.asarray(reached['joint']) - plan['return_joint_raw'])) > 500:
                        raise ValueError('返回关节角度误差超过 0.5 度')
                    self.update(state='completed', message='已完成夹持、抬升并返回记录姿态；夹爪保持闭合。')
                    return
                self.update(state='completed', message=f"已完成夹持并竖直抬升 {plan['lift_height_m'] * 100:g} 厘米，请观察物品是否保持夹持。")
                return
            time.sleep(.1)
        raise TimeoutError('未确认稳定接触，停止本次抓取')
