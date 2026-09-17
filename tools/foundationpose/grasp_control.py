"""First-trial grasp coordinator. Preview by default; robot writes are gated."""
from pathlib import Path
import fcntl
import hashlib
import json
import subprocess
import threading
import time

import numpy as np
import yaml

from grasp_geometry import controller_pose, object_in_base, plan_grasp, workspace_blockers, contact_result
from grasp_robot import RobotClient


class GraspController:
    def __init__(self, root, tracker, config_path, mode='preview'):
        self.root, self.tracker, self.config_path, self.mode = Path(root), tracker, Path(config_path), mode
        self.lock = threading.RLock()
        self.cancelled = threading.Event()
        self.thread = None
        self.actuated = False
        self.robot = None
        self.status = {'state': 'idle', 'busy': False, 'mode': mode, 'motion_enabled': mode == 'execute',
                       'message': '首轮试抓：仅夹持，不抬起。' if mode == 'execute' else '方案预览模式，不发送运动或夹爪写入命令。'}

    def snapshot(self):
        with self.lock:
            return dict(self.status)

    def update(self, **values):
        with self.lock:
            self.status.update(values)

    def hardware_status(self):
        config = json.loads(self.config_path.read_text())
        robot = RobotClient(config['robot_ip'], config['robot_port'], writes=False)
        try:
            mode = robot.query('get_rm_plus_mode')['mode']
            base = robot.query('get_rm_plus_base_info')['base_info']
            state = robot.query('get_rm_plus_state_info')['state_info']
            names = {0:'正在松开',1:'正在闭合',2:'位置到位停止',3:'力控到位停止',4:'触觉到位停止',5:'电流保护停止',6:'发生故障'}
            return {'mode': mode, 'base': base, 'state': state, 'state_text': names.get(state.get('dof_state', [-1])[0], '未知'),
                    'force_units': config['gripper']['force_units'], 'motion_sent': False}
        except Exception as error:
            raise ValueError('读取夹爪失败：' + str(error)) from error

    def start(self):
        with self.lock:
            if self.status['busy']:
                raise ValueError('已有抓取任务正在处理')
            self.cancelled.clear()
            self.actuated = False
            self.status.update(state='checking', busy=True, message='检查定位、机械臂、夹爪与抓取路径…', error=None, plan=None)
            self.thread = threading.Thread(target=self.run, daemon=True)
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

    def paired_observation(self):
        # Only static-arm pairing is supported; never apply current arm pose to
        # an old image taken while the eye-in-hand camera was moving.
        first = self.arm_state()
        bracket_start = time.time()
        deadline = time.monotonic() + 3
        while time.monotonic() < deadline:
            self.check_cancel()
            observation = self.tracker.grasp_observation()
            if observation['time'] >= bracket_start:
                break
            time.sleep(.04)
        else:
            raise ValueError('没有获得机器人静止查询之后的新鲜视觉位姿')
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
        last = start.copy()
        for waypoint in plan['waypoints']:
            target = np.array(waypoint['controller_pose'], dtype=float) / np.array([1e6]*3 + [1e3]*3)
            steps = max(1, int(np.ceil(np.linalg.norm(target[:3] - last[:3]) / .01)))
            points.extend((last + (target - last) * n / steps).tolist() for n in range(1, steps + 1))
            last = target
        text = ' '.join(map(str, tool_pose)) + '\n' + ' '.join(str(v / 1000) for v in arm['joint']) + '\n' + str(len(points)) + '\n'
        text += '\n'.join(' '.join(map(str, row)) for row in points) + '\n'
        result = subprocess.run([str(executable)], input=text, capture_output=True, text=True, timeout=10)
        fk = [line for line in result.stdout.splitlines() if line.startswith('FK ')]
        ik = [line for line in result.stdout.splitlines() if line.startswith('IK ')]
        if not fk:
            return {'ok': False, 'reason': 'SDK 正解检查没有有效输出'}
        forward = np.array(list(map(float, fk[0].split()[1:])))
        error = float(np.linalg.norm(forward[:3] - start[:3]))
        angle_error = float(np.max(np.abs(np.arctan2(np.sin(forward[3:] - start[3:]), np.cos(forward[3:] - start[3:])))))
        if error > .002 or angle_error > .01:
            return {'ok': False, 'reason': '离线 SDK 与控制器当前 TCP 正解不一致', 'position_error_m': error}
        previous = np.array(arm['joint']) / 1000
        for line in ik:
            row = list(map(float, line.split()[1:]))
            if row[0] != 0 or row[1] != 0 or np.max(np.abs(np.array(row[2:]) - previous)) > 10:
                return {'ok': False, 'reason': '路径存在逆解失败、自碰撞/限位或关节跳变'}
            previous = np.array(row[2:])
        return {'ok': result.returncode == 0 and len(ik) == len(points), 'sampled_poses': len(points),
                'fk_position_error_m': error, 'scope': 'SDK kinematics and self-collision only; not full environment collision checking'}

    def run(self):
        report = {'started_at': time.time(), 'mode': self.mode, 'robot_motion_sent': False}
        robot_lock = None
        try:
            config = json.loads(self.config_path.read_text())
            report['config_sha256'] = hashlib.sha256(self.config_path.read_bytes()).hexdigest()
            initial = self.tracker.grasp_observation()
            if initial['object_id'] not in config['objects']:
                raise ValueError('当前物品尚未配置抓取位置；首轮仅支持怡宝水瓶')
            if self.mode == 'execute':
                lock_path = self.root / 'work/grasp/robot.lock'
                lock_path.parent.mkdir(parents=True, exist_ok=True)
                robot_lock = lock_path.open('a')
                fcntl.flock(robot_lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
            self.robot = RobotClient(config['robot_ip'], config['robot_port'], writes=self.mode == 'execute')
            handeye, tool = self.calibration(config)
            software = self.robot.query('get_arm_software_info')
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
            grip = config['gripper']
            if (base.get('manu'), base.get('id'), base.get('dof'), base.get('type')) != (grip['vendor'], grip['device_id'], 1, 1):
                raise ValueError('夹爪身份不匹配')
            if contact_result(gripper_state, grip) == 'fault' or gripper_state.get('speed') != [0]:
                raise ValueError('夹爪异常或正在运动')
            if gripper_state.get('dof_state') == [3] or any(gripper_state.get('force', [])):
                raise ValueError('夹爪可能已夹持物体，请先人工处理')
            arm, observation = self.paired_observation()
            profile = config['objects'].get(observation['object_id'])
            if profile is None:
                raise ValueError('当前物品尚未配置抓取位置；首轮仅支持怡宝水瓶')
            base_object = object_in_base(controller_pose(arm['pose']), tool['flange_T_grip_tcp'], handeye['flange_T_color_optical_camera'], observation['pose'])
            plan = plan_grasp(arm['pose'], base_object, profile, config)
            plan.update(object_id=observation['object_id'], generation=observation['generation'], captured_at=observation['time'],
                        current_arm=arm, gripper_base=base, gripper_state=gripper_state,
                        tool_response=tool['controller_readback']['tool_response_raw'], work_response=work,
                        force_limit_raw=grip['force_limit_raw'], force_units=grip['force_units'])
            blockers = workspace_blockers(plan, config)
            if not base.get('force') or not grip['force_control_verified']:
                blockers.append('夹爪力控单位及限力实际生效尚未验证')
            if not grip['contact_state_verified']:
                blockers.append('夹爪力控接触状态尚未实机验证')
            for register in (1180, 1220):
                try:
                    self.robot.register(register)
                except Exception as error:
                    blockers.append(str(error))
            plan['ik_check'] = self.check_ik(plan, arm, tool, config)
            if not plan['ik_check']['ok']:
                blockers.append(plan['ik_check'].get('reason', '路径逆解检查失败'))
            plan['execution_blockers'] = blockers
            plan['can_execute'] = self.mode == 'execute' and not blockers
            report['plan'] = plan
            self.update(plan=plan)
            self.check_cancel()
            if not plan['can_execute']:
                self.update(state='blocked' if blockers else 'preview', message='已生成方案；未发送运动。' + ('；'.join(blockers) if blockers else '当前为预览模式。'))
                return
            self.execute(plan, observation, config, report)
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

    def execute(self, plan, observation, config, report):
        grip = config['gripper']
        if any(type(grip[key]) is not int for key in ['force_target_raw', 'force_limit_raw', 'speed_limit_raw', 'open_position']):
            raise ValueError('夹爪参数必须为整数原始值')
        # Recheck snapshot, identity and unchanged arm immediately before writes.
        now = self.tracker.grasp_observation()
        if now['generation'] != observation['generation'] or now['object_id'] != observation['object_id'] or time.time() - observation['time'] > config['max_plan_age_s']:
            raise ValueError('抓取方案已过期或物品已切换')
        if np.linalg.norm(np.asarray(now['pose'])[:3, 3] - np.asarray(observation['pose'])[:3, 3]) > .005:
            raise ValueError('目标在规划后发生移动')
        current = self.arm_state()
        if np.max(np.abs(np.asarray(current['joint']) - plan['current_arm']['joint'])) > 30:
            raise ValueError('机械臂在规划后发生移动')
        if not (0 < grip['force_target_raw'] <= grip['force_limit_raw'] <= plan['gripper_base']['force_up'][0]):
            raise ValueError('夹爪力参数越界')
        if not (0 < grip['speed_limit_raw'] <= plan['gripper_base']['speed_up'][0]):
            raise ValueError('夹爪速度参数越界')
        self.check_cancel()
        self.actuated = True
        # No arm movement unless both settings can be read back exactly.
        for register, value in [(1180, grip['speed_limit_raw']), (1220, grip['force_limit_raw'])]:
            self.robot.set_register(register, value)
            if self.robot.register(register) != value:
                raise ValueError('夹爪限速/限力回读不一致，拒绝机械臂运动')
        self.update(state='opening', message='张开夹爪')
        self.robot.open_gripper(grip['open_position'])
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
        def watchdog():
            self.check_cancel()
            if self.robot.query('get_current_tool_frame') != plan['tool_response'] or self.robot.query('get_current_work_frame') != plan['work_response']:
                raise ValueError('运动期间工具或工作坐标系发生变化')
            state = self.arm_state()
            point = controller_pose(state['pose'])[:3, 3]
            if np.any(point < np.asarray(config['workspace_min_m'])) or np.any(point > np.asarray(config['workspace_max_m'])):
                raise ValueError('机械臂离开约定工作区')
        for waypoint in plan['waypoints']:
            self.update(state='moving', message='执行路径：' + waypoint['name'])
            report['robot_motion_sent'] = True
            self.robot.move_linear(waypoint['controller_pose'], config['arm_speed_percent'], self.cancelled, watchdog)
            reached = self.arm_state()
            if np.linalg.norm((np.array(reached['pose'][:3]) - waypoint['controller_pose'][:3]) / 1e6) > .002:
                raise ValueError('机械臂实际位置与目标不符')
        self.update(state='closing', message='低力度闭合，等待力控接触反馈')
        self.robot.set_register(1470, grip['force_target_raw'])
        deadline = time.monotonic() + grip['close_timeout_s']
        stable = 0
        while time.monotonic() < deadline:
            self.check_cancel()
            state = self.robot.query('get_rm_plus_state_info')['state_info']
            result = contact_result(state, grip)
            if result in ('fault', 'empty_close'):
                raise ValueError('夹爪异常或空夹到位')
            stable = stable + 1 if result == 'force_contact' else 0
            if stable >= 3:
                self.update(state='contact', message='检测到力控接触，保持当前位置；未抬起，不等同于抓牢验收。', gripper_contact=state)
                return
            time.sleep(.1)
        raise TimeoutError('未确认稳定接触，停止本次抓取')
