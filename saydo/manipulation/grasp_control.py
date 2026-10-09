"""First-trial grasp coordinator. Button-triggered execution with robot write gates."""

import pathlib
import copy
import fcntl
import hashlib
import json
import os
import re
import tempfile
import threading
import time
import traceback
from typing import Optional

import numpy as np
import yaml

from saydo.geometry import grasp_geometry
from saydo.robot import arms
from saydo.robot import collision_model
from saydo.robot import grasp_robot
from saydo.robot import motion_validation
from saydo.perception import camera_checks
from saydo.perception import vision_refinement
from saydo.manipulation import reorientation_control
from saydo.manipulation import placement_control
from saydo.manipulation import grasp_profiles_control
from saydo.geometry import pose_symmetry
from saydo.manipulation import gripper_diagnostics
from saydo.manipulation import motion_speed
from saydo.manipulation import object_motion_control
from saydo.scene import scene_control
from saydo.manipulation import fixed_pick_place
from saydo.manipulation import motion_recovery
from saydo.objects import object_catalog
from saydo.objects import grasp_library


# 关节与观测姿态相差不超过这个角度，就认为在观测姿态。取 2 度：复位
# 到位的误差在 0.05 度以内，而 2 度的关节偏差对应相机移动约 1～2 厘米，
# 此时相机校正的误差仍可忽略。
OBSERVATION_POSE_TOLERANCE_DEG = 2.


class GraspController(
    fixed_pick_place.FixedPickPlaceMixin,
    motion_recovery.MotionRecoveryMixin,
    scene_control.SceneMixin,
    object_motion_control.ObjectMotionMixin,
    motion_speed.MotionSpeedMixin,
    reorientation_control.ReorientationMixin,
    placement_control.PlacementMixin,
    grasp_profiles_control.GraspProfilesMixin,
):
    """一台机械臂的动作控制器：抓取、放置、调姿、持物移动和复位的总入口。

    各类动作的实现分在各个父类里，本类保存它们共用的状态：当前任务状态
    `status`、持物记录 `held_object`、取消标志和与控制器的连接。同一台臂
    同一时刻只处理一个任务（`status['busy']`）；每台臂创建一个实例，
    实例之间互不共享状态，所以两台臂可以同时各做各的。
    """
    def __init__(
        self,
        root: pathlib.Path,
        tracker: object,
        config_path: pathlib.Path,
        mode: str = 'execute',
        arm: Optional[arms.Arm] = None,
    ) -> None:
        """创建一台臂的动作控制器；不连接控制器，不发送任何指令。

        每台臂一个控制器，各有自己的任务状态、持物记录和设备锁，
        所以两台臂可以同时各执行一个任务。

        Args:
            root: 项目根目录。
            tracker: 这台臂的视觉来源（提供当前画面、物品位姿和相机
                身份）；只做不需要视觉的操作时可以是 None。
            config_path: 这台臂的抓取配置文件。
            mode: `execute` 允许发送运动和夹爪指令；`preview` 只生成
                方案。
            arm: 这个控制器管的是哪台臂；缺省为 1 臂。
        """
        self.root, self.tracker, self.config_path, self.mode = (
            pathlib.Path(root), tracker, pathlib.Path(config_path), mode
        )
        self.arm = arm if arm is not None else arms.default()
        # 任务诊断、场景快照、方案库都写在这台臂自己的目录里。
        self.state_dir = self.arm.state_path(self.root)
        self.robot_lock_path = self.arm.lock_path(self.root)
        self.lock = threading.RLock()
        self.cancelled = threading.Event()
        self.thread = None
        self.actuated = False
        self.robot = None
        self.held_object = None
        self.force_override = None
        self.force_acknowledged = False
        self.force_error = None
        self.status = {
            'state': 'idle',
            'busy': False,
            'arm': self.arm.arm_id,
            'mode': mode,
            'motion_enabled': mode == 'execute',
            'message': (
                '抓取后按配置竖直抬升。'
                if mode == 'execute'
                else '方案预览模式，不发送运动或夹爪写入命令。'
            ),
        }

    def collision_environment(
        self, config: dict
    ) -> collision_model.Environment:
        """返回这台臂规划时要避开的障碍物：桌面和本机登记的立杆。

        Args:
            config: 这台臂的抓取配置，桌面高度取自 `table_z_m`。

        Returns:
            障碍物设置，坐标在这台臂的基座坐标系里。

        Raises:
            ValueError: 本机的障碍物文件格式不对或数值无效。
        """
        return collision_model.load_environment(
            self.root, self.arm.arm_id, config['table_z_m']
        )

    def camera_reuse_path(self, config: dict) -> pathlib.Path:
        """返回这台臂的“同型号相机复用手眼标定”策略文件的路径。

        Args:
            config: 这台臂的抓取配置。`camera_reuse_file` 缺省时用
                1 臂最初的位置。

        Returns:
            策略文件的路径；文件不一定存在，不存在表示不允许复用。
        """
        return self.root / config.get(
            'camera_reuse_file', 'config/calibration/camera_reuse.yaml'
        )

    def snapshot(self):
        with self.lock:
            config = json.loads(self.config_path.read_text())
            try:
                reset_available = (
                    grasp_geometry.return_joint_target(config) is not None
                )
            except (ValueError, TypeError, KeyError, AttributeError):
                reset_available = False
            return {
                **self.status,
                'active_scene_id': self.scenes().active,
                'force_config': self.force_config(),
                'motion_speed_config': self.motion_speed_configuration(),
                'grasp_configuration': self.grasp_configuration(config),
                'open_position_config': {
                    'value': config['gripper']['open_position'],
                    'minimum': 0,
                    'maximum': 1000,
                },
                'reset_available': reset_available,
                'zero_reset_available': True,
                'held_object': self.held_snapshot(),
                'placement_config': self.placement_configuration(),
                'pregrasp_distance_m': config.get('pregrasp_distance_m'),
            }

    def set_open_position(self, value: int) -> dict:
        """把抓取前的夹爪张开位置保存到这台臂的抓取配置；不发送夹爪指令。

        Args:
            value: 张开位置，夹爪原始值 0–1000（1000 为完全张开，不是毫米）。

        Returns:
            保存结果；`motion_sent` 恒为 False，新值下一次抓取时生效。

        Raises:
            ValueError: 数值无效、这台臂有任务在进行，或配置被其他程序
                同时修改。
            BlockingIOError: 这台臂的设备锁被其他程序占用。
        """
        if type(value) is not int or not 0 <= value <= 1000:
            raise ValueError('抓取时夹爪打开位置必须为 0–1000 的整数原始值')
        with self.lock:
            if self.status['busy'] or self.status.get('stopping'):
                raise ValueError('机械臂任务进行中，不能更改抓取张开位置')
            directory = self.state_dir
            directory.mkdir(parents=True, exist_ok=True)
            report = {
                'started_at': time.time(),
                'requested_open_position': value,
                'saved': False,
                'motion_sent': False,
            }
            temporary = None
            try:
                with (directory / 'robot.lock').open('a') as lock:
                    fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
                    original = self.config_path.read_text()
                    config = json.loads(original)
                    report['previous_open_position'] = config['gripper'][
                        'open_position'
                    ]
                    config['gripper']['open_position'] = value
                    updated, count = re.subn(
                        r'("open_position"\s*:\s*)\d+',
                        lambda m: m[1] + str(value),
                        original,
                    )
                    if count != 1 or json.loads(updated) != config:
                        raise ValueError(
                            '配置中的 open_position 格式不明确，未保存'
                        )
                    with tempfile.NamedTemporaryFile(
                        mode='w',
                        dir=self.config_path.parent,
                        prefix='.grasp-',
                        delete=False,
                    ) as output:
                        temporary = pathlib.Path(output.name)
                        os.fchmod(
                            output.fileno(),
                            self.config_path.stat().st_mode & 0o777,
                        )
                        output.write(updated)
                        output.flush()
                        os.fsync(output.fileno())
                    if self.config_path.read_text() != original:
                        raise ValueError('配置同时被其他程序修改，请重新保存')
                    temporary.replace(self.config_path)
                    report['saved'] = True
                    self.status.update(plan=None)
            except Exception as error:
                report.update(error=str(error), error_type=type(error).__name__)
                raise
            finally:
                if temporary is not None:
                    temporary.unlink(missing_ok=True)
                records = directory / 'open_position_commands'
                records.mkdir(exist_ok=True)
                (records / f'{time.time_ns()}.json').write_text(
                    json.dumps(report, ensure_ascii=False, indent=2)
                )
            return {
                'value': value,
                'saved': True,
                'motion_sent': False,
                'message': '已保存到抓取配置，下一次抓取生效；本次不移动夹爪。',
            }

    def start_reset(self, target_kind='observation'):
        if target_kind not in ('observation', 'zero'):
            raise ValueError('未知的机械臂复位目标')
        label = (
            '机械臂复位(零位)'
            if target_kind == 'zero'
            else '机械臂复位(观测桌面)'
        )
        with self.lock:
            if self.status['busy'] or self.status.get('stopping'):
                raise ValueError('已有机械臂任务正在处理')
            if self.mode != 'execute':
                raise ValueError('预览模式不发送机械臂复位指令')
            config = json.loads(self.config_path.read_text())
            target = (
                [0] * 7
                if target_kind == 'zero'
                else grasp_geometry.return_joint_target(config)
            )
            if target is None:
                raise ValueError('尚未配置机械臂复位目标，无法复位')
            config['arm_speed_percent'] = motion_speed.checked_speed(
                config['arm_speed_percent']
            )
            self.cancelled.clear()
            self.actuated = False
            self.robot = None
            self.status.update(
                state='resetting',
                operation='reset',
                busy=True,
                error=None,
                plan=None,
                target_object_id=None,
                target_captured_at=None,
                reset_target_kind=target_kind,
                message=f'正在检查机械臂，准备执行{label}…',
            )
            self.thread = threading.Thread(
                target=self.run_reset,
                args=(config, target, target_kind),
                daemon=True,
            )
            self.thread.start()
            return {
                'accepted': True,
                'reset_target_kind': target_kind,
                'message': f'已开始{label}，夹爪开合保持不变。',
            }

    def return_to_joints(self, target, speed, report):
        self.check_cancel()
        self.robot.motion_stage = report['active_stage']
        self.robot.move_joints(target, speed, self.cancelled, self.check_cancel)
        self.check_cancel()
        reached = self.arm_state()
        report.setdefault('arrival_checks', []).append(
            {
                'stage': report['active_stage'],
                'target_joint': target,
                'actual_arm_state': reached,
            }
        )
        actual = np.asarray(reached['joint'], dtype=float)
        if (
            actual.shape != (7,)
            or not np.isfinite(actual).all()
            or np.max(np.abs(actual - target)) > 500
        ):
            raise ValueError('返回关节角度误差超过 0.5 度或反馈无效')

    def run_reset(
        self, config: dict, target: list, target_kind: str = 'observation'
    ) -> None:
        """把机械臂移动到复位姿态；会发送运动指令，夹爪开合保持不变。

        在工作线程里运行。先占住这台臂的设备锁，核对型号、关节使能、工具
        参数和夹爪状态，确认机械臂已停稳，再用关节运动到目标。本函数不
        抛出异常：结果写入 `status` 和记录目录下的 `reset_jobs/`；出错时
        如果已经发出运动，先发送停止。

        Args:
            config: 这台臂的抓取配置。
            target: 七个关节的目标角度，控制器原始单位（0.001 度）。
            target_kind: `observation`（已记录的观测桌面姿态）或 `zero`
                （七关节零位），只影响提示文字和记录。
        """
        label = (
            '机械臂复位(零位)'
            if target_kind == 'zero'
            else '机械臂复位(观测桌面)'
        )
        destination = (
            '七关节零位' if target_kind == 'zero' else '已记录的观测桌面姿态'
        )
        directory = self.state_dir
        report = {
            'started_at': time.time(),
            'operation': 'reset',
            'reset_target_kind': target_kind,
            'target_joint_raw': target,
            'speed_percent': config['arm_speed_percent'],
            'robot_motion_sent': False,
            'active_stage': '复位前检查',
        }
        robot_lock = None
        try:
            directory.mkdir(parents=True, exist_ok=True)
            robot_lock = (directory / 'robot.lock').open('a')
            fcntl.flock(robot_lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
            self.robot = grasp_robot.RobotClient(
                config['robot_ip'], config['robot_port'], writes=True
            )
            self.robot.motion_log = report.setdefault('motion_events', [])
            self.check_cancel()
            if (
                self.robot.query('get_arm_software_info').get('Product_version')
                != 'RM75-BI'
            ):
                raise ValueError('机械臂型号不匹配')
            if (
                (
                    self.robot.query('get_joint_en_state').get('en_state')
                    != [1] * 7
                )
                or (
                    self.robot.query('get_joint_err_flag').get('err_flag')
                    != [0] * 7
                )
            ):
                raise ValueError('关节使能或错误状态异常')
            tool = yaml.safe_load((self.root / config['tool_file']).read_text())
            if (
                self.robot.query('get_current_tool_frame')
                != tool['controller_readback']['tool_response_raw']
            ):
                raise ValueError('当前工具/负载与记录不一致')
            mode = self.robot.query('get_rm_plus_mode').get('mode')
            report['rm_plus_mode'] = mode
            if mode != grasp_robot.RM_PLUS_MODE:
                raise ValueError(
                    f'夹爪 RM Plus 通信未就绪（当前模式 {mode}，要求 115200）；'
                    '机械臂关节已通过使能检查，请恢复夹爪通信后重试'
                )
            grip_state = self.robot.query(
                'get_rm_plus_state_info'
            )['state_info']
            report['initial_gripper_state'] = copy.deepcopy(grip_state)
            if (
                (
                    grasp_geometry.contact_result(grip_state, config['gripper'])
                    == 'fault'
                )
                or grip_state.get('speed') != [0]
            ):
                raise ValueError('夹爪异常或正在运动，不能复位')
            first = np.asarray(self.arm_state()['joint'], dtype=float)
            time.sleep(.15)
            last = np.asarray(self.arm_state()['joint'], dtype=float)
            if (
                first.shape != (7,)
                or last.shape != (7,)
                or not np.isfinite([first, last]).all()
                or np.max(np.abs(last - first)) > 30
            ):
                raise ValueError('机械臂未停稳或关节反馈无效，不能复位')
            # 复位是从当前姿态直接转到目标关节角，发令前先查这条路径。
            path_check = collision_model.check_path(
                [last / 1000, np.asarray(target, dtype=float) / 1000],
                tool['flange_T_grip_tcp'],
                self.collision_environment(config),
            )
            report['collision_check'] = path_check
            if path_check['violation'] is not None:
                raise ValueError(
                    '复位路径不安全：' + path_check['violation']['message']
                    + '；请先手动把机械臂移到空旷处再复位'
                )
            self.check_cancel()
            self.actuated = True
            report.update(active_stage=label, robot_motion_sent=True)
            self.update(message=f'正在返回{destination}，夹爪开合保持不变…')
            self.return_to_joints(target, config['arm_speed_percent'], report)
            self.update(
                state='completed',
                message=f'机械臂已到达{destination}，夹爪开合保持不变。',
            )
        except Exception as error:
            if self.actuated:
                with self.lock:
                    self.held_object = None
                try:
                    self.robot.stop()
                except Exception as stop_error:
                    report['stop_error'] = str(stop_error)
            detail = f"{report['active_stage']}：{error}"
            report.update(
                error=detail,
                error_type=type(error).__name__,
                traceback=traceback.format_exc(),
            )
            self.update(
                state='cancelled' if self.cancelled.is_set() else 'failed',
                message=detail,
                error=detail,
            )
        finally:
            report['result'] = {**self.status, 'busy': False}
            records = directory / 'reset_jobs'
            try:
                records.mkdir(parents=True, exist_ok=True)
                (records / f'{time.time_ns()}.json').write_text(
                    json.dumps(report, ensure_ascii=False, indent=2)
                )
            finally:
                if robot_lock is not None:
                    robot_lock.close()
                self.update(busy=False)

    def force_config(self):
        grip = json.loads(self.config_path.read_text())['gripper']
        value = (
            self.force_override
            if self.force_override is not None
            else grip['force_target_raw']
        )
        message = (
            '力度指令已下发并收到成功回复；下一次抓取也使用此值。'
            if self.force_acknowledged
            else '尚未从本次页面下发；下一次抓取将使用此值。'
        )
        if self.mode == 'preview':
            message = '预览模式：仅更新下一次方案的配置，不下发硬件。'
        if self.force_error:
            message = (
                '下发未确认：'
                + self.force_error
                + '；设备实际值未知，未自动重试。'
            )
        return {
            'value': value,
            'minimum': 1,
            'maximum': 100,
            'hardware_applied': self.force_acknowledged,
            'write_acknowledged': self.force_acknowledged,
            'readback_verified': False,
            'error': self.force_error,
            'message': message,
        }

    def set_force_config(self, value: int) -> dict:
        """设置夹爪力度；执行模式下会向夹爪写入力度寄存器，不产生运动。

        预览模式只把数值记在内存里供下一次方案使用，不连接硬件。

        Args:
            value: 力度原始值 1–100，不是牛顿。

        Returns:
            当前力度配置，其中 `write_acknowledged` 表示本次是否已下发并
            收到应答。

        Raises:
            ValueError: 数值无效、这台臂有任务在进行，或下发未得到确认。
        """
        with self.lock:
            if self.status['busy'] or self.status.get('stopping'):
                raise ValueError('抓取任务进行中，不能更改力度')
            if type(value) is not int or not 1 <= value <= 100:
                raise ValueError(
                    '力度必须为 1–100 的整数原始值，不能按牛顿输入'
                )
            if self.mode == 'preview':
                self.force_override = value
                self.status.update(plan=None)
                return self.force_config()
            config = json.loads(self.config_path.read_text())
            grip = config['gripper']
            directory = self.state_dir
            directory.mkdir(parents=True, exist_ok=True)
            report = {
                'started_at': time.time(),
                'requested_force_raw': value,
                'write_attempted': False,
                'write_acknowledged': False,
            }
            self.status.update(plan=None)
            try:
                # Share the grasp lock so another service cannot write during a
                # grasp.
                with (directory / 'robot.lock').open('a') as robot_lock:
                    fcntl.flock(robot_lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
                    robot = grasp_robot.RobotClient(
                        config['robot_ip'], config['robot_port'], writes=True
                    )
                    mode = robot.query('get_rm_plus_mode').get('mode')
                    if mode != grasp_robot.RM_PLUS_MODE:
                        raise ValueError(grasp_robot.rm_plus_mode_error(mode))
                    base = robot.query('get_rm_plus_base_info')['base_info']
                    if (
                        (
                            base.get('manu'),
                            base.get('id'),
                            base.get('dof'),
                            base.get('type'),
                        )
                        != (grip['vendor'], grip['device_id'], 1, 1)
                    ):
                        raise ValueError('夹爪身份不匹配')
                    state = robot.query('get_rm_plus_state_info')['state_info']
                    if (
                        grasp_geometry.contact_result(state, grip) == 'fault'
                        or state.get('speed') != [0]
                    ):
                        raise ValueError('夹爪异常或正在运动，不能更改力度')
                    if (
                        not base.get('force')
                        or not 1 <= value <= base['force_up'][0]
                    ):
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
                (records / f'{time.time_ns()}.json').write_text(
                    json.dumps(report, ensure_ascii=False, indent=2)
                )
            return self.force_config()

    def open_gripper(self) -> dict:
        """把夹爪完全张开；会发送夹爪指令，并清除这台臂的持物记录。

        Returns:
            指令的应答情况。应答只表示夹爪接受了指令，不表示已张开到位。

        Raises:
            ValueError: 预览模式、这台臂有任务在进行、夹爪身份或状态
                异常，或指令未得到确认。
        """
        with self.lock:
            if self.status['busy'] or self.status.get('stopping'):
                raise ValueError('抓取任务进行中，不能打开夹爪')
            if self.mode != 'execute':
                raise ValueError('预览模式不发送夹爪运动指令')
            config = json.loads(self.config_path.read_text())
            grip = config['gripper']
            directory = self.state_dir
            directory.mkdir(parents=True, exist_ok=True)
            report = {
                'started_at': time.time(),
                'target_position': 1000,
                'write_attempted': False,
                'write_acknowledged': False,
            }
            try:
                with (directory / 'robot.lock').open('a') as robot_lock:
                    fcntl.flock(robot_lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
                    robot = grasp_robot.RobotClient(
                        config['robot_ip'], config['robot_port'], writes=True
                    )
                    mode = robot.query('get_rm_plus_mode').get('mode')
                    if mode != grasp_robot.RM_PLUS_MODE:
                        raise ValueError(grasp_robot.rm_plus_mode_error(mode))
                    base = robot.query('get_rm_plus_base_info')['base_info']
                    if (
                        (
                            base.get('manu'),
                            base.get('id'),
                            base.get('dof'),
                            base.get('type'),
                        )
                        != (grip['vendor'], grip['device_id'], 1, 1)
                    ):
                        raise ValueError('夹爪身份不匹配')
                    state = robot.query('get_rm_plus_state_info')['state_info']
                    report['initial_gripper_state'] = state
                    if (
                        grasp_geometry.contact_result(state, grip) == 'fault'
                        or state.get('speed') != [0]
                    ):
                        raise ValueError('夹爪异常或正在运动')
                    self.update_scene_instance(
                        self.held_object, 'release_unknown'
                    )
                    report['write_attempted'] = True
                    self.held_object = None
                    robot.move_gripper(1000)
                    report['write_acknowledged'] = True
                    self.status.update(plan=None)
            except Exception as error:
                report['error'] = str(error)
                raise ValueError('打开夹爪未确认：' + str(error)) from error
            finally:
                records = directory / 'open_commands'
                records.mkdir(exist_ok=True)
                (records / f'{time.time_ns()}.json').write_text(
                    json.dumps(report, ensure_ascii=False, indent=2)
                )
            return {
                'target_position': 1000,
                'write_acknowledged': True,
                'message': (
                    '打开指令已接受，目标位置 1000；'
                    '实际到位可通过“读取夹爪状态”确认。'
                ),
            }

    def update(self, **values):
        with self.lock:
            self.status.update(values)
            observer = getattr(self, 'agent_observer', None)
            if observer is not None:
                observer(self.status)

    def hardware_status(self):
        config = json.loads(self.config_path.read_text())
        robot = grasp_robot.RobotClient(
            config['robot_ip'], config['robot_port'], writes=False
        )
        try:
            software = robot.query('get_arm_software_info')
            mode = robot.query('get_rm_plus_mode')['mode']
            base = robot.query('get_rm_plus_base_info')['base_info']
            state = robot.query('get_rm_plus_state_info')['state_info']
            names = {
                0: '正在松开',
                1: '正在闭合',
                2: '位置到位停止',
                3: '力控到位停止',
                4: '触觉到位停止',
                5: '电流保护停止',
                6: '发生故障',
            }
            return {
                'mode': mode,
                'base': base,
                'state': state,
                'state_text': names.get(
                    state.get('dof_state', [-1])[0], '未知'
                ),
                'force_units': config['gripper']['force_units'],
                'motion_sent': False,
                'controller_version': software.get(
                    'ctrl_info', {}
                ).get('version'),
                'force_control_verified': config['gripper'][
                    'force_control_verified'
                ],
                'register_readback_available': False,
            }
        except Exception as error:
            raise ValueError('读取夹爪失败：' + str(error)) from error

    def profile_for(
        self, config: dict, observation: dict, tool: dict, handeye: dict
    ) -> dict:
        """取出抓取要用的夹法：配置里的固定夹法，或物品当前选用的那一套。

        Args:
            config: 这台臂的抓取配置。
            observation: 视觉观测，含物品编号和模型文件的 SHA-256。
            tool: 这台臂的工具记录。
            handeye: `calibration` 返回的手眼记录。

        Returns:
            夹法（夹持点、朝向、接近方向）。

        Raises:
            ValueError: 夹法记录时的模型、工具或手眼标定与现在不一致。
        """
        profile = copy.deepcopy(config['grasp_profile'])
        if profile['orientation'] != 'follow_object':
            return profile
        value = self.grasp_library().object(observation['object_id'])
        records = value['grasps']
        record = next(
            (r for r in records if r['id'] == value['selected_id']),
            records[0] if records else None,
        )
        if record is not None:
            error = grasp_library.binding_error(
                record,
                observation.get('model_sha256'),
                tool['controller_readback']['tool_response_raw'],
                grasp_library.binding_handeye(handeye),
            )
            if error:
                raise ValueError(error)
            profile.update(record['profile'])
        return profile

    def teach(
        self,
        use_current_point: bool = False,
        gripper_parameters: Optional[dict] = None,
        expected_object_id: Optional[str] = None,
    ) -> dict:
        """把夹爪当前相对物品的姿态记录成一套新夹法；只读，不发送运动指令。

        操作者先手动把夹爪摆到想要的夹持姿态，本函数读取机械臂位姿和
        视觉给出的物品位姿，算出两者的相对关系并存入夹法库。

        Args:
            use_current_point: True 时连夹持位置一起记录；False 时只记录
                朝向，位置沿用抓取配置里的值。
            gripper_parameters: 与这套夹法绑定的夹爪力度和张开位置；
                缺省用该物品的默认值。
            expected_object_id: 调用方认为正在跟踪的物品，用来防止页面
                显示的物品与实际跟踪的物品不一致。

        Returns:
            新夹法的编号、内容和绑定的夹爪参数。

        Raises:
            ValueError: 没有可用的物品位姿、这台臂有任务在进行、工具或
                工作坐标系与记录不一致，或夹法未通过几何检查。
        """
        if type(use_current_point) is not bool:
            raise ValueError('记录位置选项必须为布尔值')
        observation = copy.deepcopy(self.tracker.grasp_observation())
        if (
            not observation.get('object_id')
            or not observation.get('model_sha256')
        ):
            raise ValueError('记录夹法需要已登记的物品模型')
        if (
            expected_object_id is not None
            and observation['object_id'] != expected_object_id
        ):
            raise ValueError('当前物品已变化，请重新确认记录夹法的物品和参数')
        if gripper_parameters is None:
            gripper_parameters = grasp_library.object_gripper_defaults(
                observation['object_id']
            )
        if (
            not isinstance(gripper_parameters, dict)
            or set(gripper_parameters) != {'force_target_raw', 'open_position'}
        ):
            raise ValueError('记录夹法必须提供力度和抓取张开位置')
        gripper_parameters = grasp_library.checked_bound_overrides(
            gripper_parameters
        )
        with self.lock:
            if self.status['busy'] or self.status.get('stopping'):
                raise ValueError('已有抓取任务正在处理')
            self.status.update(
                busy=True,
                state='teaching',
                message='正在读取当前夹法，不发送运动…',
                error=None,
            )
            self.actuated = False
            self.cancelled.clear()
        directory = self.state_dir
        report = {
            'at': time.time(),
            'observation': observation,
            'use_current_point': use_current_point,
            'gripper_parameters': gripper_parameters,
            'motion_sent': False,
        }
        try:
            directory.mkdir(parents=True, exist_ok=True)
            with (directory / 'robot.lock').open('a') as lock:
                fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
                config = json.loads(self.config_path.read_text())
                self.robot = grasp_robot.RobotClient(
                    config['robot_ip'], config['robot_port'], writes=False
                )
                arm, observation = self.paired_observation(observation)
                handeye, tool = self.calibration(config)
                if (
                    self.robot.query('get_current_tool_frame')
                    != tool['controller_readback']['tool_response_raw']
                ):
                    raise ValueError('当前工具参数与记录不一致')
                work = self.robot.query('get_current_work_frame')
                if (
                    work.get('frame_name') != 'Base'
                    or work.get('pose') != [0] * 6
                ):
                    raise ValueError('当前工作坐标系必须是零偏移 Base')
                camera_checks.check_camera_observation(handeye, observation)
                report['camera_calibration'] = handeye.get(
                    'runtime_camera_binding'
                )
                base_tcp = grasp_geometry.controller_pose(arm['pose'])
                base_object = grasp_geometry.object_in_base(
                    base_tcp,
                    tool['flange_T_grip_tcp'],
                    handeye['flange_T_color_optical_camera'],
                    observation['pose'],
                )
                relative = np.linalg.inv(base_object) @ base_tcp
                profile = copy.deepcopy(config['grasp_profile'])
                profile.update(
                    orientation='follow_object',
                    object_R_tcp=relative[:3, :3].tolist(),
                )
                # User-confirmed tool forward axis.
                profile['approach_axis_tcp'] = [0, 0, 1]
                if use_current_point:
                    profile['grasp_point_object_m'] = relative[:3, 3].tolist()
                # Validate rotation, point, approach and distances before
                # saving.
                grasp_geometry.plan_grasp(
                    arm['pose'], base_object, profile, config
                )
                self.check_cancel()
                record = {
                    'profile': profile,
                    'model_sha256': observation['model_sha256'],
                    'overrides': gripper_parameters,
                    'tool_response': tool[
                        'controller_readback'
                    ]['tool_response_raw'],
                    'flange_T_camera': grasp_library.binding_handeye(
                        handeye
                    ),
                    'recorded_at': time.time(),
                }
                if observation.get('pose_mode') == '5d':
                    record['profile_5d'] = pose_symmetry.effective_profile(
                        record, '5d'
                    )
                entry = self.grasp_library().append_taught(
                    observation['object_id'], record
                )
                report['grasp_id'] = entry['id']
                report['record'] = record
                message = (
                    '已新增并选用当前物品的夹法，保存朝向'
                    + (
                        '及夹持位置'
                        if use_current_point
                        else '；夹持位置采用记录时的文件配置'
                    )
                )
                message += (
                    f"；夹爪力度 {gripper_parameters['force_target_raw']}，"
                    f"抓取张开位置 {gripper_parameters['open_position']}。"
                )
                if observation.get('pose_mode') == '5d':
                    message += (
                        '；5D夹法仅使用Z轴高度与接近夹角，中心X/Y已投影到0，'
                        '绕轴方位由规划选择。'
                    )
                self.update(
                    state='taught',
                    message=message,
                    plan=None,
                    grasp_library_changed_at=time.time(),
                )
                return {
                    'message': message,
                    'profile': pose_symmetry.effective_profile(
                        record, observation.get('pose_mode', '6d')
                    ),
                    'grasp_id': entry['id'],
                    'gripper_parameters': gripper_parameters,
                    'motion_sent': False,
                }
        except Exception as error:
            report['error'] = str(error)
            self.update(
                state='failed',
                message='记录夹法失败：' + str(error),
                error=str(error),
            )
            raise ValueError('记录夹法失败：' + str(error)) from error
        finally:
            self.update(busy=False)
            records = (
                self.grasp_library().object_directory(observation['object_id'])
                / 'teach_commands'
            )
            records.mkdir(parents=True, exist_ok=True)
            (records / f'{time.time_ns()}.json').write_text(
                json.dumps(report, ensure_ascii=False, indent=2)
            )

    def start(
        self,
        preview=False,
        expected_object_id=None,
        scene_id=None,
        instance_id=None,
    ):
        # Take the tracker lock before the grasp lock (snapshot uses the same
        # order).
        if scene_id is not None:
            observation = self.scene_observation(
                scene_id, instance_id, expected_object_id
            )
        else:
            if self.scenes().active is not None:
                raise ValueError(
                    '已有场景采集上下文，必须引用准备完成的scene_id和instance_i'
                    'd'
                )
            observation = copy.deepcopy(self.tracker.grasp_observation())
        if (
            expected_object_id is not None
            and observation.get('object_id') != expected_object_id
        ):
            raise ValueError('当前跟踪物品已变化，拒绝抓取其他物品')
        with self.lock:
            if self.status['busy'] or self.status.get('stopping'):
                raise ValueError('已有抓取任务正在处理')
            self.cancelled.clear()
            self.actuated = False
            self.status.update(
                state='checking',
                operation='grasp',
                busy=True,
                message='已锁定当前物品，检查机械臂、夹爪与抓取路径…',
                error=None,
                plan=None,
                target_object_id=observation['object_id'],
                target_captured_at=observation['time'],
            )
            self.thread = threading.Thread(
                target=self.run,
                args=(observation,),
                kwargs={'preview': preview},
                daemon=True,
            )
            self.thread.start()

    def abort(self):
        self.cancelled.set()
        with self.lock:
            active, robot = self.status['busy'] and self.actuated, self.robot
        if active and robot is not None:
            try:
                robot.stop()
            except Exception as error:
                self.update(
                    error=str(error),
                    message='停止指令未全部确认，请使用现场急停',
                )
                raise

    def check_cancel(self):
        if self.cancelled.is_set():
            raise ValueError('机械臂任务已取消')
        guard = getattr(self, '_fixed_guard', None)
        if guard is not None:
            guard()

    def calibration(self, config: dict) -> tuple:
        """读取这台臂的手眼标定和工具记录，并确认它们可以使用。

        确认三件事：标定和工具原点已被用户接受；原始记录文件没有被改动
        （比对 SHA-256）；当前相机就是标定时的那台，或配置允许同型号
        相机复用。配置了相机校正时，把校正合进手眼矩阵：之后所有
        “相机坐标换算到基座坐标”的计算用的都是校正后的矩阵。

        Args:
            config: 这台臂的抓取配置。

        Returns:
            `(handeye, tool)` 两份记录。`handeye` 里附带
            `runtime_camera_binding`（相机与标定的对应关系）和
            `runtime_camera`（当前相机身份）。

        Raises:
            ValueError: 标定未被接受、原始记录被改动，或相机与标定不匹配。
        """
        handeye = yaml.safe_load(
            (self.root / config['handeye_file']).read_text()
        )
        tool = yaml.safe_load((self.root / config['tool_file']).read_text())
        if (
            handeye['acceptance_status'] != 'user_accepted_usable'
            or not config['nominal_tcp_user_accepted']
        ):
            raise ValueError('手眼标定或当前名义 TCP 未被接受')
        for record, source in [
            (handeye, handeye['source']), (tool, tool['controller_readback'])
        ]:
            if (
                hashlib.sha256(
                    (self.root / source['path']).read_bytes()
                ).hexdigest()
                != source['sha256']
            ):
                raise ValueError('标定原始记录哈希变化')
        policy_path = self.camera_reuse_path(config)
        policy = (
            yaml.safe_load(policy_path.read_text())
            if policy_path.exists()
            else {}
        )
        camera = copy.deepcopy(self.tracker.state.get('camera'))
        handeye['runtime_camera_binding'] = camera_checks.camera_binding(
            handeye, camera, policy or {}
        )
        handeye['runtime_camera'] = camera
        refinement = self.vision_refinement(config)
        # 夹法绑定比对的是标定原值；几何换算用校正后的矩阵。
        handeye['calibrated_flange_T_color_optical_camera'] = handeye[
            'flange_T_color_optical_camera'
        ]
        handeye['flange_T_color_optical_camera'] = (
            vision_refinement.refined_flange_T_camera(
                handeye['flange_T_color_optical_camera'], refinement
            ).tolist()
        )
        handeye['vision_refinement'] = (
            None
            if refinement is None
            else {
                'file': config['vision_refinement_file'],
                'measured_at': refinement.get('measured_at'),
                'correction': refinement['correction'],
            }
        )
        return handeye, tool

    def vision_refinement(self, config: dict) -> Optional[dict]:
        """读取这台臂的相机校正。

        Args:
            config: 这台臂的抓取配置；`vision_refinement_file` 是校正
                文件相对项目根目录的路径。

        Returns:
            校正记录；配置里没有写校正文件时为 None。

        Raises:
            ValueError: 校正文件是别的臂的、未被接受或数值无效。
        """
        name = config.get('vision_refinement_file')
        if not name:
            return None
        return vision_refinement.load(self.root / name, self.arm.arm_id)

    def observation_pose_note(self, config: dict, arm: dict) -> dict:
        """判断机械臂当前是否在观测姿态，供场景快照标注精度。

        相机校正是在观测姿态下测的，只在那里准到 1～2 毫米；在别的
        姿态采集的快照，物品位置约有 3～6 毫米的误差。

        Args:
            config: 这台臂的抓取配置，观测姿态取自其中的复位目标。
            arm: 控制器报告的机械臂状态，关节为原始单位（0.001 度）。

        Returns:
            含 `at_observation_pose`（是否在观测姿态；没有配置观测
            姿态时为 None）和 `max_joint_difference_deg`。
        """
        try:
            target = grasp_geometry.return_joint_target(config)
        except (ValueError, TypeError, KeyError, AttributeError):
            target = None
        if target is None:
            return {
                'at_observation_pose': None,
                'max_joint_difference_deg': None,
            }
        difference_deg = float(
            np.abs(
                np.asarray(arm['joint'], dtype=float) - np.asarray(target)
            ).max()
            / 1000
        )
        return {
            'at_observation_pose': (
                difference_deg <= OBSERVATION_POSE_TOLERANCE_DEG
            ),
            'max_joint_difference_deg': difference_deg,
            'tolerance_deg': OBSERVATION_POSE_TOLERANCE_DEG,
        }

    def arm_state(self):
        state = self.robot.query('get_current_arm_state')['arm_state']
        if any(state.get('err', [1])):
            raise ValueError('机械臂存在错误状态')
        return state

    def paired_observation(self, observation):
        # Bind the click-time image to the stationary arm immediately at job
        # start.
        # Do not replace the frozen image with a later tracker result.
        if time.time() - observation['time'] > .6:
            raise ValueError('开始时的视觉快照已过期，请重新点击抓取')
        first = self.arm_state()
        middle = self.arm_state()
        time.sleep(.15)
        last = self.arm_state()
        joints = (
            np.array([x['joint'] for x in [first, middle, last]], dtype=float)
            / 1000
        )
        poses = np.array(
            [x['pose'] for x in [first, middle, last]], dtype=float
        )
        if (
            np.ptp(joints, axis=0).max() > .03
            or np.ptp(poses[:, :3], axis=0).max() > 200
            or np.ptp(poses[:, 3:], axis=0).max() > 3
        ):
            raise ValueError('机械臂未停稳，不能绑定手眼位姿')
        if time.time() - observation['time'] > .6:
            raise ValueError('抓取视觉快照已过期')
        return last, observation

    @staticmethod
    def check_gripper_start(state, grip):
        position = state.get('pos')
        if (
            not isinstance(position, list)
            or len(position) != 1
            or type(position[0]) not in (int, float)
            or not np.isfinite(position[0])
            or not 0 <= position[0] <= 1000
        ):
            raise ValueError('夹爪位置反馈无效，无法确认复位状态')
        if (
            grasp_geometry.contact_result(state, grip) == 'fault'
            or state.get('speed') != [0]
        ):
            raise ValueError('夹爪异常或正在运动')
        # The device can retain force-stop status after manual opening
        # (pos=1000).
        # A full manual/UI opening remains a reset even when the grasp opening
        # is smaller.
        opened = any(
            abs(position[0] - target) <= 3
            for target in (grip['open_position'], 1000)
        )
        if (
            not opened
            and (state.get('dof_state') == [3] or any(state.get('force', [])))
        ):
            raise ValueError(
                f"夹爪可能已夹持物体，请先人工张开复位（当前位置 "
                f"{position[0]}，抓取张开目标 {grip['open_position']}，"
                f"完全张开目标 1000，状态 {state.get('dof_state')}，力度反馈 "
                f"{state.get('force')}）"
            )

    def prepare_grasp_plan(self, observation, config, arm, report):
        handeye, tool = self.calibration(config)
        software = self.robot.query('get_arm_software_info')
        report['controller_software'] = software
        if (
            software.get('Product_version') != 'RM75-BI'
            or (
                software['ctrl_info']['version']
                != handeye['installation']['controller_firmware']
            )
        ):
            raise ValueError('机械臂型号/固件与标定记录不匹配')
        if (
            self.robot.query('get_current_tool_frame')
            != tool['controller_readback']['tool_response_raw']
        ):
            raise ValueError('当前工具/TCP/负载与 grip_tcp 记录不一致')
        work = self.robot.query('get_current_work_frame')
        if work.get('frame_name') != 'Base' or work.get('pose') != [0] * 6:
            raise ValueError('当前工作坐标系必须是零偏移 Base')
        if (
            self.robot.query('get_joint_en_state').get('en_state') != [1] * 7
            or self.robot.query('get_joint_err_flag').get('err_flag') != [0] * 7
        ):
            raise ValueError('关节使能或错误状态异常')
        mode = self.robot.query('get_rm_plus_mode').get('mode')
        if mode != grasp_robot.RM_PLUS_MODE:
            raise ValueError(grasp_robot.rm_plus_mode_error(mode))
        base = self.robot.query('get_rm_plus_base_info')['base_info']
        gripper_state = self.robot.query('get_rm_plus_state_info')['state_info']
        report['initial_gripper_state'] = copy.deepcopy(gripper_state)
        grip = config['gripper']
        if (
            (
                base.get('manu'),
                base.get('id'),
                base.get('dof'),
                base.get('type'),
            )
            != (grip['vendor'], grip['device_id'], 1, 1)
        ):
            raise ValueError('夹爪身份不匹配')
        camera_checks.check_camera_observation(handeye, observation)
        report['camera_calibration'] = handeye.get('runtime_camera_binding')
        report['gripper_base'] = base
        base_object = (
            np.asarray(observation['base_T_object'])
            if observation.get('scene_id')
            else grasp_geometry.object_in_base(
                grasp_geometry.controller_pose(arm['pose']),
                tool['flange_T_grip_tcp'],
                handeye['flange_T_color_optical_camera'],
                observation['pose'],
            )
        )
        if observation.get('pose_mode_revision') is not None:
            current_model = object_catalog.select_object(
                self.tracker.models_root, observation['object_id']
            )
            if (
                current_model['pose_mode_revision']
                != observation['pose_mode_revision']
            ):
                raise ValueError('模型位姿设置已变化，请重新跟踪后抓取')
        plan, profile, config = self.select_grasp_candidate(
            config, observation, tool, handeye, base_object, arm, report
        )
        grip = config['gripper']
        report['effective_gripper_config'] = dict(grip)
        plan.update(
            object_id=observation['object_id'],
            generation=observation['generation'],
            captured_at=observation['time'],
            target_locked=True,
            pose_mode=observation.get('pose_mode', '6d'),
            pose_mode_revision=observation.get('pose_mode_revision'),
            model_sha256=observation.get('model_sha256'),
            camera_calibration=handeye.get('runtime_camera_binding'),
            object_name=observation.get('object_name'),
            current_arm=arm,
            gripper_base=base,
            gripper_state=gripper_state,
            tool_response=tool['controller_readback']['tool_response_raw'],
            work_response=work,
            flange_T_grip_tcp=tool['flange_T_grip_tcp'],
            force_limit_raw=grip['force_limit_raw'],
            force_units=grip['force_units'],
        )
        if observation.get('scene_id'):
            plan.update(
                scene_id=observation['scene_id'],
                instance_id=observation['instance_id'],
            )
        blockers = grasp_geometry.workspace_blockers(plan, config)
        if not base.get('force'):
            blockers.append('当前夹爪未声明支持力度控制')
        report['plan'] = plan
        plan['motion_validation'] = {
            'method': 'ik_fk_endpoints_then_controller_movej_p_movel_movej',
            'offline_path_checked': False,
            'ik_fk_checked': not blockers,
            'message': (
                '抓取航点先做逆解及正解回验，每段发令前按实际关节重查；'
                '控制器完成后等待稳定到位。未检查完整中间轨迹或环境碰撞。'
            ),
        }
        plan['execution_blockers'] = blockers
        report['grasp_profile'] = profile
        return plan, config

    def run(self, observation: dict, preview: bool = False) -> None:
        """规划并执行一次抓取和抬升；执行模式下会发送运动指令和夹爪指令。

        在工作线程里运行。执行模式下先占住这台臂的设备锁，核对机械臂、
        夹爪和标定，选出可行的夹法和路径，再依次到预抓取位、下探、闭合、
        抬升，最后按配置返回。本函数不抛出异常：结果写入 `status` 和
        记录目录下的 `jobs/`；出错时如果已经发出运动，先发送停止。

        Args:
            observation: 点击抓取时冻结的视觉观测，或场景快照里的一个实例。
            preview: True 时只生成方案，不发送任何写入指令。
        """
        mode = 'preview' if preview else self.mode
        report = {
            'started_at': time.time(),
            'mode': mode,
            'robot_motion_sent': False,
            'locked_observation': copy.deepcopy(observation),
        }
        robot_lock = None
        try:
            with self.lock:
                config = json.loads(self.config_path.read_text())
                config['arm_speed_percent'] = motion_speed.checked_speed(
                    config['arm_speed_percent']
                )
            report['effective_gripper_config'] = dict(config['gripper'])
            report['config_sha256'] = hashlib.sha256(
                self.config_path.read_bytes()
            ).hexdigest()
            if mode == 'execute':
                lock_path = self.robot_lock_path
                lock_path.parent.mkdir(parents=True, exist_ok=True)
                robot_lock = lock_path.open('a')
                fcntl.flock(robot_lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
            self.robot = grasp_robot.RobotClient(
                config['robot_ip'],
                config['robot_port'],
                writes=mode == 'execute',
            )
            self.robot.motion_log = report.setdefault('motion_events', [])
            if observation.get('scene_id'):
                self.validate_scene(observation['scene_id'], config)
                arm = self.stationary_scene_arm()
            else:
                arm, observation = self.paired_observation(observation)
            plan, config = self.prepare_grasp_plan(
                observation, config, arm, report
            )
            blockers = plan['execution_blockers']
            plan['can_execute'] = mode == 'execute' and not blockers
            report['plan'] = plan
            self.update(plan=plan)
            self.check_cancel()
            if not plan['can_execute']:
                self.update(
                    state='blocked' if blockers else 'preview',
                    message=(
                        '已生成方案；未发送运动。'
                        + (
                            (
                                '\n阻塞原因：\n'
                                + '\n'.join(
                                    f'{i+1}. {reason}'
                                    for i, reason in enumerate(blockers)
                                )
                            )
                            if blockers
                            else (
                                '当前为预览模式，航点基础检查已通过；'
                                '各段轨迹均将在实际运动时由控制器规划。'
                            )
                        )
                    ),
                )
                return
            library = self.grasp_library()
            if (
                grasp_library.revision(library.object(observation['object_id']))
                != plan['grasp_library_revision']
            ):
                raise ValueError('夹法库在规划后变化，请重新预览')
            if observation.get('scene_id'):
                self.validate_scene(observation['scene_id'], config)
                self.update_scene_instance(observation, 'moving')
            self.execute(plan, config, report)
            if observation.get('scene_id'):
                self.update_scene_instance(observation, 'held')
            if (
                self.status['state'] == 'completed'
                and plan['grasp_id'] != 'config'
            ):
                try:
                    library.mark_success(
                        observation['object_id'],
                        plan['grasp_id'],
                        plan['grasp_library_revision'],
                        plan.get('pose_mode', '6d'),
                    )
                    self.update(grasp_library_changed_at=time.time())
                except (OSError, ValueError) as error:
                    report['grasp_history_error'] = str(error)
        except Exception as error:
            if self.actuated:
                gripper_diagnostics.record_failure_before_stop(report, error)
                stop_record = report['failure_stop'] = {
                    'requested_at': time.time(),
                    'commands_acknowledged': False,
                    'stationarity_checked': False,
                }
                try:
                    self.robot.stop()
                    stop_record['commands_acknowledged'] = True
                except Exception as stop_error:
                    report['stop_error'] = str(stop_error)
                    stop_record['error'] = str(stop_error)
                finally:
                    stop_record['finished_at'] = time.time()
            stage = report.get('active_stage', '抓取准备')
            detail = f'{stage}：{error}'
            self.update(
                state='cancelled' if self.cancelled.is_set() else 'failed',
                error=detail,
                message=detail,
            )
            report.update(
                error=detail,
                error_type=type(error).__name__,
                traceback=traceback.format_exc(),
            )
        finally:
            if (
                observation.get('scene_id')
                and self.status['state'] in ('failed', 'cancelled')
            ):
                try:
                    self.update_scene_instance(
                        observation, 'unknown' if self.actuated else 'static'
                    )
                except Exception as e:
                    report['scene_record_error'] = str(e)
            self.update(busy=False)
            report['result'] = self.snapshot()
            if robot_lock is not None:
                robot_lock.close()
            directory = self.state_dir / 'jobs'
            directory.mkdir(parents=True, exist_ok=True)
            (directory / f'{time.time_ns()}.json').write_text(
                json.dumps(report, indent=2, ensure_ascii=False)
            )

    def verify_motion_targets(
        self,
        plan: dict,
        config: dict,
        report: dict,
        arm: dict,
        waypoints: list,
        timeout: float = 3.,
    ) -> dict:
        """发令前离线检查一串路点；只做计算，不发送指令。

        检查逆解、正解回验、关节连续性，以及机械臂和夹爪会不会碰到
        桌面或立杆。结果追加到 `report['kinematics_checks']`。

        Args:
            plan: 抓取方案，取其中的 `flange_T_grip_tcp`。
            config: 这台臂的抓取配置。
            report: 本次任务的诊断记录。
            arm: 机械臂当前状态，关节为原始单位（0.001 度）。
            waypoints: 要检查的路点序列。
            timeout: 逆解搜索的时间上限，单位秒。

        Returns:
            检查记录，含各路点的关节解 `joint_solutions_deg`。

        Raises:
            ValueError: 缺少离线检查器，或检查不通过（原因写在消息里）。
        """
        self.check_cancel()
        executable = (self.root / config['ik_checker']).resolve()
        if not executable.is_file():
            raise ValueError(
                '缺少支持正解回验的离线检查器，请重新构建 grasp_ik.cc'
            )
        valid, detail = motion_validation.check_waypoints(
            executable,
            {'flange_T_grip_tcp': plan['flange_T_grip_tcp']},
            arm,
            waypoints,
            timeout=timeout,
            environment=self.collision_environment(config),
        )
        report.setdefault('kinematics_checks', []).append(
            {
                'stage': report.get('active_stage'),
                'waypoints': [w['name'] for w in waypoints],
                'initial_arm': copy.deepcopy(arm),
                **detail,
            }
        )
        self.check_cancel()
        if not valid:
            raise ValueError(detail['reason'])
        return detail

    def verify_waypoint(self, plan, config, report, waypoint):
        """Offline check before one grasp segment; a fixed plan gets the joint solution of each of its samples."""
        fixed = plan.get('fixed_joint_targets', {}).get(waypoint['name'])
        if fixed is None:
            self.verify_motion_targets(
                plan, config, report, self.arm_state(), [waypoint]
            )
            return None
        # A 7-joint arm reaches one pose with many joint sets, and the
        # controller's movel was
        # measured swinging them by up to 35 degrees. A fixed plan therefore
        # never sends movel:
        # the whole remaining path is re-solved from real feedback before every
        # segment, and the
        # segment is then driven through the solved joint set of each of its
        # samples.
        points = getattr(self, '_fixed_path_points', None) or []
        own = [
            i
            for i, p in enumerate(points)
            if p['name'].startswith(waypoint['name'] + '_sample_')
        ]
        if not own:
            raise ValueError('固定路径缺少航点采样：' + waypoint['name'])
        detail = self.verify_motion_targets(
            plan, config, report, self.arm_state(), points[own[0]:]
        )
        solved = detail['joint_solutions_deg'][:own[-1] - own[0] + 1]
        motion_validation.note_joint_change(
            report,
            waypoint['name'],
            'resolved_remaining_path_vs_plan',
            fixed,
            np.asarray(solved[-1]) * 1000,
        )
        return solved

    def follow_joint_samples(
        self, solutions_deg, speed, watchdog, report, stage
    ):
        """Drive one segment as joint moves through its verified samples; returns the last joint target."""
        target = None
        for solution in solutions_deg:
            if len(solution) != 7 or not np.isfinite(solution).all():
                raise ValueError('缺少已验证的关节解，不能执行关节方案')
            target = np.rint(np.asarray(solution) * 1000).astype(int).tolist()
            self.robot.move_joints(target, speed, self.cancelled, watchdog)
            if (
                np.max(np.abs(np.asarray(self.arm_state()['joint']) - target))
                > 500
            ):
                raise ValueError('关节运动未到达所选关节解，停止固定流程')
        if target is None:
            raise ValueError('缺少已验证的关节解，不能执行关节方案')
        report.setdefault('sampled_joint_segments', []).append(
            {
                'stage': stage,
                'joint_moves': len(solutions_deg),
                'final_joint_raw': target,
            }
        )
        return target

    def execute(self, plan, config, report):
        grip = config['gripper']
        grasp_geometry.minimum_table_clearance(config)
        if any(
            type(grip[key]) is not int
            for key in [
                'force_target_raw',
                'force_limit_raw',
                'open_position',
                'closed_position',
            ]
        ):
            raise ValueError('夹爪参数必须为整数原始值')
        # The base-frame target is fixed for the job; tracking loss or target
        # motion
        # must not change it. Only robot state is rechecked before actuation.
        current = self.arm_state()
        if (
            np.max(
                np.abs(
                    np.asarray(current['joint']) - plan['current_arm']['joint']
                )
            )
            > 30
        ):
            raise ValueError('机械臂在规划后发生移动')
        if not (
            0
            < grip['force_target_raw']
            <= grip['force_limit_raw']
            <= plan['gripper_base']['force_up'][0]
        ):
            raise ValueError('夹爪力参数越界')
        self.check_cancel()
        self.actuated = True
        with self.lock:
            self.held_object = None
        # User verified register 1220 + position following on the installed
        # firmware.
        # Numeric register readback is unavailable; reject failed write
        # acknowledgements.
        report['active_stage'] = '设置张开前力度'
        self.robot.set_register(1220, grip['force_limit_raw'])
        report['active_stage'] = '张开夹爪'
        self.update(state='opening', message='张开夹爪')
        self.robot.move_gripper(grip['open_position'])
        deadline = time.monotonic() + 8
        while time.monotonic() < deadline:
            self.check_cancel()
            state = self.robot.query('get_rm_plus_state_info')['state_info']
            if grasp_geometry.contact_result(state, grip) == 'fault':
                raise ValueError('夹爪张开过程中异常')
            if (
                state['speed'] == [0]
                and abs(state['pos'][0] - grip['open_position']) <= 3
            ):
                break
            time.sleep(.1)
        else:
            raise TimeoutError('夹爪未到达张开位置')
        lifting = False
        watchdog = self.check_cancel

        def move_waypoint(waypoint):
            watchdog()
            names = {
                'pregrasp': '调整朝向并到达预抓取点',
                'grasp': '接近夹持点',
                'lift': '夹持后竖直抬升',
            }
            report['active_stage'] = names[waypoint['name']]
            self.update(
                state='lifting' if lifting else 'moving',
                message='执行路径：' + names[waypoint['name']],
            )
            solved = self.verify_waypoint(plan, config, report, waypoint)
            report['robot_motion_sent'] = True
            self.robot.motion_stage = report['active_stage']
            if solved is not None:
                self.follow_joint_samples(
                    solved,
                    config['arm_speed_percent'],
                    watchdog,
                    report,
                    report['active_stage'],
                )
            else:
                move = (
                    self.robot.move_joint_pose
                    if waypoint['name'] == 'pregrasp'
                    else self.robot.move_linear
                )
                move(
                    waypoint['controller_pose'],
                    config['arm_speed_percent'],
                    self.cancelled,
                    watchdog,
                )
            check = {'stage': report['active_stage']}
            report.setdefault('arrival_checks', []).append(check)
            motion_validation.wait_for_pose(
                self.arm_state,
                waypoint['controller_pose'],
                self.check_cancel,
                check,
            )

        for waypoint in plan['waypoints']:
            if waypoint['name'] != 'lift':
                move_waypoint(waypoint)
        report['active_stage'] = '设置力度并闭合夹爪'
        self.update(state='closing', message='按指定力度闭合，等待力控接触反馈')
        diagnostics = gripper_diagnostics.CloseDiagnostics(report, grip)
        diagnostics.command(
            'set_rm_plus_reg',
            {'addr': 1220, 'data': [grip['force_limit_raw']]},
            lambda: self.robot.set_register(1220, grip['force_limit_raw']),
        )
        diagnostics.command(
            'hand_follow_pos',
            {'hand_pos': [grip['closed_position']]},
            lambda: self.robot.move_gripper(grip['closed_position']),
        )
        deadline = time.monotonic() + grip['close_timeout_s']
        stable = 0
        while time.monotonic() < deadline:
            self.check_cancel()
            sample = diagnostics.sample(self.robot)
            state, result = sample['state'], sample['classification']
            sample['consecutive_contact_samples_before'] = stable
            if result in ('fault', 'empty_close'):
                sample['consecutive_contact_samples'] = 0
                diagnostics.decision(result, sample)
                raise ValueError(
                    gripper_diagnostics.contact_failure_message(
                        result, state, grip
                    )
                )
            stable = (
                stable + 1 if result in ('force_contact', 'force_stop') else 0
            )
            sample['consecutive_contact_samples'] = stable
            if stable >= 3:
                diagnostics.decision('contact_confirmed', sample)
                self.update(gripper_contact=state)
                self.capture_attachment(plan, report)
                lifting = True
                move_waypoint(
                    next(w for w in plan['waypoints'] if w['name'] == 'lift')
                )
                watchdog()
                self.finish_attachment(report)
                self.update(
                    state='completed',
                    message=(
                        f"已完成夹持并竖直抬升 {plan['lift_height_m'] * 100:g} "
                        f"厘米，停留在抬升位置并保持夹持。"
                    ),
                )
                return
            time.sleep(.1)
        diagnostics.decision('timeout', diagnostics.data['last_sample'])
        last_state = (diagnostics.data['last_sample'] or {}).get('state')
        raise TimeoutError(
            '闭合确认超时，未确认稳定接触，停止本次抓取'
            + (
                (
                    '；最后反馈：'
                    + gripper_diagnostics.feedback_summary(last_state)
                )
                if last_state is not None
                else '；尚无有效反馈'
            )
        )
