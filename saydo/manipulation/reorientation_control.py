"""Fixed-TCP object orientation shares task locks and preserves successful holds."""

import copy
from saydo.manipulation import motion_speed
import fcntl
import json
import pathlib
import threading
import time
import traceback
import uuid

import numpy as np
import yaml

from saydo.geometry import grasp_geometry
from saydo.objects import grasp_library
from saydo.robot import grasp_robot
from saydo.robot import motion_validation
from saydo.manipulation import reorientation_visual
from saydo.geometry import pose_symmetry
from saydo.geometry import rotation_steps
from saydo.manipulation import reorientation_geometry


class ReorientationMixin:
    """调姿：夹着物品把它转到指定朝向，完成后继续夹持。
    """

    def held_snapshot(self):
        if self.held_object is None:
            return None
        held = self.held_object
        result = {
            k: copy.deepcopy(held[k])
            for k in [
                'hold_id',
                'object_id',
                'object_name',
                'model_sha256',
                'base_T_object_at_grasp',
                'object_T_tcp',
                'current_object_rotation',
                'orientation_version',
                'pose_mode',
                'pose_mode_revision',
                'min_table_clearance_m',
                'scene_id',
                'instance_id',
            ]
            if k in held
        }
        if held.get('pose_mode') == '5d':
            result['spin_observable'] = False
            if held.get('current_object_rotation') is not None:
                result['current_axis_z'] = np.asarray(
                    held['current_object_rotation']
                )[:, 2].tolist()
        return result

    def capture_attachment(self, plan, report):
        if not plan.get('object_id') or not plan.get('model_sha256'):
            return
        reached = self.arm_state()
        base_tcp = grasp_geometry.controller_pose(reached['pose'])
        held = {
            'hold_id': uuid.uuid4().hex,
            'object_id': plan['object_id'],
            'object_name': plan.get('object_name'),
            'pose_mode': plan.get('pose_mode', '6d'),
            'pose_mode_revision': plan.get('pose_mode_revision'),
            'model_sha256': plan['model_sha256'],
            'base_T_object_at_grasp': plan['base_T_object'],
            'object_T_tcp': (
                np.linalg.inv(grasp_geometry.transform(plan['base_T_object']))
                @ base_tcp
            ).tolist(),
            'tool_response': plan['tool_response'],
            'contact_position': self.status['gripper_contact']['pos'][0],
            'recorded_at': time.time(),
            'evidence': (
                'frozen_visual_pose_and_gripper_feedback_not_slip_measurement'
            ),
        }
        if plan.get('scene_id'):
            held.update(
                scene_id=plan['scene_id'], instance_id=plan['instance_id']
            )
        parameters = plan.get('effective_parameters', {})
        if 'min_table_clearance_m' in parameters:
            held['min_table_clearance_m'] = parameters['min_table_clearance_m']
        report['held_object'] = held

    def finish_attachment(self, report):
        held = copy.deepcopy(report.get('held_object'))
        if held is not None:
            checks = report.get('arrival_checks', [])
            arm = checks[-1].get('actual_arm_state') if checks else None
            if arm is None:
                arm = self.arm_state()
            current = (
                grasp_geometry.controller_pose(arm['pose'])
                @ np.linalg.inv(grasp_geometry.transform(held['object_T_tcp']))
            )
            held.update(
                current_object_rotation=current[:3, :3].tolist(),
                orientation_version=0,
            )
        with self.lock:
            self.held_object = held

    def reorientation_model(self, object_id=None):
        with self.lock:
            held = copy.deepcopy(self.held_object)
        if not object_id:
            object_id = (
                held['object_id']
                if held
                else self.tracker.state.get('object_id')
            )
        geometry = reorientation_geometry.model_geometry(
            self.tracker.models_root, object_id
        )
        try:
            geometry['appearance'] = reorientation_visual.model_appearance(
                self.tracker.models_root, object_id
            )
        except (ValueError, OSError, TypeError) as error:
            geometry['appearance'] = {
                'status': 'unavailable', 'error': str(error)
            }
        geometry['initial_rotation'] = (
            copy.deepcopy(
                held.get(
                    'current_object_rotation',
                    np.asarray(held['base_T_object_at_grasp'])[:3, :3].tolist(),
                )
            )
            if held and held['object_id'] == object_id
            else np.eye(3).tolist()
        )
        geometry['holding'] = bool(held and held['object_id'] == object_id)
        geometry['pivot_object_m'] = (
            np.asarray(held['object_T_tcp'])[:3, 3].tolist()
            if geometry['holding']
            else None
        )
        geometry['preview_mode'] = 'adaptive_or_fixed_orientation'
        geometry['minimum_final_clearance_m'] = .10
        geometry['motion_command'] = 'movej_verified_joint_solutions'
        return geometry

    def start_reorient(self, value, preview=False):
        rotation = reorientation_geometry.checked_rotation(
            value.get('rotation')
        )
        with self.lock:
            if self.status['busy'] or self.status.get('stopping'):
                raise ValueError('已有机械臂任务正在处理')
            held = copy.deepcopy(self.held_object)
            if held is None:
                raise ValueError(
                    '本次服务尚无成功抓取记录，请先抓取物品后再调整姿态'
                )
            if (
                value.get('hold_id') != held['hold_id']
                or value.get('object_id') != held['object_id']
                or value.get('model_sha256') != held['model_sha256']
            ):
                raise ValueError('预览模型或夹持记录已变化，请重新加载物品模型')
            if (
                type(value.get('orientation_version')) is not int
                or (
                    value['orientation_version']
                    != held.get('orientation_version', 0)
                )
            ):
                raise ValueError(
                    '夹持朝向版本已变化或缺失，请刷新状态后重新规划'
                )
            if 'max_shift_m' in value:
                raise ValueError('自适应位置已取消位移上限，请刷新页面后重试')
            position_mode = reorientation_geometry.checked_position_request(
                value.get('position_mode', 'fixed')
            )
            step = rotation_steps.checked_step(
                value.get('max_rotation_step_deg', 15.)
            )
            self.cancelled.clear()
            self.actuated = False
            self.robot = None
            self.status.update(
                state='reorient_planning',
                operation='reorient',
                busy=True,
                error=None,
                plan=None,
                reorientation_plan=None,
                recovery=None,
                diagnostics_ref=None,
                target_object_id=held['object_id'],
                message=(
                    '正在检查目标姿态与最终离桌高度…'
                    if position_mode == 'adaptive'
                    else '正在检查固定终点位置下的目标姿态…'
                ),
            )
            self.thread = threading.Thread(
                target=self.run_reorient,
                args=(held, rotation.tolist(), preview, position_mode, step),
                daemon=True,
            )
            self.thread.start()
        return {
            'accepted': True,
            'mode': (
                'preview' if preview or self.mode == 'preview' else 'execute'
            ),
        }

    def reorientation_hardware(self, held, config):
        if (
            self.robot.query('get_arm_software_info').get('Product_version')
            != 'RM75-BI'
        ):
            raise ValueError('机械臂型号不匹配')
        if (
            self.robot.query('get_joint_en_state').get('en_state') != [1] * 7
            or self.robot.query('get_joint_err_flag').get('err_flag') != [0] * 7
        ):
            raise ValueError('关节使能或错误状态异常')
        tool = yaml.safe_load((self.root / config['tool_file']).read_text())
        if (
            self.robot.query('get_current_tool_frame') != held['tool_response']
            or (
                tool['controller_readback']['tool_response_raw']
                != held['tool_response']
            )
        ):
            raise ValueError('工具/负载已变化，不能沿用抓取关系调整姿态')
        work = self.robot.query('get_current_work_frame')
        if work.get('frame_name') != 'Base' or work.get('pose') != [0] * 6:
            raise ValueError('姿态调整要求零偏移 Base 工作坐标系')
        mode = self.robot.query('get_rm_plus_mode').get('mode')
        if mode != grasp_robot.RM_PLUS_MODE:
            raise ValueError(grasp_robot.rm_plus_mode_error(mode))
        base = self.robot.query('get_rm_plus_base_info')['base_info']
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
        self.check_reorient_contact(held, config)
        first = self.arm_state()
        time.sleep(.15)
        last = self.arm_state()
        a, b = (
            np.asarray(first['joint'], dtype=float),
            np.asarray(last['joint'], dtype=float),
        )
        if (
            a.shape != (7,)
            or b.shape != (7,)
            or not np.isfinite([a, b]).all()
            or np.max(np.abs(a - b)) > 30
        ):
            raise ValueError('机械臂未停稳，不能规划姿态调整')
        return last, tool

    def check_reorient_contact(self, held, config):
        state = self.robot.query('get_rm_plus_state_info')['state_info']
        if (
            (
                grasp_geometry.contact_result(state, config['gripper'])
                not in ('force_contact', 'force_stop')
            )
            or abs(state['pos'][0] - held['contact_position']) > 20
        ):
            with self.lock:
                self.held_object = None
            raise ValueError('夹持反馈已变化或未保持夹持，停止姿态调整')
        return state

    def retain_reorientation_hold(self, held, config, report):
        """After a confirmed stop, refresh a still-valid hold using read-only feedback.

        Controller motion errors do not by themselves invalidate the gripper's
        attachment. They remain visible and the next execution still checks them.
        Never resurrect a hold already invalidated by contact feedback.
        """
        record = report['hold_retention'] = {
            'retained': False, 'arm_samples': [], 'gripper_samples': []
        }
        try:
            with self.lock:
                if (
                    self.held_object is None
                    or self.held_object['hold_id'] != held['hold_id']
                ):
                    raise ValueError('夹持记录已失效或变化，不能自动恢复')
            if report.get('stop_error'):
                raise ValueError('停止未确认，不能保留可重试的夹持记录')
            tool = yaml.safe_load((self.root / config['tool_file']).read_text())
            if (
                (
                    self.robot.query('get_current_tool_frame')
                    != held['tool_response']
                )
                or (
                    tool['controller_readback']['tool_response_raw']
                    != held['tool_response']
                )
            ):
                raise ValueError('工具/负载已变化')
            work = self.robot.query('get_current_work_frame')
            if work.get('frame_name') != 'Base' or work.get('pose') != [0] * 6:
                raise ValueError('当前不是零偏移 Base 坐标系')
            mode = self.robot.query('get_rm_plus_mode').get('mode')
            if mode != grasp_robot.RM_PLUS_MODE:
                raise ValueError(grasp_robot.rm_plus_mode_error(mode))
            base = self.robot.query('get_rm_plus_base_info')['base_info']
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
                raise ValueError('夹爪身份已变化')
            mesh = reorientation_geometry.model_geometry(
                self.tracker.models_root, held['object_id']
            )
            if mesh['model_sha256'] != held['model_sha256']:
                raise ValueError('物品模型已变化')
            previous = None
            stable = 0
            for _ in range(6):
                state = self.check_reorient_contact(held, config)
                record['gripper_samples'].append(copy.deepcopy(state))
                arm = self.robot.query('get_current_arm_state')['arm_state']
                pose = grasp_geometry.controller_pose(arm['pose'])
                joints = np.asarray(arm['joint'], dtype=float)
                if joints.shape != (7,) or not np.isfinite(joints).all():
                    raise ValueError('关节反馈无效')
                record['arm_samples'].append(copy.deepcopy(arm))
                steady = (
                    previous is not None
                    and (
                        np.max(np.abs(joints - previous[0])) <= 30
                        and (
                            np.linalg.norm(pose[:3, 3] - previous[1][:3, 3])
                            <= .0002
                        )
                        and (
                            grasp_geometry.rotation_error(
                                pose[:3, :3], previous[1][:3, :3]
                            )
                            <= np.deg2rad(.15)
                        )
                    )
                )
                stable = stable + 1 if steady else 1
                if stable >= 3:
                    break
                previous = (joints, pose)
                time.sleep(.1)
            else:
                raise ValueError('停止后的位姿反馈未稳定')
            current = (
                pose
                @ np.linalg.inv(grasp_geometry.transform(held['object_T_tcp']))
            )
            updated = copy.deepcopy(held)
            updated.update(
                current_object_rotation=current[:3, :3].tolist(),
                orientation_version=held.get('orientation_version', 0) + 1,
            )
            with self.lock:
                if (
                    self.held_object is None
                    or self.held_object['hold_id'] != held['hold_id']
                ):
                    raise ValueError('核对期间夹持记录已变化')
                self.held_object = updated
            record.update(
                retained=True,
                stationary_observed=True,
                current_arm=arm,
                controller_errors=arm.get('err'),
                orientation_version=updated['orientation_version'],
            )
            return True
        except Exception as error:
            record.update(reason=str(error), error_type=type(error).__name__)
            with self.lock:
                if (
                    self.held_object is not None
                    and self.held_object['hold_id'] == held['hold_id']
                ):
                    self.held_object = None
            return False

    def restore_reorientation_report(self, report_path: pathlib.Path) -> bool:
        """从一份失败的调姿记录恢复夹持记录；只读，不发送运动指令。

        仅在启动时由操作者显式指定、且操作者已确认物品仍被同一次夹持
        夹着时使用，每份记录只能用一次。程序不会自动翻找旧记录来恢复
        夹持：这个入口只用于补救旧版本服务丢失的记录。

        Args:
            report_path: 这台臂 `reorient_jobs/` 目录下的一份调姿记录。

        Returns:
            是否已恢复夹持记录。失败原因写入 `status` 和恢复记录，不抛出
            异常。
        """
        recovery = {
            'started_at': time.time(),
            'restored': False,
            'robot_motion_sent': False,
        }
        directory = self.state_dir / 'reorientation_recovery'
        try:
            if self.status['busy'] or self.held_object is not None:
                raise ValueError('已有任务或夹持记录，不能从历史记录恢复')
            path = report_path.resolve()
            jobs = (self.state_dir / 'reorient_jobs').resolve()
            if (
                path.parent != jobs
                or not path.stem.isdigit()
                or path.suffix != '.json'
            ):
                raise ValueError('仅支持本项目姿态调整任务日志')
            latest = max(jobs.glob('*.json'), key=lambda p: p.stat().st_mtime)
            if path != latest:
                raise ValueError('只能恢复最近一次姿态调整失败记录')
            previous = json.loads(path.read_text())
            held = previous['held_object']
            if (
                previous.get('operation') != 'reorient'
                or previous.get('mode') != 'execute'
                or not previous.get('error')
                or previous.get('stop_error')
            ):
                raise ValueError('该记录不是停止已确认的姿态调整失败任务')
            if (
                self.tracker.state.get('object_id') != held['object_id']
                or (
                    self.tracker.state.get('model_sha256')
                    != held['model_sha256']
                )
            ):
                raise ValueError('当前选中的物品与失败记录不一致')
            directory.mkdir(parents=True, exist_ok=True)
            # Consuming the claim before queries prevents a later restart from
            # silently reusing this old grasp, including after a failed
            # recovery.
            with (directory / (path.stem + '.claimed')).open('x') as claim:
                claim.write(str(time.time()) + '\n')
            recovery['source_report'] = str(path.relative_to(self.root))
            config = json.loads(self.config_path.read_text())
            with self.robot_lock_path.open('a') as robot_lock:
                fcntl.flock(robot_lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
                self.robot = grasp_robot.RobotClient(
                    config['robot_ip'], config['robot_port'], writes=False
                )
                self.held_object = copy.deepcopy(held)
                if not self.retain_reorientation_hold(held, config, recovery):
                    raise ValueError(recovery['hold_retention']['reason'])
            recovery['restored'] = True
            self.update(
                state='failed',
                operation='reorient',
                error=previous['error'],
                reorientation_plan=None,
                reorientation_retry_available=True,
                message=(
                    previous['error']
                    + (
                        '；已只读核对并恢复本次夹持记录，可重新选择姿态。'
                        '控制器错误仍由执行前检查处理。'
                    )
                ),
            )
        except Exception as error:
            recovery.update(error=str(error), error_type=type(error).__name__)
            self.update(
                state='failed',
                operation='reorient',
                error='夹持记录恢复失败：' + str(error),
                message='夹持记录恢复失败：' + str(error),
                reorientation_retry_available=False,
            )
        finally:
            self.robot = None
            directory.mkdir(parents=True, exist_ok=True)
            (directory / f'{time.time_ns()}.json').write_text(
                json.dumps(recovery, ensure_ascii=False, indent=2)
            )
        return recovery['restored']

    def run_reorient(
        self,
        held: dict,
        rotation: list,
        preview: bool = False,
        position_mode: str = 'fixed',
        max_rotation_step_deg: float = 15.,
    ) -> None:
        """规划并执行一次调姿；执行模式下会发送运动指令，夹爪保持闭合。

        在工作线程里运行，占住这台臂的设备锁。本函数不抛出异常：结果
        写入 `status` 和记录目录下的 `reorient_jobs/`；出错时如果已经
        发出运动，先发送停止。

        Args:
            held: 这台臂当前的夹持记录。
            rotation: 物品的目标朝向，相对这台臂基座的 3×3 旋转矩阵。
            preview: True 时只生成方案，不发送任何写入指令。
            position_mode: `fixed` 表示物品位置不变；其他取值允许为了
                可达而平移，见 `reorientation_geometry`。
            max_rotation_step_deg: 相邻路径点之间物品转动的上限，单位度。
        """
        mode = 'preview' if preview else self.mode
        report = {
            'started_at': time.time(),
            'operation': 'reorient',
            'mode': mode,
            'held_object': held,
            'requested_rotation': rotation,
            'position_mode': position_mode,
            'robot_motion_sent': False,
            'active_stage': '姿态调整前检查',
        }
        directory = self.state_dir / 'reorient_jobs'
        robot_lock = None
        try:
            directory.mkdir(parents=True, exist_ok=True)
            robot_lock = (directory.parent / 'robot.lock').open('a')
            fcntl.flock(robot_lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
            config = grasp_library.held_clearance_config(
                json.loads(self.config_path.read_text()), held
            )
            settings = reorientation_geometry.reorientation_settings(
                motion_speed.reorientation_speed_settings(
                    config,
                    json.loads(
                        (
                            self.root / self.arm.reorientation_file
                        ).read_text()
                    ),
                )
            )
            settings['max_rotation_step_deg'] = rotation_steps.checked_step(
                max_rotation_step_deg
            )
            report['settings'] = settings
            report['configuration_binding'] = self.motion_binding(config)
            if held.get('scene_id'):
                self.validate_scene(held['scene_id'], config)
            self.robot = grasp_robot.RobotClient(
                config['robot_ip'],
                config['robot_port'],
                writes=mode == 'execute',
            )
            self.robot.motion_log = report.setdefault('motion_events', [])
            self.check_cancel()
            arm, tool = self.reorientation_hardware(held, config)
            report['initial_arm'] = copy.deepcopy(arm)
            report['tool'] = copy.deepcopy(tool)
            mesh = reorientation_geometry.model_geometry(
                self.tracker.models_root, held['object_id']
            )
            if mesh['model_sha256'] != held['model_sha256']:
                raise ValueError('物品模型已变化，不能沿用抓取关系调整姿态')
            if mesh.get('pose_mode', '6d') != held.get('pose_mode', '6d'):
                raise ValueError('物品位姿模式已变化，不能沿用旧夹持记录')
            checker = (self.root / config['ik_checker']).resolve()
            if not checker.is_file():
                raise ValueError('缺少姿态调整航点可达性检查程序')
            report['active_stage'] = '检查姿态路径及最终离桌高度'
            plan = reorientation_geometry.plan_reorientation(
                held,
                mesh['vertices'],
                rotation,
                arm,
                config,
                settings,
                lambda waypoints, timeout: motion_validation.check_waypoints(
                    checker, tool, arm, waypoints, timeout,
                    environment=self.collision_environment(config),
                ),
                self.check_cancel,
                position_mode,
                report.setdefault('search_attempts', []),
            )
            plan.update(
                object_id=held['object_id'],
                object_name=held['object_name'],
                hold_id=held['hold_id'],
                orientation_version=held.get('orientation_version', 0),
                orientation_tolerance_deg=settings['orientation_tolerance_deg'],
                can_execute=mode == 'execute',
                flange_T_grip_tcp=tool['flange_T_grip_tcp'],
            )
            report['plan'] = plan
            self.update(reorientation_plan=plan)
            self.check_cancel()
            if mode != 'execute':
                self.update(
                    state='reorient_preview',
                    message=(
                        f"方案已生成：共{len(plan['waypoints'])}个路点，"
                        f"旋转段最大{settings['max_rotation_step_deg']:g}度；"
                        f"末端移位 {plan['offset_distance_m']*100:.1f} 厘米，"
                        f"未发送运动。"
                    ),
                )
                return
            if (
                np.max(
                    np.abs(np.asarray(self.arm_state()['joint']) - arm['joint'])
                )
                > 30
            ):
                raise ValueError('机械臂在姿态调整规划后发生移动')
            self.check_reorient_contact(held, config)
            self.execute_reorient(
                plan, held, config, settings, report, mesh['vertices']
            )
        except Exception as error:
            retained = False
            if self.actuated:
                try:
                    self.robot.stop()
                    report['stop_acknowledged'] = True
                except Exception as stop_error:
                    report['stop_error'] = str(stop_error)
                retained = self.retain_reorientation_hold(held, config, report)
            detail = f"{report['active_stage']}：{error}"
            report.update(
                error=detail,
                error_type=type(error).__name__,
                traceback=traceback.format_exc(),
            )
            message = detail
            if retained:
                message += '；夹持记录已保留，是否可自动重新规划见恢复检查。'
                if any(report['hold_retention'].get('controller_errors') or []):
                    message += '控制器仍有错误，执行前需先恢复设备状态。'
            recovery = (
                self.assess_motion_recovery(held, config, report)
                if 'config' in locals()
                else {'can_replan': False}
            )
            self.update(
                state='cancelled' if self.cancelled.is_set() else 'failed',
                message=message,
                error=detail,
                recovery=recovery,
                reorientation_plan=None,
                reorientation_retry_available=(
                    retained or self.held_object is not None
                ),
            )
        finally:
            report['result'] = {**self.status, 'busy': False}
            try:
                directory.mkdir(parents=True, exist_ok=True)
                filename = f'{time.time_ns()}.json'
                (directory / filename).write_text(
                    json.dumps(report, ensure_ascii=False, indent=2)
                )
                self.update(
                    diagnostics_ref={
                        'directory': 'reorient_jobs', 'file': filename
                    }
                )
            except Exception as error:
                self.update(
                    state='failed',
                    error='诊断记录失败：' + str(error),
                    recovery={'can_replan': False, 'outcome_unknown': True},
                )
            finally:
                if robot_lock is not None:
                    robot_lock.close()
                self.update(busy=False)

    def execute_reorient(self, plan, held, config, settings, report, vertices):
        start = np.asarray(
            plan.get('start_tcp_position_m', plan.get('fixed_tcp_position_m'))
        )
        # Keep tests and archived fixed-position plans usable without any
        # translation fallback.
        if start.shape != (3,):
            raise ValueError('姿态计划缺少有效的起始位置')
        tolerance = settings['position_tolerance_m']
        points = np.asarray(vertices, dtype=float)
        if (
            points.ndim != 2
            or points.shape[1] != 3
            or not len(points)
            or not np.isfinite(points).all()
        ):
            raise ValueError('缺少有效的物品模型，不能监测运动中桌面碰撞')
        attachment_inv = np.linalg.inv(
            grasp_geometry.transform(held['object_T_tcp'])
        )
        report['motion_validation_scope'] = (
            'segmented_verified_movej_endpoints_and_live_feedback_not_full_path'
        )
        reached = None
        deadline = time.monotonic() + 90
        for index, target in enumerate(plan['waypoints'], 1):
            report['active_waypoint_index'] = index
            end = np.asarray(target['position_m'])
            translating = target['name'] == 'translate'

            def read_motion_state():
                self.check_cancel()
                if time.monotonic() > deadline:
                    raise TimeoutError('调姿执行超过90秒预算')
                actual = self.arm_state()
                pose = grasp_geometry.controller_pose(actual['pose'])
                # A controller-planned joint move may curve in Cartesian space
                # and change orientation in transit.
                obj = pose @ attachment_inv
                bottom = float(
                    pose_symmetry.object_bounds(
                        obj, points, held.get('pose_mode', '6d')
                    )[0][2]
                )
                report['minimum_observed_object_z_m'] = min(
                    report.get('minimum_observed_object_z_m', bottom), bottom
                )
                if bottom < config['table_z_m'] - 1e-6:
                    raise ValueError(
                        '运动反馈中的物品模型低于配置桌面，停止运动'
                    )
                self.check_reorient_contact(held, config)
                return actual

            def watchdog():
                return grasp_geometry.controller_pose(
                    read_motion_state()['pose']
                )

            report['active_stage'] = (
                '关节运动至选定位置'
                if translating
                else '关节运动至指定物品姿态'
            )
            self.update(
                state='reorienting', message=report['active_stage'] + '…'
            )
            # Reject a stale plan before its first command, not just a point
            # anywhere along its path.
            reached = watchdog()
            if np.linalg.norm(reached[:3, 3] - start) > tolerance:
                raise ValueError('末端已离开本段规划起点')
            detail = self.verify_motion_targets(
                plan, config, report, self.arm_state(), [target]
            )
            solutions = (
                detail.get('joint_solutions_deg')
                if isinstance(detail, dict)
                else None
            )
            if (
                not solutions
                or len(solutions[-1]) != 7
                or not np.isfinite(solutions[-1]).all()
            ):
                raise ValueError('缺少已验证的关节解')
            joint_target = np.rint(
                np.asarray(solutions[-1]) * 1000
            ).astype(int).tolist()
            self.actuated = True
            report['robot_motion_sent'] = True
            self.robot.motion_stage = report['active_stage']
            report.setdefault('commanded_joint_targets', []).append(
                {'stage': target['name'], 'joint_raw': joint_target}
            )
            self.robot.move_joints(
                joint_target,
                settings['arm_speed_percent'],
                self.cancelled,
                watchdog,
            )

            def arrival_state():
                return read_motion_state()

            check = {'stage': report['active_stage']}
            report.setdefault('arrival_checks', []).append(check)
            actual = motion_validation.wait_for_pose(
                arrival_state,
                target['controller_pose'],
                self.check_cancel,
                check,
                tolerance,
                np.deg2rad(settings['orientation_tolerance_deg']),
            )
            if np.max(np.abs(np.asarray(actual['joint']) - joint_target)) > 500:
                raise ValueError('未到达所选关节解')
            reached = grasp_geometry.controller_pose(actual['pose'])
            report['completed_waypoints'] = (
                report.get('completed_waypoints', 0) + 1
            )
            start = end
        if (
            plan.get('position_mode') == 'adaptive'
            and (
                reached[2, 3]
                <= plan['table_z_m'] + plan['minimum_final_clearance_m']
            )
        ):
            raise ValueError('实际末端最终高度未高于桌面 10 厘米，停止本次任务')
        actual_object = (
            reached
            @ np.linalg.inv(grasp_geometry.transform(held['object_T_tcp']))
        )
        updated = copy.deepcopy(held)
        updated['current_object_rotation'] = actual_object[:3, :3].tolist()
        updated['orientation_version'] = held.get('orientation_version', 0) + 1
        with self.lock:
            self.held_object = updated
        shifted = plan.get('offset_distance_m', 0.)
        self.update(
            state='completed',
            message=(
                f'已将物品摆至指定姿态，末端移位 {shifted*100:.1f} 厘米，'
                f'夹爪继续夹持。'
            ),
        )
