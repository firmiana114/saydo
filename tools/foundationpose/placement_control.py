"""Table placement shares the grasp task/device locks and never moves in preview."""
import copy
import fcntl
import hashlib
import json
import os
from pathlib import Path
import tempfile
import threading
import time
import traceback

import numpy as np

from grasp_geometry import controller_pose, rotation_error, contact_result
from grasp_robot import RobotClient
from motion_validation import check_waypoints, wait_for_pose
from reorientation_geometry import model_geometry, pose_waypoint
from placement_geometry import (placement_settings, plan_placement, object_vertices,
                                check_tcp_workspace, placement_validation_points, placement_motion_method)


def settings_revision(settings):
    return hashlib.sha256(json.dumps(settings, sort_keys=True).encode()).hexdigest()


class PlacementMixin:
    @property
    def placement_path(self):
        return self.root / 'config/grasp/placement.json'

    def placement_configuration(self):
        try:
            value = json.loads(self.placement_path.read_text()) if self.placement_path.exists() else {}
            settings = placement_settings(value, require_region=False)
            table = json.loads(self.config_path.read_text())['table_z_m']
            if type(table) not in (int, float) or not np.isfinite(table):
                raise ValueError('桌面高度无效')
            return {'settings': settings, 'revision': settings_revision(settings), 'table_z_m': table,
                    'configured': settings['region_xy_m'] is not None, 'error': None}
        except (ValueError, OSError, KeyError, TypeError) as error:
            return {'configured': False, 'error': str(error)}

    def save_placement_config(self, value):
        if not isinstance(value, dict) or set(value) != {'region_xy_m', 'release_gap_m', 'revision'}:
            raise ValueError('请提交桌面矩形范围、释放间隙及当前配置版本')
        with self.lock:
            if self.status['busy']:
                raise ValueError('机械臂任务进行中，不能更改放置设置')
            directory = self.root / 'work/grasp';directory.mkdir(parents=True, exist_ok=True)
            temporary = None
            with (directory / 'robot.lock').open('a') as lock:
                fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
                original = self.placement_path.read_text() if self.placement_path.exists() else None
                old = placement_settings(json.loads(original) if original is not None else {}, require_region=False)
                if value['revision'] != settings_revision(old):
                    raise ValueError('放置设置已变化，请重新载入后保存')
                settings = placement_settings({**old, **{k: value[k] for k in ('region_xy_m', 'release_gap_m')}})
                self.placement_path.parent.mkdir(parents=True, exist_ok=True)
                try:
                    with tempfile.NamedTemporaryFile(mode='w', dir=self.placement_path.parent,
                                                     prefix='.placement-', delete=False) as output:
                        temporary = Path(output.name)
                        os.fchmod(output.fileno(), self.placement_path.stat().st_mode & 0o777 if original is not None else 0o644)
                        output.write(json.dumps(settings, ensure_ascii=False, indent=2) + '\n')
                        output.flush();os.fsync(output.fileno())
                    latest = self.placement_path.read_text() if self.placement_path.exists() else None
                    if latest != original:
                        raise ValueError('放置配置同时被其他程序修改，未保存')
                    temporary.replace(self.placement_path)
                finally:
                    if temporary is not None:
                        temporary.unlink(missing_ok=True)
                self.status.update(placement_plan=None)
                records = directory / 'placement_settings';records.mkdir(exist_ok=True)
                (records / f'{time.time_ns()}.json').write_text(json.dumps({
                    'previous': old, 'saved': settings, 'motion_sent': False,
                }, ensure_ascii=False, indent=2))
            return {**self.placement_configuration(), 'saved': True, 'motion_sent': False}

    def start_placement(self, value, preview=False):
        if not isinstance(value, dict):
            raise ValueError('放置请求必须为 JSON 对象')
        with self.lock:
            if self.status['busy']:
                raise ValueError('已有机械臂任务正在处理')
            held = copy.deepcopy(self.held_object)
            if held is None:
                raise ValueError('尚无有效夹持记录，请先成功抓取物品')
            for key in ('hold_id', 'object_id', 'model_sha256', 'orientation_version'):
                if value.get(key) != held.get(key, 0 if key == 'orientation_version' else None):
                    raise ValueError('夹持记录或姿态已变化，请刷新后重新预览')
            current = self.placement_configuration()
            if current['error']:
                raise ValueError(current['error'])
            settings = placement_settings(current['settings'])
            if value.get('revision') != current['revision']:
                raise ValueError('放置设置已变化，请重新载入后预览')
            self.cancelled.clear();self.actuated = False;self.robot = None
            self.status.update(state='placement_planning', operation='place', busy=True, error=None,
                               plan=None, placement_plan=None, target_object_id=held['object_id'],
                               message='根据当前夹持朝向计算高度，并搜索桌面范围内的可达位置…')
            self.thread = threading.Thread(target=self.run_placement, args=(held, settings, preview), daemon=True)
            self.thread.start()
        return {'accepted': True, 'mode': 'preview' if preview or self.mode == 'preview' else 'execute'}

    def placement_config_unchanged(self, report):
        if hashlib.sha256(self.config_path.read_bytes()).hexdigest() != report['config_sha256']:
            raise ValueError('抓取/桌面配置在规划后变化，停止本次放置')
        current = self.placement_configuration()
        if current.get('revision') != report['settings_revision']:
            raise ValueError('放置设置在规划后变化，停止本次放置')

    def run_placement(self, held, settings, preview=False):
        mode = 'preview' if preview else self.mode
        report = {'started_at': time.time(), 'operation': 'place', 'mode': mode,
                  'held_object': held, 'settings': settings, 'settings_revision': settings_revision(settings),
                  'robot_motion_sent': False, 'release_attempted': False, 'release_confirmed': False,
                  'contact_confirmed': False, 'active_stage': '放置前检查'}
        directory = self.root / 'work/grasp/placement_jobs';robot_lock = None
        try:
            directory.mkdir(parents=True, exist_ok=True)
            robot_lock = (directory.parent / 'robot.lock').open('a')
            fcntl.flock(robot_lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
            raw = self.config_path.read_bytes();config = json.loads(raw)
            report['config_sha256'] = hashlib.sha256(raw).hexdigest()
            self.placement_config_unchanged(report)
            self.robot = RobotClient(config['robot_ip'], config['robot_port'], writes=mode == 'execute')
            self.robot.motion_log = report.setdefault('motion_events', [])
            self.check_cancel()
            arm, tool = self.reorientation_hardware(held, config)
            report.update(initial_arm=copy.deepcopy(arm), tool=tool)
            mesh = model_geometry(self.tracker.models_root, held['object_id'])
            if mesh['model_sha256'] != held['model_sha256']:
                raise ValueError('物品模型已变化，不能沿用夹持关系放置')
            checker = (self.root / config['ik_checker']).resolve()
            if not checker.is_file():
                raise ValueError('缺少支持正解回验的离线检查器')
            report['active_stage'] = '搜索放置位置并检查运动端点及下降/退出直线路径'
            plan = plan_placement(held, mesh['vertices'], arm, config, settings,
                lambda points, timeout: check_waypoints(checker, tool, arm, points, timeout),
                self.check_cancel, report.setdefault('search_attempts', []))
            plan.update(object_id=held['object_id'], object_name=held.get('object_name'), hold_id=held['hold_id'],
                        model_sha256=held['model_sha256'], flange_T_grip_tcp=tool['flange_T_grip_tcp'])
            report['plan'] = plan
            self.update(placement_plan=plan)
            self.check_cancel();self.placement_config_unchanged(report)
            if mode != 'execute':
                self.update(state='placement_preview', message='放置方案已生成：关节运动至上方，按当前朝向直线下降并留间隙释放。本次未发送运动。')
                return
            current = self.arm_state()
            if np.max(np.abs(np.asarray(current['joint']) - arm['joint'])) > 30:
                raise ValueError('机械臂在放置规划后发生移动，请重新规划')
            self.execute_placement(plan, held, config, settings, report, mesh['vertices'])
        except Exception as error:
            if self.actuated:
                with self.lock:
                    self.held_object = None
                try:
                    self.robot.stop()
                except Exception as stop_error:
                    report['stop_error'] = str(stop_error)
            detail = f"{report['active_stage']}：{error}"
            if report['release_attempted']:
                detail += '；已尝试松爪，不会自动重新夹取或继续退出'
            report.update(error=detail, error_type=type(error).__name__, traceback=traceback.format_exc())
            self.update(state='cancelled' if self.cancelled.is_set() else 'failed', message=detail, error=detail)
        finally:
            report['result'] = {**self.status, 'busy': False}
            try:
                directory.mkdir(parents=True, exist_ok=True)
                (directory / f'{time.time_ns()}.json').write_text(json.dumps(report, ensure_ascii=False, indent=2))
            finally:
                if robot_lock is not None:
                    robot_lock.close()
                self.update(busy=False)

    def execute_placement(self, plan, held, config, settings, report, vertices):
        previous = list(plan['current_arm']['pose'])
        names = {'raise': '关节运动至转移高度', 'transfer': '关节运动至放置点上方',
                 'lower': '保持朝向下降至释放高度', 'withdraw': '松爪后沿 grip_tcp 的 -Z 方向后撤',
                 'retreat': '后撤后竖直退出'}
        for target in plan['waypoints']:
            retreating = target['name'] in ('withdraw', 'retreat')
            if retreating:
                if not report.get('release_confirmed'):
                    raise ValueError('未确认夹爪张开，不能后撤或退出')
                # Anchor post-release motions to the last measured arrival pose.
                # Recheck these corrected targets against actual joints before sending.
                adjusted = controller_pose(previous)
                if target['name'] == 'withdraw':
                    adjusted[:3, 3] -= adjusted[:3, 2] * plan['withdraw_distance_m']
                else:
                    adjusted[2, 3] += settings['retreat_height_m']
                target = pose_waypoint(target['name'], adjusted)
                target.update(method='linear')
                target['controller_pose'][3:] = list(previous[3:])
            joint_motion = placement_motion_method(target['name']) == 'joint_pose'
            report['active_stage'] = names[target['name']]
            self.update(state='placing', message=report['active_stage'] + '…')
            self.check_cancel();self.placement_config_unchanged(report)
            start = controller_pose(previous); end = controller_pose(target['controller_pose'])
            check_tcp_workspace(end, config)

            def read_motion_state():
                self.check_cancel()
                actual = self.arm_state();pose = controller_pose(actual['pose'])
                check_tcp_workspace(pose, config)
                if not joint_motion:
                    if rotation_error(pose[:3, :3], start[:3, :3]) > np.deg2rad(settings['orientation_tolerance_deg']):
                        raise ValueError('放置过程中实际朝向偏离规划朝向')
                    delta = end[:3, 3] - start[:3, 3]
                    fraction = np.clip(np.dot(pose[:3, 3] - start[:3, 3], delta) / max(np.dot(delta, delta), 1e-12), 0, 1)
                    if np.linalg.norm(pose[:3, 3] - start[:3, 3] - fraction * delta) > .01:
                        raise ValueError('放置反馈偏离本段直线路径超过 10 毫米')
                if not retreating:
                    self.check_reorient_contact(held, config)
                    world = object_vertices(pose, held, vertices);bottom = float(world[:, 2].min())
                    report['minimum_observed_object_z_m'] = min(report.get('minimum_observed_object_z_m', bottom), bottom)
                    if bottom < config['table_z_m'] - 1e-6:
                        raise ValueError('物品模型低于配置桌面，停止放置')
                else:
                    self.check_placement_open(config, require_open=True)
                self.check_cancel()
                return actual

            arm = read_motion_state()
            if (np.linalg.norm(controller_pose(arm['pose'])[:3, 3] - start[:3, 3]) > settings['position_tolerance_m']):
                raise ValueError('末端已离开放置路径的规划起点')
            if rotation_error(controller_pose(arm['pose'])[:3, :3], start[:3, :3]) > np.deg2rad(settings['orientation_tolerance_deg']):
                raise ValueError('末端朝向已偏离本段规划起点')
            # Joint motion checks only its endpoint. Linear motion checks the
            # commanded constant-orientation line from the measured XYZ; small
            # angular settling error is bounded by feedback/arrival checks.
            sampling_start = list(arm['pose'][:3]) + list(target['controller_pose'][3:])
            points = placement_validation_points(sampling_start, [target], settings['linear_sample_step_m'])
            self.verify_motion_targets(plan, config, report, arm, points)
            self.check_cancel();self.placement_config_unchanged(report)
            self.actuated = True;report['robot_motion_sent'] = True
            self.robot.motion_stage = report['active_stage']
            speed = settings['descent_speed_percent'] if target['name'] in ('lower', 'withdraw', 'retreat') else settings['transfer_speed_percent']
            report.setdefault('executed_waypoints', []).append(copy.deepcopy(target))
            move = self.robot.move_joint_pose if joint_motion else self.robot.move_linear
            move(target['controller_pose'], speed, self.cancelled, read_motion_state)
            record = {'stage': report['active_stage']};report.setdefault('arrival_checks', []).append(record)
            actual = wait_for_pose(read_motion_state, target['controller_pose'], self.check_cancel, record,
                                   settings['position_tolerance_m'], np.deg2rad(settings['orientation_tolerance_deg']))
            previous = list(actual['pose'])
            if target['name'] == 'lower':
                world = object_vertices(controller_pose(actual['pose']), held, vertices)
                bottom = float(world[:, 2].min());report['release_bottom_z_m'] = bottom
                if not config['table_z_m'] - 1e-6 <= bottom <= config['table_z_m'] + settings['release_gap_m'] + settings['position_tolerance_m']:
                    raise ValueError('实际模型最低点未到达允许的释放高度，不能松爪')
                self.check_cancel();self.placement_config_unchanged(report)
                self.check_reorient_contact(held, config)
                report['active_stage'] = '张开夹爪释放物品'
                self.update(state='releasing', message='已到达释放高度，张开夹爪…')
                self.check_cancel()
                report['release_attempted'] = True
                with self.lock:
                    self.held_object = None
                self.robot.move_gripper(1000)
                deadline = time.monotonic() + 8;stable = 0
                while time.monotonic() < deadline:
                    self.check_cancel()
                    state = self.check_placement_open(config)
                    report['release_gripper_state'] = state
                    stable = stable + 1 if state['speed'] == [0] and abs(state['pos'][0] - 1000) <= 3 else 0
                    if stable >= 3:
                        report['release_confirmed'] = True
                        break
                    time.sleep(.1)
                else:
                    raise TimeoutError('夹爪未确认完全张开，停止后续退出')
                self.check_cancel()
        self.update(state='completed', message='已完成几何放置、张开夹爪、沿工具 -Z 后撤并竖直退出；未通过传感器确认接触或落桌后的稳定性。')

    def check_placement_open(self, config, require_open=False):
        state = self.robot.query('get_rm_plus_state_info')['state_info']
        position = state.get('pos')
        if (not isinstance(position, list) or len(position) != 1 or type(position[0]) not in (int, float)
                or not np.isfinite(position[0]) or not 0 <= position[0] <= 1000
                or contact_result(state, config['gripper']) == 'fault'):
            raise ValueError('释放物品时夹爪反馈无效或存在故障')
        if require_open and (state.get('speed') != [0] or abs(position[0] - 1000) > 3):
            raise ValueError('退出时夹爪未保持完全张开')
        return state
