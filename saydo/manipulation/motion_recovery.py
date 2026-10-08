"""Read-only recovery assessment; an error is never permission to replay a path."""

import copy
import json
import numpy as np
from scipy.spatial.transform import Rotation
from saydo.contracts import motion_contract
from saydo.geometry import grasp_geometry
from saydo.manipulation import reorientation_geometry
from saydo.manipulation import object_motion_geometry


class MotionRecoveryMixin:
    def assess_motion_recovery(self, held, config, report):
        result = {
            'can_replan': False,
            'motion_sent': bool(self.actuated),
            'release_attempted': bool(report.get('release_attempted')),
            'failed_stage': report.get('active_stage'),
            'failed_waypoint_index': report.get('active_waypoint_index'),
            'planned_waypoints': len(
                (report.get('plan') or {}).get('waypoints', [])
            ),
            'completed_waypoints': report.get('completed_waypoints', 0),
            'requires_new_plan': True,
            'automatic_retry_of_same_plan': False,
        }
        retained = report.get('hold_retention', {})
        last = (report.get('motion_events') or [{}])[-1]
        try:
            reply = json.loads(
                (last.get('completion_reply') or {}).get('raw', '')
            )
        except (ValueError, TypeError):
            reply = {}
        known_completion = (
            isinstance(reply, dict)
            and last.get('accepted') is True
            and reply.get('state') == 'current_trajectory_state'
            and type(reply.get('trajectory_state')) is bool
        )
        result['outcome_unknown'] = bool(
            self.actuated
            and (
                not known_completion
                or not report.get('stop_acknowledged')
                or not retained.get('stationary_observed')
            )
        )
        try:
            if self.cancelled.is_set():
                raise ValueError('任务已取消，不自动恢复')
            if (report.get('plan') or {}).get('kind') == 'release':
                raise ValueError('释放类任务失败不自动恢复')
            if report.get('release_attempted'):
                raise ValueError('已尝试松爪，禁止自动恢复运动')
            if (
                getattr(self, 'agent_recording_error', None)
                or report.get('plan_recording_error')
            ):
                raise ValueError('执行记录不确定')
            with self.lock:
                current = copy.deepcopy(self.held_object)
            if not current or current['hold_id'] != held['hold_id']:
                raise ValueError('夹持关系无效')
            if (
                current['object_T_tcp'] != held['object_T_tcp']
                or current['tool_response'] != held['tool_response']
            ):
                raise ValueError('夹持变换/工具改变')
            retained = report.get('hold_retention', {})
            if self.actuated:
                if not known_completion:
                    raise ValueError(
                        '原运动结果未知，停止反馈不能替代原指令完成/失败证据'
                    )
                if (
                    not report.get('stop_acknowledged')
                    or not retained.get('stationary_observed')
                    or not retained.get('retained')
                ):
                    raise ValueError('停止或夹持保留未确认')
                if any(retained.get('controller_errors') or [1]):
                    raise ValueError('控制器仍有错误')
                if any(
                    s.get('speed') != [0]
                    for s in retained.get('gripper_samples', [])
                ):
                    raise ValueError('夹爪尚未停稳')
            # Reuse strict frame, enabled-joint, robot-model identity and
            # contact checks.
            arm, tool = self.reorientation_hardware(current, config)
            grip = self.check_reorient_contact(current, config)
            if grip.get('speed') != [0]:
                raise ValueError('夹爪尚未停稳')
            original = (
                (report.get('plan') or {}).get('configuration_binding')
                or report.get('configuration_binding')
            )
            if original is None or self.motion_binding(config) != original:
                raise ValueError('配置已变化或缺少绑定证据')
            mesh = reorientation_geometry.model_geometry(
                self.tracker.models_root, current['object_id']
            )
            if (
                mesh['model_sha256'] != current['model_sha256']
                or mesh.get('pose_mode', '6d') != current.get('pose_mode', '6d')
            ):
                raise ValueError('模型已变化')
            if current.get('scene_id'):
                self.validate_scene(current['scene_id'], config)
            tcp = grasp_geometry.controller_pose(arm['pose'])
            object_motion_geometry.validate_tcp(
                tcp, current, mesh['vertices'], config
            )
            pose = (
                tcp
                @ np.linalg.inv(
                    grasp_geometry.transform(current['object_T_tcp'])
                )
            )
            plan = report.get('plan') or {}
            goal = plan.get('goal', {})
            constraint = {
                'base_T_object_target': plan.get('base_T_object_target'),
                'pose_mode': current.get('pose_mode', '6d'),
                'position_mode': goal.get(
                    'position_mode', report.get('position_mode', 'fixed')
                ),
                'position_required': bool(
                    goal and goal.get('position_mode') == 'fixed'
                ),
                'tcp_position_required': (
                    not bool(goal)
                    and report.get('position_mode', 'fixed') == 'fixed'
                ),
                'object_T_tcp': current['object_T_tcp'],
                'reference_point_object_m': goal.get(
                    'reference_point_object_m', [0, 0, 0]
                ),
            }
            if (
                not constraint['base_T_object_target']
                and report.get('requested_rotation') is not None
            ):
                target = pose.copy()
                target[:3, :3] = report['requested_rotation']
                target[:3, 3] = (
                    tcp[:3, 3]
                    - (
                        target[:3, :3]
                        @ np.asarray(current['object_T_tcp'])[:3, 3]
                    )
                )
                constraint['base_T_object_target'] = target.tolist()
            target = grasp_geometry.transform(
                constraint['base_T_object_target']
            )
            attachment = grasp_geometry.transform(current['object_T_tcp'])
            constraint['tcp_position_m'] = (target @ attachment)[:3, 3].tolist()
            constraint['actual_tcp_position_m'] = tcp[:3, 3].tolist()
            suggested = {
                **copy.deepcopy(motion_contract.DEFAULT_GOAL),
                'position_mode': constraint['position_mode'],
                'reference_point_object_m': constraint[
                    'reference_point_object_m'
                ],
                'target_rpy_deg': Rotation.from_matrix(target[:3, :3]).as_euler(
                    'xyz', degrees=True
                ).tolist(),
                'target_position_m': (
                    (
                        target[:3, 3]
                        + (
                            target[:3, :3]
                            @ constraint['reference_point_object_m']
                        )
                    ).tolist()
                    if constraint['position_mode'] == 'fixed'
                    else None
                ),
                'path_mode': goal.get('path_mode', 'joint'),
                'max_rotation_step_deg': min(
                    10.,
                    goal.get(
                        'max_rotation_step_deg',
                        plan.get('max_rotation_step_deg', 15.),
                    ),
                ),
            }
            self.check_cancel()
            if getattr(self, 'agent_recording_error', None):
                raise ValueError('恢复核对期间任务记录失败')
            result.update(
                can_replan=True,
                goal_for_replan=suggested,
                goal_constraint=constraint,
                stationary_confirmed=True,
                grip_confirmed=True,
                controller_errors=arm.get('err'),
                hold_id=current['hold_id'],
                object_id=current['object_id'],
                orientation_version=current.get('orientation_version', 0),
                actual_arm=arm,
                base_T_object_actual=pose.tolist(),
                next_action='plan_from_current_feedback_with_new_hold_version',
            )
        except Exception as error:
            result['reason'] = str(error)
        report['recovery'] = result
        return result
