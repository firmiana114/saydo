"""Stationary, all-targets-ready scene acquisition and frozen-frame motion references."""

import copy
import fcntl
import hashlib
import json
from pathlib import Path
import threading
import time
import numpy as np
from saydo.perception.camera_checks import check_camera_observation
from saydo.geometry.grasp_geometry import controller_pose, transform
from saydo.robot.grasp_robot import RobotClient
from saydo.objects.object_catalog import select_object
from saydo.scene.scene_snapshot import SceneStore, arm_still
from saydo.scene.scene_features import object_features
from saydo.contracts.scene_contract import checked_targets


class SceneMixin:
    def scenes(self):
        if not hasattr(self, '_scenes'):
            self._scenes = SceneStore(self.root)
        return self._scenes

    def scene_binding(self, config):
        paths = {
            'tool': self.root / config['tool_file'],
            'handeye': self.root / config['handeye_file'],
            'camera_reuse': self.root / 'config/calibration/camera_reuse.yaml',
        }
        return {
            'files': {
                k: (
                    hashlib.sha256(p.read_bytes()).hexdigest()
                    if p.exists()
                    else None
                )
                for k, p in paths.items()
            },
            'robot': [config['robot_ip'], config['robot_port']],
        }

    def validate_scene(self, ident, config=None):
        scene = self.scenes().require(ident)
        config = config or json.loads(self.config_path.read_text())
        if self.scene_binding(config) != scene['binding']:
            raise ValueError('场景绑定的设备或手眼/TCP标定已变化')
        for row in scene['instances']:
            info = select_object(self.tracker.models_root, row['object_id'])
            if (
                (
                    hashlib.sha256(Path(info['mesh']).read_bytes()).hexdigest()
                    != row['model_sha256']
                )
                or (
                    info.get('pose_mode_revision')
                    != row.get('pose_mode_revision')
                )
            ):
                raise ValueError('场景中的模型或位姿模式已变化')
            if (
                object_features(
                    self.tracker.models_root, row['object_id']
                )['feature_revision']
                != row['feature_revision']
            ):
                raise ValueError('场景模型特征已变化')
        return scene

    def scene_snapshot(self, ident):
        with self.lock:
            scene = self.scenes().get(ident)
            try:
                self.validate_scene(ident)
                scene['valid_for_execution'] = True
            except (ValueError, OSError) as e:
                scene.update(valid_for_execution=False, invalid_reason=str(e))
            for row in scene['instances']:
                row['feature_count'] = len(row.get('features', []))
                row.pop('features', None)
            return scene

    def start_scene_capture(self, targets, environment_static, request_id):
        targets = checked_targets(targets)
        if environment_static is not True:
            raise ValueError('当前场景快照要求执行期间环境不变')
        with self.tracker.lock, self.lock:
            if self.status['busy'] or self.status.get('stopping'):
                raise ValueError('任务忙碌或停止未确认')
            if self.held_object is not None:
                raise ValueError('须在抓取前采集全部目标，当前已有夹持记录')
            if (
                not self.tracker.state.get('model_ready')
                or self.tracker.state.get('state') == 'switching'
            ):
                raise ValueError('视觉模型尚未准备好')
            scene = self.scenes().begin(targets)
            self.cancelled.clear()
            self.actuated = False
            self.robot = None
            self.status.update(
                state='scene_capturing',
                operation='capture_scene_snapshot',
                busy=True,
                error=None,
                scene_id=scene['scene_id'],
                scene_snapshot=None,
                message='静止采集同一帧全部目标，未发送运动',
            )
            self.thread = threading.Thread(
                target=self.run_scene_capture,
                args=(scene, request_id),
                daemon=True,
            )
            self.thread.start()
        return {
            'accepted': True,
            'scene_id': scene['scene_id'],
            'motion_sent': False,
        }

    def stationary_scene_arm(self):
        a = self.arm_state()
        time.sleep(.15)
        b = self.arm_state()
        arm_still(a, b)
        return b

    def run_scene_capture(self, scene, request_id):
        try:
            directory = self.root / 'work/grasp'
            directory.mkdir(parents=True, exist_ok=True)
            with (directory / 'robot.lock').open('a') as lock:
                fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
                config = json.loads(self.config_path.read_text())
                binding = self.scene_binding(config)
                self.robot = RobotClient(
                    config['robot_ip'], config['robot_port'], writes=False
                )
                handeye, tool = self.calibration(config)
                if (
                    self.robot.query('get_current_tool_frame')
                    != tool['controller_readback']['tool_response_raw']
                ):
                    raise ValueError('工具坐标与标定不一致')
                work = self.robot.query('get_current_work_frame')
                if (
                    work.get('frame_name') != 'Base'
                    or work.get('pose') != [0] * 6
                ):
                    raise ValueError('场景须使用零偏移Base')
                self.stationary_scene_arm()
                with self.tracker.lock:
                    frame = copy.deepcopy(self.tracker.latest)
                    self.tracker.generation += 1
                    self.tracker.command = None
                    self.tracker.state.update(
                        state='paused', camera_T_object=None
                    )
                if frame is None:
                    raise ValueError('没有相机画面')
                observation = {
                    'time': frame['time'],
                    'camera': frame['camera'],
                    'K': np.asarray(frame['K']).tolist(),
                }
                arm, _ = self.paired_observation(observation)
                check_camera_observation(handeye, observation)
                base_camera = (
                    controller_pose(arm['pose'])
                    @ np.linalg.inv(transform(tool['flange_T_grip_tcp']))
                    @ transform(handeye['flange_T_color_optical_camera'])
                )
                scene.update(
                    binding=binding,
                    request_id=request_id,
                    captured_at=frame['time'],
                    frame_sequence=frame['sequence'],
                    camera=frame['camera'],
                    K=np.asarray(frame['K']).tolist(),
                    capture_arm=arm,
                    base_T_camera=base_camera.tolist(),
                )
                evidence = self.scenes().root / scene['scene_id']
                evidence.mkdir(exist_ok=True)
                np.savez_compressed(
                    evidence / 'frame.npz',
                    bgr=frame['bgr'],
                    depth=frame['depth'],
                    K=frame['K'],
                )
                scene['frame_evidence'] = scene['scene_id'] + '/frame.npz'
                self.scenes().save(scene)
                deadline = time.monotonic() + 180

                def check():
                    self.check_cancel()
                    if time.monotonic() > deadline:
                        raise ValueError('场景采集超过180秒预算')
                    arm_still(arm, self.arm_state())

                rows = self.tracker.localize_frozen_scene(
                    frame,
                    scene['instances'],
                    check,
                    lambda message: self.update(message=message),
                )
                check()
                if self.scene_binding(config) != binding:
                    raise ValueError('采集期间标定变化')
                expected = {
                    (r['instance_id'], r['object_id'], r['role'])
                    for r in scene['instances']
                }
                if (
                    len(rows) != len(expected)
                    or (
                        {
                            (r['instance_id'], r['object_id'], r['role'])
                            for r in rows
                        }
                        != expected
                    )
                ):
                    raise ValueError('场景目标未全部返回或身份不一致')
                if any(
                    r.get('quality', {}).get('good') is not True for r in rows
                ):
                    raise ValueError('场景包含未通过质量检查的目标')
                for row in rows:
                    row.update(
                        base_T_object=(
                            base_camera @ transform(row['camera_T_object'])
                        ).tolist(),
                        state='static',
                        static_pose_usable=True,
                    )
                scene.update(
                    instances=rows,
                    state='ready',
                    quality_scope=(
                        'single_frozen_rgbd_frame_model_fit_not_metrology'
                    ),
                    assumptions=[
                        'environment_static_during_task',
                        'feature_identity_in_estimated_model_frame',
                    ],
                    ready_at=time.time(),
                )
                self.scenes().save(scene)
                self.validate_scene(scene['scene_id'], config)
                self.update(
                    state='scene_ready',
                    scene_snapshot=self.scene_snapshot(scene['scene_id']),
                    message=(
                        '全部目标已定位并固定为基座坐标；后续动作无需重跟踪'
                    ),
                )
        except Exception as e:
            scene.update(state='failed', error=str(e))
            self.scenes().records[scene['scene_id']] = copy.deepcopy(scene)
            try:
                self.scenes().save(scene)
            except OSError:
                pass
            self.update(
                state='cancelled' if self.cancelled.is_set() else 'failed',
                error=str(e),
                scene_snapshot=self.scene_snapshot(scene['scene_id']),
                message='场景采集失败：' + str(e),
            )
        finally:
            self.update(busy=False)

    def scene_observation(self, ident, instance_id, object_id):
        scene = self.validate_scene(ident)
        row = next(
            (r for r in scene['instances'] if r['instance_id'] == instance_id),
            None,
        )
        if (
            row is None
            or row['object_id'] != object_id
            or row['role'] != 'source'
            or row['state'] != 'static'
        ):
            raise ValueError('抓取必须引用快照中尚未移动的source实例')
        return {
            'object_id': object_id,
            'object_name': row['object_name'],
            'pose': row['camera_T_object'],
            'base_T_object': row['base_T_object'],
            'time': scene['captured_at'],
            'camera': scene['camera'],
            'K': scene['K'],
            'generation': scene['frame_sequence'],
            'model_sha256': row['model_sha256'],
            'pose_mode': row['pose_mode'],
            'pose_mode_revision': row.get('pose_mode_revision'),
            'scene_id': ident,
            'instance_id': instance_id,
        }

    def scene_reference(self, goal):
        scene = self.validate_scene(goal['scene_id'])
        row = next(
            (
                r
                for r in scene['instances']
                if r['instance_id'] == goal['reference_instance_id']
            ),
            None,
        )
        if row is None or row['state'] != 'static':
            raise ValueError('参考实例已移动或不存在，不能沿用原静态位姿')
        if row['pose_mode'] == '5d':
            raise ValueError('5D实例的横向轴不可观测，不能用作完整参考坐标系')
        frame = transform(row['base_T_object'])
        feature = goal['reference_feature_id']
        if feature is not None:
            found = next(
                (f for f in row['features'] if f['id'] == feature), None
            )
            if found is None:
                raise ValueError('命名模型特征不存在')
            frame = frame @ transform(found['object_T_feature'])
        return (
            frame,
            {
                'name': 'scene_object',
                'scene_id': scene['scene_id'],
                'instance_id': row['instance_id'],
                'feature_kind': (
                    found.get('kind') if feature is not None else None
                ),
                'feature_id': feature,
                'base_T_reference': frame.tolist(),
                'captured_at': scene['captured_at'],
                'feature_revision': row['feature_revision'],
                'model_sha256': row['model_sha256'],
            },
        )

    def update_scene_instance(self, held, state, pose=None):
        if held and held.get('scene_id'):
            self.scenes().transition(
                held['scene_id'], held['instance_id'], state, pose
            )
