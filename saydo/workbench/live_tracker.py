#!/usr/bin/env python3
"""D435i/web adapter on official FoundationPose register/track_one; button-triggered grasp control."""

import argparse
import collections
import datetime
import fcntl
import hashlib
import http.server
import json
import logging
import pathlib
import signal
import socket
import sys
import threading
import time
import traceback
import urllib.request
import urllib.parse
from typing import Any, Dict, Optional

import cv2
import numpy as np

from saydo.perception import live_checks
from saydo.objects import object_catalog
from saydo.perception import llm_detection
from saydo.perception import local_grounding
from saydo.perception import local_qwen
from saydo.geometry import pose_symmetry
from saydo.manipulation import grasp_control
from saydo.workbench import live_events
from saydo.perception import camera_checks
from saydo.perception import vision_refinement
from saydo.robot import arms
from saydo.workbench import agent_api as workbench_agent_api
from saydo.workbench import agent_router
from saydo.workbench import arm_camera
from saydo.workbench import arm_vision
from saydo.workbench import no_vision
from saydo.workbench import task_runner_proxy

from saydo import paths

ROOT = paths.ROOT
# 工作台页面与页面脚本所在的目录。
WEB = pathlib.Path(__file__).resolve().with_name('web')
LOG = logging.getLogger(__name__)


class Tracker:
    """8089 的核心对象：读取 1 臂手上相机、跟踪物品，并持有各臂的控制器。

    视觉部分（相机、找框、位姿估计）目前只服务 1 臂。`arms` 里是本机
    已配置的每台臂的动作控制器，`grasp` 是其中 1 臂的那一个。
    """

    def __init__(self, args: argparse.Namespace) -> None:
        self.args = args
        self.models_root = pathlib.Path(
            getattr(args, 'models_root', ROOT / 'work/object_models')
        )
        self.object_info = next(
            (
                row
                for row in object_catalog.scan_objects(self.models_root)
                if row['mesh'] == str(args.mesh.resolve())
            ),
            {
                'id': None,
                'name': args.mesh.parent.name,
                'mesh': str(args.mesh),
                'detector_class': object_catalog.inferred_class(str(args.mesh)),
            },
        )
        if getattr(args, 'detector_class', None):
            self.object_info['detector_class'] = args.detector_class
        self.detector_backend = getattr(args, 'detector', 'qwen')
        self.llm_detector = (
            llm_detection.LLMBoxDetector()
            if self.detector_backend == 'llm'
            else None
        )
        self.local_detector = (
            local_qwen.LocalQwenDetector(ROOT)
            if self.detector_backend == 'qwen'
            else local_grounding.LocalGroundingDetector(ROOT)
        )
        self.scene_job = None
        self.pending_object = None
        self.lock = threading.RLock()
        self.stop = threading.Event()
        self.latest = self.frozen = self.command = None
        # LLM: sampled RGB-D buffer, bounded to about 484 MB.
        self.history = collections.deque(
            maxlen=300 if self.llm_detector else 180
        )
        self.pose_history = collections.deque(maxlen=60)
        self.jpeg = None
        self.generation = 0
        self.retry_at = 0.
        self.last_attempt_sequence = -1
        self.state = {
            'state': 'loading',
            'error': None,
            'model': str(args.mesh),
            'sequence': 0,
            'tracked_frames': 0,
            'registered_frames': 0,
            'camera_T_object': None,
            'pose_accepted_for_robot_control': False,
            'auto_recover': args.auto_recover,
            'recovery_attempts': 0,
            'recovery_interval_s': args.recovery_interval,
        }
        self.output = (
            ROOT
            / 'work/foundationpose_live'
            / datetime.datetime.now().strftime('%Y%m%d_%H%M%S_%f')
        )
        self.output.mkdir(parents=True)
        self.events = live_events.EventLog(self.output / 'events.jsonl')
        self.last_heartbeat = 0.
        self.state['output'] = str(self.output)
        self.state.update(
            model_sha256=hashlib.sha256(args.mesh.read_bytes()).hexdigest(),
            object_id=self.object_info['id'],
            object_name=self.object_info['name'],
            pose_mode=self.object_info.get('pose_mode', '6d'),
            pose_mode_revision=self.object_info.get('pose_mode_revision'),
            detector_class=self.object_info['detector_class'],
            auto_detection_supported=bool(
                self.llm_detector
                or self.object_info.get(self.local_detector.settings_key)
                or self.object_info['detector_class']
            ),
            detector_backend=self.detector_backend,
            local_detector_name=(
                self.local_detector.name
                if (
                    not self.llm_detector
                    and self.object_info.get(self.local_detector.settings_key)
                )
                else None
            ),
            detection_model=llm_detection.MODEL if self.llm_detector else None,
            detection_effort=(
                llm_detection.EFFORT if self.llm_detector else None
            ),
            effective_detection_prompt=self.object_info.get(
                'effective_detection_prompt', self.object_info['name']
            ),
        )
        # 1 臂的取图线程加载 RealSense 库后放在这里，其余相机的线程
        # 从这里取用（库装在 work/ 下，只在那条线程里加入搜索路径）。
        self._realsense = None
        self._realsense_loaded = threading.Event()
        # 实时跟踪接在哪台臂手上的相机；页面可以切换，见 set_live_arm。
        self.live_arm_id = arms.DEFAULT_ARM_ID
        self.state['live_arm'] = self.live_arm_id
        self.arms = self._create_controllers(args)
        # 页面的各项操作都通过 self.grasp 作用在当前实时跟踪的那台臂上。
        self.grasp = self.arms[self.live_arm_id]
        if getattr(args, 'resume_reorientation_report', None):
            self.grasp.restore_reorientation_report(
                args.resume_reorientation_report
            )
        (self.output / 'session.json').write_text(
            json.dumps(
                {
                    'mesh': str(args.mesh),
                    'mesh_sha256': hashlib.sha256(
                        args.mesh.read_bytes()
                    ).hexdigest(),
                    'camera_serial': args.serial,
                    'native_color': [1280, 720, 15],
                    'native_depth': [640, 360, 15],
                    'inference_resolution': [640, 360],
                    'algorithm': (
                        'official FoundationPose register(iteration=5), '
                        'track_one(iteration=2)'
                    ),
                    'robot_commands': False,
                    'pose_frame': 'color optical camera; translation metres',
                    'quality_gate': (
                        'heuristic only; transparent surface depth and model '
                        'dimensions unvalidated'
                    ),
                },
                indent=2,
            )
        )
        self.event(
            'session_started',
            arms=list(self.arms),
            history_frames=self.history.maxlen,
            catchup_max_step_s=.3,
            catchup_max_budget_s=30. if self.llm_detector else 12.,
            catchup_target_age_s=.25,
        )

    def _create_controllers(
        self, args: argparse.Namespace
    ) -> Dict[str, grasp_control.GraspController]:
        """为本机已配置的每台臂创建一个动作控制器；不连接控制器。

        每台臂各有一个视觉来源（`self.visions`）。1 臂的相机由本跟踪器
        的取图线程读取；其余的臂如果在抓取配置里写了 `camera_serial`，
        就各开一条取图线程；没写的使用占位视觉来源：读取状态、复位、
        夹爪和停止可用，需要画面的操作会明确报错。

        Args:
            args: 启动参数；`grasp_config` 只覆盖 1 臂的抓取配置路径。

        Returns:
            臂编号到动作控制器的映射，顺序与登记表一致。
        """
        mode = getattr(args, 'grasp_mode', 'execute')
        controllers = {}
        self.arm_cameras = []
        self.visions = {}
        for arm in arms.configured(ROOT):
            if arm.arm_id == arms.DEFAULT_ARM_ID:
                vision = arm_vision.ArmVision(self, arm)
                config_path = pathlib.Path(
                    getattr(args, 'grasp_config', arm.config_path(ROOT))
                )
                if self._depth_quadratic_per_m(arm, config_path):
                    raise ValueError(
                        f'{arm.label}的相机校正里带有深度校正，'
                        '但本跟踪器读取的相机尚不支持深度校正'
                    )
            else:
                config_path = arm.config_path(ROOT)
                serial = json.loads(config_path.read_text()).get(
                    'camera_serial'
                )
                if serial:
                    vision = arm_camera.ArmCamera(
                        self,
                        arm,
                        serial,
                        self._depth_quadratic_per_m(arm, config_path),
                    )
                    self.arm_cameras.append(vision)
                else:
                    vision = no_vision.NoVision(self.models_root, arm)
            controller = grasp_control.GraspController(
                ROOT, vision, config_path, mode, arm
            )
            vision.grasp = controller
            self.visions[arm.arm_id] = vision
            controllers[arm.arm_id] = controller
        return controllers

    @staticmethod
    def _depth_quadratic_per_m(
        arm: arms.Arm, config_path: pathlib.Path
    ) -> float:
        """读取一台臂相机的深度校正系数；没有配置相机校正时为 0。"""
        if not config_path.is_file():
            return 0.
        name = json.loads(config_path.read_text()).get(
            'vision_refinement_file'
        )
        if not name:
            return 0.
        refinement = vision_refinement.load(ROOT / name, arm.arm_id)
        return float(refinement['depth_quadratic_per_m'])

    def realsense_module(self, timeout_s: float = 30.) -> Optional[Any]:
        """返回 1 臂取图线程已加载的 RealSense 库。

        Args:
            timeout_s: 最多等待多久，单位秒。

        Returns:
            `pyrealsense2` 模块；在限时内没有加载成功时为 None。
        """
        self._realsense_loaded.wait(timeout_s)
        return self._realsense

    def event(self, event, **values):
        return self.events.write(
            event,
            **{
                'generation': self.generation,
                'object_id': self.state.get('object_id'),
                **values,
            },
        )

    def capture(self) -> None:
        """1 臂相机的取图线程：持续把最新的彩色与深度帧交给 1 臂的视觉来源。

        实时跟踪接在 1 臂上时，视觉来源会把画面同时送进 `latest`。

        相机出错时记录原因并让整个服务停止，由操作者处理后重启。
        """
        sys.path.insert(0, str(ROOT / 'work/foundationpose_realsense_py311'))
        import pyrealsense2 as rs
        self._realsense = rs
        self._realsense_loaded.set()
        pipe = rs.pipeline()
        started = False
        try:
            devices = [
                {
                    'serial': d.get_info(rs.camera_info.serial_number),
                    'name': d.get_info(rs.camera_info.name),
                }
                for d in rs.context().query_devices()
            ]
            identity = camera_checks.select_camera(devices, self.args.serial)
            cfg = rs.config()
            cfg.enable_device(identity['serial'])
            cfg.enable_stream(rs.stream.color, 1280, 720, rs.format.bgr8, 15)
            cfg.enable_stream(rs.stream.depth, 640, 360, rs.format.z16, 15)
            profile = pipe.start(cfg)
            started = True
            scale = profile.get_device().first_depth_sensor().get_depth_scale()
            align = rs.align(rs.stream.color)
            for _ in range(20):
                pipe.wait_for_frames(4000)
            sequence = 0
            last_camera_number = None
            while not self.stop.is_set():
                frames = pipe.wait_for_frames(4000)
                # Consume the newest complete frameset rather than a queued
                # older one.
                for _ in range(8):
                    newer = pipe.poll_for_frames()
                    if not newer:
                        break
                    frames = newer
                received_at = time.time()
                native_color, native_depth = (
                    frames.get_color_frame(), frames.get_depth_frame()
                )
                if native_color.get_frame_number() == last_camera_number:
                    continue
                if (
                    native_color.get_frame_timestamp_domain()
                    != native_depth.get_frame_timestamp_domain()
                ):
                    raise ValueError('RGB/depth timestamp domains differ')
                if (
                    abs(
                        native_color.get_timestamp()
                        - native_depth.get_timestamp()
                    )
                    > 100
                ):
                    continue
                timestamp = native_color.get_timestamp() / 1000
                clock_is_wall = (
                    native_color.get_frame_timestamp_domain()
                    in (
                        rs.timestamp_domain.global_time,
                        rs.timestamp_domain.system_time,
                    )
                )
                captured_at = (
                    timestamp
                    if clock_is_wall and 0 <= received_at - timestamp < 10
                    else received_at
                )
                align_started = time.perf_counter()
                aligned = align.process(frames)
                color, depth = (
                    aligned.get_color_frame(), aligned.get_depth_frame()
                )
                intr = color.profile.as_video_stream_profile().intrinsics
                if any(abs(v) > 1e-12 for v in intr.coeffs):
                    raise ValueError(
                        'Nonzero color distortion requires rectification'
                    )
                K = np.array(
                    [
                        [intr.fx / 2, 0, intr.ppx / 2],
                        [0, intr.fy / 2, intr.ppy / 2],
                        [0, 0, 1.],
                    ]
                )
                camera = {
                    **identity,
                    'native_size': [intr.width, intr.height],
                    'image_size': [640, 360],
                    'frame': 'color_optical_frame',
                    'K': K.tolist(),
                    'distortion_coefficients': list(intr.coeffs),
                }
                if sequence == 0:
                    self.event('camera_connected', camera=camera)
                    session_path = self.output / 'session.json'
                    session = json.loads(session_path.read_text())
                    session.update(
                        camera_serial=identity['serial'], camera=camera
                    )
                    session_path.write_text(json.dumps(session, indent=2))
                bgr = cv2.resize(
                    np.asanyarray(color.get_data()),
                    (640, 360),
                    interpolation=cv2.INTER_AREA,
                )
                depth_m = cv2.resize(
                    np.asanyarray(depth.get_data()).astype(np.float32) * scale,
                    (640, 360),
                    interpolation=cv2.INTER_NEAREST,
                )
                sequence += 1
                last_camera_number = native_color.get_frame_number()
                frame = {
                    'bgr': bgr,
                    'depth': depth_m,
                    'K': K,
                    'camera': camera,
                    'sequence': sequence,
                    'camera_frame_number': int(native_color.get_frame_number()),
                    'time': captured_at,
                    'received_at': received_at,
                    'sensor_age_at_receive_s': received_at - captured_at,
                    'alignment_seconds': time.perf_counter() - align_started,
                }
                self.visions[arms.DEFAULT_ARM_ID].publish(frame)
        except Exception as error:
            self.event(
                'camera_failed',
                error=str(error),
                traceback=traceback.format_exc(),
            )
            with self.lock:
                self.state.update(
                    state='camera_error', error=str(error), camera_T_object=None
                )
            self.stop.set()
        finally:
            if started:
                pipe.stop()

    def accept_frame(self, arm_id: str, frame: dict) -> None:
        """接收一帧相机画面；只采用当前实时跟踪的那台臂的画面。

        各臂的取图线程都会调用本函数。

        Args:
            arm_id: 这一帧来自哪台臂手上的相机。
            frame: 一帧画面，含彩色图、深度图、内参、相机身份和时间。
        """
        with self.lock:
            if arm_id != self.live_arm_id:
                return
            self.latest = frame
            if (
                not self.llm_detector
                or not self.history
                or frame['time'] - self.history[-1]['time'] >= .1
            ):
                self.history.append(frame)
            self.state['sequence'] = frame['sequence']
            self.state['camera'] = frame['camera']
            self.state['camera_frame_at'] = frame['time']
            self.state['sensor_age_at_receive_s'] = frame[
                'sensor_age_at_receive_s'
            ]
            self.state['alignment_seconds'] = frame.get('alignment_seconds')

    def arm_choices(self) -> list:
        """列出本机的各台臂，供页面显示“当前机械臂”的切换按钮。

        Returns:
            每台臂一项：编号 `arm`、名称 `label`、是否有相机可做实时
            跟踪 `camera`、是不是当前实时跟踪的臂 `live`。
        """
        return [
            {
                'arm': arm_id,
                'label': controller.arm.label,
                'camera': isinstance(
                    self.visions.get(arm_id), arm_vision.ArmVision
                ),
                'live': arm_id == self.live_arm_id,
            }
            for arm_id, controller in self.arms.items()
        ]

    def set_live_arm(self, arm_id: Any) -> dict:
        """把实时跟踪切换到另一台臂手上的相机；不发送任何机械臂指令。

        定位模型只有一份，实时跟踪同一时间只能接一台相机。切换后页面
        上的画面、找框、跟踪以及抓取、示教、复位等操作都作用在新的
        这台臂上；原来那台臂仍可通过场景快照定位和动作。切换会丢弃
        当前的跟踪结果，状态回到“已暂停”。

        Args:
            arm_id: 要切换到的臂的编号。

        Returns:
            含 `live_arm`（切换后的臂）、`changed`（是否真的换了）和
            `motion_sent`（恒为 False）。

        Raises:
            ValueError: 臂不存在或没有接相机、有臂正在执行任务、模型
                正在切换，或正在采集场景快照。
        """
        if arm_id not in self.arms:
            raise ValueError(f'本机没有配置机械臂 {arm_id!r}')
        vision = self.visions[arm_id]
        if not isinstance(vision, arm_vision.ArmVision):
            raise ValueError(
                f'{self.arms[arm_id].arm.label}没有接相机，不能做实时跟踪'
            )
        with self.lock:
            if arm_id == self.live_arm_id:
                return {
                    'live_arm': arm_id, 'changed': False, 'motion_sent': False
                }
            for controller in self.arms.values():
                with controller.lock:
                    occupied = (
                        controller.status['busy']
                        or controller.status.get('stopping')
                    )
                if occupied:
                    raise ValueError(
                        f'{controller.arm.label}任务进行中，不能切换当前机械臂'
                    )
            if self.state['state'] == 'switching':
                raise ValueError('模型正在切换，请加载完成后再切换机械臂')
            if self.scene_job is not None:
                raise ValueError('正在采集场景快照，请完成后再切换机械臂')
            previous = self.live_arm_id
            frame = vision.own_frame()
            # 代次加一：处理循环里按旧相机算到一半的结果随之作废。
            self.generation += 1
            self.command = None
            self.frozen = None
            self.jpeg = None
            self.history.clear()
            self.pose_history.clear()
            self.last_attempt_sequence = -1
            self.live_arm_id = arm_id
            self.grasp = self.arms[arm_id]
            self.latest = frame
            self.state.update(
                state='paused',
                error=None,
                live_arm=arm_id,
                camera_T_object=None,
                pose_frame_at=None,
                quality={},
                bottle_candidates=[],
                camera=frame['camera'] if frame else None,
                sequence=frame['sequence'] if frame else 0,
            )
            if frame:
                self.state['camera_frame_at'] = frame['time']
            else:
                self.state.pop('camera_frame_at', None)
            self.event('live_arm_changed', previous=previous, arm=arm_id)
        return {'live_arm': arm_id, 'changed': True, 'motion_sent': False}

    def localize_frozen_scene(self, frame, targets, check, progress):
        done = threading.Event()
        cancel = threading.Event()
        job = {
            'done': done,
            'cancel': cancel,
            'frame': frame,
            'targets': targets,
            'progress': progress,
        }
        with self.lock:
            if self.scene_job is not None:
                raise ValueError('已有场景采集请求')
            self.scene_job = job
        error = None
        while not done.wait(.1):
            try:
                check()
            except Exception as failure:
                error = error or failure
                cancel.set()
            if self.stop.is_set():
                raise ValueError('视觉服务已停止')
        if error:
            raise error
        check()
        if 'error' in job:
            raise ValueError(job['error'])
        return job['result']

    def freeze(self):
        with self.lock:
            if self.grasp.snapshot()['busy']:
                raise ValueError('抓取任务处理中，请先停止抓取')
            if self.latest is None or time.time() - self.latest['time'] > 2:
                raise ValueError('还没有新鲜的相机画面')
            if not self.state.get('model_ready'):
                raise ValueError('模型仍在加载')
            self.generation += 1
            self.pose_history.clear()
            self.frozen = self.latest
            self.command = None
            self.state.update(
                state='selecting',
                error=None,
                camera_T_object=None,
                selection_id=self.generation,
            )
            self.jpeg = None
            return {
                'selection_id': self.generation, 'width': 640, 'height': 360
            }

    def initialize(self, value):
        box = live_checks.checked_box(value['box'], 640, 360)
        with self.lock:
            if (
                self.state['state'] != 'selecting'
                or value['selection_id'] != self.generation
            ):
                raise ValueError('框选画面已失效，请重新选择目标')
            self.command = (self.generation, self.frozen, box)
            self.state.update(
                state='initializing', error=None, camera_T_object=None
            )

    def automatic(self, recovery=False):
        with self.lock:
            if not self.state.get('auto_detection_supported'):
                raise ValueError('该物品没有可用的自动检测类别，请使用手动框选')
            self.freeze()
            self.command = (self.generation, self.frozen, None)
            self.state.update(
                state='detecting',
                bottle_candidates=[],
                error=None,
                detection_started_at=time.time(),
                detection_seconds=None,
            )
            if not recovery:
                self.state['recovery_attempts'] = 0
            if recovery:
                self.state['recovery_attempts'] += 1

    def mark_lost(self, error):
        with self.lock:
            self.event('tracking_lost', error=error)
            self.retry_at = (
                time.monotonic()
                + max(
                    self.args.recovery_interval, 5. if self.llm_detector else 0.
                )
            )
            exhausted = bool(
                self.llm_detector and self.state['recovery_attempts'] >= 2
            )
            if exhausted:
                error += '；自动识别重试已达2次，请修改提示词或手动重新识别'
            self.state.update(
                state=(
                    'recovering'
                    if (
                        self.args.auto_recover
                        and self.state.get('auto_detection_supported')
                        and not exhausted
                    )
                    else 'lost'
                ),
                error=error,
                camera_T_object=None,
            )

    def recover_if_due(self):
        with self.lock:
            if self.grasp.snapshot()['busy']:
                return False
            age = time.time() - self.latest['time'] if self.latest else None
            new_frame = (
                self.latest is not None
                and self.latest['sequence'] != self.last_attempt_sequence
            )
            if (
                new_frame
                and live_checks.recovery_due(
                    self.state['state'],
                    self.args.auto_recover,
                    time.monotonic(),
                    self.retry_at,
                    age,
                )
            ):
                self.automatic(recovery=True)
                return True
            return False

    def choose(self, value):
        with self.lock:
            if (
                self.state['state'] != 'choose_candidate'
                or value['selection_id'] != self.generation
            ):
                raise ValueError('候选画面已失效，请重新识别')
            if (
                self.llm_detector
                and (
                    time.time() - self.frozen['time']
                    > llm_detection.MAX_FRAME_AGE_S
                )
            ):
                raise ValueError('候选画面已超过30秒，请重新识别')
            index = value['index']
            candidates = self.state['bottle_candidates']
            if not isinstance(index, int) or not 0 <= index < len(candidates):
                raise ValueError('候选编号无效')
            self.command = (
                self.generation, self.frozen, candidates[index]['box']
            )
            self.state.update(
                state='initializing', error=None, camera_T_object=None
            )

    def pause(self):
        with self.lock:
            if self.grasp.snapshot()['busy']:
                raise ValueError('抓取任务处理中，请使用“停止抓取”')
            if self.state['state'] == 'switching':
                raise ValueError('模型正在切换，请加载完成后再暂停')
            self.generation += 1
            self.command = None
            self.frozen = None
            self.jpeg = None
            self.state.update(state='paused', camera_T_object=None)
            self.event('paused')

    def switch_object(self, object_id):
        info = object_catalog.select_object(self.models_root, object_id)
        with self.lock:
            if self.grasp.snapshot()['busy']:
                raise ValueError('抓取任务处理中，不能切换物品')
            self.generation += 1
            self.command = None
            self.pose_history.clear()
            self.frozen = None
            self.jpeg = None
            self.pending_object = (self.generation, info)
            self.state.update(
                state='switching',
                model_ready=False,
                camera_T_object=None,
                error=None,
                pending_object_name=info['name'],
                quality={},
                bottle_candidates=[],
                pose_frame_at=None,
                model_extents_m=None,
                auto_detection_supported=False,
            )

    def set_detection_prompt(self, value):
        if (
            not isinstance(value, dict)
            or set(value) != {'id', 'prompt', 'revision'}
        ):
            raise ValueError('请提交物品、识别提示词和当前版本')
        with self.lock, self.grasp.lock:
            if self.grasp.status['busy'] or self.grasp.status.get('stopping'):
                raise ValueError('任务进行中，请结束任务后修改识别提示词')
            if self.state['state'] == 'switching':
                raise ValueError('模型正在切换，请加载完成后再保存')
            row = object_catalog.save_detection_prompt(
                self.models_root,
                value['id'],
                value['prompt'],
                value['revision'],
            )
            if self.state.get('object_id') == row['id']:
                self.object_info.update(row)
                self.state['effective_detection_prompt'] = row[
                    'effective_detection_prompt'
                ]
                if (
                    self.state['state']
                    not in ('tracking', 'paused', 'ready', 'loading')
                ):
                    self.generation += 1
                    self.command = self.frozen = self.jpeg = None
                    self.pose_history.clear()
                    self.state.update(
                        state='ready',
                        camera_T_object=None,
                        bottle_candidates=[],
                        error='提示词已更新，请重新识别',
                    )
            self.event(
                'detection_prompt_changed',
                object_id=row['id'],
                revision=row['detection_prompt_revision'],
            )
            return {'object': row, 'motion_sent': False}

    def set_pose_mode(self, value: dict) -> dict:
        """保存一个物品的位姿模式（6D 或 5D）；不发送任何指令。

        位姿模式是物品模型的设置，各臂共用，而且它决定夹持记录怎么
        解释。所以要求全部机械臂都没有任务在执行，也都没有夹着物品。

        Args:
            value: 含物品编号 `id`、位姿模式 `pose_mode` 和页面载入时的
                模型设置版本 `revision`。

        Returns:
            保存后的物品信息；`motion_sent` 恒为 False。

        Raises:
            ValueError: 请求无效、有臂在执行任务或夹着物品，或模型设置
                在页面载入之后已被改动。
            BlockingIOError: 有一台臂的设备锁被占用。
        """
        if (
            not isinstance(value, dict)
            or set(value) != {'id', 'pose_mode', 'revision'}
        ):
            raise ValueError('请提交物品、位姿模式和当前模型设置版本')
        with self.lock, self.grasp.lock:
            for controller in self.arms.values():
                with controller.lock:
                    occupied = (
                        controller.status['busy']
                        or controller.status.get('stopping')
                        or controller.held_object is not None
                    )
                if occupied:
                    raise ValueError(
                        f'{controller.arm.label}任务进行中或仍持有物品，'
                        '不能改变位姿模式'
                    )
            with arms.all_locked(self.grasp.root):
                row = object_catalog.save_pose_mode(
                    self.models_root,
                    value['id'],
                    value['pose_mode'],
                    value['revision'],
                )
                # 各臂已算好的方案都是按旧的位姿模式算的，一并作废。
                for controller in self.arms.values():
                    with controller.lock:
                        controller.status.update(
                            plan=None,
                            reorientation_plan=None,
                            placement_plan=None,
                            grasp_library_changed_at=time.time(),
                        )
                if self.state.get('object_id') == row['id']:
                    self.switch_object(row['id'])
                self.event(
                    'pose_mode_changed',
                    object_id=row['id'],
                    pose_mode=row['pose_mode'],
                )
                return {'object': row, 'motion_sent': False}

    def snapshot(self) -> dict:
        """返回页面轮询用的完整状态；不连接任何设备。

        Returns:
            跟踪状态的副本，另加画面与位姿距今的秒数、当前实时跟踪
            的那台臂的动作控制器状态（`grasp`）和各臂的列表（`arms`）。
            位姿过期（超过 2 秒）或质量不合格时不给出位姿。
        """
        with self.lock:
            result = dict(self.state)
            now = time.time()
            result['camera_age_s'] = (
                now - result['camera_frame_at']
                if 'camera_frame_at' in result
                else None
            )
            result['pose_age_s'] = (
                now - result['pose_frame_at']
                if result.get('pose_frame_at') is not None
                else None
            )
            if (
                result['state'] != 'tracking'
                or result['pose_age_s'] is None
                or result['pose_age_s'] > 2
                or not result.get('quality', {}).get('good')
            ):
                result['camera_T_object'] = None
                result['pose_semantics'] = None
            result['grasp'] = self.grasp.snapshot()
            result['arms'] = self.arm_choices()
            return result

    def grasp_observation(self):
        with self.lock:
            s = self.state
            if (
                s['state'] != 'tracking'
                or not s.get('quality', {}).get('good')
                or s.get('camera_T_object') is None
            ):
                raise ValueError('当前没有可靠跟踪位姿')
            if time.time() - s['pose_frame_at'] > .5:
                raise ValueError('视觉位姿过期，不能用于抓取')
            rows = [
                r
                for r in self.pose_history
                if r['generation'] == self.generation and r['time'] >= s[
                    'pose_frame_at'
                ] - 1.0
            ]
            if len(rows) < 5 or rows[-1]['time'] - rows[0]['time'] < .5:
                raise ValueError(
                    '请保持物品和机械臂静止，等待最近 1 秒内至少 5 个有效位姿，'
                    '且覆盖至少半秒'
                )
            points = np.array([r['pose'] for r in rows])[:, :3, 3]
            if np.max(np.linalg.norm(points - points[-1], axis=1)) > .005:
                raise ValueError('目标位姿仍在变化，请先保持静止')
            rotations = np.array([r['pose'] for r in rows])[:, :3, :3]
            mode = s.get('pose_mode', '6d')
            angles = [
                pose_symmetry.object_rotation_error(
                    rotation, rotations[-1], mode
                )
                for rotation in rotations
            ]
            if np.max(angles) > np.deg2rad(15):
                raise ValueError('目标朝向不稳定，请等待稳定跟踪')
            return {
                'pose': s['camera_T_object'],
                'time': s['pose_frame_at'],
                'generation': self.generation,
                'object_id': s.get('object_id'),
                'object_name': s.get('object_name'),
                'K': s.get('pose_K'),
                'pose_mode': mode,
                'pose_mode_revision': s.get('pose_mode_revision'),
                'model_sha256': s.get('model_sha256'),
                'camera': s.get('pose_camera'),
            }

    def image(self):
        with self.lock:
            state = self.state['state']
            frame = (
                self.frozen
                if state in ('selecting', 'choose_candidate')
                else self.latest
            )
            cached = self.jpeg if state == 'tracking' else None
            age = time.time() - (self.state.get('pose_frame_at') or time.time())
            candidates = (
                list(self.state.get('bottle_candidates', []))
                if state == 'choose_candidate'
                else []
            )
        if cached is not None and age <= 2:
            return cached
        if frame is None:
            return None
        bgr = frame['bgr'].copy()
        for index, candidate in enumerate(candidates):
            x0, y0, x1, y1 = candidate['box']
            cv2.rectangle(bgr, (x0, y0), (x1, y1), (0, 220, 255), 2)
            cv2.putText(
                bgr, str(index + 1), (x0, max(35, y0)), 0, .9, (0, 220, 255), 2
            )
        label = (
            'STALE / NEED REINITIALIZATION'
            if state == 'tracking' and age > 2
            else state.upper()
        )
        cv2.putText(bgr, label, (10, 26), 0, .65, (0, 170, 255), 2)
        return cv2.imencode(
            '.jpg', bgr, [cv2.IMWRITE_JPEG_QUALITY, 75]
        )[1].tobytes()

    def preview_key(self):
        with self.lock:
            mode = self.state['state']
            cached = (
                mode == 'tracking'
                and self.jpeg is not None
                and time.time() - self.state.get('pose_frame_at', 0) <= 2
            )
            frame = (
                self.frozen
                if mode in ('selecting', 'choose_candidate')
                else self.latest
            )
            return (
                self.generation,
                mode,
                (
                    self.state.get('pose_sequence')
                    if cached
                    else (frame['sequence'] if frame else -1)
                ),
            )

    def catch_up(self, generation, frame, pose, advance):
        """Advance the upstream tracker over a bounded recent-frame buffer."""
        started = time.perf_counter()
        count = 0
        step_seconds = .06
        last_progress = started
        with self.lock:
            if generation != self.generation or self.stop.is_set():
                return None
            self.state.update(state='catching_up', camera_T_object=None)
        initial_age = max(0., time.time() - frame['time'])
        budget = min(
            30. if self.llm_detector else 12., max(3., initial_age * 2 + 1)
        )
        self.event(
            'catchup_started',
            generation=generation,
            initial_age_s=initial_age,
            budget_s=budget,
        )
        while True:
            with self.lock:
                if generation != self.generation or self.stop.is_set():
                    self.event(
                        'catchup_cancelled', generation=generation, steps=count
                    )
                    return None
                if time.time() - frame['time'] <= .25:
                    self.event(
                        'catchup_completed',
                        generation=generation,
                        steps=count,
                        seconds=time.perf_counter() - started,
                        result_age_s=time.time() - frame['time'],
                    )
                    return frame, pose, count, time.perf_counter() - started
                history = list(self.history)
                stride = min(.3, max(.12, step_seconds * 2))
                self.state['catchup_progress'] = {
                    'steps': count,
                    'elapsed_s': time.perf_counter() - started,
                    'remaining_age_s': time.time() - frame['time'],
                    'step_seconds': step_seconds,
                    'stride_s': stride,
                    'budget_s': budget,
                    'history_span_s': (
                        history[-1]['time'] - history[0]['time']
                        if history
                        else 0
                    ),
                }
                target = live_checks.catchup_frame(
                    history, frame['sequence'], frame['time'], stride
                )
            if time.perf_counter() - started > budget:
                raise ValueError('重新定位后未能及时追上实时画面，重新识别')
            if target is None:
                self.stop.wait(.01)
                continue
            step_started = time.perf_counter()
            pose = advance(target, pose)
            duration = time.perf_counter() - step_started
            step_seconds = (
                duration if count == 0 else .5 * step_seconds + .5 * duration
            )
            frame = target
            count += 1
            if time.perf_counter() - last_progress >= 1:
                self.event(
                    'catchup_progress',
                    generation=generation,
                    **self.state['catchup_progress'],
                )
                last_progress = time.perf_counter()

    def inference(self):
        try:
            import torch
            torch.set_num_threads(2)
            from saydo.perception import live_detection
            import trimesh
            import nvdiffrast.torch as dr
            from estimater import FoundationPose
            from learning.training.predict_score import ScorePredictor
            from learning.training.predict_pose_refine import PoseRefinePredictor
            from saydo.perception import batched_refiner
            from segment_anything import SamPredictor, sam_model_registry
            from torchvision.models.detection import ssdlite320_mobilenet_v3_large, SSDLite320_MobileNet_V3_Large_Weights
            from Utils import draw_posed_3d_box, draw_xyz_axis, project_3d_to_2d, nvdiffrast_render
            mesh = trimesh.load(self.args.mesh, force='mesh')
            if (
                not np.isfinite(mesh.vertices).all()
                or not (.01 < mesh.extents.max() < 1)
            ):
                raise ValueError(
                    'Model must be a finite metre-scale object mesh'
                )
            origin, extents = trimesh.bounds.oriented_bounds(mesh)
            bbox = np.stack([-extents / 2, extents / 2])
            context = dr.RasterizeCudaContext()
            refiner = PoseRefinePredictor()
            scorer = ScorePredictor()
            if self.detector_backend == 'qwen':
                refiner = batched_refiner.BatchedRefiner(refiner)
                scorer = batched_refiner.batch_score_features(scorer)
            estimator = FoundationPose(
                model_pts=mesh.vertices,
                model_normals=mesh.vertex_normals,
                mesh=mesh,
                scorer=scorer,
                refiner=refiner,
                debug_dir=str(self.output),
                debug=0,
                glctx=context,
            )
            sam = SamPredictor(
                sam_model_registry['vit_b'](
                    checkpoint=str(
                        ROOT / 'work/foundationpose_setup/sam_vit_b_01ec64.pth'
                    )
                ).to('cuda')
            )
            if self.detector_backend == 'qwen':
                sam = batched_refiner.AutocastSam(sam)
            torch.hub.set_dir(str(ROOT / 'work/foundationpose_setup/torch_hub'))
            detector_weights = SSDLite320_MobileNet_V3_Large_Weights.COCO_V1
            detector = (
                None
                if self.llm_detector or self.detector_backend == 'qwen'
                else ssdlite320_mobilenet_v3_large(
                    weights=detector_weights
                ).eval().cuda()
            )
            local_settings = (
                self.object_info.get(self.local_detector.settings_key)
                if not self.llm_detector
                else None
            )
            if local_settings or self.detector_backend == 'qwen':
                self.local_detector.ensure_loaded()
            detector_class = self.object_info['detector_class']
            detector_label = (
                detector_weights.meta['categories'].index(detector_class)
                if (
                    detector_class in detector_weights.meta['categories']
                    and detector is not None
                )
                else None
            )
            logging.getLogger().setLevel(logging.WARNING)
            with self.lock:
                if self.stop.is_set():
                    return
                self.state.update(
                    state='ready',
                    model_ready=True,
                    local_detector_runtime=getattr(
                        self.local_detector, 'ready', None
                    ),
                    model_extents_m=mesh.extents.tolist(),
                    auto_detection_supported=bool(
                        self.llm_detector
                        or local_settings
                        or detector_label is not None
                    ),
                )
            previous = None
            last_sequence = -1
            failures = 0
            auto_attempted = False
            with (
                self.output / 'poses.jsonl'
            ).open('a', buffering=1) as records:
                while not self.stop.is_set():
                    with self.lock:
                        pending = self.pending_object
                        self.pending_object = None
                    if pending:
                        switch_generation, info = pending
                        try:
                            next_mesh = trimesh.load(info['mesh'], force='mesh')
                            if (
                                not np.isfinite(next_mesh.vertices).all()
                                or not (.01 < next_mesh.extents.max() < 1)
                            ):
                                raise ValueError('模型尺寸异常，请确认采用米制')
                            next_origin, next_extents = (
                                trimesh.bounds.oriented_bounds(next_mesh)
                            )
                            next_estimator = FoundationPose(
                                model_pts=next_mesh.vertices,
                                model_normals=next_mesh.vertex_normals,
                                mesh=next_mesh,
                                scorer=estimator.scorer,
                                refiner=estimator.refiner,
                                glctx=context,
                                debug_dir=str(self.output),
                                debug=0,
                            )
                            with self.lock:
                                if switch_generation != self.generation:
                                    continue
                                mesh, origin, bbox = (
                                    next_mesh,
                                    next_origin,
                                    np.stack(
                                        [-next_extents / 2, next_extents / 2]
                                    ),
                                )
                                estimator = next_estimator
                                self.object_info = info
                                local_settings = (
                                    info.get(self.local_detector.settings_key)
                                    if not self.llm_detector
                                    else None
                                )
                                if (
                                    local_settings
                                    or self.detector_backend == 'qwen'
                                ):
                                    self.local_detector.ensure_loaded()
                                detector_class = info['detector_class']
                                detector_label = (
                                    detector_weights.meta['categories'].index(
                                        detector_class
                                    )
                                    if (
                                        (
                                            detector_class
                                            in detector_weights.meta[
                                                'categories'
                                            ]
                                        )
                                        and detector is not None
                                    )
                                    else None
                                )
                                self.state.update(
                                    state='ready',
                                    model_ready=True,
                                    model_sha256=hashlib.sha256(
                                        pathlib.Path(info['mesh']).read_bytes()
                                    ).hexdigest(),
                                    model=info['mesh'],
                                    object_id=info['id'],
                                    pose_mode=info.get('pose_mode', '6d'),
                                    pose_mode_revision=info.get(
                                        'pose_mode_revision'
                                    ),
                                    object_name=info['name'],
                                    pending_object_name=None,
                                    detector_class=detector_class,
                                    auto_detection_supported=bool(
                                        self.llm_detector
                                        or local_settings
                                        or detector_label is not None
                                    ),
                                    model_extents_m=mesh.extents.tolist(),
                                    initialization_seconds=None,
                                    catchup_seconds=None,
                                    catchup_frames=0,
                                    tracked_frames=0,
                                    registered_frames=0,
                                    recovery_attempts=0,
                                    auto_recover=(
                                        self.args.auto_recover
                                        and bool(
                                            self.llm_detector
                                            or local_settings
                                            or detector_label is not None
                                        )
                                    ),
                                    local_detector_name=(
                                        self.local_detector.name
                                        if local_settings
                                        else None
                                    ),
                                    effective_detection_prompt=info.get(
                                        'effective_detection_prompt',
                                        info['name'],
                                    ),
                                )
                                previous, last_sequence, failures = None, -1, 0
                                auto_attempted = not self.args.auto_start
                                event = {
                                    'event': 'object_switched',
                                    'at': time.time(),
                                    'generation': switch_generation,
                                    'object': info,
                                    'mesh_sha256': hashlib.sha256(
                                        pathlib.Path(info['mesh']).read_bytes()
                                    ).hexdigest(),
                                }
                                with (
                                    self.output / 'model_changes.jsonl'
                                ).open('a') as changes:
                                    changes.write(
                                        json.dumps(event, ensure_ascii=False)
                                        + '\n'
                                    )
                                self.event(
                                    'object_switched',
                                    mesh=info['mesh'],
                                    mesh_sha256=event['mesh_sha256'],
                                )
                        except Exception as error:
                            self.event(
                                'model_switch_failed',
                                generation=switch_generation,
                                error=str(error),
                                traceback=traceback.format_exc(),
                            )
                            with self.lock:
                                if switch_generation == self.generation:
                                    self.state.update(
                                        state='model_error',
                                        error=str(error),
                                        model_ready=False,
                                        camera_T_object=None,
                                    )
                            traceback.print_exc()
                        continue
                    with self.lock:
                        scene_job = self.scene_job
                        self.scene_job = None
                    if scene_job is not None:
                        try:
                            from saydo.scene import scene_perception

                            def scene_check():
                                if (
                                    scene_job['cancel'].is_set()
                                    or self.stop.is_set()
                                ):
                                    raise ValueError('场景采集已取消')

                            scene_job['result'] = (
                                scene_perception.localize_scene(
                                    scene_job['frame'],
                                    scene_job['targets'],
                                    self.models_root,
                                    estimator,
                                    sam,
                                    context,
                                    self.llm_detector,
                                    scene_check,
                                    scene_job['progress'],
                                    self.local_detector,
                                )
                            )
                        except Exception as error:
                            scene_job['error'] = str(error)
                            traceback.print_exc()
                        finally:
                            scene_job['done'].set()
                        # Do not resume an old live pose after spending time on
                        # scene inference.
                        previous = None
                        last_sequence = -1
                        with self.lock:
                            self.state.update(
                                state='paused', camera_T_object=None
                            )
                        continue
                    if self.recover_if_due():
                        continue
                    with self.lock:
                        command = self.command
                        self.command = None
                        active = self.state['state'] == 'tracking'
                        generation = self.generation
                        frame = self.latest
                        startup_ready = self.state['state'] == 'ready'
                    if (
                        self.args.auto_start
                        and (
                            self.llm_detector
                            or local_settings
                            or detector_label is not None
                        )
                        and not auto_attempted
                        and frame is not None
                        and startup_ready
                    ):
                        self.automatic()
                        auto_attempted = True
                        continue
                    if command:
                        generation, frame, box = command
                    elif (
                        not active
                        or frame is None
                        or frame['sequence'] == last_sequence
                    ):
                        self.stop.wait(.02)
                        continue
                    elif time.time() - frame['time'] > 2:
                        self.mark_lost('相机画面过期，等待新画面后重新识别')
                        continue
                    started = time.perf_counter()
                    self.last_attempt_sequence = frame['sequence']
                    rgb = cv2.cvtColor(frame['bgr'], cv2.COLOR_BGR2RGB)
                    mask = None
                    initialization_seconds = None
                    catchup_count = 0
                    catchup_seconds = 0.
                    stage = (
                        'detection'
                        if command and box is None
                        else ('segmentation' if command else 'tracking')
                    )
                    timings = {}
                    last_health = None
                    attempt_object_id = self.state.get('object_id')
                    if command:
                        self.event(
                            'attempt_started',
                            generation=generation,
                            object_id=attempt_object_id,
                            sequence=frame['sequence'],
                            input_age_s=time.time() - frame['time'],
                            automatic=box is None,
                        )
                        self.state['catchup_progress'] = None
                    try:
                        with torch.inference_mode():
                            if command:
                                if box is None:
                                    stage_started = time.perf_counter()
                                    if self.llm_detector:
                                        with self.lock:
                                            prompt = self.object_info.get(
                                                'effective_detection_prompt',
                                                self.object_info['name'],
                                            )
                                        candidates = self.llm_detector.detect(
                                            frame,
                                            prompt,
                                            lambda: self.stop.is_set() or generation != self.generation,
                                        )
                                    elif local_settings:
                                        candidates = self.local_detector.detect(
                                            frame,
                                            local_settings,
                                            lambda: self.stop.is_set() or generation != self.generation,
                                        )
                                    elif detector is not None:
                                        prediction = (
                                            live_detection.predict_detection(
                                                detector, rgb
                                            )
                                        )
                                        candidates = (
                                            live_checks.bottle_candidates(
                                                prediction[
                                                    'boxes'
                                                ].detach().cpu().numpy(),
                                                prediction[
                                                    'labels'
                                                ].detach().cpu().numpy(),
                                                prediction[
                                                    'scores'
                                                ].detach().cpu().numpy(),
                                                detector_label,
                                            )
                                        )
                                    else:
                                        raise ValueError(
                                            '当前物品没有Qwen检测配置，'
                                            '请手动框选'
                                        )
                                    timings['detection_s'] = (
                                        time.perf_counter() - stage_started
                                    )
                                    self.event(
                                        'detection_completed',
                                        generation=generation,
                                        candidates=candidates,
                                        seconds=timings['detection_s'],
                                    )
                                    if not candidates:
                                        raise ValueError(
                                            '没有检测到可靠的目标，'
                                            '请让物品完整入镜'
                                        )
                                    with self.lock:
                                        if generation != self.generation:
                                            continue
                                        self.state['bottle_candidates'] = (
                                            candidates
                                        )
                                        self.state['detection_seconds'] = (
                                            timings['detection_s']
                                        )
                                        self.state['local_detector_runtime'] = (
                                            getattr(
                                                self.local_detector,
                                                'ready',
                                                None,
                                            )
                                        )
                                        self.state['detection_method'] = (
                                            (
                                                f'{llm_detection.MODEL}/'
                                                f'{llm_detection.EFFORT} -> '
                                                f'SAM -> '
                                                f'FoundationPose'
                                            )
                                            if self.llm_detector
                                            else (
                                                (
                                                    self.local_detector.name
                                                    + (
                                                        ' -> SAM -> FoundationP'
                                                        'ose'
                                                    )
                                                )
                                                if local_settings
                                                else (
                                                    f'Torchvision SSDLite COCO '
                                                    f'{detector_class} -> SAM '
                                                    f'-> FoundationPose'
                                                )
                                            )
                                        )
                                        if len(candidates) > 1:
                                            self.state.update(
                                                state='choose_candidate',
                                                error=None,
                                            )
                                            continue
                                        self.state['state'] = 'initializing'
                                    box = candidates[0]['box']
                                if (
                                    generation != self.generation
                                    or self.stop.is_set()
                                ):
                                    raise llm_detection.DetectionCancelled()
                                if (
                                    self.llm_detector
                                    and (
                                        time.time() - frame['time']
                                        > llm_detection.MAX_FRAME_AGE_S
                                    )
                                ):
                                    raise ValueError(
                                        '识别帧已超过30秒，请重新识别'
                                    )
                                stage = 'segmentation'
                                stage_started = time.perf_counter()
                                sam.set_image(rgb)
                                masks, scores, _ = sam.predict(
                                    box=np.asarray(box), multimask_output=True
                                )
                                mask = masks[int(np.argmax(scores))]
                                timings['segmentation_s'] = (
                                    time.perf_counter() - stage_started
                                )
                                valid_pixels = int(
                                    (
                                        (frame['depth'] >= .1)
                                        & (frame['depth'] <= 2)
                                        & mask
                                    ).sum()
                                )
                                self.event(
                                    'segmentation_completed',
                                    generation=generation,
                                    mask_pixels=int(mask.sum()),
                                    valid_depth_pixels=valid_pixels,
                                    seconds=timings['segmentation_s'],
                                    box=np.asarray(box).tolist(),
                                )
                                if mask.sum() < 100 or valid_pixels < 100:
                                    raise ValueError(
                                        '目标有效深度不足，请调整距离或视角后重'
                                        '新框选'
                                    )
                                stage = 'registration'
                                stage_started = time.perf_counter()
                                estimator.pose_last = None
                                pose = estimator.register(
                                    K=frame['K'],
                                    rgb=rgb,
                                    depth=frame['depth'].copy(),
                                    ob_mask=mask,
                                    iteration=5,
                                )
                                timings['registration_s'] = (
                                    time.perf_counter() - stage_started
                                )
                                self.event(
                                    'registration_completed',
                                    generation=generation,
                                    seconds=timings['registration_s'],
                                    input_age_s=time.time() - frame['time'],
                                )
                                if estimator.pose_last is None:
                                    raise ValueError(
                                        'FoundationPose '
                                        '首帧初始化没有产生有效状态'
                                    )
                                previous = None
                                stamp = (
                                    self.output / f'initialization_{generation}'
                                )
                                stamp.mkdir()
                                cv2.imwrite(
                                    str(stamp / 'color.png'), frame['bgr']
                                )
                                cv2.imwrite(
                                    str(stamp / 'mask.png'),
                                    mask.astype(np.uint8) * 255,
                                )
                                np.save(stamp / 'depth_m.npy', frame['depth'])
                                np.savetxt(stamp / 'K.txt', frame['K'])
                                initialization_seconds = (
                                    time.perf_counter() - started
                                )
                                if not np.isfinite(pose).all():
                                    raise ValueError(
                                        'FoundationPose returned nonfinite pose'
                                    )

                                def checked_render(
                                    current_pose,
                                    current_frame,
                                    last_pose=None,
                                    initial_mask=None,
                                ):
                                    nonlocal last_health
                                    _, rd, _ = nvdiffrast_render(
                                        K=current_frame['K'],
                                        H=360,
                                        W=640,
                                        mesh=mesh,
                                        ob_in_cams=torch.as_tensor(
                                            current_pose[None],
                                            device='cuda',
                                            dtype=torch.float32,
                                        ),
                                        glctx=context,
                                    )
                                    health = live_checks.pose_health(
                                        current_pose,
                                        rd[0].detach().cpu().numpy(),
                                        current_frame['depth'],
                                        last_pose,
                                        initial_mask,
                                    )
                                    last_health = health
                                    if not health['good']:
                                        raise ValueError(
                                            '重新定位或追帧结果不可靠，重新识别'
                                        )

                                stage = 'initial_quality'
                                checked_render(pose, frame, initial_mask=mask)

                                def advance(target, last_pose):
                                    candidate = estimator.track_one(
                                        rgb=cv2.cvtColor(
                                            target['bgr'], cv2.COLOR_BGR2RGB
                                        ),
                                        depth=target['depth'].copy(),
                                        K=target['K'],
                                        iteration=2,
                                    )
                                    checked_render(candidate, target, last_pose)
                                    return candidate

                                stage = 'catchup'
                                caught = self.catch_up(
                                    generation, frame, pose, advance
                                )
                                if caught is None:
                                    continue
                                frame, pose, catchup_count, catchup_seconds = (
                                    caught
                                )
                                timings['catchup_s'] = catchup_seconds
                                rgb = cv2.cvtColor(
                                    frame['bgr'], cv2.COLOR_BGR2RGB
                                )
                                # Initialization mask belongs to the older
                                # reference frame.
                                mask = None
                            else:
                                pose = estimator.track_one(
                                    rgb=rgb,
                                    depth=frame['depth'].copy(),
                                    K=frame['K'],
                                    iteration=2,
                                )
                            if not np.isfinite(pose).all():
                                raise ValueError(
                                    'FoundationPose returned nonfinite pose'
                                )
                            _, rendered, _ = nvdiffrast_render(
                                K=frame['K'],
                                H=360,
                                W=640,
                                mesh=mesh,
                                ob_in_cams=torch.as_tensor(
                                    pose[None],
                                    device='cuda',
                                    dtype=torch.float32,
                                ),
                                glctx=context,
                            )
                            rendered = rendered[0].detach().cpu().numpy()
                            quality = live_checks.pose_health(
                                pose, rendered, frame['depth'], previous, mask
                            )
                            last_health = quality
                        stage = 'publish'
                        duration = time.perf_counter() - started
                        failures = 0 if quality['good'] else failures + 1
                        lost = (
                            not quality['good']
                            and (bool(command) or failures >= 3)
                        )
                        pose_mode = self.state.get('pose_mode', '6d')
                        overlay = (
                            rgb.copy()
                            if pose_mode == '5d'
                            else draw_posed_3d_box(
                                frame['K'],
                                img=rgb.copy(),
                                ob_in_cam=pose @ np.linalg.inv(origin),
                                bbox=bbox,
                            )
                        )
                        visible = rendered > 0
                        overlay[visible] = (
                            .75 * overlay[visible]
                            + .25 * np.array([0, 200, 255])
                        ).astype(np.uint8)
                        axis_points = np.vstack([np.zeros(3), np.eye(3) * .05])
                        axis_depths = (
                            axis_points @ pose[:3, :3].T + pose[:3, 3]
                        )[:, 2]
                        if quality['good'] and np.all(axis_depths > .001):
                            # Draw the actual mesh frame returned as
                            # camera_T_object,
                            # not the separately rotated oriented-bounding-box
                            # frame.
                            if pose_mode == '5d':
                                start = project_3d_to_2d(
                                    np.array([0., 0., 0., 1.]), frame['K'], pose
                                )
                                end = project_3d_to_2d(
                                    np.array([0., 0., .05, 1.]),
                                    frame['K'],
                                    pose,
                                )
                                cv2.line(
                                    overlay,
                                    tuple(map(int, start)),
                                    tuple(map(int, end)),
                                    (0, 100, 255),
                                    3,
                                )
                            else:
                                overlay = draw_xyz_axis(
                                    overlay,
                                    ob_in_cam=pose,
                                    scale=.05,
                                    K=frame['K'],
                                    thickness=3,
                                    is_input_rgb=True,
                                )
                            for axis, name, color in [
                                (0, 'X', (255, 0, 0)),
                                (1, 'Y', (0, 255, 0)),
                                (2, 'Z', (0, 100, 255)),
                            ]:
                                if pose_mode == '5d' and axis != 2:
                                    continue
                                endpoint = np.array([0., 0., 0., 1.])
                                endpoint[axis] = .05
                                u, v = project_3d_to_2d(
                                    endpoint, frame['K'], pose
                                )
                                if 0 <= u < 640 and 0 <= v < 360:
                                    location = (int(u) + 3, int(v) - 3)
                                    cv2.putText(
                                        overlay,
                                        name,
                                        location,
                                        0,
                                        .55,
                                        (0, 0, 0),
                                        3,
                                    )
                                    cv2.putText(
                                        overlay,
                                        name,
                                        location,
                                        0,
                                        .55,
                                        color,
                                        1,
                                    )
                        bgr = cv2.cvtColor(overlay, cv2.COLOR_RGB2BGR)
                        label = (
                            'LOST - RESELECT'
                            if lost
                            else (
                                'TRACKING' if quality['good'] else 'UNCERTAIN'
                            )
                        )
                        cv2.putText(
                            bgr,
                            f'{label} | {1/max(duration,.001):.1f} inference '
                            f'FPS',
                            (8, 25),
                            0,
                            .6,
                            (0, 255, 0) if quality['good'] else (0, 120, 255),
                            2,
                        )
                        with self.lock:
                            if generation != self.generation:
                                continue
                            mode = self.state.get('pose_mode', '6d')
                            published = pose_symmetry.canonical_pose(pose, mode)
                            self.state.update(
                                state='lost' if lost else 'tracking',
                                camera_T_object=(
                                    published.tolist()
                                    if quality['good']
                                    else None
                                ),
                                pose_semantics=pose_symmetry.pose_fields(
                                    published, mode
                                ),
                                quality=quality,
                                inference_seconds=duration,
                                pose_frame_at=frame['time'],
                                pose_sequence=frame['sequence'],
                                pose_K=frame['K'].tolist(),
                                pose_camera=frame.get('camera'),
                                error=None,
                            )
                            if command:
                                self.state.update(
                                    initialization_seconds=(
                                        initialization_seconds
                                    ),
                                    catchup_frames=catchup_count,
                                    catchup_seconds=catchup_seconds,
                                )
                            self.state[
                                'registered_frames'
                                if command
                                else 'tracked_frames'
                            ] += 1
                            if quality['good'] and not lost:
                                self.pose_history.append(
                                    {
                                        'pose': published.tolist(),
                                        'time': frame['time'],
                                        'generation': generation,
                                    }
                                )
                            if lost:
                                self.mark_lost(
                                    '深度或位姿匹配不可靠，等待重新识别'
                                )
                            self.frozen = None
                            self.jpeg = cv2.imencode(
                                '.jpg', bgr, [cv2.IMWRITE_JPEG_QUALITY, 75]
                            )[1].tobytes()
                            record = {
                                **self.snapshot(),
                                'camera_frame_number': frame[
                                    'camera_frame_number'
                                ],
                                'processed_at': time.time(),
                            }
                            status_temp = self.output / 'status.tmp'
                            status_temp.write_text(json.dumps(record, indent=2))
                            status_temp.replace(self.output / 'status.json')
                            (self.output / 'latest.jpg').write_bytes(self.jpeg)
                        records.write(json.dumps(record) + '\n')
                        if command:
                            self.event(
                                'attempt_completed',
                                generation=generation,
                                object_id=attempt_object_id,
                                seconds=time.perf_counter() - started,
                                timings=timings,
                                quality=quality,
                                result_age_s=time.time() - frame['time'],
                                catchup_steps=catchup_count,
                                outcome='lost' if lost else 'tracking',
                            )
                        if time.monotonic() - self.last_heartbeat >= 5:
                            self.event(
                                'tracking_summary',
                                generation=generation,
                                sequence=frame['sequence'],
                                inference_s=duration,
                                result_age_s=time.time() - frame['time'],
                                quality=quality,
                                tracked_frames=self.state['tracked_frames'],
                            )
                            self.last_heartbeat = time.monotonic()
                        previous = np.asarray(pose).copy()
                        last_sequence = frame['sequence']
                    except llm_detection.DetectionCancelled:
                        continue
                    except Exception as error:
                        failure = self.event(
                            'attempt_failed' if command else 'frame_failed',
                            generation=generation,
                            object_id=attempt_object_id,
                            stage=stage,
                            error=str(error),
                            error_type=type(error).__name__,
                            traceback=traceback.format_exc(),
                            seconds=time.perf_counter() - started,
                            timings=timings,
                            input_age_s=time.time() - frame['time'],
                            quality=last_health,
                            catchup=(
                                self.state.get('catchup_progress')
                                if stage == 'catchup'
                                else None
                            ),
                        )
                        with self.lock:
                            if generation == self.generation:
                                self.state['last_failure'] = failure
                                self.mark_lost(str(error))
                        print('Frame failure:', repr(error), flush=True)
                        if not isinstance(error, ValueError):
                            traceback.print_exc()
        except Exception as error:
            self.event(
                'model_failed',
                error=str(error),
                traceback=traceback.format_exc(),
            )
            with self.lock:
                self.state.update(
                    state='model_error', error=str(error), camera_T_object=None
                )
            print('Model failure:', repr(error), flush=True)


def abort_all(tracker: Any) -> None:
    """取消全部机械臂的当前任务；正在运动的臂会收到停止指令。

    逐台处理，一台失败不影响其余各臂；全部处理完后再把第一处失败抛出。

    Args:
        tracker: 跟踪器，名下有各臂的动作控制器。

    Raises:
        Exception: 有臂的停止指令未得到确认时，抛出第一处的原始错误。
    """
    first_error = None
    for arm_id, controller in agent_router.controllers_of(tracker).items():
        try:
            controller.abort()
        except Exception as error:
            # 保护代码：一台臂停止失败时仍必须继续停其余的臂。
            LOG.warning('%s 停止指令未确认：%r', arm_id, error)
            if first_error is None:
                first_error = error
    if first_error is not None:
        raise first_error


def main() -> None:
    """启动 8089 服务：读取相机、跟踪物品，并提供页面和 Agent 接口。"""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        '--mesh',
        type=pathlib.Path,
        default=ROOT / 'work/object_models/ob_yibao_350ml/model/model.obj',
    )
    parser.add_argument(
        '--models-root', type=pathlib.Path, default=ROOT / 'work/object_models'
    )
    parser.add_argument(
        '--detector',
        choices=('llm', 'ssdlite', 'qwen'),
        default='qwen',
        help=(
            'Bounding-box detector: resident Qwen NF4 (default), remote llm, '
            'or per-object local/SSDLite'
        ),
    )
    parser.add_argument(
        '--detector-class',
        help=(
            'Override initial model COCO class; unknown classes use manual '
            'initialization'
        ),
    )
    parser.add_argument(
        '--grasp-mode',
        choices=['preview', 'execute'],
        default='execute',
        help=(
            'Default execute: the grasp button may send motion after all '
            'checks; preview explicitly disables robot writes'
        ),
    )
    parser.add_argument(
        '--grasp-config',
        type=pathlib.Path,
        default=ROOT / 'config/grasp/trial.json',
    )
    parser.add_argument(
        '--resume-reorientation-report',
        type=pathlib.Path,
        help=(
            'One-use read-only recovery of the latest failed reorientation; '
            'operator must confirm the same grasp was never released or moved'
        ),
    )
    parser.add_argument(
        '--serial',
        help='Select a camera by serial; default: the only connected D435I',
    )
    parser.add_argument(
        '--agent-token-file',
        type=pathlib.Path,
        help=(
            'Enable authenticated Agent routes using an owner-only bearer '
            'token file'
        ),
    )
    parser.add_argument('--port', type=int, default=8089)
    parser.add_argument(
        '--host',
        default='127.0.0.1',
        help='HTTP bind address; use the MSI LAN IP for direct LAN access',
    )
    parser.add_argument(
        '--auto-start',
        action='store_true',
        help='Detect a bottle and initialize once after camera/model startup',
    )
    parser.add_argument(
        '--no-auto-recover',
        dest='auto_recover',
        action='store_false',
        help='Disable automatic detection/reinitialization after tracking loss',
    )
    parser.add_argument(
        '--recovery-interval',
        type=float,
        default=0.,
        help=(
            'Recovery cooldown in seconds; default 0 retries immediately on a '
            'fresh frame'
        ),
    )
    args = parser.parse_args()
    if not np.isfinite(args.recovery_interval) or args.recovery_interval < 0:
        parser.error('recovery-interval must be nonnegative')
    cv2.setNumThreads(1)
    runtime = ROOT / 'work/foundationpose_runtime'
    sys.path[:0] = [
        str(runtime), str(runtime / 'bundlesdf'), str(runtime / 'mycpp/build')
    ]
    base = ROOT / 'work/foundationpose_live'
    base.mkdir(exist_ok=True)
    camera_lock = (base / 'camera.lock').open('a')
    fcntl.flock(camera_lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
    # Release the existing preview through its documented API; never reset
    # hardware.
    try:
        with urllib.request.urlopen(
            'http://127.0.0.1:8088/api/status', timeout=2
        ) as response:
            status = json.load(response)
        if status['state'] in ('streaming', 'starting', 'stopping'):
            request = urllib.request.Request(
                'http://127.0.0.1:8088/api/pause', data=b'', method='POST'
            )
            with urllib.request.urlopen(request, timeout=8):
                pass
    except OSError:
        pass
    tracker = Tracker(args)
    agent_api = (
        agent_router.ArmRouter(tracker, args.agent_token_file)
        if args.agent_token_file
        else None
    )
    task_runner = task_runner_proxy.TaskRunnerProxy()

    class Handler(http.server.BaseHTTPRequestHandler):
        def log_message(self, *unused):
            pass

        def reply(self, status, body, mime='application/json'):
            if not isinstance(body, bytes):
                body = json.dumps(body, ensure_ascii=False).encode()
            self.send_response(status)
            self.send_header('Content-Type', mime)
            self.send_header('Cache-Control', 'no-store')
            if self.path.startswith('/task-runner/'):
                self.send_header(
                    'Content-Security-Policy',
                    "default-src 'self'; img-src 'self' data:; style-src "
                    "'self' 'unsafe-inline'; script-src 'self'; "
                    "frame-ancestors 'self'",
                )
                self.send_header('X-Content-Type-Options', 'nosniff')
            self.send_header('Content-Length', str(len(body)))
            self.end_headers()
            try:
                self.wfile.write(body)
            except (BrokenPipeError, ConnectionResetError):
                pass

        def do_GET(self):
            path = self.path.split('?')[0]
            if task_runner.matches(self.path):
                try:
                    self.reply(*task_runner.request('GET', self.path))
                except ValueError as error:
                    self.reply(400, {'error': str(error)})
                return
            if path.startswith('/api/agent/'):
                if agent_api is None:
                    self.reply(
                        404,
                        {
                            'error': {
                                'code': 'AGENT_DISABLED',
                                'message': 'Agent 接口未启用',
                            }
                        },
                    )
                elif not agent_api.authorized(
                    self.headers.get('Authorization')
                ):
                    self.reply(
                        401,
                        {
                            'error': {
                                'code': 'UNAUTHORIZED',
                                'message': 'Agent 认证失败',
                            }
                        },
                    )
                else:
                    try:
                        params = urllib.parse.parse_qs(
                            urllib.parse.urlsplit(self.path).query
                        )
                        self.reply(200, agent_api.get(path, params))
                    except workbench_agent_api.AgentError as error:
                        self.reply(400, {'error': error.payload})
                    except (
                        ValueError, KeyError, TypeError, RuntimeError, OSError
                    ) as error:
                        self.reply(
                            400,
                            {
                                'error': {
                                    'code': 'READ_FAILED', 'message': str(error)
                                }
                            },
                        )
                return
            if path == '/':
                self.reply(
                    200,
                    WEB.joinpath('live_tracker.html').read_bytes(),
                    'text/html; charset=utf-8',
                )
            elif path == '/reorientation_viewer.js':
                self.reply(
                    200,
                    WEB.joinpath('reorientation_viewer.js').read_bytes(),
                    'application/javascript; charset=utf-8',
                )
            elif path == '/placement_panel.js':
                self.reply(
                    200,
                    WEB.joinpath('placement_panel.js').read_bytes(),
                    'application/javascript; charset=utf-8',
                )
            elif path == '/api/reorient/model':
                try:
                    params = urllib.parse.parse_qs(
                        urllib.parse.urlsplit(self.path).query
                    )
                    self.reply(
                        200,
                        tracker.grasp.reorientation_model(
                            params.get('object_id', [None])[0]
                        ),
                    )
                except (ValueError, TypeError, KeyError, OSError) as error:
                    self.reply(400, {'error': str(error)})
            elif path == '/grasp_editor.js':
                self.reply(
                    200,
                    WEB.joinpath('grasp_editor.js').read_bytes(),
                    'application/javascript; charset=utf-8',
                )
            elif path == '/teach_grasp_panel.js':
                self.reply(
                    200,
                    WEB.joinpath('teach_grasp_panel.js').read_bytes(),
                    'application/javascript; charset=utf-8',
                )
            elif path in ('/arm_speed_panel.js', '/arm_switch_panel.js'):
                self.reply(
                    200,
                    WEB.joinpath(path[1:]).read_bytes(),
                    'application/javascript; charset=utf-8',
                )
            elif path in ('/api/grasp/profiles', '/api/grasp/editor-model'):
                try:
                    params = urllib.parse.parse_qs(
                        urllib.parse.urlsplit(self.path).query
                    )
                    object_id = params.get('object_id', [None])[0]
                    result = (
                        tracker.grasp.grasp_profiles(object_id)
                        if path.endswith('/profiles')
                        else tracker.grasp.grasp_editor_model(object_id)
                    )
                    self.reply(200, result)
                except (ValueError, TypeError, KeyError, OSError) as error:
                    self.reply(400, {'error': str(error)})
            elif path == '/api/status':
                self.reply(200, tracker.snapshot())
            elif path == '/api/objects':
                self.reply(
                    200,
                    {
                        'objects': object_catalog.scan_objects(
                            tracker.models_root
                        )
                    },
                )
            elif path == '/frame.jpg':
                frame = tracker.image()
                self.reply(200, frame, 'image/jpeg') if frame else self.reply(
                    503, {'error': '等待相机画面'}
                )
            elif path == '/stream.mjpg':
                self.connection.settimeout(3)
                self.connection.setsockopt(
                    socket.IPPROTO_TCP, socket.TCP_NODELAY, 1
                )
                self.connection.setsockopt(
                    socket.SOL_SOCKET, socket.SO_SNDBUF, 16384
                )
                self.send_response(200)
                self.send_header(
                    'Content-Type', 'multipart/x-mixed-replace; boundary=frame'
                )
                self.send_header('Cache-Control', 'no-store')
                self.end_headers()
                last_key = None
                try:
                    while not tracker.stop.is_set():
                        key = tracker.preview_key()
                        if key == last_key:
                            tracker.stop.wait(.015)
                            continue
                        jpeg = tracker.image()
                        if jpeg is None:
                            tracker.stop.wait(.05)
                            continue
                        metadata = f'X-Preview-State: {key[1]}\r\nX-Preview-Sequence: {key[2]}\r\n'.encode()
                        self.wfile.write(
                            b'--frame\r\nContent-Type: image/jpeg\r\n'
                            + metadata
                            + b'Content-Length: '
                            + str(len(jpeg)).encode()
                            + b'\r\n\r\n'
                            + jpeg
                            + b'\r\n'
                        )
                        self.wfile.flush()
                        last_key = key
                        tracker.stop.wait(.04)
                except (
                    BrokenPipeError, ConnectionResetError, TimeoutError, OSError
                ):
                    pass
            else:
                self.reply(404, {'error': 'not found'})

        def do_POST(self):
            if self.path.startswith('/api/agent/'):
                if agent_api is None:
                    self.reply(
                        404,
                        {
                            'error': {
                                'code': 'AGENT_DISABLED',
                                'message': 'Agent 接口未启用',
                            }
                        },
                    )
                    return
                if not agent_api.authorized(self.headers.get('Authorization')):
                    self.reply(
                        401,
                        {
                            'error': {
                                'code': 'UNAUTHORIZED',
                                'message': 'Agent 认证失败',
                            }
                        },
                    )
                    return
            origin = self.headers.get('Origin')
            if origin and origin != 'http://' + self.headers.get('Host', ''):
                self.reply(403, {'error': 'origin mismatch'})
                return
            try:
                size = int(self.headers.get('Content-Length', 0))
                task_request = task_runner.matches(self.path)
                if (
                    task_request
                    and (
                        self.headers.get('Content-Type', '').split(';')[0]
                        != 'application/json'
                    )
                ):
                    raise ValueError('Task requests require application/json')
                if (
                    not 0
                    <= size
                    <= (
                        20000
                        if (
                            task_request
                            or self.path == '/api/object/detection-prompt'
                        )
                        else 4096
                    )
                ):
                    raise ValueError('Invalid request size')
                value = json.loads(self.rfile.read(size) or b'{}')
                if task_request:
                    response = task_runner.request('POST', self.path, value)
                    if (
                        self.path == '/api/task-runs/stop'
                        and response[0] >= 500
                    ):
                        abort_all(tracker)
                    self.reply(*response)
                    return
                if self.path.startswith('/api/agent/'):
                    result = agent_api.post(self.path, value)
                elif self.path == '/api/freeze':
                    result = tracker.freeze()
                elif self.path == '/api/live-arm':
                    result = tracker.set_live_arm(value.get('arm'))
                elif self.path == '/api/object':
                    tracker.switch_object(value['id'])
                    result = {'accepted': True}
                elif self.path == '/api/object/detection-prompt':
                    result = tracker.set_detection_prompt(value)
                elif self.path == '/api/object/pose-mode':
                    result = tracker.set_pose_mode(value)
                elif self.path == '/api/automatic':
                    tracker.automatic()
                    result = {'accepted': True}
                elif self.path == '/api/choose':
                    tracker.choose(value)
                    result = {'accepted': True}
                elif self.path == '/api/initialize':
                    tracker.initialize(value)
                    result = {'accepted': True}
                elif self.path == '/api/pause':
                    tracker.pause()
                    result = {'paused': True}
                elif self.path == '/api/grasp':
                    tracker.grasp.start()
                    result = {'accepted': True, 'mode': args.grasp_mode}
                elif self.path == '/api/grasp/preview':
                    tracker.grasp.start(preview=True)
                    result = {'accepted': True, 'mode': 'preview'}
                elif self.path == '/api/grasp/profiles':
                    result = tracker.grasp.edit_grasp_profiles(value)
                elif self.path == '/api/grasp/teach':
                    result = tracker.grasp.teach(
                        value.get('use_current_point', False),
                        gripper_parameters=value.get('gripper_parameters'),
                        expected_object_id=value.get('object_id'),
                    )
                elif self.path == '/api/gripper/status':
                    result = tracker.grasp.hardware_status()
                elif self.path == '/api/gripper/open':
                    result = tracker.grasp.open_gripper()
                elif self.path == '/api/gripper/force-config':
                    result = tracker.grasp.set_force_config(value['value'])
                elif self.path == '/api/gripper/open-position':
                    result = tracker.grasp.set_open_position(value['value'])
                elif self.path == '/api/arm/reset':
                    result = tracker.grasp.start_reset()
                elif self.path == '/api/arm/speed':
                    if set(value) != {'value', 'revision'}:
                        raise ValueError('请提交速度百分比和当前配置版本')
                    result = tracker.grasp.set_arm_speed(
                        value['value'], value['revision']
                    )
                elif self.path == '/api/arm/reset-zero':
                    result = tracker.grasp.start_reset('zero')
                elif self.path == '/api/reorient/preview':
                    result = tracker.grasp.start_reorient(value, preview=True)
                elif self.path == '/api/reorient':
                    result = tracker.grasp.start_reorient(value)
                elif self.path == '/api/place/config':
                    result = tracker.grasp.save_placement_config(value)
                elif self.path == '/api/place/preview':
                    result = tracker.grasp.start_placement(value, preview=True)
                elif self.path == '/api/place':
                    result = tracker.grasp.start_placement(value)
                elif self.path == '/api/grasp/stop':
                    abort_all(tracker)
                    result = {'cancelled': True}
                elif self.path == '/api/stop':
                    abort_all(tracker)
                    tracker.stop.set()
                    result = {'stopping': True}
                    threading.Thread(
                        target=server.shutdown, daemon=True
                    ).start()
                else:
                    self.reply(404, {'error': 'not found'})
                    return
                self.reply(200, result)
            except workbench_agent_api.AgentError as error:
                self.reply(400, {'error': error.payload})
            except (
                ValueError, KeyError, TypeError, RuntimeError, OSError
            ) as error:
                self.reply(400, {'error': str(error)})

    server = http.server.ThreadingHTTPServer((args.host, args.port), Handler)
    threads = [
        threading.Thread(target=tracker.capture),
        threading.Thread(target=tracker.inference),
    ]
    # 其余各臂手上相机的取图线程；没有配置时这个列表是空的。
    threads += [
        threading.Thread(target=camera.run)
        for camera in getattr(tracker, 'arm_cameras', [])
    ]
    for thread in threads:
        thread.start()

    def shutdown(*unused):
        try:
            abort_all(tracker)
        except Exception as error:
            print('Grasp stop failure:', error, flush=True)
        tracker.stop.set()
        if hasattr(getattr(tracker, 'local_detector', None), 'close'):
            tracker.local_detector.close()
        threading.Thread(target=server.shutdown, daemon=True).start()

    signal.signal(signal.SIGTERM, shutdown)
    signal.signal(signal.SIGINT, shutdown)
    try:
        server.serve_forever()
    finally:
        tracker.stop.set()
        if hasattr(getattr(tracker, 'local_detector', None), 'close'):
            tracker.local_detector.close()
        for thread in threads:
            thread.join(timeout=15)
        server.server_close()
        tracker.event('session_stopped')
        tracker.events.close()
        camera_lock.close()


if __name__ == '__main__':
    main()
