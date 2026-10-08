#!/usr/bin/env python3
"""Capture one stationary RGB-D view for a fixed object's later reconstruction."""

import argparse
import contextlib
import datetime
import fcntl
import hashlib
import json
from pathlib import Path
import sys
import time
import urllib.error
import urllib.request
import uuid

import cv2
import numpy as np
import pyrealsense2 as rs
import yaml

from geometry_checks import camera_pose, depth_millimeters, duplicate, validate_configs
from depth_preview import export_previews

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / 'tools' / 'handeye'))
from capture import Robot
from geometry import pose_matrix, stationarity


def sha(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def write_json(path, value):
    tmp = path.with_suffix('.tmp')
    tmp.write_text(
        json.dumps(value, ensure_ascii=False, indent=2, allow_nan=False) + '\n'
    )
    tmp.replace(path)


def release_preview():
    url = 'http://127.0.0.1:8088/api/'

    def status():
        with urllib.request.urlopen(url + 'status', timeout=3) as response:
            return json.load(response)

    try:
        current = status()
    except urllib.error.URLError:
        return 'monitor_unavailable; device open must succeed without reset'
    if current['state'] in ('streaming', 'starting', 'stopping'):
        request = urllib.request.Request(url + 'pause', data=b'', method='POST')
        with urllib.request.urlopen(request, timeout=8) as response:
            json.load(response)
        for _ in range(20):
            if status()['state'] == 'idle':
                return 'paused; not automatically resumed'
            time.sleep(.2)
        raise RuntimeError('Preview did not release camera')
    if current['state'] not in ('idle', 'error'):
        raise RuntimeError('Unexpected preview state: ' + repr(current))
    return 'already stopped; not automatically resumed'


def run(args, session, report):
    paths = {
        'handeye': ROOT / 'config/calibration/handeye.yaml',
        'tool': ROOT / 'config/calibration/tool_grip_tcp.yaml',
    }
    configs = {
        key: yaml.safe_load(path.read_text()) for key, path in paths.items()
    }
    handeye, tool = configs['handeye'], configs['tool']
    validate_configs(handeye, tool)
    hashes = {key: sha(path) for key, path in paths.items()}
    for config in configs.values():
        source = (
            ROOT / config['source']['path']
            if 'source' in config
            else ROOT / config['controller_readback']['path']
        )
        expected = (
            config['source']['sha256']
            if 'source' in config
            else config['controller_readback']['sha256']
        )
        if sha(source) != expected:
            raise ValueError('Calibration source hash mismatch: ' + str(source))
        if 'source' in config:
            original = json.loads(source.read_text())
            if not np.allclose(
                original['flange_T_color_optical_camera'],
                config['flange_T_color_optical_camera'],
                rtol=0,
                atol=1e-12,
            ):
                raise ValueError(
                    'Hand-eye matrix differs from original accepted result'
                )
    report['calibration_sha256'] = hashes
    robot = Robot(args.robot_ip)

    def frames():
        return {
            name: robot.query(name)['reply']
            for name in ('get_current_tool_frame', 'get_current_work_frame')
        }

    reference = frames()
    actual_tool = reference['get_current_tool_frame']
    work = reference['get_current_work_frame']
    if actual_tool != tool['controller_readback']['tool_response_raw']:
        raise ValueError(
            'Current tool pose/load differs from saved grip_tcp configuration'
        )
    if not np.allclose(
        pose_matrix(actual_tool['pose']),
        tool['flange_T_grip_tcp'],
        rtol=0,
        atol=1e-9,
    ):
        raise ValueError('Tool SI transform differs from controller readback')
    if work.get('frame_name') != 'Base' or work.get('pose') != [0] * 6:
        raise ValueError('Current work frame must be zero-offset Base')
    info = robot.query('get_arm_software_info')['reply']
    if (
        info.get('Product_version') != handeye['installation']['robot_model']
        or (
            info['ctrl_info']['version']
            != handeye['installation']['controller_firmware']
        )
    ):
        raise ValueError('Robot model/firmware differs from calibration')
    report['robot_frames'] = reference
    report['robot_software'] = info
    observations = report['observations'] = []

    def observe():
        value = robot.query('get_current_arm_state')
        observations.append(value)
        return value

    for _ in range(3):
        observe()
        time.sleep(.15)
    stationarity([row['reply']['arm_state'] for row in observations])

    def to_camera(observation):
        return camera_pose(
            pose_matrix(observation['reply']['arm_state']['pose']),
            tool['flange_T_grip_tcp'],
            handeye['flange_T_color_optical_camera'],
        )

    samples = []
    for path in sorted((session / 'views').glob('*/sample.json')):
        saved = json.loads(path.read_text())
        if saved['status'] == 'candidate':
            samples.append(saved)
    report['existing_view_count'] = len(samples)
    match = duplicate(to_camera(observations[-1]), samples)
    if match:
        report.update(status='duplicate', duplicate_of=match)
        return
    manifest_path = session / 'session.json'
    manifest = (
        json.loads(manifest_path.read_text())
        if manifest_path.exists()
        else None
    )
    if samples and manifest is None:
        raise ValueError(
            'Existing accepted views are missing the session manifest'
        )
    if (
        manifest
        and (
            manifest['calibration_sha256'] != hashes
            or manifest['robot_ip'] != args.robot_ip
        )
    ):
        raise ValueError('Session configuration changed; use a new session')
    report['preview'] = release_preview()
    print('相机预览已释放，正在采集一个静止视角……', flush=True)
    time.sleep(2)
    pipe = rs.pipeline()
    cfg = rs.config()
    cfg.enable_device(handeye['camera']['serial'])
    cfg.enable_stream(rs.stream.color, 1280, 720, rs.format.bgr8, 15)
    cfg.enable_stream(rs.stream.depth, 1280, 720, rs.format.z16, 15)
    profile = pipe.start(cfg)
    try:
        scale = profile.get_device().first_depth_sensor().get_depth_scale()
        for _ in range(20):
            pipe.wait_for_frames(4000)
        observations.clear()
        for _ in range(3):
            observe()
            time.sleep(.15)
        before = observe()
        for _ in range(2):
            wait_start = time.monotonic()
            frameset = pipe.wait_for_frames(4000)
            received = time.monotonic()
            received_unix = time.time()
        after = observe()
        for _ in range(2):
            time.sleep(.15)
            observe()
        report['stationarity'] = stationarity(
            [row['reply']['arm_state'] for row in observations]
        )
        color_native, depth_native = (
            frameset.get_color_frame(), frameset.get_depth_frame()
        )
        if (
            color_native.get_frame_timestamp_domain()
            != depth_native.get_frame_timestamp_domain()
        ):
            raise ValueError('RGB/depth timestamp domains differ')
        delta_ms = abs(
            color_native.get_timestamp() - depth_native.get_timestamp()
        )
        if (
            delta_ms > 100
            or after['received_monotonic'] - before['request_monotonic'] > .5
            or received - wait_start > .2
        ):
            raise ValueError('Capture timing window too large')
        report['timing'] = {
            'method': 'stationary bracket; not hardware synchronization',
            'frame_received_monotonic': received,
            'frame_received_unix': received_unix,
            'color_timestamp_ms': color_native.get_timestamp(),
            'depth_timestamp_ms': depth_native.get_timestamp(),
            'timestamp_domain': str(color_native.get_frame_timestamp_domain()),
            'rgb_depth_delta_ms': delta_ms,
            'color_frame_number': color_native.get_frame_number(),
            'depth_frame_number': depth_native.get_frame_number(),
        }
        aligned = rs.align(rs.stream.color).process(frameset)
        color, depth = aligned.get_color_frame(), aligned.get_depth_frame()
        bgr = np.asanyarray(color.get_data()).copy()
        raw = np.asanyarray(depth.get_data()).copy()
        intr = color.profile.as_video_stream_profile().intrinsics
        K = [[intr.fx, 0, intr.ppx], [0, intr.fy, intr.ppy], [0, 0, 1]]
        if (
            raw.shape != bgr.shape[:2]
            or any(abs(x) > 1e-12 for x in intr.coeffs)
        ):
            raise ValueError(
                'Unsupported image shape or nonzero color distortion'
            )
        if not np.allclose(K, handeye['camera']['K'], rtol=0, atol=.05):
            raise ValueError('Color intrinsics differ from saved calibration')
        camera = {
            'serial': handeye['camera']['serial'],
            'frame': 'color_optical_frame',
            'size': [intr.width, intr.height],
            'fps': 15,
            'K': K,
            'distortion_model': str(intr.model),
            'distortion_coefficients': list(intr.coeffs),
            'device_depth_scale_m': scale,
            'depth_alignment': 'rs.align(color)',
        }
        if manifest and manifest['camera'] != camera:
            raise ValueError('Camera parameters changed during this session')
        report['camera'] = camera
        depth_mm = depth_millimeters(raw, scale)
        valid = depth_mm > 0
        report['depth_quality'] = {
            'whole_image_valid_fraction': float(valid.mean()),
            'whole_image_median_depth_mm': (
                float(np.median(depth_mm[valid])) if valid.any() else None
            ),
            'object_depth_quality': 'not_evaluated_without_mask',
        }
        if valid.mean() < .05:
            raise ValueError(
                'Less than 5 percent of the whole image has valid depth'
            )
        report['image_quality'] = {
            'laplacian_variance': float(
                cv2.Laplacian(
                    cv2.cvtColor(bgr, cv2.COLOR_BGR2GRAY), cv2.CV_64F
                ).var()
            )
        }
        pose = to_camera(before)
        match = duplicate(pose, samples)
        if match:
            report.update(status='duplicate', duplicate_of=match)
            return
        if frames() != reference:
            raise ValueError('Tool/work configuration changed during capture')
        if hashes != {key: sha(path) for key, path in paths.items()}:
            raise ValueError('Calibration files changed during capture')
        report['base_T_grip_tcp'] = pose_matrix(
            before['reply']['arm_state']['pose']
        ).tolist()
        report['base_T_flange'] = (
            np.asarray(report['base_T_grip_tcp'])
            @ np.linalg.inv(np.asarray(tool['flange_T_grip_tcp']))
        ).tolist()
        report['base_T_color_optical_camera'] = pose.tolist()
        report['color_optical_camera_T_base'] = np.linalg.inv(pose).tolist()
        out = session / 'views' / report['sample_id']
        for name, data in [
            ('color.png', bgr),
            ('depth_raw.png', raw),
            ('depth_mm.png', depth_mm),
        ]:
            if not cv2.imwrite(str(out / name), data):
                raise OSError('Image write failed: ' + name)
        depth_m = raw.astype(np.float32) * scale
        np.save(out / 'depth_m.npy', depth_m)
        report['depth_visualization'] = export_previews(out, bgr, depth_m)
        np.savetxt(out / 'K.txt', K)
        np.savetxt(out / 'base_T_camera.txt', pose)
        report['files_sha256'] = {
            p.name: sha(p) for p in out.iterdir() if p.is_file()
        }
        report['depth_units'] = {
            'depth_raw.png': 'device units',
            'depth_mm.png': 'millimeters, uint16',
            'depth_m.npy': 'meters, float32',
        }
        report['mask_status'] = (
            'not_created; object segmentation required before reconstruction'
        )
        if manifest is None:
            manifest = {
                'schema_version': 1,
                'session': args.session,
                'robot_ip': args.robot_ip,
                'camera': camera,
                'calibration_sha256': hashes,
                'object_motion_policy': (
                    'Object and robot base must stay fixed across every view'
                ),
                'world_frame': 'robot base; not a centered object frame',
                'reconstruction_status': 'not_started',
            }
            write_json(manifest_path, manifest)
        report['status'] = 'candidate'
    finally:
        pipe.stop()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--session', default='tape_20260915')
    parser.add_argument('--robot-ip', default='192.168.1.19')
    args = parser.parse_args()
    if (
        not args.session
        or any(
            (
                c
                not in (
                    'abcdefghijklmnopqrstuvwxyzABCDEFGHIJKLMNOPQRSTUVWXYZ012345'
                    '6789_-'
                )
            )
            for c in args.session
        )
    ):
        parser.error('Session must use letters, digits, underscore or hyphen')
    session = ROOT / 'work/object_scan' / args.session
    session.mkdir(parents=True, exist_ok=True)
    with (session / '.lock').open('a') as lock:
        try:
            fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
        except BlockingIOError:
            parser.exit(
                1, 'Another capture is already running for this session\n'
            )
        sample_id = (
            datetime.datetime.now().strftime('%H%M%S')
            + '_'
            + uuid.uuid4().hex[:8]
        )
        out = session / 'views' / sample_id
        out.mkdir(parents=True, exist_ok=False)
        report = {
            'schema_version': 1,
            'sample_id': sample_id,
            'status': 'rejected',
            'observations': [],
            'created_at': datetime.datetime.now().astimezone().isoformat(),
            'frame_convention': 'A_T_B maps coordinates in B into A',
            'object_stationarity': 'required, not automatically verified',
            'robot_motion_commands_sent': False,
            'thresholds': {
                'joint_span_deg': .03,
                'tcp_translation_span_m': .0002,
                'tcp_rotation_span_deg': .15,
                'duplicate_translation_mm': 10,
                'duplicate_rotation_deg': 5,
            },
        }
        try:
            run(args, session, report)
        except Exception as exc:
            report['error'] = str(exc)
        finally:
            write_json(out / 'sample.json', report)
        summary = {
            'directory': str(out),
            'status': report['status'],
            'error': report.get('error'),
            'duplicate_of': report.get('duplicate_of'),
            'stationarity': report.get('stationarity'),
            'depth_quality': report.get('depth_quality'),
        }
        write_json(session / 'latest_capture.json', summary)
        print(json.dumps(summary, ensure_ascii=False, indent=2))
        return (
            0
            if report['status'] == 'candidate'
            else 2
            if report['status'] == 'duplicate'
            else 1
        )


if __name__ == '__main__':
    raise SystemExit(main())
