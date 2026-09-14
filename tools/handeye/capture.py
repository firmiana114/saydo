#!/usr/bin/env python3
"""Capture one stationary sample. Robot protocol is restricted to read-only queries."""
import argparse
import contextlib
import datetime
import hashlib
import json
import socket
import subprocess
import time
import uuid
from pathlib import Path

import cv2
import numpy as np
import pyrealsense2 as rs
from geometry import board_pose, pose_matrix, stationarity, zero_reference_frames

ROOT = Path(__file__).resolve().parents[2]
READ_ONLY = {'get_arm_software_info', 'get_current_arm_state', 'get_current_tool_frame', 'get_current_work_frame', 'get_joint_en_state', 'get_joint_err_flag'}


class Robot:
    def __init__(self, ip):
        self.ip = ip

    def query(self, command):
        if command not in READ_ONLY:
            raise ValueError('Robot command is not read-only')
        started = time.monotonic()
        with socket.create_connection((self.ip, 8080), timeout=2) as sock:
            sock.settimeout(2)
            sock.sendall((json.dumps({'command': command})+'\r\n').encode())
            with sock.makefile('rb') as stream:
                for _ in range(8):
                    data = stream.readline(16384)
                    if not data:
                        raise RuntimeError('Controller closed connection')
                    reply = json.loads(data)
                    if reply.get('state') == command.removeprefix('get_') or reply.get('command') == command.removeprefix('get_'):
                        return {'request_monotonic': started, 'received_monotonic': time.monotonic(), 'received_unix': time.time(), 'reply': reply}
            raise RuntimeError('Matching controller reply not received')


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--robot-ip', default='192.168.1.19')
    parser.add_argument('--session', default=datetime.date.today().isoformat())
    args = parser.parse_args()
    if not args.session or any(c not in 'abcdefghijklmnopqrstuvwxyzABCDEFGHIJKLMNOPQRSTUVWXYZ0123456789_-' for c in args.session):
        parser.error('Session must contain only letters, numbers, underscore and hyphen')
    sample_id = datetime.datetime.now().strftime('%H%M%S')+'_'+uuid.uuid4().hex[:8]
    out = ROOT/'work'/'handeye'/args.session/sample_id
    out.mkdir(parents=True, exist_ok=False)
    report = {'schema_version': 1, 'sample_id': sample_id, 'status': 'rejected', 'handeye_calibrated': False,
              'board': {'inner_corners': [8, 11], 'square_size_m': .03, 'size_source': 'user stated; physical measurement pending'},
              'frame_convention': 'A_T_B maps coordinates from B to A', 'observations': [],
              'thresholds': {'joint_span_deg': .03, 'translation_span_m': .0002, 'rotation_span_deg': .15,
                             'max_bracket_s': .5, 'max_frame_wait_s': .2, 'max_reprojection_rms_px': 1.0},
              'threshold_scope': 'provisional single-sample screening, not hand-eye accuracy acceptance'}
    pipe = rs.pipeline()
    started = False
    try:
        robot = Robot(args.robot_ip)
        def observe():
            value = robot.query('get_current_arm_state')
            report['observations'].append(value)
            return value
        preflight = {cmd: robot.query(cmd)['reply'] for cmd in READ_ONLY if cmd != 'get_current_arm_state'}
        report['preflight'] = preflight
        if preflight['get_arm_software_info'].get('Product_version') != 'RM75-BI':
            raise ValueError('Unexpected robot identity')
        zero_reference_frames(preflight['get_current_tool_frame'], preflight['get_current_work_frame'])
        if preflight['get_joint_en_state'].get('en_state') != [1]*7 or preflight['get_joint_err_flag'].get('err_flag') != [0]*7:
            raise ValueError('Joint enable/error check failed')
        report['motion_readiness'] = 'NOT_ASSESSED: tool mass/center of mass and workspace clearance need separate verification'
        # Refuse to compete with preview or other video capture processes.
        nodes = list(Path('/dev').glob('video*'))
        if nodes:
            users = subprocess.run(['fuser', *map(str, nodes)], capture_output=True, text=True, timeout=3)
            if users.returncode not in (0, 1) or users.stdout.strip():
                raise RuntimeError('Video device in use; release camera preview before capturing')
        devices = list(rs.context().query_devices())
        matches = [d for d in devices if 'D435' in d.get_info(rs.camera_info.name).upper()]
        if len(matches) != 1:
            raise ValueError('Expected exactly one D435i camera')
        serial = matches[0].get_info(rs.camera_info.serial_number)
        config = rs.config()
        config.enable_device(serial)
        config.enable_stream(rs.stream.color, 1280, 720, rs.format.bgr8, 15)
        profile = pipe.start(config)
        started = True
        for _ in range(20):
            pipe.wait_for_frames(4000)
        for _ in range(3):
            observe()
            time.sleep(.2)
        # Bracket fresh frames with actual controller responses. Only static samples are supported.
        before = observe()
        for _ in range(3):
            wait_start = time.monotonic()
            frames = pipe.wait_for_frames(4000)
            receive_time = time.monotonic()
        frame = frames.get_color_frame()
        after = observe()
        for _ in range(2):
            time.sleep(.2)
            observe()
        report['stationarity'] = stationarity([v['reply']['arm_state'] for v in report['observations']])
        report['timing'] = {'method': 'stationary_robot_bracket_not_hardware_synchronization',
                            'before': before['request_monotonic'], 'frame_received_monotonic': receive_time,
                            'after': after['received_monotonic'], 'frame_wait_s': receive_time-wait_start,
                            'camera_timestamp_ms': frame.get_timestamp(), 'camera_clock_domain': str(frame.get_frame_timestamp_domain()),
                            'frame_number': frame.get_frame_number(), 'host_unix_saved': time.time()}
        if after['received_monotonic']-before['request_monotonic'] > .5 or receive_time-wait_start > .2:
            raise ValueError('Capture timing window too large')
        im = np.asanyarray(frame.get_data()).copy()
        intr = frame.profile.as_video_stream_profile().intrinsics
        report['camera'] = {'serial': serial, 'name': profile.get_device().get_info(rs.camera_info.name),
                            'frame': 'color_optical_frame', 'size': [intr.width, intr.height], 'fps': 15,
                            'K': [[intr.fx, 0, intr.ppx], [0, intr.fy, intr.ppy], [0, 0, 1]],
                            'distortion_model': str(intr.model), 'distortion_coefficients': list(intr.coeffs)}
        if any(abs(c) > 1e-12 for c in intr.coeffs):
            raise ValueError('Nonzero distortion requires model-specific handling; not silently treated as pinhole')
        if not cv2.imwrite(str(out/'color.png'), im):
            raise RuntimeError('Could not save original image')
        report['image_sha256'] = hashlib.sha256((out/'color.png').read_bytes()).hexdigest()
        found, corners = cv2.findChessboardCornersSB(cv2.cvtColor(im, cv2.COLOR_BGR2GRAY), (8, 11), cv2.CALIB_CB_NORMALIZE_IMAGE)
        if not found:
            raise ValueError('All 88 board corners were not found')
        report['corners_px'] = corners.reshape(-1, 2).tolist()
        report['board_pose_candidates'] = board_pose(corners, np.array(report['camera']['K']), np.zeros(5))
        best = report['board_pose_candidates'][0]
        if best['reprojection_rms_px'] > 1.0:
            raise ValueError('Board reprojection error exceeds screening threshold')
        report['base_T_flange'] = pose_matrix(before['reply']['arm_state']['pose']).tolist()
        # A board pose is not the hand-eye transform. Preserve both planar solutions for review.
        report['camera_T_board_candidate'] = best['camera_T_board']
        report['requires_review'] = ['physical square-size measurement', 'consistent board corner origin/direction',
                                     'planar pose ambiguity', 'independent multi-pose validation']
        last_tool = robot.query('get_current_tool_frame')['reply']
        last_work = robot.query('get_current_work_frame')['reply']
        if last_tool != preflight['get_current_tool_frame'] or last_work != preflight['get_current_work_frame']:
            raise ValueError('Tool/work frame configuration changed during capture')
        annotated = im.copy()
        cv2.drawChessboardCorners(annotated, (8, 11), corners, True)
        cv2.imwrite(str(out/'corners.jpg'), annotated)
        report['status'] = 'candidate'
    except Exception as exc:
        report['error'] = str(exc)
    finally:
        if started:
            with contextlib.suppress(Exception):
                pipe.stop()
        (out/'sample.json').write_text(json.dumps(report, indent=2, allow_nan=False))
    print(json.dumps({'directory': str(out), 'status': report['status'], 'error': report.get('error'),
                      'stationarity': report.get('stationarity'),
                      'reprojection_rms_px': report.get('board_pose_candidates', [{}])[0].get('reprojection_rms_px')}, ensure_ascii=False))
    return 0 if report['status'] == 'candidate' else 1


if __name__ == '__main__':
    raise SystemExit(main())
