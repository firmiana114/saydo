#!/usr/bin/env python3
"""Capture one stationary sample. Robot protocol is restricted to read-only queries."""

import argparse
import contextlib
import datetime
import hashlib
import json
import os
import socket
import subprocess
import time
import uuid
import pathlib
from typing import List, Optional

import cv2
import numpy as np
import pyrealsense2 as rs
from saydo.calibration import geometry

from saydo import paths

ROOT = paths.ROOT
READ_ONLY = {
    'get_arm_software_info',
    'get_current_arm_state',
    'get_current_tool_frame',
    'get_current_work_frame',
    'get_joint_en_state',
    'get_joint_err_flag',
}


class Robot:
    def __init__(self, ip):
        self.ip = ip

    def query(self, command):
        if command not in READ_ONLY:
            raise ValueError('Robot command is not read-only')
        started = time.monotonic()
        with socket.create_connection((self.ip, 8080), timeout=2) as sock:
            sock.settimeout(2)
            sock.sendall((json.dumps({'command': command}) + '\r\n').encode())
            with sock.makefile('rb') as stream:
                for _ in range(8):
                    data = stream.readline(16384)
                    if not data:
                        raise RuntimeError('Controller closed connection')
                    reply = json.loads(data)
                    if (
                        reply.get('state') == command.removeprefix('get_')
                        or reply.get('command') == command.removeprefix('get_')
                    ):
                        return {
                            'request_monotonic': started,
                            'received_monotonic': time.monotonic(),
                            'received_unix': time.time(),
                            'reply': reply,
                        }
            raise RuntimeError('Matching controller reply not received')


def select_camera(serial: Optional[str] = None) -> dict:
    """在已连接的相机里选出要采集的那台 D435i；只枚举，不打开设备。

    两台机械臂手上各有一台 D435i，同时连着时必须指明用哪一台，
    否则可能拿 1 臂相机的图去配 2 臂的位姿。

    Args:
        serial: 相机序列号（RealSense SDK 报告的那个）；None 表示
            “只连着一台 D435i 时就用它”。

    Returns:
        含 `serial`（SDK 序列号）、`name`（型号）和 `asic_serial`
        （芯片序列号，系统的 USB 设备信息里记的是它，用来找这台相机
        对应的视频设备节点；读不到时为 None）。

    Raises:
        ValueError: 找不到指定的相机，或没有指定而连接的 D435i 不是
            恰好一台。
    """
    cameras = []
    for device in rs.context().query_devices():
        name = device.get_info(rs.camera_info.name)
        if 'D435' not in name.upper():
            continue
        cameras.append(
            {
                'serial': device.get_info(rs.camera_info.serial_number),
                'name': name,
                'asic_serial': (
                    device.get_info(rs.camera_info.asic_serial_number)
                    if device.supports(rs.camera_info.asic_serial_number)
                    else None
                ),
            }
        )
    connected = '、'.join(camera['serial'] for camera in cameras) or '无'
    if serial is None:
        if len(cameras) != 1:
            raise ValueError(
                f'连接着 {len(cameras)} 台 D435i（{connected}），'
                '请用 --serial 指定要采集的那一台'
            )
        return cameras[0]
    for camera in cameras:
        if camera['serial'] == serial:
            return camera
    raise ValueError(
        f'没有找到序列号为 {serial} 的 D435i；已连接的 D435i：{connected}'
    )


def video_nodes(
    asic_serial: Optional[str],
    sysfs: pathlib.Path = pathlib.Path('/sys/class/video4linux'),
    dev: pathlib.Path = pathlib.Path('/dev'),
) -> List[str]:
    """列出一台相机在系统里对应的视频设备节点。

    Args:
        asic_serial: 相机的芯片序列号；None 表示无法区分相机。
        sysfs: 系统列出视频设备的目录，测试时换成临时目录。
        dev: 设备节点所在目录，测试时换成临时目录。

    Returns:
        视频设备节点的路径。无法区分相机时返回全部视频节点，这样占用
        检查会偏严（别的相机在用也算占用），不会漏掉真正的占用。
    """
    everything = sorted(str(path) for path in dev.glob('video*'))
    if asic_serial is None:
        return everything
    nodes = []
    for item in sysfs.glob('video*'):
        for parent in item.resolve().parents:
            recorded = parent / 'serial'
            if (parent / 'idVendor').exists() and recorded.exists():
                if recorded.read_text().strip() == asic_serial:
                    nodes.append(str(dev / item.name))
                break
    return sorted(nodes) or everything


def camera_users(camera: dict) -> List[int]:
    """查出哪些进程正占用着一台相机；不打开设备。

    只看这一台相机，所以 8089 在用 1 臂相机时仍可以采集 2 臂相机。

    Args:
        camera: `select_camera` 的返回值。

    Returns:
        占用者的进程号，升序；不含本进程。

    Raises:
        RuntimeError: 系统工具 fuser 运行失败，无法判断是否被占用。
    """
    nodes = video_nodes(camera.get('asic_serial'))
    if not nodes:
        return []
    result = subprocess.run(
        ['fuser', *nodes], capture_output=True, text=True, timeout=3
    )
    if result.returncode not in (0, 1):
        raise RuntimeError(
            f'无法检查相机占用状态（fuser 退出码 {result.returncode}）'
        )
    return sorted(
        {
            int(word)
            for word in result.stdout.split()
            if word.isdigit() and int(word) != os.getpid()
        }
    )


def main() -> int:
    """采集一组静止样本并写入会话目录；只读查询机械臂，不发送任何指令。

    Returns:
        进程退出码：0 表示样本通过筛选成为候选，1 表示被拒收。无论
        哪种结果都会留下 `sample.json`，拒收原因写在其中。
    """
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--robot-ip', default='192.168.1.19')
    parser.add_argument(
        '--serial',
        help='要采集的 D435i 的序列号；只连着一台时可以不写',
    )
    parser.add_argument('--session', default=datetime.date.today().isoformat())
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
        parser.error(
            'Session must contain only letters, numbers, underscore and hyphen'
        )
    sample_id = (
        datetime.datetime.now().strftime('%H%M%S') + '_' + uuid.uuid4().hex[:8]
    )
    out = ROOT / 'work' / 'handeye' / args.session / sample_id
    out.mkdir(parents=True, exist_ok=False)
    report = {
        'schema_version': 1,
        'sample_id': sample_id,
        'status': 'rejected',
        'handeye_calibrated': False,
        'board': {
            'inner_corners': [8, 11],
            'square_size_m': .03,
            'size_source': 'user stated; physical measurement pending',
        },
        'frame_convention': 'A_T_B maps coordinates from B to A',
        'observations': [],
        'thresholds': {
            'joint_span_deg': .03,
            'translation_span_m': .0002,
            'rotation_span_deg': .15,
            'max_bracket_s': .5,
            'max_frame_wait_s': .2,
            'max_reprojection_rms_px': 1.0,
        },
        'threshold_scope': (
            'provisional single-sample screening, not hand-eye accuracy '
            'acceptance'
        ),
    }
    pipe = rs.pipeline()
    started = False
    try:
        robot = Robot(args.robot_ip)

        def observe():
            value = robot.query('get_current_arm_state')
            report['observations'].append(value)
            return value

        preflight = {
            cmd: robot.query(cmd)['reply']
            for cmd in READ_ONLY
            if cmd != 'get_current_arm_state'
        }
        report['preflight'] = preflight
        if (
            preflight['get_arm_software_info'].get('Product_version')
            != 'RM75-BI'
        ):
            raise ValueError('Unexpected robot identity')
        geometry.zero_reference_frames(
            preflight['get_current_tool_frame'],
            preflight['get_current_work_frame'],
        )
        if (
            preflight['get_joint_en_state'].get('en_state') != [1] * 7
            or preflight['get_joint_err_flag'].get('err_flag') != [0] * 7
        ):
            raise ValueError('Joint enable/error check failed')
        report['motion_readiness'] = (
            'NOT_ASSESSED: tool mass/center of mass and workspace clearance '
            'need separate verification'
        )
        camera = select_camera(args.serial)
        serial = camera['serial']
        # 不与预览或其他取图进程抢同一台相机；别的相机被占用不受影响。
        users = camera_users(camera)
        if users:
            raise RuntimeError(
                f'相机 {serial} 正被进程 {users} 占用；'
                '请先释放这台相机的预览或停止占用它的服务'
            )
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
        # Bracket fresh frames with actual controller responses. Only static
        # samples are supported.
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
        report['stationarity'] = geometry.stationarity(
            [v['reply']['arm_state'] for v in report['observations']]
        )
        report['timing'] = {
            'method': 'stationary_robot_bracket_not_hardware_synchronization',
            'before': before['request_monotonic'],
            'frame_received_monotonic': receive_time,
            'after': after['received_monotonic'],
            'frame_wait_s': receive_time - wait_start,
            'camera_timestamp_ms': frame.get_timestamp(),
            'camera_clock_domain': str(frame.get_frame_timestamp_domain()),
            'frame_number': frame.get_frame_number(),
            'host_unix_saved': time.time(),
        }
        if (
            after['received_monotonic'] - before['request_monotonic'] > .5
            or receive_time - wait_start > .2
        ):
            raise ValueError('Capture timing window too large')
        im = np.asanyarray(frame.get_data()).copy()
        intr = frame.profile.as_video_stream_profile().intrinsics
        report['camera'] = {
            'serial': serial,
            'name': profile.get_device().get_info(rs.camera_info.name),
            'frame': 'color_optical_frame',
            'size': [intr.width, intr.height],
            'fps': 15,
            'K': [[intr.fx, 0, intr.ppx], [0, intr.fy, intr.ppy], [0, 0, 1]],
            'distortion_model': str(intr.model),
            'distortion_coefficients': list(intr.coeffs),
        }
        if any(abs(c) > 1e-12 for c in intr.coeffs):
            raise ValueError(
                'Nonzero distortion requires model-specific handling; not '
                'silently treated as pinhole'
            )
        if not cv2.imwrite(str(out / 'color.png'), im):
            raise RuntimeError('Could not save original image')
        report['image_sha256'] = hashlib.sha256(
            (out / 'color.png').read_bytes()
        ).hexdigest()
        found, corners = cv2.findChessboardCornersSB(
            cv2.cvtColor(im, cv2.COLOR_BGR2GRAY),
            (8, 11),
            cv2.CALIB_CB_NORMALIZE_IMAGE,
        )
        if not found:
            raise ValueError('All 88 board corners were not found')
        report['corners_px'] = corners.reshape(-1, 2).tolist()
        report['board_pose_candidates'] = geometry.board_pose(
            corners, np.array(report['camera']['K']), np.zeros(5)
        )
        best = report['board_pose_candidates'][0]
        if best['reprojection_rms_px'] > 1.0:
            raise ValueError(
                'Board reprojection error exceeds screening threshold'
            )
        report['base_T_flange'] = geometry.pose_matrix(
            before['reply']['arm_state']['pose']
        ).tolist()
        # A board pose is not the hand-eye transform. Preserve both planar
        # solutions for review.
        report['camera_T_board_candidate'] = best['camera_T_board']
        report['requires_review'] = [
            'physical square-size measurement',
            'consistent board corner origin/direction',
            'planar pose ambiguity',
            'independent multi-pose validation',
        ]
        last_tool = robot.query('get_current_tool_frame')['reply']
        last_work = robot.query('get_current_work_frame')['reply']
        if (
            last_tool != preflight['get_current_tool_frame']
            or last_work != preflight['get_current_work_frame']
        ):
            raise ValueError(
                'Tool/work frame configuration changed during capture'
            )
        annotated = im.copy()
        cv2.drawChessboardCorners(annotated, (8, 11), corners, True)
        cv2.imwrite(str(out / 'corners.jpg'), annotated)
        report['status'] = 'candidate'
    except Exception as exc:
        report['error'] = str(exc)
    finally:
        if started:
            with contextlib.suppress(Exception):
                pipe.stop()
        (out / 'sample.json').write_text(
            json.dumps(report, indent=2, allow_nan=False)
        )
    print(
        json.dumps(
            {
                'directory': str(out),
                'status': report['status'],
                'error': report.get('error'),
                'stationarity': report.get('stationarity'),
                'reprojection_rms_px': report.get(
                    'board_pose_candidates', [{}]
                )[0].get('reprojection_rms_px'),
            },
            ensure_ascii=False,
        )
    )
    return 0 if report['status'] == 'candidate' else 1


if __name__ == '__main__':
    raise SystemExit(main())
