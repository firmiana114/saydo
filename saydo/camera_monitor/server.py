#!/usr/bin/env python3
"""Read-only, on-demand dual-camera MJPEG monitor. No robot writes."""

import argparse
import base64
import fcntl
import json
import logging
import os
import pathlib
import signal
import socket
import subprocess
import sys
import threading
import time
import uuid
import http.server
import urllib.parse
from typing import Optional

from saydo import paths

# 预览页面与页面脚本所在的目录。
WEB = pathlib.Path(__file__).resolve().with_name('web')
PROJECT = paths.ROOT
LOG = logging.getLogger('camera_monitor')
# 页面与接口里各路相机的键。2 臂手上的相机只有配置了序列号才出现。
ARM2_CAMERA_KEY = 'd435i_arm2'
# 把相机让给采集程序的最长时间，单位秒。采集程序结束时会主动归还；
# 这个上限只是防止它异常退出后预览永远连不上。
MAX_HOLD_S = 60.


def camera_users(serial):
    nodes = []
    for item in pathlib.Path('/sys/class/video4linux').glob('video*'):
        for parent in item.resolve().parents:
            vendor = parent / 'idVendor'
            device_serial = parent / 'serial'
            if vendor.exists() and device_serial.exists():
                if (
                    vendor.read_text().strip() == '8086'
                    and device_serial.read_text().strip() == serial
                ):
                    nodes.append('/dev/' + item.name)
                break
    if not nodes:
        return []
    result = subprocess.run(
        ['fuser', *nodes], capture_output=True, text=True, timeout=3
    )
    if result.returncode not in (0, 1):
        raise RuntimeError('无法检查相机占用状态，请检查 fuser。')
    return sorted(
        {
            int(p)
            for p in result.stdout.split()
            if p.isdigit() and int(p) != os.getpid()
        }
    )


class Camera:
    """一路相机的按需预览：有人看时才打开设备，最后一个人离开就释放。

    Attributes:
        camera_id: 这路相机在页面和接口里的键。
        name: 相机型号，用来在已连接的设备里找到它。
        role: `eye_in_hand`（装在机械臂手上）或 `movable_global`
            （不固定的全局相机）。
        arm: 装在哪台机械臂手上（`arm1`、`arm2`）；全局相机为 None。
        hold_until: 让给采集程序的截止时刻（`time.monotonic()` 的值）；
            在此之前不接受新的预览连接。
    """

    def __init__(
        self,
        width: int,
        height: int,
        fps: int,
        camera_id: str = 'd435i',
        serial: Optional[str] = None,
        tracker: Optional[object] = None,
        arm: Optional[str] = None,
    ) -> None:
        """创建一路预览；不打开设备。

        Args:
            width: 请求的图像宽度，单位像素。
            height: 请求的图像高度，单位像素。
            fps: 请求的帧率。
            camera_id: 这路相机的键；`d405` 是全局相机，其余都是手上
                的 D435i。
            serial: 相机序列号；None 表示只连着一台该型号时就用它。
            tracker: 8089 的客户端；给出时优先共享 8089 的画面而不
                打开设备。
            arm: 相机装在哪台机械臂手上；全局相机为 None。
        """
        self.width, self.height, self.target_fps = width, height, fps
        self.requested_size = (width, height)
        self.cv = threading.Condition()
        self.thread = None
        self.stop_event = threading.Event()
        self.epoch = 0
        self.clients = 0
        self.state = 'idle'
        self.error = None
        self.jpeg = None
        self.sequence = 0
        self.frame_at = None
        self.fps = 0.0
        self.camera_id, self.serial, self.tracker = camera_id, serial, tracker
        is_global = camera_id == 'd405'
        self.name = 'RealSense D405' if is_global else 'RealSense D435I'
        self.role = 'movable_global' if is_global else 'eye_in_hand'
        self.arm = arm
        self.hold_until = 0.
        self.source = None
        self.frame_metadata = {}
        self.usb = None

    def status(self) -> dict:
        """返回这路相机的当前状态；不打开设备。

        Returns:
            状态、相机身份、帧率和最近一帧的时间等。`held_s` 是还要
            让给采集程序多少秒，0 表示没有被让出。
        """
        with self.cv:
            return dict(
                state=self.state,
                arm=self.arm,
                held_s=round(
                    max(0., self.hold_until - time.monotonic()), 1
                ),
                error=self.error,
                camera=self.name,
                usb=self.usb,
                camera_id=self.camera_id,
                camera_serial=self.serial,
                camera_role=self.role,
                source=self.source,
                width=self.width,
                height=self.height,
                target_fps=self.target_fps,
                fps=round(self.fps, 1),
                clients=self.clients,
                sequence=self.sequence,
                frame_at=self.frame_at,
                frame_age_ms=(
                    round((time.time() - self.frame_at) * 1000)
                    if self.frame_at
                    else None
                ),
            )

    def hold(self, seconds: float) -> None:
        """把相机让给采集程序一段时间：这期间不接受新的预览连接。

        为什么需要它：页面在预览断开后会自动重连。如果不先挡住，
        重连会和采集程序抢同一台相机，让采集失败。

        Args:
            seconds: 让出的时长，单位秒；0 表示立即归还。超过上限
                `MAX_HOLD_S` 时按上限计。
        """
        seconds = min(max(0., float(seconds)), MAX_HOLD_S)
        with self.cv:
            self.hold_until = time.monotonic() + seconds if seconds else 0.

    def subscribe(self) -> int:
        """登记一个预览连接，必要时启动采集线程。

        Returns:
            本次采集的代次，断开时原样交还给 `unsubscribe`。

        Raises:
            RuntimeError: 相机已让给采集程序、正在释放，或连接数已满。
        """
        with self.cv:
            remaining = self.hold_until - time.monotonic()
            if remaining > 0:
                raise RuntimeError(
                    f'相机已让给采集程序，约 {remaining:.0f} 秒内归还。'
                )
            if (
                self.thread
                and self.thread.is_alive()
                and self.stop_event.is_set()
            ):
                raise RuntimeError('相机正在释放，请稍后重试。')
            if self.clients >= 4:
                raise RuntimeError('同时预览上限为 4 个连接。')
            self.clients += 1
            if not self.thread or not self.thread.is_alive():
                self.stop_event = threading.Event()
                self.epoch += 1
                self.state, self.error, self.jpeg = 'starting', None, None
                self.frame_at, self.fps = None, 0.0
                self.thread = threading.Thread(
                    target=self.capture,
                    args=(self.epoch, self.stop_event),
                    daemon=True,
                )
                self.thread.start()
            return self.epoch

    def publish(
        self, jpeg, captured_at, sequence, metadata, source, width, height
    ):
        if (
            not jpeg.startswith(b'\xff\xd8')
            or not 0 <= time.time() - captured_at <= 1
        ):
            raise RuntimeError('相机图像无效或超过一秒时效。')
        with self.cv:
            self.jpeg, self.frame_at, self.sequence = (
                jpeg, captured_at, sequence
            )
            self.frame_metadata = metadata
            self.source, self.width, self.height = source, width, height
            self.state = 'streaming'
            self.cv.notify_all()

    def fresh_frame(self):
        """Wait for a frame captured after this observation request, without starting a device."""
        after = time.time()
        with self.cv:
            if self.state != 'streaming':
                raise RuntimeError('该相机没有活动预览。')
            self.cv.wait_for(
                lambda: (
                    self.frame_at is not None and self.frame_at >= after
                ) or self.state != 'streaming' or self.stop_event.is_set(),
                timeout=2,
            )
            if (
                self.stop_event.is_set()
                or self.state != 'streaming'
                or self.frame_at is None
                or self.frame_at < after
            ):
                raise RuntimeError('没有取得请求之后的新鲜相机帧。')
            return {
                **self.frame_metadata,
                'camera_role': self.role,
                'camera_serial': self.serial,
                'camera_name': self.name,
                'captured_at': self.frame_at,
                'sequence': self.sequence,
                'image_size': [self.width, self.height],
                'mime_type': 'image/jpeg',
                'image_base64': base64.b64encode(self.jpeg).decode(),
                'observation_id': uuid.uuid4().hex,
                'timestamp_basis': 'host_frame_received_unix_seconds',
                'capture_source': 'camera_monitor',
            }

    def unsubscribe(self, epoch):
        with self.cv:
            if epoch != self.epoch:
                return
            self.clients = max(0, self.clients - 1)
            if self.clients == 0:
                self.stop_event.set()
                self.cv.notify_all()

    def stop(self):
        with self.cv:
            self.stop_event.set()
            if self.state in ('starting', 'streaming'):
                self.state = 'stopping'
            self.cv.notify_all()
            thread = self.thread
        if thread:
            thread.join(timeout=6)
        return self.status()

    def capture(self, epoch, stop):
        pipeline = None
        device_lock = None
        started = False
        try:
            if self.tracker is not None:
                # Only GET the raw camera frame; never pause or reconfigure
                # 8089.
                from saydo.agent import client
                try:
                    first = self.tracker.request('/api/agent/frame')
                except client.BackendError as exc:
                    if exc.payload.get('code') != 'BACKEND_UNAVAILABLE':
                        raise RuntimeError(
                            '8089 原始画面不可用：' + str(exc)
                        ) from exc
                else:
                    self.capture_tracker(first, stop)
                    return
            import cv2
            import numpy as np
            import pyrealsense2 as rs
            devices = rs.context().query_devices()
            matches = [
                d
                for d in devices
                if d.get_info(rs.camera_info.name) == self.name and (
                    self.serial is None
                    or d.get_info(rs.camera_info.serial_number) == self.serial
                )
            ]
            if len(matches) != 1:
                raise RuntimeError(
                    '没有找到唯一的指定相机，请检查连接或指定序列号。'
                )
            self.serial = matches[0].get_info(rs.camera_info.serial_number)
            if self.camera_id == 'd405':
                directory = PROJECT / 'work/global_camera'
                directory.mkdir(parents=True, exist_ok=True)
                device_lock = (directory / (self.serial + '.lock')).open('a')
                try:
                    fcntl.flock(device_lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
                except BlockingIOError as exc:
                    raise RuntimeError(
                        'D405 正在被全局观察使用，请稍后重试。'
                    ) from exc
            if camera_users(self.serial):
                raise RuntimeError(
                    '该相机已被其他程序占用；不会抢占或复位设备。'
                )
            pipeline = rs.pipeline()
            config = rs.config()
            config.enable_device(self.serial)
            config.enable_stream(
                rs.stream.color,
                *self.requested_size,
                rs.format.bgr8,
                self.target_fps,
            )
            profile = pipeline.start(config)
            started = True
            if (
                (
                    profile.get_device().get_info(rs.camera_info.serial_number)
                    != self.serial
                )
                or (
                    profile.get_device().get_info(rs.camera_info.name)
                    != self.name
                )
            ):
                raise RuntimeError('实际打开的相机身份不匹配。')
            with self.cv:
                self.name = profile.get_device().get_info(rs.camera_info.name)
                self.usb = profile.get_device().get_info(
                    rs.camera_info.usb_type_descriptor
                )
            if self.camera_id == 'd405':
                for _ in range(15):
                    if stop.is_set():
                        return
                    pipeline.wait_for_frames(3000)
            last, count = time.monotonic(), 0
            while not stop.is_set():
                frames = pipeline.wait_for_frames(3000)
                if stop.is_set():
                    break
                color = frames.get_color_frame()
                if not color:
                    continue
                captured_at = time.time()
                ok, encoded = cv2.imencode(
                    '.jpg',
                    np.asanyarray(color.get_data()),
                    [cv2.IMWRITE_JPEG_QUALITY, 82],
                )
                if not ok:
                    raise RuntimeError('图像编码失败。')
                count += 1
                now = time.monotonic()
                with self.cv:
                    if now - last >= 1:
                        self.fps, count, last = count / (now - last), 0, now
                # Discard a frame delayed by encoding/scheduling; keep waiting
                # for
                # fresh device frames instead of permanently ending this
                # preview.
                if not 0 <= time.time() - captured_at <= 1:
                    continue
                self.publish(
                    encoded.tobytes(),
                    captured_at,
                    color.get_frame_number(),
                    {
                        'sensor_timestamp_ms': color.get_timestamp(),
                        'sensor_timestamp_domain': str(
                            color.get_frame_timestamp_domain()
                        ),
                    },
                    'device',
                    color.get_width(),
                    color.get_height(),
                )
        except Exception as exc:
            LOG.warning('Camera stream stopped: %s', exc)
            with self.cv:
                self.error = str(exc)
                self.state = 'error'
        finally:
            if started:
                try:
                    pipeline.stop()
                except Exception as exc:
                    LOG.warning('Camera release: %s', exc)
            if device_lock:
                device_lock.close()
            with self.cv:
                self.clients = 0
                if self.state != 'error':
                    self.state = 'idle'
                self.jpeg, self.frame_at, self.fps = None, None, 0.0
                self.cv.notify_all()

    def capture_tracker(self, first, stop):
        last, count, data = time.monotonic(), 0, first
        while not stop.is_set():
            identity = data.get('camera') or {}
            if (
                identity.get('name') != self.name
                or not identity.get('serial')
                or (self.serial and identity['serial'] != self.serial)
            ):
                raise RuntimeError('8089 相机身份不匹配；不会显示另一台设备。')
            self.serial = identity['serial']
            jpeg = base64.b64decode(data['image_base64'], validate=True)
            if (
                len(jpeg) > 4 * 1024 * 1024
                or data.get('mime_type') != 'image/jpeg'
            ):
                raise RuntimeError('8089 相机图像格式无效。')
            if data['sequence'] != self.sequence or self.jpeg is None:
                self.publish(
                    jpeg,
                    data['captured_at'],
                    data['sequence'],
                    {},
                    'tracker',
                    data['width'],
                    data['height'],
                )
                count += 1
            elif not 0 <= time.time() - data['captured_at'] <= 1:
                raise RuntimeError('8089 相机帧已过期。')
            now = time.monotonic()
            if now - last >= 1:
                with self.cv:
                    self.fps = count / (now - last)
                count, last = 0, now
            if stop.wait(1 / self.target_fps):
                break
            data = self.tracker.request('/api/agent/frame')


def monitor_status(cameras):
    rows = {key: camera.status() for key, camera in cameras.items()}
    state = next(
        (
            state
            for state in ('streaming', 'starting', 'stopping', 'error')
            if any(row['state'] == state for row in rows.values())
        ),
        'idle',
    )
    return {**rows['d435i'], 'state': state, 'cameras': rows}


class Handler(http.server.BaseHTTPRequestHandler):
    """预览服务的请求处理：页面、状态、画面流、截图和释放预览。"""

    protocol_version = 'HTTP/1.1'

    def log_message(self, fmt, *args):
        if '/api/status' not in str(args[0]):
            LOG.info('%s %s', self.client_address[0], fmt % args)

    def send_body(
        self,
        code,
        body,
        kind='application/json; charset=utf-8',
        disposition=None,
    ):
        self.send_response(code)
        self.send_header('Content-Type', kind)
        self.send_header('Content-Length', str(len(body)))
        self.send_header('Cache-Control', 'no-store')
        self.send_header('X-Content-Type-Options', 'nosniff')
        if disposition:
            self.send_header('Content-Disposition', disposition)
        self.end_headers()
        self.wfile.write(body)

    def do_GET(self):
        path = urllib.parse.urlsplit(self.path).path
        if path == '/':
            return self.send_body(
                200,
                (WEB / 'index.html').read_bytes(),
                'text/html; charset=utf-8',
            )
        if path == '/app.js':
            return self.send_body(
                200,
                (WEB / 'app.js').read_bytes(),
                'text/javascript; charset=utf-8',
            )
        if path == '/api/status':
            return self.send_body(
                200, json.dumps(monitor_status(self.server.cameras)).encode()
            )
        routes = {
            '/stream': ('d435i', 'stream'),
            '/snapshot.jpg': ('d435i', 'snapshot'),
        }
        for key in self.server.cameras:
            routes.update(
                {
                    f'/stream/{key}': (key, 'stream'),
                    f'/snapshot/{key}.jpg': (key, 'snapshot'),
                    f'/api/cameras/{key}/status': (key, 'status'),
                    f'/api/cameras/{key}/frame': (key, 'frame'),
                }
            )
        if path not in routes:
            return self.send_body(404, b'{"error":"Not found"}')
        key, operation = routes[path]
        camera = self.server.cameras[key]
        if operation == 'status':
            return self.send_body(200, json.dumps(camera.status()).encode())
        if operation == 'frame':
            try:
                return self.send_body(
                    200, json.dumps(camera.fresh_frame()).encode()
                )
            except RuntimeError as exc:
                return self.send_body(
                    409, json.dumps({'error': str(exc)}).encode()
                )
        if operation == 'snapshot':
            with camera.cv:
                jpeg = (
                    camera.jpeg
                    if (
                        camera.state == 'streaming'
                        and camera.frame_at
                        and 0 <= time.time() - camera.frame_at <= 1
                    )
                    else None
                )
            return (
                self.send_body(
                    200,
                    jpeg,
                    'image/jpeg',
                    f'attachment; filename="{key}-snapshot.jpg"',
                )
                if jpeg
                else self.send_body(409, b'{"error":"Start preview first"}')
            )
        if self.headers.get('Sec-Fetch-Site') == 'cross-site':
            return self.send_body(
                403, b'{"error":"Cross-site streaming denied"}'
            )
        try:
            epoch = camera.subscribe()
        except RuntimeError as exc:
            return self.send_body(409, json.dumps({'error': str(exc)}).encode())
        try:
            self.connection.settimeout(5)
            self.send_response(200)
            self.send_header(
                'Content-Type', 'multipart/x-mixed-replace; boundary=frame'
            )
            self.send_header('Cache-Control', 'no-store')
            self.send_header('Connection', 'close')
            self.end_headers()
            previous = -1
            while True:
                with camera.cv:
                    camera.cv.wait_for(
                        lambda: (
                            camera.jpeg is not None
                            and camera.sequence != previous
                        ) or camera.stop_event.is_set() or camera.state in (
                            'error', 'idle'
                        ),
                        timeout=4,
                    )
                    if (
                        epoch != camera.epoch
                        or camera.stop_event.is_set()
                        or camera.state in ('error', 'idle')
                    ):
                        break
                    jpeg, sequence = camera.jpeg, camera.sequence
                if not jpeg or sequence == previous:
                    continue
                self.wfile.write(
                    b'--frame\r\nContent-Type: image/jpeg\r\nContent-Length: '
                    + str(len(jpeg)).encode()
                    + b'\r\nX-Sequence: '
                    + str(sequence).encode()
                    + b'\r\n\r\n'
                    + jpeg
                    + b'\r\n'
                )
                self.wfile.flush()
                previous = sequence
        except (
            BrokenPipeError, ConnectionResetError, TimeoutError, socket.timeout
        ):
            pass
        finally:
            camera.unsubscribe(epoch)
            self.close_connection = True

    def do_POST(self) -> None:
        """处理释放预览、让出相机和归还相机三类请求；不控制机械臂。

        `/api/pause` 释放全部预览。`/api/cameras/<键>/pause` 释放一路，
        请求体里带 `{"hold_s": 秒数}` 时同时把相机让给采集程序。
        `/api/cameras/<键>/resume` 提前归还。
        """
        cameras = self.server.cameras
        routes = {'/api/pause': ('pause', list(cameras))}
        for key in cameras:
            routes[f'/api/cameras/{key}/pause'] = ('pause', [key])
            routes[f'/api/cameras/{key}/resume'] = ('resume', [key])
        if self.path not in routes:
            return self.send_body(404, b'{"error":"Not found"}')
        origin = self.headers.get('Origin')
        if origin and origin != 'http://' + self.headers.get('Host', ''):
            return self.send_body(403, b'{"error":"Origin mismatch"}')
        operation, keys = routes[self.path]
        hold_s = 0.
        try:
            size = int(self.headers.get('Content-Length') or 0)
            if not 0 <= size <= 200:
                raise ValueError('请求体过大')
            body = json.loads(self.rfile.read(size) or b'{}')
            if not isinstance(body, dict) or set(body) - {'hold_s'}:
                raise ValueError('只接受 hold_s')
            hold_s = body.get('hold_s', 0)
            if type(hold_s) not in (int, float) or not 0 <= hold_s:
                raise ValueError('hold_s 必须是非负数')
        except ValueError as exc:
            return self.send_body(
                400, json.dumps({'error': str(exc)}).encode()
            )
        for key in keys:
            if operation == 'resume':
                cameras[key].hold(0)
                continue
            # 先挡住新连接再停，避免页面在两步之间重连进来。
            if hold_s and len(keys) == 1:
                cameras[key].hold(hold_s)
                LOG.info('%s 让给采集程序 %.0f 秒', key, hold_s)
            cameras[key].stop()
        self.send_body(200, json.dumps(monitor_status(cameras)).encode())


def main() -> None:
    """启动相机预览服务；只读取相机，不控制机械臂。"""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--host', default='0.0.0.0')
    parser.add_argument('--port', type=int, default=8088)
    parser.add_argument('--width', type=int, default=1280)
    parser.add_argument('--height', type=int, default=720)
    parser.add_argument('--fps', type=int, default=15)
    parser.add_argument('--eye-in-hand-serial')
    parser.add_argument('--fixed-serial')
    parser.add_argument(
        '--arm2-serial',
        help=(
            '2 臂手上 D435i 的序列号；缺省读取配置里的 '
            'arm2_eye_in_hand_serial，都没有时不提供这一路'
        ),
    )
    parser.add_argument(
        '--tracker-url',
        help=(
            '8089 HTTP origin; share its raw D435i frames without opening that '
            'device'
        ),
    )
    parser.add_argument(
        '--token-file',
        type=pathlib.Path,
        default=(
            pathlib.Path.home() / '.local/share/saydo-mcp/robot-backend-token'
        ),
    )
    args = parser.parse_args()
    if min(args.width, args.height, args.fps) <= 0:
        parser.error('width、height、fps 必须为正整数')
    config_path = PROJECT / 'config/cameras/global.json'
    settings = (
        json.loads(config_path.read_text()) if config_path.exists() else {}
    )
    eye_serial = args.eye_in_hand_serial or settings.get('eye_in_hand_serial')
    fixed_serial = (
        args.fixed_serial
        or (
            settings.get('serial')
            if settings.get('camera_name') == 'RealSense D405'
            else None
        )
    )
    arm2_serial = args.arm2_serial or settings.get('arm2_eye_in_hand_serial')
    known = [s for s in (eye_serial, fixed_serial, arm2_serial) if s]
    if len(set(known)) != len(known):
        parser.error('各路相机的序列号不能相同')
    if arm2_serial and not eye_serial:
        # 两台 D435i 同时连着时，1 臂那一路也必须写明序列号，否则它在
        # 8089 停止时会因为“找不到唯一的 D435I”而无法直接采集。
        parser.error('提供 2 臂相机时，必须同时指定 1 臂相机的序列号')
    tracker = None
    if args.tracker_url:
        from saydo.agent import client
        tracker = client.RobotBackend(
            args.tracker_url, args.token_file, timeout=2
        )
    logging.basicConfig(
        level=logging.INFO, format='%(asctime)s %(levelname)s %(message)s'
    )
    server = http.server.ThreadingHTTPServer((args.host, args.port), Handler)
    server.daemon_threads = True
    size = (args.width, args.height, args.fps)
    server.cameras = {
        'd435i': Camera(*size, 'd435i', eye_serial, tracker, arm='arm1'),
        'd405': Camera(*size, 'd405', fixed_serial),
    }
    if arm2_serial:
        # 2 臂的相机不经 8089：8089 目前只读 1 臂的相机。
        server.cameras[ARM2_CAMERA_KEY] = Camera(
            *size, ARM2_CAMERA_KEY, arm2_serial, arm='arm2'
        )

    def shutdown(*_):
        threading.Thread(target=server.shutdown, daemon=True).start()

    signal.signal(signal.SIGTERM, shutdown)
    signal.signal(signal.SIGINT, shutdown)
    LOG.info(
        'Monitor listening on %s:%s; camera stays idle until /stream is opened',
        args.host,
        args.port,
    )
    LOG.info(
        '提供的相机：%s',
        '，'.join(
            f'{key}={camera.serial or "自动"}'
            for key, camera in server.cameras.items()
        ),
    )
    try:
        server.serve_forever()
    finally:
        for camera in server.cameras.values():
            camera.stop()
        server.server_close()


if __name__ == '__main__':
    main()
