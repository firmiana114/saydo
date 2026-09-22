#!/usr/bin/env python3
"""Read-only, on-demand D435i MJPEG monitor. No robot interfaces."""
import argparse
import json
import logging
import os
from pathlib import Path
import signal
import socket
import subprocess
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

import cv2
import numpy as np
import pyrealsense2 as rs

ROOT = Path(__file__).resolve().parent
LOG = logging.getLogger('camera_monitor')


def camera_users():
    nodes = []
    for item in Path('/sys/class/video4linux').glob('video*'):
        for parent in item.resolve().parents:
            vendor = parent / 'idVendor'
            product = parent / 'idProduct'
            if vendor.exists() and product.exists():
                if vendor.read_text().strip() == '8086' and product.read_text().strip() == '0b3a':
                    nodes.append('/dev/' + item.name)
                break
    if not nodes:
        return []
    result = subprocess.run(['fuser', *nodes], capture_output=True, text=True, timeout=3)
    if result.returncode not in (0, 1):
        raise RuntimeError('无法检查相机占用状态，请检查 fuser。')
    return sorted({int(p) for p in result.stdout.split() if p.isdigit() and int(p) != os.getpid()})


class Camera:
    def __init__(self, width, height, fps):
        self.width, self.height, self.target_fps = width, height, fps
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
        self.name = 'Intel RealSense D435i'
        self.usb = None

    def status(self):
        with self.cv:
            return dict(state=self.state, error=self.error, camera=self.name, usb=self.usb,
                        width=self.width, height=self.height, target_fps=self.target_fps,
                        fps=round(self.fps, 1), clients=self.clients, sequence=self.sequence,
                        frame_at=self.frame_at,
                        frame_age_ms=round((time.time()-self.frame_at)*1000) if self.frame_at else None)

    def subscribe(self):
        with self.cv:
            if self.thread and self.thread.is_alive() and self.stop_event.is_set():
                raise RuntimeError('相机正在释放，请稍后重试。')
            if self.clients >= 4:
                raise RuntimeError('同时预览上限为 4 个连接。')
            self.clients += 1
            if not self.thread or not self.thread.is_alive():
                self.stop_event = threading.Event()
                self.epoch += 1
                self.state, self.error, self.jpeg = 'starting', None, None
                self.frame_at, self.fps = None, 0.0
                self.thread = threading.Thread(target=self.capture, args=(self.epoch, self.stop_event), daemon=True)
                self.thread.start()
            return self.epoch

    def unsubscribe(self, epoch):
        with self.cv:
            if epoch != self.epoch:
                return
            self.clients = max(0, self.clients-1)
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
        pipeline = rs.pipeline()
        started = False
        try:
            users = camera_users()
            if users:
                raise RuntimeError('相机已被其他程序占用；请先结束采集，再开始预览。')
            devices = rs.context().query_devices()
            selected = next((d for d in devices if '435' in d.get_info(rs.camera_info.name)), None)
            if selected is None:
                raise RuntimeError('未发现 D435i，请检查 USB 数据线。')
            config = rs.config()
            config.enable_device(selected.get_info(rs.camera_info.serial_number))
            config.enable_stream(rs.stream.color, self.width, self.height, rs.format.bgr8, self.target_fps)
            profile = pipeline.start(config)
            started = True
            with self.cv:
                self.name = profile.get_device().get_info(rs.camera_info.name)
                self.usb = profile.get_device().get_info(rs.camera_info.usb_type_descriptor)
            last, count = time.monotonic(), 0
            while not stop.is_set():
                frames = pipeline.wait_for_frames(3000)
                if stop.is_set():
                    break
                color = frames.get_color_frame()
                if not color:
                    continue
                ok, encoded = cv2.imencode('.jpg', np.asanyarray(color.get_data()), [cv2.IMWRITE_JPEG_QUALITY, 82])
                if not ok:
                    raise RuntimeError('图像编码失败。')
                count += 1
                now = time.monotonic()
                with self.cv:
                    if now-last >= 1:
                        self.fps, count, last = count/(now-last), 0, now
                    self.jpeg = encoded.tobytes()
                    self.sequence += 1
                    self.frame_at = time.time()
                    self.state = 'streaming'
                    self.cv.notify_all()
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
            with self.cv:
                self.clients = 0
                if self.state != 'error':
                    self.state = 'idle'
                self.jpeg, self.frame_at, self.fps = None, None, 0.0
                self.cv.notify_all()


class Handler(BaseHTTPRequestHandler):
    protocol_version = 'HTTP/1.1'

    def log_message(self, fmt, *args):
        if '/api/status' not in str(args[0]):
            LOG.info('%s %s', self.client_address[0], fmt % args)

    def send_body(self, code, body, kind='application/json; charset=utf-8', disposition=None):
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
        path = self.path.split('?', 1)[0]
        if path == '/':
            return self.send_body(200, (ROOT/'index.html').read_bytes(), 'text/html; charset=utf-8')
        if path == '/api/status':
            return self.send_body(200, json.dumps(self.server.camera.status()).encode())
        if path == '/snapshot.jpg':
            with self.server.camera.cv:
                jpeg = self.server.camera.jpeg if self.server.camera.state == 'streaming' else None
            return self.send_body(200, jpeg, 'image/jpeg', 'attachment; filename="d435i-snapshot.jpg"') if jpeg else self.send_body(409, b'{"error":"Start preview first"}')
        if path != '/stream':
            return self.send_body(404, b'{"error":"Not found"}')
        if self.headers.get('Sec-Fetch-Site') == 'cross-site':
            return self.send_body(403, b'{"error":"Cross-site streaming denied"}')
        camera = self.server.camera
        try:
            epoch = camera.subscribe()
        except RuntimeError as exc:
            return self.send_body(409, json.dumps({'error': str(exc)}).encode())
        try:
            self.connection.settimeout(5)
            self.send_response(200)
            self.send_header('Content-Type', 'multipart/x-mixed-replace; boundary=frame')
            self.send_header('Cache-Control', 'no-store')
            self.send_header('Connection', 'close')
            self.end_headers()
            previous = -1
            while True:
                with camera.cv:
                    camera.cv.wait_for(lambda: (camera.jpeg is not None and camera.sequence != previous) or camera.stop_event.is_set() or camera.state in ('error','idle'), timeout=4)
                    if epoch != camera.epoch or camera.stop_event.is_set() or camera.state in ('error','idle'):
                        break
                    jpeg, sequence = camera.jpeg, camera.sequence
                if not jpeg or sequence == previous:
                    continue
                self.wfile.write(b'--frame\r\nContent-Type: image/jpeg\r\nContent-Length: ' + str(len(jpeg)).encode() + b'\r\nX-Sequence: ' + str(sequence).encode() + b'\r\n\r\n' + jpeg + b'\r\n')
                self.wfile.flush()
                previous = sequence
        except (BrokenPipeError, ConnectionResetError, TimeoutError, socket.timeout):
            pass
        finally:
            camera.unsubscribe(epoch)
            self.close_connection = True

    def do_POST(self):
        if self.path != '/api/pause':
            return self.send_body(404, b'{"error":"Not found"}')
        origin = self.headers.get('Origin')
        if origin and origin != 'http://' + self.headers.get('Host', ''):
            return self.send_body(403, b'{"error":"Origin mismatch"}')
        self.send_body(200, json.dumps(self.server.camera.stop()).encode())


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--host', default='0.0.0.0')
    parser.add_argument('--port', type=int, default=8088)
    parser.add_argument('--width', type=int, default=1280)
    parser.add_argument('--height', type=int, default=720)
    parser.add_argument('--fps', type=int, default=15)
    args = parser.parse_args()
    logging.basicConfig(level=logging.INFO, format='%(asctime)s %(levelname)s %(message)s')
    server = ThreadingHTTPServer((args.host, args.port), Handler)
    server.daemon_threads = True
    server.camera = Camera(args.width, args.height, args.fps)
    def shutdown(*_):
        threading.Thread(target=server.shutdown, daemon=True).start()
    signal.signal(signal.SIGTERM, shutdown)
    signal.signal(signal.SIGINT, shutdown)
    LOG.info('Monitor listening on %s:%s; camera stays idle until /stream is opened', args.host, args.port)
    try:
        server.serve_forever()
    finally:
        server.camera.stop()
        server.server_close()


if __name__ == '__main__':
    main()
