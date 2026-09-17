#!/usr/bin/env python3
"""D435i/web adapter on official FoundationPose register/track_one; no robot API."""
import argparse
import datetime
import fcntl
import hashlib
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
import json
import logging
from pathlib import Path
import signal
import socket
import sys
import threading
import time
import urllib.request

import cv2
import numpy as np

from live_checks import checked_box, pose_health, bottle_candidates

ROOT = Path(__file__).resolve().parents[2]


class Tracker:
    def __init__(self, args):
        self.args = args
        self.lock = threading.RLock()
        self.stop = threading.Event()
        self.latest = self.frozen = self.command = None
        self.jpeg = None
        self.generation = 0
        self.state = {'state': 'loading', 'error': None, 'model': str(args.mesh), 'sequence': 0,
                      'tracked_frames': 0, 'registered_frames': 0, 'camera_T_object': None,
                      'pose_accepted_for_robot_control': False}
        self.output = ROOT / 'work/foundationpose_live' / datetime.datetime.now().strftime('%Y%m%d_%H%M%S_%f')
        self.output.mkdir(parents=True)
        self.state['output'] = str(self.output)
        (self.output / 'session.json').write_text(json.dumps({'mesh': str(args.mesh), 'mesh_sha256': hashlib.sha256(args.mesh.read_bytes()).hexdigest(),
             'camera_serial': args.serial, 'native_color': [1280, 720, 15], 'native_depth': [640, 360, 15],
             'inference_resolution': [640, 360], 'algorithm': 'official FoundationPose register(iteration=5), track_one(iteration=2)',
             'robot_commands': False, 'pose_frame': 'color optical camera; translation metres',
             'quality_gate': 'heuristic only; transparent surface depth and model dimensions unvalidated'}, indent=2))

    def capture(self):
        sys.path.insert(0, str(ROOT / 'work/foundationpose_realsense_py311'))
        import pyrealsense2 as rs
        pipe = rs.pipeline()
        started = False
        try:
            cfg = rs.config()
            cfg.enable_device(self.args.serial)
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
                # Consume the newest complete frameset rather than a queued older one.
                for _ in range(8):
                    newer = pipe.poll_for_frames()
                    if not newer:
                        break
                    frames = newer
                received_at = time.time()
                native_color, native_depth = frames.get_color_frame(), frames.get_depth_frame()
                if native_color.get_frame_number() == last_camera_number:
                    continue
                if native_color.get_frame_timestamp_domain() != native_depth.get_frame_timestamp_domain():
                    raise ValueError('RGB/depth timestamp domains differ')
                if abs(native_color.get_timestamp() - native_depth.get_timestamp()) > 100:
                    continue
                timestamp = native_color.get_timestamp() / 1000
                clock_is_wall = native_color.get_frame_timestamp_domain() in (rs.timestamp_domain.global_time, rs.timestamp_domain.system_time)
                captured_at = timestamp if clock_is_wall and 0 <= received_at - timestamp < 10 else received_at
                align_started = time.perf_counter()
                aligned = align.process(frames)
                color, depth = aligned.get_color_frame(), aligned.get_depth_frame()
                intr = color.profile.as_video_stream_profile().intrinsics
                if any(abs(v) > 1e-12 for v in intr.coeffs):
                    raise ValueError('Nonzero color distortion requires rectification')
                K = np.array([[intr.fx / 2, 0, intr.ppx / 2], [0, intr.fy / 2, intr.ppy / 2], [0, 0, 1.]])
                bgr = cv2.resize(np.asanyarray(color.get_data()), (640, 360), interpolation=cv2.INTER_AREA)
                depth_m = cv2.resize(np.asanyarray(depth.get_data()).astype(np.float32) * scale, (640, 360), interpolation=cv2.INTER_NEAREST)
                sequence += 1
                last_camera_number = native_color.get_frame_number()
                frame = {'bgr': bgr, 'depth': depth_m, 'K': K, 'sequence': sequence,
                         'camera_frame_number': int(native_color.get_frame_number()), 'time': captured_at,
                         'received_at': received_at, 'sensor_age_at_receive_s': received_at - captured_at,
                         'alignment_seconds': time.perf_counter() - align_started}
                with self.lock:
                    self.latest = frame
                    self.state['sequence'] = sequence
                    self.state['camera_frame_at'] = frame['time']
                    self.state['sensor_age_at_receive_s'] = frame['sensor_age_at_receive_s']
                    self.state['alignment_seconds'] = frame['alignment_seconds']
        except Exception as error:
            with self.lock:
                self.state.update(state='camera_error', error=str(error), camera_T_object=None)
            self.stop.set()
        finally:
            if started:
                pipe.stop()

    def freeze(self):
        with self.lock:
            if self.latest is None or time.time() - self.latest['time'] > 2:
                raise ValueError('还没有新鲜的相机画面')
            if not self.state.get('model_ready'):
                raise ValueError('模型仍在加载')
            self.generation += 1
            self.frozen = self.latest
            self.command = None
            self.state.update(state='selecting', error=None, camera_T_object=None, selection_id=self.generation)
            self.jpeg = None
            return {'selection_id': self.generation, 'width': 640, 'height': 360}

    def initialize(self, value):
        box = checked_box(value['box'], 640, 360)
        with self.lock:
            if self.state['state'] != 'selecting' or value['selection_id'] != self.generation:
                raise ValueError('框选画面已失效，请重新选择目标')
            self.command = (self.generation, self.frozen, box)
            self.state.update(state='initializing', error=None, camera_T_object=None)

    def automatic(self):
        self.freeze()
        with self.lock:
            self.command = (self.generation, self.frozen, None)
            self.state.update(state='detecting', bottle_candidates=[], error=None)

    def choose(self, value):
        with self.lock:
            if self.state['state'] != 'choose_candidate' or value['selection_id'] != self.generation:
                raise ValueError('候选画面已失效，请重新识别')
            index = value['index']
            candidates = self.state['bottle_candidates']
            if not isinstance(index, int) or not 0 <= index < len(candidates):
                raise ValueError('候选编号无效')
            self.command = (self.generation, self.frozen, candidates[index]['box'])
            self.state.update(state='initializing', error=None, camera_T_object=None)

    def pause(self):
        with self.lock:
            self.generation += 1
            self.command = None
            self.frozen = None
            self.jpeg = None
            self.state.update(state='paused', camera_T_object=None)

    def snapshot(self):
        with self.lock:
            result = dict(self.state)
            now = time.time()
            result['camera_age_s'] = now - result['camera_frame_at'] if 'camera_frame_at' in result else None
            result['pose_age_s'] = now - result['pose_frame_at'] if 'pose_frame_at' in result else None
            if result['state'] != 'tracking' or result['pose_age_s'] is None or result['pose_age_s'] > 2 or not result.get('quality', {}).get('good'):
                result['camera_T_object'] = None
            return result

    def image(self):
        with self.lock:
            state = self.state['state']
            frame = self.frozen if state in ('selecting', 'initializing', 'detecting', 'choose_candidate') else self.latest
            cached = self.jpeg if state in ('tracking', 'lost') else None
            age = time.time() - self.state.get('pose_frame_at', time.time())
            candidates = list(self.state.get('bottle_candidates', [])) if state == 'choose_candidate' else []
        if cached is not None and age <= 2:
            return cached
        if frame is None:
            return None
        bgr = frame['bgr'].copy()
        for index, candidate in enumerate(candidates):
            x0, y0, x1, y1 = candidate['box']
            cv2.rectangle(bgr, (x0, y0), (x1, y1), (0, 220, 255), 2)
            cv2.putText(bgr, str(index + 1), (x0, max(35, y0)), 0, .9, (0, 220, 255), 2)
        label = 'STALE / NEED REINITIALIZATION' if state == 'tracking' and age > 2 else state.upper()
        cv2.putText(bgr, label, (10, 26), 0, .65, (0, 170, 255), 2)
        return cv2.imencode('.jpg', bgr, [cv2.IMWRITE_JPEG_QUALITY, 75])[1].tobytes()

    def inference(self):
        try:
            import torch
            torch.set_num_threads(2)
            import trimesh
            import nvdiffrast.torch as dr
            from estimater import FoundationPose
            from learning.training.predict_score import ScorePredictor
            from learning.training.predict_pose_refine import PoseRefinePredictor
            from segment_anything import SamPredictor, sam_model_registry
            from torchvision.models.detection import ssdlite320_mobilenet_v3_large, SSDLite320_MobileNet_V3_Large_Weights
            from Utils import draw_posed_3d_box, nvdiffrast_render
            mesh = trimesh.load(self.args.mesh, force='mesh')
            if not np.isfinite(mesh.vertices).all() or not (.01 < mesh.extents.max() < 1):
                raise ValueError('Model must be a finite metre-scale object mesh')
            origin, extents = trimesh.bounds.oriented_bounds(mesh)
            bbox = np.stack([-extents / 2, extents / 2])
            context = dr.RasterizeCudaContext()
            estimator = FoundationPose(model_pts=mesh.vertices, model_normals=mesh.vertex_normals, mesh=mesh,
                scorer=ScorePredictor(), refiner=PoseRefinePredictor(), debug_dir=str(self.output), debug=0, glctx=context)
            sam = SamPredictor(sam_model_registry['vit_b'](checkpoint=str(ROOT / 'work/foundationpose_setup/sam_vit_b_01ec64.pth')).to('cuda'))
            torch.hub.set_dir(str(ROOT / 'work/foundationpose_setup/torch_hub'))
            detector_weights = SSDLite320_MobileNet_V3_Large_Weights.COCO_V1
            detector = ssdlite320_mobilenet_v3_large(weights=detector_weights).eval().cuda()
            bottle_label = detector_weights.meta['categories'].index('bottle')
            logging.getLogger().setLevel(logging.WARNING)
            with self.lock:
                if self.stop.is_set():
                    return
                self.state.update(state='ready', model_ready=True, model_extents_m=mesh.extents.tolist())
            previous = None
            last_sequence = -1
            failures = 0
            auto_attempted = False
            with (self.output / 'poses.jsonl').open('a', buffering=1) as records:
                while not self.stop.is_set():
                    with self.lock:
                        command = self.command
                        self.command = None
                        active = self.state['state'] == 'tracking'
                        generation = self.generation
                        frame = self.latest
                        startup_ready = self.state['state'] == 'ready'
                    if self.args.auto_start and not auto_attempted and frame is not None and startup_ready:
                        self.automatic()
                        auto_attempted = True
                        continue
                    if command:
                        generation, frame, box = command
                    elif not active or frame is None or frame['sequence'] == last_sequence:
                        self.stop.wait(.02)
                        continue
                    elif time.time() - frame['time'] > 2:
                        with self.lock:
                            self.state.update(state='lost', error='相机画面过期，请重新初始化', camera_T_object=None)
                        continue
                    started = time.perf_counter()
                    rgb = cv2.cvtColor(frame['bgr'], cv2.COLOR_BGR2RGB)
                    mask = None
                    try:
                        with torch.inference_mode():
                            if command:
                                if box is None:
                                    prediction = detector([torch.from_numpy(rgb).permute(2, 0, 1).float().cuda() / 255])[0]
                                    candidates = bottle_candidates(prediction['boxes'].detach().cpu().numpy(),
                                        prediction['labels'].detach().cpu().numpy(), prediction['scores'].detach().cpu().numpy(), bottle_label)
                                    if not candidates:
                                        raise ValueError('没有检测到可靠的水瓶；请让整瓶入镜后重试，或使用手动框选')
                                    with self.lock:
                                        if generation != self.generation:
                                            continue
                                        self.state['bottle_candidates'] = candidates
                                        self.state['detection_method'] = 'Torchvision SSDLite COCO bottle -> SAM -> FoundationPose'
                                        if len(candidates) > 1:
                                            self.state.update(state='choose_candidate', error=None)
                                            continue
                                        self.state['state'] = 'initializing'
                                    box = candidates[0]['box']
                                sam.set_image(rgb)
                                masks, scores, _ = sam.predict(box=np.asarray(box), multimask_output=True)
                                mask = masks[int(np.argmax(scores))]
                                if mask.sum() < 100 or ((frame['depth'] >= .1) & (frame['depth'] <= 2) & mask).sum() < 100:
                                    raise ValueError('目标有效深度不足，请调整距离或视角后重新框选')
                                estimator.pose_last = None
                                pose = estimator.register(K=frame['K'], rgb=rgb, depth=frame['depth'].copy(), ob_mask=mask, iteration=5)
                                if estimator.pose_last is None:
                                    raise ValueError('FoundationPose 首帧初始化没有产生有效状态')
                                previous = None
                                stamp = self.output / f'initialization_{generation}'
                                stamp.mkdir()
                                cv2.imwrite(str(stamp / 'color.png'), frame['bgr'])
                                cv2.imwrite(str(stamp / 'mask.png'), mask.astype(np.uint8) * 255)
                                np.save(stamp / 'depth_m.npy', frame['depth'])
                                np.savetxt(stamp / 'K.txt', frame['K'])
                            else:
                                pose = estimator.track_one(rgb=rgb, depth=frame['depth'].copy(), K=frame['K'], iteration=2)
                            if not np.isfinite(pose).all():
                                raise ValueError('FoundationPose returned nonfinite pose')
                            _, rendered, _ = nvdiffrast_render(K=frame['K'], H=360, W=640, mesh=mesh,
                                ob_in_cams=torch.as_tensor(pose[None], device='cuda', dtype=torch.float32), glctx=context)
                            rendered = rendered[0].detach().cpu().numpy()
                            quality = pose_health(pose, rendered, frame['depth'], previous, mask)
                        duration = time.perf_counter() - started
                        failures = 0 if quality['good'] else failures + 1
                        lost = not quality['good'] and (bool(command) or failures >= 3)
                        overlay = draw_posed_3d_box(frame['K'], img=rgb.copy(), ob_in_cam=pose @ np.linalg.inv(origin), bbox=bbox)
                        visible = rendered > 0
                        overlay[visible] = (.75 * overlay[visible] + .25 * np.array([0, 200, 255])).astype(np.uint8)
                        bgr = cv2.cvtColor(overlay, cv2.COLOR_RGB2BGR)
                        label = 'LOST - RESELECT' if lost else ('TRACKING' if quality['good'] else 'UNCERTAIN')
                        cv2.putText(bgr, f'{label} | {1/max(duration,.001):.1f} inference FPS', (8, 25), 0, .6, (0, 255, 0) if quality['good'] else (0, 120, 255), 2)
                        with self.lock:
                            if generation != self.generation:
                                continue
                            self.state.update(state='lost' if lost else 'tracking', camera_T_object=pose.tolist() if quality['good'] else None,
                                quality=quality, inference_seconds=duration, pose_frame_at=frame['time'],
                                pose_sequence=frame['sequence'], error=None)
                            self.state['registered_frames' if command else 'tracked_frames'] += 1
                            self.frozen = None
                            self.jpeg = cv2.imencode('.jpg', bgr, [cv2.IMWRITE_JPEG_QUALITY, 75])[1].tobytes()
                            record = {**self.snapshot(), 'camera_frame_number': frame['camera_frame_number'], 'processed_at': time.time()}
                            status_temp = self.output / 'status.tmp'
                            status_temp.write_text(json.dumps(record, indent=2))
                            status_temp.replace(self.output / 'status.json')
                            (self.output / 'latest.jpg').write_bytes(self.jpeg)
                        records.write(json.dumps(record) + '\n')
                        previous = np.asarray(pose).copy()
                        last_sequence = frame['sequence']
                    except Exception as error:
                        with self.lock:
                            if generation == self.generation:
                                self.state.update(state='lost', error=str(error), camera_T_object=None)
                        print('Frame failure:', repr(error), flush=True)
        except Exception as error:
            with self.lock:
                self.state.update(state='model_error', error=str(error), camera_T_object=None)
            print('Model failure:', repr(error), flush=True)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--mesh', type=Path, default=ROOT / 'work/object_models/ob_yibao_350ml/model/model.obj')
    parser.add_argument('--serial', default='135122077750')
    parser.add_argument('--port', type=int, default=8089)
    parser.add_argument('--host', default='127.0.0.1', help='HTTP bind address; use the MSI LAN IP for direct LAN access')
    parser.add_argument('--auto-start', action='store_true', help='Detect a bottle and initialize once after camera/model startup')
    args = parser.parse_args()
    cv2.setNumThreads(1)
    runtime = ROOT / 'work/foundationpose_runtime'
    sys.path[:0] = [str(runtime), str(runtime / 'bundlesdf'), str(runtime / 'mycpp/build')]
    base = ROOT / 'work/foundationpose_live'
    base.mkdir(exist_ok=True)
    camera_lock = (base / 'camera.lock').open('a')
    fcntl.flock(camera_lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
    # Release the existing preview through its documented API; never reset hardware.
    try:
        with urllib.request.urlopen('http://127.0.0.1:8088/api/status', timeout=2) as response:
            status = json.load(response)
        if status['state'] in ('streaming', 'starting', 'stopping'):
            request = urllib.request.Request('http://127.0.0.1:8088/api/pause', data=b'', method='POST')
            with urllib.request.urlopen(request, timeout=8):
                pass
    except OSError:
        pass
    tracker = Tracker(args)

    class Handler(BaseHTTPRequestHandler):
        def log_message(self, *unused):
            pass

        def reply(self, status, body, mime='application/json'):
            if not isinstance(body, bytes):
                body = json.dumps(body, ensure_ascii=False).encode()
            self.send_response(status)
            self.send_header('Content-Type', mime)
            self.send_header('Cache-Control', 'no-store')
            self.send_header('Content-Length', str(len(body)))
            self.end_headers()
            try:
                self.wfile.write(body)
            except (BrokenPipeError, ConnectionResetError):
                pass

        def do_GET(self):
            path = self.path.split('?')[0]
            if path == '/':
                self.reply(200, Path(__file__).with_name('live_tracker.html').read_bytes(), 'text/html; charset=utf-8')
            elif path == '/api/status':
                self.reply(200, tracker.snapshot())
            elif path == '/frame.jpg':
                frame = tracker.image()
                self.reply(200, frame, 'image/jpeg') if frame else self.reply(503, {'error': '等待相机画面'})
            elif path == '/stream.mjpg':
                self.connection.settimeout(3)
                self.connection.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
                self.connection.setsockopt(socket.SOL_SOCKET, socket.SO_SNDBUF, 16384)
                self.send_response(200)
                self.send_header('Content-Type', 'multipart/x-mixed-replace; boundary=frame')
                self.send_header('Cache-Control', 'no-store')
                self.end_headers()
                last_key = None
                try:
                    while not tracker.stop.is_set():
                        with tracker.lock:
                            mode = tracker.state['state']
                            cached = mode in ('tracking', 'lost') and tracker.jpeg is not None and time.time() - tracker.state.get('pose_frame_at', 0) <= 2
                            frame = tracker.frozen if mode in ('selecting', 'initializing', 'detecting', 'choose_candidate') else tracker.latest
                            key = (tracker.generation, mode, tracker.state.get('pose_sequence') if cached else (frame['sequence'] if frame else -1))
                        if key == last_key:
                            tracker.stop.wait(.015)
                            continue
                        jpeg = tracker.image()
                        if jpeg is None:
                            tracker.stop.wait(.05)
                            continue
                        self.wfile.write(b'--frame\r\nContent-Type: image/jpeg\r\nContent-Length: ' + str(len(jpeg)).encode() + b'\r\n\r\n' + jpeg + b'\r\n')
                        self.wfile.flush()
                        last_key = key
                        tracker.stop.wait(.04)
                except (BrokenPipeError, ConnectionResetError, TimeoutError, OSError):
                    pass
            else:
                self.reply(404, {'error': 'not found'})

        def do_POST(self):
            origin = self.headers.get('Origin')
            if origin and origin != 'http://' + self.headers.get('Host', ''):
                self.reply(403, {'error': 'origin mismatch'})
                return
            try:
                size = int(self.headers.get('Content-Length', 0))
                if not 0 <= size <= 4096:
                    raise ValueError('Invalid request size')
                value = json.loads(self.rfile.read(size) or b'{}')
                if self.path == '/api/freeze':
                    result = tracker.freeze()
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
                elif self.path == '/api/stop':
                    tracker.stop.set()
                    result = {'stopping': True}
                    threading.Thread(target=server.shutdown, daemon=True).start()
                else:
                    self.reply(404, {'error': 'not found'})
                    return
                self.reply(200, result)
            except (ValueError, KeyError, TypeError) as error:
                self.reply(400, {'error': str(error)})

    server = ThreadingHTTPServer((args.host, args.port), Handler)
    threads = [threading.Thread(target=tracker.capture), threading.Thread(target=tracker.inference)]
    for thread in threads:
        thread.start()
    def shutdown(*unused):
        tracker.stop.set()
        threading.Thread(target=server.shutdown, daemon=True).start()
    signal.signal(signal.SIGTERM, shutdown)
    signal.signal(signal.SIGINT, shutdown)
    try:
        server.serve_forever()
    finally:
        tracker.stop.set()
        for thread in threads:
            thread.join(timeout=15)
        server.server_close()
        camera_lock.close()


if __name__ == '__main__':
    main()
