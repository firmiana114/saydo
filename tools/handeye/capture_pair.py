#!/usr/bin/env python3
"""Collect one stationary dual-camera calibration candidate. Never move the robot."""
import argparse
import base64
from concurrent.futures import ThreadPoolExecutor
import contextlib
import datetime
import fcntl
import hashlib
import json
from pathlib import Path
import sys
import threading
import time
import urllib.parse
import urllib.request
import uuid
from zoneinfo import ZoneInfo

import cv2
import numpy as np
import yaml

ROOT = Path(__file__).resolve().parents[2]
sys.path[:0] = [str(ROOT), str(Path(__file__).resolve().parent)]
from capture import Robot
from geometry import board_pose, pose_matrix, stationarity
from tools.foundationpose.camera_checks import camera_binding
from tools.robot_agent.client import NoRedirect

IDS = ('d435i', 'd405')
LIMITS = {'max_frame_age_s':.6, 'max_pair_skew_s':.2, 'max_robot_bracket_s':1.,
          'max_board_shift_px':1., 'max_reprojection_rms_px':1.,
          'joint_span_deg':.03, 'translation_span_m':.0002, 'rotation_span_deg':.15}


def atomic_json(path, value):
    temporary = path.with_suffix('.tmp')
    temporary.write_text(json.dumps(value, ensure_ascii=False, indent=2, allow_nan=False))
    temporary.replace(path)


def digest(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


class Monitor:
    def __init__(self, url):
        parsed = urllib.parse.urlsplit(url)
        if parsed.scheme != 'http' or not parsed.hostname or parsed.username or parsed.password or parsed.path not in ('','/') or parsed.query or parsed.fragment:
            raise ValueError('监控地址须为不含路径、凭据的固定HTTP地址')
        self.url = url.rstrip('/')
        self.opener = urllib.request.build_opener(urllib.request.ProxyHandler({}), NoRedirect())

    def get(self, path):
        allowed = ['/api/status'] + [f'/api/cameras/{key}/calibration-frame' for key in IDS]
        if path not in allowed:
            raise ValueError('采集只允许读取状态及标定图像')
        with self.opener.open(self.url+path, timeout=4) as response:
            raw = response.read(12*1024*1024+1)
        if len(raw) > 12*1024*1024:
            raise ValueError('相机响应超出大小限制')
        return json.loads(raw)

    def pair(self):
        with ThreadPoolExecutor(max_workers=2) as pool:
            futures = {key:pool.submit(self.get, f'/api/cameras/{key}/calibration-frame') for key in IDS}
            return {key:future.result() for key,future in futures.items()}

    def leases(self):
        return [PreviewLease(self, key) for key in IDS]


class PreviewLease:
    """Drain one temporary viewer; closing it releases only this viewer, never all previews."""
    def __init__(self, monitor, key):
        self.monitor, self.key = monitor, key
        self.response = None
        self.stop = threading.Event()
        self.ready = threading.Event()
        self.error = None
        self.thread = None

    def start(self):
        def drain():
            try:
                with self.monitor.opener.open(self.monitor.url+'/stream/'+self.key, timeout=4) as response:
                    self.response = response;self.ready.set()
                    while not self.stop.is_set() and response.read(65536):
                        pass
            except Exception as exc:
                if not self.stop.is_set():self.error = type(exc).__name__+': '+str(exc)
            finally:
                self.ready.set()
        self.thread = threading.Thread(target=drain, daemon=True);self.thread.start()
        if not self.ready.wait(5) or self.error:
            raise RuntimeError('无法建立临时预览：'+str(self.error or self.key))

    def close(self):
        self.stop.set()
        if self.response:
            with contextlib.suppress(Exception):self.response.close()
        if self.thread:self.thread.join(timeout=4)


def flange_pose(state, tool, work):
    if work.get('frame_name') != 'Base' or work.get('pose') != [0]*6:
        raise ValueError('请使用零变换Base工作坐标系；脚本不会切换坐标系')
    if not tool.get('tool_name') or not isinstance(tool.get('pose'),list):
        raise ValueError('当前工具偏移缺失，无法换算法兰位姿')
    return pose_matrix(state['pose']) @ np.linalg.inv(pose_matrix(tool['pose']))


def decode_frame(frame, key, serial):
    expected_role = 'eye_in_hand' if key == 'd435i' else 'fixed_global'
    expected_name = 'RealSense D435I' if key == 'd435i' else 'RealSense D405'
    camera = frame.get('camera') or {}
    if (frame.get('camera_serial') != serial or frame.get('camera_role') != expected_role
            or camera.get('serial') != serial or camera.get('name') != expected_name
            or frame.get('camera_name') != expected_name or camera.get('frame') != 'color_optical_frame'):
        raise ValueError(key+'相机身份或光学坐标系不匹配')
    age = time.time()-frame['captured_at']
    if not 0 <= age <= LIMITS['max_frame_age_s']:
        raise ValueError(key+'图像超过采集时效')
    matrix = np.asarray(camera.get('K'),dtype=float)
    coeffs = np.asarray(camera.get('distortion_coefficients'),dtype=float)
    if (matrix.shape != (3,3) or not np.isfinite(matrix).all() or matrix[0,0] <= 0 or matrix[1,1] <= 0
            or not np.allclose(matrix[2],[0,0,1]) or coeffs.shape != (5,) or not np.isfinite(coeffs).all()):
        raise ValueError(key+'实际图像内参或畸变参数缺失')
    if np.any(np.abs(coeffs) > 1e-12):
        raise ValueError(key+'非零畸变需要模型专属处理，不能直接当作OpenCV针孔参数')
    kinds = {'image/jpeg':('.jpg',b'\xff\xd8'),'image/png':('.png',b'\x89PNG\r\n\x1a\n')}
    kind = frame.get('mime_type')
    if kind not in kinds:raise ValueError(key+'图像编码不受支持')
    raw = base64.b64decode(frame['image_base64'],validate=True)
    suffix, magic = kinds[kind]
    if len(raw) > 8*1024*1024 or not raw.startswith(magic):raise ValueError(key+'图像内容无效')
    image = cv2.imdecode(np.frombuffer(raw,np.uint8),cv2.IMREAD_COLOR)
    if image is None:raise ValueError(key+'图像无法解码')
    size = [image.shape[1],image.shape[0]]
    if size != frame.get('image_size') or size != camera.get('image_size'):
        raise ValueError(key+'图像尺寸与内参来源不一致')
    if size != [1280,720] or camera.get('native_size') != [1280,720] or kind != 'image/png' or frame.get('lossless_source') is not True:
        raise ValueError(key+'必须使用原生1280×720、编码前PNG；请关闭8089并在8088重新开始原生预览')
    return raw, suffix, image


def analyze_board(image, camera, board, reverse=False):
    pattern = tuple(board['inner_corners'])
    found, corners = cv2.findChessboardCornersSB(cv2.cvtColor(image,cv2.COLOR_BGR2GRAY),pattern,cv2.CALIB_CB_NORMALIZE_IMAGE)
    if not found:raise ValueError('未找到全部棋盘角点：'+str(pattern))
    raw = corners.reshape(-1,2)
    ordered = raw[::-1].copy() if reverse else raw.copy()
    candidates = board_pose(ordered,np.asarray(camera['K']),np.zeros(5),*pattern,board['square_size_m'])
    if candidates[0]['reprojection_rms_px'] > LIMITS['max_reprojection_rms_px']:
        raise ValueError('棋盘重投影误差超过1像素')
    annotated = image.copy()
    cv2.drawChessboardCorners(annotated,pattern,ordered.astype(np.float32).reshape(-1,1,2),True)
    for index,label in [(0,'0 / origin'),(pattern[0]-1,'+X'),(len(ordered)-1,'last')]:
        point = tuple(np.rint(ordered[index]).astype(int))
        cv2.circle(annotated,point,5,(0,0,255),2)
        cv2.putText(annotated,label,(point[0]+7,point[1]-7),cv2.FONT_HERSHEY_SIMPLEX,.5,(0,0,255),2)
    return {'corners_px':ordered.tolist(),'detected_corners_px':raw.tolist(),
            'corner_order_reversed':reverse,'board_pose_candidates':candidates,
            'reprojection_rms_px':candidates[0]['reprojection_rms_px']}, annotated


def capture_once(args, root=ROOT, robot=None, monitor=None):
    session = Path(args.output_root)/args.session
    session.mkdir(parents=True,exist_ok=True)
    leases = []
    with (session/'.capture.lock').open('a') as lock:
        fcntl.flock(lock,fcntl.LOCK_EX | fcntl.LOCK_NB)
        stamp = datetime.datetime.now(ZoneInfo('Asia/Singapore')).strftime('%Y%m%d_%H%M%S')
        sample_id = stamp+'_'+uuid.uuid4().hex[:8]
        out = session/sample_id;out.mkdir()
        report = {'schema_version':1,'sample_id':sample_id,'status':'rejected','role':args.role,
                  'calibration_type':'fixed_camera_via_moving_eye_in_hand_camera',
                  'motion_sent':False,'calibration_solved':False,'usable_for_execution':False,
                  'frame_convention':'A_T_B maps B coordinates into A; translation metres',
                  'thresholds':LIMITS,'observations':[],'images':{},'created_at':time.time(),
                  'requires_review':['physical board dimensions','same physical board corner origin in both cameras',
                                     'planar pose ambiguity','existing D435i mounting accuracy','independent calibration validation']}
        try:
            settings_path = root/'config/cameras/global.json'
            handeye_path = root/'config/calibration/handeye.yaml'
            policy_path = root/'config/calibration/camera_reuse.yaml'
            inputs = [settings_path,handeye_path] + ([policy_path] if policy_path.exists() else [])
            hashes = {str(p.relative_to(root)):digest(p) for p in inputs}
            settings = json.loads(settings_path.read_text())
            serials = {'d435i':settings['eye_in_hand_serial'],'d405':settings['serial']}
            if settings.get('camera_name') != 'RealSense D405' or len(set(serials.values())) != 2:
                raise ValueError('本机配置必须分别绑定D435i和D405')
            handeye = yaml.safe_load(handeye_path.read_text())
            policy = yaml.safe_load(policy_path.read_text()) if policy_path.exists() else {}
            if handeye.get('acceptance_status') != 'user_accepted_usable':raise ValueError('既有手眼标定未被接受')
            source = handeye['source']
            if digest(root/source['path']) != source['sha256']:raise ValueError('既有手眼原始记录摘要不一致')
            hashes[source['path']] = source['sha256']
            transform = np.asarray(handeye['flange_T_color_optical_camera'],dtype=float)
            if (transform.shape != (4,4) or not np.isfinite(transform).all()
                    or not np.allclose(transform[3],[0,0,0,1])
                    or not np.allclose(transform[:3,:3].T@transform[:3,:3],np.eye(3),atol=1e-5)
                    or not np.isclose(np.linalg.det(transform[:3,:3]),1)):
                raise ValueError('既有手眼变换无效')
            board = {'inner_corners':[args.columns,args.rows], 'square_size_m':args.square_mm/1000,
                     'size_source':'operator_argument_or_existing_30mm_default_not_independently_measured'}
            report.update(board=board,input_hashes=hashes,handeye={'source':source,'flange_T_D435i':transform.tolist(),
                          'camera_reuse_policy':policy,'source_file_sha256':digest(handeye_path)})
            robot = robot or Robot(args.robot_ip)
            monitor = monitor or Monitor(args.monitor_url)
            preflight = {command:robot.query(command)['reply'] for command in
                         ('get_arm_software_info','get_current_tool_frame','get_current_work_frame','get_joint_en_state','get_joint_err_flag')}
            report['preflight'] = preflight
            if preflight['get_arm_software_info'].get('Product_version') != 'RM75-BI':raise ValueError('机器人型号不匹配')
            if preflight['get_joint_en_state'].get('en_state') != [1]*7 or preflight['get_joint_err_flag'].get('err_flag') != [0]*7:
                raise ValueError('关节使能/错误状态不满足采集条件；脚本不使能或清错')
            tool,work = preflight['get_current_tool_frame'],preflight['get_current_work_frame']
            def observe():
                value = robot.query('get_current_arm_state')
                report['observations'].append(value)
                flange_pose(value['reply']['arm_state'],tool,work)
                return value
            for _ in range(3):observe();time.sleep(.1)
            stationarity([v['reply']['arm_state'] for v in report['observations']])
            leases = monitor.leases()
            for lease in leases:lease.start()
            deadline = time.monotonic()+5
            while True:
                status = monitor.get('/api/status')['cameras']
                if any(row.get('state') == 'error' for row in status.values()):raise ValueError('相机预览异常：'+str({k:v.get('error') for k,v in status.items()}))
                if all(status[key]['state'] == 'streaming' for key in IDS):break
                if time.monotonic() >= deadline:raise ValueError('两路相机未在5秒内准备好')
                time.sleep(.1)
            before = observe();pairs = []
            for index in range(2):
                pair = monitor.pair()
                decoded = {key:decode_frame(pair[key],key,serials[key]) for key in IDS}
                for key in IDS:
                    raw,suffix,_ = decoded[key]
                    filename = key+('_check' if index else '')+suffix
                    (out/filename).write_bytes(raw)
                    report['images'][filename] = {'frame':{k:v for k,v in pair[key].items() if k != 'image_base64'},
                                                   'file':filename,'sha256':digest(out/filename)}
                pairs.append((pair,decoded));observe()
            after = report['observations'][-1]
            for _ in range(2):time.sleep(.1);observe()
            report['stationarity'] = stationarity([v['reply']['arm_state'] for v in report['observations']])
            if after['received_monotonic']-before['request_monotonic'] > LIMITS['max_robot_bracket_s']:
                raise ValueError('图像配对的机器人状态时间窗口超过1秒')
            final_tool = robot.query('get_current_tool_frame')['reply'];final_work = robot.query('get_current_work_frame')['reply']
            if final_tool != tool or final_work != work:raise ValueError('采集中工具或工作坐标系发生变化')
            if any(digest(root/path) != value for path,value in hashes.items()):raise ValueError('采集中相机/手眼配置发生变化')
            report['base_T_tcp'] = pose_matrix(before['reply']['arm_state']['pose']).tolist()
            report['flange_T_tcp'] = pose_matrix(tool['pose']).tolist()
            report['base_T_flange'] = flange_pose(before['reply']['arm_state'],tool,work).tolist()
            report['base_T_D435i'] = (np.asarray(report['base_T_flange'])@transform).tolist()
            report['camera_binding'] = camera_binding(handeye,pairs[0][0]['d435i']['camera'],policy)
            report['timing'] = {'method':'stationary_robot_and_board_not_hardware_synchronized',
                                'before':before,'after':after,'pairs':[]}
            analyzed = []
            for index,(pair,decoded) in enumerate(pairs):
                skew = abs(pair['d435i']['captured_at']-pair['d405']['captured_at'])
                if skew > LIMITS['max_pair_skew_s']:raise ValueError('两机位采集时间差超过200毫秒')
                if any(not before['received_unix'] <= pair[key]['captured_at'] <= after['received_unix'] for key in IDS):
                    raise ValueError('图像时间不在实际机器人反馈包围窗口内')
                report['timing']['pairs'].append({'skew_s':skew,'captured_at':{key:pair[key]['captured_at'] for key in IDS}})
                row = {}
                for key in IDS:
                    frame = pair[key];raw,suffix,image = decoded[key]
                    if index and frame['sequence'] == pairs[0][0][key]['sequence']:raise ValueError(key+'重复帧不能作为静止验证')
                    if index and frame['camera'] != pairs[0][0][key]['camera']:raise ValueError(key+'采集过程中内参变化')
                    filename = key+('_check' if index else '')+suffix
                    metadata = {k:v for k,v in frame.items() if k != 'image_base64'}
                    row[key],annotated = analyze_board(image,frame['camera'],board,args.d405_reverse_corners and key == 'd405')
                    row[key].update(frame=metadata,file=filename,sha256=digest(out/filename))
                    report['images'][filename] = row[key]
                    if index == 0 and not cv2.imwrite(str(out/(key+'_corners.jpg')),annotated):
                        raise ValueError('无法保存角点检查图')
                analyzed.append(row)
            shifts = {key:float(np.linalg.norm(np.asarray(analyzed[1][key]['corners_px'])-np.asarray(analyzed[0][key]['corners_px']),axis=1).max()) for key in IDS}
            report['board_stationarity'] = {'max_corner_shift_px':shifts}
            if any(value > LIMITS['max_board_shift_px'] for value in shifts.values()):raise ValueError('两次采集间棋盘角点移动超过1像素，请让板停稳后重新运行')
            manifest = {'schema_version':1,'board':board,'camera_binding':report['camera_binding'],
                        'cameras':{key:pairs[0][0][key]['camera'] for key in IDS},'handeye_file_sha256':digest(handeye_path),
                        'd405_reverse_corners':args.d405_reverse_corners}
            manifest_path = session/'session.json'
            if manifest_path.exists() and json.loads(manifest_path.read_text()) != manifest:
                raise ValueError('该会话已有不同相机/内参/棋盘/手眼定义，请使用新的--session')
            if not manifest_path.exists():atomic_json(manifest_path,manifest)
            report['status'] = 'candidate'
        except Exception as exc:
            report.update(error=str(exc),error_type=type(exc).__name__)
        finally:
            for lease in reversed(leases):lease.close()
            report['finished_at'] = time.time()
            atomic_json(out/'sample.json',report)
        return out,report


def main():
    parser = argparse.ArgumentParser(description='单次双相机标定采集：每运行一次保存一组，只读机器人，不运动、不切换工具。')
    parser.add_argument('--session',default='d405_base1_'+datetime.datetime.now(ZoneInfo('Asia/Singapore')).strftime('%Y%m%d'))
    parser.add_argument('--role',choices=('training','validation'),default='training')
    parser.add_argument('--robot-ip',default='192.168.1.19')
    parser.add_argument('--monitor-url',default='http://127.0.0.1:8088')
    parser.add_argument('--columns',type=int,default=8,help='每行内角点数，默认8')
    parser.add_argument('--rows',type=int,default=11,help='每列内角点数，默认11')
    parser.add_argument('--square-mm',type=float,default=30.,help='实测格长，单位毫米；沿用既有30mm默认值')
    parser.add_argument('--d405-reverse-corners',action='store_true',help='人工核对两个机位的物理原点后，反转D405角点顺序；同会话保持一致')
    parser.add_argument('--output-root',type=Path,default=ROOT/'work/dual_camera_calibration')
    args = parser.parse_args()
    if not args.session or any(c not in 'abcdefghijklmnopqrstuvwxyzABCDEFGHIJKLMNOPQRSTUVWXYZ0123456789_-' for c in args.session):parser.error('session仅允许字母、数字、下划线和连字符')
    if min(args.columns,args.rows) < 2 or max(args.columns,args.rows) > 50 or not np.isfinite(args.square_mm) or not 0 < args.square_mm <= 1000:parser.error('棋盘尺寸参数无效')
    try:out,report = capture_once(args)
    except Exception as exc:
        print(json.dumps({'status':'error','error':str(exc),'motion_sent':False},ensure_ascii=False));return 1
    print(json.dumps({'directory':str(out),'status':report['status'],'role':report['role'],'error':report.get('error'),
                      'reprojection_rms_px':{name:value.get('reprojection_rms_px') for name,value in report['images'].items()},
                      'board_stationarity':report.get('board_stationarity'),'motion_sent':False,
                      'note':'候选数据尚须核对物理角点原点；本脚本不求解或写入D405外参'},ensure_ascii=False))
    return 0 if report['status'] == 'candidate' else 2


if __name__ == '__main__':raise SystemExit(main())
