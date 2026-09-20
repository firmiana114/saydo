"""Camera identity and per-device intrinsics for an explicitly reusable mounting."""
import numpy as np


def select_camera(devices, requested_serial=None):
    candidates = [d for d in devices if d['serial'] == requested_serial] if requested_serial else [
        d for d in devices if d['name'] == 'RealSense D435I']
    if not candidates:
        raise ValueError('未找到指定相机' if requested_serial else '未找到 RealSense D435I 相机')
    if len(candidates) != 1:
        raise ValueError('检测到多台 D435I，请用 --serial 指定相机')
    return candidates[0]


def camera_binding(handeye, camera, policy):
    if not isinstance(camera, dict) or not camera.get('serial'):
        raise ValueError('尚未读取实际相机身份，请等待相机采集')
    original = handeye['camera']
    if camera.get('name') != original['name']:
        raise ValueError(f"相机型号与手眼标定不匹配：当前 {camera.get('name')}，标定 {original['name']}")
    reused = camera['serial'] != original['serial']
    if reused and not (policy.get('enabled') is True and policy.get('mount_unchanged_confirmed') is True
                       and policy.get('camera_model') == camera['name']
                       and policy.get('handeye_source_sha256') == handeye['source']['sha256']
                       and policy.get('installation_id') == handeye['installation']['id']):
        raise ValueError(f"相机与手眼标定不匹配：当前 {camera['serial']}，标定 {original['serial']}；未配置同型号同安装复用")
    return {'mode': 'same_model_same_mount' if reused else 'original_camera',
            'active_serial': camera['serial'], 'calibration_serial': original['serial'],
            'camera_model': camera['name'], 'intrinsics_source': 'active_camera',
            'mount_basis': 'user_confirmed_unchanged' if reused else 'original_calibration',
            'replacement_accuracy_verified': False if reused else None}


def check_camera_observation(handeye, observation):
    observed = np.asarray(observation.get('K'), dtype=float)
    if (observed.shape != (3, 3) or not np.isfinite(observed).all()
            or observed[0, 0] <= 0 or observed[1, 1] <= 0
            or not np.allclose(observed[2], [0, 0, 1])):
        raise ValueError('实际视觉内参无效')
    runtime = handeye.get('runtime_camera')
    if runtime is None:
        expected = np.asarray(handeye['camera']['K'], dtype=float).copy()
        expected[:2] *= .5
    else:
        camera = observation.get('camera')
        if not isinstance(camera, dict) or camera != runtime:
            raise ValueError('视觉观测与当前相机身份或成像配置不匹配')
        if (camera.get('native_size') != handeye['camera']['size']
                or camera.get('image_size') != [640, 360]
                or camera.get('frame') != handeye['camera']['frame']
                or camera.get('distortion_coefficients') != [0.] * 5):
            raise ValueError('当前相机分辨率、光学坐标或畸变不受支持')
        expected = np.asarray(camera.get('K'), dtype=float)
    if expected.shape != (3, 3) or not np.allclose(observed, expected, rtol=0, atol=.05):
        raise ValueError('实际视觉内参与对应相机成像参数不匹配')
