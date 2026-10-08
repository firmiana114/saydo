"""Resident worker adapter; keeps incompatible Transformers out of FoundationPose."""

import base64
import json
import math
import os
from pathlib import Path
import select
import subprocess
import threading
import time
import uuid

from saydo import paths

ROOT = paths.ROOT


def read_qwen_settings(directory):
    p = Path(directory) / 'qwen_detection.json'
    if not p.exists():
        return None
    if p.is_symlink() or p.stat().st_size > 16384:
        raise ValueError('Qwen物品配置无效')
    d = json.loads(p.read_text())
    if not isinstance(d, dict) or set(d) != {'prompt', 'max_box_area_fraction'}:
        raise ValueError('Qwen物品配置字段无效')
    if not isinstance(d['prompt'], str) or not 1 <= len(d['prompt']) <= 800:
        raise ValueError('Qwen检测描述无效')
    v = d['max_box_area_fraction']
    if (
        type(v) not in (int, float)
        or not math.isfinite(v)
        or not .005 <= v <= .8
    ):
        raise ValueError('Qwen框面积上限无效')
    return d


def parse_boxes(text, width, height, max_area):
    text = text.strip()
    if text.startswith('```'):
        lines = text.splitlines()
        if lines[-1].strip() != '```':
            raise ValueError('Qwen JSON代码块不完整')
        text = '\n'.join(lines[1:-1])
    data = json.loads(text)
    if (
        not isinstance(data, dict)
        or set(data) != {'boxes'}
        or not isinstance(data['boxes'], list)
    ):
        raise ValueError('Qwen框格式无效')
    boxes = data['boxes']
    if len(boxes) == 4 and all(type(x) in (int, float) for x in boxes):
        boxes = [boxes]
    if len(boxes) > 16:
        raise ValueError('Qwen候选过多')
    found = []
    seen = set()
    for box in boxes:
        if (
            not isinstance(box, list)
            or len(box) != 4
            or any(
                (
                    type(v) not in (int, float)
                    or not math.isfinite(v)
                    or not 0 <= v <= 1000
                )
                for v in box
            )
        ):
            raise ValueError('Qwen框坐标无效')
        if box[0] >= box[2] or box[1] >= box[3]:
            raise ValueError('Qwen框方向无效')
        if tuple(box) in seen:
            continue
        seen.add(tuple(box))
        x0, y0, x1, y1 = [
            box[0] * width / 1000,
            box[1] * height / 1000,
            box[2] * width / 1000,
            box[3] * height / 1000,
        ]
        if (
            min(x1 - x0, y1 - y0) < 8
            or (x1 - x0) * (y1 - y0) > max_area * width * height
        ):
            continue
        raw = [math.floor(x0), math.floor(y0), math.ceil(x1), math.ceil(y1)]
        found.append(
            {
                'raw_box': raw,
                'box': [
                    max(0, raw[0] - 4),
                    max(0, raw[1] - 4),
                    min(width, raw[2] + 4),
                    min(height, raw[3] + 4),
                ],
                'area': (x1 - x0) * (y1 - y0),
                'source': 'local_qwen_vl_nf4',
            }
        )
    if len(found) > 8:
        raise ValueError('Qwen去重后候选仍超过8个')
    return found


class LocalQwenDetector:
    settings_key = 'qwen_detection'
    name = 'Qwen3-VL-4B NF4'

    def __init__(self, root=ROOT):
        self.root = Path(root)
        self.process = None
        self.ready = None
        self.lock = threading.RLock()
        self.closed = False

    def _read(self, timeout, cancelled=lambda: False):
        process = self.process
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            if self.closed or cancelled():
                self._stop_worker()
                raise ValueError('Qwen检测已取消')
            if select.select([process.stdout], [], [], .05)[0]:
                line = process.stdout.readline(65537)
                if not line or len(line) > 65536:
                    raise ValueError('Qwen工作进程退出或响应超限')
                return json.loads(line)
            if process.poll() is not None:
                raise ValueError('Qwen工作进程已退出')
        self._stop_worker()
        raise TimeoutError('Qwen工作进程等待超时')

    def ensure_loaded(self):
        with self.lock:
            if self.closed:
                raise ValueError('Qwen检测器已关闭')
            if (
                self.process is not None
                and self.process.poll() is None
                and self.ready
            ):
                return
            if self.process is not None:
                self._stop_worker()
            p = self.root / 'config/vision/qwen_local.json'
            config = json.loads(p.read_text())
            if (
                set(config) != {'python', 'model_dir', 'placement'}
                or (
                    config['placement']
                    not in ('cpu_vision', 'cpu_vision_head', 'gpu')
                )
            ):
                raise ValueError('Qwen运行配置无效')
            if not all(
                (
                    isinstance(config[k], str)
                    and Path(config[k]).is_absolute()
                    and Path(config[k]).exists()
                )
                for k in ('python', 'model_dir')
            ):
                raise ValueError('Qwen环境/权重须为存在的绝对路径')
            directory = self.root / 'work/qwen_detection'
            directory.mkdir(parents=True, exist_ok=True)
            env = dict(
                os.environ,
                HF_HUB_OFFLINE='1',
                TRANSFORMERS_OFFLINE='1',
                HF_HUB_DISABLE_TELEMETRY='1',
                TOKENIZERS_PARALLELISM='false',
            )
            for key in (
                'PYTHONPATH',
                'TORCH_FORCE_WEIGHTS_ONLY_LOAD',
                'CUDA_HOME',
                'CPLUS_INCLUDE_PATH',
            ):
                env.pop(key, None)
            with (
                directory / ('worker_' + str(time.time_ns()) + '.log')
            ).open('ab') as log:
                self.process = subprocess.Popen(
                    [
                        config['python'],
                        str(Path(__file__).with_name('qwen_worker.py')),
                        str(p),
                    ],
                    stdin=subprocess.PIPE,
                    stdout=subprocess.PIPE,
                    stderr=log,
                    text=True,
                    bufsize=1,
                    env=env,
                    start_new_session=True,
                )
            try:
                result = self._read(120)
                if result.get('event') != 'ready':
                    raise ValueError(
                        'Qwen启动失败：' + str(result.get('error', 'unknown'))
                    )
                self.ready = result
            except BaseException:
                self._stop_worker()
                raise

    def detect(self, frame, settings, cancelled):
        import cv2
        import torch
        with self.lock:
            if cancelled():
                raise ValueError('Qwen检测已取消')
            self.ensure_loaded()
            started = time.monotonic()
            h, w = frame['bgr'].shape[:2]
            if torch.cuda.is_initialized():
                torch.cuda.empty_cache()
            if (w, h) != (640, 360):
                raise ValueError('Qwen检测要求640×360画面')
            ok, encoded = cv2.imencode(
                '.jpg', frame['bgr'], [cv2.IMWRITE_JPEG_QUALITY, 95]
            )
            if not ok:
                raise ValueError('图像编码失败')
            ident = uuid.uuid4().hex
            try:
                self.process.stdin.write(
                    json.dumps(
                        {
                            'id': ident,
                            'image_base64': base64.b64encode(encoded).decode(),
                            'prompt': settings['prompt'],
                        }
                    )
                    + '\n'
                )
                self.process.stdin.flush()
                result = self._read(20, cancelled)
                if result.get('id') != ident:
                    raise ValueError('Qwen响应编号不匹配')
                if not result.get('ok'):
                    raise ValueError(
                        'Qwen检测失败：' + str(result.get('error', 'unknown'))
                    )
                self.last_result = {
                    k: v for k, v in result.items() if k != 'text'
                }
                found = parse_boxes(
                    result['text'], w, h, settings['max_box_area_fraction']
                )
                for row in found:
                    row['detection_seconds'] = time.monotonic() - started
                return found
            except (OSError, EOFError):
                self._stop_worker()
                raise ValueError('Qwen工作进程通信失败') from None

    def close(self):
        self.closed = True
        self._stop_worker()

    def _stop_worker(self):
        process = self.process
        self.ready = None
        if process is not None:
            if process.poll() is None:
                try:
                    process.terminate()
                except ProcessLookupError:
                    pass
                try:
                    process.wait(timeout=5)
                except subprocess.TimeoutExpired:
                    process.kill()
                    process.wait(timeout=3)
            for pipe in (process.stdin, process.stdout):
                if pipe is not None:
                    try:
                        pipe.close()
                    except OSError:
                        pass
            self.process = None
