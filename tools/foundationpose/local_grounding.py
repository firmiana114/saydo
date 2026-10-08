"""Offline open-vocabulary boxes with per-object settings and a shared vision model."""

import hashlib
import json
from pathlib import Path
import sys
import time

import numpy as np

ROOT = Path(__file__).resolve().parents[2]
MODEL_REVISION = 'a2bb814dd30d776dcf7e30523b00659f4f141c71'
MODEL_SHA256 = (
    '1a2412ef99bd74bcd3c2a246fa1e48581f8889a1300c9051974741314fc042f3'
)


def read_local_detection(directory):
    path = Path(directory) / 'local_detection.json'
    if not path.exists():
        return None
    if path.is_symlink() or path.stat().st_size > 16384:
        raise ValueError('本地检测配置文件无效')
    raw = path.read_bytes()
    value = json.loads(raw)
    if (
        not isinstance(value, dict)
        or (
            set(value)
            != {
                'backend',
                'prompt',
                'box_threshold',
                'text_threshold',
                'max_box_area_fraction',
            }
        )
    ):
        raise ValueError('本地检测配置字段无效')
    if value['backend'] != 'grounding_dino':
        raise ValueError('不支持的本地检测器')
    if (
        not isinstance(value['prompt'], str)
        or not 1 <= len(value['prompt'].strip()) <= 300
    ):
        raise ValueError('本地检测描述须为1至300字符')
    for key in ('box_threshold', 'text_threshold'):
        if (
            type(value[key]) not in (int, float)
            or not np.isfinite(value[key])
            or not .1 <= value[key] <= .9
        ):
            raise ValueError('本地检测阈值须为0.1至0.9')
    fraction = value['max_box_area_fraction']
    if (
        type(fraction) not in (int, float)
        or not np.isfinite(fraction)
        or not .005 <= fraction <= .5
    ):
        raise ValueError('本地检测框面积占比上限须为0.005至0.5')
    return {**value, 'revision': hashlib.sha256(raw).hexdigest()}


def candidates_from_predictions(
    boxes, scores, threshold, width, height, max_area_fraction=1.
):
    rows = []
    for box, score in zip(boxes, scores):
        a = np.asarray(box, dtype=float)
        if (
            a.shape != (4,)
            or not np.isfinite(a).all()
            or not np.isfinite(score)
            or score < threshold
        ):
            continue
        a[[0, 2]] = np.clip(a[[0, 2]], 0, width)
        a[[1, 3]] = np.clip(a[[1, 3]], 0, height)
        if min(a[2] - a[0], a[3] - a[1]) < 8:
            continue
        if (a[2] - a[0]) * (a[3] - a[1]) > max_area_fraction * width * height:
            continue
        raw = [
            int(np.floor(a[0])),
            int(np.floor(a[1])),
            int(np.ceil(a[2])),
            int(np.ceil(a[3])),
        ]
        rows.append(
            {
                'raw_box': raw,
                'box': [
                    max(0, raw[0] - 4),
                    max(0, raw[1] - 4),
                    min(width, raw[2] + 4),
                    min(height, raw[3] + 4),
                ],
                'score': float(score),
                'area': float((a[2] - a[0]) * (a[3] - a[1])),
                'source': 'local_grounding_dino',
            }
        )
    kept = []
    for row in sorted(rows, key=lambda r: r['score'], reverse=True):
        a = np.asarray(row['raw_box'])
        duplicate = False
        for old in kept:
            b = np.asarray(old['raw_box'])
            intersection = np.maximum(
                0, np.minimum(a[2:], b[2:]) - np.maximum(a[:2], b[:2])
            ).prod()
            union = (
                np.prod(a[2:] - a[:2]) + np.prod(b[2:] - b[:2]) - intersection
            )
            if intersection / max(1, union) > .5:
                duplicate = True
                break
        if not duplicate:
            kept.append(row)
        if len(kept) == 8:
            break
    return kept


class LocalGroundingDetector:
    settings_key = 'local_detection'
    name = 'Grounding DINO Tiny'

    def __init__(self, root=ROOT):
        self.root = Path(root)
        self.processor = None
        self.model = None

    def ensure_loaded(self):
        if self.model is not None:
            return
        directory = self.root / 'work/foundationpose_setup/grounding-dino-tiny'
        source = json.loads((directory / 'source.json').read_text())
        if source.get('revision') != MODEL_REVISION:
            raise ValueError('本地检测权重版本不匹配')
        if (
            hashlib.sha256(
                (directory / 'model.safetensors').read_bytes()
            ).hexdigest()
            != MODEL_SHA256
        ):
            raise ValueError('本地检测权重哈希不匹配')
        sys.path.insert(0, str(self.root / 'work/local_detection_deps'))
        import torch
        from transformers import AutoProcessor, AutoModelForZeroShotObjectDetection
        # FoundationPose changes the global default tensor type to CUDA.
        # Load/process under a scoped CPU factory default, then move explicitly.
        with torch.device('cpu'):
            processor = AutoProcessor.from_pretrained(
                directory, local_files_only=True, trust_remote_code=False
            )
            model = AutoModelForZeroShotObjectDetection.from_pretrained(
                directory,
                local_files_only=True,
                trust_remote_code=False,
                use_safetensors=True,
                disable_custom_kernels=True,
            ).eval().to('cuda')
        self.processor, self.model = processor, model

    def detect(self, frame, settings, cancelled):
        import cv2
        import torch
        if cancelled():
            raise ValueError('本地识别已取消')
        if float(np.std(cv2.cvtColor(frame['bgr'], cv2.COLOR_BGR2GRAY))) < 2:
            return []
        self.ensure_loaded()
        started = time.monotonic()
        rgb = cv2.cvtColor(frame['bgr'], cv2.COLOR_BGR2RGB)
        height, width = rgb.shape[:2]
        with torch.device('cpu'), torch.inference_mode():
            inputs = self.processor(
                images=rgb, text=settings['prompt'], return_tensors='pt'
            ).to('cuda')
            outputs = self.model(**inputs)
            found = self.processor.post_process_grounded_object_detection(
                outputs,
                inputs.input_ids,
                threshold=settings['box_threshold'],
                text_threshold=settings['text_threshold'],
                target_sizes=[(height, width)],
            )[0]
            boxes = found['boxes'].detach().cpu().numpy()
            scores = found['scores'].detach().cpu().numpy()
        if cancelled():
            raise ValueError('本地识别已取消')
        if time.monotonic() - started > 10:
            raise ValueError('本地识别超过10秒，未采用迟到结果')
        return candidates_from_predictions(
            boxes,
            scores,
            settings['box_threshold'],
            width,
            height,
            settings['max_box_area_fraction'],
        )
