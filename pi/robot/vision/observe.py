"""纯图像观测入口；像素候选不授予运动权限，也不增加成功计数。"""

import json
import hashlib
from pathlib import Path

from .legacy.material_vision.core import MaterialDetector
from .legacy.ring_vision.core import RingDetector
from .legacy.placement_vision.core import cleaned

ROOT = Path(__file__).resolve().parents[3]


class Detector:
    """复用旧阈值与数字模板；每帧独立检测，不伪造时序确认。"""

    def __init__(self, config=None):
        config = Path(config or ROOT / 'config/vision')
        self.profiles = {
            name: json.loads((config / f'{name}.json').read_text(encoding='utf-8'))
            for name in ('material_profile', 'ring_profile', 'station_color_profile')
        }
        self.material = MaterialDetector(self.profiles['material_profile'])
        self.ring = RingDetector(self.profiles['ring_profile'])
        self.version = hashlib.sha256(json.dumps(self.profiles, sort_keys=True).encode()).hexdigest()

    def process(self, frame, mode):
        if mode == 'material':
            items = cleaned(self.material.detect(frame))
        elif mode == 'ring':
            items = self.ring.detect(frame)
        elif mode == 'qr':
            import cv2
            from .legacy.qr_camera.task_codec import parse_task
            text, points, _ = cv2.QRCodeDetector().detectAndDecode(frame)
            items = []
            if text:
                geometry = {}
                if points is not None:
                    corners = points.reshape(-1, 2)
                    low, high = corners.min(axis=0), corners.max(axis=0)
                    geometry = dict(center_px=((low + high) / 2).tolist(),
                                    bbox_px=[float(low[0]), float(low[1]), float(high[0]), float(high[1])])
                try:
                    items.append(dict(text=text, task=parse_task(text), valid=True, **geometry))
                except ValueError:
                    items.append(dict(text=text, valid=False, **geometry))
        else:
            raise ValueError('未知识别模式')
        return dict(items=items, image_size=[frame.shape[1], frame.shape[0]],
                    recognition_mode=mode,
                    bbox_format='xywh' if mode == 'material' else 'xyxy',
                    profile_sha256=self.version, coordinate_frame='image_pixels',
                    metric_valid=False, motion_authority=False,
                    confirmation='候选观测；未确认持料、存车或释放')
