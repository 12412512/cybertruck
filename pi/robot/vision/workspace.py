"""视觉工作台的样本与标定草稿；只保存文件，不访问设备或授予动作权限。"""

import copy
import hashlib
import json
import math
import os
import threading
import time
import uuid
from collections import OrderedDict
from pathlib import Path

import numpy as np


def canonical(value):
    return json.dumps(value, ensure_ascii=False, sort_keys=True, allow_nan=False)


def rectangle(value, size):
    if (not isinstance(value, list) or len(value) != 4 or
            any(type(v) is not int for v in value) or
            not 0 <= value[0] < value[2] <= size[0] or
            not 0 <= value[1] < value[3] <= size[1]):
        raise ValueError('框选区域须位于原图内，且宽高大于零')
    return value


def planar_fit(points):
    """用训练点拟合，独立检查点只用于误差报告；结果始终是未应用草稿。"""
    train = [p for p in points if not p['check']]
    checks = [p for p in points if p['check']]
    if len(train) < 4 or not checks:
        raise ValueError('至少需要4个拟合点和1个独立检查点，建议6点以上')
    pixels = np.array([[p['u'], p['v']] for p in points], dtype=float)
    world = np.array([[p['x_mm'], p['y_mm']] for p in points], dtype=float)
    if not np.isfinite(pixels).all() or not np.isfinite(world).all():
        raise ValueError('标定点不能包含非有限数值')
    if len(np.unique(pixels, axis=0)) != len(points) or len(np.unique(world, axis=0)) != len(points):
        raise ValueError('拟合点与独立检查点必须是不同位置')

    def normalize(values):
        center = values.mean(axis=0)
        scale = math.sqrt(2) / np.linalg.norm(values - center, axis=1).mean()
        transform = np.array([[scale, 0, -scale*center[0]], [0, scale, -scale*center[1]], [0, 0, 1]])
        return (np.column_stack([values, np.ones(len(values))]) @ transform.T)[:, :2], transform

    indices = [i for i, p in enumerate(points) if not p['check']]
    a, ta = normalize(pixels[indices])
    b, tb = normalize(world[indices])
    rows = []
    for (u, v), (x, y) in zip(a, b):
        rows.extend([[-u, -v, -1, 0, 0, 0, x*u, x*v, x],
                     [0, 0, 0, -u, -v, -1, y*u, y*v, y]])
    matrix = np.asarray(rows)
    if np.linalg.matrix_rank(matrix) < 8:
        raise ValueError('标定点几何退化，请将点分布在二维平面的不同位置')
    _, _, vectors = np.linalg.svd(matrix, full_matrices=True)
    h = np.linalg.inv(tb) @ vectors[-1].reshape(3, 3) @ ta
    h /= np.linalg.norm(h)
    if np.linalg.matrix_rank(h) != 3:
        raise ValueError('平面映射不可逆')
    projected = np.column_stack([pixels, np.ones(len(points))]) @ h.T
    if np.any(np.abs(projected[:, 2]) < 1e-10):
        raise ValueError('标定点映射到无穷远，不能使用')
    residual = np.linalg.norm(projected[:, :2] / projected[:, 2:] - world, axis=1)
    if not np.isfinite(residual).all():
        raise ValueError('标定结果无效')
    return dict(homography=h.tolist(), unit='mm', direction='image_pixels_to_plane_mm',
                fit_max_mm=float(max(residual[indices])),
                check_max_mm=float(max(residual[i] for i, p in enumerate(points) if p['check'])),
                residual_mm=residual.tolist(), metric_valid=False, applied=False)


class Workspace:
    """小容量帧引用与不可覆盖版本；浏览器不能提交伪造图像作为实时证据。"""

    def __init__(self, root, clock=time.monotonic):
        self.root = Path(root)
        self.clock = clock
        self.lock = threading.RLock()
        self.frames = OrderedDict()

    def remember(self, result, image_bytes, finalize=None):
        if not image_bytes or len(image_bytes) > 4*1024*1024:
            raise ValueError('图像为空或超过4MiB保存上限')
        image_bytes = bytes(image_bytes)
        digest = hashlib.sha256(image_bytes).hexdigest()
        if result.get('image_sha256', digest) != digest:
            raise ValueError('图像与观测记录不匹配')
        record = copy.deepcopy({k: v for k, v in result.items() if k != 'image'})
        record['image_sha256'] = digest
        key = uuid.uuid4().hex
        with self.lock:
            # 代理帧龄最后核对，覆盖哈希/深拷贝/等待缓存锁的时间；失败不留引用。
            if finalize is not None:
                record.update(finalize())
            self.frames[key] = (self.clock(), record, image_bytes)
            while len(self.frames) > 12:
                self.frames.popitem(last=False)
        return {**result, **record, 'observation_id': key}

    def frame(self, key):
        with self.lock:
            value = self.frames.get(key)
            if value is None or self.clock() - value[0] > 1200:
                raise ValueError('画面引用已失效，请读取新帧后保存')
            if hashlib.sha256(value[2]).hexdigest() != value[1].get('image_sha256'):
                raise ValueError('缓存图像与观测记录不匹配')
            return copy.deepcopy(value[1]), value[2]

    def _write(self, folder, value, image=None):
        with self.lock:
            directory = self.root / folder
            directory.mkdir(parents=True, exist_ok=True)
            records = list(directory.glob('*.json'))
            if len(records) >= 200:
                raise ValueError('本类记录已达200条，请先归档；不会自动删除历史数据')
            used = sum(p.stat().st_size for p in self.root.glob('*/*.jpg'))
            if used + len(image or b'') > 512*1024*1024:
                raise ValueError('视觉图像已达512MiB，请先归档；不会自动清除旧样本')
            previous = self.list(folder)
            identity = value.get('name') or str(value.get('color_id'))
            revision = 1 + max((v['revision'] for v in previous
                                if (v.get('name') or str(v.get('color_id'))) == identity), default=0)
            value = dict(value, id=uuid.uuid4().hex, created_s=time.time(),
                         revision=revision, domain='vision_draft', applied=False, physical_validated=False)
            if image is not None:
                value['image_sha256'] = hashlib.sha256(image).hexdigest()
            value['sha256'] = hashlib.sha256(canonical(value).encode()).hexdigest()
            path = directory / (value['id'] + '.json')
            # 新版本用独立文件名，异常退出不会覆盖旧版本。
            temp = path.with_suffix('.tmp')
            try:
                if image is not None:
                    with path.with_suffix('.jpg').open('xb') as handle:
                        handle.write(image)
                with temp.open('x', encoding='utf-8') as handle:
                    handle.write(canonical(value))
                    handle.flush()
                    os.fsync(handle.fileno())
                os.replace(temp, path)
            except Exception:
                temp.unlink(missing_ok=True)
                if image is not None:
                    path.with_suffix('.jpg').unlink(missing_ok=True)
                raise
            return value

    def list(self, folder):
        with self.lock:
            result = []
            for path in (self.root / folder).glob('*.json'):
                try:
                    value, _ = self.get(folder, path.stem)
                    keys = ('id', 'name', 'created_s', 'revision', 'sha256', 'domain', 'source_domain',
                            'label', 'stage', 'color_id', 'slot', 'applied', 'physical_validated')
                    result.append({k: value[k] for k in keys if k in value})
                except (OSError, ValueError, KeyError):
                    continue
            return sorted(result, key=lambda v: v['created_s'], reverse=True)

    def get(self, folder, key):
        if len(key) != 32 or any(c not in '0123456789abcdef' for c in key):
            raise ValueError('记录编号无效')
        path = self.root / folder / (key + '.json')
        value = json.loads(path.read_text(encoding='utf-8'))
        if not isinstance(value, dict) or value.get('id') != key:
            raise ValueError('记录结构无效')
        original = {k: v for k, v in value.items() if k != 'sha256'}
        if hashlib.sha256(canonical(original).encode()).hexdigest() != value.get('sha256'):
            raise ValueError('记录校验失败')
        return value, path.with_suffix('.jpg')

    def image(self, key):
        """一次读取并校验同一记录的图像；调用方直接使用这些字节，不再打开文件。"""
        value, path = self.get('samples', key)
        digest = value.get('image_sha256')
        if (not isinstance(digest, str) or len(digest) != 64 or
                any(c not in '0123456789abcdef' for c in digest)):
            raise ValueError('样本缺少图像校验值')
        with path.open('rb') as handle:
            data = handle.read(4*1024*1024 + 1)
        if not data or len(data) > 4*1024*1024 or hashlib.sha256(data).hexdigest() != digest:
            raise ValueError('样本图像损坏或与记录不匹配')
        return value, data

    def save_sample(self, data, device):
        data = copy.deepcopy(data)
        observation, image = self.frame(data.pop('observation_id'))
        return self._write('samples', dict(data, observation=observation, device=device,
                           source_domain=observation['domain'], image_sha256=hashlib.sha256(image).hexdigest(),
                           annotation_kind='operator_label', snapshot_relation='at_save_not_exposure',
                           device_recorded_s=time.time(), success_count_changed=False), image)

    def save_draft(self, data):
        data = copy.deepcopy(data)
        observation, _ = self.frame(data.pop('observation_id'))
        size = observation['image_size']
        data['roi'] = rectangle(data['roi'], size)
        if data.get('grasp_roi') is not None:
            rectangle(data['grasp_roi'], size)
        for point in data['points']:
            if not 0 <= point['u'] < size[0] or not 0 <= point['v'] < size[1]:
                raise ValueError('标定像素点超出当前图像')
        fit = planar_fit(data['points']) if data['points'] else None
        return self._write('drafts', dict(data, fit=fit, image_size=size,
                           camera=observation.get('camera'), source_domain=observation['domain'],
                           source_profile_sha256=observation['profile_sha256'], metric_valid=False))

    def color(self, key, rect, color_id):
        import cv2
        from .legacy.material_vision.core import sample_color
        observation, image = self.frame(key)
        rectangle(rect, observation['image_size'])
        frame = cv2.imdecode(np.frombuffer(image, np.uint8), cv2.IMREAD_COLOR)
        if frame is None:
            raise ValueError('样本图像无法解码')
        model = sample_color(frame, rect, color_id)
        x1, y1, x2, y2 = rect
        ok, crop = cv2.imencode('.jpg', frame[y1:y2, x1:x2])
        if not ok:
            raise ValueError('颜色区域无法编码')
        return self._write('colors', dict(color_id=color_id, model=model, roi=rect,
                           camera=observation.get('camera'), frame_id=observation['frame_id'],
                           stream_id=observation.get('stream_id'), image_size=observation['image_size'],
                           source_domain=observation['domain'],
                           source_image_sha256=hashlib.sha256(image).hexdigest(),
                           source_profile_sha256=observation['profile_sha256']), crop.tobytes())
