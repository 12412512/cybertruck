"""相机工作台：旧样本回放与固定本机视觉代理；不接触机器人控制接口。"""

import json
import base64
from typing import Literal
from pathlib import Path
from urllib.request import urlopen
from urllib.error import URLError

from fastapi import APIRouter, HTTPException, Request
from fastapi.responses import Response, FileResponse
from pydantic import BaseModel, ConfigDict, Field

from ..vision.workspace import Workspace

ROOT = Path(__file__).resolve().parents[3]
LIVE = 'http://127.0.0.1:8092'


class Binding(BaseModel):
    model_config = ConfigDict(extra='forbid')
    kind: Literal['pose', 'sequence']
    id: str = Field(min_length=1, max_length=48)
    revision: int = Field(ge=1)
    sha256: str = Field(pattern=r'^[0-9a-f]{64}$')


class Sample(BaseModel):
    model_config = ConfigDict(extra='forbid')
    observation_id: str = Field(pattern=r'^[0-9a-f]{32}$')
    name: str = Field(min_length=1, max_length=64)
    stage: Literal['raw', 'car_slot', 'rough', 'storage', 'stack'] = 'raw'
    label: Literal['unknown', 'empty', 'present', 'held', 'stored', 'released', 'placed'] = 'unknown'
    color_id: int | None = Field(default=None, ge=1, le=6)
    slot: int | None = Field(default=None, ge=1, le=3)
    note: str = Field(default='', max_length=1000)
    binding: Binding | None = None


class Point(BaseModel):
    model_config = ConfigDict(extra='forbid', allow_inf_nan=False)
    u: float = Field(ge=0, le=4096)
    v: float = Field(ge=0, le=4096)
    x_mm: float = Field(ge=-10000, le=10000)
    y_mm: float = Field(ge=-10000, le=10000)
    check: bool = False


class Draft(BaseModel):
    model_config = ConfigDict(extra='forbid')
    observation_id: str = Field(pattern=r'^[0-9a-f]{32}$')
    name: str = Field(min_length=1, max_length=64)
    plane: Literal['raw_table', 'floor', 'car_slot', 'stack_top'] = 'raw_table'
    roi: list[int] = Field(min_length=4, max_length=4)
    grasp_roi: list[int] | None = Field(default=None, min_length=4, max_length=4)
    points: list[Point] = Field(default_factory=list, max_length=24)
    binding: Binding | None = None


class Color(BaseModel):
    model_config = ConfigDict(extra='forbid')
    observation_id: str = Field(pattern=r'^[0-9a-f]{32}$')
    roi: list[int] = Field(min_length=4, max_length=4)
    color_id: int = Field(ge=1, le=6)


def router(storage=None, device_client=None):
    api = APIRouter(prefix='/api/vision')
    store = storage or Workspace(ROOT / 'output/vision/workbench')

    def device():
        from .workbench import CoreClient
        try:
            value = (device_client or CoreClient('http://127.0.0.1:8091')).request('/state')
            return {key: value.get(key) for key in ('connected', 'domain', 'firmware', 'boot_id',
                    'session_id', 'state_age_ms', 'base', 'mechanism', 'manual_debug', 'poses', 'sequences')}
        except HTTPException:
            return dict(connected=False, domain='physical', poses=[], sequences=[])

    def same_origin(request):
        origin = request.headers.get('origin')
        if request.headers.get('x-workbench') != '1' or (origin and origin != str(request.base_url).rstrip('/')):
            raise HTTPException(403, '仅接受工作台同源保存请求')

    def validate_binding(binding, snapshot):
        if binding is None:
            return
        records = snapshot.get('poses' if binding['kind'] == 'pose' else 'sequences') or []
        if not any(v.get('id') == binding['id'] and v.get('revision') == binding['revision'] and
                   v.get('sha256') == binding['sha256'] for v in records):
            raise ValueError('关联姿态版本已不可读取，请刷新版本；不会修改或执行姿态')

    def save(call):
        try:
            return call()
        except ValueError as error:
            raise HTTPException(400, str(error)) from error
        except OSError as error:
            raise HTTPException(503, '记录未保存，请检查磁盘空间或目录权限') from error

    def samples():
        try:
            values = json.loads((ROOT / 'output/vision/samples.json').read_text(encoding='utf-8'))
            return [v for v in values if isinstance(v, dict) and isinstance(v.get('id'), str) and
                    isinstance(v.get('file'), str) and Path(v['file']).name == v['file']]
        except (OSError, ValueError, TypeError):
            return []

    def sample_bytes(sample):
        try:
            path = ROOT / 'output/vision/samples' / sample['file']
            if path.stat().st_size > 4*1024*1024:
                raise HTTPException(413, '历史图像超过4MiB上限')
            return path.read_bytes()
        except OSError:
            raise HTTPException(404, '历史图像文件不存在，请重新准备回放样本')

    @api.get('/state')
    def state():
        try:
            with urlopen(LIVE + '/state', timeout=1) as response:
                live = json.load(response)
        except (OSError, ValueError, URLError):
            live = dict(available=False, reason='相机专用服务未连接；不会使用模拟画面替代实时画面')
        return dict(live=live, samples=[dict(id=s['id'], name=s['file']) for s in samples()],
                    motion_authority=False, metric_valid=False)

    @api.get('/workspace')
    def workspace():
        snapshot = device()
        return dict(samples=store.list('samples'), drafts=store.list('drafts'), colors=store.list('colors'),
                    bindings=[dict(kind=kind, **{k: v.get(k) for k in ('id', 'revision', 'sha256')})
                              for kind, name in (('pose', 'poses'), ('sequence', 'sequences'))
                              for v in snapshot.get(name) or [] if v.get('sha256')],
                    device_connected=snapshot.get('connected') is True,
                    quota_per_kind=200, motion_authority=False)

    @api.post('/samples')
    def save_sample(value: Sample, request: Request):
        same_origin(request)
        snapshot = device()
        data = value.model_dump()
        def write():
            validate_binding(data['binding'], snapshot)
            return store.save_sample(data, snapshot)
        return save(write)

    @api.post('/drafts')
    def save_draft(value: Draft, request: Request):
        same_origin(request)
        data = value.model_dump()
        def write():
            validate_binding(data['binding'], device())
            return store.save_draft(data)
        return save(write)

    @api.post('/colors')
    def save_color(value: Color, request: Request):
        same_origin(request)
        return save(lambda: store.color(value.observation_id, value.roi, value.color_id))

    @api.get('/records/{kind}/{key}')
    def record(kind: str, key: str):
        if kind not in ('samples', 'drafts', 'colors'):
            raise HTTPException(404, '记录分类不存在')
        try:
            value, _ = store.get(kind, key)
        except (ValueError, OSError):
            raise HTTPException(404, '记录不存在')
        return value

    @api.get('/samples/{key}/image')
    def sample_image(key: str):
        try:
            _, path = store.get('samples', key)
            if not path.is_file():
                raise ValueError('图像不存在')
        except (ValueError, OSError):
            raise HTTPException(404, '样本图像不存在')
        return FileResponse(path, media_type='image/jpeg', headers={'Cache-Control': 'no-store'})

    @api.get('/samples/{key}/observation')
    def sample_observation(key: str):
        try:
            value, path = store.get('samples', key)
            result = dict(value['observation'], domain='saved_sample', camera=value['observation'].get('camera'),
                          original_domain=value['source_domain'], image=f'/api/vision/samples/{key}/image')
            return store.remember(result, path.read_bytes())
        except (ValueError, OSError, KeyError):
            raise HTTPException(404, '样本无法读取')

    @api.get('/replay/{sample_id}')
    def replay(sample_id: str, mode: str = 'material'):
        import cv2
        import numpy as np
        from ..vision.observe import Detector
        if mode not in ('material', 'ring', 'qr'):
            raise HTTPException(400, '未知识别模式')
        sample = next((s for s in samples() if s['id'] == sample_id), None)
        if sample is None:
            raise HTTPException(404, '样本不存在')
        data = sample_bytes(sample)
        frame = cv2.imdecode(np.frombuffer(data, np.uint8), cv2.IMREAD_COLOR)
        if frame is None:
            raise HTTPException(422, '无法解码样本')
        result = dict(Detector().process(frame, mode), domain='historical_replay', camera=sample.get('camera', 'material'),
                      source=sample, image=f'/api/vision/image/{sample_id}', frame_id=sample_id)
        return store.remember(result, data)

    @api.get('/image/{sample_id}')
    def image(sample_id: str):
        sample = next((s for s in samples() if s['id'] == sample_id), None)
        if sample is None:
            raise HTTPException(404, '样本不存在')
        return Response(sample_bytes(sample),
                        media_type='image/jpeg', headers={'Cache-Control': 'no-store'})

    @api.get('/live/{resource}')
    def live(resource: str, role: str = 'material', mode: str = 'material'):
        if resource not in ('frame', 'observation') or role not in ('material', 'qr') or mode not in ('material', 'ring', 'qr'):
            raise HTTPException(400, '未知视觉请求')
        try:
            with urlopen(f'{LIVE}/{resource}?role={role}&mode={mode}', timeout=2) as response:
                raw = response.read(6*1024*1024 + 1)
                if len(raw) > 6*1024*1024:
                    raise ValueError('视觉响应超过上限')
                if resource == 'observation':
                    result = json.loads(raw)
                    if (result.get('domain') != 'camera_live' or result.get('camera') != role or
                            not isinstance(result.get('age_ms'), (int, float)) or not 0 <= result['age_ms'] <= 250):
                        raise ValueError('视觉来源或帧龄不正确')
                    encoded = result.get('image', '')
                    if not encoded.startswith('data:image/jpeg;base64,'):
                        raise ValueError('视觉图像格式错误')
                    image = base64.b64decode(encoded.partition(',')[2], validate=True)
                    return store.remember(dict(result, recognition_mode=mode), image)
                return Response(raw, media_type=response.headers.get_content_type(),
                                headers={'Cache-Control': 'no-store'})
        except (OSError, URLError, ValueError, KeyError):
            raise HTTPException(503, '实时相机不可用')

    return api
