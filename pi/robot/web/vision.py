"""相机工作台：旧样本回放与固定本机视觉代理；不接触机器人控制接口。"""

import json
import base64
import math
import time
from typing import Literal
from pathlib import Path
from urllib.request import urlopen
from urllib.error import URLError

from fastapi import APIRouter, HTTPException, Request
from fastapi.responses import Response
from pydantic import BaseModel, ConfigDict, Field

from ..vision.workspace import Workspace, canonical

ROOT = Path(__file__).resolve().parents[3]
LIVE = 'http://127.0.0.1:8092'


class LiveClient:
    """视觉专用GET连接池，避免每帧重建SSH通道；不重试、不访问设备控制。"""
    def __init__(self):
        import httpx
        self.http = httpx.Client(trust_env=False, follow_redirects=False,
            transport=httpx.HTTPTransport(retries=0),
            limits=httpx.Limits(max_connections=2, max_keepalive_connections=2, keepalive_expiry=30))

    def read(self, path, timeout=2):
        import httpx
        if path.split('?')[0] not in ('/state', '/frame', '/observation'):
            raise ValueError('未知视觉只读路径')
        try:
            with self.http.stream('GET', LIVE + path, timeout=timeout) as response:
                response.raise_for_status()
                chunks, total = [], 0
                for chunk in response.iter_bytes():
                    total += len(chunk)
                    if total > 6*1024*1024:
                        raise ValueError('视觉响应超过上限')
                    chunks.append(chunk)
                return b''.join(chunks), response.headers.get('content-type', 'application/octet-stream')
        except httpx.HTTPError as error:
            raise OSError('视觉只读请求失败') from error

    def close(self):
        self.http.close()


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
    observation_order: int | None = Field(default=None, ge=1, le=999, strict=True)


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
    observation_order: int | None = Field(default=None, ge=1, le=999, strict=True)


class Color(BaseModel):
    model_config = ConfigDict(extra='forbid')
    observation_id: str = Field(pattern=r'^[0-9a-f]{32}$')
    roi: list[int] = Field(min_length=4, max_length=4)
    color_id: int = Field(ge=1, le=6)


def router(storage=None, device_client=None, live_client=None):
    api = APIRouter(prefix='/api/vision')
    store = storage or Workspace(ROOT / 'output/vision/workbench')

    def read_live(path, timeout):
        if live_client is not None:
            return live_client.read(path, timeout)
        with urlopen(LIVE + path, timeout=timeout) as response:
            return response.read(6*1024*1024 + 1), (response.headers.get_content_type() if hasattr(response, 'headers') else 'application/json')

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

    def observation_reference(data, snapshot):
        """用户标注的组后命令姿态；不是曝光时实测姿态，也不授予执行许可。"""
        binding, order = data.get('binding'), data.get('observation_order')
        validate_binding(binding, snapshot)
        if order is None:
            return None
        if binding is None or binding['kind'] != 'sequence':
            raise ValueError('观察顺序号需要关联一个已保存的联调版本')
        from ..device.mech_sequence import digest, groups
        record = next(v for v in snapshot['sequences'] if v['id'] == binding['id'] and
                      v['revision'] == binding['revision'] and v['sha256'] == binding['sha256'])
        if digest(record) != record['sha256']:
            raise ValueError('联调版本内容校验失败')
        state = {}
        for group in groups(record['rows']):
            for member in group['members']:
                state[member['axis']] = {k: member[k] for k in ('target_m', 'target_rad') if k in member}
            if group['order'] == order:
                return dict(kind='operator_selected_after_group', order=order,
                            commanded_pose=state, actual_pose_verified=False,
                            exposure_synchronized=False, motion_authority=False)
        raise ValueError('联调版本中不存在这个顺序号；填写表中顺序号，不是行号')

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
            raw, _ = read_live('/state', 1)
            live = json.loads(raw)
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
            data['observation_reference'] = observation_reference(data, snapshot)
            return store.save_sample(data, snapshot)
        return save(write)

    @api.post('/drafts')
    def save_draft(value: Draft, request: Request):
        same_origin(request)
        data = value.model_dump()
        def write():
            data['observation_reference'] = observation_reference(data, device())
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
            _, data = store.image(key)
        except (ValueError, OSError):
            raise HTTPException(404, '样本图像不存在或校验失败')
        return Response(data, media_type='image/jpeg', headers={'Cache-Control': 'no-store'})

    @api.get('/samples/{key}/observation')
    def sample_observation(key: str):
        try:
            value, data = store.image(key)
            result = dict(value['observation'], domain='saved_sample', camera=value['observation'].get('camera'),
                          original_domain=value['source_domain'], image_sha256=value['image_sha256'],
                          image=f'/api/vision/samples/{key}/image')
            return store.remember(result, data)
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
        if resource not in ('frame', 'observation') or role not in ('material', 'qr') or mode not in ('preview', 'material', 'ring', 'qr'):
            raise HTTPException(400, '未知视觉请求')
        try:
            started = time.monotonic()
            raw, content_type = read_live(f'/{resource}?role={role}&mode={mode}', 2)
            if len(raw) > 6*1024*1024:
                raise ValueError('视觉响应超过上限')
            if resource == 'observation':
                result = json.loads(raw)
                if not isinstance(result, dict):
                    raise ValueError('视觉响应结构错误')
                # json.loads可接受NaN/Infinity或溢出的1e999；整帧须能安全序列化。
                canonical(result)
                source_age = result.get('age_ms')
                if (result.get('domain') != 'camera_live' or result.get('camera') != role or
                        type(source_age) not in (int, float) or not math.isfinite(source_age) or
                        not 0 <= source_age <= 250):
                    raise ValueError('视觉来源或帧龄不正确')
                encoded = result.get('image', '')
                if not isinstance(encoded, str) or not encoded.startswith('data:image/jpeg;base64,'):
                    raise ValueError('视觉图像格式错误')
                image = base64.b64decode(encoded.partition(',')[2], validate=True)
                def final_age():
                    # 只累加本机请求耗时；Pi与PC的单调时钟不能直接相减。
                    elapsed = max(0., (time.monotonic() - started)*1000)
                    # 新相机返回本次服务耗时；它已包含在源帧龄和整段RTT中，不能重复累计。
                    # 不比较两台机器的绝对时钟，旧服务缺字段时仍使用原保守上界。
                    server_elapsed = result.get('server_elapsed_ms', 0)
                    if (type(server_elapsed) not in (int, float) or not math.isfinite(server_elapsed) or
                            not 0 <= server_elapsed <= elapsed):
                        raise ValueError('视觉服务耗时不正确')
                    age = source_age + elapsed - server_elapsed
                    if age > 250:
                        raise ValueError('视觉响应传输或处理后已过期')
                    # 前端仍叠加浏览器往返耗时，得到更保守的显示上界。
                    return dict(source_age_ms=source_age, proxy_elapsed_ms=elapsed, age_ms=age,
                                age_scope=('source_age_plus_transport' if 'server_elapsed_ms' in result
                                           else 'source_age_plus_proxy_request'))
                return store.remember(dict(result, recognition_mode=mode), image, finalize=final_age)
            return Response(raw, media_type=content_type,
                            headers={'Cache-Control': 'no-store'})
        except ValueError as error:
            raise HTTPException(503, str(error)) from error
        except (OSError, URLError, KeyError):
            raise HTTPException(503, '实时相机不可用')

    return api
