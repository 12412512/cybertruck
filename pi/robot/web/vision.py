"""相机工作台：旧样本回放与固定本机视觉代理；不接触机器人控制接口。"""

import json
from pathlib import Path
from urllib.request import urlopen
from urllib.error import URLError

from fastapi import APIRouter, HTTPException
from fastapi.responses import Response

ROOT = Path(__file__).resolve().parents[3]
LIVE = 'http://127.0.0.1:8092'


def router():
    api = APIRouter(prefix='/api/vision')

    def samples():
        try:
            return json.loads((ROOT / 'output/vision/samples.json').read_text(encoding='utf-8'))
        except FileNotFoundError:
            return []

    @api.get('/state')
    def state():
        try:
            with urlopen(LIVE + '/state', timeout=1) as response:
                live = json.load(response)
        except (OSError, ValueError, URLError):
            live = dict(available=False, reason='相机专用服务未连接；不会使用模拟画面替代实时画面')
        return dict(live=live, samples=[dict(id=s['id'], name=s['file']) for s in samples()],
                    motion_authority=False, metric_valid=False)

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
        data = (ROOT / 'output/vision/samples' / sample['file']).read_bytes()
        frame = cv2.imdecode(np.frombuffer(data, np.uint8), cv2.IMREAD_COLOR)
        if frame is None:
            raise HTTPException(422, '无法解码样本')
        return dict(Detector().process(frame, mode), domain='historical_replay',
                    source=sample, image=f'/api/vision/image/{sample_id}', frame_id=sample_id)

    @api.get('/image/{sample_id}')
    def image(sample_id: str):
        sample = next((s for s in samples() if s['id'] == sample_id), None)
        if sample is None:
            raise HTTPException(404, '样本不存在')
        return Response((ROOT / 'output/vision/samples' / sample['file']).read_bytes(),
                        media_type='image/jpeg', headers={'Cache-Control': 'no-store'})

    @api.get('/live/{resource}')
    def live(resource: str, role: str = 'material', mode: str = 'material'):
        if resource not in ('frame', 'observation') or role not in ('material', 'qr') or mode not in ('material', 'ring', 'qr'):
            raise HTTPException(400, '未知视觉请求')
        try:
            with urlopen(f'{LIVE}/{resource}?role={role}&mode={mode}', timeout=2) as response:
                return Response(response.read(), media_type=response.headers.get_content_type(),
                                headers={'Cache-Control': 'no-store'})
        except (OSError, URLError):
            raise HTTPException(503, '实时相机不可用')

    return api
