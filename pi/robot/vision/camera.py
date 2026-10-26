"""显式启动的相机专用服务；无串口、CAN、动作接口。仅在Linux树莓派运行。"""

import argparse
import base64
import hashlib
import json
import threading
import time
import uuid
from pathlib import Path

from .observe import Detector, ROOT


class Capture:
    """单设备独占、最新帧缓存；断开即失效，不回退到其他摄像头。"""

    def __init__(self, device, size):
        self.device = device
        self.size = size
        self.lock = threading.Lock()
        self.stop = threading.Event()
        self.latest = None
        self.error = '等待采集'
        self.thread = None

    def start(self):
        self.thread = threading.Thread(target=self.run, daemon=True)
        self.thread.start()

    def run(self):
        import cv2
        import fcntl
        cap = None
        guard = None
        try:
            if not Path(self.device).exists():
                raise RuntimeError('指定USB相机未连接')
            # 按真实设备路径锁定；同一服务的消费者不重复打开设备。
            token = hashlib.sha256(str(Path(self.device).resolve()).encode()).hexdigest()[:16]
            guard = open(f'/tmp/gc-camera-{token}.lock', 'a')
            fcntl.flock(guard, fcntl.LOCK_EX | fcntl.LOCK_NB)
            cap = cv2.VideoCapture(self.device, cv2.CAP_V4L2)
            cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'MJPG'))
            cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.size[0])
            cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.size[1])
            cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
            if not cap.isOpened():
                raise RuntimeError('相机打开失败或被其他程序占用')
            sequence = 0
            while not self.stop.is_set():
                ok, frame = cap.read()
                captured = time.monotonic()
                if not ok:
                    raise RuntimeError('采集失败，请检查USB连接')
                if list(frame.shape[1::-1]) != self.size:
                    raise RuntimeError('实际分辨率与旧标定不一致')
                sequence += 1
                with self.lock:
                    self.latest = (sequence, captured, frame)
                    self.error = None
        except Exception as error:
            with self.lock:
                self.latest = None
                self.error = str(error)
        finally:
            if cap is not None:
                cap.release()
            if guard is not None:
                guard.close()

    def read(self):
        with self.lock:
            latest = self.latest
        if latest is None or time.monotonic() - latest[1] > .25:
            raise ValueError(self.error or '图像超过250毫秒，禁止使用旧画面')
        return latest


def create_app(config=None):
    from contextlib import asynccontextmanager
    from fastapi import FastAPI, HTTPException
    from fastapi.responses import Response
    import cv2
    # 沿用旧live.py的单线程设置，避免小图识别抢占过多CPU线程。
    cv2.setNumThreads(1)

    settings = json.loads(Path(config or ROOT / 'config/vision/cameras.json').read_text(encoding='utf-8'))
    captures = {role: Capture(settings[role], settings['resolution']) for role in ('material', 'qr')}
    detector = Detector()
    detect_lock = threading.Lock()
    stream_id = uuid.uuid4().hex

    @asynccontextmanager
    async def lifespan(app):
        for capture in captures.values():
            capture.start()
        yield
        for capture in captures.values():
            capture.stop.set()
        for capture in captures.values():
            capture.thread.join(timeout=2)

    app = FastAPI(lifespan=lifespan, docs_url=None, redoc_url=None)

    @app.get('/state')
    def state():
        devices = {}
        for role, capture in captures.items():
            try:
                sequence, stamp, _ = capture.read()
                devices[role] = dict(available=True, frame_id=sequence,
                                     age_ms=round((time.monotonic()-stamp)*1000, 1))
            except ValueError as error:
                devices[role] = dict(available=False, reason=str(error))
        return dict(available=any(v['available'] for v in devices.values()), cameras=devices,
                    stream_id=stream_id, motion_authority=False,
                    timestamp_kind='主机读取完成时刻；非曝光时刻')

    def read(role):
        if role not in captures:
            raise HTTPException(400, '未知相机')
        try:
            return captures[role].read()
        except ValueError as error:
            raise HTTPException(503, str(error))

    @app.get('/observation')
    def observation(role: str = 'material', mode: str = 'material'):
        with detect_lock:
            # 排队后再取最新帧；不能把等待识别锁的时间算成相机故障。
            sequence, stamp, frame = read(role)
            try:
                result = detector.process(frame, mode)
            except ValueError as error:
                raise HTTPException(400, str(error))
        age = time.monotonic() - stamp
        if age > .25:
            raise HTTPException(503, '识别完成时图像已过期')
        ok, image = cv2.imencode('.jpg', frame)
        if not ok:
            raise HTTPException(503, '图像编码失败')
        encoded = 'data:image/jpeg;base64,' + base64.b64encode(image).decode()
        age = time.monotonic() - stamp
        if age > .25:
            raise HTTPException(503, '图像编码后已过期')
        # 图像与叠加绑定同一帧，避免预览和识别各拉一张产生错位。
        return dict(result, domain='camera_live', camera=role, frame_id=sequence,
                    stream_id=stream_id, captured_s=stamp, processed_s=time.monotonic(),
                    timestamp_kind='host_read_complete_monotonic',
                    age_ms=round(age*1000, 1), image=encoded)

    @app.get('/frame')
    def frame(role: str = 'material'):
        _, _, image = read(role)
        ok, encoded = cv2.imencode('.jpg', image)
        if not ok:
            raise HTTPException(503, '图像编码失败')
        return Response(encoded.tobytes(), media_type='image/jpeg')

    return app


def main():
    import uvicorn
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--port', type=int, default=8092)
    args = parser.parse_args()
    uvicorn.run(create_app(), host='127.0.0.1', port=args.port)


if __name__ == '__main__':
    main()
