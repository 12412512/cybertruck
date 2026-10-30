"""同一树莓派的圆环只读订阅与异步记录；串口线程不等待HTTP或磁盘。"""
import copy
import json
import threading
import time
from urllib.request import build_opener, ProxyHandler


class RingFeed:
    def __init__(self, runtime=None, clock=time.monotonic):
        self.runtime=runtime
        self.clock=clock
        self.lock=threading.Lock()
        self.stop=threading.Event()
        self.active=False
        self.latest=None
        self.error='圆环订阅尚未启动'
        self.write_error=''
        self.pending=None
        self.thread=None

    def start(self):
        self.thread=threading.Thread(target=self.run,name='rough-ring-readonly',daemon=True)
        self.thread.start()

    def enable(self, enabled):
        with self.lock:
            self.active=bool(enabled)
            if not enabled:self.latest=None

    def snapshot(self):
        with self.lock:
            return copy.deepcopy(self.latest),self.error,self.write_error

    def save(self, value):
        if self.runtime:
            with self.lock:self.pending=copy.deepcopy(value)

    def run(self):
        http=build_opener(ProxyHandler({}))
        while True:
            with self.lock:
                enabled=self.active
                pending,self.pending=self.pending,None
            if self.stop.is_set() and pending is None:break
            if pending is not None:
                try:
                    from ..workbench.storage import write_json
                    write_json(self.runtime/'rough_centering_samples.json',pending)
                    with self.lock:self.write_error=''
                except (OSError,ValueError) as exc:
                    with self.lock:self.write_error='微调记录保存失败：'+str(exc)
            if enabled and not self.stop.is_set():
                try:
                    # 固定回环GET；不打开相机或串口，也不接受浏览器传入的观测作为运动依据。
                    with http.open('http://127.0.0.1:8092/observation?role=material&mode=ring&include_image=false',timeout=.35) as reply:
                        raw=reply.read(512*1024+1)
                    if len(raw)>512*1024:raise ValueError('圆环元数据超过上限')
                    value=json.loads(raw)
                    json.dumps(value,allow_nan=False)
                    if not isinstance(value,dict):raise ValueError('圆环响应结构无效')
                    with self.lock:self.latest=(value,self.clock());self.error=''
                except (OSError,ValueError,TypeError) as exc:
                    with self.lock:self.latest=None;self.error='圆环订阅不可用：'+str(exc)
            self.stop.wait(.14)

    def close(self):
        self.enable(False)
        self.stop.set()
        if self.thread:self.thread.join(timeout=.8)
