"""实车机构唯一串口核心；必须由命令行显式指定串口，启动只读。"""

import argparse
from contextlib import asynccontextmanager
from collections import OrderedDict
import json
import hashlib
from pathlib import Path
import secrets
import threading
import time

from fastapi import FastAPI, HTTPException
from pydantic import BaseModel, ConfigDict, Field
from .device.mechanism import MechanismLink
from .runtime import ProcessLock


class DeviceCommand(BaseModel):
    model_config = ConfigDict(extra='forbid', allow_inf_nan=False)
    op: str = Field(max_length=24)
    client: str = Field(min_length=1, max_length=64)
    token: str = Field(default='', max_length=64)
    request_id: str = Field(min_length=1, max_length=64)
    args: dict = Field(default_factory=dict)


class DeviceCore:
    def __init__(self, clock=time.monotonic, runtime=None, site_ready=False):
        self.link = MechanismLink()
        self.clock = clock
        self.lock = threading.RLock()
        self.owner = self.token = ''
        self.until = 0.
        self.cache = OrderedDict()
        self.site_ready = site_ready
        self.pose_path = Path(runtime) / 'physical_poses.json' if runtime else None
        self.poses = []
        if self.pose_path and self.pose_path.exists():
            self.poses = json.loads(self.pose_path.read_text(encoding='utf-8'))

    def tick(self, transport):
        now = self.clock()
        with self.lock:
            if self.token and now >= self.until:
                self.link.emergency('实车调试控制权过期，停止并失效会话')
                self.owner = self.token = ''
            wire = self.link.step(now)
        if wire:
            written = transport.write(wire)
            if written != len(wire):
                raise OSError('串口未完整写入')
        data = transport.read(4096)
        with self.lock:
            if data:
                self.link.feed(data, self.clock())

    def state(self):
        with self.lock:
            return dict(self.link.view(self.clock()), owner=self.owner, poses=self.poses, site_ready=self.site_ready,
                        lease_remaining_s=max(0., self.until-self.clock()) if self.token else 0.)

    def command(self, value):
        value = DeviceCommand.model_validate(value)
        now = self.clock()
        with self.lock:
            if value.op == 'stop':
                self.link.emergency('Web请求停止；保持夹爪，不自动恢复')
                self.owner = self.token = ''
                return dict(status='STOP_REQUESTED', domain='physical')
            if value.op == 'claim':
                if self.token and now < self.until and (value.client != self.owner or value.token != self.token):
                    raise ValueError('另一浏览器持有实车控制权')
                if self.link.latched:
                    raise ValueError('设备会话已失效，需人工只读核对后重启设备核心')
                self.owner = value.client
                if not self.token:
                    self.token = secrets.token_hex(24)
                self.until = now + 1.5
                return dict(token=self.token, domain='physical')
            if not self.token or now >= self.until or value.client != self.owner or not secrets.compare_digest(value.token, self.token):
                raise ValueError('实车控制权已失效')
            if value.op == 'renew':
                self.until = now + 1.5
                return dict(ok=True)
            if value.op != 'mechanism':
                raise ValueError('本入口只允许机构调试，不开放底盘或清故障')
            key = (self.token, value.request_id)
            encoded = value.model_dump_json()
            if key in self.cache:
                previous, result = self.cache[key]
                if previous != encoded:
                    raise ValueError('相同请求ID内容发生变化')
                return result
            args = dict(value.args)
            action = args.pop('action', '')
            if not self.site_ready and action not in ('teach', 'attach', 'invalidate'):
                raise ValueError('本次烧录后尚未确认实车条件；设备核心保持只读，不使能/PWM')
            if action == 'teach':
                pose_id = args.get('pose_id', '')
                if not pose_id or len(pose_id) > 48 or not all(c.isalnum() or c == '_' for c in pose_id):
                    raise ValueError('实车姿态编号无效')
                targets = args.get('targets', {})
                from .device.mechanism import payload
                payload('pose', dict(targets=targets, revision=1, physical_taught=True), {}, 1)
                record = dict(id=pose_id, revision=1 + max((r['revision'] for r in self.poses if r['id'] == pose_id), default=0),
                              targets=targets, domain='physical_draft', physical_validated=False)
                record['sha256'] = hashlib.sha256(json.dumps(record, sort_keys=True).encode()).hexdigest()
                self.poses.append(record)
                if self.pose_path:
                    from .workbench.storage import write_json
                    write_json(self.pose_path, self.poses)
                result = dict(status='SAVED_DRAFT', revision=record['revision'], domain='physical_draft')
            else:
                if action == 'pose':
                    record = next((r for r in self.poses if r['id'] == args.get('pose_id') and r['revision'] == args.get('revision')), None)
                    if record is None:
                        raise ValueError('实车示教版本不存在')
                    body = {k: v for k, v in record.items() if k != 'sha256'}
                    if hashlib.sha256(json.dumps(body, sort_keys=True).encode()).hexdigest() != record['sha256']:
                        raise ValueError('实车示教版本校验失败')
                    args['targets'] = record['targets']
                result = self.link.submit(value.request_id, action, args, now)
            self.cache[key] = (encoded, result)
            while len(self.cache) > 100:
                self.cache.popitem(last=False)
            return result


def create_app(core, transport=None):
    stop = threading.Event()

    def loop():
        while not stop.is_set():
            try:
                core.tick(transport)
            except Exception as exc:
                with core.lock:
                    core.link.emergency(f'设备核心停止：{exc}')
                    wire = core.link.step(core.clock())
                try:
                    if wire:
                        transport.write(wire)
                except OSError:
                    pass
                break
            stop.wait(.005)

    @asynccontextmanager
    async def lifespan(app):
        thread = threading.Thread(target=loop, name='device-owner', daemon=True)
        if transport:
            thread.start()
        try:
            yield
        finally:
            stop.set()
            if transport:
                thread.join(timeout=1)
                with core.lock:
                    core.link.emergency('设备核心退出')
                    wire = core.link.step(core.clock())
                try:
                    transport.write(wire)
                finally:
                    transport.close()

    app = FastAPI(lifespan=lifespan, docs_url=None, redoc_url=None)

    @app.get('/state')
    def state():
        return core.state()

    @app.post('/command')
    def command(value: DeviceCommand):
        try:
            return core.command(value)
        except (ValueError, TypeError, KeyError) as exc:
            raise HTTPException(409, str(exc)) from exc

    return app


def main():
    import serial
    import uvicorn
    parser = argparse.ArgumentParser(description='实车机构核心：仅显式命令可使能/建零/PWM')
    parser.add_argument('--serial', required=True, help='固定by-id或明确COM设备，不自动搜索')
    parser.add_argument('--runtime', type=Path, default=Path('output/runtime'))
    parser.add_argument('--port', type=int, default=8091)
    parser.add_argument('--site-ready', action='store_true', help='仅本次烧录后用户明确确认现场才可设置')
    args = parser.parse_args()
    args.runtime.mkdir(parents=True, exist_ok=True)
    lock = ProcessLock(args.runtime / 'core.lock')
    transport = serial.Serial(port=None, baudrate=115200, timeout=.01, write_timeout=.1)
    try:
        transport.dtr = transport.rts = False
        transport.port = args.serial
        if hasattr(transport, 'exclusive'):
            transport.exclusive = True
        transport.open()
        time.sleep(.5)  # USB打开可能复位；静默后用新boot/session握手。
        transport.reset_input_buffer()
        uvicorn.run(create_app(DeviceCore(runtime=args.runtime, site_ready=args.site_ready), transport), host='127.0.0.1', port=args.port, log_level='warning')
    finally:
        transport.close()
        lock.close()


if __name__ == '__main__':
    main()
