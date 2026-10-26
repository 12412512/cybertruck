"""实车机构唯一串口核心；必须由命令行显式指定串口，启动只读。"""

import argparse
from contextlib import asynccontextmanager
from collections import OrderedDict, deque
import json
import hashlib
from pathlib import Path
import secrets
import threading
import time

from fastapi import FastAPI, HTTPException
from pydantic import BaseModel, ConfigDict, Field
from .device.mechanism import MechanismLink
from .device.mech_sequence import Sequence, record as sequence_record
from .device.recovery import Recovery
from .runtime import ProcessLock


class DeviceCommand(BaseModel):
    model_config = ConfigDict(extra='forbid', allow_inf_nan=False)
    op: str = Field(max_length=24)
    client: str = Field(min_length=1, max_length=64)
    token: str = Field(default='', max_length=64)
    request_id: str = Field(min_length=1, max_length=64)
    args: dict = Field(default_factory=dict)


class DeviceCore:
    def __init__(self, clock=time.monotonic, runtime=None, site_ready=False, commissioning=False):
        if commissioning and not site_ready:
            raise ValueError('机械调试模式需要本次现场许可；默认保持只读')
        self.link = MechanismLink(commissioning=commissioning)
        self.clock = clock
        self.lock = threading.RLock()
        self.owner = self.token = ''
        self.until = 0.
        self.cache = OrderedDict()
        self.safety_events = deque(maxlen=32)
        self.site_ready = site_ready
        self.pose_path = Path(runtime) / 'physical_poses.json' if runtime else None
        self.poses = []
        self.sequence = Sequence()
        self.recovery = Recovery(runtime)
        self.sequence_path = Path(runtime)/'physical_sequences.json' if runtime else None
        self.sequences = json.loads(self.sequence_path.read_text(encoding='utf-8')) if self.sequence_path and self.sequence_path.exists() else []
        if self.pose_path and self.pose_path.exists():
            self.poses = json.loads(self.pose_path.read_text(encoding='utf-8'))

    def tick(self, transport):
        now = self.clock()
        with self.lock:
            if self.token and now >= self.until:
                self.stop_control('控制权租约过期；动作锁定，只读反馈保留', 'lease-expired', self.owner)
            try:
                error = self.recovery.advance(self,now)
            except (OSError, ValueError) as exc:
                self.recovery.status = 'FAILED'
                error = self.recovery.reason = f'恢复归档/校验失败，保持停止：{exc}'
            if not error:
                error = self.sequence.advance(self.link,now)
            if error:
                self.stop_control(error,'program-failed',self.owner)
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
                        sequences=self.sequences, sequence=self.sequence.view(),
                        recovery=self.recovery.view(),
                        safety_events=list(self.safety_events),
                        lease_remaining_s=max(0., self.until-self.clock()) if self.token else 0.)

    def stop_control(self, reason, source, client):
        """由核心锁保护；不记录控制令牌，停止后仍由唯一串口读取反馈。"""
        self.link.emergency(reason)
        self.sequence.cancel(reason)
        self.recovery.cancel(reason)
        self.owner = self.token = ''
        self.until = 0.
        self.safety_events.append(dict(at_s=self.clock(), wall_s=time.time(),
            source=source, client=client, reason=reason))

    def command(self, value):
        value = DeviceCommand.model_validate(value)
        now = self.clock()
        with self.lock:
            if value.op == 'stop':
                source = value.args.get('source', 'explicit-stop')
                if not isinstance(source, str) or len(source) > 48:
                    source = 'external-stop'
                self.stop_control('Web请求停止；动作锁定，只读反馈保留', source, value.client)
                return dict(status='STOP_REQUESTED', domain='physical')
            if value.op == 'release':
                if (not self.token or value.client != self.owner or
                        not secrets.compare_digest(value.token, self.token)):
                    return dict(status='IGNORED_OLD_OWNER', domain='physical')
                source = value.args.get('source', 'page-release')
                if not isinstance(source, str) or len(source) > 48:
                    source = 'page-release'
                self.stop_control('页面退出控制；动作锁定，只读反馈保留', source, value.client)
                return dict(status='STOP_REQUESTED', domain='physical')
            if value.op == 'observe':
                # 诊断附着不取得运动租约；浏览器关闭后由唯一核心继续保活与只读轮询。
                if (self.site_ready or self.token or value.token or value.args or
                        self.link.firmware not in ('GC-BASE-0.3.23', 'GC-BASE-0.3.24', 'GC-BASE-0.3.25', 'GC-BASE-0.3.26', 'GC-BASE-0.3.27', 'GC-BASE-0.3.28', 'GC-BASE-0.3.29', 'GC-BASE-0.3.30', 'GC-BASE-0.3.31', 'GC-BASE-0.3.32', 'GC-BASE-0.3.33', 'GC-BASE-0.3.34')):
                    raise ValueError('只读附着要求独立只读模式、无人持有控制权且无附加参数')
                key = ('readonly:' + value.client, value.request_id)
                encoded = value.model_dump_json()
                if key in self.cache:
                    previous, result = self.cache[key]
                    if previous != encoded:
                        raise ValueError('相同只读请求ID内容发生变化')
                    return result
                result = self.link.submit(value.request_id, 'attach', {}, now)
                self.cache[key] = (encoded, result)
                while len(self.cache) > 100:
                    self.cache.popitem(last=False)
                return result
            if value.op == 'prepare':
                if not self.site_ready or self.token or value.token:
                    raise ValueError('重新准备要求现场许可且无人持有控制权')
                key = ('prepare:' + str(self.link.boot) + ':' + value.client, value.request_id)
                encoded = value.model_dump_json()
                if key in self.cache:
                    previous, result = self.cache[key]
                    if previous != encoded:
                        raise ValueError('相同准备请求ID内容发生变化')
                    return result
                result = self.link.prepare(value.request_id, value.args, now)
                self.cache[key] = (encoded, result)
                while len(self.cache) > 100:
                    self.cache.popitem(last=False)
                return result
            if value.op == 'recover':
                if not self.site_ready or self.token or value.token:
                    raise ValueError('故障恢复需要现场许可且无人持有控制权')
                key = ('recover:'+value.client, value.request_id)
                encoded = value.model_dump_json()
                if key in self.cache:
                    old, result = self.cache[key]
                    if old != encoded:
                        raise ValueError('相同恢复请求内容变化')
                    return result
                self.recovery.start(self.link, value.args, now)
                self.cache[key] = (encoded, dict(status='RECOVERY_STARTED',domain='physical',action_sent=False))
                while len(self.cache)>100:
                    self.cache.popitem(last=False)
                return self.cache[key][1]
            if self.recovery.active:
                raise ValueError('正在归档和恢复；请等待结果，可停止取消')
            if value.op == 'claim':
                if self.token and now < self.until and (value.client != self.owner or value.token != self.token):
                    raise ValueError('另一浏览器持有实车控制权')
                if self.link.action_locked:
                    raise ValueError('动作已停止并锁定；反馈保持在线，需核对后重新准备')
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
            if self.sequence.status=='RUNNING':
                raise ValueError('联调执行中，禁止插入单项、改模式或重复执行；可停止整组')
            if not self.site_ready and action not in ('teach', 'program_save', 'attach', 'invalidate'):
                raise ValueError('本次烧录后尚未确认实车条件；设备核心保持只读，不使能/PWM')
            if action == 'program_save':
                name=args.get('id','')
                revision=1+max((r['revision'] for r in self.sequences if r['id']==name),default=0)
                saved=sequence_record(name,args.get('rows'),revision)
                from .device.mechanism import payload
                from .device.mech_sequence import groups
                for stage in groups(saved['rows']):
                    payload('group',dict(members=stage['members']),{},1)
                self.sequences.append(saved)
                if self.sequence_path:
                    from .workbench.storage import write_json
                    write_json(self.sequence_path,self.sequences)
                result=dict(status='SAVED_DRAFT',revision=revision,domain='physical_draft')
            elif action == 'program_run':
                saved=next((r for r in self.sequences if r['id']==args.get('id') and r['revision']==args.get('revision')),None)
                if saved is None:
                    raise ValueError('联调表版本不存在，先保存')
                self.sequence.start(saved,args.get('mode','ordered'),self.link,now)
                result=dict(status='PROGRAM_STARTED',domain='physical',action_sent=False)
            elif action == 'teach':
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
                    core.link.reset(f'设备核心停止：{exc}')
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
    parser.add_argument('--mechanism-commissioning', action='store_true', help='显式开放兼容固件的机械调试，仍检查故障、反馈、零点和范围')
    args = parser.parse_args()
    if args.mechanism_commissioning and not args.site_ready:
        parser.error('机械调试模式必须同时确认本次现场条件')
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
        uvicorn.run(create_app(DeviceCore(runtime=args.runtime, site_ready=args.site_ready,
            commissioning=args.mechanism_commissioning), transport), host='127.0.0.1', port=args.port, log_level='warning')
    finally:
        transport.close()
        lock.close()


if __name__ == '__main__':
    main()
