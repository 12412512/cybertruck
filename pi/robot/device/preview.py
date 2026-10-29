"""只在内存中收发GC协议，复用正式C执行器；没有串口或设备打开接口。"""

import ctypes as c
import json
import math
from pathlib import Path
import threading
import uuid

from . import protocol as p
from .tracking import Goal
from ..nav.frames import Pose, compose
from ..nav.field import Field
from ..track_sim import Plant

MODEL_LOCK = threading.Lock()
ROOT = Path(__file__).resolve().parents[3]


class PreviewTransport:
    def __init__(self, library, bundle, *, key_at_s=2., faults=None):
        if not MODEL_LOCK.locked():
            raise ValueError('主机协议模型必须由唯一联调工作线程持有')
        self.lib = c.CDLL(str(library))
        self.lib.Preview_Init.argtypes = [c.c_uint64]
        self.lib.Preview_Load.argtypes = [c.c_uint32, c.c_void_p, c.c_uint32]
        self.lib.Preview_Tick.argtypes = [c.c_uint32, c.POINTER(c.c_int64), c.POINTER(c.c_int16),
                                        c.c_int16, c.c_int16, c.c_uint8, c.c_uint8]
        self.lib.Preview_Target.argtypes = [c.POINTER(c.c_int16)]
        self.lib.Preview_Push.argtypes = [c.c_char_p, c.c_uint32]
        self.lib.Preview_Read.argtypes = [c.c_void_p]
        self.lib.Preview_Read.restype = c.c_uint16
        self.lib.Preview_Fault.argtypes = [c.c_uint32, c.c_uint32]
        self.lib.Preview_Init(uuid.uuid4().int & ((1 << 64)-1) or 1)
        self.zone = 1 if bundle['parameters']['start'] == 'start_1' else 2
        self.profile = json.loads((ROOT/'config/switch_trials.json').read_text(encoding='utf-8'))
        array = (Goal * len(bundle['goals']))(*(Goal.from_buffer_copy(bytes.fromhex(g)) for g in bundle['goals']))
        if not self.lib.Preview_Load(self.zone, array, len(array)):
            raise ValueError('正式C模型拒绝装载：检查段序、限速和单段期限')
        self.bundle_sha256 = bundle['sha256']
        self.plant = Plant(ideal=True, radius_m=self.profile['wheel_radius_m'],
                           k_m=self.profile['half_length_m']+self.profile['half_width_m'])
        self.field, self.origin = Field(data=bundle['field']), Pose(*bundle['map_origin'])
        self.now, self.key_at_s, self.faults = 0., key_at_s, faults or {}
        self.commands, self.trace, self.sweep_samples, self.collision = [], [], 0, ''
        self._feed()

    def write(self, wire):
        self.commands.extend(packet.kind for packet in p.Decoder().feed(wire))
        self.lib.Preview_Push(wire, len(wire))
        return len(wire)

    def read(self, size=4096):
        if self.faults.get('disconnect_at_s', math.inf) <= self.now:
            raise OSError('合成夹具断线，结果未知')
        data = c.create_string_buffer(256)
        count = self.lib.Preview_Read(data)
        return data.raw[:count]

    def _feed(self):
        yaw_scale, gyro_scale = self.profile['angle_scale_rad'], self.profile['gyro_scale_rad_s']
        sign = self.profile['yaw_sign']
        yaw = round((self.plant.yaw * sign + .7)/yaw_scale)
        yaw = ((yaw+32768) & 0xffff)-32768
        signs = self.profile['feedback_sign']
        counts = (c.c_int64*4)(*[round(t*65536)*s for t,s in zip(self.plant.turns,signs)])
        rpm = (c.c_int16*4)(*[round(v)*s for v,s in zip(self.plant.rpm,signs)])
        self.lib.Preview_Tick(self.plant.ms,counts,rpm,yaw,round(self.plant.wz*sign/gyro_scale),
            self.key_at_s is not None and self.now >= self.key_at_s,self.zone-1)

    def sleep(self, seconds):
        for _ in range(max(1,math.ceil(seconds/.002))):
            target = (c.c_int16*4)()
            self.lib.Preview_Target(target)
            self.plant.targets = [v*s for v,s in zip(target,self.profile['command_sign'])]
            self.plant.integrate()
            self.now = self.plant.ms/1000
            if self.now >= self.faults.get('at_s',math.inf):
                self.lib.Preview_Fault(self.faults.get('can',0),self.faults.get('drop',0))
            pose = compose(self.origin,Pose(self.plant.x,self.plant.y,self.plant.yaw))
            speed = max(map(abs,self.plant.rpm))*2*math.pi*self.plant.radius_m/60
            margin = (math.sqrt(2)*speed+(self.field.radius+.04)*speed/self.plant.k_m)*.002
            errors = self.field.errors(pose,f'start_{self.zone}',margin,tracking=False)
            self.sweep_samples += 1
            if errors and not self.collision:
                self.collision = '模型扫掠失败：'+','.join(errors)
                self.lib.Preview_Fault(0x10000,0)
            if self.plant.ms % 100 == 0:
                self.trace.append(pose.values())
                if len(self.trace)>2000:self.trace=self.trace[::2]
            if self.plant.ms % 10 == 0:self._feed()


def execute(library, bundle, runtime, *, progress=None, cancel=None, key_at_s=2., faults=None):
    from ..drive_core import run_trial
    from ..nav.bridge import validate
    validate(bundle)
    if not MODEL_LOCK.acquire(blocking=False):
        raise ValueError('另一个主机执行链正在运行')
    rows, transport = [], None
    runtime = Path(runtime)
    runtime.mkdir(parents=True,exist_ok=True)
    try:
        transport = PreviewTransport(library,bundle,key_at_s=key_at_s,faults=faults)
        def record(kind, **values):
            row = dict(kind=kind,mono_s=transport.now,**values)
            rows.append(row)
            if progress and kind == 'state':
                progress(dict(stage=values['stage'],segment=values['route_segment'],
                    execution=values['execution'],elapsed_s=transport.now,trace=list(transport.trace),
                    rpm=list(transport.plant.rpm)))
        try:
            result = run_trial(transport,transport.profile,'straight',runtime,record,
                clock=lambda:transport.now,sleep=transport.sleep,simulation=True,wait_start=True,
                planned_route=bundle,cancel=lambda:bool(transport.collision) or (cancel is not None and cancel()))
            status, reason = 'COMPLETE', ''
        except Exception as exc:
            result = None
            status, reason = 'CANCELLED' if isinstance(exc,InterruptedError) else 'FAILED',str(exc)
        stop = next((row for row in reversed(rows) if row['kind']=='stop_result'),None)
        report = dict(schema=1,domain='simulation',physical_validated=False,authorizes_motion=False,
            status=status,reason=transport.collision or reason,source_plan_id=bundle['source_plan_id'],
            bundle_sha256=bundle['sha256'],result=result,events=rows,
            stop_confirmed=bool(stop and stop['confirmed']),commands=transport.commands,
            elapsed_s=transport.now,sweep_samples=transport.sweep_samples,trace=transport.trace,
            rpm=list(transport.plant.rpm),execution=stop.get('execution') if stop else None,
            endpoint=compose(transport.origin,Pose(transport.plant.x,transport.plant.y,transport.plant.yaw)).values())
        (runtime/'report.json').write_text(json.dumps(report,ensure_ascii=False,indent=2),encoding='utf-8')
        return report
    finally:
        MODEL_LOCK.release()
