"""雷达唯一串口所有者；默认只读，显式现场区号才开放有界静扫。"""
import argparse
import json
from contextlib import asynccontextmanager
from pathlib import Path
import threading
import time
from urllib.request import urlopen

from fastapi import FastAPI, HTTPException, Request
from .lidar.health import Health
from .lidar.scan import StaticScan
from .lidar.calibrate import Calibration
from .lidar.calibration import digest
from .runtime import ProcessLock

from .lidar.contracts import ScanRequest, CalibrationPrepare, CalibrationCommit

def _still_evidence(value, received, now):
    """共用停车证据，不改变或覆盖真实故障字段。"""
    base, mech = value.get('base') or {}, value.get('mechanism') or {}
    age = value.get('state_age_ms')
    return (value.get('domain') == 'physical' and value.get('connected') is True and
            type(age) in (int,float) and 0 <= age+(now-received)*1000 < 250 and
            0 <= now-received < .25 and value.get('boot_id') and value.get('session_id') and
            base.get('stopped') is True and
            not value.get('owner') and not value.get('site_ready') and
            all(mech.get(key) == 0 for key in ('pwm_mask','axis_state','servo_state','pose_state',
                                             'axis_id','servo_id','pose_id','zero_valid','fault')))


def board_still(value, received, now):
    """普通静扫保持原无故障门槛，反馈龄连同缓存时间不得超250ms。"""
    base = value.get('base') or {}
    return bool(_still_evidence(value, received, now) and base.get('mode') == 0 and base.get('fault') == 0)


def calibration_still(value, received, now, allow_can_latched=False):
    """仅人工固定雷达的独立校准可接纳已锁定CAN故障；底盘许可不改变。"""
    if board_still(value, received, now):
        return True
    base = value.get('base') or {}
    return bool(allow_can_latched and _still_evidence(value, received, now) and
        base.get('mode') == 6 and base.get('fault') == 0x800)


class LidarCore:
    def __init__(self, node=1, clock=time.monotonic, zone=None, runtime=None,
                 calibration_prepare=False, calibration_write=False, calibration_can_locked=False):
        if calibration_write and not calibration_prepare:
            raise ValueError('校准写入必须同时开放只读准备')
        if calibration_can_locked and not calibration_write:
            raise ValueError('独立CAN锁存校准模式必须明确开放单次校准写入')
        self.health = Health(node)
        self.clock = clock
        self.lock = threading.RLock()
        self.zone = zone
        self.runtime = Path(runtime) if runtime else None
        self.job = None
        self.last_scan = None
        self.last_calibration = None
        self.calibration_enabled = calibration_prepare
        self.calibration_write_enabled = calibration_write
        self.calibration_can_locked = calibration_can_locked
        self.job_can_locked = False
        self.placement_id = ''
        self.backup_saved = False
        self.requests = {}
        self.witness, self.witness_at = {}, -1.
        self.identity = None

    def board(self, value, received):
        with self.lock:
            self.witness, self.witness_at = value, received

    def start_scan(self, request):
        now = self.clock()
        with self.lock:
            encoded = request.model_dump_json()
            if request.request_id in self.requests:
                previous, job_id = self.requests[request.request_id]
                if previous != encoded:
                    raise ValueError('同一扫描请求ID内容变化')
                return dict(id=job_id,status='ACCEPTED',repeated=True)
            if len(self.requests) >= 32:
                raise ValueError('本次现场扫描次数达到32次上限，需重新核对现场')
            if self.zone != request.zone or self.zone not in (1,2):
                raise ValueError('缺少与本次人工摆放匹配的厂家区域许可')
            if self.job is not None:
                raise ValueError('上一静扫尚未结束')
            if (not self.health.view(now)['connected'] or self.health.pending or
                    self.health.info['running']):
                raise ValueError('雷达握手未就绪、事务未结束或已在扫描')
            if not board_still(self.witness,self.witness_at,now):
                raise ValueError('底盘无新鲜无故障停车证据；不启动雷达')
            self.identity = self.witness['boot_id'],self.witness['session_id']
            self.job = StaticScan(self.health.node,request.zone,request.duration_s,now,self.health.sequence)
            self.requests[request.request_id] = encoded,self.job.id
            return dict(id=self.job.id,status='ACCEPTED',repeated=False)

    def prepare_calibration(self, request):
        now = self.clock()
        with self.lock:
            encoded = 'calibration_prepare:'+request.model_dump_json()
            if request.request_id in self.requests:
                previous, identity = self.requests[request.request_id]
                if previous != encoded:
                    raise ValueError('同一请求ID内容变化')
                return dict(id=identity, status='ACCEPTED', repeated=True)
            if not self.calibration_enabled or self.zone != request.zone or not request.manual_fixed_deployed:
                raise ValueError('校准准备未开放、区号不符或未确认人工展开固定')
            if len(self.requests) >= 32 or self.job is not None:
                raise ValueError('有未结束事务或本次请求容量已满')
            if not self.health.view(now)['connected'] or self.health.pending or self.health.info['running']:
                raise ValueError('雷达握手未就绪、事务未结束或已在扫描')
            self.identity = self.witness.get('boot_id'), self.witness.get('session_id')
            self.placement_id, self.backup_saved = request.placement_id, False
            self.job_can_locked = False
            self.job = Calibration(self.health.node, request.zone, now, self.health.sequence)
            self.requests[request.request_id] = encoded, self.job.id
            return dict(id=self.job.id, status='ACCEPTED', repeated=False)

    def commit_calibration(self, request):
        now = self.clock()
        with self.lock:
            encoded = 'calibration_commit:'+request.model_dump_json()
            if request.request_id in self.requests:
                previous, identity = self.requests[request.request_id]
                if previous != encoded:
                    raise ValueError('同一请求ID内容变化')
                return dict(id=identity, status='ACCEPTED', repeated=True)
            if not self.calibration_write_enabled:
                raise ValueError('本核心仅开放只读准备，校准写入关闭')
            if not isinstance(self.job, Calibration) or request.calibration_id != self.job.id:
                raise ValueError('不是当前校准准备票据')
            if len(self.requests) >= 32 or not request.confirm_save or request.placement_id != self.placement_id:
                raise ValueError('没有确认当前摆放/保存范围，或请求容量已满')
            if request.allow_can_latched and not self.calibration_can_locked:
                raise ValueError('服务未开放人工固定雷达的独立CAN锁存校准模式')
            if (not calibration_still(self.witness, self.witness_at, now, request.allow_can_latched) or
                    self.identity != (self.witness.get('boot_id'), self.witness.get('session_id'))):
                raise ValueError('校准写入缺少同一启动身份的新鲜无故障停车证据')
            if not self.backup_saved or not self.runtime:
                raise ValueError('旧位姿尚未完整归档，不允许写入')
            saved = json.loads((self.runtime/'calibrations'/(self.job.id+'.backup.json')).read_text(encoding='utf-8'))
            content = dict(zone=self.job.zone, information=saved['information'], persisted=saved['persisted'])
            if saved['backup_version'] != self.job.backup_version or digest(content) != self.job.backup_version:
                raise ValueError('磁盘备份内容或版本变化，不允许写入')
            self.job.approve(request.backup_version, now)
            self.job_can_locked = request.allow_can_latched
            if self.job_can_locked:
                self.calibration_write_enabled = False  # 本次许可只消费一次，不供后续自动提交。
            self.requests[request.request_id] = encoded, self.job.id
            return dict(id=self.job.id, status='ACCEPTED', repeated=False)

    def save_backup(self):
        if not isinstance(self.job, Calibration) or self.job.phase != 'PREPARED' or self.backup_saved:
            return
        if not self.runtime:
            return
        directory = self.runtime/'calibrations'
        directory.mkdir(parents=True, exist_ok=True)
        value = dict(domain='physical_lidar_backup', live=False, id=self.job.id,
            zone=self.job.zone, information=self.job.info, persisted=self.job.backup,
            backup_version=self.job.backup_version, board_identity=self.identity,
            placement_id=self.placement_id, observed_at_s=time.time(), authorizes_motion=False)
        path = directory/(self.job.id+'.backup.json')
        with path.open('x', encoding='utf-8') as stream:
            json.dump(value, stream, ensure_ascii=False, indent=2)
        self.backup_saved = True

    def finish(self):
        report = self.job.view(include_frames=True)
        report['observed_at_s'] = time.time()
        report['board_identity'] = self.identity
        report['placement_source'] = 'user confirmed start zone and heading; not inferred from photos'
        if isinstance(self.job, Calibration):
            report['radar_only_can_latched'] = self.job_can_locked
        if self.runtime:
            directory = self.runtime/('calibrations' if isinstance(self.job, Calibration) else 'scans')
            directory.mkdir(parents=True,exist_ok=True)
            (directory/(self.job.id+'.json')).write_text(json.dumps(report,ensure_ascii=False,indent=2),encoding='utf-8')
            (directory/(self.job.id+'.rx.hex')).write_text(self.job.wire.hex(),encoding='ascii')
        if isinstance(self.job, Calibration):
            self.last_calibration = dict(self.job.view(), observed_at_s=report['observed_at_s'],
                placement_id=self.placement_id, backup_saved=self.backup_saved)
        else:
            self.last_scan = dict(self.job.view(), observed_at_s=report['observed_at_s'])
        sequence, node = self.job.sequence,self.job.node
        self.job = None
        self.health = Health(node)
        self.health.sequence = sequence

    def tick(self, transport):
        with self.lock:
            now = self.clock()
            if self.job:
                needs_still = not isinstance(self.job, Calibration) or self.job.needs_still
                still = (calibration_still(self.witness, self.witness_at, now, self.job_can_locked)
                    if isinstance(self.job, Calibration) else board_still(self.witness, self.witness_at, now))
                if needs_still and (not still or
                        self.identity != (self.witness.get('boot_id'),self.witness.get('session_id'))):
                    self.job.abort('底盘反馈失效、故障、身份变化或失去停车证据',now)
                wire = self.job.step(now)
            else:
                wire = self.health.step(now)
        if wire and transport.write(wire) != len(wire):
            raise OSError('雷达串口未完整写入')
        data = transport.read(4096)
        if data:
            with self.lock:
                if self.job:
                    self.job.feed(data,self.clock())
                else:
                    self.health.feed(data,self.clock())
        with self.lock:
            self.save_backup()
            if self.job and self.job.phase in self.job.TERMINAL:
                self.finish()

    def state(self):
        with self.lock:
            value = self.health.view(self.clock())
            if self.job:
                value.update(connected=self.clock()-self.job.last_rx < 3,status=self.job.phase,
                    information=self.job.info or self.health.info,
                    reason='实机有界静扫，车体不运动；未配准地图')
            value['scan'] = self.job.view() if isinstance(self.job, StaticScan) else self.last_scan
            value['calibration'] = (dict(self.job.view(), placement_id=self.placement_id,
                backup_saved=self.backup_saved) if isinstance(self.job, Calibration) else self.last_calibration)
            value['calibration_capabilities'] = dict(prepare=self.calibration_enabled,
                write=self.calibration_write_enabled, board_ready=bool(board_still(self.witness,self.witness_at,self.clock())),
                isolated_can=self.calibration_can_locked,
                isolated_board_ready=calibration_still(self.witness,self.witness_at,self.clock(),self.calibration_can_locked))
            if isinstance(self.job, Calibration):
                value['reason'] = '校准事务'+self.job.phase+'；准备阶段不扫描、不写参数，提交后仍不授权车体动作'
            elif self.last_scan and not self.job:
                value['reason'] = '上次静扫'+self.last_scan['status']+'；厂家坐标未配准'
            value['diagnostic_zone'] = self.zone
            return value


def create_app(core, transport=None):
    stopping = threading.Event()
    def watch():
        while not stopping.is_set():
            try:
                with urlopen('http://127.0.0.1:8091/state',timeout=.15) as response:
                    data = response.read(65537)
                    if len(data) > 65536:
                        raise ValueError('底盘状态超过容量')
                    value = json.loads(data)
                core.board(value,core.clock())
            except (OSError,ValueError):
                core.board({},core.clock())
            stopping.wait(.05)

    def loop():
        while not stopping.is_set():
            try:
                core.tick(transport)
            except Exception as exc:
                with core.lock:
                    core.health.fail(f'雷达只读核心停止：{exc}')
                    if core.job:
                        core.job.abort('雷达串口异常，STOP未确认',core.clock())
                        try:
                            transport.write(core.job.transmit(4,core.clock()))
                        except OSError:
                            pass
                        core.job.phase = 'FAILED'
                        core.finish()
                        core.health.fail(f'雷达串口异常，停止未确认：{exc}')
                break
            stopping.wait(.005)

    @asynccontextmanager
    async def lifespan(app):
        thread = threading.Thread(target=loop, name='lidar-owner', daemon=True)
        watcher = threading.Thread(target=watch,name='lidar-still-watch',daemon=True)
        if transport:
            if core.zone:
                watcher.start()
            thread.start()
        try:
            yield
        finally:
            stopping.set()
            if transport:
                thread.join(timeout=1)
                if core.zone:
                    watcher.join(timeout=1)
                with core.lock:
                    if core.job:
                        core.job.abort('雷达核心退出，STOP未确认',core.clock())
                        try:
                            transport.write(core.job.transmit(4,core.clock()))
                        except OSError:
                            pass
                        core.job.phase = 'FAILED'
                        core.finish()
                transport.close()

    app = FastAPI(lifespan=lifespan, docs_url=None, redoc_url=None)
    @app.get('/state')
    def state():
        return core.state()
    if core.zone:
        @app.post('/scan')
        def scan(value:ScanRequest, request:Request):
            if request.headers.get('x-lidar-workbench') != '1':
                raise HTTPException(403,'只接受明确的静扫诊断请求')
            try:
                return core.start_scan(value)
            except ValueError as exc:
                raise HTTPException(409,str(exc)) from exc
    if core.zone and core.calibration_enabled:
        def check_header(request):
            if request.headers.get('x-lidar-workbench') != '1':
                raise HTTPException(403, '只接受明确的校准准备请求')

        @app.post('/calibration/prepare')
        def calibration_prepare(value:CalibrationPrepare, request:Request):
            check_header(request)
            try:
                return core.prepare_calibration(value)
            except ValueError as exc:
                raise HTTPException(409, str(exc)) from exc

        @app.post('/calibration/commit')
        def calibration_commit(value:CalibrationCommit, request:Request):
            check_header(request)
            try:
                return core.commit_calibration(value)
            except (ValueError, OSError, KeyError) as exc:
                raise HTTPException(409, str(exc)) from exc

        @app.post('/calibration/{identity}/cancel')
        def calibration_cancel(identity:str, request:Request):
            check_header(request)
            with core.lock:
                if not isinstance(core.job, Calibration) or core.job.id != identity:
                    raise HTTPException(409, '不是当前校准事务')
                core.job.cancel(core.clock())
                return dict(id=identity, status='STOP')
    return app


def distinct_port(lidar, stm32):
    """必须明确两个现存物理端口，拒绝同一设备的不同符号链接。"""
    radar_path, board_path = Path(lidar), Path(stm32)
    if not radar_path.is_absolute() or not board_path.is_absolute():
        raise ValueError('设备路径必须为绝对路径')
    if not radar_path.exists() or not board_path.exists() or radar_path.samefile(board_path):
        raise ValueError('雷达/STM32设备不存在或指向同一串口，拒绝打开')
    return str(radar_path)


def main():
    import serial
    import uvicorn
    parser = argparse.ArgumentParser(description='只读01/02/13握手；不扫描、不改区、不写标定')
    parser.add_argument('--serial', required=True)
    parser.add_argument('--stm32-serial', required=True)
    parser.add_argument('--runtime', type=Path, required=True)
    parser.add_argument('--node', type=int, default=1)
    parser.add_argument('--port', type=int, default=8093)
    parser.add_argument('--stationary-zone',type=int,choices=(1,2),help='仅本次人工确认的静止摆放区，不授予车辆运动权限')
    parser.add_argument('--calibration-prepare', action='store_true', help='开放只读重新取备份；不启动09')
    parser.add_argument('--calibration-write', action='store_true', help='明确开放受控提交入口；仍需票据、确认及无故障停车')
    parser.add_argument('--calibration-can-locked', action='store_true', help='仅本次人工固定雷达校准：保留CAN0x800和底盘锁定，许可消费一次')
    args = parser.parse_args()
    if (args.calibration_prepare or args.calibration_write) and not args.stationary_zone:
        parser.error('校准入口需要本轮明确静止区号')
    if args.calibration_write and not args.calibration_prepare:
        parser.error('写入入口必须先开放只读准备')
    if args.calibration_can_locked and not args.calibration_write:
        parser.error('独立校准模式需要明确单次写入许可')
    path = distinct_port(args.serial, args.stm32_serial)
    lock = ProcessLock(args.runtime/'lidar.lock')
    transport = serial.Serial(port=None, baudrate=115200, timeout=.01, write_timeout=.1, exclusive=True)
    try:
        transport.dtr = transport.rts = False
        transport.port = path
        transport.open()
        time.sleep(.2)
        transport.reset_input_buffer()
        core = LidarCore(args.node,zone=args.stationary_zone,runtime=args.runtime,
            calibration_prepare=args.calibration_prepare, calibration_write=args.calibration_write,
            calibration_can_locked=args.calibration_can_locked)
        uvicorn.run(create_app(core, transport),host='127.0.0.1',port=args.port,log_level='warning')
    finally:
        transport.close()
        lock.close()


if __name__=='__main__':
    main()
