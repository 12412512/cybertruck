"""唯一离线控制核心：控制权、去重、点动租约、参数版本和规划任务。"""

from collections import OrderedDict, deque
from concurrent.futures import ThreadPoolExecutor
import copy
import ctypes as c
import json
from pathlib import Path
import secrets
import threading
import time
import uuid

from .contracts import ARGUMENTS, Command, PlanInput
from .planning import make_plan, restore_plan
from .scene import make_field, normalize
from .storage import write_json, payload, versions
from .simulation import Simulation
from .mechanism import Mechanism
from .transfer import Transfer
from .program import Program
from ..device.integration import preparation


class Rejected(ValueError):
    pass


class Workbench:
    def __init__(self, library, runtime, clock=time.monotonic):
        self.library, self.runtime, self.clock = Path(library), Path(runtime), clock
        self.runtime.mkdir(parents=True, exist_ok=True)
        self.lock = threading.RLock()
        self.sim = Simulation(self.library)
        self.mech = Mechanism(self.runtime, self.clock)
        self.transfer = Transfer(self.mech)
        self.program = Program(self.mech,self.library)
        self.preparation = preparation()
        self.owner, self.token, self.lease_until, self.sequence = '', '', 0., -1
        self.manual_until = 0.
        self.cache = OrderedDict()
        self.events = deque(maxlen=100)
        self._load_events()
        self.plans = OrderedDict()
        self.job = dict(status='IDLE')
        self.planning_cancel = threading.Event()
        self.execution_cancel = threading.Event()
        self.execution_check = dict(schema=1,domain='simulation',status='IDLE',
            authorizes_motion=False,physical_validated=False,trace=[])
        self.lidar = dict(status='IDLE', domain='simulation', live=False, usable=False,
                          authorizes_motion=False, reason='尚未回放；历史扫描不自动恢复为当前观测')
        self.preview_available = hasattr(c.CDLL(str(self.library)), 'Preview_Load')
        try:
            old = json.loads((self.runtime/'execution_check.json').read_text(encoding='utf-8'))
            if old.get('schema') == 1 and old.get('domain') == 'simulation':
                self.execution_check.update(old,authorizes_motion=False,physical_validated=False)
                self.execution_check['visible'] = False
                if old.get('status') == 'RUNNING':
                    self.execution_check.update(status='INTERRUPTED',reason='核心重启；不自动重跑或续跑')
        except (OSError,ValueError,TypeError):
            pass
        self.draft = normalize(PlanInput())
        self.draft_revision = 0
        self.executor = ThreadPoolExecutor(max_workers=1, thread_name_prefix='route-plan')
        self.closed = False
        self.log_error = ''
        self.field = make_field(self.draft).data
        self.saved = versions(self.runtime)
        self._restore_workspace()
        self.sim.reset(self.draft.start)
        self.record('核心启动', '离线模型；真实设备接口未启用')

    def _persist(self):
        value = dict(schema=2, draft=self.draft.model_dump())
        if self.job['status'] == 'READY':
            value['last_plan'] = payload(self.plans[self.job['plan']['id']])
        write_json(self.runtime / 'workspace.json', value)

    def _restore_workspace(self):
        try:
            value = json.loads((self.runtime / 'workspace.json').read_text(encoding='utf-8'))
            self.draft = normalize(PlanInput.model_validate(value['draft']))
            self.field = make_field(self.draft).data
            if value.get('last_plan'):
                plan = restore_plan(value['last_plan'], self.library)
                if plan.view['parameters'] == self.draft.model_dump():
                    self.plans[plan.view['id']] = plan
                    self.job = dict(id=plan.view['id'], status='READY', plan=plan.view, restored=True)
        except (OSError, ValueError, TypeError, KeyError) as exc:
            if (self.runtime / 'workspace.json').exists():
                self.record('工作区读取失败', str(exc))

    def _load_events(self):
        path = self.runtime / 'events.jsonl'
        try:
            with path.open('rb') as stream:
                stream.seek(0, 2)
                size = stream.tell()
                stream.seek(max(0, size - 65536))
                if size > 65536:
                    stream.readline()
                lines = stream.read().decode('utf-8').splitlines()
            for line in reversed(lines[-100:]):
                event = json.loads(line)
                if all(key in event for key in ('at', 'name', 'detail')):
                    self.events.append(event)
        except (OSError, ValueError, UnicodeError, TypeError):
            pass

    def record(self, name, detail=''):
        event = dict(at=time.time(), name=name, detail=detail)
        self.events.appendleft(event)
        path = self.runtime / 'events.jsonl'
        try:
            if path.exists() and path.stat().st_size >= 1024 * 1024:
                path.replace(self.runtime / 'events.previous.jsonl')
            with path.open('a', encoding='utf-8') as stream:
                stream.write(json.dumps(event, ensure_ascii=False) + '\n')
        except OSError as exc:
            self.log_error = str(exc)  # 日志写入失败不能阻断点动停车。

    def _expire(self):
        now = self.clock()
        if self.sim.mode == 'MANUAL' and (now >= self.manual_until or now >= self.lease_until):
            self.sim.halt('点动续约超时，已请求模型停车')
            self.record('自动停车', '点动帧超过350ms或控制权过期')
        if self.token and now >= self.lease_until:
            self.execution_cancel.set()
            self.program.cancel('离线控制权过期')
            self.transfer.cancel('离线控制权过期，不自动续跑')
            if self.mech.busy:
                self.mech.stop('控制权过期，机构调试已停止')
            self.owner, self.token = '', ''

    def _idle(self):
        if self.program.busy:
            raise Rejected('工位姿态流程正在联调，请先停止或等待完成')
        if self.execution_check['status'] == 'RUNNING':
            raise Rejected('执行链离线联调未结束，请先取消并等待模型停车')
        if self.transfer.busy:
            raise Rejected('转存流程未结束，底盘模型不能运动')
        if self.mech.busy:
            raise Rejected('机构运行中，禁止底盘运动')
        if self.mech.zero_version and not self.mech.transport_ready():
            raise Rejected('机构未满足模拟运输互锁：零点有效、升降/伸缩收回、转臂180°')
        if not self.sim.stopped():
            raise Rejected('请先停车并等待模型轮速稳定')
        if self.sim.mode == 'FAILED':
            raise Rejected('模型故障仍保留，请显式重置离线场景')

    def _scan_matches(self, plan):
        observation = plan.view.get('lidar_source')
        if observation and (not self.lidar.get('usable') or observation['scan_id'] != self.lidar.get('id') or
                            observation['scan_sha256'] != self.lidar.get('sha256')):
            raise Rejected('雷达计划的扫描已替换、清空或失效；请回放并重新规划')

    def _lidar(self, arguments):
        from ..lidar.fixtures import fixture
        from ..lidar.replay import replay
        from ..lidar.planning import parameters, source
        self._idle()
        if arguments.action == 'clear':
            self.planning_cancel.set()
            if self.job['status'] == 'PLANNING':
                self.job = dict(status='IDLE',reason='扫描已清空，取消待完成规划')
            self.lidar = dict(status='IDLE',domain='simulation',live=False,usable=False,
                              authorizes_motion=False,reason='已清空回放；旧雷达计划不可继续执行')
            values = self.draft.model_dump()
            values['obstacles'] = [item for item in values['obstacles'] if not item['id'].startswith('lidar_')]
            self.draft = PlanInput.model_validate(values)
            self.draft_revision += 1
            self.field = make_field(self.draft).data
            self._persist()
            return dict(ok=True,domain='simulation',draft=self.draft.model_dump(),draft_revision=self.draft_revision)
        if arguments.action == 'replay':
            request = arguments.parameters or self.draft
            wire, profile = fixture(arguments.case,request.start,[edge[0] for edge in make_field(request).data['edges']])
            if arguments.wire_hex:
                wire = bytes.fromhex(arguments.wire_hex)
                profile.update(source='external_unregistered',synthetic=False,map_transform=None,
                               coverage=[],coverage_kind='unknown',empty_coverage_valid=False)
            self.planning_cancel.set()
            if self.job['status'] == 'PLANNING':
                self.job = dict(status='IDLE',reason='新扫描取代未完成规划')
            self.execution_check['visible'] = False
            self.lidar = replay(wire,profile,request.start)
            self.lidar['case'] = 'external' if arguments.wire_hex else arguments.case
            folder = self.runtime/'lidar'
            folder.mkdir(exist_ok=True)
            write_json(folder/f'{self.lidar["id"]}.json',dict(scan=self.lidar,wire_hex=wire.hex(),profile=profile))
            self.record('雷达离线回放',f'{self.lidar["status"]} / {self.lidar["reason"]}')
            return dict(ok=True,domain='simulation',scan_id=self.lidar['id'],
                        usable=self.lidar['usable'],reason=self.lidar['reason'])
        if arguments.scan_id != self.lidar.get('id'):
            raise Rejected('扫描ID已过期，不能拿旧观测覆盖当前场景')
        try:
            request, roads = parameters(self.lidar,arguments.parameters)
        except ValueError as exc:
            raise Rejected(str(exc)) from exc
        self.lidar['road_states'] = roads
        self.lidar['binding'] = True
        return self._queue_plan(request, source(self.lidar,roads))

    def _queue_plan(self, arguments, observation=None):
        if observation is None and self.lidar.get('binding'):
            from ..lidar.planning import parameters, source
            try:
                arguments, roads = parameters(self.lidar,arguments)
            except ValueError as exc:
                raise Rejected('当前雷达场景仍绑定：'+str(exc)+'；需改区时先清空回放') from exc
            self.lidar['road_states'] = roads
            observation = source(self.lidar,roads)
        self.planning_cancel.set()
        self.planning_cancel = threading.Event()
        self.execution_check['visible'] = False
        self.draft = normalize(arguments)
        self.draft_revision += 1
        self.field = make_field(self.draft).data
        job_id = uuid.uuid4().hex
        self.job = dict(id=job_id,status='PLANNING',reason='正式规划器后台计算中')
        self._persist()
        self.executor.submit(self._plan,job_id,self.draft,self.planning_cancel,observation)
        self.record('雷达规划提交' if observation else '规划提交',f'{arguments.source} / {arguments.speed_m_s:.2f}m/s')
        return dict(ok=True,domain='simulation',job_id=job_id,draft=self.draft.model_dump(),draft_revision=self.draft_revision)

    def command(self, value):
        command = Command.model_validate(value)
        arguments = ARGUMENTS[command.op].model_validate(command.args)
        with self.lock:
            self._expire()
            if command.op == 'stop':
                self.execution_cancel.set()
                self.program.cancel('工作台停止')
                self.transfer.cancel('工作台停止')
                self.sim.halt('工作台停止')
                self.mech.stop('工作台停止，保持夹爪估计')
                self.owner, self.token = '', ''
                self.record('停止', '撤销控制权；迟到点动不能重新启动')
                return dict(ok=True, domain='simulation')
            if command.op == 'claim':
                if self.token and (self.owner != command.client or not secrets.compare_digest(command.token, self.token)):
                    raise Rejected('另一浏览器持有控制权')
                if not self.token:
                    self.owner, self.token, self.sequence = command.client, secrets.token_hex(24), -1
                    self.record('取得控制权', command.client[:8])
                self.lease_until = self.clock() + 1.5
                return dict(ok=True, token=self.token, lease_s=1.5, domain='simulation')
            if not self.token or not secrets.compare_digest(command.token, self.token) or command.client != self.owner:
                raise Rejected('控制权已过期或属于另一浏览器，请重新获取')
            key = (self.token, command.request_id)
            encoded = command.model_dump_json()
            if key in self.cache:
                old, result = self.cache[key]
                if old != encoded:
                    raise Rejected('同一请求ID不能更改内容')
                return copy.deepcopy(result)  # 重发不延长点动有效期。
            if command.sequence <= self.sequence:
                raise Rejected('迟到或乱序命令已拒绝')
            self.sequence = command.sequence
            self.lease_until = self.clock() + 1.5
            result = self._apply(command.op, arguments)
            self.cache[key] = (encoded, result)
            while len(self.cache) > 128:
                self.cache.popitem(last=False)
            return copy.deepcopy(result)

    def _apply(self, op, arguments):
        if self.execution_check['status'] == 'RUNNING' and op not in ('renew','release','verify_cancel'):
            raise Rejected('执行链联调持有底盘模型，不能插入动作或修改路线')
        if self.program.busy and op not in ('program','renew','release'):
            raise Rejected('工位姿态流程持有机构模型，不能插入动作或导航')
        if op == 'verify_cancel':
            self.execution_cancel.set()
            return dict(ok=True,domain='simulation')
        if op in ('plan','run','reset','move','manual','defaults','load'):
            self.execution_check['visible'] = False
        if op == 'lidar':
            return self._lidar(arguments)
        if op == 'verify':
            self._idle()
            if not self.preview_available:
                raise Rejected('需要重建并重载PC离线核心，未打开任何设备')
            plan = self.plans.get(arguments.plan_id)
            if (plan is None or self.job.get('status') != 'READY' or self.job['plan']['id'] != arguments.plan_id
                    or plan.view['parameters'] != self.draft.model_dump()):
                raise Rejected('计划版本或草稿不一致，请先重新规划')
            self._scan_matches(plan)
            job_id = uuid.uuid4().hex
            self.execution_cancel = threading.Event()
            self.execution_check = dict(schema=1,domain='simulation',status='RUNNING',stage='validating',
                id=job_id,source_plan_id=plan.view['id'],total=len(plan.specs),segment=0,elapsed_s=0.,trace=[],
                authorizes_motion=False,physical_validated=False,stop_confirmed=False,visible=True)
            write_json(self.runtime/'execution_check.json',self.execution_check)
            self.executor.submit(self._verify,job_id,plan,self.execution_cancel)
            self.record('执行链离线联调','合成PC1/PC0；正式Comm/Start/Exec；不接设备')
            return dict(ok=True,job_id=job_id,domain='simulation')
        if self.transfer.busy and op not in ('transfer','mechanism','renew','release'):
            raise Rejected('离线转存尚未结束，禁止修改或启动其他动作')
        if op == 'program':
            if arguments.action == 'stop':
                self.program.cancel('操作员停止工位流程')
                return dict(ok=True,domain='simulation')
            if self.program.busy or self.mech.busy or not self.sim.stopped() or self.sim.mode == 'FAILED':
                raise Rejected('先等待底盘及机构模型停车，且无运行中的流程')
            plan = self.plans.get(self.job.get('plan',{}).get('id')) if self.job['status']=='READY' else None
            try:
                if arguments.action == 'save':
                    record = self.program.save(arguments,plan)
                else:
                    record = self.program.selected(arguments)
                if arguments.action == 'compile':
                    if plan:
                        self._scan_matches(plan)
                    result = self.program.compile(record,plan)
                elif arguments.action == 'run':
                    self.program.run(record)
                    result = dict(status='ACCEPTED',domain='simulation')
                else:
                    result = dict(program=record,domain='simulation')
            except ValueError as exc:
                raise Rejected(str(exc)) from exc
            self.record('工位流程'+arguments.action,arguments.id+f' v{record["revision"]}')
            return dict(ok=True,**result)
        if op == 'transfer':
            if not self.sim.stopped() or self.sim.mode == 'FAILED':
                raise Rejected('转存流程要求底盘模型已停车且无故障')
            try:
                result=self.transfer.command(arguments)
            except ValueError as exc:
                raise Rejected(str(exc)) from exc
            self.record('离线转存请求', arguments.action)
            return result
        if op == 'mechanism':
            if self.transfer.busy:
                if arguments.action not in ('stop','invalidate'):
                    raise Rejected('转存流程持有机构模型，不能插入单轴或示教动作')
                self.transfer.cancel('机构模型停止/基准失效')
            if arguments.action not in ('stop', 'invalidate'):
                if not self.sim.stopped() or self.sim.mode == 'FAILED':
                    raise Rejected('底盘模型未停稳或故障未处理，机构互锁拒绝')
            try:
                renewed = self.mech.renew_jog(arguments)
                result = renewed or self.mech.command(arguments)
            except ValueError as exc:
                self.record('机构请求拒绝', str(exc))
                raise Rejected(str(exc)) from exc
            if not renewed:
                self.record('机构离线请求', f'{arguments.action} / {result.get("action_id", "配置")}')
            return result
        if op == 'release':
            self.execution_cancel.set()
            self.program.cancel('释放离线控制权')
            self.transfer.cancel('释放离线控制权')
            if self.mech.busy:
                self.mech.stop('释放控制权，机构调试已停止')
            if self.sim.mode == 'MANUAL':
                self.sim.halt('已释放控制权')
            self.owner, self.token = '', ''
        elif op == 'manual_stop':
            if self.sim.mode == 'MANUAL':
                self.sim.halt('方向控制已松开')
                self.record('点动停车', '保留控制权；拒绝更早序号的迟到点动')
        elif op == 'manual':
            if self.sim.mode != 'MANUAL':
                self._idle()
            self.sim.jog((arguments.vx_m_s, arguments.vy_m_s, arguments.wz_rad_s))
            self.manual_until = self.clock() + .35
        elif op == 'move':
            self._idle()
            if max(abs(arguments.x_m), abs(arguments.y_m), abs(arguments.yaw_rad)) == 0:
                raise Rejected('目标不能全为零')
            self.sim.move(arguments)
            self.record('定量动作开始', str(arguments.model_dump()))
        elif op == 'reset':
            if self.mech.busy:
                raise Rejected('机构运行中不能重置底盘模型')
            if not self.sim.stopped():
                raise Rejected('请先停车并等待模型轮速稳定')
            self.sim.reset(arguments.start)
            self.record('重置离线场景', arguments.start)
        elif op == 'plan':
            return self._queue_plan(arguments)
        elif op == 'defaults':
            if self.mech.busy:
                raise Rejected('机构运行中不能恢复初始场景')
            if not self.sim.stopped():
                raise Rejected('请先停止模型，再恢复初始设置')
            self.planning_cancel.set()
            self.draft = normalize(PlanInput())
            self.draft_revision += 1
            self.field = make_field(self.draft).data
            self.job = dict(status='IDLE')
            self.lidar = dict(status='IDLE',domain='simulation',live=False,usable=False,
                              authorizes_motion=False,reason='已恢复初始场景，旧扫描不再生效')
            self.sim.reset('start_1')
            self._persist()
            self.record('恢复初始设置', '起点1、0.30m/s、100RPM、工位间隙50mm、清空障碍；保留历史版本')
        elif op == 'load':
            if self.mech.busy:
                raise Rejected('机构运行中不能读取导航场景')
            if not self.sim.stopped():
                raise Rejected('请先停车再读取场景版本')
            path = self.runtime / 'plans' / f'{arguments.plan_id}.json'
            try:
                plan = restore_plan(json.loads(path.read_text(encoding='utf-8')), self.library)
                if plan.view['id'] != arguments.plan_id:
                    raise ValueError('文件名与版本ID不一致')
            except (OSError, KeyError, TypeError, ValueError) as exc:
                raise Rejected(f'版本读取失败：{exc}') from exc
            self.planning_cancel.set()
            self.draft = PlanInput.model_validate(plan.view['parameters'])
            self.draft_revision += 1
            self.field = plan.view['field']
            self.plans[plan.view['id']] = plan
            self.job = dict(id=plan.view['id'], status='READY', plan=plan.view, restored=True)
            self.sim.reset(self.draft.start)
            self._persist()
            self.record('读取离线版本', plan.view['id'])
        elif op in ('run', 'save'):
            plan = self.plans.get(arguments.plan_id)
            if plan is None:
                raise Rejected('路线版本不存在或已过期，请重新规划')
            if op == 'run':
                self._idle()
                self._scan_matches(plan)
                if (self.job['status'] != 'READY' or self.job['plan']['id'] != arguments.plan_id
                        or plan.view['parameters'] != self.draft.model_dump()):
                    raise Rejected('当前草稿与目标版本不一致，请重新规划或读取该版本')
                self.sim.run(plan)
                self.record('离线路线开始', f'{len(plan.specs)}段 / {plan.view["sha256"][:10]}')
            else:
                target = self.runtime / 'plans'
                target.mkdir(exist_ok=True)
                path = target / f'{plan.view["id"]}.json'
                # 每个版本独立保存，不覆盖历史成功配置或实车白名单。
                if not path.exists():
                    write_json(path, payload(plan))
                self.saved = versions(self.runtime)
                self.record('保存离线版本', plan.view['id'])
        result = dict(ok=True, domain='simulation')
        if op in ('load', 'defaults'):
            result.update(draft=self.draft.model_dump(), draft_revision=self.draft_revision)
        return result

    def _plan(self, job_id, arguments, cancel, observation=None):
        def progress(value):
            with self.lock:
                if self.job.get('id') == job_id and not cancel.is_set():
                    self.job['progress'] = value
        try:
            plan = make_plan(self.library, arguments, cancelled=cancel.is_set, progress=progress)
            if observation:
                plan.view['lidar_source'] = copy.deepcopy(observation)
            with self.lock:
                if self.closed or cancel.is_set() or self.job.get('id') != job_id:
                    return
                self.plans[plan.view['id']] = plan
                while len(self.plans) > 8:
                    self.plans.popitem(last=False)
                self.job = dict(id=job_id, status='READY', plan=plan.view)
                self._persist()
                self.record('规划完成', f'{len(plan.specs)}段 / 参考估计{plan.view["seconds"]:.2f}s')
        except Exception as exc:
            with self.lock:
                if self.closed or cancel.is_set() or self.job.get('id') != job_id:
                    return
                self.job = dict(id=job_id, status='FAILED', reason=str(exc))
                self.record('规划失败', str(exc))

    def _verify(self, job_id, plan, cancel):
        from ..nav.bridge import prepare
        from ..device.preview import execute
        def progress(value):
            with self.lock:
                if self.execution_check.get('id') == job_id:
                    self.execution_check.update(value)
        try:
            bundle = prepare(plan,self.library)
            folder = self.runtime/'execution_checks'/job_id
            folder.mkdir(parents=True,exist_ok=True)
            write_json(folder/'bundle.json',bundle)
            report = execute(self.library,bundle,folder,progress=progress,cancel=cancel.is_set)
            result = dict(status=report['status'],reason=report['reason'],elapsed_s=report['elapsed_s'],
                stop_confirmed=report['stop_confirmed'],trace=report['trace'],
                rpm=report['rpm'],execution=report['execution'],
                completed=len(report['result']['completed_segments']) if report['result'] else 0,
                bundle_sha256=bundle['sha256'],sweep_samples=report['sweep_samples'])
        except Exception as exc:
            result = dict(status='FAILED',reason=str(exc),stop_confirmed=False)
        with self.lock:
            if self.execution_check.get('id') == job_id:
                self.execution_check.update(result,authorizes_motion=False,physical_validated=False)
                write_json(self.runtime/'execution_check.json',self.execution_check)
                self.record('执行链离线结果',f'{result["status"]} / {result.get("reason", "")}')

    def tick(self, gap_s=.01):
        with self.lock:
            before = self.sim.mode
            mech_before = self.mech.mode
            self._expire()
            if self.mech.busy and (not self.sim.stopped() or self.sim.mode == 'FAILED'):
                self.mech.fail('执行中底盘未停稳或发生故障')
            self.mech.tick(gap_s)
            self.program.tick(self.sim.stopped() and self.sim.mode != 'FAILED')
            if self.transfer.busy and gap_s <= .1:
                # 仅离线演示加速模型积分，不改变控制循环/反馈超时或实车参数。
                for _ in range(self.transfer.frozen['configuration']['pace']-1):
                    self.mech.tick(gap_s)
            if self.transfer.busy and gap_s > .1:
                self.transfer.cancel('离线控制循环超时',failed=True)
            self.transfer.tick(self.sim.stopped() and self.sim.mode != 'FAILED')
            if mech_before != self.mech.mode and self.mech.mode in ('COMPLETE', 'FAILED', 'STOPPED'):
                self.record('机构模型结果', f'{self.mech.mode} / {self.mech.reason}')
            if gap_s > .1 and self.sim.mode in ('MANUAL', 'RUNNING'):
                self.sim.fail('离线控制循环超时')
                self.record('控制周期超时', f'{gap_s:.3f}s')
            self.sim.step()
            if before != self.sim.mode and self.sim.mode in ('COMPLETE', 'FAILED', 'STOPPED'):
                self.record(dict(COMPLETE='离线动作完成', FAILED='离线动作失败', STOPPED='模型停车确认')[self.sim.mode],
                            f'已完成{len(self.sim.results)}段；{self.sim.reason}')

    def snapshot(self):
        with self.lock:
            self._expire()
            return dict(available=True, domain='simulation', real_connected=False,
                        server_at_s=self.clock(), field=self.field, simulation=self.sim.view(),
                        owner=dict(client=self.owner, remaining_s=max(0, self.lease_until - self.clock()) if self.token else 0),
                        planning=copy.deepcopy(self.job),
                        draft=self.draft.model_dump(), draft_revision=self.draft_revision,
                        saved_versions=list(self.saved),
                        active_plan=copy.deepcopy(self.sim.plan.view) if self.sim.plan else None,
                        events=list(self.events), log_error=self.log_error,
                        mechanism=self.mech.view(), transfer=self.transfer.view(),
                        program=self.program.view(self.plans.get(self.job.get('plan',{}).get('id')) if self.job['status']=='READY' else None),
                        integration=copy.deepcopy(self.preparation),
                        execution_check=dict(copy.deepcopy(self.execution_check),available=self.preview_available),
                        lidar=copy.deepcopy(self.lidar),
                        capabilities=dict(manual=True, move=True, planning=True,
                                          real_motion=False, mechanism=True))

    def close(self):
        with self.lock:
            self.closed = True
            self.execution_cancel.set()
            self.program.cancel('离线核心退出')
            self.planning_cancel.set()
            self.sim.halt('离线核心退出')
            self.transfer.cancel('离线核心退出')
            self.mech.stop('离线核心退出')
        self.executor.shutdown(wait=False, cancel_futures=True)
