"""唯一离线控制核心：控制权、去重、点动租约、参数版本和规划任务。"""

from collections import OrderedDict, deque
from concurrent.futures import ThreadPoolExecutor
import copy
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


class Rejected(ValueError):
    pass


class Workbench:
    def __init__(self, library, runtime, clock=time.monotonic):
        self.library, self.runtime, self.clock = Path(library), Path(runtime), clock
        self.runtime.mkdir(parents=True, exist_ok=True)
        self.lock = threading.RLock()
        self.sim = Simulation(self.library)
        self.mech = Mechanism(self.runtime, self.clock)
        self.owner, self.token, self.lease_until, self.sequence = '', '', 0., -1
        self.manual_until = 0.
        self.cache = OrderedDict()
        self.events = deque(maxlen=100)
        self._load_events()
        self.plans = OrderedDict()
        self.job = dict(status='IDLE')
        self.planning_cancel = threading.Event()
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
            if self.mech.busy:
                self.mech.stop('控制权过期，机构调试已停止')
            self.owner, self.token = '', ''

    def _idle(self):
        if self.mech.busy:
            raise Rejected('机构运行中，禁止底盘运动')
        if self.mech.zero_version and not self.mech.transport_ready():
            raise Rejected('机构未满足模拟运输互锁：零点有效、升降/伸缩收回、转臂180°')
        if not self.sim.stopped():
            raise Rejected('请先停车并等待模型轮速稳定')
        if self.sim.mode == 'FAILED':
            raise Rejected('模型故障仍保留，请显式重置离线场景')

    def command(self, value):
        command = Command.model_validate(value)
        arguments = ARGUMENTS[command.op].model_validate(command.args)
        with self.lock:
            self._expire()
            if command.op == 'stop':
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
        if op == 'mechanism':
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
            self.planning_cancel.set()
            self.planning_cancel = threading.Event()
            self.draft = normalize(arguments)
            self.draft_revision += 1
            self.field = make_field(self.draft).data
            job_id = uuid.uuid4().hex
            self.job = dict(id=job_id, status='PLANNING', reason='正式规划器后台计算中')
            self._persist()
            self.executor.submit(self._plan, job_id, self.draft, self.planning_cancel)
            self.record('规划提交', f'{arguments.source} / {arguments.speed_m_s:.2f}m/s')
            return dict(ok=True, job_id=job_id)
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

    def _plan(self, job_id, arguments, cancel):
        def progress(value):
            with self.lock:
                if self.job.get('id') == job_id and not cancel.is_set():
                    self.job['progress'] = value
        try:
            plan = make_plan(self.library, arguments, cancelled=cancel.is_set, progress=progress)
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

    def tick(self, gap_s=.01):
        with self.lock:
            before = self.sim.mode
            mech_before = self.mech.mode
            self._expire()
            if self.mech.busy and (not self.sim.stopped() or self.sim.mode == 'FAILED'):
                self.mech.fail('执行中底盘未停稳或发生故障')
            self.mech.tick(gap_s)
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
                        mechanism=self.mech.view(),
                        capabilities=dict(manual=True, move=True, planning=True,
                                          real_motion=False, mechanism=True))

    def close(self):
        with self.lock:
            self.closed = True
            self.planning_cancel.set()
            self.sim.halt('离线核心退出')
            self.mech.stop('离线核心退出')
        self.executor.shutdown(wait=False, cancel_futures=True)
