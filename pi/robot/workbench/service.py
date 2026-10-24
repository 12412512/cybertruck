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

from .contracts import ARGUMENTS, Command, MECHANISM
from .planning import make_plan
from .simulation import Simulation
from ..nav.field import Field


class Rejected(ValueError):
    pass


class Workbench:
    def __init__(self, library, runtime, clock=time.monotonic):
        self.library, self.runtime, self.clock = Path(library), Path(runtime), clock
        self.runtime.mkdir(parents=True, exist_ok=True)
        self.lock = threading.RLock()
        self.sim = Simulation(self.library)
        self.owner, self.token, self.lease_until, self.sequence = '', '', 0., -1
        self.manual_until = 0.
        self.cache = OrderedDict()
        self.events = deque(maxlen=100)
        self._load_events()
        self.plans = OrderedDict()
        self.job = dict(status='IDLE')
        self.executor = ThreadPoolExecutor(max_workers=1, thread_name_prefix='route-plan')
        self.closed = False
        self.log_error = ''
        self.field = Field().data
        self.record('核心启动', '离线模型；真实设备接口未启用')

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
            self.owner, self.token = '', ''

    def _idle(self):
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
            raise Rejected('机械适配器尚未接入；未发送目标、未建立零点')
        if op == 'release':
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
            if not self.sim.stopped():
                raise Rejected('请先停车并等待模型轮速稳定')
            self.sim.reset(arguments.start)
            self.record('重置离线场景', arguments.start)
        elif op == 'plan':
            if self.job['status'] == 'PLANNING':
                raise Rejected('已有规划正在计算，请等待结果')
            job_id = uuid.uuid4().hex
            self.job = dict(id=job_id, status='PLANNING', reason='正式规划器后台计算中')
            self.executor.submit(self._plan, job_id, arguments)
            self.record('规划提交', f'{arguments.source} / {arguments.speed_m_s:.2f}m/s')
            return dict(ok=True, job_id=job_id)
        elif op in ('run', 'save'):
            plan = self.plans.get(arguments.plan_id)
            if plan is None:
                raise Rejected('路线版本不存在或已过期，请重新规划')
            if op == 'run':
                self._idle()
                self.sim.run(plan)
                self.record('离线路线开始', f'{len(plan.specs)}段 / {plan.view["sha256"][:10]}')
            else:
                target = self.runtime / 'plans'
                target.mkdir(exist_ok=True)
                path = target / f'{plan.view["id"]}.json'
                # 每个版本独立保存，不覆盖历史成功配置或实车白名单。
                if not path.exists():
                    path.write_text(json.dumps(dict(plan.view, specs=[s.hex() for s in plan.specs]),
                                               ensure_ascii=False, indent=2), encoding='utf-8')
                self.record('保存离线版本', plan.view['id'])
        return dict(ok=True, domain='simulation')

    def _plan(self, job_id, arguments):
        try:
            plan = make_plan(self.library, arguments)
            with self.lock:
                if self.closed:
                    return
                self.plans[plan.view['id']] = plan
                while len(self.plans) > 8:
                    self.plans.popitem(last=False)
                self.job = dict(id=job_id, status='READY', plan=plan.view)
                self.record('规划完成', f'{len(plan.specs)}段 / 参考估计{plan.view["seconds"]:.2f}s')
        except Exception as exc:
            with self.lock:
                self.job = dict(id=job_id, status='FAILED', reason=str(exc))
                self.record('规划失败', str(exc))

    def tick(self, gap_s=.01):
        with self.lock:
            before = self.sim.mode
            self._expire()
            if gap_s > .1 and self.sim.mode in ('MANUAL', 'RUNNING'):
                self.sim.halt('离线控制循环超时')
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
                        active_plan=copy.deepcopy(self.sim.plan.view) if self.sim.plan else None,
                        events=list(self.events), log_error=self.log_error,
                        mechanism=copy.deepcopy(MECHANISM),
                        capabilities=dict(manual=True, move=True, planning=True,
                                          real_motion=False, mechanism=False))

    def close(self):
        with self.lock:
            self.closed = True
            self.sim.halt('离线核心退出')
        self.executor.shutdown(wait=False, cancel_futures=True)
