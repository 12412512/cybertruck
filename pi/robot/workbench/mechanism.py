"""机构离线状态机：无设备依赖，零点/角度估计不会写入真实状态。"""

import copy
import hashlib
import json
import math
from pathlib import Path

from .storage import write_json


CONFIG = Path(__file__).resolve().parents[3] / 'config' / 'mechanism_preview.json'
AXES = ('lift', 'reach')
SERVOS = ('arm', 'tray', 'gripper')


class Mechanism:
    def __init__(self, runtime, clock, config=CONFIG):
        self.clock, self.runtime = clock, Path(runtime)
        self.cfg = json.loads(Path(config).read_text(encoding='utf-8'))
        self.config_hash = hashlib.sha256(json.dumps(self.cfg, sort_keys=True).encode()).hexdigest()
        self.positions = dict(lift=0., reach=0., arm=None, tray=None, gripper=None)
        self.hold_possible = False
        self.zero_valid, self.zero_version = False, 0
        self.mode, self.fault, self.reason = 'IDLE', '', '先建立模拟基准，再显式锚定舵机估计'
        self.active, self.queue, self.segment = None, [], None
        self.action_id, self.results = 0, []
        self.updated = self.clock()
        self.pose_versions = []
        self.storage_error = ''
        self._load()

    @property
    def busy(self):
        return self.active is not None

    def _load(self):
        try:
            value = json.loads((self.runtime / 'mechanism_poses.json').read_text(encoding='utf-8'))
            if value['config_hash'] != self.config_hash:
                raise ValueError('机构配置已变化，示教历史不自动启用')
            for pose in value['poses']:
                self._targets(pose['targets'])
                if pose['sha256'] != self._hash(pose['targets']):
                    raise ValueError('示教校验失败')
            self.pose_versions = value['poses'][-100:]
        except FileNotFoundError:
            pass
        except (OSError, ValueError, KeyError, TypeError) as exc:
            self.storage_error = str(exc)

    def _hash(self, value):
        return hashlib.sha256(json.dumps(value, sort_keys=True).encode()).hexdigest()

    def _ready(self):
        if self.fault:
            raise ValueError(f'机构模型故障保留：{self.fault}；请显式失效并重建模拟基准')
        if not self.zero_valid:
            raise ValueError('模拟零点未建立；真实机构尚未接入')
        if self.clock() - self.updated > self.cfg['feedback_max_age_s']:
            raise ValueError('机构模型反馈超过250ms')

    def _bounds(self, axis):
        if axis in AXES:
            item = self.cfg['axes'][axis]
            return item['min_m'], item['max_m']
        item = self.cfg['servos'][axis]
        return math.radians(item['min_deg']), math.radians(item['max_deg'])

    def _target(self, axis, value):
        if axis not in AXES + SERVOS or value is None or not math.isfinite(value):
            raise ValueError('轴或目标无效')
        low, high = self._bounds(axis)
        if not low <= value <= high:
            raise ValueError(f'{axis}目标超过离线软限位')
        return value

    def _targets(self, targets):
        if set(targets) != set(AXES + SERVOS):
            raise ValueError('姿态必须包含两个直线轴和三个舵机')
        for axis, value in targets.items():
            self._target(axis, value)
        self._endpoint(targets)

    def _endpoint(self, state):
        rule = self.cfg['interlocks']
        if state['reach'] > rule['rotation_reach_max_m']:
            if (state['lift'] < rule['extension_lift_min_m'] or state['arm'] is None
                    or abs(state['arm'] - math.radians(rule['extension_arm_deg'])) > .001):
                raise ValueError('伸出互锁：升降须下降至少45mm，转臂须在180°')

    def _check(self, axis, target, state):
        self._target(axis, target)
        rule = self.cfg['interlocks']
        if abs(target - state[axis]) < 1e-9:
            return
        if axis in ('arm', 'tray'):
            if state['lift'] > rule['rotation_lift_max_m'] or state['reach'] > rule['rotation_reach_max_m']:
                raise ValueError('旋转互锁：先收回伸缩并将升降回到顶部1mm内')
            if axis == 'tray' and (state['arm'] is None or abs(state['arm'] - math.radians(rule['extension_arm_deg'])) > .001):
                raise ValueError('料盘互锁：转臂先到180°避让')
        if axis == 'lift' and state['reach'] > rule['rotation_reach_max_m']:
            raise ValueError('升降互锁：伸缩轴尚未收回1mm内')
        future = dict(state, **{axis: target})
        self._endpoint(future)

    def stop(self, reason='操作员停止', failed=False):
        if self.active:
            self._finish('FAILED' if failed else 'CANCELLED', reason)
        self.queue, self.segment = [], None
        self.mode = 'FAILED' if failed or self.fault else 'STOPPED'
        self.reason = reason  # 保留夹爪及其他命令估计，不自动张爪、回零或收臂。

    def fail(self, reason):
        self.fault = self.fault or reason
        self.stop(reason, failed=True)

    def _finish(self, status, evidence):
        if status == 'COMPLETE' and self.active.get('releases_hold'):
            self.hold_possible = False
        self.results.append(dict(action_id=self.active['id'], status=status, evidence=evidence,
                                 domain='simulation', positions=dict(self.positions)))
        self.results = self.results[-40:]
        self.active = None
        self.segment = None
        self.mode = 'COMPLETE' if status == 'COMPLETE' else status
        self.reason = evidence

    def _start(self, steps, action, release_confirmed=False):
        state = dict(self.positions)
        for axis, target in steps:
            if state[axis] is None:
                raise ValueError('舵机估计尚未锚定')
            self._check(axis, target, state)
            if axis == 'gripper' and self.hold_possible and target < self._bounds('gripper')[1] - 1e-6 and not release_confirmed:
                raise ValueError('保夹互锁：张爪前需明确允许释放（仅模拟）')
            state[axis] = target
        self.action_id += 1
        self.active = dict(id=self.action_id, action=action, status='ACCEPTED', step=0,
                           total=len(steps), zero_version=self.zero_version, started=self.clock())
        for axis, target in steps:
            if axis == 'gripper':
                if target >= self._bounds('gripper')[1] - 1e-6:
                    self.hold_possible = True  # 保持命令即锁存；中途取消不能假定物料已脱离。
                elif release_confirmed:
                    self.active['releases_hold'] = True
        self.queue = list(steps)
        self.mode, self.reason = 'RUNNING', '已接受；等待离线调度'
        return dict(ok=True, action_id=self.action_id, status='ACCEPTED', steps=steps, domain='simulation')

    def _pose_steps(self, targets):
        self._targets(targets)
        state = dict(self.positions)
        steps = []
        # 复用旧流程先收回/抬升再旋转的逐步思想；每步模型停稳再交接，不同时运行全部轴。
        for axis, value in [('reach', 0.), ('lift', 0.), ('arm', math.radians(self.cfg['interlocks']['extension_arm_deg'])),
                            ('tray', targets['tray']), ('arm', targets['arm']),
                            ('lift', targets['lift']), ('reach', targets['reach']),
                            ('gripper', targets['gripper'])]:
            self._check(axis, value, state)
            if abs(state[axis] - value) > 1e-9:
                steps.append((axis, value))
            state[axis] = value
        return steps

    def command(self, goal):
        action = goal.action
        if action == 'stop':
            self.stop('机构独立停止')
            return dict(ok=True, domain='simulation')
        if action == 'invalidate':
            self.stop('模拟基准已失效')
            self.zero_valid = False
            return dict(ok=True, domain='simulation')
        if self.busy:
            raise ValueError('机构动作运行中，请先停止或等待完成')
        if action == 'reset_model':
            if not goal.confirmed:
                raise ValueError('需明确确认丢弃机构模型位置估计')
            self.positions = dict(lift=0., reach=0., arm=None, tray=None, gripper=None)
            self.hold_possible = False
            self.zero_valid = False
            self.fault, self.mode = '', 'IDLE'
            self.reason = '离线模型已复位，基准无效；示教历史保留，无实车输出'
            return dict(ok=True, domain='simulation')
        if action == 'zero':
            if not goal.confirmed:
                raise ValueError('需明确确认只重建模拟基准')
            if any(abs(self.positions[axis]) > 1e-6 for axis in AXES):
                raise ValueError('模型不在最高/最近端；先返回基准，不能假清零掩盖位移')
            self.zero_valid = True
            self.zero_version += 1
            self.fault, self.mode, self.reason = '', 'IDLE', '模拟最高/最近端基准已建立；不是实车建零'
            self.updated = self.clock()
            return dict(ok=True, zero_version=self.zero_version, domain='simulation')
        self._ready()
        if goal.zero_version != self.zero_version:
            raise ValueError('零点版本已变化，旧目标拒绝执行')
        if action == 'anchor':
            if not goal.confirmed or any(abs(self.positions[a]) > 1e-6 for a in AXES):
                raise ValueError('舵机估计锚定需明确确认且直线轴在基准')
            if any(self.positions[a] is not None for a in SERVOS):
                raise ValueError('已有舵机估计，禁止通过重新锚定跳过动作')
            for axis in SERVOS:
                self.positions[axis] = math.radians(self.cfg['servos'][axis]['anchor_deg'])
            self.reason = '仅设置模拟初始估计180°/30°/0°；无PWM输出'
            return dict(ok=True, domain='simulation')
        if any(self.positions[a] is None for a in SERVOS):
            raise ValueError('请先显式锚定模拟舵机估计')
        if action in ('teach', 'preview'):
            targets = goal.targets
            self._targets(targets)
            steps = self._pose_steps(targets)
            if action == 'preview':
                return dict(ok=True, steps=steps, domain='simulation', physical_validated=False)
            if not goal.pose_id or not goal.name:
                raise ValueError('示教需填写姿态编号和名称')
            previous = [p for p in self.pose_versions if p['id'] == goal.pose_id]
            revision = max((p['revision'] for p in previous), default=0) + 1
            if goal.revision != revision - 1:
                raise ValueError('姿态历史已变化，请读取最新版本后再保存')
            pose = dict(id=goal.pose_id, name=goal.name, revision=revision, targets=dict(targets),
                        sha256=self._hash(targets), config_hash=self.config_hash,
                        zero_version=self.zero_version, physical_validated=False, domain='simulation')
            history = (self.pose_versions + [pose])[-100:]
            write_json(self.runtime / 'mechanism_poses.json', dict(config_hash=self.config_hash, poses=history))
            self.pose_versions = history
            return dict(ok=True, pose=pose, domain='simulation')
        if action == 'pose':
            selected = next((p for p in self.pose_versions if p['id'] == goal.pose_id and p['revision'] == goal.revision), None)
            if not selected:
                raise ValueError('姿态版本不存在；候选姿态需先预览并保存')
            return self._start(self._pose_steps(selected['targets']), 'pose', goal.release_confirmed)
        axis = goal.axis
        if action == 'jog':
            if axis not in AXES or not goal.velocity_m_s:
                raise ValueError('点动仅用于直线轴，速度不能为零')
            speed = goal.velocity_m_s
            if abs(speed) > self.cfg['axes'][axis]['speed_m_s']:
                raise ValueError('点动速度超过离线限制')
            target = self._bounds(axis)[int(speed > 0)]
        else:
            target = goal.target_m if axis in AXES else goal.target_rad
        result = self._start([(axis, target)], action, goal.release_confirmed)
        if action == 'jog':
            self.active.update(lease_until=self.clock() + self.cfg['jog_lease_s'], jog_speed=abs(speed),
                               direction=math.copysign(1, speed), velocity=0.)
        return result

    def renew_jog(self, goal):
        if (self.busy and self.queue and self.active['action'] == 'jog' and goal.action == 'jog'
                and goal.zero_version == self.zero_version and goal.axis == self.queue[0][0]
                and goal.velocity_m_s and self.active['direction'] == math.copysign(1, goal.velocity_m_s)
                and abs(goal.velocity_m_s) == self.active['jog_speed']):
            self.active['lease_until'] = self.clock() + self.cfg['jog_lease_s']
            return dict(ok=True, action_id=self.active['id'], status='RUNNING', domain='simulation')
        return None

    def tick(self, dt=.01):
        now = self.clock()
        if self.busy and (dt > .1 or now - self.updated > self.cfg['feedback_max_age_s']):
            self.fail('机构模型控制/反馈过期')
        self.updated = now
        if not self.busy:
            return
        if not self.zero_valid or self.active['zero_version'] != self.zero_version:
            self.fail('执行中零点失效')
            return
        if self.active['action'] == 'jog' and now >= self.active['lease_until']:
            self.stop('机构点动350ms续约超时')
            return
        if not self.queue:
            self._finish('COMPLETE', '各步骤已按模型位置/稳定窗口完成；无实物证据')
            return
        axis, target = self.queue[0]
        if self.active['action'] == 'jog':
            # 点动按受限加速推进；松开/续约失效即取消，模型不模拟轴刹车惯性。
            desired = self.active['jog_speed']
            velocity = min(desired, self.active['velocity'] + self.cfg['axes'][axis]['accel_m_s2'] * dt)
            self.active['velocity'] = velocity
            current = self.positions[axis]
            next_value = current + self.active['direction'] * velocity * dt
            next_value = min(next_value, target) if self.active['direction'] > 0 else max(next_value, target)
            self._check(axis, next_value, self.positions)
            self.positions[axis] = next_value
            self.active['status'] = 'RUNNING'
            if abs(next_value - target) < 1e-9:
                self.queue = []
                self._finish('COMPLETE', '机构点动到达模型软限位，已停止')
            return
        if self.segment is None:
            start = self.positions[axis]
            self._check(axis, target, self.positions)
            if axis in AXES:
                speed = self.active.get('jog_speed', self.cfg['axes'][axis]['speed_m_s'])
                accel = self.cfg['axes'][axis]['accel_m_s2']
                duration = max(.1, abs(target - start) * 1.875 / speed,
                               math.sqrt(abs(target - start) * 5.774 / accel))
            else:
                duration = max(.1, abs(target - start) * 1.875 / math.radians(self.cfg['servos'][axis]['rate_deg_s']))
            self.segment = dict(start=start, duration=duration, elapsed=0.)
            self.active.update(status='RUNNING', direction=math.copysign(1, target - start))
        segment = self.segment
        segment['elapsed'] += dt
        ratio = min(1., segment['elapsed'] / segment['duration'])
        blend = ratio ** 3 * (10 - 15 * ratio + 6 * ratio ** 2)
        self.positions[axis] = segment['start'] + (target - segment['start']) * blend
        settle = .1 if axis in AXES else self.cfg['settle_s']
        if segment['elapsed'] >= segment['duration'] + settle:
            self.positions[axis] = target
            self.queue.pop(0)
            self.active['step'] += 1
            self.segment = None

    def transport_ready(self):
        rule = self.cfg['interlocks']
        return (self.zero_valid and not self.busy and not self.fault
                and self.positions['lift'] <= rule['rotation_lift_max_m']
                and self.positions['reach'] <= rule['rotation_reach_max_m']
                and self.positions['arm'] is not None
                and abs(self.positions['arm'] - math.radians(rule['extension_arm_deg'])) < .001)

    def view(self):
        return dict(available=True, backend='offline_model', domain='simulation', real_connected=False,
                    physical_validated=False, zero_valid=self.zero_valid, zero_version=self.zero_version,
                    mode=self.mode, busy=self.busy, fault=self.fault, reason=self.reason,
                    hold_possible=self.hold_possible,
                    config_hash=self.config_hash, config=self.cfg, storage_error=self.storage_error,
                    transport_ready=self.transport_ready(), active=copy.deepcopy(self.active),
                    results=copy.deepcopy(self.results), poses=copy.deepcopy(self.pose_versions),
                    axes=[dict(id=a, position_m=self.positions[a] if self.zero_valid else None,
                               limits_m=self._bounds(a), feedback='synthetic') for a in AXES],
                    servos=[dict(id=a, estimate_rad=self.positions[a], actual_rad=None,
                                 limits_rad=self._bounds(a), feedback='none',
                                 pulse_us=None if self.positions[a] is None else self.pulse(a)) for a in SERVOS])

    def pulse(self, axis):
        item = self.cfg['servos'][axis]
        degrees = math.degrees(self.positions[axis])
        ratio = degrees / item['range_deg']
        return round(500 + 2000 * (1 - ratio if item['reverse'] else ratio))
