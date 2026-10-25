"""实车机构协议：唯一会话、显式整数目标、无自动建零或动作重放。"""

import math
import struct
from collections import deque
from .link import Link
from . import protocol as p
from .execution import STATUS_DATA

FIELDS = ('schema uptime_ms attached fault zero_version zero_valid config_mask base_stopped '
          'axis_state axis_reason axis_id servo_state servo_reason servo_id pose_state pose_reason '
          'pose_id revision step_index step_count channel target_mdeg pwm_mask stop_confirmed '
          'lift_um reach_um stable_samples motion_uncertain').split()


def payload(action, args, state, token):
    """网页SI单位只在此转换一次；绝不读取模拟零点或模型位置。"""
    version = args.get('zero_version', 0)
    if type(version) is not int or not 0 <= version <= 0xFFFFFFFF:
        raise ValueError('实车零点版本无效')
    prefix = struct.pack('<II', token, version)
    action_id = max(state.get(key, 0) for key in ('axis_id', 'servo_id', 'pose_id')) + 1
    if action_id >= 0xFFFFFFF0:
        raise ValueError('动作编号耗尽，禁止回绕')
    if action == 'attach':
        return 48, struct.pack('<II', token, 0x4D454348)
    if action == 'enable':
        if args.get('confirmed') is not True or args.get('axis') not in ('lift', 'reach'):
            raise ValueError('轴使能需要独立现场确认')
        return 49, prefix + bytes([int(args['axis'] == 'reach')])
    if action == 'zero':
        return 50, prefix + bytes([args.get('confirmed') is True])
    if action == 'anchor':
        targets = args.get('targets', {})
        angles = [integer(targets.get(key), 1000 * 180 / math.pi) for key in ('tray', 'arm', 'gripper')]
        return 51, prefix + struct.pack('<3iB', *angles, args.get('confirmed') is True)
    if action == 'invalidate':
        return 57, prefix
    if action == 'move':
        actuator = args.get('axis')
        if actuator in ('lift', 'reach'):
            rpm = args.get('rpm', 5)
            if type(rpm) is not int or not 1 <= rpm <= 30:
                raise ValueError('单轴转速必须为1～30RPM')
            return 52, prefix + struct.pack('<IiIHB', action_id,
                integer(args.get('target_m'), 1e6), 15000, rpm, int(actuator == 'reach'))
        channel = dict(tray=1, arm=2, gripper=3).get(actuator)
        if not channel:
            raise ValueError('执行器无效')
        return 53, prefix + struct.pack('<IiIBB', action_id,
            integer(args.get('target_rad'), 180000 / math.pi), 30000, channel,
            args.get('release_confirmed') is True)
    if action == 'confirm':
        if args.get('confirmed') is not True or state.get('servo_state') != 5:
            raise ValueError('只能人工确认当前已完成命令；不能用计时或模型确认')
        return 54, prefix + struct.pack('<IIIiBI', state['servo_id'], token,
            state['uptime_ms'], state['target_mdeg'], state['channel'], 1)
    if action == 'next':
        if args.get('action_id') != state.get('pose_id') or args.get('revision') != state.get('revision') or args.get('step_index') != state.get('step_index'):
            raise ValueError('继续必须绑定当前实车姿态、版本和步骤')
        return 56, prefix + struct.pack('<IIB', args['action_id'], args['revision'], args['step_index'])
    if action == 'pose':
        targets = args.get('targets', {})
        if args.get('physical_taught') is not True or not args.get('revision'):
            raise ValueError('只允许显式确认的实车示教版本，禁止直接执行模型示教')
        values = [integer(targets.get(key), 1e6 if key in ('lift', 'reach') else 180000 / math.pi)
                  for key in ('lift', 'reach', 'arm', 'tray', 'gripper')]
        return 55, prefix + struct.pack('<III5iBB', action_id, args['revision'], 120000,
            *values, args.get('single_step', True) is True, args.get('release_confirmed') is True)
    raise ValueError('不支持的实车动作；本版不开放连续点动')


def integer(value, scale):
    if type(value) not in (int, float) or not math.isfinite(value):
        raise ValueError('缺少有限数值目标')
    result = round(value * scale)
    if not -2**31 <= result < 2**31:
        raise ValueError('目标超出整数接口范围')
    return result


class MechanismLink(Link):
    def __init__(self):
        super().__init__(capabilities=5)
        self.commands = deque(maxlen=1)
        self.results = deque(maxlen=40)
        self.actual = {}
        self.actual_at = self.base_at = -1.
        self.base = {}
        self.last_poll = -1.
        self.poll_kind = 58
        self.latched = False
        self.stop_wire = b''
        self.operation = None
        self.token = 0

    def reset(self, reason):
        # 失联后只请求停止，不自动重新握手/重发动作。
        self.latched = True
        self.commands.clear()
        self.operation = None
        self.message = reason
        self.stop_wire = p.Packet(p.STOP, self.boot or 0, self.session, self.request + 1, 250, b'').encode()
        self.bound = False
        self.pending = None

    def emergency(self, reason):
        self.reset(reason)

    def step(self, now):
        if self.stop_wire:
            data, self.stop_wire = self.stop_wire, b''
            return data
        if self.latched:
            return b''
        return super().step(now)

    def extra_request(self, now):
        if self.commands:
            kind, data, request_id = self.commands.popleft()
            self.operation = request_id
            return kind, data
        if now - self.last_poll >= .04:
            self.last_poll = now
            self.poll_kind = 35 if self.poll_kind == 58 else 58
            return self.poll_kind, b''
        return None

    def custom_reply(self, packet, request, now, sent=None):
        if request.kind not in range(48, 59) and request.kind != 35:
            return False
        if packet.boot != self.boot or not packet.payload:
            self.reset('实车启动身份/回包无效')
            return True
        if packet.payload[0] == p.BUSY and request.kind != 58 and request.kind != 35:
            # BUSY代表未进入命令；只重试同一不可变载荷，板端操作编号去重。
            self.commands.appendleft((request.kind, request.payload, self.operation))
            self.operation = None
            return True
        if packet.payload[0] != p.OK:
            self.reset('实车协议拒绝；动作不重放')
            return True
        data = packet.payload[1:]
        if request.kind == 58:
            if len(data) != 112:
                self.reset('机构状态长度无效')
            else:
                values = struct.unpack('<28I', data)
                if values[0] != 1:
                    self.reset('机构状态版本无效')
                else:
                    self.actual = dict(zip(FIELDS, values))
                    for key in ('lift_um', 'reach_um', 'target_mdeg'):
                        if self.actual[key] & 0x80000000:
                            self.actual[key] -= 2**32
                    self.actual_at = now
        elif request.kind == 35:
            if len(data) != STATUS_DATA.size:
                self.reset('底盘状态长度无效')
            else:
                values = STATUS_DATA.unpack(data)
                self.base = dict(mode=values[0], fault=values[1], stopped=bool(values[5]))
                self.base_at = now
        else:
            if len(data) != 4:
                self.reset('机构动作应答长度无效')
            else:
                result, = struct.unpack('<I', data)
                self.results.appendleft(dict(request_id=self.operation, result=result,
                                             status='ACCEPTED' if not result else 'REJECTED'))
            self.operation = None
        return True

    def submit(self, request_id, action, args, now):
        if (self.latched or not self.bound or self.firmware not in ('GC-BASE-0.3.14', 'GC-BASE-0.3.15', 'GC-BASE-0.3.17', 'GC-BASE-0.3.18')
                or now - self.actual_at >= .25 or now - self.base_at >= .25):
            raise ValueError('实车固件/会话/反馈未就绪')
        if self.commands or self.operation or (self.pending and self.pending[0].kind in range(48, 58)):
            raise ValueError('上一条实车命令尚未完成应答')
        if action != 'attach' and (self.base.get('fault') or self.actual.get('fault') or not self.actual.get('base_stopped')):
            raise ValueError('实车故障或底盘未停稳；禁止动作')
        if action != 'attach' and args.get('zero_version', 0) != self.actual.get('zero_version'):
            raise ValueError('实车零点版本变化')
        self.token += 1
        kind, data = payload(action, args, self.actual, self.token)
        self.commands.append((kind, data, request_id))
        return dict(status='QUEUED', request_id=request_id, domain='physical', action_sent=False)

    def view(self, now):
        fresh = self.bound and not self.latched and now - self.actual_at < .25 and now - self.base_at < .25
        return dict(available=True, domain='physical', connected=fresh, real_connected=fresh,
                    firmware=self.firmware, boot_id=str(self.boot), session_id=str(self.session),
                    latched=self.latched, reason=self.message, mechanism=self.actual if fresh else None,
                    base=self.base if fresh else None, results=list(self.results),
                    state_age_ms=None if self.actual_at < 0 else max(0, round((now-self.actual_at)*1000)))
