"""厂家两点校准事务；先只读备份，明确批准后重读核对，09只发一次。"""
import copy
import struct
import uuid
from collections import deque

from .calibration import digest
from .protocol import Decoder, frame, calibration_frame, hello, persisted, scan_status, isolated_extensions
from .vendor_calibration import decode_result, verify_saved


class Calibration:
    TERMINAL = {'COMPLETE', 'FAILED', 'CANCELLED'}

    def __init__(self, node, zone, now, sequence=0):
        if type(zone) is not int or zone not in (1, 2):
            raise ValueError('校准区域为1或2')
        self.id = uuid.uuid4().hex
        self.node, self.zone, self.sequence = node, zone, sequence
        self.phase = 'HELLO'
        self.pending = None
        self.info = self.backup = self.backup_version = None
        self.prepared_at = self.write_at = self.progress_at = None
        self.begun = self.last_rx = now
        self.heartbeat = now
        self.prepared_poll = now
        self.result = self.saved = self.verified = None
        self.write_issued = self.approved = False
        self.cancelled = False
        self.stop_ack = self.stop_status = False
        self.stop_deadline = None
        self.errors, self.commands = [], []
        self.decoder, self.wire = Decoder(), bytearray()
        self.extensions = deque(maxlen=16)

    @property
    def needs_still(self):
        return self.approved or self.write_issued

    def transmit(self, kind, now, payload=b''):
        if kind == 9:
            if not self.approved or self.write_issued or self.phase != 'CALIBRATING':
                raise ValueError('校准未批准或已发送，禁止重复写入')
            wire = calibration_frame(self.node, self.sequence, self.zone)
            self.write_issued = True
            self.write_at = self.progress_at = now
        else:
            if kind not in (1, 2, 4, 0x13):
                raise ValueError('校准事务不允许其他设备命令')
            wire = frame(self.node, kind, self.sequence, payload)
        self.sequence = (self.sequence+1) & 65535
        self.commands.append(dict(command=kind, at_s=now, payload_hex=(bytes([self.zone]) if kind == 9 else payload).hex()))
        return wire

    def begin(self, phase, kind, now):
        self.phase = phase
        self.pending = dict(kind=kind, deadline=now+1.5, ack=False, data=False)
        return self.transmit(kind, now)

    def approve(self, version, now):
        if self.pending is not None:
            raise ValueError('只读事务未结束，请稍后重试相同提交请求')
        if self.phase != 'PREPARED' or self.backup_version != version or now-self.prepared_at > 60:
            raise ValueError('准备票据已过期、状态或备份版本不符，请重新只读准备')
        self.approved = True
        self.phase = 'RECHECK_HELLO'

    def abort(self, reason, now):
        if self.phase in self.TERMINAL:
            return
        if str(reason) not in self.errors:
            self.errors.append(str(reason))
        if self.phase != 'STOP':
            self.phase, self.pending, self.stop_deadline = 'STOP', None, None

    def cancel(self, now):
        if self.phase in self.TERMINAL:
            return
        self.cancelled = True
        self.abort('人工取消，不自动重试校准', now)

    def step(self, now):
        if self.phase in self.TERMINAL:
            return b''
        if self.pending and now >= self.pending['deadline']:
            self.abort('校准事务等待ACK/数据超时，不重发09', now)
        if self.phase == 'PREPARED' and now-self.prepared_at > 60:
            self.cancelled = True
            self.abort('校准准备票据60秒未使用，已取消', now)
        if self.write_at is not None and self.phase not in ('STOP', 'VERIFY_NEW'):
            if now-self.write_at >= 8 or now-self.progress_at >= 3.5:
                self.abort('14圈校准超过8秒或进度静默3.5秒，不自动重试', now)
        if self.phase == 'STOP':
            if self.stop_deadline is None:
                self.stop_deadline = now+.4
                return self.transmit(4, now)
            if self.stop_ack and self.stop_status:
                self.phase = ('CANCELLED' if self.cancelled else
                    'COMPLETE' if self.verified and not self.errors else 'FAILED')
            elif now >= self.stop_deadline:
                self.errors.append('校准STOP未获得新ACK及停止状态双确认')
                self.phase = 'FAILED'
            return b''
        if self.phase in ('HELLO', 'RECHECK_HELLO') and self.pending is None:
            return self.begin(self.phase, 1, now)
        if self.pending and self.pending['ack'] and self.pending['data']:
            phase, self.pending = self.phase, None
            if phase == 'HELLO':
                return self.begin('BACKUP', 0x13, now)
            if phase == 'BACKUP':
                self.backup_version = digest(dict(zone=self.zone, information=self.info, persisted=self.backup))
                self.prepared_at, self.phase = now, 'PREPARED'
                self.prepared_poll = now+2
            elif phase == 'RECHECK_HELLO':
                return self.begin('RECHECK_BACKUP', 0x13, now)
            elif phase == 'RECHECK_BACKUP':
                return self.begin('CALIBRATING', 9, now)
            elif phase == 'CALIBRATING':
                return self.begin('VERIFY_NEW', 1, now)
            elif phase == 'VERIFY_NEW':
                self.phase = 'STOP'
                return self.step(now)
        if self.phase == 'CALIBRATING' and self.pending and self.pending['ack']:
            # ACK已到后改由8秒总限/3.5秒进度限约束，不将ACK超时当整轮期限。
            self.pending['deadline'] = self.write_at+8
        if self.phase == 'PREPARED' and self.pending is None and now >= self.prepared_poll:
            self.prepared_poll = now+2
            return self.begin('PREPARED', 1, now)
        if now >= self.heartbeat:
            self.heartbeat = now+.5
            return self.transmit(2, now)
        return b''

    def feed(self, wire, now):
        if self.phase in self.TERMINAL:
            return
        if len(self.wire)+len(wire) > 262144:
            self.abort('校准报文超过256KiB', now)
            return
        self.wire.extend(wire)
        unknown_before = self.decoder.unknown
        for node, kind, sequence, payload in self.decoder.feed(wire):
            try:
                if node != self.node:
                    raise ValueError('校准出现非目标节点')
                self.last_rx = now
                if kind == 0x82:
                    if len(payload) != 2:
                        raise ValueError('校准ACK长度错误')
                    if self.phase == 'STOP' and self.stop_deadline is not None and payload == b'\4\0':
                        self.stop_ack = True
                    if self.pending and payload[0] == self.pending['kind']:
                        if payload[1]:
                            raise ValueError('设备拒绝校准事务')
                        self.pending['ack'] = True
                elif (kind == 0x81 and self.phase in ('HELLO', 'PREPARED', 'RECHECK_HELLO', 'VERIFY_NEW')
                        and self.pending and self.pending['kind'] == 1):
                    value = hello(payload)
                    if value['zone'] != self.zone or value['running']:
                        raise ValueError('厂家区号不符或设备仍在扫描')
                    if self.phase == 'HELLO':
                        self.info = value
                    elif self.phase in ('PREPARED', 'RECHECK_HELLO') and value != self.info:
                        raise ValueError('批准后厂家信息变化，取消写入')
                    elif self.phase == 'VERIFY_NEW':
                        if (value['firmware'] != self.info['firmware'] or
                                any(value[key] != self.result[key] for key in ('zone', 'x_mm', 'y_mm', 'heading_deg'))):
                            raise ValueError('校准后HELLO与本轮结果不符')
                        self.info = value
                    self.pending['data'] = True
                elif (kind == 0x8E and self.phase in ('BACKUP', 'RECHECK_BACKUP', 'CALIBRATING')
                        and self.pending):
                    value = persisted(payload)
                    if self.phase == 'BACKUP':
                        old_pose = value[f'zone{self.zone}_pose']
                        if any(old_pose[key] != self.info[key] for key in ('zone', 'x_mm', 'y_mm', 'heading_deg')):
                            raise ValueError('新读取的保存位姿与HELLO不一致')
                        self.backup = copy.deepcopy(value)
                        self.pending['data'] = True
                    elif self.phase == 'RECHECK_BACKUP':
                        if value != self.backup:
                            raise ValueError('批准后保存记录变化，取消写入')
                        self.pending['data'] = True
                    elif self.saved is not None and self.saved != value:
                        raise ValueError('本轮收到冲突保存记录')
                    else:
                        self.saved = value
                elif kind == 0x88 and self.phase == 'CALIBRATING':
                    value = decode_result(payload)
                    if value['zone'] != self.zone or self.result is not None and self.result != value:
                        raise ValueError('本轮校准结果错区或冲突')
                    self.result = value
                elif kind == 0x83:
                    status = scan_status(payload)
                    if self.phase == 'CALIBRATING' and status['status'] == 5 and status['mode'] == 3:
                        if status['frame_count'] > 14:
                            raise ValueError('校准进度超出14圈')
                        self.progress_at = now
                    if (self.phase == 'STOP' and self.stop_deadline is not None and
                            status['status'] == 3 and not status['running']):
                        self.stop_status = True
                elif kind == 0x8A:
                    raise ValueError('厂家校准错误，保存状态不得猜测：'+payload.hex())
                elif kind == 0x89:
                    if len(payload) != 25 or struct.unpack('<IHHHIIIHB', payload)[-1] & 3:
                        raise ValueError('校准STATS长度或溢出标志异常')
                if self.phase == 'CALIBRATING' and self.result is not None and self.saved is not None:
                    self.verified = verify_saved(self.backup, self.saved, self.result, self.zone)
                    self.pending['data'] = True
            except (ValueError, struct.error) as error:
                self.abort(str(error), now)
        try:
            self.extensions.extend(isolated_extensions(self.decoder, unknown_before, self.info, self.node))
        except ValueError as error:
            self.abort(str(error), now)
        if self.decoder.bad:
            self.abort('校准CRC或帧长错误', now)

    def view(self, include_frames=False):
        return dict(id=self.id, domain='physical_lidar_calibration', live=self.phase not in self.TERMINAL,
            status=self.phase, zone=self.zone, information=self.info, backup=self.backup,
            backup_version=self.backup_version, write_issued=self.write_issued,
            write_may_have_occurred=self.write_issued, result=self.result, saved=self.saved,
            verified=self.verified, stop_confirmed=self.stop_ack and self.stop_status,
            errors=list(self.errors), commands=list(self.commands), extensions=list(self.extensions),
            decode_bad=self.decoder.bad, decode_unknown=self.decoder.unknown,
            real_map_registered=False, authorizes_motion=False)
