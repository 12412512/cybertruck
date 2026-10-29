"""旧05/19/03/04有界静扫状态机；只有唯一雷达核心调用，无底盘命令。"""
from collections import deque
import struct
import uuid

from .protocol import (Decoder, CloudAssembler, frame, hello, scan_status,
                       scan_grid, scan_result, isolated_extensions)


class StaticScan:
    TERMINAL = {'COMPLETE', 'FAILED'}

    def __init__(self, node, zone, duration_s, now, sequence=0):
        if zone not in (1, 2) or type(zone) is not int or not 2 <= duration_s <= 10:
            raise ValueError('静扫区域为1/2，窗口限定2～10秒')
        self.id = uuid.uuid4().hex
        self.node, self.zone, self.duration_s = node, zone, duration_s
        self.sequence = sequence
        self.decoder, self.cloud = Decoder(), CloudAssembler()
        self.phase = 'HELLO'
        self.pending = None
        self.begun = now
        self.scanning_since = None
        self.heartbeat = 0.
        self.info = None
        self.last_rx = now
        self.errors = []
        self.stop_ack = self.stop_status = False
        self.stop_deadline = None
        self.clouds, self.grids, self.results = {}, {}, {}
        self.events = deque(maxlen=128)
        self.commands = []
        self.wire = bytearray()
        self.extensions = deque(maxlen=16)
        self.tail_partial = False

    def transmit(self, kind, now, payload=b''):
        if kind not in (1, 2, 3, 4, 5, 0x19):
            raise ValueError('静扫不允许写标定或发送运动命令')
        wire = frame(self.node, kind, self.sequence, payload)
        self.sequence = (self.sequence+1) & 65535
        self.commands.append(dict(command=kind, at_s=now, payload_hex=payload.hex()))
        return wire

    def begin(self, phase, kind, now, payload=b''):
        self.phase = phase
        self.pending = dict(kind=kind, ack=False, data=False, deadline=now+1.5)
        return self.transmit(kind, now, payload)

    def abort(self, reason, now):
        if self.phase in self.TERMINAL:
            return
        if str(reason) not in self.errors:
            self.errors.append(str(reason))
        if self.phase != 'STOP':
            self.phase = 'STOP'
            self.pending = None
            self.stop_deadline = None

    def step(self, now):
        if self.phase in self.TERMINAL:
            return b''
        if self.pending and now >= self.pending['deadline']:
            self.abort('雷达事务等待ACK/状态超时', now)
        if self.phase != 'STOP' and (now-self.last_rx > 3 or now-self.begun > self.duration_s+8):
            self.abort('雷达失联或静扫总期限超时', now)
        if self.phase == 'STOP':
            if self.stop_deadline is None:
                self.stop_deadline = now+1.5
                return self.transmit(4, now)
            if self.stop_ack and self.stop_status:
                self.tail_partial = self.cloud.current is not None or bool(self.grids)
                if self.decoder.bad or self.cloud.dropped or self.tail_partial:
                    self.errors.append('扫描拆包、缺块或尾部未完整结束')
                if len(set(self.clouds) & set(self.results)) < 3:
                    self.errors.append('不足三个完整且配对的点云/结果')
                self.phase = 'FAILED' if self.errors else 'COMPLETE'
                return b''
            if now >= self.stop_deadline:
                self.errors.append('扫描STOP未获得ACK和停止状态双确认')
                self.phase = 'FAILED'
                return b''
        elif self.phase == 'HELLO' and self.pending is None:
            return self.begin('HELLO', 1, now)
        elif self.pending and self.pending['ack'] and self.pending['data']:
            phase = self.phase
            self.pending = None
            if phase == 'HELLO':
                if self.info['running']:
                    self.abort('扫描前设备已经运行，先停止且不自动重试', now)
                else:
                    return self.begin('ZONE', 5, now, bytes([self.zone]))
            elif phase == 'ZONE':
                return self.begin('CLOUD', 0x19, now, b'\1')
            elif phase == 'CLOUD':
                return self.begin('START', 3, now)
            elif phase == 'START':
                self.phase = 'SCANNING'
                self.scanning_since = now
        elif self.phase == 'SCANNING' and now-self.scanning_since >= self.duration_s:
            self.phase = 'STOP'
            self.stop_deadline = None
            return self.step(now)
        if now >= self.heartbeat:
            self.heartbeat = now+.5
            return self.transmit(2, now)
        return b''

    def feed(self, wire, now):
        if self.phase in self.TERMINAL:
            return
        if len(self.wire)+len(wire) > 1048576:
            self.abort('扫描报文超过1MiB容量', now)
            return
        self.wire.extend(wire)
        unknown_before = self.decoder.unknown
        for node, kind, sequence, payload in self.decoder.feed(wire):
            if node != self.node:
                self.abort('扫描出现非目标节点帧', now)
                continue
            self.last_rx = now
            self.events.append(dict(type=kind, sequence=sequence, at_s=now))
            try:
                if kind == 0x81:
                    value = hello(payload)
                    if self.phase == 'HELLO':
                        self.info = value
                        if self.pending:
                            self.pending['data'] = True
                    elif self.phase == 'ZONE':
                        if value['zone'] != self.zone or value['running']:
                            raise ValueError('厂家工作区回读不匹配或仍在扫描')
                        self.info = value
                        if self.pending:
                            self.pending['data'] = True
                    elif value['zone'] != self.zone:
                        raise ValueError('扫描期间厂家区号改变')
                elif kind == 0x82:
                    if len(payload) != 2:
                        raise ValueError('扫描ACK长度错误')
                    if payload[1] and payload[0] in (1, 3, 4, 5, 0x19):
                        raise ValueError('设备拒绝扫描事务')
                    if self.phase == 'STOP' and self.stop_deadline is not None and payload == b'\4\0':
                        self.stop_ack = True
                    if self.pending and payload[0] == self.pending['kind'] and not payload[1]:
                        self.pending['ack'] = True
                        if self.phase == 'CLOUD':
                            self.pending['data'] = True
                elif kind == 0x83:
                    status = scan_status(payload)
                    if self.info is not None:
                        self.info = dict(self.info, running=status['running'])
                    if self.phase == 'START' and status['mode'] == 0 and status['running']:
                        self.pending['data'] = True
                    if (self.phase == 'STOP' and self.stop_deadline is not None and
                            status['status'] == 3 and not status['running']):
                        self.stop_status = True
                elif kind == 0x8A:
                    raise ValueError('扫描设备错误：'+payload.hex())
                elif kind in (0x84, 0x85, 0x86) and self.phase in ('START','SCANNING','STOP'):
                    cloud = self.cloud.feed(kind, payload)
                    if cloud:
                        key = cloud['scan_sequence']
                        if key in self.clouds or len(self.clouds) >= 128:
                            raise ValueError('扫描序号重复或数量超过128')
                        if self.clouds:
                            previous = next(reversed(self.clouds.values()))
                            interval = (cloud['device_timestamp_ms']-previous['device_timestamp_ms']) & 0xffffffff
                            advance = (key-previous['scan_sequence']) & 0xffffffff
                            if not 0 < interval <= 3000 or not 0 < advance < 0x80000000:
                                raise ValueError('扫描时间戳或序号倒退/重复')
                        self.clouds[key] = cloud
                elif kind == 0x8D and self.phase in ('START','SCANNING','STOP'):
                    key, ids = scan_grid(payload)
                    if key in self.grids or key in self.results or len(self.grids) >= 128:
                        raise ValueError('重复或过量GRID')
                    self.grids[key] = ids
                elif kind == 0x87 and self.phase in ('START','SCANNING','STOP'):
                    if len(payload) < 8:
                        raise ValueError('RESULT长度错误')
                    key = struct.unpack_from('<I',payload)[0]
                    if key not in self.grids or key not in self.clouds:
                        raise ValueError('RESULT缺少完整点云或配对GRID')
                    self.results[key] = scan_result(payload,key,self.grids.pop(key))
                elif kind == 0x89:
                    if len(payload) != 25 or struct.unpack('<IHHHIIIHB',payload)[-1] & 3:
                        raise ValueError('扫描STATS溢出或长度错误')
            except (ValueError, struct.error) as exc:
                self.abort(str(exc), now)
        try:
            self.extensions.extend(isolated_extensions(self.decoder,unknown_before,self.info,self.node))
        except ValueError as exc:
            self.abort(str(exc),now)
        if self.decoder.bad:
            self.abort('扫描CRC错误', now)

    def view(self, include_frames=False):
        matched = sorted(set(self.clouds) & set(self.results))
        latest = self.clouds[matched[-1]] if matched else None
        return dict(id=self.id, domain='physical_lidar_scan', live=self.phase not in self.TERMINAL,
            status=self.phase, zone=self.zone, duration_s=self.duration_s,
            errors=self.errors, stop_confirmed=self.stop_ack and self.stop_status,
            cloud_frames=len(self.clouds), result_frames=len(self.results), matched_frames=len(matched),
            nonempty_cloud_frames=sum(bool(c['points']) for c in self.clouds.values()),
            display_point_count_max=max((len(c['points']) for c in self.clouds.values()),default=0),
            source_point_count_max=max((c['source_point_count'] for c in self.clouds.values()),default=0),
            scan_sequences=matched,
            decode_bad=self.decoder.bad, decode_unknown=self.decoder.unknown,
            dropped_clouds=self.cloud.dropped, partial_tail=self.tail_partial,
            information=self.info, raw_points=latest['points'] if latest else [],
            frames=[self.results[key] for key in matched] if include_frames else [],
            commands=self.commands, extensions=list(self.extensions),
            real_map_registered=False, authorizes_motion=False, coverage='unknown',
            note='实机静扫诊断，厂家cyqm_field坐标；没有地图配准，零目标不代表空场')
