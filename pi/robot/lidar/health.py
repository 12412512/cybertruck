"""启明v6只读通信状态机；复用旧驱动01/02/13，不启动扫描或修改配置。"""
from collections import deque

from .protocol import Decoder, frame, hello, persisted, isolated_extensions


class Health:
    def __init__(self, node=1):
        if type(node) is not int or not 1 <= node <= 254:
            raise ValueError('雷达节点需1～254')
        self.node = node
        self.decoder = Decoder()
        self.sequence = 0
        self.pending = None
        self.info = self.saved = None
        self.ready = False
        self.completed = set()
        self.fault = ''
        self.last_rx = self.last_hello = -1.
        self.heartbeat = self.poll = 0.
        self.events = deque(maxlen=128)
        self.extensions = deque(maxlen=16)
        self.sent = {1:0, 2:0, 0x13:0}

    def fail(self, reason):
        self.fault = str(reason)
        self.pending = None
        self.ready = False

    def send(self, kind, now):
        if kind not in self.sent:
            raise ValueError('通信验收只允许01/02/13')
        wire = frame(self.node, kind, self.sequence)
        self.sequence = (self.sequence + 1) & 65535
        self.sent[kind] += 1
        self.events.append(dict(direction='TX', command=kind, at_s=now, wire_hex=wire.hex()))
        return wire

    def step(self, now):
        if self.fault:
            return b''
        if self.pending and now >= self.pending['deadline']:
            self.fail(f'雷达命令0x{self.pending["kind"]:02X}等待ACK及数据超时')
            return b''
        if self.ready and now-self.last_rx > 5:
            self.fail('雷达通信超过5秒无有效新帧，需人工重新接入')
            return b''
        if not self.pending and (1 not in self.completed or 0x13 not in self.completed or now >= self.poll):
            kind = 0x13 if 1 in self.completed and 0x13 not in self.completed else 1
            self.pending = dict(kind=kind, deadline=now+1.5, ack=False, data=False)
            return self.send(kind, now)
        if now >= self.heartbeat:
            self.heartbeat = now+.5
            return self.send(2, now)
        return b''

    def feed(self, wire, now):
        if self.fault:
            return
        unknown_before = self.decoder.unknown
        for node, kind, sequence, data in self.decoder.feed(wire):
            if node != self.node:
                continue
            self.events.append(dict(direction='RX', type=kind, sequence=sequence, at_s=now, data_hex=data.hex()))
            try:
                if kind == 0x81:
                    value = hello(data)
                    if self.info is not None and value != self.info:
                        raise ValueError('雷达版本、区号或保存位姿发生变化，需重新核对')
                    self.info = value
                    self.last_hello = now
                    if self.pending and self.pending['kind'] == 1:
                        self.pending['data'] = True
                elif kind == 0x8E:
                    value = persisted(data)
                    if self.saved is not None and value != self.saved:
                        raise ValueError('雷达保存配置发生变化')
                    self.saved = value
                    if self.pending and self.pending['kind'] == 0x13:
                        self.pending['data'] = True
                elif kind == 0x82:
                    if len(data) != 2:
                        raise ValueError('雷达ACK长度无效')
                    if data[0] in self.sent and data[1]:
                        raise ValueError(f'雷达拒绝命令0x{data[0]:02X}，状态{data[1]}')
                    if self.pending and data[0] == self.pending['kind']:
                        self.pending['ack'] = True
                elif kind == 0x8A:
                    raise ValueError('雷达上报设备错误：'+data.hex())
                # 点云及状态广播不构成当前健康握手的完成条件。
                self.last_rx = now
                if self.pending and self.pending['ack'] and self.pending['data']:
                    self.completed.add(self.pending['kind'])
                    self.pending = None
                    self.ready = {1, 0x13} <= self.completed
                    self.poll = now+2
            except ValueError as exc:
                self.fail(exc)
                break
        # 实机1.37在GET_PERSISTED后附带CRC有效的0x90/4字节参数。
        # 旧SDK未说明其字段，只隔离归档；绝不作为握手、标定、扫描或规划数据。
        try:
            self.extensions.extend(isolated_extensions(self.decoder,unknown_before,self.info,self.node))
        except ValueError as exc:
            self.fail(exc)
        if self.decoder.bad:
            self.fail('雷达CRC或帧长度异常')

    def view(self, now):
        connected = bool(self.ready and not self.fault and 0 <= now-self.last_rx < 5
                         and 0 <= now-self.last_hello < 3.5)
        return dict(available=True, domain='physical_lidar', connected=connected,
            status='FAULT' if self.fault else 'CONNECTED' if connected else 'CONNECTING',
            reason=self.fault or ('基础通信正常；0x90扩展参数仅归档、未解码；未扫描/配准' if self.extensions
                                  else '只读通信；未启动扫描，未配准地图'),
            information=self.info, persisted=self.saved, node=self.node,
            state_age_ms=None if self.last_rx < 0 else round((now-self.last_rx)*1000),
            hello_age_ms=None if self.last_hello < 0 else round((now-self.last_hello)*1000),
            decode_good=self.decoder.good, decode_bad=self.decoder.bad,
            decode_unknown=self.decoder.unknown, last_unknown=self.decoder.last_unknown,
            extensions=list(self.extensions),
            commands_sent=dict(self.sent), events=list(self.events)[-16:],
            authorizes_motion=False, scan_allowed=False, real_map_registered=False,
            frame_id='cyqm_field', coverage='unknown', points=[], obstacles=[])
