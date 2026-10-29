"""GC只读协议。COBS与有界流解码改编自旧雷达cyqm_protocol.py。"""

import binascii
import struct
from dataclasses import dataclass

HELLO, OPEN, HEART, STATE, DIAG, STOP = range(1, 7)
OK, BOOT, SESSION, BUSY, EXPIRED, FORMAT, READONLY = range(7)
HEADER = struct.Struct('<2sBBQQIHH')
MAX_PAYLOAD = 128
MAX_ENCODED = 159
STATE_DATA = struct.Struct('<IIHIIII')


def cobs_encode(data):
    result = bytearray([0])
    pos, code = 0, 1
    for value in data:
        if value == 0:
            result[pos] = code
            pos, code = len(result), 1
            result.append(0)
        else:
            result.append(value)
            code += 1
            if code == 255:
                result[pos] = code
                pos, code = len(result), 1
                result.append(0)
    result[pos] = code
    return bytes(result)


def cobs_decode(data):
    result, pos = bytearray(), 0
    while pos < len(data):
        code = data[pos]
        pos += 1
        if not code or pos + code - 1 > len(data):
            raise ValueError('COBS长度无效')
        result.extend(data[pos:pos + code - 1])
        pos += code - 1
        if code != 255 and pos < len(data):
            result.append(0)
    return bytes(result)


@dataclass(frozen=True)
class Packet:
    kind: int
    boot: int = 0
    session: int = 0
    request: int = 0
    ttl_ms: int = 250
    payload: bytes = b''

    def encode(self):
        if len(self.payload) > MAX_PAYLOAD:
            raise ValueError('负载超长')
        raw = HEADER.pack(b'GC', 1, self.kind, self.boot, self.session,
                          self.request, self.ttl_ms, len(self.payload)) + self.payload
        return cobs_encode(raw + struct.pack('<H', binascii.crc_hqx(raw, 0xFFFF))) + b'\0'


class Decoder:
    """超长帧丢弃至下一个00，防止截断尾部被解释成新命令。"""

    def __init__(self):
        self.partial = bytearray()
        self.dropping = False
        self.good = self.bad = 0

    def feed(self, data):
        packets = []
        for value in data:
            if value:
                if not self.dropping:
                    self.partial.append(value)
                    if len(self.partial) > MAX_ENCODED:
                        self.partial.clear()
                        self.dropping = True
                        self.bad += 1
                continue
            if self.dropping:
                self.dropping = False
                continue
            segment = bytes(self.partial)
            self.partial.clear()
            if not segment:
                continue
            try:
                raw = cobs_decode(segment)
                if not 30 <= len(raw) <= 158:
                    raise ValueError('帧长无效')
                magic, version, kind, boot, session, request, ttl, size = HEADER.unpack(raw[:28])
                if magic != b'GC' or version != 1 or size > MAX_PAYLOAD or len(raw) != 30 + size:
                    raise ValueError('帧头无效')
                if binascii.crc_hqx(raw[:-2], 0xFFFF) != int.from_bytes(raw[-2:], 'little'):
                    raise ValueError('CRC错误')
                packets.append(Packet(kind, boot, session, request, ttl, raw[28:-2]))
                self.good += 1
            except (ValueError, struct.error):
                self.bad += 1
        return packets
