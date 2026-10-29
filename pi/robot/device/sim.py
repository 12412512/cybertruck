"""明确标识的只读演示设备，只模拟协议；不导入串口或执行器。"""

import secrets
import struct

from . import protocol as p


class SimTransport:
    def __init__(self, clock):
        self.clock = clock
        self.started = clock()
        self.boot = secrets.randbits(64) or 1
        self.decoder = p.Decoder()
        self.buffer = bytearray()
        self.session = 0
        self.generation = 1
        self.heart_at = self.started
        self.progress = self.hearts = 0

    def write(self, wire):
        now = self.clock()
        if self.session and now - self.heart_at >= .5:
            self.session = 0
            self.generation += 1
        for packet in self.decoder.feed(wire):
            result, payload = p.OK, b''
            if packet.kind == p.HELLO:
                payload = struct.pack('<II16s', self.generation, 1, b'SIM-GC-RO')
            elif packet.boot != self.boot:
                result = p.BOOT
            elif packet.kind == p.OPEN:
                if packet.payload != struct.pack('<I', self.generation) or not packet.session:
                    result = p.SESSION
                elif self.session and self.session != packet.session:
                    result = p.BUSY
                elif not self.session:
                    self.session = packet.session
                    self.heart_at = now
                    self.progress = 0
            elif not self.session or packet.session != self.session:
                result = p.SESSION
            elif packet.kind == p.HEART and len(packet.payload) == 4:
                progress, = struct.unpack('<I', packet.payload)
                if 0 < (progress - self.progress) & 0xFFFFFFFF < 0x80000000:
                    self.progress = progress
                    self.heart_at = now
                    self.hearts += 1
            elif packet.kind == p.STATE:
                payload = p.STATE_DATA.pack(int((now - self.started) * 1000) & 0xFFFFFFFF,
                                            0, 3, self.hearts, self.decoder.good, self.decoder.bad, 0)
            elif packet.kind == p.DIAG:
                payload = struct.pack('<II', 0, self.generation)
            else:
                result = p.READONLY
            self.buffer.extend(p.Packet(packet.kind | 0x80, self.boot, packet.session,
                                        packet.request, 0, bytes([result]) + payload).encode())
        return len(wire)

    def read(self, size=4096):
        data = bytes(self.buffer[:size])
        del self.buffer[:size]
        return data

    def close(self):
        self.buffer.clear()
