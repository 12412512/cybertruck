"""反馈时间负载；离线模型与新MotionLink共用，实车仍待新固件验收。"""

from dataclasses import dataclass
import math
import struct

from . import protocol as p

FEEDBACK = 0x10
WIRE = struct.Struct('<BBBB7I3q')
MASK = 0xffffffff
HALF = 0x80000000


def delta(new, old):
    return (new - old) & MASK


@dataclass(frozen=True)
class Feedback:
    boot: int
    session: int
    kind: int
    address: int
    first_ms: int
    last_ms: int
    handled_ms: int
    sequence: int
    epoch: int
    source_epoch: int
    report_ms: int
    values: tuple
    response_s: float
    rtt_ms: float

    @classmethod
    def read(cls, packet, request, sent_s, received_s):
        # 上层先用正式COBS/CRC Decoder解帧；此处核对本次读请求和身份。
        if (request.kind != FEEDBACK or not request.boot or not request.session
                or len(request.payload) not in (0, 2) or packet.kind != FEEDBACK | 0x80
                or (packet.boot, packet.session, packet.request) !=
                   (request.boot, request.session, request.request)
                or len(packet.payload) != WIRE.size + 1 or packet.payload[0] != p.OK):
            raise ValueError('反馈不属于本次会话/请求或长度错误')
        if not all(type(t) in (float, int) and math.isfinite(t) for t in (sent_s, received_s)):
            raise ValueError('主控时间无效')
        rtt = (received_s - sent_s) * 1000
        if sent_s < 0 or not 0 <= rtt < 250:
            raise ValueError('反馈请求往返超时或时间倒退')
        version, kind, address, flags, *fields = WIRE.unpack(packet.payload[1:])
        if request.payload and request.payload != bytes([kind, address]):
            raise ValueError('反馈通道与本次选择不一致')
        first, last, handled, sequence, epoch, source_epoch, report, *values = fields
        if version != 1 or flags != 0 or not epoch or not source_epoch:
            raise ValueError('反馈版本、时钟代次或采样标志无效')
        if (delta(last, first) >= HALF or delta(report, first) >= HALF
                or delta(handled, last) > delta(report, last)
                or delta(report, last) + rtt >= 250):
            raise ValueError('接收/处理/上报时间无序或反馈过期')
        if kind in (1, 2, 3):
            limit = {1: (-5000, 5000), 2: (-MASK, MASK), 3: (0, 255)}[kind]
            if (not address or first != last or values[1:] != [0, 0]
                    or not limit[0] <= values[0] <= limit[1]):
                raise ValueError('电机原始反馈值无效')
        elif kind in (4, 5):
            if address or not all(-32768 <= value <= 32767 for value in values):
                raise ValueError('IMU原始反馈值无效')
        else:
            raise ValueError('未知反馈通道')
        return cls(packet.boot, packet.session, kind, address, first, last, handled,
                   sequence, epoch, source_epoch, report, tuple(values), received_s, rtt)

    def age_ms(self, now_s):
        if not math.isfinite(now_s) or now_s < self.response_s:
            raise ValueError('主控时间倒退')
        # MCU差值只在同一MCU时钟计算；主控只加往返上界和本机存留时间。
        return delta(self.report_ms, self.last_ms) + self.rtt_ms + (now_s - self.response_s) * 1000


class FeedbackCache:
    def __init__(self, boot, session):
        if not boot or not session:
            raise ValueError('必须显式绑定启动/会话身份')
        self.boot, self.session = boot, session
        self.samples = {}
        self.sources = {}
        self.fault = None

    def push(self, sample):
        key = (sample.kind, sample.address)
        if self.fault:
            raise ValueError('时间历史已失效，需要重新核对并重建')
        previous = self.samples.get(key)
        group = 'can' if sample.kind <= 3 else 'imu'
        epochs = (sample.epoch, sample.source_epoch)
        if (sample.boot, sample.session) != (self.boot, self.session):
            self.fault = '启动或会话改变'
        elif group in self.sources and self.sources[group] != epochs:
            self.fault = '同一设备源的接收连续性改变'
        elif previous is not None:
            if (sample.epoch, sample.source_epoch) != (previous.epoch, previous.source_epoch):
                self.fault = '接收连续性改变'
            elif sample.sequence == previous.sequence:
                if sample == previous:
                    return False
                # 同帧可由新请求再次上报，原始内容/接收时间必须完全一致。
                if (sample.first_ms, sample.last_ms, sample.handled_ms, sample.values) == (
                        previous.first_ms, previous.last_ms, previous.handled_ms, previous.values):
                    return False
                self.fault = '相同序号内容冲突'
            elif (delta(sample.sequence, previous.sequence) >= HALF
                  or delta(sample.last_ms, previous.last_ms) >= HALF
                  or sample.response_s < previous.response_s):
                self.fault = '序号或时间倒退'
        if self.fault:
            self.samples.clear()
            raise ValueError(self.fault)
        self.samples[key] = sample
        self.sources[group] = epochs
        return True

    def get(self, kind, address, now_s):
        sample = self.samples.get((kind, address))
        if self.fault or sample is None:
            return None
        try:
            age = sample.age_ms(now_s)
        except ValueError:
            self.fault = '主控时间倒退'
            self.samples.clear()
            return None
        if age >= 250:
            return None
        return sample
