"""端到端模拟：C反馈编码→GC帧→请求/时效校验→C连续里程计。无串口/执行器。"""

import argparse
import json
import math
from pathlib import Path
import time

from .device import protocol as p
from .device.feedback import Feedback, FeedbackCache, FEEDBACK
from .device.odometry import Config, Geometry, Odometry, Stamp

MASK = 0xffffffff


def fixture():
    # 仅为可重复的合成模型，地址11～14特意不套用本车未经核对的轮位。
    config = Config()
    config.geometry = Geometry(.05, .1, .12, 1)
    config.counts_per_rev[:] = [65536] * 4
    config.address[:] = [11, 12, 13, 14]
    config.sign[:] = [1, -1, 1, -1]
    config.yaw_scale_rad = math.pi / 32768
    config.yaw_sign = 1
    config.max_yaw_rad_s = 5
    config.max_rpm = 100
    config.margin_counts = 2
    config.fresh_ms = 250
    config.gap_ms = 100
    config.can_epoch = 3
    config.imu_epoch = 5
    config.imu_source_epoch = 7
    config.calibration = 1
    return config


class Model:
    def __init__(self, library, body=(.15, .03, .25), base_ms=0, duration_ms=8000, config=None):
        self.config = config if config is not None else fixture()
        self.native = Odometry(library, self.config, base_ms)
        self.base_ms = base_ms
        self.duration_ms = duration_ms
        self.body = body
        self.rpm = self.native.inverse(*body)
        self.cache = FeedbackCache(17, 19)
        self.decoder = p.Decoder()
        self.sequence = self.request = self.frames = 0
        self.elapsed_ms = -10
        self.status = 'RUNNING'
        self.latest = self.native.snapshot(base_ms)
        self.history = []

    def deliver(self, kind, address, values, elapsed_ms, epoch, source_epoch):
        tick = (self.base_ms + elapsed_ms) & MASK
        self.sequence = (self.sequence + 1) & MASK
        stamp = Stamp(tick, tick, (tick + 2) & MASK, self.sequence, epoch)
        payload = self.native.encode(kind, address, stamp, source_epoch, values, (tick + 4) & MASK)
        self.request = (self.request + 1) & MASK
        request = p.Packet(FEEDBACK, 17, 19, self.request, 250)
        response = p.Packet(FEEDBACK | 0x80, 17, 19, self.request, 250, bytes([p.OK]) + payload)
        wire = response.encode()
        # 拆分传递，使用正式GC解码器和同一请求校验；两时钟偏置不直接相减。
        decoded = self.decoder.feed(wire[:7]) + self.decoder.feed(wire[7:])
        sent = 1000 + elapsed_ms / 1000
        sample = Feedback.read(decoded[0], request, sent, sent + .002)
        if self.cache.push(sample):
            self.native.push(sample, (tick + 6) & MASK)
        self.frames += 1

    def advance(self, elapsed_ms, commit=True):
        if elapsed_ms <= self.elapsed_ms:
            raise ValueError('模拟时间必须单调前进')
        self.elapsed_ms = elapsed_ms
        seconds = min(elapsed_ms, self.duration_ms) / 1000
        vx, vy, wz = self.body
        angle = wz * seconds
        raw_yaw = ((round(angle / self.config.yaw_scale_rad) + 32768) % 65536) - 32768
        self.deliver(4, 0, (0, 0, raw_yaw), elapsed_ms,
                     self.config.imu_epoch, self.config.imu_source_epoch)
        # 每10ms只返回一轮位置，四轮各20～30ms错开，绝不同时贴同一个tick。
        wheel = (elapsed_ms // 10) % 4
        counts = round(self.rpm[wheel] * seconds / 60 * self.config.counts_per_rev[wheel] * self.config.sign[wheel])
        self.deliver(2, self.config.address[wheel], (counts, 0, 0), elapsed_ms,
                     self.config.can_epoch, self.config.can_epoch)
        now = (self.base_ms + elapsed_ms + 6) & MASK
        if commit:
            self.native.step(now)
        self.latest = self.native.snapshot(now)
        self.status = 'FAULT' if self.latest['fault'] else 'COMPLETE' if elapsed_ms >= self.duration_ms else 'RUNNING'
        changed = not self.history or any(abs(self.latest[key] - self.history[-1][key]) > 1e-9
                                         for key in ('x_m', 'y_m', 'yaw_rad'))
        if self.latest['valid'] and changed:
            self.history.append({key: self.latest[key] for key in ('x_m', 'y_m', 'yaw_rad', 'tick_ms')})
            # 有界保留起点与运动轨迹；静止回报不把已完成轨迹挤出显示。
            if len(self.history) > 300:
                self.history = self.history[::2]
        return self.snapshot()

    def truth(self):
        if not self.latest['valid']:
            return None
        start = min((self.latest['origin_ms'] - self.base_ms) & MASK, self.duration_ms) / 1000
        end = min((self.latest['tick_ms'] - self.base_ms) & MASK, self.duration_ms) / 1000
        duration = end - start
        vx, vy, wz = self.body
        angle = wz * duration
        if abs(wz) < 1e-12:
            return dict(x_m=vx * duration, y_m=vy * duration, yaw_rad=angle)
        return dict(x_m=(vx * math.sin(angle) - vy * (1 - math.cos(angle))) / wz,
                    y_m=(vx * (1 - math.cos(angle)) + vy * math.sin(angle)) / wz, yaw_rad=angle)

    def snapshot(self):
        truth = self.truth()
        return dict(domain='simulation', status=self.status, pose=dict(self.latest),
                    truth=truth, error_m=None if truth is None else math.hypot(
                        self.latest['x_m'] - truth['x_m'], self.latest['y_m'] - truth['y_m']),
                    elapsed_ms=self.elapsed_ms, frames=self.frames, history=list(self.history),
                    source='C反馈编码→GC/CRC→时效缓存→C异步连续里程计',
                    calibration='合成模型1，非本车参数', physical_pose_valid=False)


class OdomDemo:
    def __init__(self, library):
        self.model = Model(library)
        self.started = time.monotonic()

    def step(self, now):
        target = int((now - self.started) * 1000 / 10) * 10
        # 有界追赶，仿真计算不阻塞通信保活；逻辑时间与比赛用时严格区分。
        for _ in range(16):
            if self.model.elapsed_ms >= target:
                break
            self.model.advance(self.model.elapsed_ms + 10)

    def snapshot(self):
        return self.model.snapshot()


def main():
    parser = argparse.ArgumentParser(description='仅离线连续里程计回放')
    parser.add_argument('--library', type=Path, required=True)
    parser.add_argument('--output', type=Path, default=Path('output/odometry/report.json'))
    args = parser.parse_args()
    model = Model(args.library)
    for tick in range(0, 8201, 10):
        model.advance(tick)
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(model.snapshot(), ensure_ascii=False, indent=2), encoding='utf-8')
    print(json.dumps({key: value for key, value in model.snapshot().items() if key != 'history'}, ensure_ascii=False))


if __name__ == '__main__':
    main()
