"""新固件只读验收：复用正式会话与时间负载，不提供配置/使能/运动命令。"""

import argparse
from collections import Counter
from dataclasses import asdict
import json
from pathlib import Path
import sys
import struct
import time

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'pi'))
from robot.device.execution import MotionLink, STATUS
from robot.device.feedback import FEEDBACK, Feedback
from robot.device import protocol as p
from robot.drive_core import require_owner_stopped
from robot.main import open_transport
from robot.runtime import ProcessLock


class Diagnostic(MotionLink):
    def __init__(self):
        super().__init__()
        self.last_can = -1.
        self.channel = 0
        self.records = []
        self.uart_diag = None
        self.last_uart = -1.

    def extra_request(self, now):
        selected = super().extra_request(now)
        if selected is not None:
            return selected
        if now - self.last_uart >= 1:
            self.last_uart = now
            return 36, b''
        if now - self.last_can >= .15:
            self.last_can = now
            wheel, kind = divmod(self.channel, 3)
            self.channel = (self.channel + 1) % 12
            return FEEDBACK, bytes([kind + 1, wheel + 1])
        return None

    def custom_reply(self, packet, request, now, sent=None):
        if request.kind == 36:
            if packet.boot == request.boot and len(packet.payload) == 73 and packet.payload[0] == p.OK:
                values = struct.unpack('<18I', packet.payload[1:])
                names = ('rx_bytes','overflows','errors','restart_errors','first_error_code',
                         'first_error_ms','last_error_code','last_error_ms','startup_flags')
                self.uart_diag = [dict(zip(names, values[index:index+9])) for index in (0, 9)]
            return True
        result = super().custom_reply(packet, request, now, sent)
        if request.kind == FEEDBACK and packet.payload[:1] == bytes([p.OK]):
            sample = Feedback.read(packet, request, sent, now)
            self.records.append(asdict(sample))
        return result


def summarize(link, states, requests):
    channels = Counter((v['kind'], v['address']) for v in link.records)
    latest = states[-1] if states else {}
    execution = [v['execution'] for v in states if v.get('execution')]
    wheels = []
    for address in range(1, 5):
        speed = [v['values'][0] for v in link.records if (v['kind'], v['address']) == (1, address)]
        position = [v['values'][0] for v in link.records if (v['kind'], v['address']) == (2, address)]
        status = [v['values'][0] for v in link.records if (v['kind'], v['address']) == (3, address)]
        wheels.append(dict(address=address, speed_samples=len(speed), position_samples=len(position),
                           status_samples=len(status), rpm_range=[min(speed), max(speed)] if speed else None,
                           position_range=[min(position), max(position)] if position else None,
                           status_values=sorted(set(status))))
    checks = dict(firmware=latest.get('firmware') == 'GC-BASE-0.3.3',
                  no_actions=bool(execution) and all(v['action_id'] == 0 and v['action_state'] == 0
                                                   and v['config_version'] == 0 for v in execution),
                  disabled=bool(execution) and all(v['mode'] == 0 and not v['fault'] for v in execution),
                  startup_stop=bool(execution) and all(v['stop_confirmed'] for v in execution),
                  can_healthy=bool(execution) and all(v['can_errors'] == 0 for v in execution),
                  can_advancing=len(execution) > 1 and execution[-1]['can_rx_sequence'] > execution[0]['can_rx_sequence'],
                  wheel_feedback=all(min(w['speed_samples'], w['position_samples'], w['status_samples']) >= 3
                                     and w['rpm_range'] == [0, 0]
                                     and all(not value & 12 for value in w['status_values']) for w in wheels),
                  imu_feedback=channels[(4, 0)] >= 5 and channels[(5, 0)] >= 5,
                  readonly_commands=all(k in (p.HELLO, p.OPEN, p.HEART, p.STATE, p.DIAG, STATUS, FEEDBACK, 36)
                                        for k in requests),
                  uart_clean=bool(link.uart_diag) and all(not (v['overflows'] or v['errors'] or v['restart_errors']) for v in link.uart_diag),
                  crc_clean=link.decoder.bad == 0)
    return dict(passed=all(checks.values()), checks=checks, wheels=wheels,
                can_rx_delta=(execution[-1]['can_rx_sequence'] - execution[0]['can_rx_sequence']) if len(execution) > 1 else None,
                firmware=latest.get('firmware'), boot_id=latest.get('boot_id'),
                uart_diag=link.uart_diag,
                requests=dict(Counter(requests)), states=len(states), feedback_samples=len(link.records),
                first_execution=execution[0] if execution else None,
                last_execution=execution[-1] if execution else None,
                imu=[v for v in link.records[-40:] if v['kind'] in (4, 5)],
                physical_direction='沿用用户顺时针增加确认；本次只读静止，实体转向响应留到短转验证')


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--port', required=True)
    parser.add_argument('--owner-url', default='http://127.0.0.1:8080')
    parser.add_argument('--seconds', type=float, default=30)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    if not 5 <= args.seconds <= 60 or args.output.exists():
        parser.error('窗口5～60秒，输出不得覆盖旧证据')
    require_owner_stopped(args.owner_url)
    args.output.parent.mkdir(parents=True, exist_ok=True)
    lock = ProcessLock(args.output.parent / 'core.lock')
    transport = None
    link, states, requests = Diagnostic(), [], []
    try:
        transport = open_transport('serial', args.port)
        deadline = time.monotonic() + args.seconds
        sampled = -1.
        while time.monotonic() < deadline:
            now = time.monotonic()
            link.feed(transport.read(4096), now)
            wire = link.step(now)
            if wire:
                packets = p.Decoder().feed(wire)
                kind = packets[0].kind
                if kind not in (p.HELLO, p.OPEN, p.HEART, p.STATE, p.DIAG, STATUS, FEEDBACK, 36):
                    raise RuntimeError('只读命令白名单违反')
                requests.append(kind)
                if transport.write(wire) != len(wire):
                    raise OSError('只读查询写入不完整')
            if now - sampled >= .1:
                states.append(link.snapshot(now))
                sampled = now
            time.sleep(.005)
    finally:
        if transport is not None:
            transport.close()
        lock.close()
    result = summarize(link, states, requests)
    with args.output.open('x', encoding='utf-8') as file:
        json.dump(dict(summary=result, states=states, feedback=link.records), file, ensure_ascii=False, indent=2)
    print(json.dumps(result, ensure_ascii=False))
    if not result['passed']:
        raise SystemExit(1)


if __name__ == '__main__':
    main()
