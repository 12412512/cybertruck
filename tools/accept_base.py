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
from robot.device.mechanism import FIELDS
from robot.drive_core import require_owner_stopped
from robot.main import open_transport
from robot.runtime import ProcessLock

READONLY_KINDS = {p.HELLO,p.OPEN,p.HEART,p.STATE,p.DIAG,STATUS,FEEDBACK,36,37,38,58,60}
MODERN = ('GC-BASE-0.3.20','GC-BASE-0.3.21','GC-BASE-0.3.22','GC-BASE-0.3.23','GC-BASE-0.3.24')


class Diagnostic(MotionLink):
    def __init__(self):
        super().__init__()
        self.last_can = -1.
        self.channel = 0
        self.records = []
        self.uart_diag = None
        self.last_uart = -1.
        self.last_mech = -1.
        self.mech_state = None

    def extra_request(self, now):
        selected = super().extra_request(now)
        if selected is not None:
            return selected
        # 静止验收也明确读取“无首次错误”页头，不能把未读取当成空记录。
        if (self.firmware in ('GC-BASE-0.3.12', *MODERN) and self.execution
                and self.execution['mode'] in (0, 1, 6) and not self.execution['can_errors']
                and now - self.last_can_capture >= 1):
            self.last_can_capture = now
            return 38, b'\0'
        if self.firmware in MODERN and now-self.last_mech >= .25:
            self.last_mech = now
            return 58,b''
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
        if request.kind == 58:
            if packet.boot == request.boot and packet.payload[:1] == bytes([p.OK]) and len(packet.payload) == 113:
                self.mech_state = dict(zip(FIELDS,struct.unpack('<28I',packet.payload[1:])))
                if self.mech_state['schema'] != 1:self.failed=True
            else:
                self.failed=True
            return True
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


def summarize(link, states, requests, expected_firmware='GC-BASE-0.3.6'):
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
    checks = dict(firmware=latest.get('firmware') == expected_firmware,
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
                  readonly_commands=bool(requests) and all(k in READONLY_KINDS for k in requests),
                  uart_clean=bool(link.uart_diag) and all(not (v['overflows'] or v['errors'] or v['restart_errors']) for v in link.uart_diag),
                  crc_clean=link.decoder.bad == 0,
                  can_capture_clean=(latest.get('firmware') not in ('GC-BASE-0.3.12', *MODERN) or
                                     (latest.get('can_capture_status') is not None and
                                      not latest['can_capture_status']['valid'] and
                                      not latest.get('can_capture_error'))),
                  fault_record_clean=(latest.get('firmware') not in ('GC-BASE-0.3.9', 'GC-BASE-0.3.10', 'GC-BASE-0.3.11', 'GC-BASE-0.3.12', *MODERN) or
                                      (latest.get('odom_fault_record') is not None and
                                       not latest['odom_fault_record']['valid'])))
    if expected_firmware in MODERN:
        physical = [s['physical_start'] for s in states if s.get('physical_start')]
        mech = getattr(link,'mech_state',None)
        identities={(s.get('boot_id'),s.get('session_id')) for s in states if s.get('execution')}
        checks.update(identity_stable=len(identities)==1 and all(all(v) for v in identities),
            feedback_coverage=len(execution)>=len(states)*.9,
            key_idle=bool(physical) and all(not (v['fired'] or v['beep'] or v['ticket']) for v in physical),
            pwm_off=bool(mech) and mech['pwm_mask']==0 and mech['axis_state']==0 and
                mech['servo_state']==0 and mech['pose_state']==0 and not mech['motion_uncertain'])
    return dict(passed=all(checks.values()), checks=checks, wheels=wheels,
                mechanism_state=getattr(link,'mech_state',None),
                can_rx_delta=(execution[-1]['can_rx_sequence'] - execution[0]['can_rx_sequence']) if len(execution) > 1 else None,
                firmware=latest.get('firmware'), boot_id=latest.get('boot_id'),
                odom_fault_record=latest.get('odom_fault_record'),
                can_fault_record=latest.get('can_fault_record'),
                can_capture_error=latest.get('can_capture_error'),
                can_capture_status=latest.get('can_capture_status'),
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
    parser.add_argument('--expect-firmware', default='GC-BASE-0.3.6',
                        choices=('GC-BASE-0.3.6', 'GC-BASE-0.3.7', 'GC-BASE-0.3.8', 'GC-BASE-0.3.9', 'GC-BASE-0.3.10', 'GC-BASE-0.3.11', 'GC-BASE-0.3.12', *MODERN))
    args = parser.parse_args()
    if not 5 <= args.seconds <= 60 or args.output.exists():
        parser.error('窗口5～60秒，输出不得覆盖旧证据')
    require_owner_stopped(args.owner_url)
    args.output.parent.mkdir(parents=True, exist_ok=True)
    lock = ProcessLock(args.output.parent / 'core.lock')
    transport = None
    link, states, requests = Diagnostic(), [], []
    failure = ''
    try:
        transport = open_transport('serial', args.port)
        deadline = time.monotonic() + args.seconds
        sampled = -1.
        while time.monotonic() < deadline:
            now = time.monotonic()
            link.feed(transport.read(4096), now)
            if link.failed:
                raise RuntimeError('只读验收会话/通信中断；停止采集，不自动重连')
            wire = link.step(now)
            if wire:
                packets = p.Decoder().feed(wire)
                kind = packets[0].kind
                if kind not in READONLY_KINDS:
                    raise RuntimeError('只读命令白名单违反')
                requests.append(kind)
                if transport.write(wire) != len(wire):
                    raise OSError('只读查询写入不完整')
            if now - sampled >= .1:
                states.append(link.snapshot(now))
                sampled = now
            time.sleep(.005)
    except (Exception,KeyboardInterrupt) as exc:
        failure = f'{type(exc).__name__}: {exc}'
    finally:
        if transport is not None:
            transport.close()
        lock.close()
    result = summarize(link, states, requests, args.expect_firmware)
    if failure:result.update(passed=False,collection_error=failure)
    with args.output.open('x', encoding='utf-8') as file:
        json.dump(dict(summary=result, states=states, feedback=link.records), file, ensure_ascii=False, indent=2)
    print(json.dumps(result, ensure_ascii=False))
    if not result['passed']:
        raise SystemExit(1)


if __name__ == '__main__':
    main()
