"""正式GC/执行器/跟踪器的主机闭环；不打开真实串口。"""

import ctypes as c
import json
import math
import os
from pathlib import Path
import subprocess
import sys
import tempfile
import unittest
from unittest.mock import patch
import time
from fastapi.testclient import TestClient

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'pi'))
from robot.device import protocol as p
from robot.device.execution import MotionLink, CONFIG, ARM, GOAL, STATUS, STATUS_DATA, config_payload, trial
from robot.drive_core import run_trial, require_owner_stopped, observe
from robot.track_sim import Plant
from robot.web.app import create_app
from robot.runtime import save_snapshot, load_snapshot
from robot.device.feedback import Feedback, FEEDBACK
from robot.device.tracking import Reference


def profile():
    value = json.loads((ROOT / 'config/base_trials.json').read_text(encoding='utf-8'))
    # 合成车体的已知参数；不能回写实车候选或声称本车方向已确认。
    value.update(yaw_sign=1, half_length_m=.10, half_width_m=.12)
    return value


def compile_c(output, sources, extra=()):
    command = [sys.executable, '-m', 'ziglang', 'cc', '-std=c99', '-O2',
               '-Wall', '-Wextra', '-Werror', '-UNDEBUG', '-Wno-misleading-indentation']
    for name in ('Control', 'Device', 'Comm', 'Config'):
        command += ['-I', str(ROOT / 'firmware/User' / name)]
    command += [*extra, *map(str, sources), '-o', str(output)]
    result = subprocess.run(command, capture_output=True, text=True, encoding='utf-8', errors='replace')
    if result.returncode:
        raise RuntimeError(result.stderr)


class Device:
    def __init__(self, library):
        self.lib = library
        self.lib.Device_Init()
        self.tick = self.request = 0
        self.lib.Device_Tick(0, (c.c_int64 * 4)(), (c.c_int16 * 4)(), 0, 0)

    def write(self, wire):
        self.lib.Device_Push(wire, len(wire))
        return len(wire)

    def read(self, _=4096):
        output = c.create_string_buffer(160)
        size = self.lib.Device_Read(output)
        return output.raw[:size]

    def call(self, kind, payload=b'', session=456):
        self.request += 1
        self.write(p.Packet(kind, 123, session, self.request, 250, payload).encode())
        return p.Decoder().feed(self.read())[0]

    def open(self):
        hello = self.call(p.HELLO, session=0)
        self.call(p.OPEN, hello.payload[1:5])

    def step(self, counts=None, rpm=None, yaw=0, gyro=0, heart=True):
        self.tick += 10
        if heart and self.tick % 100 == 0:
            self.call(p.HEART, self.tick.to_bytes(4, 'little'))
        self.lib.Device_Tick(self.tick, (c.c_int64 * 4)(*(counts or [0] * 4)),
                             (c.c_int16 * 4)(*(rpm or [0] * 4)), yaw, gyro)

    def arm(self):
        self.open()
        self.call(CONFIG, config_payload(profile()))
        for _ in range(8):
            self.step()
        self.assert_ok(self.call(ARM))
        for _ in range(8):
            self.step()

    @staticmethod
    def assert_ok(packet):
        if packet.payload[0] != p.OK:
            raise AssertionError(packet.payload)


class SimTransport(Device):
    def __init__(self, library):
        super().__init__(library)
        self.plant = Plant(ideal=True)
        self.now = 0.
        self.requests = []
        self.signs = [1, -1, 1, -1]
        self.sensor_sign = 1

    def write(self, wire):
        self.requests += p.Decoder().feed(wire)
        return super().write(wire)

    def sleep(self, seconds):
        steps = max(1, math.ceil(seconds / .002))
        for _ in range(steps):
            target = (c.c_int16 * 4)()
            self.lib.Device_Target(target)
            self.plant.targets = [v * s for v, s in zip(target, self.signs)]
            self.plant.integrate()
            tick = self.plant.ms
            if tick % 10 == 0:
                counts = (c.c_int64 * 4)(*[round(v * 65536) * s for v, s in zip(self.plant.turns, self.signs)])
                rpm = (c.c_int16 * 4)(*[round(v) * s for v, s in zip(self.plant.rpm, self.signs)])
                yaw = round(self.sensor_sign * self.plant.yaw / profile()['angle_scale_rad'])
                gyro = round(self.sensor_sign * self.plant.wz / profile()['gyro_scale_rad_s'])
                self.lib.Device_Tick(tick, counts, rpm, yaw, gyro)
            self.now = tick / 1000


class ExecutionTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        output = ROOT / 'output/tests'
        output.mkdir(exist_ok=True, parents=True)
        control = ROOT / 'firmware/User/Control'
        comm = ROOT / 'firmware/User/Comm'
        library = output / f'execution-{os.getpid()}.dll'
        compile_c(library, [*(control / name for name in ('exec.c', 'track.c', 'ref.c', 'odom.c',
                    'kin.c', 'yaw.c', 'curve.c', 'line.c', 'path.c')),
                    ROOT / 'firmware/User/Device/stamp.c', comm / 'frame.c', comm / 'comm.c',
                    comm / 'execute.c', comm / 'feedback.c', ROOT / 'tests/execution_native.c'], extra=['-shared'])
        cls.lib = c.CDLL(str(library))
        cls.lib.Device_Push.argtypes = [c.c_char_p, c.c_uint]
        cls.lib.Device_Read.argtypes = [c.c_void_p]
        cls.lib.Device_Read.restype = c.c_uint16
        cls.lib.Device_Tick.argtypes = [c.c_uint32, c.POINTER(c.c_int64), c.POINTER(c.c_int16), c.c_int16, c.c_int16]
        cls.lib.Device_Target.argtypes = [c.POINTER(c.c_int16)]
        cls.lib.Device_Value.argtypes = [c.c_uint]
        cls.lib.Device_Value.restype = c.c_uint32
        cls.lib.Device_Ref.argtypes = [c.c_double, c.POINTER(Reference)]

    def setUp(self):
        self.device = Device(self.lib)

    def test_boot_and_configuration_never_enable(self):
        self.device.open()
        self.device.assert_ok(self.device.call(CONFIG, config_payload(profile())))
        for _ in range(20):
            self.device.step()
        self.assertEqual(self.lib.Device_Value(4), 0)
        self.assertEqual(self.lib.Device_Value(1), 0)
        self.assertEqual(self.lib.Device_Value(3), 0)

    def test_goal_requires_separate_arm(self):
        self.device.open()
        self.device.call(CONFIG, config_payload(profile()))
        self.assertNotEqual(self.device.call(GOAL, trial('straight')).payload[0], p.OK)
        self.assertEqual(self.lib.Device_Value(1), 0)

    def test_config_binds_current_imu_epoch_and_later_change_stops(self):
        self.lib.Device_Epoch(2)
        self.device.step()
        self.device.arm()
        self.assertEqual(self.lib.Device_Value(0), 3)
        self.lib.Device_Epoch(3)
        for _ in range(30):
            self.device.step()
        self.assertEqual(self.lib.Device_Value(2), 1)
        self.assertEqual(self.lib.Device_Value(0), 6)

    def test_duplicate_goal_and_conflict_do_not_replay(self):
        self.device.arm()
        goal = trial('straight')
        self.device.assert_ok(self.device.call(GOAL, goal))
        self.device.assert_ok(self.device.call(GOAL, goal))
        self.assertNotEqual(self.device.call(GOAL, trial('side')).payload[0], p.OK)
        self.assertEqual(self.lib.Device_Value(1), 1)

    def test_session_expiry_stops_once(self):
        self.device.arm()
        self.device.call(GOAL, trial('straight'))
        for _ in range(90):
            self.device.step(heart=False)
        self.assertEqual(self.lib.Device_Value(2), 1)
        self.assertEqual(self.lib.Device_Value(0), 6)

    def test_can_fault_stops_and_never_rearms(self):
        self.device.arm()
        self.lib.Device_Error(2)
        self.device.step()
        for _ in range(50):
            self.device.step()
        self.assertEqual(self.lib.Device_Value(2), 1)
        self.assertEqual(self.lib.Device_Value(0), 6)
        self.assertNotEqual(self.device.call(ARM).payload[0], p.OK)

    def test_long_goal_and_arc_are_blocked(self):
        self.device.arm()
        goal = bytearray(trial('straight'))
        import struct
        struct.pack_into('<f', goal, 12, 1.)
        self.assertNotEqual(self.device.call(GOAL, bytes(goal)).payload[0], p.OK)
        struct.pack_into('<I', goal, 8, 3)
        self.assertNotEqual(self.device.call(GOAL, bytes(goal)).payload[0], p.OK)

    def test_only_bounded_fixed_heading_arc_is_accepted(self):
        import struct
        self.device.arm()
        # 不扩大总类权限：只接受这一轮已验算的小弧线，禁止任意半径或转姿。
        for offset, code, value in ((8, 'I', 3), (8, 'I', 5), (12, 'f', .01),
                                    (20, 'f', .1), (24, 'f', .02), (32, 'f', .07),
                                    (32, 'f', float('nan')), (40, 'i', -1), (44, 'i', 1),
                                    (48, 'f', .025), (72, 'I', 1), (76, 'f', .1),
                                    (80, 'f', .02), (92, 'f', .001)):
            goal = bytearray(trial('arc'))
            struct.pack_into('<' + code, goal, offset, value)
            self.assertNotEqual(self.device.call(GOAL, bytes(goal)).payload[0], p.OK)
        self.assertEqual(self.device.call(GOAL, trial('arc')).payload[0], p.OK)

    def test_arc_reference_stays_within_geometry_and_motion_limits(self):
        self.device.arm()
        self.assertEqual(self.device.call(GOAL, trial('arc')).payload[0], p.OK)
        point = Reference()
        maximum = dict(velocity=0., acceleration=0., jerk=0., wheel_rpm=0.)
        for index in range(3001):
            self.assertEqual(self.lib.Device_Ref(index / 3000, c.byref(point)), 1)
            self.assertLessEqual(math.hypot(point.x_m, point.y_m), .15)
            self.assertGreaterEqual(point.x_m, -1e-6)
            self.assertGreaterEqual(point.y_m, -1e-6)
            self.assertAlmostEqual(point.yaw_rad, 0.)
            maximum['velocity'] = max(maximum['velocity'], math.hypot(point.vx_m_s, point.vy_m_s))
            maximum['acceleration'] = max(maximum['acceleration'], math.hypot(point.ax_m_s2, point.ay_m_s2))
            maximum['jerk'] = max(maximum['jerk'], math.hypot(point.jx_m_s3, point.jy_m_s3))
            maximum['wheel_rpm'] = max(maximum['wheel_rpm'],
                                       (abs(point.vx_m_s) + abs(point.vy_m_s)) * 60 / (2 * math.pi * .05))
        for key, limit in dict(velocity=.018, acceleration=.08, jerk=.3, wheel_rpm=6).items():
            self.assertLessEqual(maximum[key], limit * 1.0003, (key, maximum[key]))
        report = dict(maximum=maximum, target_m_rad=[point.x_m, point.y_m, point.yaw_rad], samples=3001)
        (ROOT / 'output/tests/arc-reference.json').write_text(json.dumps(report, indent=2), encoding='utf-8')

    def test_imu_sign_missing_blocks_before_serial(self):
        value = profile()
        value['yaw_sign'] = None
        with self.assertRaises(ValueError):
            config_payload(value)

    def test_clockwise_positive_sensor_becomes_counterclockwise_positive(self):
        transport = SimTransport(self.lib)
        transport.sensor_sign = -1
        value = profile()
        value['yaw_sign'] = -1
        with tempfile.TemporaryDirectory() as directory:
            result = run_trial(transport, value, 'turn', directory, lambda *a, **k: None,
                               clock=lambda: transport.now, sleep=transport.sleep)
        self.assertTrue(result['controller_complete'])
        self.assertGreater(transport.plant.yaw, .15)
        self.assertLess(transport.plant.yaw, .20)

    def test_owner_running_with_disconnected_board_still_blocks(self):
        with patch('robot.drive_core.build_opener'):
            with self.assertRaises(ValueError):
                require_owner_stopped('http://172.20.10.4:8080')

    def test_observe_reads_raw_imu_without_arming(self):
        transport = SimTransport(self.lib)
        with tempfile.TemporaryDirectory() as directory:
            observe(transport, directory, 1.5, clock=lambda: transport.now, sleep=transport.sleep)
            state = json.loads((Path(directory) / 'state.json').read_text(encoding='utf-8'))
        self.assertTrue(state['execution']['stop_confirmed'])
        self.assertIn('4', state['raw_imu'])
        self.assertIn('5', state['raw_imu'])
        self.assertFalse(any(v.kind in (CONFIG, ARM, GOAL, p.STOP) for v in transport.requests))
        self.assertEqual(self.lib.Device_Value(4), 0)

    def test_selected_feedback_channel_and_original_stamp(self):
        self.device.open()
        self.device.step(yaw=123, gyro=45)
        reply = self.device.call(FEEDBACK, bytes([4, 0]))
        request = p.Packet(FEEDBACK, 123, 456, self.device.request, 250, bytes([4, 0]))
        sample = Feedback.read(reply, request, .010, .012)
        self.assertEqual(sample.values[2], 123)
        self.assertEqual(sample.last_ms, 10)
        wrong = p.Packet(FEEDBACK, 123, 456, self.device.request, 250, bytes([5, 0]))
        with self.assertRaises(ValueError):
            Feedback.read(reply, wrong, .010, .012)

    def test_motion_stop_preempts_pending_and_never_replays(self):
        link = MotionLink()
        link.bound = True
        link.boot = 123
        link.last_heart = 1
        link.submit(GOAL, trial('straight'))
        first = p.Decoder().feed(link.step(1))[0]
        self.assertEqual(first.kind, GOAL)
        link.stop()
        stopped = p.Decoder().feed(link.step(1.01))[0]
        self.assertEqual(stopped.kind, p.STOP)
        self.assertNotEqual(first.request, stopped.request)
        link.feed(p.Packet(GOAL | 0x80, 123, link.session, first.request, 0, b'\0').encode(), 1.02)
        self.assertEqual(link.pending[0].kind, p.STOP)

    def test_web_trials_are_readonly_and_snapshot_expires(self):
        with tempfile.TemporaryDirectory() as directory:
            client = TestClient(create_app(directory))
            self.assertFalse(client.get('/api/base/trials').json()['executable'])
            self.assertEqual(client.post('/api/base/trials', json={}).status_code, 405)
            state = dict(mode='serial', readonly=True, connected=False, snapshot_at_s=time.monotonic(),
                         snapshot_wall_s=time.time(), execution={'mode': 4}, execution_age_ms=100)
            target = Path(directory) / 'state.json'
            save_snapshot(target, state)
            self.assertIsNone(load_snapshot(target, now=state['snapshot_at_s'] + .2)['execution'])

    def test_stale_wheel_or_imu_stops(self):
        for mask in (1, 2, 4, 8):
            self.device = Device(self.lib)
            self.device.arm()
            self.device.call(GOAL, trial('straight'))
            self.lib.Device_Drop(mask)
            for _ in range(60):
                self.device.step()
            self.assertEqual(self.lib.Device_Value(2), 1)
            self.assertEqual(self.lib.Device_Value(0), 6)

    def test_normal_stop_can_observe_without_automatic_enable(self):
        self.device.arm()
        self.device.call(p.STOP)
        for _ in range(40):
            self.device.step()
        self.assertEqual(self.lib.Device_Value(0), 1)
        enabled = self.lib.Device_Value(4)
        self.device.call(CONFIG, config_payload(profile()))
        for _ in range(20):
            self.device.step()
        self.assertEqual(self.lib.Device_Value(4), enabled)
        self.assertEqual(self.lib.Device_Value(1), 1)

    def test_link_fault_during_normal_stop_cannot_rearm(self):
        self.device.arm()
        self.device.call(p.STOP)
        self.lib.Device_LinkFault(2)
        for _ in range(30):
            self.device.step(heart=False)
        self.assertEqual(self.lib.Device_Value(0), 6)
        self.assertNotEqual(self.lib.Device_Value(7), 0)
        self.assertEqual(self.lib.Device_Value(1), 1)

    def test_stop_reply_precedes_ordinary_queue(self):
        self.device.arm()
        self.device.write(p.Packet(STATUS, 123, 456, 99, 250).encode())
        self.device.write(p.Packet(p.STOP, 123, 456, 100, 250).encode())
        self.assertEqual(p.Decoder().feed(self.device.read())[0].kind, p.STOP | 0x80)
        self.assertEqual(self.lib.Device_Value(2), 1)

    def test_zero_tx_ack_is_required_for_completion(self):
        transport = SimTransport(self.lib)
        self.lib.Device_Ack(0)
        with tempfile.TemporaryDirectory() as directory:
            with self.assertRaises(ValueError):
                run_trial(transport, profile(), 'straight', directory, lambda *a, **k: None,
                          clock=lambda: transport.now, sleep=transport.sleep)
        self.assertNotEqual(self.lib.Device_Value(5), 4)
        self.assertEqual(sum(v.kind == GOAL for v in transport.requests), 1)

    def test_real_core_protocol_runs_short_cases_offline(self):
        summary = []
        for case in ('straight', 'side', 'turn', 'diagonal', 'arc'):
            transport = SimTransport(self.lib)
            records = []
            with tempfile.TemporaryDirectory() as directory:
                try:
                    result = run_trial(transport, profile(), case, directory,
                                       lambda kind, **values: records.append(dict(kind=kind, **values)),
                                       clock=lambda: transport.now, sleep=transport.sleep)
                except Exception:
                    (ROOT / 'output/drive/failed-trial.json').write_text(
                        json.dumps(dict(case=case, records=records), ensure_ascii=False), encoding='utf-8')
                    raise
                final = json.loads((Path(directory) / 'state.json').read_text(encoding='utf-8'))
                stop = next(v for v in reversed(records) if v['kind'] == 'stop_result')
                self.assertTrue(final['execution']['stop_confirmed'])
                self.assertEqual(final['execution'], stop['execution'])
            self.assertTrue(result['controller_complete'], (case, records[-5:]))
            self.assertEqual(sum(v.kind == GOAL for v in transport.requests), 1)
            self.assertEqual(sum(v.kind == p.STOP for v in transport.requests), 1)
            summary.append(dict(case=case, elapsed_ms=transport.plant.ms,
                                true_x_m=transport.plant.x, true_y_m=transport.plant.y,
                                true_yaw_rad=transport.plant.yaw, result=result))
        (ROOT / 'output/tests/execution-summary.json').write_text(
            json.dumps(summary, ensure_ascii=False, indent=2), encoding='utf-8')

    def test_lost_goal_ack_never_replays(self):
        class LostAck(SimTransport):
            def read(self, count=4096):
                data = super().read(count)
                packets = p.Decoder().feed(data)
                return b'' if any(v.kind == GOAL | 0x80 for v in packets) else data
        transport = LostAck(self.lib)
        with tempfile.TemporaryDirectory() as directory:
            with self.assertRaises(ValueError):
                run_trial(transport, profile(), 'straight', directory, lambda *a, **k: None,
                          clock=lambda: transport.now, sleep=transport.sleep)
        self.assertEqual(sum(v.kind == GOAL for v in transport.requests), 1)
        self.assertEqual(sum(v.kind == p.STOP for v in transport.requests), 1)


class ReusedBusTests(unittest.TestCase):
    def test_carcan13_transport_bus_and_protocol(self):
        output = ROOT / 'output/tests'
        output.mkdir(exist_ok=True, parents=True)
        device = ROOT / 'firmware/User/Device'
        cases = [('can_transport', ['can_tx.c'], ['-include', str(ROOT / 'tests/can_hal_stub.h')]),
                 ('motor_bus', ['bus.c', 'drive.c'], []), ('motor_protocol', ['drive.c'], [])]
        for name, sources, options in cases:
            program = output / f'reused_{name}.exe'
            compile_c(program, [ROOT / f'tests/reused_{name}.c', *(device / name for name in sources)],
                      extra=['-DCAR_CHASSIS_HOST_TEST', *options])
            for args in ([str(i)] for i in range(35)) if name == 'motor_bus' else [[]]:
                subprocess.run([str(program), *args], check=True, capture_output=True)


if __name__ == '__main__':
    unittest.main()
