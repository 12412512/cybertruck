"""真实驱动未接入前的接收时间链测试；正式C解码与Python跨语言互通。"""

import ctypes as c
from dataclasses import replace
import math
import os
from pathlib import Path
import random
import struct
import subprocess
import sys
import unittest

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'pi'))
from robot.device import protocol as p
from robot.device.feedback import Feedback, FeedbackCache, FEEDBACK, WIRE
from robot.device.can_diag import diagnose, compare


def packet(payload, boot=17, session=19, request=23):
    reply = p.Packet(FEEDBACK | 0x80, boot, session, request, 250, bytes([p.OK]) + payload)
    return p.Decoder().feed(reply.encode())[0]


class FeedbackTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        output = ROOT / 'output/tests'
        output.mkdir(exist_ok=True, parents=True)
        target = output / (f'feedback-{os.getpid()}.dll' if os.name == 'nt' else 'feedback.so')
        device = ROOT / 'firmware/User/Device'
        comm = ROOT / 'firmware/User/Comm'
        command = [sys.executable, '-m', 'ziglang', 'cc', '-shared', '-std=c99', '-O2',
                   '-Wall', '-Wextra', '-Werror', '-I', str(device), '-I', str(comm),
                   *(str(device / name) for name in ('stamp.c', 'motor.c', 'imu.c')),
                   str(comm / 'feedback.c'), str(ROOT / 'tests/feedback_native.c'), '-o', str(target)]
        if os.name != 'nt':
            command.append('-fPIC')
        subprocess.run(command, check=True, capture_output=True)
        cls.lib = c.CDLL(str(target))
        u32, u8 = c.POINTER(c.c_uint32), c.POINTER(c.c_uint8)
        cls.lib.Test_Stamp.argtypes = [u32, c.c_uint32]
        cls.lib.Test_Next.argtypes = [u32, u32]
        cls.lib.Test_Motor.argtypes = [c.c_uint32, u8, c.c_uint8, c.c_uint32,
                                      c.c_uint32, c.c_uint8, c.c_uint8,
                                      c.POINTER(c.c_int64), u32]
        cls.lib.Test_Read.argtypes = [c.c_uint32, c.POINTER(c.c_int16), u32, u8]
        cls.lib.Test_Encode.argtypes = [c.c_uint8, c.c_uint8, u32,
                                       c.POINTER(c.c_int64), c.c_uint32, u8]
        cls.lib.Test_Byte.argtypes = [c.c_uint8, c.c_uint32, c.c_uint32, c.c_uint32]
        for name in ('Test_Byte', 'Test_ImuInit', 'Test_Loss', 'Test_Tick', 'Test_Counts'):
            getattr(cls.lib, name).restype = None

    def setUp(self):
        self.request = p.Packet(FEEDBACK, 17, 19, 23, 250)
        self.lib.Test_ImuInit(7)

    @staticmethod
    def times(*values):
        return (c.c_uint32 * len(values))(*values)

    def motor(self, raw, id=0x300, received=100, handled=150, extended=1, remote=0):
        values = (c.c_uint8 * 8)(*(list(raw) + [0]*(8-len(raw))))
        value, times = c.c_int64(), (c.c_uint32 * 6)()
        result = self.lib.Test_Motor(id, values, len(raw), received, handled,
                                     extended, remote, c.byref(value), times)
        return result, value.value, list(times)

    @staticmethod
    def imu_frame(kind=0x53, axes=(16384, -32768, 0x55)):
        raw = bytearray([0x55, kind]) + bytearray(struct.pack('<3h', *axes)) + bytearray(2)
        raw.append(sum(raw) & 255)
        return bytes(raw)

    def feed(self, raw, first=100, handled=150, epoch=7, step=1):
        for index, byte in enumerate(raw):
            self.lib.Test_Byte(byte, (first+step*index) & 0xffffffff, handled & 0xffffffff, epoch)

    def read(self, now=150):
        axes, times, raw = (c.c_int16 * 3)(), (c.c_uint32 * 7)(), (c.c_uint8 * 11)()
        result = self.lib.Test_Read(now, axes, times, raw)
        return result, list(axes), list(times), bytes(raw)

    def encode(self, kind=1, address=3, values=(-20, 0, 0),
               times=(101, 101, 120, 7, 2, 9), report=130):
        output = (c.c_uint8 * 56)()
        result = self.lib.Test_Encode(kind, address, self.times(*times),
                                      (c.c_int64 * 3)(*values), report, output)
        return result, bytes(output)

    def decoded(self, **kwargs):
        ok, raw = self.encode(**kwargs)
        self.assertEqual(ok, 1)
        return Feedback.read(packet(raw), self.request, 1, 1.01)

    def test_stamp_wrap_same_tick_and_duplicate_rules(self):
        old = self.times(0xfffffff8, 0xfffffffa, 2, 0xffffffff, 4)
        self.assertEqual(self.lib.Test_Stamp(old, 5), 1)
        new = self.times(4, 4, 5, 0, 4)
        self.assertEqual(self.lib.Test_Next(old, new), 1)
        same_tick = self.times(4, 4, 5, 1, 4)
        self.assertEqual(self.lib.Test_Next(new, same_tick), 1)
        self.assertEqual(self.lib.Test_Next(new, new), 0)
        self.assertEqual(self.lib.Test_Next(new, self.times(4, 4, 5, 1, 5)), 0)

    def test_stamp_old_future_processing_and_half_range(self):
        for times, now in (((100, 101, 120, 1, 1), 351),
                           ((100, 101, 120, 1, 1), 119),
                           ((101, 100, 120, 1, 1), 120),
                           ((100, 101, 120, 1, 0), 120),
                           ((0x80000000, 0, 0, 1, 1), 1)):
            self.assertEqual(self.lib.Test_Stamp(self.times(*times), now), 0)

    def test_motor_values_original_rx_tick_and_large_encoder(self):
        ok, value, times = self.motor(bytes.fromhex('35 01 00 19 6B'))
        self.assertEqual((ok, value), (1, -25))
        self.assertEqual(times, [100, 100, 150, 73, 4, 0x35])
        self.assertEqual(self.motor(bytes.fromhex('36 01 FF FF FF FF 6B'))[:2], (1, -4294967295))
        self.assertEqual(self.motor(bytes.fromhex('3A 03 6B'))[:2], (1, 3))

    def test_motor_rejects_command_ack_bad_type_and_stale(self):
        raw = bytes.fromhex('35 00 00 01 6B')
        for frame in (bytes.fromhex('FE 02 6B'), bytes.fromhex('35 02 00 01 6B'),
                      bytes.fromhex('35 00 FF FF 6B'), bytes.fromhex('35 00 00 01 00')):
            self.assertEqual(self.motor(frame)[0], 0)
        for options in (dict(id=0), dict(id=0x301), dict(extended=0), dict(remote=1),
                        dict(handled=350), dict(handled=99)):
            self.assertEqual(self.motor(raw, **options)[0], 0)

    def test_imu_raw_axes_and_first_last_are_not_processing_time(self):
        raw = self.imu_frame()
        self.feed(raw, first=100, handled=200)
        valid, axes, times, actual = self.read(200)
        self.assertEqual(valid, 1)
        self.assertEqual(axes, [16384, -32768, 0x55])
        self.assertEqual(times, [100, 110, 200, 1, 1, 7, 0x53])
        self.assertEqual(actual, raw)

    def test_imu_overlap_resync_preserves_original_ticks(self):
        raw = self.imu_frame(0x52)
        self.feed(bytes([0x55, 0x53, 1, 2]) + raw, first=100, handled=200)
        valid, axes, times, actual = self.read(200)
        self.assertEqual(valid, 1)
        self.assertEqual(times[:3], [104, 114, 200])
        self.assertEqual(actual, raw)
        self.assertGreater(times[4], 1)

    def test_imu_source_change_loss_and_stale_queue(self):
        self.feed(self.imu_frame())
        self.lib.Test_Loss(8)
        self.assertEqual(self.read()[0], 2)
        self.feed(self.imu_frame(), first=200, handled=220, epoch=8)
        self.assertEqual(self.read(220)[0], 1)
        self.feed(self.imu_frame(), first=300, handled=320, epoch=8)
        self.assertEqual(self.read(560)[0], 2)

    def test_imu_gap_corruption_processing_lag_and_wrap(self):
        raw = self.imu_frame()
        self.feed(raw[:4])
        self.lib.Test_Tick(130)
        self.feed(raw[4:], first=140)
        self.assertEqual(self.read()[0], 0)
        self.lib.Test_ImuInit(7)
        self.feed(raw[:-1] + bytes([raw[-1] ^ 1]))
        self.assertEqual(self.read()[0], 0)
        self.lib.Test_ImuInit(7)
        self.feed(raw, first=100, handled=400)
        self.assertEqual(self.read(400)[0], 0)
        self.lib.Test_ImuInit(7)
        self.feed(raw, first=0xfffffffa, handled=20)
        valid, _, times, _ = self.read(20)
        self.assertEqual(valid, 1)
        self.assertEqual(times[:3], [0xfffffffa, 4, 20])

    def test_imu_queue_overflow_invalidates_old_continuity(self):
        for index in range(17):
            self.feed(self.imu_frame(), first=100+index, handled=140, step=0)
        counts = (c.c_uint32 * 5)()
        self.lib.Test_Counts(counts)
        self.assertEqual(list(counts)[3:], [1, 16])
        for _ in range(16):
            self.assertEqual(self.read()[0], 2)
        self.feed(self.imu_frame(), first=160, handled=180)
        self.assertEqual(self.read(180)[0], 1)

    def test_imu_reversed_rx_clock_invalidates_prior_queue(self):
        self.feed(self.imu_frame(), first=100, handled=150)
        self.feed(self.imu_frame(), first=90, handled=150)
        self.assertEqual(self.read()[0], 2)
        self.assertEqual(self.read()[0], 0)
        self.feed(self.imu_frame(), first=200, handled=220)
        self.assertEqual(self.read(220)[0], 1)

    def test_c_python_500_feedback_payloads_crc_and_units(self):
        rng = random.Random(61024)
        for _ in range(500):
            kind = rng.randint(1, 5)
            address = rng.randint(1, 255) if kind <= 3 else 0
            values = {1: (rng.randint(-5000, 5000), 0, 0),
                      2: (rng.randint(-0xffffffff, 0xffffffff), 0, 0),
                      3: (rng.randint(0, 255), 0, 0),
                      4: tuple(rng.randint(-32768, 32767) for _ in range(3)),
                      5: tuple(rng.randint(-32768, 32767) for _ in range(3))}[kind]
            sample = self.decoded(kind=kind, address=address, values=values)
            self.assertEqual(sample.values, values)
            self.assertEqual((sample.first_ms, sample.last_ms, sample.handled_ms), (101, 101, 120))
            self.assertAlmostEqual(sample.age_ms(1.02), 49)

    def test_wire_bad_stamp_values_and_session_are_rejected(self):
        for options in (dict(values=(5001, 0, 0)), dict(values=(1, 1, 0)), dict(address=0),
                        dict(report=351), dict(times=(102, 101, 120, 7, 2, 9)),
                        dict(times=(100, 101, 120, 7, 0, 9))):
            self.assertEqual(self.encode(**options)[0], 0)
        _, raw = self.encode()
        for wrong in (packet(raw, boot=18), packet(raw, session=20), packet(raw, request=24)):
            with self.assertRaises(ValueError):
                Feedback.read(wrong, self.request, 1, 1.01)
        for first, last in ((1, 1.25), (2, 1), (float('nan'), 2)):
            with self.assertRaises(ValueError):
                Feedback.read(packet(raw), self.request, first, last)
        tampered = bytearray(raw)
        tampered[3] = 1
        with self.assertRaises(ValueError):
            Feedback.read(packet(bytes(tampered)), self.request, 1, 1.01)

    def test_cache_duplicates_do_not_refresh_and_conflicts_latch(self):
        cache = FeedbackCache(17, 19)
        sample = self.decoded()
        self.assertTrue(cache.push(sample))
        self.assertFalse(cache.push(replace(sample, response_s=1.2, report_ms=200)))
        self.assertIsNone(cache.get(1, 3, 1.3))
        with self.assertRaises(ValueError):
            cache.push(replace(sample, values=(1, 0, 0)))
        self.assertIsNotNone(cache.fault)
        self.assertIsNone(cache.get(1, 3, 1.02))

    def test_cache_cross_channel_epoch_reset_and_reverse_clock(self):
        sample = self.decoded()
        for changed in (replace(sample, boot=18), replace(sample, source_epoch=10, address=4),
                        replace(sample, sequence=6), replace(sample, sequence=8, last_ms=99)):
            cache = FeedbackCache(17, 19)
            cache.push(sample)
            with self.assertRaises(ValueError):
                cache.push(changed)
            self.assertEqual(cache.samples, {})
        cache = FeedbackCache(17, 19)
        cache.push(sample)
        self.assertIsNone(cache.get(1, 3, .9))
        self.assertIsNotNone(cache.fault)

    def test_payload_and_cache_uint32_wrap(self):
        sample = self.decoded(times=(0xfffffffa, 0xfffffffa, 3, 0xffffffff, 2, 9), report=5)
        cache = FeedbackCache(17, 19)
        cache.push(sample)
        next_sample = self.decoded(times=(6, 6, 7, 0, 2, 9), report=8)
        self.assertTrue(cache.push(next_sample))


def diagnostic_state():
    return dict(firmware='CARCAN13', board=dict(uptime_ms=2000000, can_errors=168503,
                action=0, can_rx_sequence=100, state=5, reason=2, bus_error=3,
                stop_observed_mask=15, pending_stops=0),
                can_diagnostics=['00000000','1C000000','001B0003','00800033','1C000009',
                                 '2','0','0','168503','0','0'],
                can_error_event=dict(valid=True, sequence=168503, tick_ms=1686069,
                    hal_error=4096, esr=0x800033, tsr=0x1c000009, tx_id=0x300,
                    tx_length=4, tx_data0=0x6b0098fe, tx_data1=0, mailbox=1,
                    transport_state=1, tx_pending=1, tx_index=0, tx_count=1))


class CanDiagnosisTests(unittest.TestCase):
    def test_current_frozen_and_software_are_separate(self):
        result = diagnose(diagnostic_state())
        self.assertEqual(result['current']['esr'], 0)
        event = result['latched']
        self.assertEqual(event['payload_hex'], 'FE 98 00 6B')
        self.assertEqual(event['address'], 3)
        self.assertTrue(event['is_stop_frame'])
        self.assertFalse(event['pure_arbitration'])
        self.assertTrue(event['mailbox0_transmit_error'])
        self.assertEqual(event['decoded_esr']['lec'], 3)
        self.assertTrue(event['decoded_esr']['passive'])
        self.assertFalse(event['decoded_esr']['bus_off'])
        self.assertEqual(event['decoded_esr']['tec_low8'], 128)
        self.assertTrue(result['software']['can_latched'])
        self.assertFalse(result['conclusions']['fault_cleared'])
        self.assertEqual(result['current']['bit_timing']['segment1'], 12)

    def test_compare_new_errors_and_restart_are_not_hidden(self):
        first, last = diagnostic_state(), diagnostic_state()
        last['board'].update(uptime_ms=2001000, can_rx_sequence=300)
        result = compare(first, last)
        self.assertEqual((result['can_error_delta'], result['action_delta'], result['can_rx_delta']), (0, 0, 200))
        last['board']['can_errors'] += 2
        self.assertEqual(compare(first, last)['can_error_delta'], 2)
        last['board']['uptime_ms'] = 20
        with self.assertRaises(ValueError):
            compare(first, last)
