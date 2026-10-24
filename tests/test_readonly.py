"""协议/固件状态机/Web只读性回归；所有传输均为主机模拟。"""

import ctypes
import os
from pathlib import Path
import random
import struct
import subprocess
import sys
import tempfile
import time
import unittest

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'pi'))

from robot.device import protocol as p
from robot.device.link import Link
from robot.runtime import ProcessLock, load_snapshot, save_snapshot
from robot.web.app import create_app
from fastapi.testclient import TestClient


class Native:
    def __init__(self):
        output = ROOT / 'output/tests'
        output.mkdir(parents=True, exist_ok=True)
        library = output / ('readonly.dll' if os.name == 'nt' else 'readonly.so')
        command = [sys.executable, '-m', 'ziglang', 'cc', '-shared', '-std=c99', '-O2',
                   '-Wall', '-Wextra', '-Werror', '-I', str(ROOT / 'firmware/User/Comm'),
                   str(ROOT / 'firmware/User/Comm/frame.c'),
                   str(ROOT / 'firmware/User/Comm/comm.c'), str(ROOT / 'tests/native.c'),
                   '-o', str(library)]
        if os.name != 'nt':
            command.insert(5, '-fPIC')
        subprocess.run(command, check=True, capture_output=True)
        self.lib = ctypes.CDLL(str(library))
        self.lib.Native_Init.argtypes = [ctypes.c_uint64, ctypes.c_uint32]
        self.lib.Native_Tick.argtypes = [ctypes.c_uint32]
        self.lib.Native_Fault.argtypes = [ctypes.c_uint32]
        self.lib.Native_Push.argtypes = [ctypes.c_char_p, ctypes.c_uint16,
                                       ctypes.c_uint32, ctypes.c_uint32]
        self.lib.Native_Read.argtypes = [ctypes.c_void_p]
        self.lib.Native_Read.restype = ctypes.c_uint16
        self.lib.Native_RoundTrip.argtypes = [ctypes.c_char_p, ctypes.c_uint16, ctypes.c_void_p]
        self.lib.Native_RoundTrip.restype = ctypes.c_uint16

    def reset(self, boot=123, now=0):
        self.lib.Native_Init(boot, now)

    def push(self, wire, now=0, received=None):
        self.lib.Native_Push(wire, len(wire), now if received is None else received, now)

    def read(self):
        out = ctypes.create_string_buffer(160)
        size = self.lib.Native_Read(out)
        return out.raw[:size]

    def request(self, kind, now=0, boot=123, session=456, payload=b'', ttl=250):
        self.push(p.Packet(kind, boot, session, 1, ttl, payload).encode(), now)
        return p.Decoder().feed(self.read())[0]

    def open(self, now=0):
        hello = self.request(p.HELLO, now, boot=0, session=0)
        generation = hello.payload[1:5]
        return self.request(p.OPEN, now, payload=generation)


class ProtocolTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.native = Native()

    def setUp(self):
        self.native.reset()

    def test_python_c_random_roundtrips(self):
        rng = random.Random(231024)
        out = ctypes.create_string_buffer(160)
        for _ in range(500):
            packet = p.Packet(rng.randrange(256), rng.getrandbits(64), rng.getrandbits(64),
                              rng.getrandbits(32), rng.randrange(65536),
                              rng.randbytes(rng.randrange(129)))
            wire = packet.encode()
            size = self.native.lib.Native_RoundTrip(wire, len(wire), out)
            self.assertEqual(out.raw[:size], wire)
            self.assertEqual(p.Decoder().feed(wire), [packet])

    def test_split_glued_empty_and_binary_payload(self):
        packet = p.Packet(4, payload=bytes(range(128)))
        wire = packet.encode()
        for index in range(len(wire) + 1):
            decoder = p.Decoder()
            self.assertEqual(decoder.feed(wire[:index]) + decoder.feed(wire[index:]), [packet])
        self.assertEqual(p.Decoder().feed(b'\0\0' + wire * 3), [packet] * 3)

    def test_crc_corruption_and_oversize_recover(self):
        wire = p.Packet(p.HELLO).encode()
        bad = bytearray(wire)
        bad[5] ^= 0x20
        decoder = p.Decoder()
        self.assertEqual(decoder.feed(bytes(bad) + b'\x01' * 500 + b'\0' + wire), [p.Packet(p.HELLO)])
        self.assertGreaterEqual(decoder.bad, 1)
        self.native.push(bytes(bad) + b'\x01' * 500 + b'\0' + wire)
        self.assertEqual(p.Decoder().feed(self.native.read())[0].kind, p.HELLO | 0x80)

    def test_wrong_magic_version_size_and_crc(self):
        raw = p.cobs_decode(p.Packet(p.HELLO).encode()[:-1])
        for index in (0, 2, 26, len(raw) - 1):
            bad = bytearray(raw)
            bad[index] ^= 1
            wire = p.cobs_encode(bad) + b'\0'
            self.assertEqual(p.Decoder().feed(wire), [])
            self.native.push(wire)
            self.assertEqual(self.native.read(), b'')

    def test_boot_session_and_readonly_rejections(self):
        self.assertEqual(self.native.open().payload[0], p.OK)
        self.assertEqual(self.native.request(p.STATE, boot=999).payload[0], p.BOOT)
        self.assertEqual(self.native.request(p.STATE, session=777).payload[0], p.SESSION)
        for kind in (0x10, 0x11, 0x12, 0x20):
            self.assertEqual(self.native.request(kind, payload=b'move').payload[0], p.READONLY)

    def test_no_random_boot_disables_open(self):
        self.native.reset(boot=0)
        self.assertEqual(self.native.open().payload[0], p.BOOT)

    def test_bad_command_shape_and_zero_ttl(self):
        self.assertEqual(self.native.request(p.HELLO, session=0, payload=b'x').payload[0], p.FORMAT)
        self.assertEqual(self.native.request(p.OPEN, payload=b'x').payload[0], p.FORMAT)
        self.assertEqual(self.native.request(p.HELLO, session=0, ttl=0).payload[0], p.EXPIRED)
        self.assertEqual(self.native.request(p.HELLO, session=0, ttl=1001).payload[0], p.EXPIRED)

    def test_duplicate_open_and_heartbeat_do_not_extend_lease(self):
        self.native.open()
        self.native.request(p.HEART, now=10, payload=struct.pack('<I', 1))
        self.native.open(now=300)
        self.native.request(p.HEART, now=499, payload=struct.pack('<I', 1))
        self.assertEqual(self.native.request(p.STATE, now=510).payload[0], p.SESSION)
        replay = self.native.request(p.OPEN, now=511, payload=struct.pack('<I', 1))
        self.assertEqual(replay.payload[0], p.SESSION)

    def test_second_client_cannot_take_bound_session(self):
        self.native.open()
        result = self.native.request(p.OPEN, session=999, payload=struct.pack('<I', 1))
        self.assertEqual(result.payload[0], p.BUSY)

    def test_stop_has_reserved_reply_but_no_physical_stop_claim(self):
        self.native.open()
        for _ in range(20):
            self.native.push(p.Packet(p.STATE, 123, 456).encode())
        self.native.push(p.Packet(p.STOP).encode())
        first = p.Decoder().feed(self.native.read())[0]
        self.assertEqual(first.kind, p.STOP | 0x80)
        self.assertEqual(first.payload[0], p.READONLY)
        self.assertEqual(p.Decoder().feed(self.native.read())[0].kind, p.STATE | 0x80)

    def test_queue_age_and_tick_wrap(self):
        late = self.native.request(p.HELLO, now=0, session=0, boot=0)
        self.assertEqual(late.payload[0], p.OK)
        wire = p.Packet(p.HELLO).encode()
        self.native.push(wire, now=251, received=0)
        self.assertEqual(p.Decoder().feed(self.native.read())[0].payload[0], p.EXPIRED)
        self.native.reset(now=0xFFFFFF00)
        self.native.open(now=0xFFFFFF00)
        self.assertEqual(self.native.request(p.STATE, now=0x20).payload[0], p.OK)
        self.assertEqual(self.native.request(p.STATE, now=0x200).payload[0], p.SESSION)

    def test_fault_flushes_partial_input_and_old_session(self):
        self.native.open()
        wire = p.Packet(p.STATE, 123, 456).encode()
        self.native.push(wire[:10])
        self.native.lib.Native_Fault(2)
        self.native.push(wire[10:])
        self.assertEqual(self.native.read(), b'')
        self.assertEqual(self.native.request(p.STATE).payload[0], p.SESSION)

    def test_fault_discards_unsent_old_healthy_state(self):
        self.native.open()
        self.native.push(p.Packet(p.STATE, 123, 456).encode())
        self.native.lib.Native_Fault(2)
        self.assertEqual(self.native.read(), b'')
        self.native.push(b'\0')
        self.native.open()
        reply = self.native.request(p.STATE)
        self.assertEqual(p.STATE_DATA.unpack(reply.payload[1:])[1], 2)

    def test_python_core_operates_real_c_peer_and_reboot(self):
        link = Link()
        for tick in range(0, 2000, 5):
            self.native.lib.Native_Tick(tick)
            wire = link.step(tick / 1000)
            if wire:
                self.native.push(wire, tick)
            link.feed(self.native.read(), tick / 1000)
        state = link.snapshot(2)
        self.assertTrue(state['connected'])
        self.assertFalse(state['ready'])
        self.assertIsNone(state['pose'])
        self.assertGreater(state['board']['heartbeats'], 10)
        old_session = link.session
        self.native.reset(boot=999, now=2000)
        for tick in range(2000, 3000, 5):
            wire = link.step(tick / 1000)
            if wire:
                self.native.push(wire, tick)
            link.feed(self.native.read(), tick / 1000)
        self.assertEqual(link.boot, 999)
        self.assertNotEqual(link.session, old_session)
        self.assertTrue(link.snapshot(3)['connected'])

    def test_lost_ack_stale_reply_and_stopped_core(self):
        link = Link()
        request = p.Decoder().feed(link.step(0))[0]
        link.step(.3)
        late = p.Packet(request.kind | 0x80, 123, request.session, request.request, 0,
                        b'\0' + struct.pack('<II16s', 1, 1, b'GC-RO-0.2.0')).encode()
        link.feed(late, .31)
        self.assertIsNone(link.boot)
        self.assertFalse(link.snapshot(.31)['connected'])
        self.native.open()
        self.native.lib.Native_Tick(501)
        self.assertEqual(self.native.request(p.STATE, now=501).payload[0], p.SESSION)


class WebTests(unittest.TestCase):
    def setUp(self):
        self.directory = tempfile.TemporaryDirectory()
        self.runtime = Path(self.directory.name)

    def tearDown(self):
        self.directory.cleanup()

    def snapshot(self, age=0):
        return dict(snapshot_at_s=time.monotonic() - age, snapshot_wall_s=time.time(), mode='sim',
                    connected=True, readonly=True, ready=False, board={'flags': 3},
                    events=[], message='模拟数据', pose=None, pose_valid=False,
                    zero_valid=False, physical_stop_confirmed=False)

    def test_missing_invalid_and_stale_snapshot_clear_online(self):
        path = self.runtime / 'state.json'
        self.assertFalse(load_snapshot(path)['connected'])
        path.write_text('broken', encoding='utf-8')
        self.assertFalse(load_snapshot(path)['connected'])
        save_snapshot(path, self.snapshot(age=1))
        self.assertFalse(load_snapshot(path)['connected'])
        save_snapshot(path, self.snapshot(age=-1))
        self.assertFalse(load_snapshot(path)['connected'])

    def test_only_get_and_push_no_action_api(self):
        with TestClient(create_app(self.runtime)) as client:
            self.assertEqual(client.get('/').status_code, 200)
            self.assertFalse(client.get('/api/state').json()['core_fresh'])
            for path in ('/api/state', '/api/move', '/api/zero', '/api/stop', '/api/config'):
                self.assertIn(client.post(path, json={'target': 100}).status_code, (404, 405))

    def test_websocket_multiple_readers_and_core_expiry(self):
        save_snapshot(self.runtime / 'state.json', self.snapshot())
        with TestClient(create_app(self.runtime)) as client:
            with client.websocket_connect('/ws') as first, client.websocket_connect('/ws') as second:
                self.assertEqual(first.receive_json()['mode'], 'sim')
                self.assertTrue(second.receive_json()['core_fresh'])
                save_snapshot(self.runtime / 'state.json', self.snapshot(age=1))
                # 已排队的快照最多保留一个推送周期，读到过期撤销状态。
                for _ in range(10):
                    if not first.receive_json()['core_fresh']:
                        break
                else:
                    self.fail('核心停止后未撤销在线状态')

    def test_exclusive_core_lock(self):
        first = ProcessLock(self.runtime / 'core.lock')
        try:
            with self.assertRaises(RuntimeError):
                ProcessLock(self.runtime / 'core.lock')
        finally:
            first.close()
        ProcessLock(self.runtime / 'core.lock').close()

    def test_separate_core_simulation_and_shutdown(self):
        command = [sys.executable, '-m', 'robot.main', '--mode', 'sim', '--runtime', str(self.runtime)]
        core = subprocess.Popen(command, cwd=ROOT / 'pi', stdout=subprocess.PIPE, stderr=subprocess.PIPE)
        try:
            deadline = time.monotonic() + 5
            while time.monotonic() < deadline:
                state = load_snapshot(self.runtime / 'state.json')
                if state.get('connected'):
                    break
                time.sleep(.05)
            self.assertTrue(state['connected'])
            self.assertEqual(state['mode'], 'sim')
            self.assertFalse(state['ready'])
        finally:
            core.terminate()
            core.communicate(timeout=5)
        time.sleep(.6)
        self.assertFalse(load_snapshot(self.runtime / 'state.json')['connected'])


if __name__ == '__main__':
    unittest.main(verbosity=2)
