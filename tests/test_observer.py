"""旧所有者观察的时效/复位/协议隔离；网络只访问测试HTTP服务。"""

from copy import deepcopy
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
import json
from pathlib import Path
import sys
import tempfile
import threading
import unittest
from unittest.mock import patch
from urllib.error import HTTPError

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'pi'))
from robot.device.observer import Observer, OwnerClient
from robot.main import run


def sample(uptime=1000):
    return dict(firmware='CARCAN13', connected=True, state_age_ms=20, stopped=False,
                board=dict(uptime_ms=uptime, state=5, state_name='FAULT', reason=2,
                           bus_error=3, can_errors=10, can_last_error=4096),
                wheels=[dict(address=1, rpm=0, position=3, status=3,
                             speed_age_ms=30, position_age_ms=20, status_age_ms=10,
                             fresh=True, speed_fresh=True, position_fresh=True)],
                imu=dict(supported=True, angle_fresh=True, gyro_fresh=True,
                         angle_age_ms=5, gyro_age_ms=10, yaw_deg=3, pitch_deg=0, roll_deg=0,
                         gx_dps=0, gy_dps=0, gz_dps=0))


class ObserverTests(unittest.TestCase):
    def connected(self):
        observer = Observer()
        observer.feed(sample(1000), 1, 1.01)
        self.assertFalse(observer.snapshot(1.01)['connected'])
        observer.feed(sample(1100), 1.1, 1.11)
        self.assertTrue(observer.snapshot(1.11)['connected'])
        return observer

    def test_fault_data_not_ready_or_gc_success(self):
        state = self.connected().snapshot(1.12)
        self.assertFalse(state['ready'])
        self.assertFalse(state['physical_stop_confirmed'])
        self.assertFalse(state['gc_verified'])
        self.assertIsNone(state['grabs'])
        self.assertEqual(state['board']['can_last_error'], 4096)
        self.assertAlmostEqual(state['state_age_ms'], 40)
        self.assertTrue(state['wheels'][0]['speed_reported'])
        self.assertFalse(state['wheels'][0]['speed_valid'])
        self.assertIsNone(state['wheels'][0]['speed_age_ms'])
        self.assertFalse(state['feedback_timing_verified'])
        self.assertTrue(state['imu']['angle_reported'])
        self.assertFalse(state['imu']['angle_fresh'])

    def test_frozen_uptime_cannot_refresh_itself(self):
        observer = self.connected()
        observer.feed(sample(1100), 1.3, 1.31)
        self.assertTrue(observer.snapshot(1.31)['connected'])
        observer.feed(sample(1100), 1.4, 1.41)
        state = observer.snapshot(1.41)
        self.assertFalse(state['connected'])
        self.assertIsNone(state['board'])
        self.assertIsNone(state['imu'])
        self.assertEqual(state['wheels'], [])

    def test_reset_and_wrap_require_correct_progress(self):
        observer = self.connected()
        observer.feed(sample(20), 1.2, 1.21)
        self.assertFalse(observer.snapshot(1.21)['connected'])
        observer.feed(sample(120), 1.3, 1.31)
        self.assertTrue(observer.snapshot(1.31)['connected'])
        observer = Observer()
        observer.feed(sample(0xfffffff0), 1, 1.01)
        observer.feed(sample(80), 1.1, 1.11)
        self.assertTrue(observer.snapshot(1.11)['connected'])

    def test_stale_source_roundtrip_and_local_expiry(self):
        for age, rtt in ((250, .01), (20, .25), (245, .01)):
            observer = self.connected()
            state = sample(1200)
            state['state_age_ms'] = age
            observer.feed(state, 1.2, 1.2 + rtt)
            self.assertFalse(observer.snapshot(1.2 + rtt)['connected'])
        observer = self.connected()
        self.assertFalse(observer.snapshot(1.4)['connected'])

    def test_individual_fields_expire_and_source_false_is_retained(self):
        observer = self.connected()
        state = sample(1200)
        state['wheels'][0]['speed_age_ms'] = 250
        state['imu']['angle_fresh'] = False
        state['imu']['gyro_age_ms'] = 250
        observer.feed(state, 1.2, 1.21)
        result = observer.snapshot(1.21)
        self.assertTrue(result['connected'])
        self.assertFalse(result['wheels'][0]['speed_valid'])
        self.assertFalse(result['wheels'][0]['speed_reported'])
        self.assertFalse(result['imu']['angle_reported'])
        self.assertFalse(result['imu']['gyro_reported'])
        self.assertFalse(result['imu']['angle_fresh'])
        self.assertFalse(result['imu']['gyro_fresh'])

    def test_malformed_and_unsupported_states(self):
        for state in ([], None, {}, dict(sample(), firmware='GC-RO-0.2.0'),
                      dict(sample(), state_age_ms=float('nan')),
                      dict(sample(), board=dict(sample()['board'], state=True))):
            with self.assertRaises(ValueError):
                Observer().feed(deepcopy(state), 1, 1.01)

    def test_origin_restriction(self):
        for origin in ('https://localhost', 'http://u:p@localhost',
                       'http://localhost/api/arm', 'http://localhost?x=1',
                       'http://localhost/#a', 'file:///dev/ttyUSB0'):
            with self.assertRaises(ValueError):
                OwnerClient(origin)

    def test_network_only_get_fixed_endpoint_no_redirect(self):
        requests = []
        class Handler(BaseHTTPRequestHandler):
            redirect = False
            def do_GET(self):
                requests.append((self.command, self.path))
                self.send_response(302 if self.redirect else 200)
                if self.redirect:
                    self.send_header('Location', '/api/arm')
                self.end_headers()
                if not self.redirect:
                    self.wfile.write(json.dumps(sample()).encode())
            def log_message(self, *args):
                pass
        server = ThreadingHTTPServer(('127.0.0.1', 0), Handler)
        thread = threading.Thread(target=server.serve_forever, daemon=True)
        thread.start()
        try:
            client = OwnerClient(f'http://127.0.0.1:{server.server_port}')
            state, sent, received = client.read()
            self.assertEqual(state['firmware'], 'CARCAN13')
            self.assertGreaterEqual(received, sent)
            Handler.redirect = True
            with self.assertRaises(HTTPError):
                client.read()
            self.assertEqual(requests, [('GET', '/api/state')] * 2)
        finally:
            server.shutdown()
            server.server_close()
            thread.join()

    def test_core_observation_never_opens_serial_or_simulation_ledger(self):
        with tempfile.TemporaryDirectory() as directory:
            runtime = Path(directory)
            ledger = runtime / 'task.sqlite3'
            ledger.write_bytes(b'not-a-simulation-ledger')
            ticks = iter((False, False, False, True))
            samples = iter((sample(100), sample(200), sample(300)))
            def read():
                import time
                now = time.monotonic()
                return next(samples), now, now
            with patch('robot.main.open_transport', side_effect=AssertionError('不得打开设备')):
                with patch.object(OwnerClient, 'read', side_effect=read):
                    run('observer', runtime=runtime, owner_url='http://127.0.0.1:8080',
                        stop=lambda: next(ticks))
            state = json.loads((runtime / 'state.json').read_text(encoding='utf-8'))
            self.assertTrue(state['connected'])
            self.assertIsNone(state['task'])
            self.assertIsNone(state['grabs'])
            self.assertEqual(ledger.read_bytes(), b'not-a-simulation-ledger')
