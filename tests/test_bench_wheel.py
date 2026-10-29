"""单轮实测工具的失效与单次发送约束；全部使用替身，不访问设备。"""

from copy import deepcopy
import sys
from pathlib import Path
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'tools'))
from bench_wheel import BenchError, run
from bench_wheel import check


def state(tick, action=0, completed=False):
    return dict(firmware='CARCAN13', connected=True, state_age_ms=10, ready=True,
                stopped=True, control_busy=False, control_mine=False, action=None,
                board=dict(state=1, reason=5 if completed else 2, bus_error=0,
                           can_errors=0, uptime_ms=tick, action=action,
                           can_rx_sequence=tick, ready_mask=15,
                           stop_observed_mask=15, pending_stops=0),
                wheels=[dict(address=n, rpm=0, speed_fresh=True, position_fresh=True,
                             fresh=True, status=0) for n in range(1, 5)],
                last_action=(dict(result='completed', action=1, targets=[6, 0, 0, 0],
                                  duration_ms=600) if completed else {}))


class Fake:
    def __init__(self, rows, lose_ack=False):
        self.rows = iter(rows)
        self.calls = []
        self.lose_ack = lose_ack
        self.now = 0

    def call(self, path, body=None):
        self.calls.append((path, body))
        if path == '/api/state':
            return deepcopy(next(self.rows))
        if path == '/api/motion':
            if self.lose_ack:
                raise TimeoutError('动作可能已经接受')
            return dict(accepted=True, token='12345678', wheel_rpm=[6, 0, 0, 0], duration_ms=600)
        return {}

    def sleep(self, duration):
        self.now += duration

    def run(self):
        run(self, 1, lambda *args, **kw: None, clock=lambda: self.now, sleep=self.sleep)


class BenchTests(unittest.TestCase):
    def test_one_motion_and_post_release_feedback(self):
        fake = Fake([state(100), state(200), state(300), state(400, 1, True),
                     state(500, 1, True), state(700, 1, True)])
        fake.run()
        self.assertEqual([body for path, body in fake.calls if path == '/api/motion'],
                         [dict(wheel_rpm=[6, 0, 0, 0], duration_ms=600)])
        self.assertEqual(sum(path == '/api/control/release' for path, _ in fake.calls), 1)
        self.assertEqual(sum(path == '/api/stop' for path, _ in fake.calls), 0)

    def test_lost_motion_ack_stops_without_replay(self):
        fake = Fake([state(100), state(200), state(300)], lose_ack=True)
        with self.assertRaises(TimeoutError):
            fake.run()
        self.assertEqual(sum(path == '/api/motion' for path, _ in fake.calls), 1)
        self.assertEqual(sum(path == '/api/stop' for path, _ in fake.calls), 1)

    def test_stale_baseline_never_acquires(self):
        stale = state(100)
        stale['wheels'][0]['speed_fresh'] = False
        fake = Fake([stale])
        with self.assertRaises(BenchError):
            fake.run()
        self.assertFalse(any(body is not None for _, body in fake.calls))

    def test_can_error_stops_once(self):
        fault = state(400, 1)
        fault['board']['can_errors'] = 1
        fake = Fake([state(100), state(200), state(300), fault])
        with self.assertRaises(BenchError):
            fake.run()
        self.assertEqual(sum(path == '/api/motion' for path, _ in fake.calls), 1)
        self.assertEqual(sum(path == '/api/stop' for path, _ in fake.calls), 1)

    def test_frozen_board_never_acquires(self):
        fake = Fake([state(100), state(100)])
        with self.assertRaises(BenchError):
            fake.run()
        self.assertFalse(any(body is not None for _, body in fake.calls))

    def test_running_status_is_bounded_cached_not_live(self):
        running = state(400, 1)
        running['board']['state'] = 4
        for wheel in running['wheels']:
            wheel.update(fresh=False, status_age_ms=300)
        check(running)
        running['wheels'][0]['status_age_ms'] = 851
        with self.assertRaises(BenchError):
            check(running)

    def test_running_stale_speed_still_stops(self):
        running = state(400, 1)
        running['board']['state'] = 4
        for wheel in running['wheels']:
            wheel.update(fresh=False, status_age_ms=300)
        running['wheels'][0]['speed_fresh'] = False
        with self.assertRaises(BenchError):
            check(running)

    def test_non_selected_wheel_motion_stops(self):
        running = state(400, 1)
        running['wheels'][1]['rpm'] = 6
        fake = Fake([state(100), state(200), state(300), running])
        with self.assertRaises(BenchError):
            fake.run()
        self.assertEqual(sum(path == '/api/stop' for path, _ in fake.calls), 1)

    def test_old_cancelled_result_is_not_current_action(self):
        running = state(400, 1)
        running['action'] = dict(token='12345678', phase='running')
        running['last_action'] = dict(result='cancelled')
        fake = Fake([state(100), state(200), state(300), running,
                     state(500, 1, True), state(600, 1, True), state(800, 1, True)])
        fake.run()
        self.assertEqual(sum(path == '/api/motion' for path, _ in fake.calls), 1)

    def test_lost_acquire_ack_revokes_without_motion(self):
        class LostAcquire(Fake):
            def call(self, path, body=None):
                result = super().call(path, body)
                if path == '/api/control/acquire':
                    raise TimeoutError('控制租约可能已取得')
                return result
        fake = LostAcquire([state(100), state(200)])
        with self.assertRaises(TimeoutError):
            fake.run()
        self.assertEqual(sum(path == '/api/motion' for path, _ in fake.calls), 0)
        self.assertEqual(sum(path == '/api/stop' for path, _ in fake.calls), 1)

    def test_post_release_waits_for_new_stop_feedback(self):
        pending = state(500, 1)
        pending.update(stopped=False, ready=False)
        pending['wheels'][0]['speed_fresh'] = False
        fake = Fake([state(100), state(200), state(300), state(400, 1, True), pending,
                     state(700, 1, True), state(900, 1, True)])
        fake.run()
        self.assertEqual(sum(path == '/api/motion' for path, _ in fake.calls), 1)


if __name__ == '__main__':
    unittest.main()
