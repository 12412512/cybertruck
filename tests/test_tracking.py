"""正式C跟踪器的闭环与异常证据；不导入任何真实串口实现。"""

import json
import math
import os
from pathlib import Path
import subprocess
import sys
import tempfile
import time
import unittest
from unittest.mock import patch

from fastapi.testclient import TestClient

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'pi'))
from robot.track_sim import ClosedLoop, goal, MASK
from robot.main import run
from robot.runtime import save_snapshot
from robot.web.app import create_app


def clone(value):
    return type(value).from_buffer_copy(bytes(value))


class TrackingTests(unittest.TestCase):
    def test_lag_feedforward_is_opt_in_and_bounded(self):
        baseline = ClosedLoop(self.library, auto_start=False)
        self.assertEqual(baseline.config.lag_ff_s, 0.)
        for value in (-.001, .151, float('nan')):
            config = clone(baseline.config)
            config.lag_ff_s = value
            with self.assertRaises(ValueError):
                ClosedLoop(self.library, config=config, auto_start=False)
        config = clone(baseline.config)
        config.lag_ff_s = .09
        candidate = ClosedLoop(self.library, config=config, auto_start=False)
        self.assertEqual(candidate.tracker.output()['state'], 0)

    @classmethod
    def setUpClass(cls):
        cls.library = ROOT / 'output/tests' / ('tracking.dll' if os.name == 'nt' else 'tracking.so')
        subprocess.run([sys.executable, '-B', str(ROOT / 'tools/build_odom.py'),
                        '--output', str(cls.library)], check=True, capture_output=True)

    def model(self, kind='diagonal', **options):
        return ClosedLoop(self.library, kind, **options)

    def settling(self):
        model = self.model(disturbance=False)
        while model.elapsed_ms < 15000:
            report = model.advance()
            if report['output']['stable_elapsed_ms'] >= 10:
                self.assertEqual(report['state'], 'SETTLING')
                return model
        self.fail('未进入稳定窗口')

    def fresh_stationary(self, model, sample, delta=10):
        """注入独立的新反馈，用于验证单个到位门槛；不改真实设备。"""
        sample = clone(sample)
        now = (model.base_ms + model.elapsed_ms + 6 + delta) & MASK
        sample.pose.tick_ms = now
        sample.pose.steps += 1
        for stamp in [*sample.position, *sample.speed, sample.yaw_stamp, sample.gyro_stamp]:
            stamp.first_ms = stamp.last_ms = stamp.handled_ms = now
            stamp.sequence += 100
        model.elapsed_ms += delta
        return sample, now

    def test_closed_loop_directions_turns_and_paths(self):
        summary = []
        for kind in ('line', 'back', 'side', 'diagonal', 'turn', 'right_turn', 'multi_turn',
                     'arc', 'right_arc', 'reverse_arc', 'fixed_arc'):
            with self.subTest(kind=kind):
                model = self.model(kind)
                report = model.run()
                out = report['output']
                self.assertEqual(report['state'], 'COMPLETE')
                self.assertEqual(out['fault'], 0)
                self.assertEqual(out['rpm'], [0] * 4)
                self.assertLess(report['reference_end_ms'], report['elapsed_ms'])
                self.assertLessEqual(out['position_error_m'], model.config.position_tol_m)
                self.assertLessEqual(out['yaw_error_rad'], model.config.yaw_tol_rad)
                self.assertGreaterEqual(out['stable_elapsed_ms'], model.config.stable_ms)
                self.assertIn((out['id'], out['zero_id']), model.acknowledged)
                self.assertFalse(report['physical_pose_valid'])
                self.assertLessEqual(report['peak_rpm'], model.config.max_rpm)
                # 真值来自独立电机/刚体模型，不能只校验观察器自己的输出。
                self.assertLess(math.hypot(report['plant']['x_m'] - out['reference']['x_m'],
                                          report['plant']['y_m'] - out['reference']['y_m']), .007)
                self.assertLess(abs(report['plant']['yaw_rad'] - out['reference']['yaw_rad']), .03)
                summary.append({key: report[key] for key in ('kind', 'state', 'elapsed_ms', 'reference_end_ms')}
                                    | dict(position_error_m=out['position_error_m'], yaw_error_rad=out['yaw_error_rad']))
        (ROOT / 'output/tests/tracking_summary.json').write_text(json.dumps(summary, indent=2), encoding='utf-8')

    def test_clock_wrap_preserves_closed_loop(self):
        first = self.model('arc').run()
        wrapped = self.model('arc', base_ms=0xfffff000).run()
        self.assertEqual(wrapped['state'], 'COMPLETE')
        self.assertEqual(first['elapsed_ms'], wrapped['elapsed_ms'])
        self.assertAlmostEqual(first['pose']['x_m'], wrapped['pose']['x_m'], places=8)

    def test_reference_end_without_ack_times_out(self):
        model = self.model(ack=False)
        report = model.run()
        self.assertIsNotNone(report['reference_end_ms'])
        self.assertEqual(report['state'], 'FAILED')
        self.assertEqual(report['output']['fault'], 7)
        self.assertEqual(report['output']['done_ms'], 0)
        self.assertEqual(report['output']['rpm'], [0] * 4)

    def test_stalled_chassis_is_not_arrival(self):
        report = self.model(stalled=True, disturbance=False).run()
        self.assertEqual(report['state'], 'FAILED')
        self.assertEqual(report['output']['fault'], 7)
        self.assertGreater(report['output']['position_error_m'], .4)

    def test_missing_speed_or_gyro_fails_with_zero_outputs(self):
        for kind in (1, 5):
            model = self.model()
            while model.elapsed_ms < 800:
                model.advance()
            model.drop.add(kind)
            report = model.run()
            self.assertEqual(report['state'], 'FAILED')
            self.assertEqual(report['output']['fault'], 2)
            self.assertEqual(report['output']['rpm'], [0] * 4)

    def test_cached_observation_cannot_earn_stable_time(self):
        model = self.settling()
        sample = clone(model.observation)
        before = model.tracker.output()['stable_elapsed_ms']
        now = model.elapsed_ms + 6
        for delta in range(10, 171, 10):
            model.tracker.step(sample, now + delta)
            self.assertEqual(model.tracker.output()['stable_elapsed_ms'], before)
            self.assertEqual(model.tracker.output()['state'], 3)

    def test_encoder_drift_restarts_stable_window(self):
        model = self.settling()
        sample, now = self.fresh_stationary(model, model.observation)
        sample.counts[0] += 181
        model.tracker.step(sample, now)
        out = model.tracker.output()
        self.assertEqual(out['state'], 3)
        self.assertTrue(out['blockers'] & 32)
        self.assertEqual(out['stable_elapsed_ms'], 0)

    def test_gyro_motion_blocks_arrival_even_at_zero_wheel_speed(self):
        model = self.settling()
        sample, now = self.fresh_stationary(model, model.observation)
        sample.gyro_rad_s = .03
        model.tracker.step(sample, now)
        out = model.tracker.output()
        self.assertEqual(out['state'], 3)
        self.assertTrue(out['blockers'] & 16)
        self.assertEqual(out['stable_elapsed_ms'], 0)

    def test_source_change_and_control_gap_fail(self):
        for failure in ('source', 'gap', 'conflict'):
            model = self.settling()
            sample = clone(model.observation)
            now = model.elapsed_ms + 16
            if failure == 'source':
                sample.imu_source_epoch += 1
            elif failure == 'gap':
                now += 100
            else:
                sample.counts[0] += 1  # 同一序号更改值必须失败。
            model.tracker.step(sample, now)
            out = model.tracker.output()
            self.assertEqual(out['state'], 5)
            self.assertEqual(out['fault'], 3 if failure == 'source' else 4)
            self.assertEqual(out['rpm'], [0] * 4)

    def test_duplicate_conflict_busy_and_cancel(self):
        model = self.model()
        while not model.started:
            model.advance()
        now = model.elapsed_ms + 6
        self.assertEqual(model.tracker.start(model.goal, model.observation, now), 0)
        different = clone(model.goal)
        different.reference.x_m += .1
        self.assertEqual(model.tracker.start(different, model.observation, now), -2)
        for field in ('path_heading_rad','curve_velocity_m_s','yaw_begin_m','yaw_end_m','tail_velocity_m_s'):
            different = clone(model.goal)
            setattr(different.reference,field,.01)
            self.assertEqual(model.tracker.start(different,model.observation,now),-2)
        self.assertEqual(model.tracker.start(goal('line', 2), model.observation, now), -3)
        previous_output = model.tracker.output()['output_id']
        model.tracker.cancel()
        model.advance()
        self.assertEqual(model.tracker.output()['state'], 6)
        self.assertEqual(model.tracker.output()['rpm'], [0] * 4)
        self.assertFalse(model.pending)
        self.assertEqual(model.tracker.output()['done_ms'], 0)
        self.assertGreater(model.tracker.output()['output_id'], previous_output)
        self.assertEqual(model.tracker.start(goal('line', 2), model.observation, now + 10), -1)

    def test_pose_reference_without_zero_ack_cannot_complete(self):
        model = self.model(ack=False,auto_start=False)
        while model.elapsed_ms < 120:
            model.advance()
        target = goal('diagonal')
        target.reference.kind = 5
        target.reference.yaw_rad = .4
        target.reference.angular.acceleration = 2
        target.reference.angular.jerk = 12
        model.begin(target)
        report = model.run()
        self.assertEqual(report['state'],'FAILED')
        self.assertEqual(report['output']['fault'],7)
        self.assertEqual(report['output']['done_ms'],0)

    def test_wrong_or_pre_request_ack_rejected(self):
        model = self.model(ack=False)
        while model.tracker.output()['state'] != 3:
            model.advance()
        out = model.tracker.output()
        now = model.elapsed_ms + 6
        self.assertEqual(model.tracker.sent(2, out['zero_id'], now), 0)
        self.assertEqual(model.tracker.sent(1, out['zero_id'] + 1, now), 0)
        self.assertEqual(model.tracker.sent(1, out['zero_id'], now - 1), 0)
        self.assertEqual(model.tracker.output()['zero_ack_ms'], 0)

    def test_successive_goal_keeps_continuous_odometry(self):
        model = self.model('line', disturbance=False, ideal=True)
        first = model.run()
        origin = first['pose']['origin_ms']
        second = goal('side', 2)
        self.assertEqual(model.tracker.start(second, model.observation, model.elapsed_ms + 6), 1)
        model.goal = second
        result = model.run(limit_ms=15000)
        self.assertEqual(result['state'], 'COMPLETE')
        self.assertEqual(result['output']['id'], 2)
        self.assertEqual(result['pose']['origin_ms'], origin)
        self.assertGreater(result['pose']['x_m'], .39)
        self.assertLess(result['pose']['y_m'], -.29)

    def test_real_modes_reject_model_before_opening_transport(self):
        with patch('robot.main.open_transport') as opened:
            for mode in ('offline', 'observer', 'serial'):
                with self.assertRaises(ValueError):
                    run(mode=mode, track_library=self.library)
            opened.assert_not_called()

    def test_overspeed_reference_fails_instead_of_silently_shrinking(self):
        model = self.model()
        # 配置是上下文初始化时冻结的；重新构造后设置较低轮速上限。
        config = clone(model.config)
        config.max_rpm = 5
        model = self.model(config=config)
        report = model.run()
        self.assertEqual(report['state'], 'FAILED')
        self.assertEqual(report['output']['fault'], 6)
        self.assertEqual(report['output']['rpm'], [0] * 4)

    def test_late_pre_barrier_feedback_cannot_start_stability(self):
        model = self.model(ack=False, disturbance=False)
        while model.tracker.output()['state'] != 3:
            model.advance()
        now = model.elapsed_ms + 6
        out = model.tracker.output()
        self.assertEqual(model.tracker.sent(1, out['zero_id'], now), 1)
        sample = clone(model.observation)
        for delta in range(10, 101, 10):
            model.tracker.step(sample, now + delta)
            out = model.tracker.output()
            self.assertEqual(out['state'], 3)
            self.assertTrue(out['blockers'] & 8)
            self.assertEqual(out['stable_elapsed_ms'], 0)

    def test_invalid_next_goal_keeps_last_completed_result(self):
        model = self.model()
        model.run()
        previous = model.tracker.output()
        bad = goal('arc', 2)
        bad.reference.path.radius_m = -1
        self.assertEqual(model.tracker.start(bad, model.observation, model.elapsed_ms + 6), -1)
        self.assertEqual(model.tracker.output(), previous)

    def test_one_core_publishes_tracking_and_task_without_physical_values(self):
        from robot.runtime import load_snapshot
        with tempfile.TemporaryDirectory() as directory:
            command = [sys.executable, '-B', '-m', 'robot.main', '--mode', 'sim',
                       '--track-library', str(self.library), '--task-demo', '136+321+631+312',
                       '--runtime', directory]
            core = subprocess.Popen(command, cwd=ROOT / 'pi', stdout=subprocess.PIPE, stderr=subprocess.PIPE)
            try:
                deadline = time.monotonic() + 5
                while time.monotonic() < deadline:
                    state = load_snapshot(Path(directory) / 'state.json')
                    if state.get('tracking', {}).get('state') == 'RUNNING' and state.get('task'):
                        break
                    time.sleep(.05)
                self.assertEqual(state['tracking']['state'], 'RUNNING')
                self.assertEqual(state['tracking']['domain'], 'simulation')
                self.assertEqual(state['task']['domain'], 'simulation')
                self.assertFalse(state['pose_valid'])
                self.assertIsNone(state['pose'])
                self.assertIsNone(state['grabs'])
                self.assertIsNone(state['places'])
            finally:
                core.terminate()
                core.communicate(timeout=5)

    def test_readonly_web_http_ws_and_expiry(self):
        report = self.model().run()
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / 'state.json'
            snapshot = dict(mode='sim', readonly=True, connected=False, tracking=report,
                            snapshot_at_s=time.monotonic(), snapshot_wall_s=time.time())
            save_snapshot(path, snapshot)
            with TestClient(create_app(directory)) as client:
                self.assertEqual(client.get('/api/state').json()['tracking']['state'], 'COMPLETE')
                with client.websocket_connect('/ws') as socket:
                    self.assertEqual(socket.receive_json()['tracking']['output']['stable_elapsed_ms'], 150)
                self.assertIn('track-trace', client.get('/diagnostics').text)
                self.assertEqual(client.get('/static/tracking.js').status_code, 200)
                self.assertEqual(client.post('/api/move', json={}).status_code, 404)
                snapshot['snapshot_at_s'] -= 1
                save_snapshot(path, snapshot)
                expired = client.get('/api/state').json()
                self.assertFalse(expired['core_fresh'])
                self.assertNotIn('tracking', expired)


if __name__ == '__main__':
    unittest.main()
