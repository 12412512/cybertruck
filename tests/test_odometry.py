"""共同时间区间、独立解析真值和端到端仿真；不访问硬件。"""

from dataclasses import replace
import math
import os
from pathlib import Path
import subprocess
import sys
import tempfile
import unittest
from unittest.mock import patch

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'pi'))
from robot.odom_sim import Model, fixture
from robot.device.odometry import Odometry
from robot.main import run


class OdometryTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.library = ROOT / 'output/tests' / ('odometry.dll' if os.name == 'nt' else 'odometry.so')
        subprocess.run([sys.executable, '-B', str(ROOT / 'tools/build_odom.py'),
                        '--output', str(cls.library)], check=True, capture_output=True)

    def replay(self, body, duration=1200, base=0, config=None):
        model = Model(self.library, body, base, duration, config)
        for tick in range(0, duration + 101, 10):
            result = model.advance(tick)
            self.assertEqual(result['pose']['fault'], 0)
        self.assertTrue(result['pose']['valid'])
        self.assertLess(result['error_m'], .00025)
        self.assertAlmostEqual(result['pose']['yaw_rad'], result['truth']['yaw_rad'], delta=.0002)
        self.assertFalse(result['physical_pose_valid'])
        return model

    def test_forward_back_lateral_and_diagonal(self):
        for body in ((.2, 0, 0), (-.2, 0, 0), (0, .2, 0), (0, -.2, 0), (.15, .1, 0)):
            with self.subTest(body=body):
                self.replay(body)

    def test_async_pure_rotation_has_no_false_translation(self):
        for rate in (-2, 2):
            model = self.replay((0, 0, rate), duration=8000)
            pose = model.latest
            self.assertLess(math.hypot(pose['x_m'], pose['y_m']), .0001)
            self.assertGreater(abs(pose['yaw_rad']), 4*math.pi)

    def test_simultaneous_translation_rotation_against_analytic_arc(self):
        for body in ((.15, .03, .25), (.1, -.05, -.4), (-.12, .05, .3)):
            self.replay(body, duration=4000)

    def test_uint32_clock_wrap_uses_same_trajectory(self):
        normal = self.replay((.15, .03, .25))
        wrapped = self.replay((.15, .03, .25), base=0xffffff00)
        for key in ('x_m', 'y_m', 'yaw_rad'):
            self.assertAlmostEqual(normal.latest[key], wrapped.latest[key], places=8)

    def test_independent_addresses_polarities_and_encoder_scale(self):
        config = fixture()
        config.address[:] = [21, 24, 22, 23]
        config.sign[:] = [-1, 1, -1, 1]
        config.counts_per_rev[:] = [40000, 60000, 80000, 100000]
        self.replay((.15, .03, .25), config=config)

    def test_duplicate_does_not_reintegrate_and_conflict_latches(self):
        model = self.replay((.2, 0, 0))
        now = model.elapsed_ms + 6
        before = dict(model.latest)
        sample = model.cache.samples[(2, 11)]
        self.assertEqual(model.native.push(sample, now), 0)
        self.assertEqual(model.native.step(now), 0)
        after = model.native.snapshot(now)
        self.assertEqual(before['steps'], after['steps'])
        self.assertEqual(before['x_m'], after['x_m'])
        self.assertEqual(model.native.push(replace(sample, values=(sample.values[0]+1, 0, 0)), now), -1)
        self.assertEqual(model.native.snapshot(now)['fault'], 2)

    def test_epoch_loss_and_stale_feedback_freeze_last_pose(self):
        for case in ('can_epoch', 'imu_epoch', 'stale'):
            model = self.replay((.15, .03, .25))
            before = dict(model.latest)
            now = model.elapsed_ms + 6
            if case == 'stale':
                self.assertEqual(model.native.step(now + 251), -1)
            else:
                sample = model.cache.samples[(2, 11) if case == 'can_epoch' else (4, 0)]
                self.assertEqual(model.native.push(replace(sample, epoch=sample.epoch+1), now), -1)
            pose = model.native.snapshot(now)
            self.assertFalse(pose['valid'])
            self.assertNotEqual(pose['fault'], 0)
            self.assertEqual((pose['x_m'], pose['y_m']), (before['x_m'], before['y_m']))

    def test_encoder_jump_and_reverse_time_do_not_update_pose(self):
        for change in ('counts', 'time'):
            model = self.replay((.15, 0, 0))
            before = dict(model.latest)
            sample = model.cache.samples[(2, 11)]
            if change == 'counts':
                sample = replace(sample, sequence=sample.sequence+100,
                                 first_ms=sample.last_ms+40, last_ms=sample.last_ms+40,
                                 handled_ms=sample.last_ms+42, values=(sample.values[0]+1000000, 0, 0))
                now = sample.last_ms + 6
            else:
                sample = replace(sample, sequence=sample.sequence+100,
                                 first_ms=sample.last_ms-1, last_ms=sample.last_ms-1,
                                 handled_ms=sample.last_ms+2)
                now = model.elapsed_ms + 6
            self.assertEqual(model.native.push(sample, now), -1)
            pose = model.native.snapshot(now)
            self.assertEqual((pose['x_m'], pose['y_m']), (before['x_m'], before['y_m']))

    def test_lost_history_cannot_silently_rebase(self):
        model = self.replay((.15, .03, .25), duration=500)
        before = dict(model.latest)
        for tick in range(model.elapsed_ms + 10, model.elapsed_ms + 1201, 10):
            model.advance(tick, commit=False)
        self.assertEqual(model.native.step(model.elapsed_ms + 6), -1)
        pose = model.native.snapshot(model.elapsed_ms + 6)
        self.assertEqual(pose['fault'], 6)
        self.assertEqual((pose['x_m'], pose['y_m']), (before['x_m'], before['y_m']))

    def test_common_watermark_waits_without_extrapolation(self):
        model = Model(self.library)
        for tick in range(0, 91, 10):
            model.advance(tick)
        before = dict(model.latest)
        # 只更新航向，轮位置的共同覆盖末端不变。
        yaw = round(model.body[2] * .1 / model.config.yaw_scale_rad)
        model.deliver(4, 0, (0, 0, yaw), 100, 5, 7)
        self.assertEqual(model.native.step(106), 0)
        pose = model.native.snapshot(106)
        self.assertEqual(pose['tick_ms'], before['tick_ms'])
        self.assertEqual(pose['x_m'], before['x_m'])

    def test_invalid_and_unknown_calibration_configuration(self):
        for field, value in (('calibration', 0), ('can_epoch', 0), ('gap_ms', 251),
                             ('yaw_sign', 0), ('max_rpm', float('nan'))):
            config = fixture()
            setattr(config, field, value)
            with self.assertRaises(ValueError):
                Odometry(self.library, config)
        config = fixture()
        config.address[1] = config.address[0]
        with self.assertRaises(ValueError):
            Odometry(self.library, config)

    def test_trace_retains_motion_after_long_stationary_reporting(self):
        model = Model(self.library, duration_ms=1000)
        for tick in range(0, 5001, 10):
            model.advance(tick)
        history = model.snapshot()['history']
        self.assertGreater(len(history), 2)
        self.assertLessEqual(len(history), 300)
        self.assertEqual((history[0]['x_m'], history[0]['y_m']), (0, 0))
        self.assertAlmostEqual(history[-1]['x_m'], model.latest['x_m'], places=8)
        self.assertGreater(math.hypot(history[-1]['x_m'], history[-1]['y_m']), .1)

    def test_physical_modes_reject_model_before_any_device_access(self):
        for mode in ('offline', 'serial', 'observer'):
            with patch('robot.main.open_transport', side_effect=AssertionError('不得打开设备')):
                with self.assertRaises(ValueError):
                    run(mode=mode, odom_library=self.library)

    def test_one_core_can_publish_task_and_odometry_separately(self):
        from robot.runtime import load_snapshot
        import json
        import time
        with tempfile.TemporaryDirectory() as directory:
            runtime = Path(directory)
            command = [sys.executable, '-B', '-m', 'robot.main', '--mode', 'sim',
                       '--odom-library', str(self.library), '--task-demo', '136+321+631+312',
                       '--runtime', str(runtime)]
            core = subprocess.Popen(command, cwd=ROOT / 'pi', stdout=subprocess.PIPE, stderr=subprocess.PIPE)
            try:
                deadline = time.monotonic() + 5
                while time.monotonic() < deadline:
                    state = load_snapshot(runtime / 'state.json')
                    if state.get('odometry', {}).get('pose', {}).get('valid') and state.get('task'):
                        break
                    time.sleep(.05)
                self.assertEqual(state['mode'], 'sim')
                self.assertEqual(state['odometry']['domain'], 'simulation')
                self.assertEqual(state['task']['domain'], 'simulation')
                self.assertFalse(state['pose_valid'])
                self.assertIsNone(state['pose'])
                self.assertIsNone(state['grabs'])
            finally:
                core.terminate()
                core.communicate(timeout=5)
