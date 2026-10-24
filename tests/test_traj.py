"""正式C轨迹计算回归：独立几何积分、限值与边界连续性；不连接设备。"""

import ctypes as c
import math
import os
from pathlib import Path
import random
import subprocess
import sys
import unittest

ROOT = Path(__file__).resolve().parents[1]


class TrajectoryTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        output = ROOT / 'output/tests'
        output.mkdir(parents=True, exist_ok=True)
        target = output / ('traj.dll' if os.name == 'nt' else 'traj.so')
        source = ROOT / 'firmware/User/Control'
        command = [sys.executable, '-m', 'ziglang', 'cc', '-shared', '-std=c99', '-O2',
                   '-Wall', '-Wextra', '-Werror', '-I', str(source),
                   *(str(source / name) for name in ('curve.c', 'path.c', 'line.c')),
                   str(ROOT / 'tests/traj_native.c'), '-o', str(target)]
        if os.name != 'nt':
            command.extend(['-fPIC', '-lm'])
        subprocess.run(command, check=True, capture_output=True)
        cls.lib = c.CDLL(str(target))
        cls.lib.Test_Curve.argtypes = [c.c_float] * 4
        cls.lib.Test_Path.argtypes = [c.c_float] * 4 + [c.c_int] * 2
        cls.lib.Test_Line.argtypes = [c.c_float] * 2
        for name in ('Test_Time', 'Test_LineTime'):
            getattr(cls.lib, name).restype = c.c_double
        cls.lib.Test_Length.restype = c.c_float
        cls.lib.Test_Join.argtypes = [c.c_uint]
        cls.lib.Test_Join.restype = c.c_float
        cls.lib.Test_Boundary.argtypes = [c.c_uint]
        cls.lib.Test_Boundary.restype = c.c_double
        for name in ('Test_Scalar', 'Test_PathAt', 'Test_LineAt'):
            getattr(cls.lib, name).argtypes = [c.c_double, c.POINTER(c.c_float)]
        cls.lib.Test_Reached.argtypes = [c.c_uint, c.c_float, c.c_float, c.POINTER(c.c_float)]
        cls.lib.Test_Corrupt.restype = None
        cls.lib.Test_Timing.argtypes = [c.c_float] * 6
        cls.lib.Test_TimingTime.restype = c.c_double
        cls.lib.Test_TimingCount.restype = c.c_uint
        cls.lib.Test_TimingBoundary.argtypes = [c.c_uint]
        cls.lib.Test_TimingBoundary.restype = c.c_double
        cls.lib.Test_TimingAt.argtypes = [c.c_double, c.POINTER(c.c_float)]

    def at(self, name, time):
        output = (c.c_float * 14)()
        self.assertEqual(getattr(self.lib, name)(time, output), 1)
        self.assertTrue(all(math.isfinite(value) for value in output))
        return list(output)

    def test_curve_random_limits_endpoint_monotonicity_and_symmetry(self):
        rng = random.Random(241024)
        cases = [(0, .2, .5, 2), (.001, .3, .5, 2), (2, .3, .5, 2), (-2, .3, .5, 2)]
        cases += [(rng.choice((-1, 1)) * 10**rng.uniform(-4, 1),
                   10**rng.uniform(-2, 0), 10**rng.uniform(-2, 1),
                   10**rng.uniform(-1, 2)) for _ in range(50)]
        for distance, velocity, accel, jerk in cases:
            self.assertEqual(self.lib.Test_Curve(distance, velocity, accel, jerk), 1)
            total = self.lib.Test_Time()
            end = self.at('Test_Scalar', total + 1)
            self.assertAlmostEqual(end[0], distance, delta=abs(distance) * 1e-6 + 1e-8)
            self.assertEqual(end[1:5], [0, 0, 0, 1])
            previous = 0
            for index in range(401):
                point = self.at('Test_Scalar', total * index / 400)
                mirror = self.at('Test_Scalar', total * (400 - index) / 400)
                self.assertGreaterEqual(abs(point[0]) + 1e-6, previous)
                previous = abs(point[0])
                self.assertLessEqual(abs(point[1]), velocity * 1.00001)
                self.assertLessEqual(abs(point[2]), accel * 1.00001)
                self.assertLessEqual(abs(point[3]), jerk * 1.00001)
                self.assertAlmostEqual(point[0] + mirror[0], distance, delta=1e-5)
                self.assertAlmostEqual(point[1], mirror[1], delta=1e-5)

    def test_variable_speed_timing_limits_continuity_and_terminal_cap(self):
        for lead, tail, end in ((.8, .7, .14), (.3, .8, .025), (.07, .07, .14)):
            self.assertEqual(self.lib.Test_Timing(2, lead, tail, .04, .14, end), 1)
            total = self.lib.Test_TimingTime()
            count = self.lib.Test_TimingCount()
            self.assertLessEqual(count, 15)
            previous = 0
            for i in range(1001):
                p = self.at('Test_TimingAt', total*i/1000)
                self.assertGreaterEqual(p[0]+1e-6, previous)
                previous = p[0]
                self.assertLessEqual(abs(p[1]), .14001)
                self.assertLessEqual(abs(p[2]), .25001)
                self.assertLessEqual(abs(p[3]), .80001)
                if 2-tail+.001 < p[0] < 1.999:
                    self.assertLessEqual(p[1], max(end,.04)+1e-6)
            for i in range(1, count):
                at = self.lib.Test_TimingBoundary(i)
                before, after = [self.at('Test_TimingAt', at+d) for d in (-1e-7, 1e-7)]
                for field in range(3):
                    self.assertAlmostEqual(before[field], after[field], delta=2e-6)
                self.assertGreater(before[1], 0)
            self.assertEqual(self.at('Test_TimingAt', total)[1:5], [0,0,0,1])

    def test_variable_speed_timing_rejects_invalid_geometry_and_nonfinite(self):
        for values in ((2,0,.5,.04,.14,.14), (2,1,1,.04,.14,.14),
                       (2,.5,.5,.2,.14,.14), (2,.5,.5,.04,.14,0),
                       (float('nan'),.5,.5,.04,.14,.14)):
            self.assertEqual(self.lib.Test_Timing(*values), 0)

    def test_curve_phase_boundaries_continuous_and_invalid_inputs(self):
        self.assertEqual(self.lib.Test_Curve(2, .3, .5, 2), 1)
        for index in range(1, 7):
            boundary = self.lib.Test_Boundary(index)
            before = self.at('Test_Scalar', boundary - 1e-7)
            after = self.at('Test_Scalar', boundary + 1e-7)
            for field in range(3):
                self.assertAlmostEqual(before[field], after[field], delta=2e-6)
        output = (c.c_float * 14)()
        for time in (-1, float('nan'), float('inf')):
            self.assertEqual(self.lib.Test_Scalar(time, output), 0)
        for args in ((float('nan'), .3, .5, 2), (1, 0, .5, 2), (1, .3, -1, 2),
                     (1, .3, .5, float('inf')), (1001, .3, .5, 2)):
            self.assertEqual(self.lib.Test_Curve(*args), 0)

    def test_xy_diagonal_shared_time_and_exact_endpoints(self):
        for x, y in ((1, 0), (0, -1), (1, 1), (-.3, .7), (0, 0)):
            self.assertEqual(self.lib.Test_Line(x, y), 1)
            total = self.lib.Test_LineTime()
            for index in range(101):
                point = self.at('Test_LineAt', total * index / 100)
                for a, b in ((point[0], point[1]), (point[2], point[3]), (point[4], point[5])):
                    self.assertAlmostEqual(a * y - b * x, 0, delta=1e-7)
                self.assertLessEqual(math.hypot(point[2], point[3]), .300001)
            end = self.at('Test_LineAt', total + 1)
            self.assertAlmostEqual(end[0], x, places=6)
            self.assertAlmostEqual(end[1], y, places=6)
            self.assertEqual(end[2:7], [0, 0, 0, 0, 1])

    @staticmethod
    def heading(s, radius, blend):
        arc = math.pi * radius / 2 - blend
        length = arc + 2 * blend
        if s <= 0:
            return 0
        if s < blend:
            return (s - blend * math.sin(math.pi * s / blend) / math.pi) / (2 * radius)
        if s < blend + arc:
            return blend / (2 * radius) + (s - blend) / radius
        if s < length:
            return math.pi / 2 - TrajectoryTests.heading(length - s, radius, blend)
        return math.pi / 2

    def test_path_against_independent_dense_midpoint_integral(self):
        rng = random.Random(42)
        for _ in range(12):
            radius = c.c_float(rng.uniform(.08, .5)).value
            blend = c.c_float(radius * rng.uniform(.1, 1)).value
            self.assertEqual(self.lib.Test_Path(.2, .3, radius, blend, 1, 0), 1)
            length = math.pi * radius / 2 + blend
            steps = 5000
            dx = dy = 0
            for index in range(steps):
                yaw = self.heading((index + .5) * length / steps, radius, blend)
                dx += math.cos(yaw) * length / steps
                dy += math.sin(yaw) * length / steps
                if (index + 1) % 500 == 0:
                    point = self.at('Test_PathAt', .2 + (index + 1) * length / steps)
                    self.assertAlmostEqual(point[0], .2 + dx, delta=4e-7)
                    self.assertAlmostEqual(point[1], dy, delta=4e-7)
            end = self.at('Test_PathAt', self.lib.Test_Length() + 1)
            self.assertAlmostEqual(end[0], .2 + dx, delta=4e-7)
            self.assertAlmostEqual(end[1], .3 + dy, delta=4e-7)

    def test_turn_reverse_signs_and_boundary_continuity(self):
        for turn in (-1, 1):
            for reverse in (0, 1):
                self.assertEqual(self.lib.Test_Path(.2, .3, .2, .1, turn, reverse), 1)
                end = self.at('Test_PathAt', self.lib.Test_Length() + 1)
                self.assertEqual(1 if end[0] > 0 else -1, -1 if reverse else 1)
                self.assertEqual(1 if end[1] > 0 else -1, turn * (-1 if reverse else 1))
                self.assertAlmostEqual(end[8], turn * math.pi / 2, places=6)
                for index in range(4):
                    boundary = self.lib.Test_Join(index)
                    before = self.at('Test_PathAt', boundary - 1e-7)
                    after = self.at('Test_PathAt', boundary + 1e-7)
                    for field, tolerance in ((0, 1e-6), (1, 1e-6), (2, 2e-6), (3, 2e-6),
                                             (4, 2e-4), (5, 2e-4), (6, .02), (7, .02),
                                             (8, 2e-6), (9, 2e-4), (10, .02)):
                        self.assertAlmostEqual(before[field], after[field], delta=tolerance)

    def test_cached_derivatives_match_finite_differences(self):
        self.assertEqual(self.lib.Test_Path(.2, .3, .2, .1, 1, 0), 1)
        for progress in (.231, .265, .345, .425, .575):
            h = .00002
            before = self.at('Test_PathAt', progress - h)
            point = self.at('Test_PathAt', progress)
            after = self.at('Test_PathAt', progress + h)
            self.assertAlmostEqual((after[0] - before[0]) / (2*h), point[2], delta=.002)
            self.assertAlmostEqual((after[1] - before[1]) / (2*h), point[3], delta=.002)
            self.assertAlmostEqual((after[8] - before[8]) / (2*h), point[9], delta=.006)
            self.assertAlmostEqual(math.hypot(point[2], point[3]), 1, delta=2e-6)

    def test_invalid_path_integrity_and_feedback_boundary(self):
        for spec in ((.2, .3, 0, .1, 1, 0), (.2, .3, .2, .4, 1, 0),
                     (.2, .3, .2, .1, 0, 0), (.2, .3, .2, .1, 1, 2),
                     (float('nan'), .3, .2, .1, 1, 0)):
            self.assertEqual(self.lib.Test_Path(*spec), 0)
            self.assertEqual(self.lib.Test_Valid(), 0)
        self.assertEqual(self.lib.Test_Path(.2, .3, .2, .1, 1, 0), 1)
        out = (c.c_float * 2)()
        self.assertEqual(self.lib.Test_Reached(0, .19, 0, out), 0)
        self.assertEqual(self.lib.Test_Reached(0, .21, .01, out), 1)
        self.assertAlmostEqual(out[1], .01, places=6)
        self.lib.Test_Corrupt()
        self.assertEqual(self.lib.Test_Valid(), 0)
