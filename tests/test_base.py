"""底盘计算离线验证。测试几何只为数值样本，不是本车标定配置。"""

import ctypes as c
import math
import os
from pathlib import Path
import random
import subprocess
import sys
import unittest

ROOT = Path(__file__).resolve().parents[1]


class BaseTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        output = ROOT / 'output/tests'
        output.mkdir(parents=True, exist_ok=True)
        target = output / ('base.dll' if os.name == 'nt' else 'base.so')
        command = [sys.executable, '-m', 'ziglang', 'cc', '-shared', '-std=c99', '-O2',
                   '-Wall', '-Wextra', '-Werror', '-I', str(ROOT / 'firmware/User/Control'),
                   str(ROOT / 'firmware/User/Control/kin.c'), str(ROOT / 'firmware/User/Control/yaw.c'),
                   str(ROOT / 'tests/base_native.c'), '-o', str(target)]
        if os.name != 'nt':
            command.extend(['-fPIC', '-lm'])
        subprocess.run(command, check=True, capture_output=True)
        cls.lib = c.CDLL(str(target))
        pointer = c.POINTER(c.c_float)
        for name in ('Test_Inverse', 'Test_Forward'):
            getattr(cls.lib, name).argtypes = [pointer, c.c_int, pointer, pointer]
        cls.lib.Test_Limit.argtypes = [pointer, c.c_float]
        cls.lib.Test_Limit.restype = c.c_float
        cls.lib.Test_YawInit.argtypes = [c.c_uint32, c.c_float, c.c_float, c.c_uint32]
        cls.lib.Test_YawPush.argtypes = [c.c_uint32, c.c_float]
        cls.lib.Test_YawAt.argtypes = [c.c_uint32, pointer]
        cls.lib.Test_YawFresh.argtypes = [c.c_uint32]

    def setUp(self):
        self.geometry = (c.c_float * 3)(.05, .1, .12)

    def inverse(self, body, layout=1, geometry=None):
        output = (c.c_float * 4)(99, 99, 99, 99)
        result = self.lib.Test_Inverse(geometry or self.geometry, layout, (c.c_float * 3)(*body), output)
        return result, list(output)

    def yaw_at(self, tick):
        value = c.c_float(99)
        return self.lib.Test_YawAt(tick, c.byref(value)), value.value

    def test_body_axes_and_ccw_signs(self):
        for body, signs in (((.1, 0, 0), (1, 1, 1, 1)),
                            ((-.1, 0, 0), (-1, -1, -1, -1)),
                            ((0, .1, 0), (-1, 1, 1, -1)),
                            ((0, -.1, 0), (1, -1, -1, 1)),
                            ((0, 0, .5), (-1, 1, -1, 1)),
                            ((0, 0, -.5), (1, -1, 1, -1))):
            result, wheels = self.inverse(body)
            self.assertEqual(result, 1)
            self.assertEqual(tuple(1 if value > 0 else -1 for value in wheels), signs)

    def test_500_random_forward_inverse_roundtrips(self):
        rng = random.Random(2410)
        for _ in range(500):
            body = [rng.uniform(-1, 1), rng.uniform(-1, 1), rng.uniform(-4, 4)]
            result, wheels = self.inverse(body)
            self.assertEqual(result, 1)
            output = (c.c_float * 3)()
            self.assertEqual(self.lib.Test_Forward(self.geometry, 1, (c.c_float * 4)(*wheels), output), 1)
            for wanted, actual in zip(body, output):
                self.assertAlmostEqual(wanted, actual, places=5)

    def test_uniform_limit_retains_all_wheel_ratios(self):
        rpm = (c.c_float * 4)(100, -50, 25, -10)
        factor = self.lib.Test_Limit(rpm, 20)
        self.assertAlmostEqual(factor, .2)
        for actual, expected in zip(rpm, (20, -10, 5, -2)):
            self.assertAlmostEqual(actual, expected)
        self.assertEqual(self.lib.Test_Limit(rpm, 30), 1)

    def test_unknown_geometry_nonfinite_and_invalid_limit(self):
        self.assertEqual(self.inverse((1, 0, 0), layout=0), (0, [0, 0, 0, 0]))
        for radius in (0, -1, float('nan'), float('inf')):
            self.assertEqual(self.inverse((1, 0, 0), geometry=(c.c_float * 3)(radius, .1, .1))[0], 0)
        self.assertEqual(self.inverse((float('nan'), 0, 0))[0], 0)
        rpm = (c.c_float * 4)(1, 2, 3, 4)
        self.assertEqual(self.lib.Test_Limit(rpm, 0), -1)
        self.assertEqual(list(rpm), [1, 2, 3, 4])

    def test_positive_and_negative_pi_crossings(self):
        for first, last, expected in ((179, -179, 181), (-179, 179, -181)):
            self.assertEqual(self.lib.Test_YawInit(0, math.radians(first), 20, 250), 1)
            self.assertEqual(self.lib.Test_YawPush(20, math.radians(last)), 1)
            result, value = self.yaw_at(20)
            self.assertEqual(result, 1)
            self.assertAlmostEqual(value, math.radians(expected), places=5)

    def test_history_interpolation_wait_and_tick_wrap(self):
        self.lib.Test_YawInit(0xFFFFFFF0, 0, 20, 250)
        self.assertEqual(self.lib.Test_YawPush(0x10, .32), 1)
        self.assertAlmostEqual(self.yaw_at(0)[1], .16, places=5)
        self.assertEqual(self.yaw_at(0x20), (0, 0))
        self.assertEqual(self.yaw_at(0xFFFFFFE0), (-1, 0))
        self.assertEqual(self.lib.Test_YawFresh(0x100), 1)
        self.assertEqual(self.lib.Test_YawFresh(0x110), 0)

    def test_gap_jump_duplicate_and_ambiguous_half_turn_latch_fault(self):
        for tick, angle, fault in ((251, 0, 4), (10, 1, 5), (0, 0, 3),
                                   (100, math.pi, 5), (20, float('nan'), 2)):
            self.lib.Test_YawInit(0, 0, 20, 250)
            self.assertEqual(self.lib.Test_YawPush(tick, angle), 0)
            self.assertEqual(self.lib.Test_YawFault(), fault)
            self.assertEqual(self.lib.Test_YawPush(20, .01), 0)
            self.assertEqual(self.yaw_at(0), (-1, 0))

    def test_continuous_revolutions_and_ring_eviction(self):
        self.lib.Test_YawInit(0, 0, 20, 250)
        for index in range(1, 201):
            raw = (index * .1 + math.pi) % (2 * math.pi) - math.pi
            self.assertEqual(self.lib.Test_YawPush(index * 10, raw), 1)
        result, angle = self.yaw_at(2000)
        self.assertEqual(result, 1)
        self.assertAlmostEqual(angle, 20, places=4)
        self.assertEqual(self.yaw_at(0), (-1, 0))


if __name__ == '__main__':
    unittest.main(verbosity=2)
