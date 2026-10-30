import subprocess
import tempfile
import unittest
from pathlib import Path
from test_execution import ROOT,compile_c


class MicroClosureTests(unittest.TestCase):
    def test_real_timeout_residual_and_old_gain_negative_control(self):
        with tempfile.TemporaryDirectory(dir=ROOT/'output/tests') as directory:
            control=ROOT/'firmware/User/Control';comm=ROOT/'firmware/User/Comm'
            binary=Path(directory)/'micro-closure.exe'
            compile_c(binary,[control/n for n in ('exec.c','track.c','ref.c','odom.c','kin.c','yaw.c','curve.c','line.c','path.c','start.c')]+
                [ROOT/'firmware/User/Device/stamp.c',comm/'frame.c',comm/'comm.c',comm/'execute.c',comm/'feedback.c',ROOT/'tests/micro_closure.c'])
            result=subprocess.run([str(binary)],capture_output=True,text=True)
            self.assertEqual(result.returncode,0,result.stderr)
            self.assertEqual(result.stdout.count('PASS'),24)
            (ROOT/'output/tests/micro_closure.txt').write_text(result.stdout,encoding='utf-8')

    def test_transmitted_pulse_timing_and_fixed_heading(self):
        with tempfile.TemporaryDirectory(dir=ROOT/'output/tests') as directory:
            control=ROOT/'firmware/User/Control';comm=ROOT/'firmware/User/Comm'
            binary=Path(directory)/'micro-timing.exe'
            compile_c(binary,[control/n for n in ('exec.c','track.c','ref.c','odom.c','kin.c','yaw.c','curve.c','line.c','path.c','start.c')]+
                [ROOT/'firmware/User/Device/stamp.c',comm/'frame.c',comm/'comm.c',comm/'execute.c',comm/'feedback.c',ROOT/'tests/micro_timing.c'])
            result=subprocess.run([str(binary)],capture_output=True,text=True)
            self.assertEqual(result.returncode,0,result.stderr)
