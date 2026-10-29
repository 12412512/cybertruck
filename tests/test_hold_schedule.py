import subprocess
import tempfile
import unittest
from pathlib import Path
from test_base_startup import BaseStartupTests
from test_execution import ROOT


class HoldScheduleTests(unittest.TestCase):
    def test_board_pc0_micro_and_return_preserves_zero_and_pwm(self):
        with tempfile.TemporaryDirectory(dir=ROOT/'output/tests') as folder:
            binary=Path(folder)/'roundtrip.exe'
            BaseStartupTests.build(binary,ROOT/'tests/handoff_roundtrip.c',mechanism=True)
            for axis in (0,1):
                for mm in (-5,-1,1,5):
                    result=subprocess.run([str(binary),str(mm),str(axis)],capture_output=True)
                    self.assertEqual(result.returncode,0,str((axis,mm))+': '+result.stderr.decode(errors='replace'))

    def test_real_bus_keeps_axes_fresh_during_active_base(self):
        with tempfile.TemporaryDirectory(dir=ROOT/'output/tests') as folder:
            binary=Path(folder)/'hold-schedule.exe'
            BaseStartupTests.build(binary,ROOT/'tests/hold_schedule.c',mechanism=True)
            for case in (0,1,2,3):
                result=subprocess.run([str(binary),str(case)],capture_output=True)
                self.assertEqual(result.returncode,0,str(case)+': '+result.stderr.decode(errors='replace'))
