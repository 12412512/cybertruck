import subprocess
import tempfile
import unittest
from pathlib import Path

from test_mech_port import MechPortTests
from test_execution import ROOT


class HandoffTests(unittest.TestCase):
    def test_board_contract_and_hold_supervision(self):
        with tempfile.TemporaryDirectory(dir=ROOT/'output/tests') as folder:
            binary=Path(folder)/'handoff.exe'
            MechPortTests.build(binary,ROOT/'tests/mech_handoff.c')
            for case in range(12):
                with self.subTest(case=case):
                    result=subprocess.run([str(binary),str(case)],capture_output=True)
                    self.assertEqual(result.returncode,0,result.stderr.decode(errors='replace'))
