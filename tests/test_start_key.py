"""PC0准备/消抖/蜂鸣/一次启动；全部为主机夹具，无GPIO或设备调用。"""

import subprocess
import unittest
from test_execution import ROOT, compile_c


class StartTests(unittest.TestCase):
    def test_start_key_state_machine(self):
        output = ROOT / 'output/tests/start-key.exe'
        compile_c(output, [ROOT / 'tests/start_key.c', ROOT / 'firmware/User/Control/start.c'])
        subprocess.run([str(output)], check=True)
