"""实际总线接收代码的跨毫秒回归；旧取时版本在同一夹具中必须失败。"""

from pathlib import Path
import subprocess
import unittest

from test_execution import ROOT, compile_c


class StampRaceTests(unittest.TestCase):
    def test_bus_cases_and_cross_millisecond_receive(self):
        device = ROOT/'firmware/User/Device'
        output = ROOT/'output/tests/bus-race.exe'
        compile_c(output, [device/'bus.c', device/'drive.c', ROOT/'tests/reused_motor_bus.c'],
                  extra=['-DCAR_CHASSIS_HOST_TEST'])
        for case in range(39):
            result = subprocess.run([str(output), str(case)], capture_output=True)
            self.assertEqual(result.returncode, 0, (case, result.stderr))

    def test_same_fixture_reproduces_old_receive_order_failure(self):
        device = ROOT/'firmware/User/Device'
        source = (device/'bus.c').read_text(encoding='utf-8')
        old = source.replace('static void Receive(void)\n{',
                             'static void Receive(void)\n{\n    uint32_t now = HAL_GetTick();')
        old = old.replace('reply.handled_ms = HAL_GetTick();', 'reply.handled_ms = now;')
        path = ROOT/'output/tests/bus-before-race.c'
        path.write_text(old, encoding='utf-8')
        output = ROOT/'output/tests/bus-before-race.exe'
        compile_c(output, [path, device/'drive.c', ROOT/'tests/reused_motor_bus.c'],
                  extra=['-DCAR_CHASSIS_HOST_TEST'])
        result = subprocess.run([str(output), '35'], capture_output=True)
        self.assertNotEqual(result.returncode, 0)
