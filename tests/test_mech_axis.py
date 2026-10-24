"""正式单轴意图执行器的反馈闭环；全部运行于主机，不连接设备。"""
from pathlib import Path
import subprocess
import tempfile
import unittest
from test_execution import ROOT, compile_c
from fastapi.testclient import TestClient
from robot.web.app import create_app


class MechAxisTests(unittest.TestCase):
    def test_reused_axis_execution_and_fault_cases(self):
        with tempfile.TemporaryDirectory(dir=ROOT / 'output/tests') as folder:
            binary = Path(folder) / 'mech-axis.exe'
            control = ROOT / 'firmware/User/Control'
            compile_c(binary, [ROOT / 'tests/mech_axis.c', control / 'mech_axis.c',
                               control / 'mech_guard.c', ROOT / 'firmware/User/Device/drive.c'])
            lines = []
            for case in range(29):
                with self.subTest(case=case):
                    result = subprocess.run([str(binary), str(case)], capture_output=True)
                    self.assertEqual(result.returncode, 0, result.stderr.decode('utf-8', errors='replace'))
                    lines.append(result.stdout.decode())
            (ROOT / 'output/tests/mech_axis.log').write_text(''.join(lines), encoding='utf-8')

    def test_executor_has_no_hardware_binding(self):
        source = (ROOT / 'firmware/User/Control/mech_axis.c').read_text(encoding='utf-8')
        for call in ('HAL_', 'Bus_Send(', 'Can_Send(', 'PWM_Start'):
            self.assertNotIn(call, source)
        for name in ('App/app.c', 'Board/base_port.c', 'Comm/execute.c'):
            source = (ROOT / 'firmware/User' / name).read_text(encoding='utf-8')
            for call in ('Axis_Init(', 'Axis_Take(', 'Axis_Begin('):
                self.assertNotIn(call, source)

    def test_web_preparation_is_readonly_without_core_or_device_access(self):
        class NoCore:
            def request(self, *args): raise AssertionError('准备清单不能访问执行核心')
        with tempfile.TemporaryDirectory() as folder, TestClient(create_app(folder, NoCore())) as client:
            value = client.get('/api/workbench/preparation').json()
            self.assertEqual(value['domain'], 'preparation')
            self.assertFalse(value['real_motion'])
            self.assertFalse(value['axis_executor']['app_bound'])
            self.assertFalse(value['axis_executor']['device_enabled'])
            self.assertEqual(client.post('/api/workbench/preparation', json={}).status_code, 405)


if __name__ == '__main__': unittest.main()
