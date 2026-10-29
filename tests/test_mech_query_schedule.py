"""六节点正式调度、延迟回包及ID5故障保持；不访问设备。"""
import subprocess
import tempfile
import unittest
from pathlib import Path
from test_execution import ROOT
import test_base_startup


class MechQueryScheduleTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.binary = ROOT / 'output/tests/mech-query-schedule.exe'
        test_base_startup.BaseStartupTests.build(cls.binary, ROOT / 'tests/mech_query_schedule.c', mechanism=True)

    def test_six_node_schedule_and_faults(self):
        for case in range(9):
            with self.subTest(case=case):
                result = subprocess.run([str(self.binary), str(case)], capture_output=True)
                self.assertEqual(result.returncode, 0, result.stderr.decode(errors='replace'))
                print(result.stdout.decode().strip())

    def test_stop_at_every_query_phase_is_recoverable(self):
        for case in range(9,49):
            with self.subTest(case=case):
                result=subprocess.run([str(self.binary),str(case)],capture_output=True)
                self.assertEqual(result.returncode,0,result.stderr.decode(errors='replace'))

    def test_real_error_during_stop_still_latches(self):
        result=subprocess.run([str(self.binary),'49'],capture_output=True)
        self.assertEqual(result.returncode,0,result.stderr.decode(errors='replace'))

    def test_old_stop_classification_reproduces_false_bus_fault(self):
        source=(ROOT/'firmware/User/Board/base_port.c').read_text(encoding='utf-8')
        old=source.replace('Bus_Status() != CAN_SENT && !(bus_diag.error == BUS_ERROR_STOP && !Fault())',
                           'Bus_Status() != CAN_SENT')
        self.assertNotEqual(old,source)
        with tempfile.TemporaryDirectory(dir=ROOT/'output/tests') as folder:
            folder=Path(folder)
            (folder/'old_base_port.c').write_text(old,encoding='utf-8')
            base=(ROOT/'tests/base_startup.c').read_text(encoding='utf-8')
            (folder/'base_startup.c').write_text(base.replace('"../firmware/User/Board/base_port.c"',
                                                             '"old_base_port.c"'),encoding='utf-8')
            fixture=folder/'fixture.c'
            fixture.write_text((ROOT/'tests/mech_query_schedule.c').read_text(encoding='utf-8'),encoding='utf-8')
            binary=folder/'negative.exe'
            test_base_startup.BaseStartupTests.build(binary,fixture,mechanism=True)
            result=subprocess.run([str(binary),'10'],capture_output=True)
            self.assertNotEqual(result.returncode,0)
            self.assertIn(b'prepared',result.stderr)

    def test_equal_turns_exceed_feedback_deadline_with_delayed_replies(self):
        source = (ROOT/'firmware/User/Board/base_port.c').read_text(encoding='utf-8')
        old = source.replace('mech_slots[5] = {0, 1, 0, 1, 0}', 'mech_slots[2] = {0, 1}')
        old = old.replace('% 5U', '% 2U')
        self.assertNotEqual(old, source)
        with tempfile.TemporaryDirectory(dir=ROOT/'output/tests') as folder:
            folder = Path(folder)
            (folder/'old_base_port.c').write_text(old, encoding='utf-8')
            base = (ROOT/'tests/base_startup.c').read_text(encoding='utf-8')
            (folder/'base_startup.c').write_text(base.replace('"../firmware/User/Board/base_port.c"',
                                                            '"old_base_port.c"'), encoding='utf-8')
            fixture = folder/'fixture.c'
            fixture.write_text((ROOT/'tests/mech_query_schedule.c').read_text(encoding='utf-8'), encoding='utf-8')
            binary = folder/'negative.exe'
            test_base_startup.BaseStartupTests.build(binary, fixture, mechanism=True)
            result = subprocess.run([str(binary), '1'], capture_output=True)
            self.assertNotEqual(result.returncode, 0)
            self.assertIn(b'age <= 250U', result.stderr)


if __name__ == '__main__':
    unittest.main()
