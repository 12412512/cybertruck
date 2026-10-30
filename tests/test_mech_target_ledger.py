"""真实104目标/实际反馈回放，避免合成目标永远恰等于理论值。"""
import subprocess,unittest,tempfile
from pathlib import Path
from test_execution import ROOT,compile_c

class TargetLedgerTests(unittest.TestCase):
    def test_old_four_count_gate_reproduces_snapshot_timeout(self):
        control=ROOT/'firmware/User/Control'
        source=(control/'mech_axis.c').read_text(encoding='utf-8')
        gate='return axis->manual ? Mech_RawTolerance(axis->goal.axis, MECH_MANUAL_TOL_UM) : MECH_AXIS_TARGET_RAW;'
        self.assertIn(gate,source)
        # 保持真实4raw旧门槛，不靠降低测试目标或延长期限让它通过。
        source=source.replace(gate,'return axis->manual ? MECH_AXIS_TARGET_RAW : MECH_AXIS_TARGET_RAW;')
        with tempfile.TemporaryDirectory(dir=ROOT/'output/tests') as folder:
            path=Path(folder);old=path/'old_target.c';old.write_text(source,encoding='utf-8')
            program=path/'old.exe'
            compile_c(program,[ROOT/'tests/mech_target_ledger.c',control/'mech_guard.c',old,ROOT/'firmware/User/Device/drive.c'])
            result=subprocess.run([str(program),'0'],capture_output=True)
            self.assertNotEqual(result.returncode,0)
            self.assertIn('reason=104',result.stderr.decode(errors='replace'))

    def test_0341_keeps_requested_speed_and_fresh_feedback_gates(self):
        import struct
        from robot.device.mechanism import payload
        from test_mech_sequence import SequenceTests
        link=SequenceTests().ready();link.firmware='GC-BASE-0.3.41'
        state=link.view(1.01)
        self.assertEqual(state['manual_policy']['tolerance_um'],500)
        self.assertEqual(state['manual_policy']['stable_samples'],2)
        self.assertFalse(link.view(1.3)['connected'])
        _,wire=payload('move',dict(axis='reach',target_m=0,rpm=100,_manual_tuning=True,_firmware=link.firmware),dict(reach_um=84000),1)
        self.assertEqual(struct.unpack_from('<H',wire,20)[0],100)

    def test_driver_target_quantization_and_real_mismatch(self):
        control=ROOT/'firmware/User/Control';program=ROOT/'output/tests/mech-target-ledger.exe'
        compile_c(program,[ROOT/'tests/mech_target_ledger.c',control/'mech_guard.c',control/'mech_axis.c',ROOT/'firmware/User/Device/drive.c'])
        for case in range(3):
            with self.subTest(case=case):
                result=subprocess.run([str(program),str(case)],capture_output=True)
                self.assertEqual(result.returncode,0,result.stderr.decode(errors='replace'))
