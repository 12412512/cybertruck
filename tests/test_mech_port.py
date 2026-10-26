"""正式板级机构适配器：合成CAN/PWM验证启动、去重和故障链。"""
from pathlib import Path
import subprocess
import tempfile
import unittest
from test_execution import ROOT, compile_c

class MechPortTests(unittest.TestCase):
    @staticmethod
    def build(program, fixture):
        sources=[fixture,ROOT/'firmware/User/Comm/frame.c',ROOT/'firmware/User/Device/drive.c']
        sources += [ROOT/'firmware/User/Control'/name for name in ('mech_guard.c','mech_axis.c','mech_servo.c','mech_pose.c','mech_group.c')]
        compile_c(program,sources,extra=('-I',str(ROOT/'firmware/User/Board'),'-I',str(ROOT/'tests/mech_hal')))

    def test_native_board_adapter(self):
        with tempfile.TemporaryDirectory(dir=ROOT/'output/tests') as folder:
            program=Path(folder)/'mech-port.exe'
            self.build(program, ROOT/'tests/mech_port.c')
            for case in range(72):
                with self.subTest(case=case):
                    result=subprocess.run([str(program),str(case)],capture_output=True)
                    self.assertEqual(result.returncode,0,result.stderr.decode(errors='replace'))

    def test_query_regressions_detect_early_commit_and_submission_clock(self):
        source = (ROOT/'firmware/User/Board/mech_port.c').read_text(encoding='utf-8')
        original = (ROOT/'tests/mech_port.c').read_text(encoding='utf-8')
        early = source.replace('query_kind = code;',
                               'query_index = (uint8_t)((query_index + 1) % 8); query_kind = code;')
        submit = source.replace('query_pending = 1;',
                                'query_pending = 1; query_tick = HAL_GetTick(); query_finished = 1;')
        submit = submit.replace('        query_tick = now;\n        query_finished = 1;',
                                '        /* 负对照：保留提交时刻，错误地忽略完成时刻。 */')
        for label, mutant, case in [('early', early, 13), ('submit', submit, 14)]:
            with self.subTest(label=label), tempfile.TemporaryDirectory(dir=ROOT/'output/tests') as folder:
                folder = Path(folder)
                self.assertNotEqual(mutant, source)
                (folder/'mutant.c').write_text(mutant, encoding='utf-8')
                fixture = folder/'fixture.c'
                fixture.write_text(original.replace('"../firmware/User/Board/mech_port.c"',
                                                    '"mutant.c"'), encoding='utf-8')
                program = folder/'negative.exe'
                self.build(program, fixture)
                result = subprocess.run([str(program), str(case)], capture_output=True)
                self.assertNotEqual(result.returncode, 0)
