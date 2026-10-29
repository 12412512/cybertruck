"""复用旧轴夹具，联调正式舵机/姿态意图状态机；没有HAL或设备。"""
from pathlib import Path
import subprocess
import tempfile
import unittest
from test_execution import ROOT, compile_c


class MechServoPoseTests(unittest.TestCase):
    def test_native_servo_and_pose_cases(self):
        with tempfile.TemporaryDirectory(dir=ROOT / 'output/tests') as folder:
            program = Path(folder) / 'servo-pose.exe'
            control = ROOT / 'firmware/User/Control'
            compile_c(program, [ROOT / 'tests/mech_servo_pose.c',
                               *(control / name for name in ('mech_guard.c', 'mech_axis.c', 'mech_servo.c', 'mech_pose.c')),
                               ROOT / 'firmware/User/Device/drive.c'])
            records = []
            for case in range(27):
                with self.subTest(case=case):
                    result = subprocess.run([str(program), str(case)], capture_output=True)
                    self.assertEqual(result.returncode, 0, result.stderr.decode('utf-8', errors='replace'))
                    records.append(result.stdout.decode())
            (ROOT / 'output/tests/mech_servo_pose.log').write_text(''.join(records), encoding='utf-8')

    def test_modules_do_not_bind_io(self):
        for name in ('mech_servo.c', 'mech_pose.c'):
            source = (ROOT / 'firmware/User/Control' / name).read_text(encoding='utf-8')
            for call in ('HAL_', 'Bus_Send(', 'Can_Send(', 'PWM_Start'):
                self.assertNotIn(call, source)
        for name in ('App/app.c', 'Board/base_port.c', 'Comm/execute.c'):
            source = (ROOT / 'firmware/User' / name).read_text(encoding='utf-8')
            for call in ('Servo_Init(', 'Pose_Init(', 'Pose_Begin('):
                self.assertNotIn(call, source)


if __name__ == '__main__': unittest.main()
