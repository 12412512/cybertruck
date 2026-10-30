"""0.3.43终点容差：真实106回放、交接与超出边界负例。"""
import subprocess, unittest
from test_execution import ROOT, compile_c
from robot.device.mechanism import payload

class EndpointBandTests(unittest.TestCase):
    def test_recorded_endpoint_and_boundary(self):
        control=ROOT/'firmware/User/Control'
        exe=ROOT/'output/tests/mech-endpoint-band.exe'
        compile_c(exe,[ROOT/'tests/mech_endpoint_band.c',control/'mech_axis.c',control/'mech_guard.c',ROOT/'firmware/User/Device/drive.c'])
        for case in range(3):
            result=subprocess.run([str(exe),str(case)],capture_output=True)
            self.assertEqual(result.returncode,0,result.stderr.decode(errors='replace'))

    def test_43_acceleration_wire(self):
        for version in ('GC-BASE-0.3.42','GC-BASE-0.3.43'):
            _,wire=payload('move',dict(axis='reach',target_m=.05,rpm=200,accel=200,_manual_tuning=True,_firmware=version),{},1)
            self.assertEqual((len(wire),wire[-1]),(24,200))
