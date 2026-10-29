"""用户授权手动机械免轮速门槛；自动/通信保护负对照。"""
import struct,subprocess,unittest
from test_execution import ROOT
import test_base_startup as base_fixture
import test_mech_port as port_fixture
import test_physical_mechanism as fixture
from robot.device import protocol as p


class ManualParkingTests(unittest.TestCase):
    def test_native_manual_scope_and_hard_failures(self):
        program=ROOT/'output/tests/park-manual.exe'
        base_fixture.BaseStartupTests.build(program,ROOT/'tests/park_manual.c')
        for case in range(18):
            with self.subTest(case=case):
                result=subprocess.run([str(program),str(case)],capture_output=True)
                self.assertEqual(result.returncode,0,result.stderr.decode(errors='replace'))

    def test_real_port_requires_explicit_manual_and_clears_on_stop(self):
        program=ROOT/'output/tests/park-manual-port.exe'
        port_fixture.MechPortTests.build(program,ROOT/'tests/park_manual_port.c')
        for case in range(5):
            with self.subTest(case=case):
                result=subprocess.run([str(program),str(case)],capture_output=True)
                self.assertEqual(result.returncode,0,result.stderr.decode(errors='replace'))

    def test_policy_decode_requires_matching_new_firmware(self):
        for flags in (0,1,2,3,4):
            with self.subTest(flags=flags):
                link=fixture.PhysicalMechanismTests().ready();link.firmware='GC-BASE-0.3.36'
                body=struct.pack('<10I',2,0,0,0,0,1,0,0,0,0)+bytes(80)+struct.pack('<I',flags)
                req=p.Packet(69,link.boot,link.session,1,250,b'')
                reply=p.Packet(197,link.boot,link.session,1,250,bytes([p.OK])+body)
                link.custom_reply(reply,req,1.01)
                self.assertEqual(link.latched,flags==4)
                if flags<4:
                    self.assertEqual(link.park_failure['manual_speed_ignored'],bool(flags&1))
                    self.assertEqual(link.park_failure['wheel_speed_observed'],bool(flags&2))


if __name__=='__main__': unittest.main()
