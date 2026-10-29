"""手动策略只放宽几何流程，保留反馈、范围及真实确认区别。"""
from pathlib import Path
import subprocess,tempfile,unittest,struct
from test_execution import ROOT,compile_c
import test_physical_mechanism as fixtures
from robot.device.mechanism import payload
from robot.device import protocol as p

class ManualTests(unittest.TestCase):
    def test_native_manual_and_automatic_are_separate(self):
        with tempfile.TemporaryDirectory(dir=ROOT/'output/tests') as folder:
            program=Path(folder)/'manual.exe';control=ROOT/'firmware/User/Control'
            compile_c(program,[ROOT/'tests/mech_manual.c',*(control/n for n in ('mech_guard.c','mech_axis.c','mech_servo.c','mech_pose.c')),ROOT/'firmware/User/Device/drive.c'])
            for case in range(4,14):
                with self.subTest(case=case):
                    result=subprocess.run([str(program),str(case)],capture_output=True)
                    self.assertEqual(result.returncode,0,result.stderr.decode(errors='replace'))
            for case in range(4):
                with self.subTest(case=case):
                    result=subprocess.run([str(program),str(case)],capture_output=True)
                    self.assertEqual(result.returncode,0,result.stderr.decode(errors='replace'))

    def ready(self):
        link=fixtures.PhysicalMechanismTests().ready();link.firmware='GC-BASE-0.3.27';link.commissioning=True
        return link

    def test_new_firmware_preserves_fault_gate(self):
        for version in ('GC-BASE-0.3.28', 'GC-BASE-0.3.29', 'GC-BASE-0.3.30', 'GC-BASE-0.3.31'):
            for fault in (0,2048):
                link=self.ready();link.firmware=version;link.actual['zero_valid']=1
                link.base['fault']=fault
                args=dict(zero_version=1,manual_debug=True,confirmed=True)
                if fault:
                    with self.assertRaises(ValueError):link.submit('mode','mode',args,1.01)
                else:
                    link.submit('mode','mode',args,1.01)
                    self.assertEqual(link.commands[0][0],62)
                self.assertFalse(link.manual_debug)

    def test_mode_requires_explicit_current_permission_and_only_changes_after_ack(self):
        args=dict(zero_version=1,manual_debug=True,confirmed=True)
        kind,data=payload('mode',args,{},7)
        self.assertEqual((kind,struct.unpack('<IIBB',data)),(62,(7,1,1,1)))
        link=self.ready();link.actual['zero_valid']=1;link.submit('mode','mode',args,1.01)
        self.assertFalse(link.manual_debug)
        kind,data,rid=link.commands.popleft();link.operation=rid
        request=p.Packet(kind,link.boot,link.session,3,250,data)
        reply=p.Packet(kind|128,link.boot,link.session,3,0,bytes([p.OK])+struct.pack('<I',0))
        link.custom_reply(reply,request,1.02);self.assertTrue(link.manual_debug)
        with self.assertRaises(ValueError):link.submit('pose','pose',dict(zero_version=1),1.03)
        link.emergency('manual stop');self.assertFalse(link.manual_debug)

    def test_readonly_old_firmware_fault_stale_and_no_zero_still_reject(self):
        for case in ('readonly','old','fault','stale','zero','confirm'):
            link=self.ready();args=dict(zero_version=1,manual_debug=True,confirmed=True)
            if case=='readonly':link.commissioning=False
            elif case=='old':link.firmware='GC-BASE-0.3.26'
            elif case=='fault':link.base['fault']=2048
            elif case=='stale':link.actual_at=0
            elif case=='zero':link.actual['zero_valid']=0
            elif case=='confirm':args['confirmed']=False
            # 原通用夹具不默认建有效零点，本测试显式补齐非zero情形。
            if case!='zero':link.actual['zero_valid']=1
            with self.subTest(case=case),self.assertRaises(ValueError):link.submit('mode','mode',args,1.01)
            self.assertFalse(link.manual_debug);self.assertFalse(link.commands)
