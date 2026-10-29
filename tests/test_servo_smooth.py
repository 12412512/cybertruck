"""平滑舵机及停车首次快照：正式C执行器+协议，无真实设备访问。"""
from pathlib import Path
import math
import struct
import subprocess
import unittest
from test_execution import ROOT, compile_c
import test_base_startup as base_fixture
from robot.device.mechanism import MechanismLink, payload
from robot.device import protocol as p
import test_physical_mechanism as fixture


class SmoothTests(unittest.TestCase):
    def test_native_profile_and_fault_boundaries(self):
        program=ROOT/'output/tests/servo-smooth.exe'
        control=ROOT/'firmware/User/Control'
        compile_c(program,[ROOT/'tests/servo_smooth.c',*(control/n for n in
                  ('mech_guard.c','mech_axis.c','mech_servo.c')),ROOT/'firmware/User/Device/drive.c'])
        records=[]
        for case in range(15):
            with self.subTest(case=case):
                result=subprocess.run([str(program),str(case)],capture_output=True)
                self.assertEqual(result.returncode,0,result.stderr.decode(errors='replace'))
                records.append(result.stdout.decode())
        (ROOT/'output/tests/servo_smooth.log').write_text(''.join(records),encoding='utf-8')

    def test_first_parking_failure_is_frozen_and_readonly(self):
        program=ROOT/'output/tests/park-capture.exe'
        base_fixture.BaseStartupTests.build(program,ROOT/'tests/park_capture.c')
        for case in range(8):
            with self.subTest(case=case):
                result=subprocess.run([str(program),str(case)],capture_output=True)
                self.assertEqual(result.returncode,0,result.stderr.decode(errors='replace'))

    def test_new_version_timeout_covers_scurve_for_single_and_group(self):
        args=dict(axis='arm',target_rad=0,speed_rad_s=math.radians(360),
                  _manual_tuning=True,_firmware='GC-BASE-0.3.35')
        _,wire=payload('move',args,{},1)
        timeout=struct.unpack_from('<I',wire,16)[0]
        self.assertEqual(timeout,math.ceil(359/360*1000+2250+3000))
        _,group=payload('group',dict(_firmware='GC-BASE-0.3.35',members=[
            dict(axis='arm',target_rad=0,speed_rad_s=math.radians(360))]),{},1)
        self.assertEqual(struct.unpack_from('<I',group,12)[0],timeout)
        # 旧固件/旧协议保留原期限，不把尚未烧录的新行为冒充为当前行为。
        args['_firmware']='GC-BASE-0.3.34'
        _,old=payload('move',args,{},1)
        self.assertEqual(struct.unpack_from('<I',old,16)[0],math.ceil(359/360*1000+3000))

    def test_snapshot_decode_polling_and_same_boot(self):
        link=fixture.PhysicalMechanismTests().ready()
        link.firmware='GC-BASE-0.3.35';link.commissioning=link.manual_debug=True
        self.assertEqual(link.extra_request(1.01),(69,b''))
        # 下一次仍轮询正式状态，不将大诊断加入每次四状态循环。
        self.assertIn(link.extra_request(1.05)[0],(35,58,64,68))
        wire=struct.pack('<10I',1,1,583605,4,3,1,0,0,32,2)
        wire+=b''.join(struct.pack('<i4I',-3 if i==2 else 0,3,20,30,40) for i in range(4))
        request=p.Packet(69,link.boot,link.session,4,250,b'')
        answer=p.Packet(69|128,link.boot,link.session,4,250,bytes([p.OK])+wire)
        self.assertTrue(link.custom_reply(answer,request,1.06))
        self.assertEqual(link.park_failure['wheels'][2]['rpm'],-3)
        self.assertEqual(link.park_failure['action_id'],32)
        self.assertFalse(link.latched)
        wrong=p.Packet(69|128,link.boot+1,link.session,4,250,bytes([p.OK])+wire)
        link.custom_reply(wrong,request,1.07)
        self.assertTrue(link.latched)

    def test_legacy_firmware_does_not_poll_unknown_opcode(self):
        link=MechanismLink();link.firmware='GC-BASE-0.3.34'
        self.assertIn(link.extra_request(1.01)[0],(35,58,64,68))

    def test_diagnostic_does_not_starve_four_critical_states(self):
        link=fixture.PhysicalMechanismTests().ready();link.firmware='GC-BASE-0.3.35'
        seen={kind:[] for kind in (35,58,64,68,69)}
        for step in range(600):
            now=1+step*.005
            # 合成15ms往返，包含最大诊断帧的串口传输；不连接设备。
            if link.pending and now-link.pending[1]>=.015:
                link.pending=None
            wire=link.step(now)
            if wire:
                packet=p.Decoder().feed(wire)[0]
                if packet.kind in seen:seen[packet.kind].append(now)
        for kind in (35,58,64,68):
            self.assertGreater(len(seen[kind]),10)
            self.assertLess(max(b-a for a,b in zip(seen[kind],seen[kind][1:])),.25)
        self.assertLessEqual(len(seen[69]),3)

    def test_malformed_parking_record_locks_without_replaying(self):
        for offset,value in ((0,2),(4,2),(12,64),(16,5),(36,4)):
            with self.subTest(offset=offset):
                link=fixture.PhysicalMechanismTests().ready();link.firmware='GC-BASE-0.3.35'
                wire=bytearray(120);struct.pack_into('<I',wire,0,1)
                struct.pack_into('<I',wire,offset,value)
                request=p.Packet(69,link.boot,link.session,4,250,b'')
                answer=p.Packet(69|128,link.boot,link.session,4,250,bytes([p.OK])+wire)
                link.custom_reply(answer,request,1.01)
                self.assertTrue(link.latched);self.assertFalse(link.commands)

    def test_saved_sequence_uses_candidate_smooth_timeout(self):
        from robot.device.mech_sequence import Sequence, record
        import test_mech_sequence as seq_fixture
        link=seq_fixture.SequenceTests().ready();link.firmware='GC-BASE-0.3.35'
        rows=[dict(order=1,axis='arm',target_rad=0,speed_rad_s=math.radians(100))]
        sequence=Sequence();sequence.start(record('smooth_arm',rows,1),'ordered',link,1.01)
        self.assertFalse(sequence.advance(link,1.01))
        kind,wire,_=link.commands[0]
        self.assertEqual(kind,63)
        self.assertEqual(struct.unpack_from('<I',wire,12)[0],7396)
        self.assertAlmostEqual(sequence.deadline,1.01+7.396+1)


if __name__=='__main__': unittest.main()
