"""PE11平滑、其余匀速；实际设备与旧会话不参与回归。"""
import math
import struct
import subprocess
import unittest
from test_execution import ROOT, compile_c
from robot.device.mechanism import payload
from robot.device import protocol as p
import test_physical_mechanism as fixture


class ServoChannelTests(unittest.TestCase):
    def test_linear_channels_keep_rate_and_fault_boundaries(self):
        control=ROOT/'firmware/User/Control'
        program=ROOT/'output/tests/servo-channels.exe'
        compile_c(program,[ROOT/'tests/servo_channels.c',*(control/n for n in
            ('mech_guard.c','mech_axis.c','mech_servo.c')),ROOT/'firmware/User/Device/drive.c'])
        for case in range(10):
            with self.subTest(case=case):
                result=subprocess.run([str(program),str(case)],capture_output=True)
                self.assertEqual(result.returncode,0,result.stderr.decode(errors='replace'))

    def test_versioned_single_and_group_deadlines(self):
        for version in ('GC-BASE-0.3.36','GC-BASE-0.3.37'):
            for axis in ('tray','arm','gripper'):
                with self.subTest(version=version,axis=axis):
                    args=dict(axis=axis,target_rad=0,speed_rad_s=math.radians(250),
                              _manual_tuning=True,_firmware=version)
                    _,single=payload('move',args,{},1)
                    _,group=payload('group',dict(_firmware=version,members=[args]),{},1)
                    span=359 if axis=='arm' else 270
                    ramp=250/180+.25 if version.endswith('.36') or axis=='arm' else 0
                    expected=math.ceil((span/250+ramp)*1000+3000)
                    self.assertEqual(struct.unpack_from('<I',single,16)[0],expected)
                    self.assertEqual(struct.unpack_from('<I',group,12)[0],expected)

    def test_new_version_retains_parking_schema2(self):
        link=fixture.PhysicalMechanismTests().ready();link.firmware='GC-BASE-0.3.37'
        self.assertEqual(link.extra_request(1.01),(69,b''))
        data=struct.pack('<10I',2,0,0,0,0,1,0,0,0,0)+bytes(80)+struct.pack('<I',1)
        request=p.Packet(69,link.boot,link.session,1,250,b'')
        reply=p.Packet(197,link.boot,link.session,1,250,bytes([p.OK])+data)
        link.custom_reply(reply,request,1.02)
        self.assertFalse(link.latched)
        self.assertTrue(link.park_failure['manual_speed_ignored'])


if __name__=='__main__':unittest.main()
