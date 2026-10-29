"""默认夹持98度、安全上限100度；版本兼容和持续PWM的离线验证。"""
import math
import struct
import subprocess
import unittest
from test_execution import ROOT,compile_c
import test_mech_port as port_fixture
from robot.device.mechanism import payload,GRIP_HOLD_MDEG
from robot.device.mech_sequence import Sequence,record
from robot.mech_core import DeviceCore
import test_mech_sequence as fixture


class GripLimitTests(unittest.TestCase):
    def test_native_hold_range_and_stop(self):
        control=ROOT/'firmware/User/Control';program=ROOT/'output/tests/grip-limits.exe'
        compile_c(program,[ROOT/'tests/grip_limits.c',*(control/n for n in
            ('mech_guard.c','mech_axis.c','mech_servo.c','mech_pose.c')),ROOT/'firmware/User/Device/drive.c'])
        for case in range(12):
            with self.subTest(case=case):
                r=subprocess.run([str(program),str(case)],capture_output=True)
                self.assertEqual(r.returncode,0,r.stderr.decode(errors='replace'))

    def test_board_adapter_keeps_pwm_after_done_stop_and_fault(self):
        program=ROOT/'output/tests/grip-port.exe';port_fixture.MechPortTests.build(program,ROOT/'tests/grip_port.c')
        for case in range(6):
            with self.subTest(case=case):
                r=subprocess.run([str(program),str(case)],capture_output=True)
                self.assertEqual(r.returncode,0,r.stderr.decode(errors='replace'))

    def test_versioned_single_group_anchor_and_pose_bounds(self):
        for firmware in ('GC-BASE-0.3.37','GC-BASE-0.3.38'):
            for angle in (-1,0,60,98,99,100,100.001):
                for action in ('move','group','anchor','pose'):
                    with self.subTest(firmware=firmware,angle=angle,action=action):
                        target=math.radians(angle)
                        member=dict(axis='gripper',target_rad=target,speed_rad_s=math.pi/4)
                        args=dict(_firmware=firmware,_manual_tuning=True)
                        if action=='move':args.update(member)
                        elif action=='group':args['members']=[member]
                        else:args.update(targets=dict(lift=0,reach=0,tray=0,arm=math.pi,gripper=target),confirmed=True,revision=1,physical_taught=True)
                        if 0<=angle<=(100 if firmware.endswith('.38') else 98):
                            kind,wire=payload(action,args,{},1)
                            if action=='move':self.assertEqual(struct.unpack_from('<i',wire,12)[0],round(angle*1000))
                        else:
                            with self.assertRaises(ValueError):payload(action,args,{},1)

    def ready(self,firmware='GC-BASE-0.3.38'):
        link=fixture.SequenceTests().ready();link.firmware=firmware
        return link

    def test_saved_sequence_default_and_extra_closure_are_independent(self):
        self.assertEqual(GRIP_HOLD_MDEG,98000)
        for firmware in ('GC-BASE-0.3.37','GC-BASE-0.3.38'):
            for angle in (98,100):
                with self.subTest(firmware=firmware,angle=angle):
                    core=DeviceCore(clock=lambda:1.01,site_ready=True,commissioning=True);core.link=self.ready(firmware)
                    token=core.command(dict(op='claim',client='offline',request_id='claim'))['token']
                    rows=[dict(order=1,axis='gripper',target_rad=math.radians(angle),speed_rad_s=math.pi/4)]
                    cmd=dict(op='mechanism',client='offline',token=token,request_id='save',args=dict(action='program_save',id='test',rows=rows))
                    if firmware.endswith('.37') and angle==100:
                        with self.assertRaises(ValueError):core.command(cmd)
                        self.assertFalse(core.sequences);continue
                    self.assertEqual(core.command(cmd)['status'],'SAVED_DRAFT');self.assertFalse(core.link.commands)
                    self.assertEqual(core.sequences[0]['rows'],rows)
                    sequence=Sequence();sequence.start(core.sequences[0],'ordered',core.link,1.01)
                    self.assertFalse(sequence.advance(core.link,1.01))
                    self.assertEqual(struct.unpack_from('<i',core.link.commands[0][1],36)[0],angle*1000)

    def test_fault_and_mode_cannot_be_bypassed_by_new_limit(self):
        for field in ('normal','fault','mode','old_firmware'):
            link=self.ready()
            if field=='fault':link.actual['fault']=6
            if field=='mode':link.manual_debug=False
            if field=='old_firmware':link.firmware='GC-BASE-0.3.37'
            args=dict(axis='gripper',target_rad=math.radians(100),zero_version=1,_firmware='GC-BASE-0.3.38')
            if field=='normal':self.assertEqual(link.submit('target','move',args,1.01)['status'],'QUEUED')
            else:
                with self.subTest(field=field),self.assertRaises(ValueError):link.submit('target','move',args,1.01)
                self.assertFalse(link.commands)


if __name__=='__main__':unittest.main()
