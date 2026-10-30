"""Emm加速度参数贯通、旧版本兼容及独立公式检查。"""
import subprocess,unittest,struct,copy
from test_execution import ROOT,compile_c
import test_mech_port as port
from robot.device.mechanism import payload
from robot.device.axis_profile import travel_s
from robot.device.mech_sequence import groups,record

class AxisAccelTests(unittest.TestCase):
    def test_c_and_board_adapter(self):
        control=ROOT/'firmware/User/Control';program=ROOT/'output/tests/mech-accel.exe'
        compile_c(program,[ROOT/'tests/mech_accel.c',*(control/n for n in ('mech_guard.c','mech_axis.c','mech_servo.c','mech_group.c')),ROOT/'firmware/User/Device/drive.c'])
        for case in range(7):
            result=subprocess.run([str(program),str(case)],capture_output=True);self.assertEqual(result.returncode,0,result.stderr.decode(errors='replace'))
        program=ROOT/'output/tests/mech-accel-port.exe';port.MechPortTests.build(program,ROOT/'tests/mech_accel_port.c')
        for case in range(5):
            result=subprocess.run([str(program),str(case)],capture_output=True);self.assertEqual(result.returncode,0,result.stderr.decode(errors='replace'))

    def test_wire_legacy_and_per_axis_values(self):
        args=dict(axis='reach',target_m=.084,rpm=100,_manual_tuning=True,_firmware='GC-BASE-0.3.42',accel=160)
        _,new=payload('move',args,{},1);self.assertEqual(len(new),24);self.assertEqual(new[23],160)
        legacy=dict(args,_firmware='GC-BASE-0.3.41',accel=1)
        _,old=payload('move',legacy,{},1);self.assertEqual(len(old),23)
        with self.assertRaises(ValueError):payload('move',dict(legacy,accel=160),{},1)
        for acc in (-1,256,1.5,True):
            with self.assertRaises(ValueError):payload('move',dict(args,accel=acc),{},1)
        members=[dict(axis='lift',target_m=.045,rpm=120,accel=100),dict(axis='reach',target_m=.084,rpm=100,accel=200)]
        saved=copy.deepcopy(members)
        _,wire=payload('group',dict(members=members,_firmware='GC-BASE-0.3.42'),{},1)
        self.assertEqual((len(wire),wire[60:]),(62,bytes([100,200])))
        self.assertEqual(members,saved)

    def test_profile_and_deadline_match_selected_acceleration(self):
        self.assertAlmostEqual(travel_s('reach',84000,100,0),.084/.124*.6)
        self.assertGreater(travel_s('reach',84000,100,1),travel_s('reach',84000,100,160))
        self.assertGreater(travel_s('reach',84000,100,160),travel_s('reach',84000,100,200))
        _,slow=payload('move',dict(axis='reach',target_m=.084,rpm=100,_manual_tuning=True,_firmware='GC-BASE-0.3.42',accel=1),{},1)
        _,fast=payload('move',dict(axis='reach',target_m=.084,rpm=100,_manual_tuning=True,_firmware='GC-BASE-0.3.42',accel=200),{},1)
        self.assertGreater(struct.unpack_from('<I',slow,16)[0],struct.unpack_from('<I',fast,16)[0])

    def test_saved_groups_preserve_accel_without_rewriting_legacy(self):
        rows=[dict(axis='reach',target_m=.084,rpm=100,accel=160,order=1)]
        value=record('fast',rows,1);self.assertEqual(groups(value['rows'])[0]['members'][0]['accel'],160)
        old=[dict(axis='reach',target_m=.084,rpm=100,order=1)]
        self.assertNotIn('accel',record('old',old,1)['rows'][0])
