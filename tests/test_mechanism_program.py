"""姿态版本→工位流程→正式格式候选载荷；没有设备访问。"""

import copy
import json
import math
from pathlib import Path
import struct
import subprocess
import sys
import tempfile
import time
import unittest

ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'pi'))
from robot.workbench.mechanism import Mechanism
from robot.workbench.program import Program
from robot.workbench.contracts import MechGoal, ProgramGoal, PlanInput
from robot.workbench.planning import make_plan
from robot.workbench.service import Workbench, Rejected
from robot.device.mechanism import payload
from robot.nav.route import validate as validate_route


class ProgramTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.library=ROOT/f'output/tests/program-{time.time_ns()}.dll'
        subprocess.run([sys.executable,'-B',str(ROOT/'tools/build_odom.py'),'--output',str(cls.library)],check=True,capture_output=True)
        cls.plan=make_plan(cls.library,PlanInput(source='raw'))

    def setUp(self):
        self.folder=tempfile.TemporaryDirectory()
        self.now=0.
        self.mech=Mechanism(self.folder.name,lambda:self.now)
        self.mech.command(MechGoal(action='zero',confirmed=True))
        self.mech.command(MechGoal(action='anchor',zero_version=1,confirmed=True))
        self.program=Program(self.mech,self.library)
        self.base=dict(lift=0.,reach=0.,arm=math.pi,tray=math.pi/6,gripper=0.)
        self.teach('work',dict(self.base,lift=.002))
        self.teach('transport',self.base)
        self.arrival=self.program.arrivals(self.plan)[0]['after_segment']

    def tearDown(self):
        self.folder.cleanup()

    def teach(self,name,targets):
        self.mech.command(MechGoal(action='teach',pose_id=name,name=name,targets=targets,zero_version=1))

    def args(self,steps=None):
        return ProgramGoal(action='save',plan_id=self.plan.view['id'],steps=steps or [
            dict(after_segment=self.arrival,pose_id='work',revision=1),
            dict(after_segment=self.arrival,pose_id='transport',revision=1)])

    def selected(self):
        return self.program.selected(ProgramGoal(action='load',revision=1))

    def tick(self):
        self.now+=.01
        self.mech.tick(.01)
        self.program.tick(True)

    def test_save_and_restart_preserve_versions_without_zero_or_resume(self):
        record=self.program.save(self.args(),self.plan)
        restored=Mechanism(self.folder.name,lambda:self.now)
        program=Program(restored,self.library)
        self.assertFalse(restored.zero_valid)
        self.assertEqual(program.records[0]['sha256'],record['sha256'])
        self.assertFalse(program.busy)
        with self.assertRaises(ValueError):self.program.save(self.args(),self.plan)

    def test_candidate_uses_exact_formal_pose_payload_and_no_hardware_permission(self):
        self.program.save(self.args(),self.plan)
        exported=self.program.compile(self.selected(),self.plan)
        candidate=json.loads(Path(exported['path']).read_text(encoding='utf-8'))
        self.assertEqual(candidate['navigation']['source_plan_id'],self.plan.view['id'])
        self.assertEqual(exported['mechanism_steps'],2)
        self.assertFalse(candidate['executable'])
        self.assertFalse(candidate['authorizes_motion'])
        with self.assertRaises(ValueError):validate_route(candidate)
        items=[row for row in candidate['timeline'] if row['kind']=='mechanism']
        for index,item in enumerate(items):
            pose=self.selected()['steps'][index]['pose']
            kind,data=payload('pose',dict(physical_taught=True,revision=1,targets=pose['targets'],single_step=True),
                dict(pose_id=index),0)
            self.assertEqual(kind,55)
            self.assertEqual(data.hex(),item['payload_template_hex'])
            self.assertEqual(len(data),42)
            self.assertEqual(data[:8],b'\0'*8)
            self.assertIn('physical_taught',item['guards'])
        self.assertEqual(struct.unpack_from('<i',bytes.fromhex(items[0]['payload_template_hex']),20)[0],2000)
        with self.assertRaises(ValueError):payload('pose',dict(revision=1,targets=self.base),{},1)

    def test_missing_transport_route_change_and_corrupt_pose_rejected(self):
        self.program.save(self.args([dict(after_segment=self.arrival,pose_id='work',revision=1)]),self.plan)
        with self.assertRaisesRegex(ValueError,'收拢'):self.program.compile(self.selected(),self.plan)
        changed=copy.deepcopy(self.plan);changed.view['id']='0'*32
        with self.assertRaisesRegex(ValueError,'路线版本'):self.program.compile(self.selected(),changed)
        self.mech.pose_versions[0]['sha256']='bad'
        with self.assertRaises(ValueError):self.selected()

    def test_joint_sequence_completes_and_retains_empty_real_counting(self):
        self.program.save(self.args(),self.plan)
        self.program.run(self.selected())
        for _ in range(2000):
            self.tick()
            if not self.program.busy:break
        self.assertEqual(self.program.mode,'COMPLETE',self.program.reason)
        self.assertEqual(self.program.index,2)
        self.assertTrue(self.mech.transport_ready())
        self.assertTrue(all(row['domain']=='simulation' for row in self.mech.results))

    def test_stop_and_restart_never_replay_partial_sequence(self):
        self.program.save(self.args(),self.plan)
        self.program.run(self.selected())
        self.tick()
        restarted=Program(self.mech,self.library)
        self.assertEqual(restarted.mode,'INTERRUPTED')
        self.program.cancel('测试停止')
        action=self.mech.action_id
        for _ in range(40):self.tick()
        self.assertEqual(self.mech.action_id,action)
        self.assertEqual(self.program.mode,'CANCELLED')

    def test_station_only_ordered_steps_and_physical_limit(self):
        with self.assertRaises(ValueError):self.program.save(self.args([dict(after_segment=64,pose_id='work',revision=1)]),self.plan)
        arrivals=self.program.arrivals(self.plan)
        with self.assertRaises(ValueError):self.program.save(self.args([
            dict(after_segment=arrivals[-1]['after_segment'],pose_id='work',revision=1),
            dict(after_segment=arrivals[0]['after_segment'],pose_id='transport',revision=1)]),self.plan)
        with self.assertRaises(ValueError):self.teach('invalid',dict(self.base,reach=.05))

    def test_workbench_program_ownership_and_lease_stop(self):
        with tempfile.TemporaryDirectory() as folder:
            core=Workbench(self.library,folder,clock=lambda:self.now)
            core.mech=self.mech;core.program=self.program
            core.plans[self.plan.view['id']]=self.plan;core.job=dict(status='READY',plan=self.plan.view)
            sequence=0;token=''
            def send(op,args={}):
                nonlocal sequence
                sequence+=1
                return core.command(dict(op=op,args=args,client='program_test',token=token,sequence=sequence,request_id=str(sequence)))
            token=send('claim')['token']
            try:
                send('program',self.args().model_dump())
                send('program',dict(action='run',revision=1))
                for op,args in [('move',dict(x_m=.1)),('mechanism',dict(action='move',axis='arm',target_rad=math.pi,zero_version=1))]:
                    with self.assertRaises(Rejected):send(op,args)
                core.tick(.01)
                self.now+=2
                core.snapshot()
                self.assertEqual(self.program.mode,'CANCELLED')
                self.assertFalse(self.mech.busy)
            finally:core.close()


if __name__=='__main__':unittest.main()
