"""规划包经真实协议进入正式C执行器，全部为独立Plant和合成GPIO。"""

import ctypes as c
import copy
import hashlib
import json
from pathlib import Path
import subprocess
import sys
import tempfile
import time
import unittest
import uuid
from unittest.mock import patch

ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'pi'))
from robot.nav.bridge import prepare,validate
from robot.nav.frames import Pose,compose
from robot.nav.route import digest
from robot.workbench.planning import Plan,make_plan
from robot.workbench.contracts import PlanInput,Obstacle
from robot.workbench.service import Workbench,Rejected
from robot.device.preview import execute,PreviewTransport
from robot.drive_core import run_trial


def frozen(zone):
    route=json.loads((ROOT/f'config/start{zone}_route.json').read_text(encoding='utf-8'))
    specs=[bytes.fromhex(g)[8:] for g in route['goals']]
    view=dict(schema=2,id=uuid.uuid4().hex,domain='simulation',executable=False,
        origin=route['map_origin'],parameters=route['parameters'],field=route['field'],source='batch',
        reference=[compose(Pose(*route['map_origin']),Pose(*p)).values() for p in route['reference_trace']],
        legs=route['legs'],sha256=hashlib.sha256(b''.join(specs)).hexdigest())
    return Plan(view,specs)


class PlannedTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.library=ROOT/f'output/tests/planned-{time.time_ns()}.dll'
        subprocess.run([sys.executable,'-B',str(ROOT/'tools/build_odom.py'),'--output',str(cls.library)],check=True,capture_output=True)

    def test_both_starts_protocol_order_and_final_stop(self):
        for zone in (1,2):
            with self.subTest(zone=zone),tempfile.TemporaryDirectory() as folder,patch('robot.main.open_transport',side_effect=AssertionError('禁止设备访问')):
                bundle=prepare(frozen(zone),self.library)
                report=execute(self.library,bundle,folder)
                self.assertEqual(report['status'],'COMPLETE',report['reason'])
                self.assertTrue(report['stop_confirmed'])
                self.assertFalse(report['authorizes_motion'])
                self.assertEqual(report['commands'].count(32),1)
                self.assertEqual(report['commands'].count(33),1)
                self.assertEqual(report['commands'].count(34),9)
                complete=[r['action_id'] for r in report['events'] if r['kind']=='controller_complete']
                self.assertEqual(complete,list(range(1,10)))
                sessions={(r['boot_id'],r['session_id']) for r in report['events'] if r['kind']=='state' and r.get('execution')}
                self.assertEqual(len(sessions),1)
                self.assertGreater(report['sweep_samples'],30000)

    def test_new_obstacle_plan_is_not_a_frozen_whitelist(self):
        request=PlanInput(source='custom',targets=['qr'],obstacles=[Obstacle(id='test',x_m=.65,y_m=2.1)])
        plan=make_plan(self.library,request)
        bundle=prepare(plan,self.library)
        with tempfile.TemporaryDirectory() as folder:
            report=execute(self.library,bundle,folder)
        self.assertEqual(report['status'],'COMPLETE',report['reason'])
        self.assertNotEqual(bundle['source_specs_sha256'],prepare(frozen(1),self.library)['source_specs_sha256'])
        self.assertEqual(report['commands'].count(34),len(plan.specs))

    def test_limits_scene_change_and_domain_rejected(self):
        plan=frozen(1)
        for change in ('speed','scene','origin'):
            altered=copy.deepcopy(plan)
            if change=='speed':altered.view['parameters']['speed_m_s']=.31
            if change=='scene':altered.view['field']['body']['front_m']=.14
            if change=='origin':altered.view['origin'][2]=3.14
            with self.subTest(change=change),self.assertRaises(ValueError):prepare(altered,self.library)
        bundle=prepare(plan,self.library);bundle['domain']='commissioning';bundle['sha256']=digest(bundle)
        with self.assertRaises(ValueError):validate(bundle)

    def test_real_transport_cannot_use_dynamic_bundle(self):
        bundle=prepare(frozen(1),self.library)
        profile=json.loads((ROOT/'config/switch_trials.json').read_text(encoding='utf-8'))
        for simulation in (False,True):
            with self.assertRaisesRegex(ValueError,'禁止串口'):
                run_trial(object(),profile,'straight','unused',lambda *a,**k:None,
                    simulation=simulation,planned_route=bundle,wait_start=True)

    def test_fault_and_cancel_do_not_resume(self):
        bundle=prepare(frozen(1),self.library)
        for options in ({'faults':{'at_s':3.,'drop':1}}, {'faults':{'disconnect_at_s':3.}}, {'cancel':lambda:True}):
            with self.subTest(options=list(options)),tempfile.TemporaryDirectory() as folder:
                report=execute(self.library,bundle,folder,**options)
                self.assertNotEqual(report['status'],'COMPLETE')
                self.assertFalse(report['authorizes_motion'])
                self.assertLessEqual(report['commands'].count(33),1)
                self.assertLessEqual(report['commands'].count(34),1)
                self.assertFalse(any(r['kind']=='controller_complete' and r['action_id']==9 for r in report['events']))

    def test_workbench_interlock_and_finished_record(self):
        with tempfile.TemporaryDirectory() as folder:
            core=Workbench(self.library,folder,clock=lambda:0.)
            plan=frozen(1);core.plans[plan.view['id']]=plan
            core.draft=PlanInput.model_validate(plan.view['parameters']);core.job=dict(status='READY',plan=plan.view)
            sequence=0;token=''
            def send(op,args={}):
                nonlocal sequence
                sequence+=1
                return core.command(dict(op=op,args=args,client='test',token=token,sequence=sequence,request_id=str(sequence)))
            token=send('claim')['token']
            try:
                send('verify',dict(plan_id=plan.view['id']))
                for op,args in [('move',dict(x_m=.1)),('mechanism',dict(action='zero',confirmed=True)),('plan',{})]:
                    with self.assertRaises(Rejected):send(op,args)
                deadline=time.monotonic()+40
                while core.snapshot()['execution_check']['status']=='RUNNING' and time.monotonic()<deadline:time.sleep(.01)
                view=core.snapshot()['execution_check']
                self.assertEqual(view['status'],'COMPLETE',view)
                self.assertFalse(view['authorizes_motion'])
                self.assertEqual(view['completed'],9)
                self.assertTrue(view['stop_confirmed'])
                self.assertEqual(core.snapshot()['simulation']['mode'],'IDLE')
            finally:core.close()
