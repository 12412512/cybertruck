"""联调表、组报文及板端并行的离线验证。"""
import copy,math,struct,subprocess,unittest
from test_execution import ROOT,compile_c
from robot.device.mech_sequence import Sequence,groups,record
from robot.device.mechanism import payload
from robot.device import protocol as p
from robot.mech_core import DeviceCore
import test_physical_mechanism as fixture
from fastapi import FastAPI
from fastapi.testclient import TestClient
from robot.web.workbench import device_router

class SequenceTests(unittest.TestCase):
    def ready(self):
        link=fixture.PhysicalMechanismTests().ready();link.firmware='GC-BASE-0.3.32'
        link.commissioning=link.manual_debug=True
        link.actual.update(zero_valid=1,stable_samples=5,attached=1,config_mask=3)
        link.group=dict(id=0,state=0);link.group_at=1.
        return link

    def rows(self):
        return [dict(order=1,axis='lift',target_m=.01,rpm=5),dict(order=2,axis='reach',target_m=.008,rpm=5),
                dict(order=2,axis='arm',target_rad=2*math.pi/3,speed_rad_s=math.pi/2),
                dict(order=3,axis='lift',target_m=0,rpm=5)]

    def test_native_parallel_and_protections(self):
        control=ROOT/'firmware/User/Control';output=ROOT/'output/tests/mech-group.exe'
        compile_c(output,[ROOT/'tests/mech_group.c',*(control/n for n in ('mech_guard.c','mech_axis.c','mech_servo.c','mech_group.c')),ROOT/'firmware/User/Device/drive.c'])
        for case in range(8):
            with self.subTest(case=case):
                result=subprocess.run([str(output),str(case)],capture_output=True)
                self.assertEqual(result.returncode,0,result.stderr.decode(errors='replace'))

    def test_groups_sort_and_parallel_duplicate_rejection(self):
        self.assertEqual([len(s['members']) for s in groups(self.rows())],[1,2,1])
        self.assertEqual([s['order'] for s in groups(list(reversed(self.rows())))],[1,2,3])
        with self.assertRaises(ValueError):groups(self.rows(),'together')
        self.assertEqual(len(groups(self.rows()[:3],'together')),1)
        for row in (dict(order=0,axis='arm'),dict(order=True,axis='arm'),dict(order=1,axis='none')):
            with self.assertRaises(ValueError):groups([row])

    def test_wire_reuses_units_and_full_arm_range(self):
        members=[dict(axis='lift',target_m=.01,rpm=5),dict(axis='reach',target_m=.008,rpm=7),
                 dict(axis='arm',target_rad=359*math.pi/180,speed_rad_s=math.pi/180)]
        kind,data=payload('group',dict(members=members,zero_version=1),{},8)
        self.assertEqual((kind,len(data)),(63,60))
        values=struct.unpack('<5I5i5I',data)
        self.assertEqual(values[:5],(8,1,1,362000,11))
        self.assertEqual(values[5:10],(10000,8000,0,359000,0))
        self.assertEqual(values[10:],(5,7,0,1000,0))
        with self.assertRaises(ValueError):payload('group',dict(members=members+members[:1]),{},1)
        with self.assertRaises(ValueError):payload('group',dict(members=[dict(axis='arm',target_rad=2*math.pi)]),{},1)

    def test_waits_ack_and_matching_done_before_next(self):
        link=self.ready();seq=Sequence();seq.start(record('test',self.rows(),1),'ordered',link,1.01)
        self.assertFalse(seq.advance(link,1.01));self.assertEqual(len(link.commands),1)
        kind,data,rid=link.commands.popleft();self.assertEqual(kind,63)
        self.assertFalse(seq.advance(link,1.02));self.assertEqual(seq.index,0);self.assertFalse(link.commands)
        link.results.appendleft(dict(request_id=rid,result=0));link.group=dict(id=seq.group_id,state=1)
        self.assertFalse(seq.advance(link,1.03));self.assertFalse(link.commands)
        link.group['state']=2
        self.assertFalse(seq.advance(link,1.04));self.assertEqual(seq.index,1)
        self.assertEqual(len(link.commands),1)

    def test_fault_stop_and_reboot_never_resume(self):
        for change in ('fault','zero','boot','stale','group_fault'):
            link=self.ready();seq=Sequence();seq.start(record('test',self.rows(),1),'ordered',link,1.01)
            seq.advance(link,1.01);link.commands.clear()
            if change=='fault':link.base['fault']=2048
            if change=='zero':link.actual['zero_version']=2
            if change=='boot':link.boot+=1
            if change=='stale':link.actual_at=0
            if change=='group_fault':link.group=dict(id=seq.group_id,state=3,reason=2)
            self.assertTrue(seq.advance(link,1.02));self.assertEqual(seq.status,'FAILED')
            self.assertFalse(seq.advance(link,1.03));self.assertFalse(link.commands)

    def test_bad_late_target_rejected_before_any_command(self):
        link=self.ready();rows=self.rows();rows[-1]=dict(order=3,axis='gripper',target_rad=math.pi)
        with self.assertRaises(ValueError):Sequence().start(record('bad',rows,1),'ordered',link,1.01)
        self.assertFalse(link.commands)

    def test_current_core_save_run_dedup_and_stop(self):
        core=DeviceCore(clock=lambda:1.01,site_ready=True,commissioning=True);core.link=self.ready()
        token=core.command(dict(op='claim',client='test',request_id='claim'))['token']
        base=dict(op='mechanism',client='test',token=token)
        core.command(dict(base,request_id='save',args=dict(action='program_save',id='test',rows=self.rows())))
        cmd=dict(base,request_id='run',args=dict(action='program_run',id='test',revision=1,mode='ordered'))
        first=core.command(cmd);self.assertEqual(first,core.command(cmd));self.assertEqual(core.sequence.status,'RUNNING')
        with self.assertRaises(ValueError):core.command(dict(base,request_id='insert',args=dict(action='move',axis='arm',target_rad=0,zero_version=1)))
        core.command(dict(op='stop',client='test',request_id='stop'))
        self.assertEqual(core.sequence.status,'CANCELLED');self.assertTrue(core.link.action_locked)

    def test_group_status_decode_and_poll_rotation(self):
        link=self.ready()
        poll=[link.extra_request(t)[0] for t in (1.1,1.2,1.3)]
        self.assertEqual(poll,[35,58,64])
        request=p.Packet(64,link.boot,link.session,1,250,b'')
        reply=p.Packet(192,link.boot,link.session,1,0,bytes([p.OK])+struct.pack('<10I',1,1,9,2,0,31,3,30000,359000,98000))
        link.custom_reply(reply,request,1.1)
        self.assertEqual(link.group['id'],9);self.assertEqual(link.actual['group_id'],9)

    def test_all_groups_finish_without_replaying_or_counting_physical_success(self):
        link=self.ready();seq=Sequence();seq.start(record('test',self.rows(),1),'ordered',link,1.01)
        for index in range(3):
            self.assertFalse(seq.advance(link,1.01+index*.02))
            kind,data,rid=link.commands.popleft();self.assertEqual(kind,63)
            link.actual['group_id']=seq.group_id
            link.results.appendleft(dict(request_id=rid,result=0))
            link.group=dict(id=seq.group_id,state=2)
            # 下一调用可交接；该调用已经排入下一组时，不再调用一次。
            self.assertFalse(seq.advance(link,1.02+index*.02))
        self.assertEqual(seq.status,'DONE');self.assertFalse(link.commands)
        self.assertEqual(seq.view()['completion'],'servo_command_only')

    def test_missing_group_identity_is_bounded_and_not_retried(self):
        link=self.ready();seq=Sequence();seq.start(record('test',self.rows(),1),'ordered',link,1.01)
        seq.advance(link,1.01);link.commands.clear()
        link.actual_at=link.base_at=link.group_at=2.1
        self.assertTrue(seq.advance(link,2.1));self.assertEqual(seq.status,'FAILED');self.assertFalse(link.commands)

    def test_web_gateway_save_and_run_same_version(self):
        core=DeviceCore(clock=lambda:1.01,site_ready=True,commissioning=True);core.link=self.ready()
        class Client:
            def request(self,path,body=None):return core.state() if path=='/state' else core.command(body)
        app=FastAPI();app.include_router(device_router(Client()))
        with TestClient(app) as client:
            headers={'X-Workbench':'1'}
            def command(rid,args,token=''):
                return client.post('/api/device/command',headers=headers,json=dict(op='mechanism',client='test',request_id=rid,token=token,args=args))
            token=client.post('/api/device/command',headers=headers,json=dict(op='claim',client='test',request_id='claim')).json()['token']
            self.assertEqual(command('save',dict(action='program_save',id='test',rows=self.rows()),token).json()['status'],'SAVED_DRAFT')
            response=command('run',dict(action='program_run',id='test',revision=1,mode='ordered'),token)
            self.assertEqual(response.status_code,200)
            self.assertEqual(response.json()['status'],'PROGRAM_STARTED')
            self.assertEqual(client.get('/api/device/state').json()['sequence']['count'],3)
