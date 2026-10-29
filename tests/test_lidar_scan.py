"""正式雷达核心与静扫状态机合成联调，不访问任何串口或外设。"""
from pathlib import Path
import struct
import tempfile
import unittest

from fastapi.testclient import TestClient
from test_lidar_health import event,hello_data,saved_data,ready
from robot.lidar.protocol import cobs_decode
from robot.lidar.scan import StaticScan
from robot.lidar_core import LidarCore,ScanRequest,board_still,create_app


def board():
    return dict(domain='physical',connected=True,boot_id='new-boot',session_id='new-session',
        state_age_ms=10,site_ready=False,owner='',base=dict(mode=0,fault=0,stopped=True),
        mechanism={key:0 for key in ('pwm_mask','axis_state','servo_state','pose_state',
                                    'axis_id','servo_id','pose_id','zero_valid','fault')})


class Node:
    def __init__(self,now,case='normal'):
        self.now=now;self.case=case;self.zone=0;self.running=False;self.rx=bytearray()
        self.next_cloud=.1;self.sequence=0;self.commands=[]
    def write(self,wire):
        raw=cobs_decode(wire[:-1]);kind=raw[4];data=raw[9:-2];self.commands.append(kind)
        if kind==1:self.rx.extend(event(0x81,hello_data(self.zone,int(self.running)))+event(0x82,b'\1\0'))
        elif kind==5:
            self.zone=data[0] if self.case!='zone' else 2
            self.rx.extend(event(0x82,b'\5\0')+event(0x81,hello_data(self.zone)))
        elif kind==0x19:self.rx.extend(event(0x82,b'\x19\0'))
        elif kind==3:
            self.running=True
            self.rx.extend(event(0x82,b'\3\0')+event(0x83,b'\2\0\1\0\0'))
        elif kind==4:
            self.running=False;self.rx.extend(event(0x82,b'\4\0'))
            if self.case!='stop-status':self.rx.extend(event(0x83,b'\3\0\0\x0e\0'))
        elif kind==0x13:self.rx.extend(event(0x8e,saved_data())+event(0x82,b'\x13\0'))
        else:assert kind==2
        return len(wire)
    def read(self,_):
        now=self.now[0]
        if self.running and now>=self.next_cloud:
            self.next_cloud=now+.1;self.sequence+=1;seq=self.sequence
            if self.case!='no-cloud':
                stamp=100 if self.case=='stamp' else int(now*1000)
                data=event(0x84,struct.pack('<IIHHH',seq,stamp,100,3,3))
                data+=event(0x85,struct.pack('<IHB',seq,0,3)+struct.pack('<hhB',100,500,80)*3)
                data+=event(0x86,struct.pack('<IHBH',seq,3,0,510))
                if self.case!='sparse' or seq>=14 and (seq-14)%3==0:
                    data+=event(0x8d,struct.pack('<IBB',seq,1,9))
                    data+=event(0x87,struct.pack('<IBBBBhhBBH',seq,min(seq,14),0,0,1,100,500,min(seq,14),95,10))
                if self.case=='crc':data=data[:-2]+b'\xff\0'
                self.rx.extend(data)
        value=bytes(self.rx);self.rx.clear();return value


class ScanTests(unittest.TestCase):
    def run_scan(self,case='normal',fault=None,runtime=None):
        now=[.1];core=LidarCore(clock=lambda:now[0],zone=1,runtime=runtime);core.health=ready()
        core.board(board(),now[0]);request=ScanRequest(request_id='once',zone=1,duration_s=3 if case=='sparse' else 2)
        accepted=core.start_scan(request);node=Node(now,case)
        for index in range(1600):
            now[0]=.1+index*.005
            state=board()
            if fault=='fault' and now[0]>.8:state['base']['fault']=2048
            if fault=='identity' and now[0]>.8:state['boot_id']='changed'
            if fault!='stale' or now[0]<.8:core.board(state,now[0])
            core.tick(node)
            if core.last_scan:
                return core,node,accepted
        self.fail('bounded scan did not terminate')

    def test_complete_stop_and_archive_no_map_authority(self):
        with tempfile.TemporaryDirectory() as temp:
            core,node,accepted=self.run_scan(runtime=temp);s=core.last_scan
            self.assertEqual(s['status'],'COMPLETE',s['errors']);self.assertTrue(s['stop_confirmed'])
            self.assertGreaterEqual(s['matched_frames'],3);self.assertFalse(s['real_map_registered'])
            self.assertFalse(s['authorizes_motion']);self.assertEqual(s['coverage'],'unknown')
            self.assertEqual(set(node.commands),{1,2,3,4,5,0x19})
            self.assertEqual(node.commands.count(3),1);self.assertEqual(node.commands.count(4),1)
            self.assertTrue((Path(temp)/'scans'/(accepted['id']+'.json')).exists())

    def test_fault_stale_or_identity_change_stops_lidar_only(self):
        for fault in ('fault','stale','identity'):
            with self.subTest(fault=fault):
                core,node,_=self.run_scan(fault=fault)
                self.assertEqual(core.last_scan['status'],'FAILED');self.assertTrue(core.last_scan['stop_confirmed'])
                self.assertEqual(node.commands.count(3),1);self.assertEqual(node.commands.count(4),1)

    def test_results_are_paired_as_subset_of_clouds_not_one_result_per_revolution(self):
        core,_,_=self.run_scan('sparse');scan=core.last_scan
        self.assertEqual(scan['status'],'COMPLETE',scan['errors'])
        self.assertGreater(scan['cloud_frames'],scan['result_frames'])
        self.assertEqual(scan['matched_frames'],scan['result_frames'])
        self.assertEqual(scan['nonempty_cloud_frames'],scan['cloud_frames'])

    def test_bad_zone_crc_timestamp_and_no_cloud_do_not_pass(self):
        for case in ('zone','crc','stamp','no-cloud','stop-status'):
            with self.subTest(case=case):
                core,node,_=self.run_scan(case)
                self.assertEqual(core.last_scan['status'],'FAILED')
                self.assertIn(4,node.commands)
                if case=='stop-status':self.assertFalse(core.last_scan['stop_confirmed'])

    def test_duplicate_request_is_not_a_second_scan(self):
        now=[.1];core=LidarCore(clock=lambda:now[0],zone=1);core.health=ready();core.board(board(),.1)
        r=ScanRequest(request_id='once',zone=1,duration_s=2)
        first=core.start_scan(r);second=core.start_scan(r)
        self.assertEqual(first['id'],second['id']);self.assertTrue(second['repeated'])
        self.assertFalse(core.job.commands)
        with self.assertRaises(ValueError):core.start_scan(r.model_copy(update={'duration_s':3}))
        with self.assertRaises(ValueError):core.start_scan(r.model_copy(update={'request_id':'other'}))

    def test_stationary_gates_and_freshness(self):
        self.assertTrue(board_still(board(),1,1.1));self.assertFalse(board_still(board(),1,1.25))
        self.assertFalse(board_still(board(),1,.9))
        for field,value in [('connected',False),('site_ready',True),('owner','browser'),('state_age_ms',250)]:
            s=board();s[field]=value
            with self.subTest(field=field):self.assertFalse(board_still(s,1,1))
        for key in ('pwm_mask','axis_state','zero_valid'):
            s=board();s['mechanism'][key]=1;self.assertFalse(board_still(s,1,1))

    def test_default_readonly_and_site_specific_api(self):
        now=[.1];core=LidarCore(clock=lambda:now[0],zone=1);core.health=ready();core.board(board(),.1)
        with TestClient(create_app(core)) as web:
            body=dict(request_id='once',zone=1,duration_s=2)
            self.assertEqual(web.post('/scan',json=body).status_code,403)
            self.assertEqual(web.post('/scan',json=dict(body,zone=2),headers={'X-Lidar-Workbench':'1'}).status_code,409)
            self.assertEqual(web.post('/scan',json=body,headers={'X-Lidar-Workbench':'1'}).status_code,200)
            self.assertFalse(web.get('/state').json()['authorizes_motion'])
        with TestClient(create_app(LidarCore())) as web:
            self.assertEqual(web.post('/scan',json=body).status_code,404)

    def test_pre_stop_events_are_not_stop_evidence(self):
        scan=StaticScan(1,1,2,0);scan.abort('cancel',.1)
        scan.feed(event(0x82,b'\4\0')+event(0x83,b'\3\0\0\0\0'),.1)
        self.assertFalse(scan.stop_ack);self.assertFalse(scan.stop_status)
        self.assertEqual(cobs_decode(scan.step(.1)[:-1])[4],4)
        scan.step(1.6);self.assertEqual(scan.phase,'FAILED');self.assertFalse(scan.view()['stop_confirmed'])


if __name__=='__main__':unittest.main()
