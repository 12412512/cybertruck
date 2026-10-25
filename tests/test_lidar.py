"""厂家报文复用、未知覆盖及规划到正式协议模型；绝不访问设备。"""

import binascii
import copy
import math
from pathlib import Path
import struct
import subprocess
import sys
import tempfile
import time
import unittest
from unittest.mock import patch

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'pi'))
from robot.lidar.protocol import cobs_decode, cobs_encode
from robot.lidar.fixtures import CASES, fixture
from robot.lidar.replay import replay, map_point
from robot.lidar.planning import parameters, source
from robot.workbench.contracts import PlanInput, Obstacle
from robot.workbench.scene import make_field
from robot.workbench.planning import make_plan
from robot.workbench.service import Workbench, Rejected
from robot.nav.bridge import prepare
from robot.device.preview import execute


def capture(case='detour',start='start_1'):
    request = PlanInput(source='custom',targets=['qr'],start=start)
    wire, profile = fixture(case,start,[row[0] for row in make_field(request).data['edges']])
    return wire,profile,request


def rewrite(wire, kind, change):
    frames = wire.split(b'\0')[:-1]
    for index,frame in enumerate(frames):
        raw = bytearray(cobs_decode(frame))
        if raw[4] == kind:
            change(raw)
            raw[-2:] = struct.pack('<H',binascii.crc_hqx(raw[:-2],0xffff))
            frames[index] = cobs_encode(raw)
            break
    return b'\0'.join(frames)+b'\0'


class ScanTests(unittest.TestCase):
    def test_good_both_zones_and_empty_coverage(self):
        for start in ('start_1','start_2'):
            for case in ('detour','clear'):
                with self.subTest(start=start,case=case):
                    wire,profile,request = capture(case,start)
                    scan = replay(wire,profile,start)
                    self.assertTrue(scan['usable'],scan['reason'])
                    self.assertEqual(scan['cloud_frames'],14)
                    self.assertEqual(scan['result_frames'],14)
                    self.assertTrue(scan['stop_confirmed'])
                    self.assertEqual(len(scan['obstacles']),case=='detour')
                    self.assertFalse(scan['authorizes_motion'])
                    self.assertFalse(scan['real_map_registered'])
                    if scan['obstacles']:
                        self.assertAlmostEqual(scan['obstacles'][0]['diameter_m'],.07)

    def test_bad_cases_and_external_registration_rejected(self):
        for case in CASES.keys()-{'detour','clear','blocked'}:
            wire,profile,request = capture(case)
            scan = replay(wire,profile,request.start)
            with self.subTest(case=case):
                self.assertFalse(scan['usable'],case)
                with self.assertRaises(ValueError):parameters(scan,request)
        wire,profile,request = capture()
        profile.update(synthetic=False,source='external_unregistered')
        self.assertFalse(replay(wire,profile,request.start)['usable'])

    def test_partial_coverage_blocks_unknown_roads_and_keeps_manual_obstacles(self):
        wire,profile,request = capture()
        profile['coverage'] = []
        scan = replay(wire,profile,request.start)
        request = request.model_copy(update={'obstacles':[Obstacle(id='manual',x_m=.9,y_m=2.1),
                                                         Obstacle(id='lidar_0',x_m=.1,y_m=.1)]})
        applied, roads = parameters(scan,request)
        self.assertTrue(all(state=='UNKNOWN' for state in roads.values()))
        self.assertEqual(set(applied.blocked),set(roads))
        self.assertEqual(applied.obstacles[0].id,'manual')
        self.assertAlmostEqual(applied.obstacles[1].x_m,.65)

    def test_units_and_rotated_frame(self):
        actual = map_point(1000,500,[.2,.3,math.pi/2])
        self.assertAlmostEqual(actual[0],-.3)
        self.assertAlmostEqual(actual[1],1.3)

    def test_pairing_stop_timestamp_and_overflow_not_repaired_by_later_good_frames(self):
        wire,profile,request = capture()
        variants = [wire.rsplit(b'\0',3)[0]+b'\0',
            rewrite(wire,0x87,lambda raw:raw.__setitem__(15,1)),
            rewrite(wire,0x8D,lambda raw:raw.__setitem__(9,0)),
            rewrite(wire,0x84,lambda raw:struct.pack_into('<I',raw,13,100000))]
        for broken in variants:
            self.assertFalse(replay(broken,profile,request.start)['usable'])


class LidarLinkTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.library = ROOT/f'output/tests/lidar-{time.time_ns()}.dll'
        subprocess.run([sys.executable,'-B',str(ROOT/'tools/build_odom.py'),'--output',str(cls.library)],check=True,capture_output=True)

    def test_both_zones_detour_through_real_executor_model(self):
        for start in ('start_1','start_2'):
            wire,profile,request = capture(start=start)
            scan = replay(wire,profile,start)
            applied, roads = parameters(scan,request)
            plan = make_plan(self.library,applied)
            baseline = make_plan(self.library,request)
            plan.view['lidar_source'] = source(scan,roads)
            self.assertNotEqual(plan.view['sha256'],baseline.view['sha256'])
            self.assertTrue(plan.view['obstacle_rejections'])
            bundle = prepare(plan,self.library)
            self.assertEqual(bundle['observation']['scan_id'],scan['id'])
            with tempfile.TemporaryDirectory() as folder,patch('robot.main.open_transport',side_effect=AssertionError('禁止设备访问')):
                report = execute(self.library,bundle,folder)
            self.assertEqual(report['status'],'COMPLETE',report['reason'])
            self.assertTrue(report['stop_confirmed'])
            self.assertEqual(report['commands'].count(34),len(plan.specs))

    def test_scanned_route_still_stops_on_can_or_stale_feedback(self):
        wire,profile,request = capture()
        scan = replay(wire,profile,request.start)
        applied,roads = parameters(scan,request)
        plan = make_plan(self.library,applied)
        plan.view['lidar_source'] = source(scan,roads)
        for faults in ({'at_s':3.,'drop':1},{'at_s':3.,'can':0x800}):
            with self.subTest(faults=faults),tempfile.TemporaryDirectory() as folder:
                report = execute(self.library,prepare(plan,self.library),folder,faults=faults)
            self.assertEqual(report['status'],'FAILED')
            self.assertLessEqual(report['commands'].count(33),1)
            self.assertFalse(report['authorizes_motion'])

    def test_start_occupied_has_no_route(self):
        wire,profile,request = capture('blocked')
        applied,_ = parameters(replay(wire,profile,request.start),request)
        with self.assertRaisesRegex(ValueError,'无可行路径'):make_plan(self.library,applied)

    def test_workbench_scan_identity_interlock_and_restart_do_not_resume(self):
        with tempfile.TemporaryDirectory() as folder:
            core = Workbench(self.library,folder,clock=lambda:0.)
            token, sequence = '', 0
            def send(op,args={}):
                nonlocal token, sequence
                sequence += 1
                return core.command(dict(op=op,args=args,client='lidar_test',token=token,sequence=sequence,request_id=str(sequence)))
            token = send('claim')['token']
            request = PlanInput(source='custom',targets=['qr']).model_dump()
            try:
                value = send('lidar',dict(action='replay',parameters=request))
                self.assertTrue(value['usable'])
                with self.assertRaises(Rejected):send('lidar',dict(action='plan',scan_id='0'*32,parameters=request))
                send('lidar',dict(action='plan',scan_id=value['scan_id'],parameters=request))
                deadline = time.monotonic()+45
                while core.snapshot()['planning']['status']=='PLANNING' and time.monotonic()<deadline:time.sleep(.01)
                view = core.snapshot()
                self.assertEqual(view['planning']['status'],'READY',view['planning'])
                plan = core.plans[view['planning']['plan']['id']]
                core._scan_matches(plan)
                # 调速度仍继承扫描；删掉请求中的雷达圆圈也不能静默丢失障碍。
                changed = dict(request,speed_m_s=.2)
                send('plan',changed)
                self.assertTrue(any(item.id=='lidar_0' for item in core.draft.obstacles))
                self.assertTrue(core.lidar['binding'])
                send('lidar',dict(action='replay',parameters=request))
                with self.assertRaises(Rejected):core._scan_matches(plan)
                send('lidar',dict(action='clear'))
                with self.assertRaises(Rejected):send('verify',dict(plan_id=plan.view['id']))
                send('mechanism',dict(action='zero',confirmed=True))
                send('mechanism',dict(action='anchor',zero_version=1,confirmed=True))
                send('mechanism',dict(action='move',axis='lift',target_m=.01,zero_version=1))
                with self.assertRaises(Rejected):send('lidar',dict(action='replay',parameters=request))
            finally:
                core.close()
            restored = Workbench(self.library,folder,clock=lambda:0.)
            try:
                self.assertFalse(restored.snapshot()['lidar']['usable'])
                with self.assertRaises(Rejected):restored._scan_matches(plan)
            finally:restored.close()


if __name__ == '__main__':
    unittest.main()
