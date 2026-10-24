"""统一坐标、道路朝向、保守扫掠及正式C多段闭环；没有真实设备访问。"""

import ctypes as c
import json
import math
import os
from pathlib import Path
import random
import subprocess
import sys
import tempfile
import time
import unittest
from unittest.mock import patch

from fastapi.testclient import TestClient

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'pi'))
from robot.nav.frames import Pose, compose, relative, legacy, canvas, angle
from robot.nav.field import Field
from robot.nav.geometry import rectangle, simplify
from robot.nav.reference import specification, NativeReference
from robot.nav.planner import Planner, view
from robot.odom_sim import fixture
from robot.route_sim import RouteSimulation
from robot.main import run
from robot.runtime import save_snapshot
from robot.web.app import create_app


class FrameTests(unittest.TestCase):
    def test_north_start_rotates_map_right_into_body_right(self):
        parent = Pose(.15, 2.25, math.pi/2)
        target = Pose(1.2, 2.05, math.pi/2)
        local = relative(parent, target)
        self.assertAlmostEqual(local.x_m, -.2)
        self.assertAlmostEqual(local.y_m, -1.05)
        self.assertAlmostEqual(compose(parent, local).x_m, 1.2)

    def test_500_se2_roundtrips_and_continuous_angles(self):
        rng = random.Random(41)
        for _ in range(500):
            parent = Pose(rng.uniform(-4, 4), rng.uniform(-4, 4), rng.uniform(-20, 20))
            local = Pose(rng.uniform(-4, 4), rng.uniform(-4, 4), rng.uniform(-20, 20))
            restored = relative(parent, compose(parent, local))
            for a, b in zip(local.values(), restored.values()):
                self.assertAlmostEqual(a, b, places=10)

    def test_legacy_and_canvas_boundaries_only(self):
        self.assertEqual(legacy(1, 2, 3), Pose(1, -2, -3))
        self.assertEqual(canvas(Pose(.2, .4, .5), 2.4, 100), (20, 200, -.5))
        with self.assertRaises(ValueError):
            Pose(float('nan'), 0, 0)
        with self.assertRaises(ValueError):
            canvas(Pose(0, 0, 0), 2.4, 0)

    def test_closed_loop_display_compression_retains_middle_corners(self):
        points=[]
        for a,b in (((0,0),(2,0)),((2,0),(2,2)),((2,2),(0,2)),((0,2),(0,0))):
            points.extend([[a[0]+(b[0]-a[0])*i/1000,a[1]+(b[1]-a[1])*i/1000,0] for i in range(1001)])
        reduced=simplify(points)
        self.assertLess(len(reduced),10)
        self.assertEqual({tuple(p[:2]) for p in reduced},{(0,0),(2,0),(2,2),(0,2)})


class NavigationTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.library = ROOT / 'output/tests' / ('navigation.dll' if os.name == 'nt' else 'navigation.so')
        subprocess.run([sys.executable, '-B', str(ROOT / 'tools/build_odom.py'),
                        '--output', str(cls.library)], check=True, capture_output=True)
        cls.lib = c.CDLL(str(cls.library))
        cls.geometry = fixture().geometry
        cls.planner = Planner(cls.lib, cls.geometry)
        cls.open_roads = {e[0]:'OPEN' for e in cls.planner.data['edges']}

    def test_unknown_blocked_and_deployment_are_not_executable(self):
        self.assertEqual(self.planner.plan('start_1', 'qr')['status'], 'NO_PATH')
        roads = self.open_roads | {'start1_exit':'BLOCKED'}
        result = self.planner.plan('start_1', 'qr', roads=roads)
        self.assertEqual(result['status'], 'NO_PATH')
        self.assertEqual(result['rejected']['start1_exit'], ['BLOCKED'])
        self.assertEqual(self.planner.plan('start_1','qr',roads=self.open_roads,mode='deployment')['status'], 'NOT_READY')

    def test_invalid_request_or_unknown_geometry_rejected(self):
        for arguments in ({'start':'bad','goal':'qr'}, {'start':'start_1','goal':'qr','start_heading':1.5},
                          {'start':'start_1','goal':'qr','roads':{'bad':'OPEN'}}):
            self.assertEqual(self.planner.plan(**arguments)['status'], 'INVALID_REQUEST')
        bad = fixture().geometry
        bad.layout = 0
        with self.assertRaises(ValueError):
            Planner(self.lib, bad)

    def test_correct_north_start_and_short_exit_then_x_travel(self):
        result = self.planner.plan('start_1', 'qr', roads=self.open_roads)
        self.assertEqual(result['status'], 'OK')
        self.assertEqual(result['segments'][0].kind, 'DIAGONAL')
        self.assertAlmostEqual(result['segments'][0].reference.start.yaw_rad, math.pi/2)
        self.assertFalse(result['executable'])
        for segment in result['segments']:
            if segment.kind == 'LINE':
                local = relative(segment.reference.start, segment.reference.end)
                self.assertAlmostEqual(local.y_m, 0, delta=1e-6)

    def test_blue_zones_are_traversable_but_start_rotation_must_fit(self):
        self.assertEqual(self.planner.plan('start_1','start_2',roads=self.open_roads)['status'], 'OK')
        self.assertFalse(self.planner.field.errors(Pose(.34,2.12,0)))
        pose = self.planner.pose('start_1',1)
        result, errors = self.planner.make('TURN',pose,Pose(pose.x_m,pose.y_m,0),'test','start_1')
        self.assertIsNone(result)
        self.assertIn('FIELD_BOUNDARY', errors)

    def test_rotating_400mm_lane_detects_mid_sweep_collision(self):
        field = Field()
        field.zones = [('a',[0,0,1,2.4],rectangle([0,0,1,2.4])),
                       ('b',[1.4,0,2.4,2.4],rectangle([1.4,0,2.4,2.4]))]
        # 两个轴向终点都能放下主体，但旋转中间加净空后不够宽。
        start = Pose(1.2,1.2,0)
        end = Pose(1.2,1.2,math.pi/2)
        self.assertFalse(field.errors(start))
        self.assertFalse(field.errors(end))
        reference = NativeReference(self.lib,start,specification('TURN',start,end))
        self.assertTrue(field.check(reference))

    def test_diagonal_full_translation_checks_obstacle_between_endpoints(self):
        field = Field()
        zone = [.95,.95,1.05,1.05]
        field.zones = [('thin',zone,rectangle(zone))]
        start, end = Pose(.5,.5,0), Pose(1.5,1.5,0)
        self.assertFalse(field.errors(start)); self.assertFalse(field.errors(end))
        ref = NativeReference(self.lib,start,specification('DIAGONAL',start,end))
        self.assertIn('thin',field.check(ref))

    def test_reference_rate_bounds_cover_dense_c_samples(self):
        for radius in (.04,.08,.16,.2):
            start,end = Pose(0,0,0),Pose(0,0,math.pi/2)
            reference = NativeReference(self.lib,start,specification('ARC',start,end,radius))
            for i in range(1001):
                p=reference.point(reference.duration_s*i/1000)
                self.assertLessEqual(math.hypot(p.vx_m_s,p.vy_m_s),reference.velocity_bound*1.0001)
                self.assertLessEqual(abs(p.wz_rad_s),reference.yaw_bound*1.0001)

    def test_smoothing_has_safe_arcs_and_lower_time_cost(self):
        plan = self.planner.plan('staging','start_1',start_heading=2,roads=self.open_roads)
        self.assertEqual(plan['status'],'OK')
        self.assertTrue(any(s.kind=='ARC' for s in plan['segments']))
        self.assertLess(plan['estimate_s'],plan['before_s'])
        for a,b in zip(plan['segments'],plan['segments'][1:]):
            self.assertLess(math.dist(a.reference.end.values()[:2],b.reference.start.values()[:2]),1e-6)
            self.assertLess(abs(a.reference.end.yaw_rad-b.reference.start.yaw_rad),1e-5)
        self.assertFalse(view(plan)['executable'])

    def test_three_road_layouts_find_detour_or_clear_no_path(self):
        for blocked in ((),('center_east',),('start1_exit',)):
            roads=self.open_roads|{edge:'BLOCKED' for edge in blocked}
            goal='rough' if blocked!=('start1_exit',) else 'qr'
            start='raw' if goal=='rough' else 'start_1'
            plan=self.planner.plan(start,goal,start_heading=1,roads=roads)
            if blocked==('start1_exit',):
                self.assertEqual(plan['status'],'NO_PATH')
            else:
                self.assertEqual(plan['status'],'OK')
                for segment in plan['segments']:
                    self.assertFalse(set(segment.road.split('+'))&set(blocked))

    def test_mirrored_start_is_separately_checked(self):
        result=self.planner.plan('start_2','qr',assigned_start='start_2',roads=self.open_roads)
        self.assertEqual(result['status'],'OK')
        self.assertAlmostEqual(result['segments'][0].reference.start.yaw_rad,math.pi/2)

    def test_missing_full_envelope_remains_a_blocker(self):
        result=self.planner.plan('start_1','qr',roads=self.open_roads)
        self.assertFalse(self.planner.data['body']['complete'])
        self.assertIn('完整车体及载物包络',result['deployment_blockers'])

    def test_frame_mismatch_is_rejected_instead_of_implicitly_registered(self):
        with tempfile.TemporaryDirectory() as directory:
            data = Field().data
            data['frame_id']='arena_draft'
            path=Path(directory)/'field.json'
            path.write_text(json.dumps(data),encoding='utf-8')
            with self.assertRaises(ValueError):Field(path)

    def test_reference_spec_is_frozen_and_sampling_is_bounded(self):
        start,end=Pose(0,0,0),Pose(.2,0,0)
        ref=NativeReference(self.lib,start,specification('LINE',start,end))
        changed=ref.spec;changed.x_m=4
        self.assertAlmostEqual(ref.spec.x_m,.2,delta=1e-7)
        self.assertAlmostEqual(ref.end.x_m,.2,delta=1e-7)
        with self.assertRaises(ValueError):ref.samples(1e-12)

    def test_real_modes_reject_route_model_before_any_transport(self):
        with patch('robot.main.open_transport') as opened:
            for mode in ('offline','observer','serial'):
                with self.assertRaises(ValueError):run(mode=mode,route_library=self.library)
            with self.assertRaises(ValueError):run(mode='sim',route_library=self.library,track_library=self.library)
            opened.assert_not_called()

    def test_complete_closed_loop_keeps_origin_and_truth_clear(self):
        model=RouteSimulation(self.library)
        report=model.run()
        (ROOT/'output/tests/navigation_summary.json').write_text(json.dumps(
            {k:v for k,v in report.items() if k not in ('trace','reference_trace','field')},
            ensure_ascii=False,indent=2),encoding='utf-8')
        self.assertEqual(report['status'],'COMPLETE',report['reason'])
        self.assertEqual(report['completed'],report['actions'])
        self.assertTrue(report['origin_consistent'])
        self.assertEqual({r['origin_ms'] for r in report['results']},{report['origin_ms']})
        self.assertTrue(all(r['stable_ms']>=150 and r['zero_ack_ms']>0 for r in report['results']))
        self.assertIsNone(report['actual_pose'])
        self.assertFalse(report['executable'])
        self.assertLess(math.dist(report['truth_pose'][:2],report['initial_pose'][:2]),.006)
        self.assertGreater(max(p[0] for p in report['trace'])-min(p[0] for p in report['trace']),1.5)
        self.assertGreater(max(p[1] for p in report['trace'])-min(p[1] for p in report['trace']),1.5)

    def test_web_navigation_is_readonly_and_expires(self):
        plan=self.planner.plan('start_1','qr',roads=self.open_roads)
        with tempfile.TemporaryDirectory() as directory:
            path=Path(directory)/'state.json'
            snapshot=dict(mode='sim',readonly=True,connected=False,navigation=dict(domain='simulation',plan=view(plan)),
                          snapshot_at_s=time.monotonic(),snapshot_wall_s=time.time())
            save_snapshot(path,snapshot)
            with TestClient(create_app(directory)) as client:
                self.assertFalse(client.get('/api/state').json()['navigation']['plan']['executable'])
                with client.websocket_connect('/ws') as socket:
                    self.assertEqual(socket.receive_json()['navigation']['domain'],'simulation')
                self.assertIn('nav-map',client.get('/').text)
                self.assertEqual(client.get('/static/navigation.js').status_code,200)
                self.assertEqual(client.post('/api/route/run',json={}).status_code,404)
                snapshot['snapshot_at_s']-=1;save_snapshot(path,snapshot)
                self.assertFalse(client.get('/api/state').json()['core_fresh'])


if __name__=='__main__':
    unittest.main()
