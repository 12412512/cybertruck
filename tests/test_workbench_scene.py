"""圆形障碍扫掠、工位间隙、限速、版本恢复与后台替换的离线验证。"""

import ctypes as c
import copy
import json
import math
import os
from pathlib import Path
import random
import subprocess
import sys
import tempfile
import threading
import time
import unittest
from unittest.mock import patch

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'pi'))
from robot.nav.frames import Pose
from robot.nav.geometry import bounds, footprint, hull, intersects, circle_intersects, rectangle
from robot.nav.reference import NativeReference, specification
from robot.nav.planner import Planner
from robot.odom_sim import fixture
from robot.workbench.contracts import PlanInput, Docking, Obstacle, Move
from robot.workbench.scene import make_field
from robot.workbench.planning import make_plan, restore_plan
from robot.workbench.storage import payload
from robot.workbench.service import Workbench, Rejected
from robot.workbench.simulation import Simulation


class GeometryTests(unittest.TestCase):
    def test_circle_edge_corner_and_interior(self):
        box = rectangle([-.1, -.1, .1, .1])
        self.assertTrue(circle_intersects(box, .15, .0, .05))
        self.assertFalse(circle_intersects(box, .150001, .0, .05))
        self.assertTrue(circle_intersects(box, .0, .0, .001))
        self.assertTrue(circle_intersects(box, .125, .125, math.sqrt(2) * .025))
        self.assertFalse(circle_intersects(box, .125, .125, .03))

    def test_geometry_fast_path_keeps_1000_random_sat_results(self):
        rng = random.Random(147)
        body = dict(front_m=.145, rear_m=.145, left_m=.135, right_m=.135)
        for _ in range(1000):
            pose = Pose(rng.uniform(-1, 1), rng.uniform(-1, 1), rng.uniform(-8, 8))
            polygon = footprint(pose, body)
            original = hull(polygon)
            zone = rectangle([-.3, -.2, .4, .5])
            self.assertEqual(bounds(polygon), bounds(original))
            self.assertEqual(intersects(polygon, zone), intersects(original, zone))
            self.assertEqual(intersects(polygon, zone, (bounds(polygon), bounds(zone))), intersects(original, zone))

    def test_fifty_mm_gap_and_original_junction(self):
        field = make_field(PlanInput())
        self.assertEqual(field.data['nodes']['raw_road'], [.34, 1.2])
        surfaces = {key: box for key, box, _ in field.zones}
        poses = [Pose(*field.data['nodes']['raw'], math.pi / 2),
                 Pose(*field.data['nodes']['rough'], -math.pi / 2),
                 Pose(*field.data['nodes']['staging'], math.pi)]
        boxes = [bounds(footprint(pose, field.body)) for pose in poses]
        self.assertAlmostEqual(boxes[0][0] - surfaces['raw_surface'][2], .05)
        self.assertAlmostEqual(surfaces['rough_surface'][0] - boxes[1][2], .05)
        self.assertAlmostEqual(boxes[2][1] - surfaces['staging_surface'][3], .05)
        self.assertTrue(all(not field.errors(pose) for pose in poses))
        adjusted = make_field(PlanInput(docking=Docking(raw_m=.08, rough_m=.06, staging_m=.04)))
        self.assertAlmostEqual(adjusted.data['nodes']['raw'][0], .30)
        self.assertNotEqual(adjusted.data['version'], field.data['version'])


class SceneTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.library = ROOT / 'output/tests' / ('scene.dll' if os.name == 'nt' else 'scene.so')
        subprocess.run([sys.executable, '-B', str(ROOT / 'tools/build_odom.py'), '--output', str(cls.library)],
                       check=True, capture_output=True)

    def setUp(self):
        self.folder = tempfile.TemporaryDirectory()
        self.core = Workbench(self.library, self.folder.name, clock=lambda: 100.)
        self.seq = 0
        self.token = self.command('claim')['token']

    def tearDown(self):
        self.core.close()
        self.folder.cleanup()

    def command(self, op, args=None):
        self.seq += 1
        return self.core.command(dict(op=op, args=args or {}, client='scene_test',
                                     token=getattr(self, 'token', ''), sequence=self.seq,
                                     request_id=f'scene_{self.seq}'))

    def wait_plan(self):
        deadline = time.monotonic() + 90
        while time.monotonic() < deadline:
            value = self.core.snapshot()['planning']
            if value['status'] != 'PLANNING':
                self.assertEqual(value['status'], 'READY', value)
                return self.core.plans[value['plan']['id']]
            time.sleep(.02)
        self.fail('规划未完成')

    def test_swept_body_hits_circle_even_when_centers_and_endpoints_do_not(self):
        request = PlanInput(obstacles=[Obstacle(id='B1', x_m=.7, y_m=2.30)])
        field = make_field(request)
        start, end = Pose(.34, 2.12, 0), Pose(1.2, 2.12, 0)
        ref = NativeReference(c.CDLL(str(self.library)), start, specification('LINE', start, end))
        self.assertEqual(field.errors(start), [])
        self.assertEqual(field.errors(end), [])
        self.assertIn('obstacle:B1', field.check(ref))

    def test_dock_exit_merges_into_verified_center_turn_window(self):
        request = PlanInput(start='start_2')
        field = make_field(request)
        planner = Planner(c.CDLL(str(self.library)), fixture().geometry, field, cruise_m_s=.3)
        start = Pose(field.data['nodes']['raw'][0], field.data['nodes']['raw'][1] - 1.01e-7, math.pi / 2)
        middle = Pose(*field.data['nodes']['raw_road'], math.pi / 2)
        end = Pose(*field.data['nodes']['rough'], 3 * math.pi / 2)
        corner = Pose(*field.data['nodes']['center'], math.pi / 2)
        allowed = ('start_2',)
        lead, errors = planner.make('DIAGONAL', start, middle, 'raw_dock', allowed)
        self.assertEqual(errors, [])
        window, errors = planner.make('WINDOW', middle, end, 'center_west+center_east', allowed,
                                      corner=corner, window_m=.08)
        self.assertEqual(errors, [])
        result = planner.shortcut([lead, window], allowed)
        self.assertEqual(len(result), 1)
        self.assertLess(result[0].cost_s, lead.cost_s + window.cost_s)
        self.assertEqual(field.check(result[0].reference, allowed), [])
        self.assertLessEqual(planner.wheel_bound(result[0].reference), 100.)

    def test_dynamics_are_frozen_per_reference_without_changing_defaults(self):
        from robot.nav.reference import flow_specification
        start, end = Pose(.3, .4, 0), Pose(1.3, .4, math.pi / 2)
        lib = c.CDLL(str(self.library))
        tuned = flow_specification(lib, 'WINDOW', start, end, Pose(.8, .4, 0),
                                   window_m=.2, velocity=.45, lookahead=True,
                                   acceleration_m_s2=.5, jerk_m_s3=1.5)
        original = specification('LINE', start, end)
        self.assertAlmostEqual(original.linear.acceleration, .25)
        self.assertAlmostEqual(original.linear.jerk, .8)
        self.assertAlmostEqual(tuned.linear.acceleration, .5)
        self.assertAlmostEqual(tuned.linear.jerk, 1.5)
        reference = NativeReference(lib, start, tuned)
        for index in range(501):
            point = reference.point(reference.duration_s * index / 500)
            self.assertLessEqual(math.hypot(point.ax_m_s2, point.ay_m_s2), .5001)
            self.assertLessEqual(math.hypot(point.jx_m_s3, point.jy_m_s3), 1.5001)

    def test_proven_clear_acceleration_matches_original_adaptive_checks(self):
        field = make_field(PlanInput(obstacles=[Obstacle(id='B1', x_m=.7, y_m=2.12)]))
        lib = c.CDLL(str(self.library))
        for index in range(70):
            x = .3 + (index % 7) * .25
            y = .32 + (index // 7) * .19
            start = Pose(x, y, (index % 4) * math.pi / 2)
            kind = 'TURN' if index % 2 else 'LINE'
            end = Pose(x,y,start.yaw_rad+math.pi/2) if kind == 'TURN' else Pose(min(2.1,x+.3),y,start.yaw_rad)
            ref = NativeReference(lib,start,specification(kind,start,end))
            self.assertEqual(field.check(ref), field.check(ref,fast=False))

    def test_obstacle_replans_and_independent_body_stays_clear(self):
        plain = PlanInput(source='custom', targets=['north'], speed_m_s=.3)
        blocked = plain.model_copy(update={'obstacles':[Obstacle(id='B1', x_m=.7, y_m=2.12)]})
        direct = make_plan(self.library, plain)
        detour = make_plan(self.library, blocked)
        self.assertNotEqual(direct.view['sha256'], detour.view['sha256'])
        self.assertGreater(detour.view['distance_m'], direct.view['distance_m'])
        self.assertTrue(detour.view['obstacle_rejections'])
        simulation = Simulation(self.library)
        simulation.run(detour)
        for _ in range(20000):
            simulation.step()
            if simulation.mode in ('COMPLETE', 'FAILED'):
                break
        self.assertEqual(simulation.mode, 'COMPLETE', simulation.view())
        self.assertTrue(simulation.stopped())
        (ROOT / 'output/tests/scene_detour.json').write_text(json.dumps(dict(plan=detour.view, result=simulation.view()), ensure_ascii=False), encoding='utf-8')

    def test_one_m_s_request_keeps_wheel_limit_and_exposes_reasons(self):
        value = make_plan(self.library, PlanInput(source='custom', targets=['north'], speed_m_s=1.))
        self.assertEqual(value.view['limits']['requested_m_s'], 1.)
        self.assertLessEqual(value.view['limits']['wheel_bound_rpm'], 100.)
        self.assertLess(value.view['limits']['sampled_peak_m_s'], 1.)
        self.assertTrue(value.view['limits']['reasons'])
        self.assertFalse(value.view['executable'])

    def test_exploratory_rpm_is_model_only_and_closed_loop(self):
        plan = make_plan(self.library, PlanInput(source='custom', targets=['north'], speed_m_s=1., max_rpm=300.))
        simulation = Simulation(self.library)
        simulation.run(plan)
        self.assertEqual(simulation.model.config.max_rpm, 300.)
        self.assertGreaterEqual(simulation.model.calibration.max_rpm, 360.)
        for _ in range(6000):
            simulation.step()
            if simulation.mode in ('COMPLETE', 'FAILED'):
                break
        self.assertEqual(simulation.mode, 'COMPLETE', simulation.view())
        self.assertFalse(simulation.view()['physical_pose_valid'])

    def test_intermediate_region_parking_keeps_next_reference_continuous(self):
        plan = make_plan(self.library, PlanInput(source='custom', targets=['north', 'start_1', 'north']))
        current = Pose(*plan.view['origin'])
        lib = c.CDLL(str(self.library))
        for raw in plan.specs:
            from robot.device.tracking import Spec
            current = NativeReference(lib, current, Spec.from_buffer_copy(raw)).end
        self.assertLess(math.dist(current.values()[:2], plan.view['reference'][-1][:2]), 1e-6)

    def test_fixed_move_after_navigation_does_not_reuse_stale_scene_pointer(self):
        plan = make_plan(self.library, PlanInput(source='custom', targets=['north']))
        simulation = Simulation(self.library)
        simulation.run(plan)
        for _ in range(6000):
            simulation.step()
            if simulation.mode in ('COMPLETE', 'FAILED'):
                break
        self.assertEqual(simulation.mode, 'COMPLETE')
        origin = simulation.model.pose['origin_ms']
        simulation.move(Move(x_m=.02))
        self.assertIsNone(simulation.field)
        for _ in range(1500):
            simulation.step()
            if simulation.mode in ('COMPLETE', 'FAILED'):
                break
        self.assertEqual(simulation.mode, 'COMPLETE', simulation.view())
        self.assertEqual(simulation.model.pose['origin_ms'], origin)

    def test_unknown_or_occupied_goal_not_silently_opened(self):
        with self.assertRaisesRegex(ValueError, '无可行路径'):
            make_plan(self.library, PlanInput(source='custom', targets=['raw'],
                      obstacles=[Obstacle(id='B1', x_m=.27, y_m=1.2)]))

    def test_simulation_collision_is_latched_until_explicit_reset(self):
        plan = make_plan(self.library, PlanInput(source='custom', targets=['north']))
        simulation = Simulation(self.library)
        simulation.run(plan)
        simulation.field.obstacles.append(dict(id='unexpected',x_m=.15,y_m=2.25,diameter_m=.05))
        simulation.step()
        self.assertEqual(simulation.mode, 'FAILED')
        reason = simulation.failure_reason
        simulation.halt('普通停止不能清除模型故障')
        for _ in range(50):
            simulation.step()
        self.assertEqual(simulation.mode, 'FAILED')
        self.assertEqual(simulation.failure_reason, reason)
        simulation.reset('start_1')
        self.assertEqual(simulation.mode, 'IDLE')

    def test_defaults_preserve_versions_but_clear_draft_and_pose(self):
        self.command('plan', dict(source='custom', targets=['north'], speed_m_s=.12))
        plan = self.wait_plan()
        self.command('save', dict(plan_id=plan.view['id']))
        result = self.command('defaults')
        self.assertEqual(result['draft']['docking'], dict(raw_m=.05, rough_m=.05, staging_m=.05))
        self.assertEqual(result['draft']['obstacles'], [])
        self.assertEqual(self.core.job['status'], 'IDLE')
        self.assertEqual(self.core.sim.pose(), [.15, 2.25, .0])
        self.assertEqual(len(self.core.saved), 1)
        with self.assertRaises(Rejected):
            self.command('run', dict(plan_id=plan.view['id']))
        self.command('load', dict(plan_id=plan.view['id']))
        self.assertEqual(self.core.draft.source, 'custom')
        self.assertEqual(self.core.sim.mode, 'IDLE')
        self.assertEqual(self.core.job['status'], 'READY')

    def test_saved_workspace_restart_and_tamper_detection(self):
        self.command('plan', dict(source='custom', targets=['north'], speed_m_s=.12))
        plan = self.wait_plan()
        self.command('save', dict(plan_id=plan.view['id']))
        reopened = Workbench(self.library, self.folder.name, clock=lambda: 100.)
        try:
            self.assertEqual(reopened.job['status'], 'READY')
            self.assertEqual(reopened.draft.targets, ['north'])
            self.assertEqual(reopened.sim.mode, 'IDLE')
            self.assertFalse(reopened.snapshot()['real_connected'])
        finally:
            reopened.close()
        value = payload(plan)
        value['reference'][0][0] += .1
        with self.assertRaises(ValueError):
            restore_plan(value, self.library)

    def test_explicit_other_blue_zone_can_be_saved_and_simulated(self):
        plan = make_plan(self.library, PlanInput(source='custom', targets=['start_2']))
        restored = restore_plan(payload(plan), self.library)
        simulation = Simulation(self.library)
        simulation.run(restored)
        for _ in range(9000):
            simulation.step()
            if simulation.mode in ('COMPLETE', 'FAILED'):
                break
        self.assertEqual(simulation.mode, 'COMPLETE', simulation.view())

    def test_new_draft_cancels_old_background_result(self):
        first_started = threading.Event()
        first_exited = threading.Event()
        real = make_plan
        def slow(library, request, cancelled=None, progress=None):
            if request.speed_m_s == .11:
                first_started.set()
                while not cancelled():
                    time.sleep(.01)
                first_exited.set()
                raise InterruptedError('已取消旧草稿')
            return real(library, request, cancelled, progress)
        with patch('robot.workbench.service.make_plan', side_effect=slow):
            self.command('plan', dict(source='custom', targets=['north'], speed_m_s=.11))
            self.assertTrue(first_started.wait(1))
            self.command('plan', dict(source='custom', targets=['north'], speed_m_s=.12))
            value = self.wait_plan()
            self.assertTrue(first_exited.is_set())
            self.assertEqual(value.view['parameters']['speed_m_s'], .12)


if __name__ == '__main__':
    unittest.main()
