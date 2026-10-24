"""工作台网关、权限与独立C闭环联调；没有真实设备连接。"""

import json
import math
import os
from pathlib import Path
import subprocess
import sys
import tempfile
import time
import unittest

from fastapi.testclient import TestClient
from pydantic import ValidationError

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'pi'))
from robot.workbench.contracts import PlanInput, Command
from robot.workbench.service import Workbench, Rejected
from robot.workbench.planning import make_plan
from robot.workbench.core import create_core
from robot.web.app import create_app
from robot.web.workbench import CoreClient


class WorkbenchTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.library = ROOT / 'output/tests' / ('workbench.dll' if os.name == 'nt' else 'workbench.so')
        subprocess.run([sys.executable, '-B', str(ROOT / 'tools/build_odom.py'),
                        '--output', str(cls.library)], check=True, capture_output=True)

    def setUp(self):
        self.folder = tempfile.TemporaryDirectory()
        self.now = 100.
        self.core = Workbench(self.library, self.folder.name, clock=lambda: self.now)
        self.seq = 0
        self.token = self.send('claim')['token']

    def tearDown(self):
        self.core.close()
        self.folder.cleanup()

    def packet(self, op, args=None, **extra):
        self.seq += 1
        return dict(op=op, args=args or {}, client='browser_a', token=getattr(self, 'token', ''),
                    sequence=self.seq, request_id=f'command_{self.seq}') | extra

    def send(self, op, args=None, **extra):
        return self.core.command(self.packet(op, args, **extra))

    def steps(self, count):
        for _ in range(count):
            self.now += .01
            self.core.tick()

    def wait_plan(self):
        deadline = time.monotonic() + 30
        while time.monotonic() < deadline:
            state = self.core.snapshot()['planning']
            if state['status'] != 'PLANNING':
                self.assertEqual(state['status'], 'READY', state)
                return state['plan']
            time.sleep(.01)
        self.fail('后台规划超时')

    def test_unique_owner_and_expiry(self):
        with self.assertRaises(Rejected):
            self.send('claim', client='browser_b')
        with self.assertRaises(Rejected):
            self.send('claim', token='')  # 只知道公开client不能取走已有令牌。
        self.now += 1.6
        with self.assertRaises(Rejected):
            self.send('renew')
        self.assertTrue(self.send('claim', client='browser_b')['token'])

    def test_manual_deadman_without_browser_or_heartbeat(self):
        self.send('manual', dict(vx_m_s=.05, vy_m_s=.0, wz_rad_s=.0))
        self.steps(20)
        self.assertGreater(self.core.sim.model.plant.x, 0)
        self.assertEqual(self.core.sim.mode, 'MANUAL')
        self.steps(20)
        self.assertIn(self.core.sim.mode, ('STOPPING', 'STOPPED'))
        self.steps(100)
        self.assertTrue(self.core.sim.stopped())
        self.assertIn('点动续约', self.core.sim.reason)

    def test_duplicate_manual_does_not_extend_deadman(self):
        packet = self.packet('manual', dict(vx_m_s=.05, vy_m_s=.0, wz_rad_s=.0))
        self.core.command(packet)
        self.steps(30)
        self.core.command(packet)
        self.steps(10)
        self.assertNotEqual(self.core.sim.mode, 'MANUAL')

    def test_manual_release_keeps_owner_but_rejects_older_inflight_pulse(self):
        self.send('manual', dict(vx_m_s=.05, vy_m_s=.0, wz_rad_s=.0))
        self.steps(20)
        delayed = self.packet('manual', dict(vx_m_s=.05, vy_m_s=.0, wz_rad_s=.0))
        self.send('manual_stop')
        with self.assertRaises(Rejected):
            self.core.command(delayed)
        self.steps(80)
        self.assertTrue(self.core.sim.stopped())
        self.assertEqual(self.core.snapshot()['owner']['client'], 'browser_a')
        self.send('manual', dict(vx_m_s=.0, vy_m_s=.05, wz_rad_s=.0))
        self.steps(20)
        self.assertGreater(max(map(abs, self.core.sim.model.plant.targets)), 5)
        self.assertEqual(self.core.sim.mode, 'MANUAL')

    def test_stop_revokes_late_commands_and_preserves_odom(self):
        origin = self.core.sim.model.pose['origin_ms']
        self.send('manual', dict(vx_m_s=.05, vy_m_s=.0, wz_rad_s=.0))
        self.steps(20)
        delayed = self.packet('manual', dict(vx_m_s=.05, vy_m_s=.0, wz_rad_s=.0))
        self.send('stop')
        with self.assertRaises(Rejected):
            self.core.command(delayed)
        self.steps(100)
        self.assertEqual(origin, self.core.sim.model.pose['origin_ms'])
        self.assertTrue(self.core.sim.stopped())
        elapsed = self.core.sim.view()['elapsed_s']
        self.send('stop')
        self.assertEqual(self.core.sim.view()['elapsed_s'], elapsed)

    def test_sequence_and_request_id_mutation(self):
        packet = self.packet('renew')
        self.core.command(packet)
        self.assertEqual(self.core.command(packet), dict(ok=True, domain='simulation'))
        with self.assertRaises(Rejected):
            self.core.command(packet | dict(op='release'))
        with self.assertRaises(Rejected):
            self.send('renew', sequence=0)

    def test_wrong_token_unknown_op_and_nonfinite_values(self):
        with self.assertRaises(Rejected):
            self.send('renew', token='bad')
        for op, args in [('flash', {}), ('manual', dict(vx_m_s=float('nan'), vy_m_s=.0, wz_rad_s=.0)),
                         ('manual', dict(vx_m_s=.05, vy_m_s=.0, wz_rad_s=.0, port='COM3')),
                         ('move', dict(x_m=1.))]:
            with self.assertRaises(ValidationError):
                self.send(op, args)

    def test_draft_planning_never_moves_or_saves_without_explicit_action(self):
        session = self.core.sim.session
        self.send('plan', dict(source='raw', speed_m_s=.15))
        plan = self.wait_plan()
        self.assertEqual(self.core.sim.mode, 'IDLE')
        self.assertEqual(self.core.sim.session, session)
        self.assertFalse((Path(self.folder.name) / 'plans').exists())
        self.send('save', dict(plan_id=plan['id']))
        path = Path(self.folder.name) / 'plans' / f'{plan["id"]}.json'
        self.assertEqual(json.loads(path.read_text(encoding='utf-8'))['sha256'], plan['sha256'])

    def test_parameter_speed_scaling_changes_time_not_shape(self):
        fast = make_plan(self.library, PlanInput(source='raw', speed_m_s=.3))
        slow = make_plan(self.library, PlanInput(source='raw', speed_m_s=.15))
        self.assertGreater(slow.view['seconds'], fast.view['seconds'] * 1.8)
        self.assertNotEqual(slow.view['sha256'], fast.view['sha256'])
        for a, b in zip(slow.view['reference'][-1], fast.view['reference'][-1]):
            self.assertAlmostEqual(a, b, places=5)
        self.assertFalse(slow.view['executable'])

    def test_two_batch_closed_loop_continuous_odom_and_immutable_active_version(self):
        plan = make_plan(self.library, PlanInput(source='batch'))
        self.core.plans[plan.view['id']] = plan
        packet = self.packet('run', dict(plan_id=plan.view['id']))
        self.core.command(packet)
        session = self.core.sim.session
        self.core.command(packet)
        self.assertEqual(self.core.sim.session, session)
        original = self.core.sim.plan.view['sha256']
        other = make_plan(self.library, PlanInput(source='raw', speed_m_s=.15))
        self.core.plans[other.view['id']] = other
        self.core.job = dict(status='READY', plan=other.view)
        self.assertEqual(self.core.sim.plan.view['sha256'], original)
        origin = self.core.sim.model.pose['origin_ms']
        for _ in range(13000):
            self.steps(1)
            if self.core.sim.mode in ('COMPLETE', 'FAILED'):
                break
        view = self.core.sim.view()
        self.assertEqual(view['mode'], 'COMPLETE', view)
        self.assertEqual(view['completed'], 9)
        self.assertEqual(view['fault'], 0)
        self.assertTrue(view['stopped'])
        self.assertEqual(origin, view['odom_origin_ms'])
        self.assertEqual([r['action_id'] for r in view['results']], list(range(1, 10)))
        self.assertFalse(view['physical_pose_valid'])
        final = plan.view['reference'][-1]
        self.assertLess(math.dist(final[:2], view['pose'][:2]), .01)
        self.assertFalse(self.core.snapshot()['owner']['client'])  # 浏览器离开不会中断自动离线任务。
        report = dict(plan={k: v for k, v in plan.view.items() if k not in ('reference', 'speed_samples')}, result=view)
        (ROOT / 'output/tests/workbench_batch.json').write_text(json.dumps(report, ensure_ascii=False, indent=2), encoding='utf-8')

    def test_fixed_move_then_manual_and_move_share_odom(self):
        self.send('move', dict(x_m=.1, speed_m_s=.05))
        origin = self.core.sim.model.pose['origin_ms']
        for _ in range(1800):
            self.steps(1)
            if self.core.sim.mode == 'COMPLETE':
                break
        self.assertEqual(self.core.sim.mode, 'COMPLETE')
        self.assertAlmostEqual(self.core.sim.model.plant.x, .1, delta=.008)
        self.token = self.send('claim')['token']
        self.send('manual', dict(vx_m_s=.0, vy_m_s=.04, wz_rad_s=.0))
        self.steps(20)
        self.send('stop')
        self.steps(100)
        self.token = self.send('claim')['token']
        self.send('move', dict(x_m=.0, y_m=.04, speed_m_s=.05))
        self.assertEqual(origin, self.core.sim.model.pose['origin_ms'])
        self.assertEqual(self.core.sim.mode, 'RUNNING')

    def test_stall_cycle_stops_manual(self):
        self.send('manual', dict(vx_m_s=.05, vy_m_s=.0, wz_rad_s=.0))
        self.core.tick(gap_s=.2)
        self.assertNotEqual(self.core.sim.mode, 'MANUAL')
        self.assertIn('超时', self.core.sim.reason)

    def test_custom_closed_start_has_no_path_and_frozen_rejects_modified_start(self):
        with self.assertRaisesRegex(ValueError, '无可行路径'):
            make_plan(self.library, PlanInput(source='custom', targets=['qr'], blocked=['start1_exit']))
        with self.assertRaises(ValueError):
            make_plan(self.library, PlanInput(source='batch', start='start_2'))

    def test_custom_topology_path_uses_selected_stations_and_speed(self):
        plan = make_plan(self.library, PlanInput(source='custom', targets=['north'], speed_m_s=.12))
        self.assertEqual(plan.view['parameters']['targets'], ['north'])
        self.assertGreater(len(plan.specs), 0)
        self.assertAlmostEqual(plan.view['reference'][-1][0], 1.2, places=5)
        self.assertAlmostEqual(plan.view['reference'][-1][1], 2.12, places=5)
        self.assertLessEqual(max(p[1] for p in plan.view['speed_samples']), .12001)

    def test_mechanism_contract_is_unknown_not_fake_zero(self):
        state = self.core.snapshot()
        self.assertFalse(state['real_connected'])
        self.assertFalse(state['capabilities']['real_motion'])
        self.assertIsNone(state['mechanism']['axes'][0]['position_m'])
        self.assertFalse(state['mechanism']['zero_valid'])
        with self.assertRaisesRegex(Rejected, '尚未接入'):
            self.send('mechanism', dict(action='move', axis='lift', target_m=.01, zero_version=1))
        with self.assertRaises(ValidationError):
            self.send('mechanism', dict(action='move', axis='lift', target_rad=.1))

    def test_web_gateway_and_stream_do_not_create_device_owner(self):
        core_app = create_core(self.library, Path(self.folder.name) / 'api', background=False)
        with TestClient(core_app) as backend:
            class Client:
                def request(inner, path, body=None):
                    return (backend.get(path) if body is None else backend.post(path, json=body)).json()
            with TestClient(create_app(self.folder.name, core_client=Client())) as web:
                self.assertIn('控制工作台', web.get('/').text)
                self.assertEqual(web.get('/diagnostics').status_code, 200)
                packet = self.packet('claim')
                self.assertEqual(web.post('/api/workbench/command', json=packet).status_code, 403)
                self.assertEqual(web.post('/api/workbench/command', json=packet,
                                         headers={'X-Workbench':'1', 'Origin':'http://other'}).status_code, 403)
                response = web.post('/api/workbench/command', json=packet, headers={'X-Workbench':'1'})
                self.assertTrue(response.json()['ok'])
                with web.websocket_connect('/api/workbench/stream') as socket:
                    state = socket.receive_json()
                    self.assertTrue(state['available'])
                    self.assertEqual(state['domain'], 'simulation')
                    self.assertFalse(state['real_connected'])
                self.assertEqual(web.post('/api/workbench/command', headers={'X-Workbench':'1'},
                                         json=packet | dict(op='connect', args={'port':'COM3'})).status_code, 422)

    def test_proxy_only_allows_local_fixed_core(self):
        for url in ('http://172.20.10.4:8080', 'http://127.0.0.1:8090/foo', 'https://example.com'):
            with self.assertRaises(ValueError):
                CoreClient(url)


if __name__ == '__main__':
    unittest.main()
