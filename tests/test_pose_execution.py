"""同一机构接口的版本冻结、逐步执行和模型证据边界。"""
import math
import os
from pathlib import Path
import subprocess
import sys
import tempfile
import unittest
from test_execution import ROOT
from robot.workbench.contracts import MechGoal
from robot.workbench.mechanism import Mechanism
from robot.workbench.core import create_core
from robot.web.app import create_app
from fastapi import HTTPException
from fastapi.testclient import TestClient


class PoseExecutionTests(unittest.TestCase):
    def setUp(self):
        self.folder = tempfile.TemporaryDirectory()
        self.now = 100.
        self.mech = Mechanism(self.folder.name, lambda: self.now)
        self.send('zero', confirmed=True)
        self.send('anchor', confirmed=True)

    def tearDown(self): self.folder.cleanup()

    def send(self, action, **values):
        return self.mech.command(MechGoal(action=action, zero_version=self.mech.zero_version or None, **values))

    def tick(self, count=1):
        for _ in range(count):
            self.now += .01
            self.mech.tick()

    def save(self):
        targets = dict(self.mech.positions, tray=math.radians(60), lift=.002)
        return self.send('teach', pose_id='step_test', name='单步验证', targets=targets)['pose']

    def test_teach_preview_and_load_do_not_move(self):
        before = dict(self.mech.positions)
        pose = self.save()
        self.send('preview', targets=pose['targets'])
        self.assertEqual(self.mech.positions, before)
        self.assertFalse(self.mech.busy)
        self.assertEqual(self.mech.action_id, 0)

    def test_single_step_binds_action_revision_index_and_zero(self):
        pose = self.save()
        self.send('pose', pose_id=pose['id'], revision=1, single_step=True)
        self.tick(10)
        self.assertTrue(self.mech.active['waiting_step'])
        before = dict(self.mech.positions)
        self.tick(100)
        self.assertEqual(self.mech.positions, before)
        args = dict(action_id=self.mech.active['id'], step_index=0, revision=1)
        for change in (dict(action_id=99), dict(step_index=1), dict(revision=2)):
            with self.assertRaises(ValueError): self.send('next', **(args | change))
        self.send('next', **args)
        with self.assertRaises(ValueError): self.send('next', **args)
        for _ in range(3000):
            self.tick()
            if self.mech.active.get('waiting_step'): break
        self.assertEqual(self.mech.active['step'], 1)
        self.assertAlmostEqual(self.mech.positions['lift'], 0)
        self.send('next', action_id=args['action_id'], step_index=1, revision=1)
        for _ in range(3000):
            self.tick()
            if not self.mech.busy: break
        self.assertFalse(self.mech.busy)
        self.assertEqual(self.mech.results[-1]['status'], 'COMPLETE')
        self.assertEqual(self.mech.results[-1]['domain'], 'simulation')

    def test_command_done_is_distinct_from_model_confirmation(self):
        self.send('move', axis='tray', target_rad=math.radians(31))
        phases = []
        for _ in range(500):
            self.tick()
            if not self.mech.active: break
            phases.append(self.mech.active.get('phase'))
            self.assertFalse(self.mech.active.get('physical_confirmed', False))
        self.assertIn('COMMAND_DONE', phases)
        self.assertIn('MODEL_CONFIRMED', phases)
        self.assertLess(phases.index('COMMAND_DONE'), phases.index('MODEL_CONFIRMED'))
        self.assertTrue(all(v['actual_rad'] is None for v in self.mech.view()['servos']))

    def test_stop_at_step_boundary_cannot_resume(self):
        pose = self.save()
        self.send('pose', pose_id=pose['id'], revision=1, single_step=True)
        self.tick()
        args = dict(action_id=self.mech.active['id'], step_index=0, revision=1)
        self.send('stop')
        with self.assertRaises(ValueError): self.send('next', **args)
        self.assertEqual(self.mech.results[-1]['status'], 'CANCELLED')

    def test_saved_target_mutation_is_rejected_before_action(self):
        pose = self.save()
        self.mech.pose_versions[0]['targets']['lift'] = .003
        with self.assertRaisesRegex(ValueError, '校验'):
            self.send('pose', pose_id=pose['id'], revision=1)
        self.assertFalse(self.mech.busy)
        self.assertEqual(self.mech.action_id, 0)


class PoseGatewayTests(unittest.TestCase):
    def test_gateway_core_single_step_and_model_evidence(self):
        library = ROOT / 'output/tests' / ('pose-gateway.dll' if os.name == 'nt' else 'pose-gateway.so')
        subprocess.run([sys.executable, '-B', str(ROOT / 'tools/build_odom.py'), '--output', str(library)],
                       check=True, capture_output=True)
        with tempfile.TemporaryDirectory() as folder:
            app = create_core(library, folder, background=False)
            core = app.state.core
            now = [100.]
            core.clock = core.mech.clock = lambda: now[0]
            core.mech.updated = now[0]
            with TestClient(app) as backend:
                class Client:
                    def request(self, path, body=None):
                        response = backend.get(path) if body is None else backend.post(path, json=body)
                        if response.status_code >= 400:
                            raise HTTPException(response.status_code, response.json()['detail'])
                        return response.json()
                with TestClient(create_app(folder, Client())) as web:
                    seq, token = 0, ''
                    def packet(op, args=None):
                        nonlocal seq
                        seq += 1
                        return dict(op=op, args=args or {}, client='pose_browser', token=token,
                                    sequence=seq, request_id=f'pose_{seq}')
                    def send(body):
                        return web.post('/api/workbench/command', json=body, headers={'X-Workbench':'1'})
                    token = send(packet('claim')).json()['token']
                    for action in ('zero', 'anchor'):
                        self.assertEqual(send(packet('mechanism', dict(action=action, confirmed=True,
                            zero_version=core.mech.zero_version or None))).status_code, 200)
                    targets = dict(core.mech.positions, tray=math.radians(31))
                    body = dict(action='teach', zero_version=1, pose_id='gateway', name='网关联调', targets=targets)
                    self.assertEqual(send(packet('mechanism', body)).status_code, 200)
                    run = packet('mechanism', dict(action='pose', zero_version=1, pose_id='gateway', revision=1, single_step=True))
                    self.assertEqual(send(run).status_code, 200)
                    now[0] += .01; core.tick()
                    state = web.get('/api/workbench/state').json()
                    self.assertEqual(state['mechanism']['active']['phase'], 'WAIT_STEP')
                    self.assertFalse(state['real_connected'])
                    args = dict(action='next', zero_version=1, action_id=core.mech.active['id'], step_index=0, revision=1)
                    wrong = packet('mechanism', args | dict(revision=2))
                    self.assertEqual(send(wrong).status_code, 409)
                    advance = packet('mechanism', args)
                    result = send(advance).json()
                    self.assertEqual(send(advance).json(), result)
                    phases = []
                    for index in range(300):
                        now[0] += .01
                        if index % 50 == 0: self.assertEqual(send(packet('renew')).status_code, 200)
                        core.tick()
                        if core.mech.active: phases.append(core.mech.active.get('phase'))
                        else: break
                    self.assertIn('COMMAND_DONE', phases)
                    self.assertIn('MODEL_CONFIRMED', phases)
                    state = web.get('/api/workbench/state').json()
                    self.assertFalse(state['mechanism']['busy'])
                    self.assertFalse(state['capabilities']['real_motion'])
                    self.assertTrue(all(s['actual_rad'] is None for s in state['mechanism']['servos']))


if __name__ == '__main__': unittest.main()
