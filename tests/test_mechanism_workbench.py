"""机构软件边界与工作台联调：所有反馈为合成数据，禁止依赖串口。"""

import json
import math
import os
from pathlib import Path
import subprocess
import sys
import tempfile
import unittest

from pydantic import ValidationError

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'pi'))
from robot.workbench.contracts import MechGoal
from robot.workbench.mechanism import Mechanism
from robot.workbench.service import Workbench, Rejected


class MechanismTests(unittest.TestCase):
    def setUp(self):
        self.folder = tempfile.TemporaryDirectory()
        self.now = 100.
        self.mech = Mechanism(self.folder.name, lambda: self.now)

    def tearDown(self):
        self.folder.cleanup()

    def send(self, action, **args):
        return self.mech.command(MechGoal(action=action, zero_version=self.mech.zero_version or None, **args))

    def ready(self):
        self.send('zero', confirmed=True)
        self.send('anchor', confirmed=True)

    def steps(self, count):
        for _ in range(count):
            self.now += .01
            self.mech.tick()

    def complete(self):
        for _ in range(15000):
            if not self.mech.busy:
                return
            self.steps(1)
        self.fail('机构模型超时')

    def target(self, **changes):
        return dict(self.mech.positions, **changes)

    def test_start_unknown_and_no_auto_commands(self):
        state = self.mech.view()
        self.assertFalse(state['zero_valid'])
        self.assertIsNone(state['axes'][0]['position_m'])
        self.assertTrue(all(s['actual_rad'] is None and s['estimate_rad'] is None for s in state['servos']))
        self.assertFalse(self.mech.busy)
        self.assertEqual(self.mech.cfg['slots'], [])

    def test_explicit_zero_and_anchor(self):
        with self.assertRaises(ValueError): self.send('zero')
        self.send('zero', confirmed=True)
        self.assertTrue(self.mech.zero_valid)
        self.assertEqual(self.mech.zero_version, 1)
        with self.assertRaises(ValueError): self.send('anchor')
        self.send('anchor', confirmed=True)
        with self.assertRaises(ValueError): self.send('anchor', confirmed=True)
        self.assertTrue(self.mech.transport_ready())

    def test_old_zero_and_wrong_units_rejected(self):
        self.ready()
        self.send('zero', confirmed=True)
        with self.assertRaisesRegex(ValueError, '版本'):
            self.mech.command(MechGoal(action='move', axis='lift', target_m=.01, zero_version=1))
        with self.assertRaises(ValidationError): MechGoal(action='move', axis='lift', target_rad=.1)
        with self.assertRaises(ValidationError): MechGoal(action='move', axis='arm', target_m=.01)
        with self.assertRaises(ValidationError): MechGoal(action='move', target_m=float('nan'))

    def test_axis_s_curve_stability_and_soft_limits(self):
        self.ready()
        with self.assertRaisesRegex(ValueError, '软限位'): self.send('move', target_m=.05)
        with self.assertRaisesRegex(ValueError, '软限位'): self.send('move', target_m=-.001)
        value = self.send('move', target_m=.01)
        self.assertEqual(value['status'], 'ACCEPTED')
        previous = 0.
        for _ in range(100):
            self.steps(1)
            self.assertGreaterEqual(self.mech.positions['lift'], previous)
            self.assertLessEqual((self.mech.positions['lift'] - previous) / .01, .00301)
            previous = self.mech.positions['lift']
        self.complete()
        self.assertAlmostEqual(self.mech.positions['lift'], .01)
        self.assertEqual(self.mech.results[-1]['status'], 'COMPLETE')

    def test_rotation_interlocks_and_tray_avoidance(self):
        self.ready()
        self.send('move', target_m=.01); self.complete()
        with self.assertRaisesRegex(ValueError, '旋转互锁'):
            self.send('move', axis='arm', target_rad=math.radians(83))
        self.send('move', target_m=0.); self.complete()
        self.send('move', axis='arm', target_rad=math.radians(83)); self.complete()
        with self.assertRaisesRegex(ValueError, '料盘互锁'):
            self.send('move', axis='tray', target_rad=math.radians(60))

    def test_extension_and_lift_interlocks(self):
        self.ready()
        with self.assertRaisesRegex(ValueError, '伸出互锁'): self.send('move', axis='reach', target_m=.05)
        self.send('move', target_m=.049); self.complete()
        self.send('move', axis='reach', target_m=.05); self.complete()
        with self.assertRaisesRegex(ValueError, '升降互锁'): self.send('move', target_m=0.)
        self.send('move', axis='reach', target_m=0.); self.complete()
        self.send('move', target_m=0.); self.complete()

    def test_jog_lease_and_renew_direction(self):
        self.ready()
        self.send('jog', axis='lift', velocity_m_s=.002)
        self.steps(20)
        packet = MechGoal(action='jog', axis='lift', velocity_m_s=.002, zero_version=1)
        self.assertIsNotNone(self.mech.renew_jog(packet))
        self.steps(20)
        self.assertTrue(self.mech.busy)
        self.steps(20)
        self.assertFalse(self.mech.busy)
        self.assertGreater(self.mech.positions['lift'], 0)
        self.assertEqual(self.mech.results[-1]['status'], 'CANCELLED')
        self.assertIn('350ms', self.mech.reason)

    def test_jog_reject_velocity_and_soft_limit(self):
        self.ready()
        with self.assertRaisesRegex(ValueError, '速度'): self.send('jog', velocity_m_s=.004)
        self.send('jog', velocity_m_s=-.001)
        self.steps(1)
        self.assertEqual(self.mech.positions['lift'], 0.)
        self.assertFalse(self.mech.busy)

    def test_stop_keeps_gripper_and_fault_latched(self):
        self.ready()
        self.send('move', axis='gripper', target_rad=math.radians(98)); self.complete()
        self.send('move', target_m=.01); self.steps(50)
        held = self.mech.positions['gripper']
        self.mech.fail('模拟反馈丢失')
        self.send('stop')
        self.assertEqual(self.mech.positions['gripper'], held)
        self.assertTrue(self.mech.fault)
        with self.assertRaisesRegex(ValueError, '故障'): self.send('move', target_m=0.)
        self.send('reset_model', confirmed=True)
        self.assertFalse(self.mech.zero_valid)
        self.assertIsNone(self.mech.positions['gripper'])

    def test_release_requires_explicit_request(self):
        self.ready()
        self.send('move', axis='gripper', target_rad=math.radians(98)); self.complete()
        with self.assertRaisesRegex(ValueError, '保夹互锁'):
            self.send('move', axis='gripper', target_rad=math.radians(60))
        self.send('move', axis='gripper', target_rad=math.radians(60), release_confirmed=True)
        self.complete()
        self.assertAlmostEqual(self.mech.positions['gripper'], math.radians(60))

    def test_stale_feedback_and_cycle_failure(self):
        self.ready(); self.send('move', target_m=.01)
        self.now += .26; self.mech.tick()
        self.assertTrue(self.mech.fault)
        self.assertEqual(self.mech.results[-1]['status'], 'FAILED')

    def test_invalidated_zero_cancels_no_false_rezero(self):
        self.ready(); self.send('move', target_m=.01); self.complete()
        self.send('invalidate')
        self.assertFalse(self.mech.zero_valid)
        with self.assertRaisesRegex(ValueError, '不在最高'): self.send('zero', confirmed=True)

    def test_servo_estimate_never_real_feedback(self):
        self.ready()
        self.send('move', axis='arm', target_rad=math.radians(83)); self.steps(10)
        self.assertTrue(self.mech.busy)
        self.complete()
        servo = self.mech.view()['servos'][0]
        self.assertIsNone(servo['actual_rad'])
        self.assertEqual(servo['pulse_us'], round(500 + 2000 * (359 - 83) / 359))

    def test_teach_preview_immutable_versions_and_restart(self):
        self.ready()
        target = self.target(lift=.043, arm=math.radians(83), gripper=math.radians(98))
        steps = self.send('preview', targets=target)['steps']
        self.assertEqual([a for a,_ in steps], ['arm','lift','gripper'])
        self.assertEqual(self.mech.positions['lift'], 0.)
        self.send('teach', targets=target, pose_id='slot_1', name='料位一')
        with self.assertRaisesRegex(ValueError, '历史'): self.send('teach', targets=target, pose_id='slot_1', name='料位一')
        self.send('teach', targets=target, pose_id='slot_1', name='料位一v2', revision=1)
        other = Mechanism(self.folder.name,lambda:self.now)
        self.assertEqual(len(other.pose_versions),2)
        self.assertFalse(other.zero_valid)
        self.assertFalse(other.busy)
        self.assertTrue(all(p['physical_validated'] is False for p in other.pose_versions))

    def test_pose_executes_ordered_transition_retract_first(self):
        self.ready()
        raw = self.target(lift=.049,reach=.05,gripper=math.radians(98))
        self.send('teach', targets=raw, pose_id='raw', name='取料')
        self.send('pose', pose_id='raw', revision=1); self.complete()
        car = self.target(lift=.043,reach=0.,arm=math.radians(83))
        self.send('teach', targets=car, pose_id='car', name='存车')
        value = self.send('pose', pose_id='car', revision=1)
        self.assertEqual([axis for axis,_ in value['steps']], ['reach','lift','arm','lift'])
        self.complete()
        self.assertAlmostEqual(self.mech.positions['lift'],.043)

    def test_tampered_saved_pose_rejected(self):
        self.ready()
        self.send('teach',targets=self.target(),pose_id='p',name='原姿态')
        path = Path(self.folder.name)/'mechanism_poses.json'
        data = json.loads(path.read_text(encoding='utf-8')); data['poses'][0]['targets']['lift']=.001
        path.write_text(json.dumps(data),encoding='utf-8')
        other = Mechanism(self.folder.name,lambda:self.now)
        self.assertEqual(other.pose_versions,[])
        self.assertIn('校验',other.storage_error)

    def test_pose_cancel_preserves_partial_position_and_no_auto_resume(self):
        self.ready()
        self.send('teach', targets=self.target(lift=.043, arm=math.radians(83)), pose_id='p', name='部分取消')
        self.send('pose', pose_id='p', revision=1)
        self.steps(100)
        self.send('stop')
        partial = dict(self.mech.positions)
        self.steps(500)
        self.assertEqual(self.mech.positions, partial)
        self.assertEqual(self.mech.results[-1]['status'], 'CANCELLED')
        self.assertFalse(self.mech.busy)

    def test_pose_release_rejected_before_any_axis_moves(self):
        self.ready()
        self.send('move', axis='gripper', target_rad=math.radians(98)); self.complete()
        target = self.target(lift=.01,gripper=0.)
        self.send('teach',targets=target,pose_id='release',name='释放候选')
        with self.assertRaisesRegex(ValueError,'保夹互锁'):
            self.send('pose',pose_id='release',revision=1)
        self.assertEqual(self.mech.positions['lift'],0.)
        self.assertFalse(self.mech.busy)

    def test_partial_hold_cancel_still_requires_release_permission(self):
        self.ready()
        self.send('move',axis='gripper',target_rad=math.radians(98))
        self.steps(100); self.send('stop')
        self.assertTrue(self.mech.hold_possible)
        with self.assertRaisesRegex(ValueError,'保夹互锁'):
            self.send('move',axis='gripper',target_rad=0.)


class MechanismServiceTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.library = ROOT / 'output/tests' / f'mech-{os.getpid()}.dll'
        subprocess.run([sys.executable,'-B',str(ROOT/'tools/build_odom.py'),'--output',str(cls.library)],check=True,capture_output=True)

    def setUp(self):
        self.folder = tempfile.TemporaryDirectory(); self.now=100.; self.seq=0
        self.core=Workbench(self.library,self.folder.name,clock=lambda:self.now)
        self.token=self.send('claim')['token']

    def tearDown(self):
        self.core.close(); self.folder.cleanup()

    def packet(self,op,args=None):
        self.seq+=1
        return dict(op=op,args=args or {},client='mech_test',token=getattr(self,'token',''),sequence=self.seq,request_id=f'r{self.seq}')

    def send(self,op,args=None):
        return self.core.command(self.packet(op,args))

    def mech(self,action,**args):
        return self.send('mechanism',dict(action=action,zero_version=self.core.mech.zero_version or None,**args))

    def ready(self):
        self.mech('zero',confirmed=True); self.mech('anchor',confirmed=True)

    def steps(self,count,renew=True):
        for index in range(count):
            self.now+=.01
            if renew and index%50==0: self.send('renew')
            self.core.tick()

    def test_owner_sequence_duplicate_jog_not_renewed(self):
        self.ready()
        p=self.packet('mechanism',dict(action='jog',axis='lift',velocity_m_s=.002,zero_version=1))
        first=self.core.command(p); self.steps(25)
        self.assertEqual(first,self.core.command(p)); self.steps(15)
        self.assertFalse(self.core.mech.busy)
        with self.assertRaises(Rejected): self.core.command(p | dict(token='bad',request_id='new'))

    def test_mechanism_prevents_base_motion(self):
        self.ready(); self.mech('move',target_m=.01)
        with self.assertRaisesRegex(Rejected,'机构运行'): self.send('move',dict(x_m=.01))
        self.steps(800)
        with self.assertRaisesRegex(Rejected,'运输互锁'): self.send('move',dict(x_m=.01))

    def test_base_prevents_mechanism_and_stop_stops_both(self):
        self.send('manual',dict(vx_m_s=.05,vy_m_s=.0,wz_rad_s=.0))
        with self.assertRaisesRegex(Rejected,'底盘模型'): self.mech('zero',confirmed=True)
        self.send('stop')
        self.assertFalse(self.core.mech.busy)

    def test_owner_expiry_stops_finite_mechanism_action(self):
        self.ready(); self.mech('move',target_m=.01)
        self.steps(160,renew=False)
        self.assertFalse(self.core.mech.busy)
        self.assertIn('控制权过期',self.core.mech.reason)

    def test_release_and_late_packets_cannot_resume(self):
        self.ready(); self.mech('move',target_m=.01)
        pending=self.packet('mechanism',dict(action='move',target_m=.02,zero_version=1))
        self.send('release')
        with self.assertRaises(Rejected): self.core.command(pending)
        self.assertFalse(self.core.mech.busy)

    def test_base_reset_does_not_clear_mechanism_fault(self):
        self.ready(); self.core.mech.fail('机构合成故障')
        self.send('reset')
        self.assertEqual(self.core.mech.fault,'机构合成故障')
        self.assertFalse(self.core.snapshot()['real_connected'])

    def test_mech_state_real_values_stay_unknown(self):
        self.ready()
        state=self.core.snapshot()
        self.assertEqual(state['mechanism']['backend'],'offline_model')
        self.assertTrue(state['capabilities']['mechanism'])
        self.assertFalse(state['capabilities']['real_motion'])
        self.assertIsNone(state['mechanism']['servos'][0]['actual_rad'])


if __name__ == '__main__':
    unittest.main()
