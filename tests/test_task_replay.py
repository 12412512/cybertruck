"""B端离线合同/证据/事务回归；合成断言不能作为现场成功。"""

import copy
import json
import subprocess
from pathlib import Path
import sys
import tempfile
import unittest

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
sys.path.insert(0, str(ROOT / 'pi'))

from robot.ledger import Ledger, LedgerError
from robot.task import Task, phases, reduce
from robot.task_check import TaskError
from robot.task_replay import OfflineMission
from robot.vision.task_evidence import candidate_summary
from tools.replay_mission import Clock, context, evidence, receipt, run

CODE = '136+321+631+312'
CORE = dict(owner='sole-core', boot_id='boot-1', session_id='session-1')
PARAMETERS = dict(revision=8, sha256='c' * 64)


class ReplayTests(unittest.TestCase):
    def setUp(self):
        self.directory = tempfile.TemporaryDirectory()
        self.ledger = Ledger(Path(self.directory.name) / 'task.sqlite3')
        self.clock = Clock()
        self.mission = OfflineMission.create(self.ledger, 'owner', self.clock, CORE, PARAMETERS)
        self.mission.task.start()
        self.sequence = 0

    def tearDown(self):
        self.ledger.close()
        self.directory.cleanup()

    def running(self):
        request = self.mission.request()
        for status in ('ACCEPTED', 'RUNNING'):
            self.mission.receive(receipt(request, status, self.clock))
        return request

    def frames(self, rule):
        frames, self.sequence = evidence(self.mission, self.clock, rule, self.sequence, CODE)
        return frames

    def finish(self, request):
        for rule in phases(request['goal'], self.mission.task.state):
            self.mission.checkpoint(self.frames(rule))
        self.mission.receive(receipt(request, 'COMPLETE', self.clock))
        self.mission.confirm()

    def transfer(self, kind='PICK'):
        for _ in range(34):
            request = self.running()
            if request['goal']['kind'] == kind:
                return request
            self.finish(request)
        self.fail('未到达取放步骤')

    def assert_unchanged(self, operation):
        before = self.mission.task.state
        events = len(self.ledger.events(self.mission.task.run_id))
        with self.assertRaises((TaskError, LedgerError)):
            operation()
        self.assertEqual(self.mission.task.state, before)
        self.assertEqual(len(self.ledger.events(self.mission.task.run_id)), events)

    def test_request_is_bound_to_goal_core_parameters_and_has_no_transport(self):
        request = self.mission.request()
        self.assertEqual(request['core'], CORE)
        self.assertEqual(request['parameters'], PARAMETERS)
        self.assertEqual(request['transport'], 'none')
        self.assertEqual(request['goal']['kind'], 'SCAN')
        self.assert_unchanged(self.mission.request)
        request['goal']['kind'] = 'MOVE'
        self.assertEqual(self.mission.pending_request()['goal']['kind'], 'SCAN')

    def test_changed_session_parameters_goal_or_action_rejected(self):
        request = self.mission.request()
        variants = [('core', 'boot_id', 'other'), ('core', 'session_id', 'old'),
                    ('core', 'owner', 'second-core'), ('parameters', 'revision', 9),
                    ('parameters', 'sha256', 'd' * 64), ('goal', 'station', 'rough')]
        for parent, field, value in variants:
            invalid = receipt(request, 'ACCEPTED', self.clock)
            invalid = copy.deepcopy(invalid)
            invalid['request'][parent][field] = value
            with self.subTest(field=field):
                self.assert_unchanged(lambda: self.mission.receive(invalid))
        for field in ('run_id', 'action_id', 'session_id', 'request_sha256'):
            invalid = copy.deepcopy(receipt(request, 'ACCEPTED', self.clock))
            invalid['request'][field] = 'old'
            self.assert_unchanged(lambda: self.mission.receive(invalid))

    def test_completed_receipt_without_evidence_never_counts(self):
        request = self.transfer()
        self.mission.receive(receipt(request, 'COMPLETE', self.clock))
        self.assertEqual(self.mission.task.state['pending']['status'], 'EXECUTED')
        self.assertEqual(self.mission.task.state['counts'], dict(grabs=0, places=0))
        self.assert_unchanged(self.mission.confirm)

    def test_duplicate_and_conflicting_receipts_before_and_after_confirmation(self):
        request = self.running()
        message = receipt(request, 'COMPLETE', self.clock)
        self.mission.receive(message)
        state = self.mission.task.state
        self.clock.value += .1
        self.mission.receive(message)
        self.assertEqual(self.mission.task.state, state)
        self.mission.confirm()
        state = self.mission.task.state
        self.clock.value += 1
        self.mission.receive(message)
        self.assertEqual(self.mission.task.state, state)
        invalid = copy.deepcopy(message)
        invalid['result']['ok'] = False
        self.assert_unchanged(lambda: self.mission.receive(invalid))

    def test_unknown_execution_blocks_progress_and_preserves_unknown_inventory(self):
        request = self.transfer()
        message = receipt(request, 'UNKNOWN', self.clock)
        self.mission.receive(message)
        state = self.mission.task.state
        self.assertEqual(state['status'], 'RECONCILE_REQUIRED')
        self.assertEqual(state['pending']['status'], 'UNKNOWN')
        self.assertEqual(state['items']['b1-1']['location'], 'UNKNOWN')
        self.assertEqual(state['counts'], dict(grabs=0, places=0))
        self.assert_unchanged(self.mission.request)
        self.assert_unchanged(lambda: self.mission.receive(receipt(request, 'COMPLETE', self.clock)))

    def test_fault_at_acceptance_waits_for_reconciliation(self):
        request = self.mission.request()
        self.mission.receive(receipt(request, 'ACCEPTED', self.clock))
        self.mission.receive(receipt(request, 'FAULT', self.clock))
        self.assertEqual(self.mission.task.state['status'], 'RECONCILE_REQUIRED')

    def test_stale_future_nan_receipts_and_wrong_order_are_atomic(self):
        request = self.mission.request()
        self.assert_unchanged(lambda: self.mission.receive(receipt(request, 'COMPLETE', self.clock)))
        self.assert_unchanged(lambda: self.mission.receive(receipt(request, 'RUNNING', self.clock)))
        for stamp in (-1, 2, float('nan'), True):
            message = receipt(request, 'ACCEPTED', self.clock)
            message['observed_s'] = stamp
            self.assert_unchanged(lambda: self.mission.receive(message))

    def test_frame_camera_pose_plane_version_ring_color_and_age(self):
        request = self.transfer('PLACE')
        rule = phases(request['goal'], self.mission.task.state)[0]
        frames = self.frames(rule)
        for key, value in [('camera_id', 'old-camera'), ('pose_revision', 2),
                           ('image_size', [640, 480]), ('pose_id', 'old-pose'),
                           ('target_plane', 'wrong-plane'), ('pose_sha256', 'd' * 64),
                           ('camera_config_sha256', 'd' * 64)]:
            invalid = copy.deepcopy(frames)
            invalid[1]['context'][key] = value
            with self.subTest(key=key):
                self.assert_unchanged(lambda: self.mission.checkpoint(invalid))
        for key, value in [('ring', 1 if request['goal']['ring'] != 1 else 2), ('ring', True),
                           ('color', 5), ('frame_id', frames[0]['frame_id']), ('captured_s', -1),
                           ('domain', 'physical'), ('source_kind', 'historical_candidate'),
                           ('request_sha256', 'old'), ('pose_epoch', 0)]:
            invalid = copy.deepcopy(frames)
            invalid[1][key] = value
            self.assert_unchanged(lambda: self.mission.checkpoint(invalid))
        self.mission.checkpoint(frames)
        self.assertEqual(self.mission.task.state['counts']['places'], 0)

    def test_duplicate_checkpoint_and_wrong_color_at_source(self):
        request = self.transfer()
        rules = phases(request['goal'], self.mission.task.state)
        frames = self.frames(rules[0])
        self.mission.checkpoint(frames)
        state = self.mission.task.state
        self.mission.checkpoint(frames)
        self.assertEqual(state, self.mission.task.state)
        invalid = copy.deepcopy(frames)
        invalid[-1]['quality'] = .9
        self.assert_unchanged(lambda: self.mission.checkpoint(invalid))
        frames = self.frames(rules[1])
        frames[1]['color'] = 5
        self.assert_unchanged(lambda: self.mission.checkpoint(frames))

    def test_candidate_readiness_never_becomes_held_stored_or_released(self):
        for value in ({'local_target_ready': True}, {'ring_target_ready': True},
                      {'occupancy_observation_ready': True, 'same_color_target_observed': True},
                      {'stack_success': True}, {'items': []}):
            summary = candidate_summary(value)
            self.assertFalse(summary['confirmation_allowed'])
            self.assertEqual(summary['material_event'], 'UNKNOWN')
        request = self.transfer()
        frames = self.frames(phases(request['goal'], self.mission.task.state)[0])
        frames[0]['source_kind'] = 'camera_live'
        self.assert_unchanged(lambda: self.mission.checkpoint(frames))

    def test_recovery_requires_reconciliation_and_does_not_replay_requests(self):
        self.transfer()
        run_id = self.mission.task.run_id
        recovered = Task(self.ledger, run_id, 'new-owner', self.clock)
        recovered.recover()
        mission = OfflineMission(recovered, CORE, PARAMETERS)
        self.assertEqual(mission.task.state['pending']['status'], 'UNKNOWN')
        with self.assertRaises(TaskError):
            mission.request()
        self.ledger.audit(run_id, reduce)

    def test_parameter_drift_during_pending_action_blocks_query(self):
        self.running()
        changed = OfflineMission(self.mission.task, CORE, dict(revision=9, sha256='d' * 64))
        self.assert_unchanged(changed.pending_request)

    def test_json_boolean_cannot_replace_integer_in_request(self):
        request = self.mission.request()
        message = copy.deepcopy(receipt(request, 'ACCEPTED', self.clock))
        message['request']['goal']['batch'] = False  # SCAN的batch=0，Python中False==0。
        self.assert_unchanged(lambda: self.mission.receive(message))

    def test_old_completed_result_cannot_be_rescued_by_fresh_visual_checks(self):
        request = self.transfer()
        self.mission.receive(receipt(request, 'COMPLETE', self.clock))
        for rule in phases(request['goal'], self.mission.task.state):
            self.mission.checkpoint(self.frames(rule))
        self.assert_unchanged(self.mission.confirm)

    def test_unreleased_or_unstable_material_cannot_become_placement(self):
        request = self.transfer('PLACE')
        rules = phases(request['goal'], self.mission.task.state)
        for rule in rules[:-1]:
            self.mission.checkpoint(self.frames(rule))
        frames = self.frames(rules[-1])
        for key in ('claw_empty', 'placement_valid', 'stable'):
            invalid = copy.deepcopy(frames)
            invalid[1][key] = False
            self.assert_unchanged(lambda: self.mission.checkpoint(invalid))
        self.assertEqual(self.mission.task.state['counts']['places'], 0)

    def test_envelopes_survive_ledger_replay(self):
        request = self.transfer()
        self.finish(request)
        self.ledger.audit(self.mission.task.run_id, reduce)
        events = self.ledger.events(self.mission.task.run_id)
        self.assertTrue(any(e['payload'].get('replay_request') for e in events))
        self.assertTrue(any(e['payload'].get('replay_receipt') for e in events))
        self.assertTrue(any(e['payload'].get('replay_frames') for e in events))
        self.assertIsNone(self.mission.snapshot()['physical_counts'])


class EndToEndTests(unittest.TestCase):
    def test_adapter_import_does_not_load_device_network_or_camera_owners(self):
        code = '''
import sys
sys.path.insert(0, 'pi')
class BlockHardware:
    def find_spec(self, fullname, path=None, target=None):
        if fullname.startswith(('robot.device', 'robot.mech_core', 'robot.drive_core',
                                'robot.vision.camera', 'serial', 'socket', 'cv2')):
            raise AssertionError('设备依赖: ' + fullname)
sys.meta_path.insert(0, BlockHardware())
import robot.task_replay
'''
        subprocess.run([sys.executable, '-B', '-c', code], cwd=ROOT, check=True, timeout=10)

    def test_two_zones_six_colors_full_ledger_and_single_item_cycle(self):
        with tempfile.TemporaryDirectory() as directory:
            for zone, code in ((1, CODE), (2, '245+123+542+321')):
                runtime = Path(directory) / str(zone)
                report = run(runtime, code, zone)
                self.assertEqual(report['simulation']['status'], 'COMPLETE')
                self.assertEqual(report['simulation']['simulation_counts'], dict(grabs=12, places=12))
                self.assertEqual(report['single_item_locations'], ['tray:1',
                                 'rough:' + code.split('+')[1][0], 'tray:1',
                                 'stage:' + code.split('+')[1][0]])
                self.assertFalse(report['physical_acceptance'])
                self.assertIsNone(report['physical_counts'])
                self.assertEqual(json.loads((runtime / 'report.json').read_text(encoding='utf-8')), report)
                with self.assertRaises(ValueError):
                    run(runtime, code, zone)


if __name__ == '__main__':
    unittest.main()
