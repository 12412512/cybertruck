"""只读准备、版本绑定、单次09、保存回读与停止；所有设备均为内存夹具。"""
import json
from pathlib import Path
import struct
import tempfile
import unittest

from fastapi.testclient import TestClient
from fastapi import FastAPI, HTTPException
from test_lidar_health import event, saved_data, ready
from test_lidar_scan import board
from robot.lidar.protocol import cobs_decode
from robot.lidar.calibrate import Calibration
from robot.lidar_core import LidarCore, CalibrationPrepare, CalibrationCommit, create_app
from robot.web.workbench import lidar_router


class Node:
    def __init__(self, now, zone=1, case='good'):
        self.now, self.zone, self.case = now, zone, case
        self.saved = bytearray(saved_data()); struct.pack_into('<H', self.saved, 0, 11)
        self.rx = bytearray(); self.commands = []; self.reads = 0

    def write(self, wire):
        raw = cobs_decode(wire[:-1]); kind = raw[4]; self.commands.append(kind)
        if kind == 1:
            x, y, yaw = struct.unpack_from('<hhH', self.saved, 11 if self.zone == 1 else 18)
            if self.case == 'new_hello' and 9 in self.commands: x += 1
            self.rx.extend(event(0x81, struct.pack('<BBBBhhHBH', 1, 37, 6, self.zone, x, y, yaw, 0, 120)))
            self.rx.extend(event(0x82, b'\1\0'))
        elif kind == 0x13:
            self.reads += 1
            if self.case == 'changed_before_write' and self.reads == 2:
                struct.pack_into('<I', self.saved, 2, 79)
            self.rx.extend(event(0x8E, bytes(self.saved))+event(0x82, b'\x13\0'))
        elif kind == 9:
            assert raw[9:-2] == bytes([self.zone])
            if self.case != 'lost_ack': self.rx.extend(event(0x82, b'\11\0'))
            self.rx.extend(event(0x83, b'\5\3\1\1\0'))
            if self.case in ('missing_result', 'endless_progress', 'lost_ack'): return len(wire)
            if self.case == 'save_error':
                self.rx.extend(event(0x8A, struct.pack('<BH', 6, 1))); return len(wire)
            result_zone = 3-self.zone if self.case == 'wrong_zone' else self.zone
            payload = struct.pack('<BhhHhhhBB', result_zone, 160, 2100, 90*64, 10, -150, 64, 4, 5)
            if self.case != 'stale_saved':
                struct.pack_into('<I', self.saved, 2, 78); self.saved[6] = self.zone
                struct.pack_into('<hhH', self.saved, 11 if self.zone == 1 else 18, 160, 2100, 90*64)
            if self.case == 'other_zone':
                self.saved[18 if self.zone == 1 else 11] += 1
            packets = event(0x8E, bytes(self.saved))+event(0x88, payload)+event(0x83, b'\4\4\0\x0e\0')
            if self.case == 'crc': packets = packets[:-2]+b'\xff\0'
            self.rx.extend(packets)
        elif kind == 4:
            self.rx.extend(event(0x82, b'\4\0'))
            if self.case != 'missing_stop': self.rx.extend(event(0x83, b'\3\4\0\x0e\0'))
        else:
            assert kind == 2
        return len(wire)

    def read(self, count):
        if self.case == 'endless_progress' and 9 in self.commands and 4 not in self.commands:
            self.rx.extend(event(0x83, b'\5\3\1\x0e\0'))
        value = bytes(self.rx); self.rx.clear(); return value


class CalibrationTests(unittest.TestCase):
    def prepare(self, case='good', write=False, fault=False, zone=1):
        temp = tempfile.TemporaryDirectory(); self.addCleanup(temp.cleanup)
        now = [.1]; core = LidarCore(clock=lambda:now[0], zone=zone, runtime=temp.name,
            calibration_prepare=True, calibration_write=write)
        core.health = ready(); state = board()
        if fault: state['base'].update(mode=6, fault=2048)
        core.board(state, now[0]); node = Node(now, zone, case)
        accepted = core.prepare_calibration(CalibrationPrepare(request_id='prepare1', zone=zone,
            placement_id='zone1_manual', manual_fixed_deployed=True))
        for _ in range(50):
            core.board(state, now[0]); core.tick(node); now[0] += .01
            if core.job and core.job.phase == 'PREPARED': break
        self.assertEqual(core.job.phase, 'PREPARED')
        self.assertTrue(core.backup_saved)
        return core, node, now, accepted

    def request(self, core, identity='commit1'):
        return CalibrationCommit(request_id=identity, calibration_id=core.job.id,
            backup_version=core.job.backup_version, placement_id='zone1_manual', confirm_save=True)

    def finish(self, core, node, now, fault=False):
        for _ in range(1200):
            state = board()
            if fault and core.job and core.job.write_issued: state['base'].update(mode=6, fault=2048)
            core.board(state, now[0]); core.tick(node); now[0] += .01
            if core.job is None: return core.last_calibration
        self.fail('calibration did not terminate')

    def test_prepare_under_fault_is_readonly_and_default_write_closed(self):
        core, node, now, _ = self.prepare(fault=True)
        self.assertLessEqual(set(node.commands), {1, 2, 0x13})
        with self.assertRaisesRegex(ValueError, '写入关闭'): core.commit_calibration(self.request(core))
        self.assertFalse(core.job.write_issued)
        self.assertEqual(core.witness['base']['fault'], 2048)

    def test_both_zones_recheck_once_write_and_full_stop(self):
        for zone in (1, 2):
            core, node, now, _ = self.prepare(write=True, zone=zone)
            request = self.request(core)
            self.assertFalse(core.commit_calibration(request)['repeated'])
            self.assertTrue(core.commit_calibration(request)['repeated'])
            result = self.finish(core, node, now)
            self.assertEqual(result['status'], 'COMPLETE', result['errors'])
            self.assertTrue(result['stop_confirmed']); self.assertEqual(node.commands.count(9), 1)
            self.assertEqual(node.reads, 2); self.assertFalse(result['real_map_registered'])
            self.assertEqual(set(node.commands), {1, 2, 4, 9, 0x13})

    def test_changed_backup_cancels_before_write(self):
        core, node, now, _ = self.prepare('changed_before_write', write=True)
        core.commit_calibration(self.request(core)); result = self.finish(core, node, now)
        self.assertEqual(result['status'], 'FAILED'); self.assertNotIn(9, node.commands)

    def test_ack_only_save_error_bad_zone_stale_save_crc_and_stop_do_not_pass(self):
        for case in ('missing_result', 'lost_ack', 'save_error', 'wrong_zone', 'stale_saved',
                'other_zone', 'crc', 'missing_stop', 'new_hello', 'endless_progress'):
            with self.subTest(case=case):
                core, node, now, _ = self.prepare(case, write=True)
                core.commit_calibration(self.request(core)); result = self.finish(core, node, now)
                self.assertEqual(result['status'], 'FAILED')
                self.assertEqual(node.commands.count(9), 1)
                self.assertTrue(result['write_may_have_occurred'])
                if case == 'missing_stop': self.assertFalse(result['stop_confirmed'])

    def test_fault_before_or_during_write_never_clears_or_retries(self):
        core, node, now, _ = self.prepare(write=True, fault=True)
        with self.assertRaisesRegex(ValueError, '无故障停车'): core.commit_calibration(self.request(core))
        self.assertNotIn(9, node.commands)
        core, node, now, _ = self.prepare(write=True)
        core.commit_calibration(self.request(core)); result = self.finish(core, node, now, fault=True)
        self.assertEqual(result['status'], 'FAILED'); self.assertEqual(node.commands.count(9), 1)

    def test_binding_expiry_disk_backup_and_identity_rejected(self):
        for case in ('placement', 'version', 'identity', 'disk', 'confirm', 'expiry'):
            core, node, now, _ = self.prepare(write=True); request = self.request(core)
            if case == 'placement': request.placement_id = 'moved'
            if case == 'version': request.backup_version = 'a'*64
            if case == 'identity': core.witness['boot_id'] = 'reset'
            if case == 'confirm': request.confirm_save = False
            if case == 'expiry': now[0] += 61; core.board(board(), now[0])
            if case == 'disk':
                path = core.runtime/'calibrations'/(core.job.id+'.backup.json')
                value = json.loads(path.read_text()); value['persisted']['record_sequence'] += 1
                path.write_text(json.dumps(value))
            with self.subTest(case=case), self.assertRaises(ValueError): core.commit_calibration(request)
            self.assertNotIn(9, node.commands)

    def test_default_api_no_prepare_and_explicit_api_header_and_validation(self):
        with TestClient(create_app(LidarCore(zone=1))) as web:
            self.assertEqual(web.post('/calibration/prepare', json={}).status_code, 404)
        core, _, _, accepted = self.prepare()
        body = dict(request_id='prepare1', zone=1, placement_id='zone1_manual', manual_fixed_deployed=True)
        with TestClient(create_app(core)) as web:
            self.assertEqual(web.post('/calibration/prepare', json=body).status_code, 403)
            response = web.post('/calibration/prepare', json=body, headers={'X-Lidar-Workbench':'1'})
            self.assertEqual(response.json()['id'], accepted['id']); self.assertTrue(response.json()['repeated'])
            self.assertEqual(web.post('/calibration/commit', json=self.request(core).model_dump(),
                headers={'X-Lidar-Workbench':'1'}).status_code, 409)

    def test_old_stop_and_calibration_events_are_not_new_evidence(self):
        job = Calibration(1, 1, 0)
        job.feed(event(0x88, bytes(15))+event(0x82, b'\4\0')+event(0x83, b'\3\4\0\0\0'), 0)
        self.assertIsNone(job.result); self.assertFalse(job.stop_status)
        job.abort('cancel', .1); job.step(.1); job.step(.6)
        self.assertEqual(job.phase, 'FAILED'); self.assertFalse(job.view()['stop_confirmed'])

    def test_prepared_polls_without_writing_and_expires_without_operator(self):
        core, node, now, _ = self.prepare(fault=True)
        for _ in range(6200):
            state = board(); state['base'].update(mode=6, fault=2048)
            core.board(state, now[0]); core.tick(node); now[0] += .01
            if core.job is None: break
        self.assertEqual(core.last_calibration['status'], 'CANCELLED')
        self.assertTrue(core.last_calibration['stop_confirmed'])
        self.assertNotIn(9, node.commands); self.assertNotIn(3, node.commands)
        self.assertGreater(node.commands.count(1), 20)

    def test_operator_cancel_is_distinct_from_fault_and_never_writes(self):
        core, node, now, _ = self.prepare()
        core.job.cancel(now[0]); result = self.finish(core, node, now)
        self.assertEqual(result['status'], 'CANCELLED')
        self.assertTrue(result['stop_confirmed']); self.assertNotIn(9, node.commands)

    def test_gateway_same_origin_validated_and_core_write_still_closed(self):
        core, _, _, _ = self.prepare()
        with TestClient(create_app(core)) as owner:
            class Client:
                def request(self, path, body=None):
                    response = (owner.get(path) if body is None else
                        owner.post(path, json=body, headers={'X-Lidar-Workbench':'1'}))
                    if response.status_code >= 400:
                        raise HTTPException(response.status_code, response.json()['detail'])
                    return response.json()
            app = FastAPI(); app.include_router(lidar_router(Client()))
            with TestClient(app) as web:
                self.assertTrue(web.get('/api/lidar/calibration/capabilities').json()['gateway'])
                body = dict(request_id='prepare1', zone=1, placement_id='zone1_manual', manual_fixed_deployed=True)
                path = '/api/lidar/calibration/prepare'
                self.assertEqual(web.post(path, json=body).status_code, 403)
                self.assertEqual(web.post(path, json=body, headers={'X-Workbench':'1', 'Origin':'https://other.example'}).status_code, 403)
                self.assertEqual(web.post(path, json=body, headers={'X-Workbench':'1'}).status_code, 200)
                self.assertEqual(web.post('/api/lidar/calibration/commit', json=self.request(core).model_dump(),
                    headers={'X-Workbench':'1'}).status_code, 409)


if __name__ == '__main__':
    unittest.main()
