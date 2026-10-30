"""视觉采集、版本和几何验证；使用合成图像，不连接相机、串口或控制核心。"""

import copy
import base64
import hashlib
import io
import json
from pathlib import Path
import sys
import tempfile
from types import SimpleNamespace
import unittest
from unittest.mock import patch

import cv2
import numpy as np
from fastapi import FastAPI
from fastapi.testclient import TestClient

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'pi'))
from robot.vision.workspace import Workspace, canonical, planar_fit, rectangle
from robot.web.vision import router, LiveClient
from robot.vision.camera import Capture, create_app


class Device:
    def __init__(self):
        self.calls = []

    def request(self, path):
        self.calls.append(path)
        return dict(domain='physical', connected=True, firmware='TEST', boot_id=1, session_id=2,
                    state_age_ms=12, base=dict(stopped=True, fault=0), mechanism=dict(zero_valid=0),
                    token='NEVER_SAVE', owner='NEVER_SAVE',
                    poses=[dict(id='work_pose', revision=2, sha256='a'*64)], sequences=[])


class WorkspaceTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.now = 100.
        self.store = Workspace(self.temp.name, clock=lambda: self.now)
        self.image = np.full((480, 640, 3), (30, 30, 180), np.uint8)
        self.raw = cv2.imencode('.jpg', self.image)[1].tobytes()
        self.packet = dict(domain='historical_replay', camera='material', frame_id=17,
                           image_size=[640, 480], profile_sha256='b'*64, items=[],
                           metric_valid=False, motion_authority=False)
        self.key = self.store.remember(self.packet, self.raw)['observation_id']
        self.device = Device()
        app = FastAPI()
        app.include_router(router(self.store, self.device))
        self.client = TestClient(app)
        self.headers = {'X-Workbench': '1'}

    def tearDown(self):
        self.assertTrue(all(v == '/state' for v in self.device.calls))
        self.temp.cleanup()

    def post(self, path, value):
        return self.client.post('/api/vision/'+path, json=value, headers=self.headers)

    def sample(self, **extra):
        return dict(observation_id=self.key, name='空槽对照', label='empty', **extra)

    def test_same_frame_saved_with_source_and_no_success(self):
        result = self.post('samples', self.sample()).json()
        self.assertEqual(result['source_domain'], 'historical_replay')
        self.assertFalse(result['physical_validated'])
        self.assertFalse(result['success_count_changed'])
        self.assertNotIn('token', result['device'])
        self.assertNotIn('owner', result['device'])
        raw = self.client.get('/api/vision/samples/'+result['id']+'/image').content
        self.assertEqual(raw, self.raw)
        self.assertEqual(result['image_sha256'], hashlib.sha256(raw).hexdigest())
        self.assertEqual(result['snapshot_relation'], 'at_save_not_exposure')

    def test_save_never_fetches_another_image(self):
        with patch('robot.web.vision.urlopen', side_effect=AssertionError('no camera read')):
            self.assertEqual(self.post('samples', self.sample()).status_code, 200)

    def test_cross_origin_and_forged_image_rejected(self):
        self.assertEqual(self.client.post('/api/vision/samples', json=self.sample()).status_code, 403)
        self.assertEqual(self.client.post('/api/vision/samples', json=self.sample(),
                         headers={'X-Workbench':'1', 'Origin':'https://other.test'}).status_code, 403)
        self.assertEqual(self.post('samples', self.sample(image='forged')).status_code, 422)
        value = self.sample(); value['observation_id'] = '0'*32
        self.assertEqual(self.post('samples', value).status_code, 400)

    def test_immutable_versions_and_replay_stays_historical(self):
        first = self.post('samples', self.sample()).json()
        second = self.post('samples', self.sample()).json()
        self.assertEqual([first['revision'],second['revision']], [1,2])
        self.assertNotEqual(first['id'], second['id'])
        value = self.client.get('/api/vision/samples/'+first['id']+'/observation').json()
        self.assertEqual(value['domain'], 'saved_sample')
        self.assertEqual(value['original_domain'], 'historical_replay')
        self.assertEqual(value['frame_id'], 17)

    def test_unknown_binding_version_rejected(self):
        binding = dict(kind='pose',id='work_pose',revision=2,sha256='a'*64)
        self.assertEqual(self.post('samples', self.sample(binding=binding)).status_code, 200)
        binding['revision'] = 1
        self.assertEqual(self.post('samples', self.sample(binding=binding)).status_code, 400)

    def test_expiry_and_bounded_cache(self):
        self.now += 1201
        self.assertEqual(self.post('samples',self.sample()).status_code,400)
        for i in range(14):
            self.store.remember(dict(self.packet,frame_id=i),self.raw)
        self.assertEqual(len(self.store.frames),12)

    def stage_binding(self):
        from robot.device.mech_sequence import record
        sequence = record('观察测试', [
            dict(order=2, axis='arm', target_rad=1.0, speed_rad_s=1.0),
            dict(order=2, axis='gripper', target_rad=0.0, speed_rad_s=1.0),
            dict(order=7, axis='gripper', target_rad=1.0, speed_rad_s=1.0)], 1)
        original = self.device.request
        def state(path):
            result = original(path)
            result.update(connected=False, sequences=[sequence])
            return result
        self.device.request = state
        return dict(kind='sequence', **{k: sequence[k] for k in ('id','revision','sha256')})

    def test_observation_stage_preserves_group_and_inherited_pose_without_motion(self):
        binding = self.stage_binding()
        for order, angle in ((2,0.0),(7,1.0)):
            response = self.post('samples', self.sample(binding=binding, observation_order=order))
            self.assertEqual(response.status_code, 200)
            value = response.json()
            ref = value['observation_reference']
            self.assertEqual(ref['commanded_pose'], dict(arm=dict(target_rad=1.0),gripper=dict(target_rad=angle)))
            self.assertFalse(ref['actual_pose_verified'])
            self.assertFalse(ref['exposure_synchronized'])
            self.assertFalse(value['device']['connected'])
            self.assertFalse(value['physical_validated'])
            stored = self.client.get('/api/vision/records/samples/'+value['id']).json()
            self.assertEqual(stored['observation_reference'], ref)

    def test_observation_stage_invalid_or_missing_binding_rejected(self):
        binding = self.stage_binding()
        self.assertEqual(self.post('samples', self.sample(observation_order=2)).status_code,400)
        self.assertEqual(self.post('samples', self.sample(binding=binding,observation_order=3)).status_code,400)
        for order in (0,1000,1.5,True):
            self.assertEqual(self.post('samples',self.sample(binding=binding,observation_order=order)).status_code,422)

    def test_calibration_draft_retains_observation_stage(self):
        binding = self.stage_binding()
        result = self.post('drafts',dict(observation_id=self.key,name='观察区域',
            roi=[20,20,600,450],points=[],binding=binding,observation_order=7))
        self.assertEqual(result.status_code,200)
        self.assertEqual(result.json()['observation_reference']['order'],7)
        self.assertFalse(result.json()['applied'])

    def test_roi_only_draft_not_applied(self):
        value = dict(observation_id=self.key,name='原料ROI',roi=[20,20,600,450],points=[])
        result = self.post('drafts',value).json()
        self.assertIsNone(result['fit'])
        self.assertFalse(result['applied'])
        self.assertFalse(result['metric_valid'])
        value['roi'] = [20,20,700,450]
        self.assertEqual(self.post('drafts',value).status_code,400)

    def test_color_sampling_reuses_old_sampler(self):
        result = self.post('colors',dict(observation_id=self.key,roi=[10,10,40,40],color_id=1))
        self.assertEqual(result.status_code,200)
        self.assertEqual(result.json()['model']['sample_count'],900)
        self.assertFalse(result.json()['applied'])

    def test_corrupt_record_and_path_not_accepted(self):
        result = self.post('samples',self.sample()).json()
        path = Path(self.temp.name)/'samples'/(result['id']+'.json')
        data = json.loads(path.read_text(encoding='utf8'));data['label']='held'
        path.write_text(json.dumps(data),encoding='utf8')
        self.assertEqual(self.client.get('/api/vision/records/samples/'+result['id']).status_code,404)
        with self.assertRaises(ValueError):self.store.get('samples','../other')

    def test_changed_or_missing_image_cannot_be_read_or_replayed(self):
        other = cv2.imencode('.jpg', np.full_like(self.image, (180,30,30)))[1].tobytes()
        for replacement in (other, b'', b'x'*(4*1024*1024+1), None):
            with self.subTest(replacement_size=None if replacement is None else len(replacement)):
                saved = self.post('samples', self.sample()).json()
                path = Path(self.temp.name)/'samples'/(saved['id']+'.jpg')
                metadata = path.with_suffix('.json').read_bytes()
                if replacement is None:
                    path.unlink()
                else:
                    path.write_bytes(replacement)
                count = len(self.store.frames)
                for suffix in ('image', 'observation'):
                    response = self.client.get(f"/api/vision/samples/{saved['id']}/{suffix}")
                    self.assertEqual(response.status_code, 404)
                self.assertEqual(len(self.store.frames), count)
                self.assertEqual(path.with_suffix('.json').read_bytes(), metadata)

    def test_sample_without_image_digest_cannot_be_replayed(self):
        for digest in (None, 'x'*64, 'a'*64):
            with self.subTest(digest=digest):
                saved = self.post('samples', self.sample()).json()
                path = Path(self.temp.name)/'samples'/(saved['id']+'.json')
                saved.pop('sha256')
                if digest is None:
                    saved.pop('image_sha256')
                else:
                    saved['image_sha256'] = digest
                saved['sha256'] = hashlib.sha256(canonical(saved).encode()).hexdigest()
                path.write_text(canonical(saved), encoding='utf8')
                for suffix in ('image', 'observation'):
                    self.assertEqual(self.client.get(
                        f"/api/vision/samples/{saved['id']}/{suffix}").status_code, 404)

    def test_swapped_valid_metadata_rejected_by_record_identity(self):
        first = self.post('samples', self.sample()).json()
        second = self.post('samples', self.sample()).json()
        folder = Path(self.temp.name)/'samples'
        (folder/(first['id']+'.json')).write_bytes((folder/(second['id']+'.json')).read_bytes())
        self.assertEqual(self.client.get('/api/vision/records/samples/'+first['id']).status_code, 404)
        self.assertEqual(self.client.get('/api/vision/samples/'+first['id']+'/image').status_code, 404)

    def test_image_response_uses_checked_bytes_without_reopening_file(self):
        saved = self.post('samples', self.sample()).json()
        read = self.store.image
        def replace_after_read(key):
            result = read(key)
            (Path(self.temp.name)/'samples'/(key+'.jpg')).write_bytes(b'replaced after check')
            return result
        with patch.object(self.store, 'image', side_effect=replace_after_read):
            response = self.client.get('/api/vision/samples/'+saved['id']+'/image')
        self.assertEqual(response.status_code, 200)
        self.assertEqual(response.content, self.raw)
        self.assertEqual(response.headers['cache-control'], 'no-store')

    def test_sampling_saved_frame_stays_bound_to_verified_cached_image(self):
        saved = self.post('samples', self.sample()).json()
        replay = self.client.get('/api/vision/samples/'+saved['id']+'/observation').json()
        digest = hashlib.sha256(self.raw).hexdigest()
        self.assertEqual(replay['image_sha256'], digest)
        (Path(self.temp.name)/'samples'/(saved['id']+'.jpg')).write_bytes(b'replaced')
        result = self.post('colors', dict(observation_id=replay['observation_id'],
                                         roi=[10,10,40,40], color_id=1))
        self.assertEqual(result.status_code, 200)
        self.assertEqual(result.json()['source_image_sha256'], digest)
        self.assertEqual(result.json()['model']['sample_count'], 900)
        self.assertFalse(result.json()['applied'])
        self.assertEqual(self.client.get('/api/vision/samples/'+saved['id']+'/observation').status_code, 404)

    def test_frame_cache_rejects_mismatched_image_and_freezes_mutable_bytes(self):
        with self.assertRaises(ValueError):
            self.store.remember(dict(self.packet, image_sha256='a'*64), self.raw)
        raw = bytearray(self.raw)
        result = self.store.remember(self.packet, raw)
        raw[:] = b'x'*len(raw)
        observation, cached = self.store.frame(result['observation_id'])
        self.assertEqual(cached, self.raw)
        self.assertEqual(observation['image_sha256'], hashlib.sha256(cached).hexdigest())

    def proxy_observation(self, source_age, transfer=0., decode=0., hashing=0., copying=0.,
                          body_transform=None, **changes):
        packet = dict(self.packet, domain='camera_live', age_ms=source_age,
                      captured_s=987654321., stream_id='test-camera',
                      image='data:image/jpeg;base64,'+base64.b64encode(self.raw).decode())
        packet.update(changes)
        clock = [128.]
        class Response(io.BytesIO):
            def read(self, size=-1):
                clock[0] += transfer
                return super().read(size)
        body = json.dumps(packet)
        response = Response((body_transform(body) if body_transform else body).encode())
        def decode_image(*args, **kwargs):
            clock[0] += decode
            return base64.b64decode(*args, **kwargs)
        def hash_image(*args, **kwargs):
            clock[0] += hashing
            return hashlib.sha256(*args, **kwargs)
        def copy_metadata(value):
            clock[0] += copying
            return copy.deepcopy(value)
        with patch('robot.web.vision.urlopen', return_value=response), \
             patch('robot.web.vision.time', SimpleNamespace(monotonic=lambda: clock[0])), \
             patch('robot.web.vision.base64', SimpleNamespace(b64decode=decode_image)), \
             patch('robot.vision.workspace.hashlib', SimpleNamespace(sha256=hash_image)), \
             patch('robot.vision.workspace.copy', SimpleNamespace(deepcopy=copy_metadata)):
            return self.client.get('/api/vision/live/observation')

    def test_proxy_adds_local_transfer_and_decode_without_comparing_host_clocks(self):
        response = self.proxy_observation(50, transfer=.0625, decode=.0625)
        self.assertEqual(response.status_code, 200)
        value = response.json()
        self.assertEqual(value['source_age_ms'], 50)
        self.assertEqual(value['proxy_elapsed_ms'], 125)
        self.assertEqual(value['age_ms'], 175)
        self.assertEqual(value['captured_s'], 987654321.)
        self.assertEqual(value['age_scope'], 'source_age_plus_proxy_request')
        self.assertFalse(value['motion_authority'])
        self.assertEqual(self.store.frame(value['observation_id'])[0]['age_ms'], 175)

    def test_proxy_rejects_expired_transfer_or_decode_and_keeps_250ms_boundary(self):
        for age, transfer, decode in ((1, .375, 0), (200, 0, .0625), (250.001, 0, 0)):
            with self.subTest(age=age, transfer=transfer, decode=decode):
                count = len(self.store.frames)
                self.assertEqual(self.proxy_observation(age, transfer, decode).status_code, 503)
                self.assertEqual(len(self.store.frames), count)
        value = self.proxy_observation(125, .0625, .0625)
        self.assertEqual(value.status_code, 200)
        self.assertEqual(value.json()['age_ms'], 250)

    def test_proxy_subtracts_only_reported_service_interval_and_keeps_network_delay(self):
        response = self.proxy_observation(150, transfer=.1875, server_elapsed_ms=125)
        self.assertEqual(response.status_code,200)
        self.assertEqual(response.json()['age_ms'],212.5)
        self.assertEqual(self.proxy_observation(150,transfer=.375,server_elapsed_ms=125).status_code,503)
        for elapsed in (-1,True,200,float('nan')):
            self.assertEqual(self.proxy_observation(150,transfer=.1875,server_elapsed_ms=elapsed).status_code,503)

    def test_proxy_rejects_non_numeric_or_non_finite_age_and_bad_image(self):
        for age in (None, True, '1', -1, float('nan'), float('inf')):
            with self.subTest(age=age):
                self.assertEqual(self.proxy_observation(age).status_code, 503)
        self.assertEqual(self.proxy_observation(1, image=None).status_code, 503)

    def test_proxy_checks_final_age_after_cache_hash_and_metadata_copy(self):
        count = len(self.store.frames)
        for delay in (dict(hashing=.25), dict(copying=.25)):
            with self.subTest(delay=delay):
                self.assertEqual(self.proxy_observation(1, **delay).status_code, 503)
                self.assertEqual(len(self.store.frames), count)
        response = self.proxy_observation(100, hashing=.0625, copying=.0625)
        self.assertEqual(response.status_code, 200)
        result = response.json()
        cached, _ = self.store.frame(result['observation_id'])
        for key in ('source_age_ms', 'proxy_elapsed_ms', 'age_ms', 'age_scope'):
            self.assertEqual(cached[key], result[key])
        self.assertEqual(result['age_ms'], 225)

    def test_proxy_rejects_nested_non_finite_json_before_caching(self):
        count = len(self.store.frames)
        for number in (float('nan'), float('inf'), -float('inf')):
            with self.subTest(number=number):
                result = self.proxy_observation(10, items=[dict(confidence=number)])
                self.assertEqual(result.status_code, 503)
                self.assertEqual(len(self.store.frames), count)
        # 使用实际JSON指数值，而非mock解析结果，覆盖json.loads的浮点溢出。
        for number in ('1e999', '-1e999'):
            with self.subTest(number=number):
                result = self.proxy_observation(10, items=[dict(center=[10, 'OVERFLOW'])],
                    body_transform=lambda body: body.replace('"OVERFLOW"', number))
                self.assertEqual(result.status_code, 503)
                self.assertEqual(len(self.store.frames), count)

    def test_quota_refuses_and_retains_existing(self):
        folder = Path(self.temp.name)/'samples';folder.mkdir()
        for i in range(200):(folder/f'{i}.json').write_text('{}')
        result = self.post('samples',self.sample())
        self.assertEqual(result.status_code,400)
        self.assertEqual(len(list(folder.glob('*.json'))),200)

    def test_disk_failure_not_reported_as_saved(self):
        with patch.object(self.store,'_write',side_effect=OSError('disk full')):
            self.assertEqual(self.post('samples',self.sample()).status_code,503)

    def test_geometric_independent_point_not_used_for_fit(self):
        points=[dict(u=u,v=v,x_mm=.5*u+10,y_mm=.25*v-5,check=False)
                for u,v in [(10,10),(400,10),(400,400),(10,400),(200,200)]]
        points.append(dict(u=100,v=120,x_mm=67,y_mm=25,check=True))
        result = planar_fit(points)
        self.assertLess(result['fit_max_mm'],1e-8)
        self.assertAlmostEqual(result['check_max_mm'],7)
        self.assertFalse(result['metric_valid'])
        with self.assertRaises(ValueError):planar_fit(points[:-1])
        degenerate=[dict(u=i,v=i,x_mm=i,y_mm=i,check=i==5) for i in range(6)]
        with self.assertRaises(ValueError):planar_fit(degenerate)

    def test_encoding_delay_invalidates_frame(self):
        import time
        from robot.vision.observe import Detector
        client=TestClient(create_app())
        now=time.monotonic()
        def slow(*args):
            time.sleep(.27)
            return True,np.array([1,2,3],dtype=np.uint8)
        with patch.object(Capture,'read',return_value=(1,now,self.image)), \
             patch.object(Detector,'process',return_value=self.packet), patch('cv2.imencode',side_effect=slow):
            self.assertEqual(client.get('/observation').status_code,503)

    def test_ring_metadata_avoids_jpeg_and_exposes_camera_identity(self):
        import time
        from robot.vision.observe import Detector
        client=TestClient(create_app())
        with patch.object(Capture,'read',side_effect=lambda:(1,time.monotonic(),self.image)), \
             patch.object(Detector,'process',return_value=self.packet),patch('cv2.imencode',side_effect=AssertionError('metadata does not encode')):
            response=client.get('/observation?role=material&mode=ring&include_image=false')
            self.assertEqual(response.status_code,200)
            value=response.json();self.assertNotIn('image',value)
            self.assertTrue(value['device_id']);self.assertEqual(value['capture_epoch'],0)

    def test_pooled_transport_only_reads_and_never_retries_failure(self):
        import httpx
        calls = []
        def handler(request):
            calls.append(request)
            raise httpx.ReadTimeout('test',request=request)
        live = LiveClient()
        live.http.close()
        live.http = httpx.Client(transport=httpx.MockTransport(handler))
        try:
            with self.assertRaises(OSError): live.read('/observation?role=material')
            self.assertEqual(len(calls),1)
            self.assertEqual(calls[0].method,'GET')
            with self.assertRaises(ValueError): live.read('/command')
            self.assertEqual(len(calls),1)
        finally: live.close()

    def test_pooled_transport_retains_response_size_limit(self):
        import httpx
        live = LiveClient()
        live.http.close()
        live.http = httpx.Client(transport=httpx.MockTransport(
            lambda request: httpx.Response(200,content=b'x'*(6*1024*1024+1))))
        try:
            with self.assertRaises(ValueError): live.read('/frame')
        finally: live.close()

    def test_preview_does_not_wait_for_slow_recognition(self):
        import time, threading
        from concurrent.futures import ThreadPoolExecutor
        from robot.vision.observe import Detector
        entered, release = threading.Event(), threading.Event()
        def process(frame, mode):
            if mode == 'material':
                entered.set(); release.wait(2)
            return dict(self.packet,recognition_performed=mode!='preview')
        with patch.object(Capture,'read',side_effect=lambda:(1,time.monotonic(),self.image)), \
             patch.object(Detector,'process',side_effect=process):
            client=TestClient(create_app())
            with ThreadPoolExecutor(max_workers=2) as pool:
                job=pool.submit(client.get,'/observation?mode=material')
                try:
                    self.assertTrue(entered.wait(1))
                    result=client.get('/observation?mode=preview')
                    self.assertEqual(result.status_code,200)
                    self.assertFalse(result.json()['recognition_performed'])
                    self.assertFalse(release.is_set())
                finally: release.set()
                job.result()

    def test_capture_retries_same_device_until_explicit_stop(self):
        capture=Capture('/same-camera',[1280,720])
        calls=[]
        def once():
            calls.append(capture.device)
            if len(calls)==2:capture.stop.set()
        with patch.object(capture,'_run_once',side_effect=once), \
             patch.object(capture.stop,'wait',side_effect=lambda timeout:capture.stop.is_set()):
            capture.run()
        self.assertEqual(calls,['/same-camera','/same-camera'])
        self.assertEqual(capture.reconnects,1)


if __name__ == '__main__':
    unittest.main()
