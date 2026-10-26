"""视觉采集、版本和几何验证；使用合成图像，不连接相机、串口或控制核心。"""

import copy
import hashlib
import json
from pathlib import Path
import sys
import tempfile
import unittest
from unittest.mock import patch

import cv2
import numpy as np
from fastapi import FastAPI
from fastapi.testclient import TestClient

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'pi'))
from robot.vision.workspace import Workspace, planar_fit, rectangle
from robot.web.vision import router
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


if __name__ == '__main__':
    unittest.main()
