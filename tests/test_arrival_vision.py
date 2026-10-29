import unittest
from robot.vision.arrival import reviewed_preview
from robot.vision.legacy.alignment_vision.core import fingerprint


class ArrivalVisionTests(unittest.TestCase):
    @staticmethod
    def preview(o,d,p,binding='a'*64):
        return reviewed_preview(o,d,p,binding,10,barrier=8,stop_capture_floor=9.3)

    def data(self):
        profile=dict(image_size=[1280,720])
        data=dict(schema_version=1,station='rough',target=2,experimental_reviewed=True,ruler_checked=True,
            repeatability_checked=True,profile_fingerprint=fingerprint(profile),
            scope='fixed_heading_empty_ring_experimental_return',inverse_mm_per_px=[[.5,0],[0,.5]],
            deadband_px=3,min_step_mm=1,max_step_mm=5,gain=.5,calibration_id=1,
            observation_binding_sha256='a'*64)
        observation=dict(position_id=2,stable=True,samples=5,span_ms=600,frame_age_ms=100,
            barrier=8,oldest_capture=9.4,delta_px=[8,0],
            profile_fingerprint=fingerprint(profile),observation_binding_sha256='a'*64)
        return observation,data,profile

    def test_reviewed_reference_gives_direction_without_authority(self):
        o,d,p=self.data();v=self.preview(o,d,p)
        self.assertEqual(v['command_mm'],[-2,0]);self.assertFalse(v['motion_authority'])

    def test_old_pose_or_post_stop_frame_and_unreviewed_matrix_refused(self):
        for key,value in [('frame_age_ms',251),('position_id',1),('oldest_capture',9.2),('barrier',7),('stable',False)]:
            o,d,p=self.data();o[key]=value
            with self.subTest(key=key),self.assertRaises(ValueError):self.preview(o,d,p)
        o,d,p=self.data();d['ruler_checked']=False
        with self.assertRaises(ValueError):self.preview(o,d,p)
        o,d,p=self.data()
        with self.assertRaises(ValueError):self.preview(o,d,p,'b'*64)

    def test_visual_error_steps_feed_same_arrival_executor_offline(self):
        import struct
        from robot.device.arrival import Arrival
        from test_arrival import link
        from robot.device.execution import SPEC_DATA
        a=Arrival();l=link();a.phase='READY';a.identity=(123,456,1)
        o,d,p=self.data();steps=[]
        # 合成局部图像模型：实际矩阵需要实车测量，不作物理证据。
        while True:
            result=self.preview(o,d,p)
            if result['state']=='DONE':break
            self.assertEqual(result['state'],'STEP')
            x,y=result['command_mm'];axis='x' if x else 'y';mm=x or y
            a.move(l,axis,mm,0);wire=l.commands.popleft();self.assertEqual(wire[0],34)
            spec=SPEC_DATA.unpack(wire[1][8:]);steps.append(mm)
            l.results.append(dict(request_id=wire[2],status='ACCEPTED',result=0))
            a.advance(l,0);l.execution.update(action_id=a.action_id,action_state=4);l.base['mode']=3
            a.advance(l,0);self.assertEqual(a.phase,'READY')
            o['delta_px'][0]+=spec[1]*2000;o['delta_px'][1]+=spec[2]*2000
            self.assertLess(len(steps),5)
        self.assertEqual(steps,[-2,-1]);self.assertLessEqual(abs(o['delta_px'][0]),3)

    def test_changed_profile_and_old_barrier_never_gain_step(self):
        for key,value in [('profile_fingerprint','b'*64),('observation_binding_sha256','b'*64),('barrier',9)]:
            o,d,p=self.data();o[key]=value
            with self.subTest(key=key),self.assertRaises(ValueError):self.preview(o,d,p)
