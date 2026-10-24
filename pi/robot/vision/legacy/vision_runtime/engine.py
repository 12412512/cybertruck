"""Frame-in / observation-out adapter for the unchanged tested vision cores.

The caller owns camera capture and serial transport. No VideoCapture, GUI,
profile writes or subprocess startup occur here. Each prepare resets histories.
"""
import copy
import json
import time

import numpy as np

from ..alignment_vision.core import AlignmentObserver,context
from ..material_vision.core import MaterialDetector,MaterialObserver
from ..ring_vision.core import RingDetector,RingObserver
from ..placement_vision.core import OccupancyObserver,SceneReference,cleaned,material_detector
from ..qr_camera.task_codec import parse_task


def json_value(value):
    if isinstance(value,np.ndarray):return value.tolist()
    if isinstance(value,np.generic):return value.item()
    raise TypeError('Unsupported observation value: '+type(value).__name__)


class FrameEngine:
    def __init__(self,material_profile,ring_profile,station_color_profile,mount_id):
        self.material_profile=copy.deepcopy(material_profile)
        self.ring_profile=copy.deepcopy(ring_profile)
        self.station_colors=copy.deepcopy(station_color_profile);self.mount_id=mount_id
        self.observer=None;self.mode=None;self.task=None;self.started=False

    def prepare(self,mode,task,station,batch,order,now,reference=None,empty_reference=None,reference_approval=None):
        if mode not in ('material_local','ring_number','occupancy','stack_candidate','alignment_pixel'):
            raise ValueError('NO_FRAME_ENGINE_FOR_DEFERRED_MODE')
        parse_task(task['raw'])
        if type(batch) is not int or batch not in (1,2) or type(order) is not int or order not in (1,2,3):
            raise ValueError('FRAME_CONTEXT_INVALID')
        if ((mode=='material_local' and station!='raw') or
            (mode!='material_local' and station not in ('rough','storage')) or
            (mode=='stack_candidate' and (station!='storage' or batch!=2))):raise ValueError('FRAME_CONTEXT_INVALID')
        self.mode=mode;self.task=copy.deepcopy(task);self.station=station;self.batch=batch;self.order=order;self.started=False
        if mode=='material_local':
            self.detector=MaterialDetector(copy.deepcopy(self.material_profile))
            self.observer=MaterialObserver(copy.deepcopy(self.material_profile),scope='local_target')
        elif mode=='ring_number':
            self.detector=RingDetector(copy.deepcopy(self.ring_profile));self.observer=RingObserver(copy.deepcopy(self.ring_profile))
        elif mode=='alignment_pixel':
            item=task['batch'+str(batch)][order-1];position=item['rough_processing_ring' if station=='rough' else 'temporary_storage_ring']
            self.detector=RingDetector(copy.deepcopy(self.ring_profile))
            self.observer=AlignmentObserver(copy.deepcopy(self.ring_profile),self.mount_id)
            # No reference is collected or implicitly approved in this adapter.
            self.ctx=copy.deepcopy(reference['context']) if reference else context(station,station+'_zone',position)
            if self.ctx['station']!=station or self.ctx['position_id']!=position:raise ValueError('REFERENCE_TARGET_MISMATCH')
            if reference:
                approval=reference_approval or {}
                if (approval.get('reference_id')!=reference['id'] or approval.get('operator_work_pose_confirmed') is not True or
                    approval.get('approved_for_pixel_observation') is not True):raise ValueError('ACTUAL_WORK_POSE_REFERENCE_UNAPPROVED')
                self.observer.load_reference(reference,self.ctx,True)
        else:
            self.detector=material_detector(copy.deepcopy(self.station_colors),self.ring_profile['roi'])
            self.observer=OccupancyObserver(copy.deepcopy(self.ring_profile['roi']))
            if empty_reference is not None:
                if not isinstance(empty_reference,SceneReference):raise ValueError('FRESH_SESSION_SCENE_REFERENCE_REQUIRED')
                self.observer.set_reference(empty_reference)
        return self.observer.run_id

    def process(self,frame,sequence,captured,now):
        if not self.observer:raise ValueError('FRAME_REQUEST_NOT_PREPARED')
        started=time.monotonic()
        size=list(frame.shape[:2][::-1]) if frame is not None else None
        fresh=frame is not None and 0<=now-captured<=.25
        candidates=self.detector.detect(frame) if fresh else []
        now+=time.monotonic()-started
        if self.mode=='material_local':
            result=self.observer.process(candidates,sequence,captured,now,self.task,self.batch,self.order-1,
                size,self.detector.uncertain_regions if fresh else [])
        elif self.mode=='ring_number':
            result=self.observer.process(candidates,sequence,captured,now,self.task,self.station,self.batch,self.order,size)
        elif self.mode=='alignment_pixel':
            result=self.observer.process(candidates,sequence,captured,now,self.ctx,size)
        else:
            candidates=cleaned(candidates);uncertain=self.detector.uncertain_regions if fresh else []
            evidence=self.observer.reference.evidence(frame,candidates,uncertain) if fresh and self.observer.reference else {}
            result=self.observer.process(candidates,uncertain,evidence,sequence,captured,now,self.task,
                                         self.station,self.batch,self.order,size)
        # Original observers clear request state on the first task/context frame.
        # Feed that actual frame once, then explicitly start the request for the
        # next new frame; no duplicate/fabricated frame is used to seed stability.
        if not self.started and self.mode in ('material_local','ring_number'):
            self.observer.begin(now);self.started=True
        # The tested cores legitimately use NumPy numeric scalars. Normalize at
        # this new JSON boundary instead of changing their geometry/classifiers.
        return json.loads(json.dumps(result,default=json_value,allow_nan=False))
