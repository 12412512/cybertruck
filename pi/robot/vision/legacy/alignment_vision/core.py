"""Fresh numbered-ring offsets relative to an explicitly confirmed pixel reference."""
import copy
import hashlib
import json
import math
import statistics
import time
import uuid
from collections import deque
from numbers import Real


def fingerprint(profile):
    return hashlib.sha256(json.dumps(profile,sort_keys=True,allow_nan=False,separators=(',',':')).encode()).hexdigest()


def context(station,site_label,position_id,anchor_id=None):
    if station not in ('rough','storage') or not isinstance(site_label,str) or not 1<=len(site_label.strip())<=64:
        raise ValueError('CONTEXT_INVALID')
    if type(position_id) is not int or position_id not in (1,2,3):raise ValueError('CONTEXT_INVALID')
    if anchor_id is not None and (type(anchor_id) is not int or anchor_id not in (1,2,3) or anchor_id==position_id):
        raise ValueError('CONTEXT_INVALID')
    return dict(station=station,site_label=site_label.strip(),position_id=position_id,anchor_id=anchor_id)


def angle(a,b):return math.degrees(math.atan2(b[1]-a[1],b[0]-a[0]))
def wrapped(value):return (value+180)%360-180
def distance(a,b):return math.hypot(a[0]-b[0],a[1]-b[1])


class AlignmentObserver:
    def __init__(self,profile,mount_id):
        self.profile=copy.deepcopy(profile);self.profile_hash=fingerprint(profile);self.mount_id=mount_id
        self.run_id=uuid.uuid4().hex;self.reference=None;self.pending=None
        self.history=deque(maxlen=5);self.last_sequence=-1;self.last_capture=None;self.last_context=None

    def arm_reference(self,ctx,confirmed_actual_pose,now):
        context(**ctx)
        if confirmed_actual_pose is not True:raise ValueError('ACTUAL_ARRIVAL_POSE_CONFIRMATION_REQUIRED')
        self.reference=None;self.history.clear();self.pending=(copy.deepcopy(ctx),now)

    def load_reference(self,data,ctx,confirmed_mount):
        context(**ctx)
        if confirmed_mount is not True:raise ValueError('UNCHANGED_MOUNT_CONFIRMATION_REQUIRED')
        if data.get('schema_version')!=1 or data.get('profile_fingerprint')!=self.profile_hash or data.get('mount_id')!=self.mount_id:
            raise ValueError('REFERENCE_CAMERA_MISMATCH')
        if data.get('context')!=ctx or data.get('plane')!='empty_ring_plane' or data.get('actual_pose_confirmed') is not True:
            raise ValueError('REFERENCE_CONTEXT_MISMATCH')
        if data.get('metric_valid') is not False or data.get('vehicle_yaw_valid') is not False:
            raise ValueError('REFERENCE_UNIT_INVALID')
        ids=[ctx['position_id']]+([ctx['anchor_id']] if ctx['anchor_id'] else [])
        for i in ids:
            values=data.get('centers_px',{}).get(str(i))
            if not isinstance(values,list) or len(values)!=2 or any(type(v) not in (int,float) or not math.isfinite(v) for v in values):
                raise ValueError('REFERENCE_POINTS_INVALID')
            if not (0<=values[0]<1280 and 0<=values[1]<720):raise ValueError('REFERENCE_POINTS_INVALID')
        if ctx['anchor_id'] and distance(data['centers_px'][str(ids[0])],data['centers_px'][str(ids[1])])<40:
            raise ValueError('REFERENCE_POINTS_INVALID')
        self.reference=copy.deepcopy(data);self.history.clear();self.pending=None

    def process(self,candidates,sequence,captured,now,ctx,image_size):
        reasons=[];ids=[]
        try:
            context(**ctx);ids=[ctx['position_id']]+([ctx['anchor_id']] if ctx['anchor_id'] else [])
        except (TypeError,KeyError,ValueError):reasons.append('CONTEXT_INVALID')
        if ctx!=self.last_context:self.history.clear();self.last_context=copy.deepcopy(ctx)
        age=(now-captured)*1000
        if captured<=0 or not 0<=age<=250:reasons.append('CAMERA_STALE')
        if sequence<=self.last_sequence or (self.last_capture is not None and captured<=self.last_capture):reasons.append('DUPLICATE_FRAME')
        if self.last_capture is not None and captured-self.last_capture>.5:reasons.append('FRAME_GAP')
        self.last_sequence=max(self.last_sequence,sequence);self.last_capture=captured
        if image_size!=self.profile['image_size']:reasons.append('RESOLUTION_MISMATCH')
        current={}
        for i in ids:
            found=[r for r in candidates if r.get('position_id')==i]
            if len(found)!=1:reasons.append(f'ID_{i}_MISSING' if not found else f'ID_{i}_DUPLICATE');continue
            r=found[0]
            if r.get('frame_clipped') or r.get('roi_clipped'):reasons.append(f'ID_{i}_CLIPPED')
            if r.get('digit_similarity',0)<.58 or r.get('digit_margin',0)<.08:reasons.append(f'ID_{i}_UNCERTAIN')
            p=r.get('center_px')
            if not isinstance(p,list) or len(p)!=2 or any(not isinstance(v,Real) or isinstance(v,bool) or not math.isfinite(v) for v in p):
                reasons.append(f'ID_{i}_CENTER_INVALID');continue
            current[i]=[float(v) for v in p]
        if len(ids)==2 and len(current)==2 and distance(current[ids[0]],current[ids[1]])<40:reasons.append('DIRECTION_BASELINE_TOO_SHORT')
        if reasons:self.history.clear()
        else:self.history.append((captured,current))
        jitter=0.;speed=0.;span=0.
        if len(self.history)>1:
            span=(self.history[-1][0]-self.history[0][0])*1000
            for i in ids:
                points=[p[i] for _,p in self.history]
                median=[statistics.median(p[k] for p in points) for k in (0,1)]
                jitter=max(jitter,max(distance(p,median) for p in points))
                speed=max(speed,max(distance(points[n],points[n-1])/(self.history[n][0]-self.history[n-1][0]) for n in range(1,len(points))))
        stable=not reasons and len(self.history)>=5 and span>=500 and jitter<=3 and speed<=8
        if not stable:reasons.append('LIVE_TARGET_UNSTABLE')
        capture_error=None
        if self.pending:
            requested,started=self.pending
            if requested!=ctx or now-started>8:
                capture_error='REFERENCE_CAPTURE_CONTEXT_CHANGED' if requested!=ctx else 'REFERENCE_CAPTURE_TIMEOUT'
                self.pending=None
            elif stable:
                centers={str(i):[statistics.median(p[i][k] for _,p in self.history) for k in (0,1)] for i in ids}
                self.reference=dict(schema_version=1,id=uuid.uuid4().hex,created_at_unix=time.time(),
                    context=copy.deepcopy(ctx),profile_fingerprint=self.profile_hash,mount_id=self.mount_id,
                    image_size=list(image_size),plane='empty_ring_plane',centers_px=centers,
                    actual_pose_confirmed=True,metric_valid=False,vehicle_yaw_valid=False,
                    source='five_fresh_stable_numbered_ring_frames',samples=5,span_ms=round(span))
                self.pending=None
        offset=None;angular=None;spacing=None
        if self.reference is None:reasons.append('ARRIVAL_REFERENCE_REQUIRED')
        elif self.reference['context']!=ctx:reasons.append('REFERENCE_CONTEXT_MISMATCH')
        elif stable:
            target=current[ctx['position_id']];base=self.reference['centers_px'][str(ctx['position_id'])]
            offset=[round(target[k]-base[k],3) for k in (0,1)]
            if ctx['anchor_id']:
                other=current[ctx['anchor_id']];base_other=self.reference['centers_px'][str(ctx['anchor_id'])]
                angular=round(wrapped(angle(target,other)-angle(base,base_other)),4)
                spacing=round(distance(target,other)-distance(base,base_other),3)
        if capture_error:reasons.append(capture_error)
        valid=offset is not None and not reasons
        return dict(schema_version=1,run_id=self.run_id,frame_sequence=sequence,captured_monotonic=captured,
            observed_at_unix=time.time(),frame_age_ms=round(age,2),context=copy.deepcopy(ctx),
            candidates=candidates,current_centers_px={str(k):v for k,v in current.items()},
            observation_stable=stable,reference_id=self.reference['id'] if self.reference else None,
            reference_pending=self.pending is not None,reference_capture_error=capture_error,
            delta_image_px=offset if valid else None,delta_image_angle_deg=angular if valid else None,
            delta_pair_spacing_px=spacing if valid else None,offset_valid=valid,
            track=dict(samples=len(self.history),span_ms=round(span),jitter_px=round(jitter,3),max_speed_px_s=round(speed,3)),
            reasons=reasons,coordinate_unit='image_pixel',angle_unit='image_degree_clockwise',
            observation_source='fresh_numbered_ring_detection',metric_valid=False,vehicle_yaw_valid=False,
            pose_mm=None,correction_command=None,action_allowed=False,motion_enabled=False,uart_opened=False,
            stacking_ready=False)
