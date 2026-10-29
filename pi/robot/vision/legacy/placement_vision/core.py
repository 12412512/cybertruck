"""Session-bound printed-position map and material observations in camera pixels."""
import copy
import math
from collections import deque
import time
import uuid

import cv2
import numpy as np
from ..qr_camera.task_codec import parse_task
from ..material_vision.core import MaterialDetector, contains, may_overlap


def task_signature(task):
    if not isinstance(task,dict) or task.get('valid') is not True:raise ValueError('TASK_INVALID')
    parsed=parse_task(task['raw'])
    return (parsed['raw'],task.get('accepted_at_unix'))


def target_for(task,station,batch,order):
    if station not in ('rough','storage') or type(batch) is not int or batch not in (1,2) or type(order) is not int or order not in (1,2,3):
        raise ValueError('CONTEXT_INVALID')
    task_signature(task);parsed=parse_task(task['raw']);item=parsed[f'batch{batch}'][order-1]
    return dict(color_id=item['color_id'],position_id=item['rough_processing_ring' if station=='rough' else 'temporary_storage_ring'])


def material_detector(profile,roi):
    # Copy the validated color/shape models; never mutate or save the old profile.
    p=copy.deepcopy(profile);p['plate_roi']=list(roi);p['plate_roi_shape']='rectangle';p['grasp_roi']=None
    return MaterialDetector(p)


def cleaned(items):
    return [{k:v for k,v in c.items() if k!='contour'} for c in items]


def overlap_ring(item,ring):
    x,y,w,h=item['bbox_px'];cx,cy=ring['center_px'];box=ring['bbox_px']
    # Dark linework that has not passed material-shape verification must cover
    # the ring centre before it can be assigned as a possible occupant.
    # A complete black identity still uses the same association as other colors.
    if item.get('color_id')==5 and not item.get('shape_consistent'):
        return x<=cx<=x+w and y<=cy<=y+h
    rx=(box[2]-box[0])*.22;ry=(box[3]-box[1])*.22
    return x<=cx+rx and x+w>=cx-rx and y<=cy+ry and y+h>=cy-ry


def small_gray(frame):
    return cv2.cvtColor(cv2.resize(frame,(640,360)),cv2.COLOR_BGR2GRAY)


def edge_map(gray):return cv2.Canny(cv2.GaussianBlur(gray,(3,3),0),40,100)


class SceneReference:
    """Valid only for this process, exact task session, camera and stationary board."""
    def __init__(self,frame,rings,task):
        task_signature(task)
        if frame.shape[:2]!=(720,1280):raise ValueError('RESOLUTION_MISMATCH')
        all_ids={i:[r for r in rings if r.get('position_id')==i] for i in (1,2,3)}
        if any(len(v)>1 for v in all_ids.values()):raise ValueError('DUPLICATE_EMPTY_RING_ID')
        by_id={}
        for position,values in all_ids.items():
            if not values:continue
            r=values[0]
            if r['frame_clipped'] or r['roi_clipped'] or r['digit_similarity']<.58 or r['digit_margin']<.08:
                continue
            by_id[position]=values
        if not by_id:raise ValueError('CLEAR_LOCAL_EMPTY_RING_REQUIRED')
        self.rings={i:copy.deepcopy(v[0]) for i,v in by_id.items()}
        self.signature=task_signature(task);self.gray=small_gray(frame);self.edges=edge_map(self.gray)
        self.id=uuid.uuid4().hex;self.created_at=time.time()

    def evidence(self,frame,materials,uncertain):
        gray=small_gray(frame);edges=edge_map(gray)
        distance=cv2.distanceTransform(255-edges,cv2.DIST_L2,3)
        yy,xx=np.indices(gray.shape);excluded=np.zeros(gray.shape,np.uint8)
        for item in materials+uncertain:
            x,y,w,h=item['bbox_px'];x1=max(0,int(x/2)-3);y1=max(0,int(y/2)-3)
            x2=min(640,int((x+w)/2)+4);y2=min(360,int((y+h)/2)+4)
            excluded[y1:y2,x1:x2]=1
        out={}
        for position,ring in self.rings.items():
            cx,cy=np.array(ring['center_px'])/2;box=np.array(ring['bbox_px'])/2
            rx=(box[2]-box[0])/2;ry=(box[3]-box[1])/2
            rho=np.sqrt(((xx-cx)/rx)**2+((yy-cy)/ry)**2)
            points=(self.edges>0)&(rho>=.62)&(rho<=1.08)&(excluded==0)
            count=int(np.count_nonzero(points))
            angles=np.arctan2((yy[points]-cy)/ry,(xx[points]-cx)/rx)
            bins=np.floor((angles+math.pi)/(2*math.pi)*12).astype(int)%12
            sectors=int(np.count_nonzero(np.bincount(bins,minlength=12)>=3))
            score=float(np.mean(distance[points]<=1.5)) if count else 0.
            # A paper cover cannot become EMPTY merely because no material color was found.
            x1,x2=max(0,int(cx-.53*rx)),min(640,int(cx+.53*rx)+1)
            y1,y2=max(0,int(cy-.53*ry)),min(360,int(cy+.53*ry)+1)
            baseline=self.gray[y1:y2,x1:x2].astype(float);current=gray[y1:y2,x1:x2].astype(float)
            delta=current-baseline;offset=float(np.median(delta))
            changed=float(np.mean(np.abs(delta-offset)>25))
            out[position]=dict(live_annulus_verified=count>=80 and sectors>=5 and score>=.8,
                visible_reference_edge_pixels=count,visible_angular_sectors=sectors,
                annulus_match=round(score,3),center_change_fraction=round(changed,3),
                empty_print_matches=changed<=.04)
        return out


class OccupancyObserver:
    def __init__(self,roi):
        self.roi=roi;self.run_id=uuid.uuid4().hex;self.reference=None
        self.tracks={i:deque(maxlen=5) for i in (1,2,3)}
        self.last_sequence=-1;self.last_capture=None;self.context=None

    def set_reference(self,reference):
        self.reference=reference
        for track in self.tracks.values():track.clear()

    def process(self,materials,uncertain,evidence,sequence,captured,now,task,station='storage',batch=2,order=1,image_size=None):
        reasons=[];expected=None;signature=None
        try:signature=task_signature(task);expected=target_for(task,station,batch,order)
        except (ValueError,KeyError,TypeError) as exc:reasons.append(str(exc))
        context=(signature,station,batch,order)
        if context!=self.context:
            self.context=context
            for track in self.tracks.values():track.clear()
        age=(now-captured)*1000
        if captured<=0 or not 0<=age<=250:reasons.append('CAMERA_STALE')
        if sequence<=self.last_sequence or (self.last_capture is not None and captured<=self.last_capture):reasons.append('DUPLICATE_FRAME')
        if self.last_capture is not None and captured-self.last_capture>.5:reasons.append('FRAME_GAP')
        self.last_sequence=max(sequence,self.last_sequence);self.last_capture=captured
        if image_size!=[1280,720]:reasons.append('RESOLUTION_MISMATCH')
        if self.reference is None:reasons.append('EMPTY_REFERENCE_REQUIRED')
        elif signature!=self.reference.signature:
            self.reference=None;reasons.append('REFERENCE_TASK_SESSION_CHANGED')
        slots=[]
        assignments={j:[] for j in (1,2,3)}
        links={index:[] for index in range(len(materials))}
        if self.reference:
            for index,item in enumerate(materials):
                for position,ring in self.reference.rings.items():
                    if overlap_ring(item,ring):assignments[position].append(index);links[index].append(position)
        for position in (1,2,3):
            why=list(reasons);raw_state='UNKNOWN';item=None;color=None
            ev=evidence.get(position,{})
            if self.reference and position not in self.reference.rings:why.append('POSITION_NOT_MAPPED')
            if self.reference and not ev.get('live_annulus_verified'):why.append('LIVE_RING_GEOMETRY_UNVERIFIED')
            if self.reference and not why:
                ring=self.reference.rings[position]
                matches=assignments[position]
                if ev.get('empty_print_matches'):
                    # Printed black linework belongs to the known empty reference.
                    if any(materials[i]['color_id']!=5 for i in matches):why.append('COLOR_EMPTY_EVIDENCE_CONFLICT')
                    else:raw_state='EMPTY'
                elif any(overlap_ring(r,ring) for r in uncertain):why.append('UNCERTAIN_MATERIAL_REGION')
                elif len(matches)!=1:why.append('OCCUPANT_MISSING' if not matches else 'MULTIPLE_OCCUPANTS')
                else:
                    index=matches[0];item=materials[index]
                    if len(links[index])!=1:why.append('MATERIAL_CROSSES_POSITIONS')
                    if item.get('clipped') or not contains(self.roi,item['bbox_px']):why.append('MATERIAL_CLIPPED')
                    if not item.get('color_calibrated') or item.get('quality_score',0)<.55:why.append('COLOR_UNVERIFIED')
                    if item['color_id']==5 and not item.get('shape_consistent'):why.append('BLACK_IDENTITY_UNVERIFIED')
                    if not why:raw_state='OCCUPIED';color=item['color_id']
            track=self.tracks[position]
            key=(raw_state,color)
            if why or raw_state=='UNKNOWN':track.clear()
            else:
                if item:
                    x,y,w,h=item['bbox_px'];center=[x+w/2,y+h/2]
                else:center=self.reference.rings[position]['center_px']
                if track and track[-1][1]!=key:track.clear()
                track.append((captured,key,*center))
            stable=False;span=0;jitter=0;speed=0;trend_speed=0;max_step=0
            if len(track)>=2:
                points=np.array([[row[0],row[2],row[3]] for row in track]);span=(points[-1,0]-points[0,0])*1000
                jitter=float(np.max(np.linalg.norm(points[:,1:]-np.median(points[:,1:],axis=0),axis=1)))
                speed=float(np.max(np.linalg.norm(np.diff(points[:,1:],axis=0),axis=1)/np.maximum(np.diff(points[:,0]),1e-6)))
                max_step=float(np.max(np.linalg.norm(np.diff(points[:,1:],axis=0),axis=1)))
                # The detector works at 480px width: one mask pixel becomes 2.67
                # source pixels. Differentiating a single edge change amplifies
                # that quantization into a false velocity. Estimate velocity over
                # the fresh five-frame window; retain raw speed for diagnosis.
                dt=points[:,0]-np.mean(points[:,0])
                trend_speed=float(np.linalg.norm(np.sum(dt[:,None]*points[:,1:],axis=0)/max(float(np.sum(dt*dt)),1e-12)))
                stable=len(track)>=5 and span>=500 and jitter<=4 and max_step<=4 and trend_speed<=8
            if not stable:why.append('OCCUPANCY_NOT_STABLE')
            slots.append(dict(position_id=position,occupancy=raw_state if not why else 'UNKNOWN',
                raw_occupancy=raw_state,observed_color_id=color if not why else None,
                material=item,position_source='session_numbered_empty_reference_with_live_annulus',
                track_center_source='fresh_material_bbox_center' if item else 'live_verified_ring_reference',
                evidence=ev,reasons=why,track=dict(samples=len(track),span_ms=round(span),jitter_px=round(jitter,2),max_speed_px_s=round(speed,2),trend_speed_px_s=round(trend_speed,2),max_step_px=round(max_step,2))))
        target=next((s for s in slots if expected and s['position_id']==expected['position_id']),None)
        target_reasons=list(target['reasons']) if target else list(reasons)
        match=bool(target and target['occupancy']=='OCCUPIED' and target['observed_color_id']==expected['color_id'])
        if target and target['occupancy']=='EMPTY':target_reasons.append('TARGET_POSITION_EMPTY')
        elif target and target['occupancy']=='OCCUPIED' and not match:target_reasons.append('WRONG_COLOR_AT_TARGET_POSITION')
        if not (station=='storage' and batch==2):target_reasons.append('NOT_SECOND_STORAGE_CONTEXT')
        return dict(schema_version=1,run_id=self.run_id,frame_sequence=sequence,captured_monotonic=captured,
            observed_at_unix=time.time(),frame_age_ms=round(age,2),task_raw=task.get('raw') if task else None,
            station=station,batch=batch,order=order,expected=expected,slots=slots,target=target,
            reference_id=self.reference.id if self.reference else None,
            occupancy_observation_ready=bool(target and target['occupancy']!='UNKNOWN'),
            same_color_target_observed=match and not target_reasons,target_reasons=target_reasons,
            first_layer_placement_correct='UNKNOWN',top_surface_pose=None,stacking_ready=False,stack_success=False,
            coordinate_unit='pixel',metric_valid=False,pose_mm=None,action_allowed=False,motion_enabled=False,uart_opened=False)
