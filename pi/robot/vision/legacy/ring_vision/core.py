"""Printed position IDs and concentric-ring pixels. No UART or robot actions."""
import math
from collections import deque
from pathlib import Path
import time
import uuid

import cv2
import numpy as np
from ..qr_camera.task_codec import parse_task
from .digits import DigitClassifier, extract_digit

DEVICE = '/dev/v4l/by-id/usb-SYD_USB_Camera_200901010001-video-index0'


def expected_position(task, station, batch, order):
    if station not in ('rough', 'storage') or batch not in (1, 2) or order not in (1, 2, 3):
        raise ValueError('Invalid station/batch/order')
    parsed = parse_task(task['raw'])
    item = parsed[f'batch{batch}'][order-1]
    return item['rough_processing_ring' if station == 'rough' else 'temporary_storage_ring']


def default_profile():
    return dict(schema_version=1, device=DEVICE, image_size=[1280, 720], roi=None,
                processing_width=640)


def validate_profile(p):
    if p.get('schema_version') != 1 or p.get('device') != DEVICE or p.get('image_size') != [1280,720]:
        raise ValueError('Unsupported ring-camera profile')
    if p.get('processing_width') != 640:
        raise ValueError('Unsupported processing width')
    roi = p.get('roi')
    if roi is not None:
        if not isinstance(roi,list) or len(roi)!=4 or any(type(x) not in (int,float) or not math.isfinite(x) for x in roi):
            raise ValueError('Invalid ROI')
        x1,y1,x2,y2=roi
        if not (0<=x1<x2<=1280 and 0<=y1<y2<=720 and x2-x1>=40 and y2-y1>=40):
            raise ValueError('Invalid ROI bounds')
    return p


class RingDetector:
    def __init__(self, profile):
        self.profile=validate_profile(profile)
        self.digits=DigitClassifier(Path(__file__).parent/'templates')

    def detect(self, frame):
        h,w=frame.shape[:2]
        if [w,h]!=self.profile['image_size']:return []
        scale=640/w
        work=cv2.resize(frame,(640,round(h*scale)))
        gray=cv2.cvtColor(work,cv2.COLOR_BGR2GRAY)
        binary=cv2.adaptiveThreshold(gray,255,cv2.ADAPTIVE_THRESH_GAUSSIAN_C,
                                    cv2.THRESH_BINARY_INV,31,9)
        contours,_=cv2.findContours(binary,cv2.RETR_LIST,cv2.CHAIN_APPROX_NONE)
        ellipses=[]
        for contour in contours:
            if len(contour)<35:continue
            try:(x,y),(a,b),angle=cv2.fitEllipse(contour)
            except cv2.error:continue
            if not (16<min(a,b) and max(a,b)<260 and min(a,b)/max(a,b)>.45):continue
            if abs(cv2.contourArea(contour)/(math.pi*a*b/4)-1)>.22:continue
            ellipses.append((x,y,a,b,angle))
        # Cluster contour evidence by centre, independently of digit identity.
        groups=[]
        for e in sorted(ellipses,key=lambda e:e[2]*e[3],reverse=True):
            for group in groups:
                x,y=group[0][:2]
                if math.hypot(e[0]-x,e[1]-y)<max(3,.06*min(e[2:4])):
                    group.append(e);break
            else:groups.append([e])
        results=[]
        angles=np.linspace(0,2*math.pi,180,endpoint=False)
        for group in groups:
            radii=sorted((a+b)/4 for _,_,a,b,_ in group)
            levels=[]
            for radius in radii:
                if not levels or radius-levels[-1]>2:levels.append(radius)
            if len(levels)<2 or levels[-1]/levels[0]<1.2:continue
            x,y=np.median(np.array(group)[:,:2],axis=0)
            # Same-plane ellipse normalisation aids rotated printed digits.
            reference=min(group,key=lambda e:e[2]*e[3]);a,b,angle=reference[2:]
            ratio=b/a
            theta=math.radians(angle)
            co,si=math.cos(theta),math.sin(theta)
            direction=np.array([[co,-si],[si,co]],np.float32)
            radii_scan=np.arange(max(8,int(levels[0]*.9)),int(levels[0]*2.1)+1)
            coverage=[]
            for radius in radii_scan:
                # Preserve ellipse aspect/angle while searching outer printed arcs.
                pts=direction@np.vstack([radius*np.cos(angles),radius*ratio*np.sin(angles)])
                xx=np.rint(x+pts[0]).astype(int);yy=np.rint(y+pts[1]).astype(int)
                good=(xx>=0)&(xx<640)&(yy>=0)&(yy<work.shape[0])
                coverage.append(np.count_nonzero(binary[yy[good],xx[good]])/len(angles))
            supported=[float(r) for r,c in zip(radii_scan,coverage) if c>=.25]
            if not supported:continue
            outer=max(supported)
            extent_x=outer*math.sqrt(co*co+ratio*ratio*si*si)
            extent_y=outer*math.sqrt(si*si+ratio*ratio*co*co)
            box=[(x-extent_x)/scale,(y-extent_y)/scale,
                 (x+extent_x)/scale,(y+extent_y)/scale]
            roi=self.profile['roi']
            if roi and not (roi[0]<=x/scale<=roi[2] and roi[1]<=y/scale<=roi[3]):continue
            src=np.float32([[x,y],[x+co*outer,y+si*outer],
                            [x-si*outer*ratio,y+co*outer*ratio]])
            transform=cv2.getAffineTransform(src,np.float32([[128,128],[228,128],[128,228]]))
            patch=cv2.warpAffine(work,transform,(256,256),borderValue=(255,255,255))
            digit=extract_digit(patch,128,128,100)
            label,score,margin,scores=self.digits.predict_detail(digit)
            raw=self.digits.predict_detail(extract_digit(work,round(x),round(y),round(outer)))
            if raw[0]!='UNKNOWN' and label!='UNKNOWN' and raw[0]!=label:
                label='UNKNOWN'
            elif raw[1]>score:
                label,score,margin,scores=raw
            clipped=box[0]<10 or box[1]<10 or box[2]>w-10 or box[3]>h-10
            roi_clipped=bool(roi and (box[0]<roi[0] or box[1]<roi[1] or box[2]>roi[2] or box[3]>roi[3]))
            results.append(dict(position_id=int(label) if label!='UNKNOWN' else None,
                center_px=[round(x/scale,2),round(y/scale,2)],bbox_px=[round(v,2) for v in box],
                concentric_levels=len(levels),digit_similarity=round(score,3),digit_margin=round(margin,3),
                digit_scores={str(k):round(v,3) for k,v in scores.items()},
                frame_clipped=bool(clipped),roi_clipped=roi_clipped,
                geometry_method='concentric_contour_ellipse',geometry_metric_valid=False))
        return results


class RingObserver:
    def __init__(self,profile):
        self.profile=profile;self.run_id=uuid.uuid4().hex;self.history=deque(maxlen=5)
        self.context=None;self.started=None;self.state='IDLE';self.last_sequence=-1;self.last_capture=None

    def begin(self,now=None):
        self.started=time.monotonic() if now is None else now
        self.state='WAITING';self.history.clear()

    def process(self,candidates,sequence,captured,now,task,station,batch,order,image_size):
        reasons=[];target_id=None;signature=None
        try:
            if not task or task.get('valid') is not True:raise ValueError('No valid task')
            target_id=expected_position(task,station,batch,order)
            signature=(task['raw'],task.get('accepted_at_unix'),station,batch,order)
        except (ValueError,KeyError,TypeError):reasons.append('TASK_INVALID')
        if signature!=self.context:
            self.context=signature;self.started=None;self.state='IDLE';self.history.clear()
        age=(now-captured)*1000
        if captured<=0 or not 0<=age<=250:reasons.append('CAMERA_STALE')
        if sequence<=self.last_sequence or (self.last_capture is not None and captured<=self.last_capture):reasons.append('DUPLICATE_FRAME')
        if self.last_capture is not None and captured-self.last_capture>.5:
            self.history.clear();reasons.append('FRAME_GAP')
        self.last_sequence=max(sequence,self.last_sequence);self.last_capture=captured
        if image_size!=self.profile['image_size']:reasons.append('RESOLUTION_MISMATCH')
        if not self.profile['roi']:reasons.append('RING_ROI_UNSET')
        matches=[c for c in candidates if c.get('position_id')==target_id] if target_id else []
        if len(matches)!=1:reasons.append('TARGET_NOT_FOUND' if not matches else 'DUPLICATE_POSITION_ID')
        target=matches[0] if len(matches)==1 else None
        if target:
            if target['digit_similarity']<.58 or target['digit_margin']<.08:reasons.append('DIGIT_UNCERTAIN')
            if target['frame_clipped'] or target['roi_clipped']:reasons.append('TARGET_CLIPPED')
        if self.started is None:reasons.append('REQUEST_NOT_STARTED')
        elif self.state=='WAITING' and now-self.started>8:self.state='TIMED_OUT'
        if self.state=='TIMED_OUT':reasons.append('REQUEST_TIMEOUT')
        if reasons:self.history.clear()
        else:self.history.append((captured,*target['center_px']))
        stable=False;span=0;jitter=0;speed=0
        if len(self.history)>=2:
            points=np.array(self.history);span=(points[-1,0]-points[0,0])*1000
            jitter=float(np.max(np.linalg.norm(points[:,1:]-np.median(points[:,1:],axis=0),axis=1)))
            speed=float(np.max(np.linalg.norm(np.diff(points[:,1:],axis=0),axis=1)/np.maximum(np.diff(points[:,0]),1e-6)))
            stable=len(points)>=5 and span>=500 and jitter<=3 and speed<=8
        if not stable:reasons.append('TARGET_UNSTABLE')
        ready=not reasons
        if ready:self.state='CONFIRMED'
        return dict(schema_version=1,run_id=self.run_id,frame_sequence=sequence,captured_monotonic=captured,
            observed_at_unix=time.time(),frame_age_ms=round(age,2),station=station,batch=batch,order=order,
            task_raw=task.get('raw') if task else None,target_position_id=target_id,candidates=candidates,
            target=target,ring_target_ready=ready,request_state=self.state,reasons=reasons,
            track=dict(samples=len(self.history),span_ms=round(span),jitter_px=round(jitter,2),max_speed_px_s=round(speed,2)),
            coordinate_unit='pixel',metric_valid=False,pose_mm=None,action_allowed=False,motion_enabled=False,
            uart_opened=False,occupancy='UNKNOWN',stacking_ready=False)
