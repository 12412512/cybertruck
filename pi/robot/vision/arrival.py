"""复用旧工程粗加工2号环的小步矩阵审查与预览；不发送动作。"""
import copy
import math
from .legacy.alignment_vision.core import fingerprint


def validate_calibration(data,profile):
    """Explicit human review is required after physical small-step measurements."""
    if data.get('schema_version')!=1 or data.get('station')!='rough' or data.get('target')!=2:
        raise ValueError('ROUGH_RING_2_CALIBRATION_REQUIRED')
    if data.get('experimental_reviewed') is not True or data.get('ruler_checked') is not True or data.get('repeatability_checked') is not True:
        raise ValueError('PHYSICAL_CALIBRATION_REVIEW_REQUIRED')
    if data.get('profile_fingerprint')!=fingerprint(profile):raise ValueError('PROFILE_CHANGED')
    if data.get('scope')!='fixed_heading_empty_ring_experimental_return':raise ValueError('EXPERIMENT_SCOPE_REQUIRED')
    matrix=data.get('inverse_mm_per_px')
    if not isinstance(matrix,list) or len(matrix)!=2 or any(not isinstance(r,list) or len(r)!=2 for r in matrix):
        raise ValueError('INVERSE_SHAPE')
    flat=[x for row in matrix for x in row]
    if any(type(x) not in (int,float) or not math.isfinite(x) or abs(x)>2 for x in flat):raise ValueError('INVERSE_FINITE_BOUNDS')
    a,b,c,d=flat;det=a*d-b*c
    if abs(det)<.01 or sum(x*x for x in flat)/abs(det)>20:raise ValueError('INVERSE_DEGENERATE')
    for key,low,high in (('deadband_px',3,6),('min_step_mm',1,5),('max_step_mm',1,5),('gain',.1,.5)):
        v=data.get(key)
        if type(v) not in (int,float) or not math.isfinite(v) or not low<=v<=high:raise ValueError('PARAMETER_'+key)
    if data['min_step_mm']>data['max_step_mm']:raise ValueError('STEP_ORDER')
    radius=data.get('max_error_px',12)
    if type(radius) not in (int,float) or not math.isfinite(radius) or not 12<=radius<=120:
        raise ValueError('PARAMETER_max_error_px')
    if type(data.get('calibration_id')) is not int or not 0<data['calibration_id']<=0xffffffff:raise ValueError('CALIBRATION_ID')
    return copy.deepcopy(data)


def preview(error,data):
    if len(error)!=2 or any(not math.isfinite(x) for x in error):raise ValueError('ERROR_FINITE')
    radius=math.hypot(*error)
    if radius>data.get('max_error_px',12):return dict(state='OUTSIDE',command_mm=None)
    if radius<=data['deadband_px']:return dict(state='DONE',command_mm=[0.,0.])
    x,y=[-data['gain']*sum(row[i]*error[i] for i in (0,1)) for row in data['inverse_mm_per_px']]
    if abs(x)>=abs(y):y=0.
    else:x=0.
    # UART float32位移换回毫米会带来~1e-7mm尾差，避免恰好1mm被误判不足。
    x,y=round(x,6),round(y,6)
    size=abs(x)+abs(y)
    if size<data['min_step_mm']:return dict(state='PRECISION_LIMIT',command_mm=None)
    if size>data['max_step_mm']:
        x*=data['max_step_mm']/size;y*=data['max_step_mm']/size
    return dict(state='STEP',command_mm=[x,y])


def reviewed_preview(observation, calibration, profile, binding_sha256, now, *, barrier, stop_capture_floor):
    """只接受停车屏障后的稳定圆环观测；计划输出不等于运动许可。"""
    data = validate_calibration(calibration, profile)
    if (not isinstance(binding_sha256, str) or len(binding_sha256) != 64 or
        data.get('observation_binding_sha256') != binding_sha256):
        raise ValueError('OBSERVATION_POSE_CHANGED')
    if (observation.get('position_id') != 2 or observation.get('stable') is not True or
        observation.get('samples', 0) < 5 or observation.get('span_ms', 0) < 500 or
        type(observation.get('frame_age_ms')) not in (int, float) or
        not 0 <= observation['frame_age_ms'] < 250 or
        observation.get('profile_fingerprint') != fingerprint(profile) or
        observation.get('observation_binding_sha256') != binding_sha256 or
        not barrier or observation.get('barrier') != barrier or
        type(observation.get('oldest_capture')) not in (int, float) or
        type(stop_capture_floor) not in (int, float) or
        observation['oldest_capture'] < stop_capture_floor or
        not 0 <= now-observation['oldest_capture'] <= 1.5):
        raise ValueError('POST_STOP_OBSERVATION_REQUIRED')
    error = observation.get('delta_px')
    if not isinstance(error, list) or len(error) != 2 or any(type(x) not in (int,float) or not math.isfinite(x) for x in error):
        raise ValueError('PIXEL_ERROR_INVALID')
    return dict(preview(error, data), motion_authority=False, physical_validated=False,
                source='legacy_micro_preview', binding_sha256=binding_sha256)
