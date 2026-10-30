"""由同一观察位四向小位移记录求局部矩阵；默认只生成未验收草稿。"""
import math
from .legacy.alignment_vision.core import fingerprint
from .centering import load_calibration


def fit_records(snapshot,record_ids,calibration_id=1,max_error_px=12):
    if snapshot.get('sha256')!=fingerprint({k:v for k,v in snapshot.items() if k!='sha256'}):
        raise ValueError('小位移记录校验失败')
    if (not isinstance(record_ids,list) or len(record_ids)<4 or len(set(record_ids))!=len(record_ids) or
            any(type(i) is not int or i<=0 for i in record_ids)):
        raise ValueError('至少选择四条不同的小位移记录')
    selected=[r for r in snapshot['records'] if r['id'] in record_ids]
    if len(selected)!=len(record_ids):raise ValueError('所选记录缺失或ID重复')
    first=selected[0]
    camera=first['after']['camera_signature'];binding=first['observation_binding_sha256']
    columns=[];reviewed=True;residuals=[]
    for axis,index in (('x',0),('y',1)):
        rows=[]
        for r in selected:
            if (r['axis'] not in ('x','y') or r['observation_binding_sha256']!=binding or
                    r['before']['camera_signature']!=camera or r['after']['camera_signature']!=camera or
                    r['before']['stream_id']!=r['after']['stream_id'] or
                    r['before']['capture_epoch']!=r['after']['capture_epoch'] or
                    not r['before']['stable'] or not r['after']['stable']):
                raise ValueError('记录混用了相机、观察姿态、采集会话或不稳定图像')
            yaws=[r['before'].get('yaw_rad'),r['after'].get('yaw_rad')]
            if (any(type(v) not in (int,float) or not math.isfinite(v) for v in yaws) or
                    abs(math.atan2(math.sin(yaws[1]-yaws[0]),math.cos(yaws[1]-yaws[0])))>math.radians(1)):
                raise ValueError('小位移期间航向变化过大或缺少航向反馈')
            if r['axis']!=axis:continue
            actual=r.get('measured_mm')
            physical=type(actual) in (int,float) and r.get('physical_reviewed') is True
            reviewed=reviewed and physical
            if not physical:
                if not isinstance(r.get('odom_delta_mm'),list) or len(r['odom_delta_mm'])!=2:
                    raise ValueError('缺少里程计或实际量尺位移')
                actual=r['odom_delta_mm'][index]
                if abs(r['odom_delta_mm'][1-index])>.5:raise ValueError('横向漂移超过小位移拟合范围')
            delta=r['delta_px']
            if (type(actual) not in (int,float) or not math.isfinite(actual) or not 1<=abs(actual)<=10 or
                    actual*r['command_mm']<=0 or not isinstance(delta,list) or len(delta)!=2 or
                    any(type(v) not in (int,float) or not math.isfinite(v) for v in delta)):
                raise ValueError('实际位移、方向或图像差分无效')
            rows.append((actual,delta))
        if not any(q>0 for q,_ in rows) or not any(q<0 for q,_ in rows):
            raise ValueError('X/Y各需正负方向记录，不能单向外推')
        weight=sum(q*q for q,_ in rows)
        column=[sum(q*p[k] for q,p in rows)/weight for k in (0,1)]
        if math.hypot(*column)<.5:raise ValueError('图像响应太小，无法可靠求方向比例')
        columns.append(column)
        residuals.extend(math.hypot(*(p[k]-q*column[k] for k in (0,1))) for q,p in rows)
    a,c=columns[0];b,d=columns[1];det=a*d-b*c
    if abs(det)<.01:raise ValueError('两方向图像响应退化，不能求逆')
    residual=max(residuals)
    if residual>3:raise ValueError('正反向拟合残差超过3像素，需检查相机或机构重复性')
    result=dict(schema_version=1,goal='camera_image_center',station='rough',target=2,
        camera_signature=camera,profile_fingerprint=fingerprint(camera),
        scope='fixed_heading_empty_ring_experimental_return',inverse_mm_per_px=[[d/det,-b/det],[-c/det,a/det]],
        observation_binding_sha256=binding,calibration_id=calibration_id,max_error_px=max_error_px,
        deadband_px=4,min_step_mm=1,max_step_mm=5,gain=.5,
        experimental_reviewed=False,ruler_checked=reviewed,repeatability_checked=False,
        physical_directions_checked=False,source_record_ids=record_ids,
        fit_max_residual_px=residual,hardware_validated=False)
    result['sha256']=fingerprint(result)
    # 用审查副本检查数值条件；输出本身仍是未审查草稿，不能驱动自动微调。
    check=dict(result,experimental_reviewed=True,ruler_checked=True,repeatability_checked=True,physical_directions_checked=True)
    check['sha256']=fingerprint({k:v for k,v in check.items() if k!='sha256'})
    load_calibration(check)
    return result
