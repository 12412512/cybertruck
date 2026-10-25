"""静止启停区的厂家坐标→map刚性配准；不修改雷达、不授予路线执行权限。"""
import hashlib
import copy
import itertools
import json
import math

from .replay import map_point


def center(report):
    """仅采用独立实扫末段的一件稳定目标，单位由厂家毫米转换为米。"""
    if (report.get('domain') != 'physical_lidar_scan' or report.get('live') is not False or
            report.get('status') != 'COMPLETE' or not report.get('stop_confirmed') or
            report.get('errors') or report.get('decode_bad') or report.get('decode_unknown') or
            report.get('zone') not in (1,2) or not report.get('board_identity')):
        raise ValueError('缺少完整、停稳、无错误的实机扫描证据')
    frames = report.get('frames',[])[-8:]
    if len(frames) < 8 or len({r['scan_sequence'] for r in frames}) != 8:
        raise ValueError('至少需要八个不同扫描序号的末段结果')
    points = []
    for frame in frames:
        if frame['frame_count'] != 14 or len(frame['objects']) != 1:
            raise ValueError('标定窗口必须只有一个完整14圈稳定目标')
        item = frame['objects'][0]
        if (item['frame_hits'] < 10 or item['confidence_percent'] < 90 or
                item['point_count'] < 8 or item['grid_id'] == 255):
            raise ValueError('标定目标质量不足')
        points.append((item['x_mm']/1000,item['y_mm']/1000))
    mean = [sum(p[i] for p in points)/len(points) for i in (0,1)]
    spread = max(math.dist(p,mean) for p in points)
    if spread > .015:
        raise ValueError('标定目标位置离散超过15mm')
    info = report['information']
    if info['zone'] != report['zone'] or info['running']:
        raise ValueError('扫描末端厂家区域或停止状态不符')
    return dict(scan_id=report['id'],zone=report['zone'],source_m=mean,spread_m=spread,
        vendor_pose=[info['x_mm'],info['y_mm'],info['heading_deg']],
        firmware=info['firmware'],board_identity=report['board_identity'])


def sample(report, left_mm, top_mm, uncertainty_mm):
    if not all(type(v) in (int,float) and math.isfinite(v) for v in (left_mm,top_mm,uncertainty_mm)):
        raise ValueError('需要实际有限尺量读数')
    if not 0 < left_mm < 2400 or not 0 < top_mm < 2400 or not 0 < uncertainty_mm <= 10:
        raise ValueError('标定点越界或尺量误差超过10mm')
    value = center(report)
    value.update(map_m=[left_mm/1000,(2400-top_mm)/1000],measurement_uncertainty_m=uncertainty_mm/1000)
    return value


def area(a,b,c):
    return ((b[0]-a[0])*(c[1]-a[1])-(b[1]-a[1])*(c[0]-a[0]))/2


def digest(value):
    return hashlib.sha256(json.dumps(value,sort_keys=True,allow_nan=False).encode()).hexdigest()


def same_profile(samples):
    first = samples[0]
    keys = ('zone','vendor_pose','firmware','board_identity')
    if any(any(value[key] != first[key] for key in keys) for value in samples):
        raise ValueError('标定期间厂家配置、区域或板端身份改变')
    if len({value['scan_id'] for value in samples}) != len(samples):
        raise ValueError('不能重复使用同一扫描作为独立标定点')


def fit(samples):
    if len(samples) < 3 or len(samples) > 12:
        raise ValueError('拟合需要3～12个独立实测点')
    same_profile(samples)
    source = [s['source_m'] for s in samples]
    target = [s['map_m'] for s in samples]
    triangles = list(itertools.combinations(range(len(samples)),3))
    largest = max(triangles,key=lambda ids:abs(area(*(target[i] for i in ids))))
    if abs(area(*(target[i] for i in largest))) < .01 or abs(area(*(source[i] for i in largest))) < .005:
        raise ValueError('标定点近似共线，旋转不可可靠确定')
    if area(*(source[i] for i in largest))*area(*(target[i] for i in largest)) <= 0:
        raise ValueError('坐标存在镜像或点位对应错误，拒绝用旋转强行配准')
    for i,j in itertools.combinations(range(len(samples)),2):
        expected, actual = math.dist(target[i],target[j]),math.dist(source[i],source[j])
        if abs(expected-actual) > max(.02,.05*expected):
            raise ValueError('对应点间距不一致，需核对尺量或目标身份')
    origin = [sum(p[i] for p in source)/len(source) for i in (0,1)]
    destination = [sum(p[i] for p in target)/len(target) for i in (0,1)]
    dot = cross = 0.
    for p,q in zip(source,target):
        x,y = p[0]-origin[0],p[1]-origin[1]
        u,v = q[0]-destination[0],q[1]-destination[1]
        dot += x*u+y*v
        cross += x*v-y*u
    yaw = math.atan2(cross,dot)
    rotated = map_point(origin[0]*1000,origin[1]*1000,[0,0,yaw])
    transform = [destination[0]-rotated[0],destination[1]-rotated[1],yaw]
    residuals = [math.dist(map_point(p[0]*1000,p[1]*1000,transform),q) for p,q in zip(source,target)]
    rms = math.sqrt(sum(v*v for v in residuals)/len(residuals))
    if rms > .015 or max(residuals) > .025:
        raise ValueError('配准残差超限，不保存为已验证配置')
    value = dict(schema=1,domain='physical_lidar_calibration',status='CANDIDATE',
        real_map_registered=False,authorizes_motion=False,coverage='unknown',
        zone=samples[0]['zone'],map_transform=transform,samples=samples,
        rms_m=rms,max_residual_m=max(residuals),
        scope='仅本次启停区静止姿态和已观测邻域；不代表全场覆盖')
    value['version'] = digest(value)
    return value


def validate(candidate, independent):
    if candidate.get('status') != 'CANDIDATE' or candidate['version'] != digest({k:v for k,v in candidate.items() if k!='version'}):
        raise ValueError('候选版本内容不符')
    same_profile(candidate['samples']+[independent])
    if min(math.dist(s['map_m'],independent['map_m']) for s in candidate['samples']) < .1:
        raise ValueError('独立复核点距已有标记不足100mm')
    predicted = map_point(independent['source_m'][0]*1000,independent['source_m'][1]*1000,candidate['map_transform'])
    error = math.dist(predicted,independent['map_m'])
    if error > .03:
        raise ValueError('独立复核误差超过30mm，不能标记配准通过')
    value = dict(candidate,status='VERIFIED_LOCAL',real_map_registered=True,
        independent_check=dict(sample=independent,predicted_m=predicted,error_m=error),
        position_uncertainty_m=max(.02,candidate['max_residual_m']+max(s['measurement_uncertainty_m'] for s in candidate['samples'])+independent['spread_m']))
    value.pop('version');value['version']=digest(value)
    return value


def audit_axes(samples, independent):
    """独立点不参与拟合；显式Y反射仅供诊断，永不发布地图配准。"""
    same_profile(samples + [independent])
    result = dict(domain='physical_lidar_calibration_diagnostic',
        real_map_registered=False, authorizes_motion=False, applied=False,
        holdout_limit_m=.03, samples=copy.deepcopy(samples), independent=copy.deepcopy(independent),
        pairs=[], hypotheses=[])
    originals = samples + [independent]
    for i, j in itertools.combinations(range(len(originals)), 2):
        result['pairs'].append(dict(scan_ids=[originals[i]['scan_id'], originals[j]['scan_id']],
            source_distance_m=math.dist(originals[i]['source_m'], originals[j]['source_m']),
            map_distance_m=math.dist(originals[i]['map_m'], originals[j]['map_m'])))
    for name, y_sign in (('original_xy', 1), ('diagnostic_flip_vendor_y', -1)):
        values = copy.deepcopy(samples)
        check = copy.deepcopy(independent)
        for value in values + [check]:
            value['source_m'][1] *= y_sign
        item = dict(hypothesis=name, status='REJECTED', authorizes_motion=False)
        try:
            candidate = fit(values)
        except ValueError as error:
            item['reason'] = str(error)
        else:
            point = check['source_m']
            predicted = map_point(point[0]*1000, point[1]*1000, candidate['map_transform'])
            item.update(map_transform=candidate['map_transform'], rms_m=candidate['rms_m'],
                max_residual_m=candidate['max_residual_m'], predicted_m=predicted,
                holdout_error_m=math.dist(predicted, check['map_m']))
            try:
                validate(candidate, check)
            except ValueError as error:
                item['reason'] = str(error)
            else:
                item.update(status='HYPOTHESIS_SUPPORTED',
                    reason='仅离线假设通过；尚未确认坐标定义、应用配置或覆盖范围')
        result['hypotheses'].append(item)
    return result
