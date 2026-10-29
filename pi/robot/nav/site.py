"""现场数据与当次map→odom配准边界；未知值不会变成已测值。"""

import json
import hashlib
import math
from pathlib import Path

from .frames import Pose, compose, relative

DEFAULT_SITE = Path(__file__).resolve().parents[3] / 'config/start1_qr_site.json'


def read_site(path=DEFAULT_SITE):
    return json.loads(Path(path).read_text(encoding='utf-8'))


def readiness(site):
    """仅表示数据齐备，仍需几何/轨迹检查与人工实测许可，绝不直接使能。"""
    required = ('field.width_m', 'field.height_m', 'body.front_m', 'body.rear_m',
                'body.left_m', 'body.right_m', 'start.from_left_m', 'start.from_top_m',
                'start.position_error_m', 'start.heading_error_rad',
                'start_zone.width_m', 'start_zone.height_m',
                'target.map_x_m', 'target.map_y_m', 'corridor.minimum_width_m')
    missing, invalid = [], []
    for key in required:
        group, name = key.split('.')
        value = site.get(group, {}).get(name)
        if value is None:
            missing.append(key)
        elif type(value) not in (int, float) or not math.isfinite(value) or value < 0:
            invalid.append(key)
        elif value == 0 and not key.endswith(('error_m', 'error_rad', 'map_x_m', 'map_y_m')):
            invalid.append(key)
    for key in ('body.complete', 'field.layout_matches_reference'):
        group, name = key.split('.')
        if site.get(group, {}).get(name) is not True:
            missing.append(key)
    if site.get('target', {}).get('node') not in ('north', 'qr', 'qr_prestop'):
        missing.append('target.node')
    if site.get('corridor', {}).get('state') != 'OPEN':
        missing.append('corridor.confirmed_open')
    for group in ('start', 'target'):
        if site.get(group, {}).get('heading_rad') != 0:
            invalid.append(group + '.heading_rad')
    status = 'INVALID_DATA' if invalid else 'WAITING_MEASUREMENTS' if missing else 'DATA_COMPLETE'
    placement = None
    placement_keys = ('field.width_m', 'field.height_m', 'body.front_m', 'body.rear_m',
                      'body.left_m', 'body.right_m', 'start.from_left_m', 'start.from_top_m')
    if not any(key in missing or key in invalid for key in placement_keys):
        body, start, field = site['body'], site['start'], site['field']
        x, top = start['from_left_m'], start['from_top_m']
        radius = math.hypot(max(body['front_m'], body['rear_m']), max(body['left_m'], body['right_m']))
        errors_known = all(type(start.get(key)) in (int, float) and math.isfinite(start[key])
                           and start[key] >= 0 for key in ('position_error_m', 'heading_error_rad'))
        uncertainty = (start['position_error_m'] + radius * start['heading_error_rad']) if errors_known else None
        clearances = dict(rear=x-body['rear_m'], front=field['width_m']-x-body['front_m'],
                          left=top-body['left_m'], right=field['height_m']-top-body['right_m'])
        placement = dict(field_clearance_m=clearances, placement_inflation_m=uncertainty)
        # 零净空无需等待道路或误差回填即可识别；未知误差不按零处理。
        if not invalid and (min(clearances.values()) <= 0
                            or uncertainty is not None and min(clearances.values()) <= uncertainty):
            status = 'PLACEMENT_UNSAFE'
    return dict(status=status, missing=missing, invalid=invalid, executable=False,
                frame_id='map', scope='start1_to_qr_prestop', registration_valid=False, placement=placement)


def initial_map_pose(site):
    if readiness(site)['status'] != 'DATA_COMPLETE':
        raise ValueError('现场数据尚未齐备，不能使用名义中心替代实际起点')
    return Pose(site['start']['from_left_m'],
                site['field']['height_m'] - site['start']['from_top_m'],
                site['start']['heading_rad'])


def bind(site, odom_pose, boot_id, odom_epoch, age_ms):
    """同一摆放状态下绑定新鲜odom；复位/换代后由valid撤销，不移动或写零。"""
    if not boot_id or not odom_epoch:
        raise ValueError('缺少当前启动身份或里程计代')
    if type(age_ms) not in (int, float) or not math.isfinite(age_ms) or not 0 <= age_ms < 250:
        raise ValueError('不能使用过期里程计建立配准')
    transform = compose(initial_map_pose(site), relative(odom_pose, Pose(0, 0, 0)))
    return dict(map_to_odom=transform.values(), boot_id=boot_id, odom_epoch=odom_epoch,
                method='measured_manual_start', position_error_m=site['start']['position_error_m'],
                heading_error_rad=site['start']['heading_error_rad'], site_sha256=site_digest(site))


def site_digest(site):
    return hashlib.sha256(json.dumps(site, sort_keys=True, allow_nan=False).encode()).hexdigest()


def valid(binding, boot_id, odom_epoch, site):
    return bool(binding and binding['boot_id'] == boot_id and binding['odom_epoch'] == odom_epoch
                and binding['site_sha256'] == site_digest(site))


def require_north(simulation=False):
    """执行前核对实放记录；同意将来摆放不等于本次已摆放。"""
    site = read_site()
    if simulation:
        return None
    # 用户明确关闭地图门槛：仅作相对起点试验，不建立虚假的map配准。
    if site.get('field_execution_guard_enabled', True) is False:
        return None
    if (readiness(site)['status'] != 'DATA_COMPLETE'
            or site.get('placement_confirmed') is not True
            or type(site.get('placement_boot_id')) is not int or site['placement_boot_id'] <= 0
            or site['start']['from_left_m'] != .15 or site['start']['from_top_m'] != .15
            or site['target']['node'] != 'north'
            or site['target']['map_x_m'] != 1.2 or site['target']['map_y_m'] != 2.12):
        raise ValueError('north需本次启动身份、实际按150/150毫米标记摆放、误差核对及开放道路；不得沿用待摆放方案')
    return site
