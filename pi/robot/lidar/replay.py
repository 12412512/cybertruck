"""停车扫描离线回放：复用旧解码与GRID/RESULT语义，不提供设备命令接口。"""

import copy
import hashlib
import json
import math
import struct
import uuid

from .protocol import Decoder, CloudAssembler, hello


def map_point(x_mm, y_mm, transform):
    x, y, yaw = transform
    cosine, sine = math.cos(yaw), math.sin(yaw)
    return [x + (cosine*x_mm - sine*y_mm)/1000,
            y + (sine*x_mm + cosine*y_mm)/1000]


def replay(wire, profile, start):
    """profile仅由本机合成夹具提供。外部报文缺少真实标定，永不自动赋予配准。"""
    if not 0 < len(wire) <= 65536:
        raise ValueError('离线报文需1～65536字节')
    decoder, cloud = Decoder(), CloudAssembler()
    profile = copy.deepcopy(profile)
    errors, clouds, results, grids, seen = [], {}, {}, {}, {}
    events, information, running, stopped, started = [], None, False, False, False
    for offset in range(0, len(wire), 37):
        for node, kind, sequence, data in decoder.feed(wire[offset:offset+37]):
            try:
                if node != profile['node']:
                    raise ValueError('雷达节点不匹配')
                key = node, kind, sequence
                if key in seen:
                    if seen[key] != data:
                        raise ValueError('重复包序号内容冲突')
                    continue
                seen[key] = data
                events.append(dict(type=kind, sequence=sequence, size=len(data)))
                if kind == 0x81:
                    value = hello(data)
                    if (value['zone'] != profile['vendor_zone'] or value['running'] or started or
                            information is not None and value != information):
                        raise ValueError('HELLO区号/运行状态变化，不能沿用旧扫描')
                    information = value
                elif kind == 0x82:
                    if len(data) != 2 or data[1]:
                        raise ValueError('ACK无效或命令被拒绝')
                    if data[0] == 3:
                        if not information or started or stopped:
                            raise ValueError('扫描启动顺序无效')
                        running = started = True
                    elif data[0] == 4:
                        if not started:
                            raise ValueError('STOP早于扫描启动')
                        stopped, running = True, False
                elif kind == 0x83:
                    if len(data) != 5 or data[2] not in (0, 1) or data[4]:
                        raise ValueError('STATUS状态/溢出异常')
                    if stopped and data[2]:
                        raise ValueError('STOP后仍报告扫描运行')
                    if data[0] == 3 and data[2] == 0 and started:
                        stopped, running = True, False
                elif kind == 0x8A:
                    raise ValueError('雷达上报ERROR')
                elif kind in (0x84, 0x85, 0x86, 0x8D, 0x87, 0x89):
                    if not running:
                        raise ValueError('扫描窗口外结果，不接入规划')
                    if kind in (0x84, 0x85, 0x86):
                        value = cloud.feed(kind, data)
                        if value:
                            seq = value['scan_sequence']
                            if seq in clouds:
                                raise ValueError('同一扫描序号被重新发布')
                            clouds[seq] = value
                    elif kind == 0x8D:
                        if len(data) < 5:
                            raise ValueError('GRID长度错误')
                        seq, count = struct.unpack_from('<IB', data)
                        ids = list(data[5:])
                        if count > 25 or len(ids) != count or any(v > 24 and v != 255 for v in ids):
                            raise ValueError('GRID数量/编号异常')
                        if seq in grids or seq in results:
                            raise ValueError('重复GRID扫描序号')
                        grids[seq] = ids
                    elif kind == 0x87:
                        if len(data) < 8:
                            raise ValueError('RESULT长度错误')
                        seq, count, locked, overflow, number = struct.unpack_from('<IBBBB', data)
                        if (number > 25 or len(data) != 8+number*8 or locked not in (0, 1) or
                                overflow or not 1 <= count <= 14 or seq not in clouds or
                                seq not in grids or len(grids[seq]) != number or seq in results):
                            raise ValueError('RESULT缺少完整点云/匹配GRID或数量/溢出异常')
                        objects = []
                        for index in range(number):
                            x, y, hits, confidence, points = struct.unpack_from('<hhBBH', data, 8+index*8)
                            if hits > 14 or confidence > 100:
                                raise ValueError('RESULT命中/置信度异常')
                            objects.append(dict(x_mm=x, y_mm=y, frame_hits=hits,
                                confidence_percent=confidence, point_count=points, grid_id=grids[seq][index]))
                        if len({item['grid_id'] for item in objects}) != len(objects):
                            raise ValueError('同帧重复栅格，身份不明确')
                        results[seq] = objects
                        del grids[seq]
                    else:
                        if len(data) != 25 or struct.unpack('<IHHHIIIHB', data)[-1] & 3:
                            raise ValueError('STATS长度/溢出异常')
            except (ValueError, struct.error) as exc:
                errors.append(str(exc))
    if decoder.bad or decoder.unknown or decoder.partial or decoder.discarding:
        errors.append('CRC/未知帧/拆包残留或超长帧异常')
    if cloud.current is not None or cloud.dropped or grids:
        errors.append('点云/GRID尚未完整结束')
    if not stopped or not started or len(clouds) < 3 or set(clouds) != set(results):
        errors.append('未确认STOP或不足三个独立完整扫描/结果')
    if clouds:
        ticks = [item['device_timestamp_ms'] for item in clouds.values()]
        elapsed = [(b-a) & 0xffffffff for a, b in zip(ticks, ticks[1:])]
        if any(not value or value > 10000 for value in elapsed) or sum(elapsed) > 10000:
            errors.append('扫描时间戳倒退/重复或超出10秒窗口')
    transform = profile.get('map_transform')
    registered = bool(profile.get('domain') == 'simulation' and profile.get('synthetic') is True and
                      transform and len(transform) == 3 and all(math.isfinite(v) for v in transform) and
                      profile.get('start') == start)
    if not registered:
        errors.append('没有与当前启停区匹配的模拟标定；真实配准未完成')
    candidates, obstacles = [], []
    minimum_frames = min(3, len(results))
    keys = sorted({item['grid_id'] for frame in results.values() for item in frame})
    for grid in keys:
        samples = [item for frame in results.values() for item in frame if item['grid_id'] == grid]
        accepted = [item for item in samples if item['frame_hits'] >= 3 and
                    item['confidence_percent'] >= 60 and item['point_count'] >= 3 and grid != 255]
        x = sum(item['x_mm'] for item in samples)/len(samples)
        y = sum(item['y_mm'] for item in samples)/len(samples)
        spread = max(math.hypot(item['x_mm']-x, item['y_mm']-y) for item in samples)/1000
        stable = len(accepted) >= minimum_frames and len(samples) == len(results) and spread <= .02
        candidate = dict(grid_id=grid, x_mm=x, y_mm=y, frames=len(samples), accepted=stable,
                         confidence_percent=min(item['confidence_percent'] for item in samples))
        candidates.append(candidate)
        if not stable:
            errors.append(f'栅格{grid}低质量/遮挡/目标不稳定，不能当作空地')
        if transform and stable and registered:
            mx, my = map_point(x, y, transform)
            uncertainty = profile['uncertainty_m']+spread
            diameter = profile['diameter_m']+2*uncertainty
            if not (0 <= mx <= 2.4 and 0 <= my <= 2.4 and .01 <= diameter <= .3):
                errors.append('障碍坐标或包络超出规划范围')
                continue
            obstacles.append(dict(id=f'lidar_{grid}', x_m=mx, y_m=my, diameter_m=diameter))
    latest = list(clouds.values())[-1] if clouds else None
    points = [map_point(item[0], item[1], transform) for item in latest['points']] if latest and registered else []
    empty = not candidates
    if empty and not profile.get('empty_coverage_valid', False):
        errors.append('零障碍但覆盖无效，保留未知')
    errors = list(dict.fromkeys(errors))
    value = dict(schema=1, id=uuid.uuid4().hex, domain='simulation', source=profile['source'],
        synthetic=bool(profile.get('synthetic')), live=False, authorizes_motion=False,
        physical_validated=False, real_map_registered=False, frame_id='cyqm_field',
        status='REJECTED' if errors else 'COMPLETE', usable=not errors,
        reason='；'.join(errors), start=start, vendor_zone=profile['vendor_zone'],
        calibration_version=profile.get('version'), simulation_registered=registered,
        map_transform=transform if registered else None, coverage=profile.get('coverage', []),
        coverage_kind=profile.get('coverage_kind','unknown'), stop_confirmed=stopped,
        cloud_frames=len(clouds), result_frames=len(results), decode_good=decoder.good,
        decode_bad=decoder.bad, scan_sequences=list(clouds), raw_sha256=hashlib.sha256(wire).hexdigest(),
        candidates=candidates, obstacles=obstacles, points=points,
        raw_points=latest['points'] if latest else [], events=events,
        diameter_m=profile.get('diameter_m'), uncertainty_m=profile.get('uncertainty_m'))
    value['sha256'] = hashlib.sha256(json.dumps(value, sort_keys=True).encode()).hexdigest()
    return value
