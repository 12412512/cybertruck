"""Web计划到正式执行器的主机联调包；不会生成实车动作许可。"""

import hashlib
import ctypes as c
import math
import struct

from ..device.tracking import Spec
from .frames import Pose, relative
from .reference import NativeReference
from .route import digest


def prepare(plan, library):
    from ..workbench.planning import restore_plan
    from ..workbench.storage import payload
    checked = restore_plan(payload(plan), library)
    parameters = checked.view['parameters']
    dynamics = parameters['dynamics']
    if (parameters['speed_m_s'] > .30 or parameters['max_rpm'] != 100 or
            dynamics['lag_ff_s'] != 0 or dynamics['wheel_slew_rpm_s'] != 180 or
            dynamics['acceleration_m_s2'] > .25 or dynamics['jerk_m_s3'] > .8 or
            dynamics['turn_rate_rad_s'] > 1.2):
        raise ValueError('执行链联调限0.30m/s、100RPM及已验证加减速档；更高参数仍可普通模拟')
    if not 1 <= len(checked.specs) <= 64:
        raise ValueError('执行链联调只接受1～64段')
    origin = current = Pose(*checked.view['origin'])
    native = c.CDLL(str(library))
    boundaries, seconds = [], 0.
    for raw in checked.specs:
        ref = NativeReference(native, current, Spec.from_buffer_copy(raw))
        if not math.isfinite(ref.duration_s) or ref.duration_s > 25:
            raise ValueError('单段参考超过25秒，需要拆段或调整到已验证速度档')
        current = ref.end
        seconds += ref.duration_s + .2
        boundaries.append(relative(origin, current).values())
    if seconds > 180:
        raise ValueError('本轮离线执行链参考总量超过180秒上限')
    route = dict(schema=1, id='planned_preview_v1', domain='simulation', executable=False,
        physical_validated=False, frame_id='odom', map_registered=False,
        source_plan_id=checked.view['id'], source_artifact_sha256=payload(checked)['artifact_sha256'],
        source_specs_sha256=hashlib.sha256(b''.join(checked.specs)).hexdigest(),
        cases=[f'preview_{i+1}' for i in range(len(checked.specs))],
        goals=[(struct.pack('<II', i+1, 30000)+raw).hex() for i,raw in enumerate(checked.specs)],
        reference_trace=[relative(origin,Pose(*p)).values() for p in checked.view['reference']],
        boundaries=boundaries, map_origin=origin.values(), parameters=parameters,
        field=checked.view['field'], estimated_s=seconds,
        anchor_policy='measured_segment_start', segment_handoff='stopped_and_settled')
    route['sha256'] = digest(route)
    validate(route)
    return route


def validate(route):
    if (route.get('id') != 'planned_preview_v1' or route.get('schema') != 1 or
            route.get('domain') != 'simulation' or route.get('executable') is not False or
            route.get('physical_validated') is not False or route.get('map_registered') is not False or
            route.get('frame_id') != 'odom' or route.get('sha256') != digest(route)):
        raise ValueError('不是离线执行链包，不能提升为实车路线')
    goals = route.get('goals', [])
    if not 1 <= len(goals) <= 64 or len(route.get('boundaries',[])) != len(goals):
        raise ValueError('联调包段数或边界不匹配')
    for index,raw in enumerate(goals):
        wire = bytes.fromhex(raw)
        if len(wire) != 96 or struct.unpack('<II',wire[:8]) != (index+1,30000):
            raise ValueError('联调目标编号、长度或期限无效')
    if hashlib.sha256(b''.join(bytes.fromhex(g)[8:] for g in goals)).hexdigest() != route['source_specs_sha256']:
        raise ValueError('联调目标不是当前Web计划的控制段')
