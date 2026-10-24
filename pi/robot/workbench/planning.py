"""复用正式规划器和RefSpec；冻结路线调速仅生成新的离线版本。"""

import ctypes as c
from dataclasses import dataclass
import hashlib
import json
from pathlib import Path
import uuid

from ..device.tracking import Spec
from ..nav.field import Field
from ..nav.frames import Pose
from ..nav.planner import Planner
from ..nav.reference import NativeReference
from ..nav.route import validate
from ..odom_sim import fixture

ROOT = Path(__file__).resolve().parents[3]


@dataclass
class Plan:
    view: dict
    specs: list


def make_plan(library, request):
    """耗时搜索在核心的后台线程进行，不持有控制循环锁。"""
    lib = c.CDLL(str(library))
    field = Field()
    start_heading = field.data['station_heading_quarters'][request.start]
    origin = Pose(*field.data['nodes'][request.start], start_heading * 1.5707963267948966)
    references, legs, specs = [], [], []
    if request.source != 'custom':
        if request.start != 'start_1' or request.blocked:
            raise ValueError('冻结路线只适用于启停区1、无封路；其他条件请选择自选站点规划')
        package = json.loads((ROOT / f'config/{request.source}_route.json').read_text(encoding='utf-8'))
        validate(package)
        stops = dict(raw=['qr', 'raw'], tour=['qr', 'raw', 'rough', 'staging', 'start_1'],
                     batch=['qr', 'raw', 'rough', 'staging', 'raw', 'rough', 'staging', 'start_1'])
        request = request.model_copy(update={'targets': stops[request.source]})
        current = origin
        ratio = request.speed_m_s / .30
        for index, raw in enumerate(package['goals']):
            spec = Spec.from_buffer_copy(bytes.fromhex(raw)[8:])
            # 一致的时间缩放保留V/A/J比例，不以缩短采样间隔假装调速。
            for limit in (spec.linear, spec.angular):
                limit.velocity *= ratio
                limit.acceleration *= ratio ** 2
                limit.jerk *= ratio ** 3
            spec.curve_velocity_m_s *= ratio
            spec.tail_velocity_m_s *= ratio
            ref = NativeReference(lib, current, spec)
            references.append(ref)
            specs.append(bytes(spec))
            legs.append(dict(name=package['cases'][index], kind=spec.kind, seconds=ref.duration_s))
            current = ref.end
    else:
        if any(node not in field.data['nodes'] for node in request.targets):
            raise ValueError('目标站点不存在')
        roads = {edge[0]: 'OPEN' for edge in field.data['edges']}
        if set(request.blocked) - roads.keys():
            raise ValueError('封路编号不存在')
        roads.update({key: 'BLOCKED' for key in request.blocked})
        planner = Planner(lib, fixture().geometry, field, cruise_m_s=request.speed_m_s)
        node, heading = request.start, start_heading
        for target in request.targets:
            if target == node:
                continue
            result = planner.plan(node, target, start_heading=heading, roads=roads,
                                  assigned_start=request.start,
                                  free_heading=target == request.start,
                                  park_region=target == request.start)
            if result['status'] != 'OK':
                raise ValueError(f'{node}→{target}无可行路径：{result["status"]}')
            for segment in result['segments']:
                references.append(segment.reference)
                specs.append(bytes(segment.reference.spec))
                legs.append(dict(name=f'{node} → {target}', kind=segment.reference.spec.kind,
                                 seconds=segment.reference.duration_s))
            node, heading = target, result['goal_heading']
    if not specs:
        raise ValueError('路线为空，请选择不同的目标站点')
    trace, speeds, offset = [], [], 0.
    for ref in references:
        trace.extend(p.values() for _, p in ref.samples(.12))
        for at, _ in ref.samples(.12):
            point = ref.point(at)
            speeds.append([offset + at, (point.vx_m_s ** 2 + point.vy_m_s ** 2) ** .5,
                           point.wz_rad_s])
        offset += ref.duration_s + .3
    digest = hashlib.sha256(b''.join(specs)).hexdigest()
    view = dict(id=uuid.uuid4().hex, sha256=digest, domain='simulation', executable=False,
                frame_id='map', origin=origin.values(), source=request.source,
                parameters=request.model_dump(), legs=legs, reference=trace, speed_samples=speeds,
                seconds=sum(ref.duration_s + .3 for ref in references),
                reason='正式C参考＋合成车体闭环；此参数版本不下发实车')
    return Plan(view, specs)
