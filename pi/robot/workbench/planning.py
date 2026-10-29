"""按当前场景搜索路线，复用正式C参考，明确请求速度与实际限速。"""

import ctypes as c
from collections import OrderedDict
import copy
from dataclasses import dataclass
import hashlib
import json
import math
import uuid
import threading
from pathlib import Path

from ..device.tracking import Spec
from ..nav.frames import Pose
from ..nav.planner import Planner
from ..nav.reference import NativeReference, SETTLE_ESTIMATE_S
from ..odom_sim import fixture
from .scene import make_field, normalize, start_zones, NAMES

_CACHE = OrderedDict()
_CACHE_LOCK = threading.Lock()


@dataclass
class Plan:
    view: dict
    specs: list


def make_plan(library, request, cancelled=None, progress=None):
    """A*在既有道路/朝向候选中按时间选优，不宣称连续平面的全局最优。"""
    request = normalize(request)
    field = make_field(request)
    key = (str(Path(library).resolve()), Path(library).stat().st_mtime_ns,
           request.model_dump_json(), json.dumps(field.data, sort_keys=True))
    if cancelled and cancelled():
        raise InterruptedError('规划被替代')
    with _CACHE_LOCK:
        cached = _CACHE.get(key)
        if cached:
            view = copy.deepcopy(cached.view)
            view['id'] = uuid.uuid4().hex
            return Plan(view, list(cached.specs))
    lib = c.CDLL(str(library))
    heading = field.data['station_heading_quarters'][request.start]
    origin = Pose(*field.data['nodes'][request.start], heading * math.pi / 2)
    roads = {edge[0]: 'OPEN' for edge in field.data['edges']}
    if set(request.blocked) - roads.keys():
        raise ValueError('封路编号不存在')
    if any(node not in field.data['nodes'] for node in request.targets):
        raise ValueError('目标站点不存在')
    roads.update({key: 'BLOCKED' for key in request.blocked})
    dynamics = request.dynamics.model_dump(exclude={'staging_slowdown', 'wheel_slew_rpm_s', 'lag_ff_s'})
    planner = Planner(lib, fixture().geometry, field, max_rpm=request.max_rpm,
                      cruise_m_s=request.speed_m_s, cancelled=cancelled,
                      **dynamics)
    staging = dict(dynamics, acceleration_m_s2=min(.45, dynamics['acceleration_m_s2']),
                   jerk_m_s3=min(1.2, dynamics['jerk_m_s3']), dock_m_s=min(.025, dynamics['dock_m_s']),
                   turn_rate_rad_s=min(1.2, dynamics['turn_rate_rad_s']))
    staging_planner = Planner(lib, fixture().geometry, field, max_rpm=request.max_rpm,
                             cruise_m_s=request.speed_m_s, cancelled=cancelled, **staging) if request.dynamics.staging_slowdown else planner
    references, legs, specs, rejected = [], [], [], {}
    node = request.start
    previous_pose = origin
    for index, target in enumerate(request.targets):
        if target == node:
            continue
        if progress:
            progress(dict(completed=index, total=len(request.targets),
                          current=f'{NAMES.get(node,node)} → {NAMES.get(target,target)}'))
        active_planner = staging_planner if target == 'staging' else planner
        result = active_planner.plan(node, target, start_heading=heading, roads=roads,
                              assigned_start=request.start, free_heading=target == request.start,
                              park_region=target == request.start, start_pose=previous_pose)
        if result['status'] != 'OK':
            reasons = sorted({reason for values in result['rejected'].values() for reason in values})
            raise ValueError(f'{NAMES.get(node,node)}→{NAMES.get(target,target)}无可行路径：'
                             f'{result["status"]}；' + '、'.join(reasons[:10]))
        if index == 0 and node == 'start_2' and target == 'qr' and heading == 0:
            # 车头保持+X，倒车斜行至扫码；闭路或障碍不能被直连候选绕过。
            direct_roads = ('start2_exit', 'north_east_road', 'qr_dock')
            if all(result['road_states'].get(road) == 'OPEN' for road in direct_roads):
                direct, errors = active_planner.make('DIAGONAL', previous_pose,
                    Pose(*field.data['nodes']['qr'], 0), '+'.join(direct_roads),
                    start_zones(request.model_dump()))
                if direct is not None and not errors:
                    result['segments'] = [direct]
                    result['goal_heading'] = 0
                else:
                    result['rejected']['start2_direct'] = errors
        rejected.update(result['rejected'])
        for segment in result['segments']:
            if len(specs) >= 100:
                raise ValueError('路线超过100个控制段，请减少站点或调整障碍物')
            references.append(segment.reference)
            specs.append(bytes(segment.reference.spec))
            legs.append(dict(name=f'{NAMES.get(node,node)} → {NAMES.get(target,target)}',
                             kind=segment.reference.spec.kind, seconds=segment.reference.duration_s,
                             roads=segment.road.split('+'), requested_m_s=request.speed_m_s,
                             segment_cap_m_s=segment.reference.spec.linear.velocity))
        node, heading = target, result['goal_heading']
        previous_pose = references[-1].end
    if not specs:
        raise ValueError('路线为空，请选择不同的目标站点')
    trace, speeds, offset = [], [], 0.
    peak, wheel_peak, distance = 0., 0., 0.
    for ref in references:
        samples = ref.samples(.12)
        points = [pose.values() for _, pose in samples]
        trace.extend(points)
        distance += sum(math.dist(a[:2], b[:2]) for a, b in zip(points, points[1:]))
        for at, _ in samples:
            point = ref.point(at)
            speed = math.hypot(point.vx_m_s, point.vy_m_s)
            peak = max(peak, speed)
            speeds.append([offset + at, speed, point.wz_rad_s])
        wheel_peak = max(wheel_peak, planner.wheel_bound(ref))
        offset += ref.duration_s + SETTLE_ESTIMATE_S
    reasons = []
    theoretical = request.max_rpm * 2 * math.pi * .05 / 60
    if request.speed_m_s > theoretical / math.sqrt(2):
        reasons.append('模型轮速上限及斜向/转姿轮速分配')
    if any(ref.spec.kind in (3, 4, 6) for ref in references):
        reasons.append('弯道曲率、转姿与靠站减速')
    if peak < request.speed_m_s * .98:
        reasons.append('短段距离及加速度/加加速度限制，不能瞬间达到请求上限')
    parameters = request.model_dump()
    config_hash = hashlib.sha256(json.dumps(parameters, sort_keys=True).encode()).hexdigest()
    view = dict(schema=2, id=uuid.uuid4().hex, sha256=hashlib.sha256(b''.join(specs)).hexdigest(),
                configuration_sha256=config_hash, domain='simulation', executable=False,
                frame_id='map', origin=origin.values(), source=request.source,
                parameters=parameters, field=field.data, legs=legs, reference=trace,
                speed_samples=speeds, seconds=offset, distance_m=distance,
                settle_estimate_s=SETTLE_ESTIMATE_S,
                limits=dict(requested_m_s=request.speed_m_s, sampled_peak_m_s=peak,
                            wheel_bound_rpm=wheel_peak, max_rpm=request.max_rpm,
                            theoretical_straight_m_s=theoretical, reasons=reasons),
                selection='道路朝向A*按预计时间搜索，再复用正式参考平滑；不是连续平面全局最优证明',
                obstacle_rejections=dict(list((key, values) for key, values in rejected.items()
                                              if any(value.startswith('obstacle:') for value in values))[:40]),
                reason='当前场景重新规划；合成车体参数不代表实车1m/s已验收')
    result = Plan(view, specs)
    with _CACHE_LOCK:
        _CACHE[key] = Plan(copy.deepcopy(view), list(specs))
        while len(_CACHE) > 8:
            _CACHE.popitem(last=False)
    return result


def restore_plan(value, library):
    """读取本机已保存的参数与RefSpec，只恢复预览，不自动运行。"""
    from .contracts import PlanInput, Run
    from .storage import artifact_digest
    Run(plan_id=value['id'])
    if value.get('artifact_sha256') != artifact_digest(value):
        raise ValueError('保存配置或显示轨迹校验失败')
    parameters = PlanInput.model_validate(value['parameters'])
    if value.get('schema') != 2 or value.get('domain') != 'simulation' or value.get('executable') is not False:
        raise ValueError('不是当前版本的离线场景')
    specs = [bytes.fromhex(raw) for raw in value['specs']]
    if not 1 <= len(specs) <= 100 or any(len(raw) != c.sizeof(Spec) for raw in specs):
        raise ValueError('路线段格式或数量无效')
    if hashlib.sha256(b''.join(specs)).hexdigest() != value['sha256']:
        raise ValueError('保存路线摘要不一致')
    scene = make_field(parameters)
    if scene.data != value['field']:
        raise ValueError('地图或停靠参数已变化，请重新规划')
    current = Pose(*value['origin'])
    heading = scene.data['station_heading_quarters'][parameters.start]
    expected = Pose(*scene.data['nodes'][parameters.start], heading * math.pi / 2)
    if current != expected:
        raise ValueError('保存路线的起点与配置不一致')
    lib = c.CDLL(str(library))
    for raw in specs:
        ref = NativeReference(lib, current, Spec.from_buffer_copy(raw))
        if scene.check(ref, start_zones(parameters.model_dump())):
            raise ValueError('保存轨迹不能通过当前场景碰撞检查')
        current = ref.end
    restored = {key: item for key, item in value.items() if key not in ('specs', 'artifact_sha256')}
    # 老版本缺少独立动力参数时补同值默认，保留冻结Spec，不重新规划或改速度。
    restored['parameters'] = parameters.model_dump()
    restored['configuration_sha256'] = hashlib.sha256(json.dumps(restored['parameters'], sort_keys=True).encode()).hexdigest()
    return Plan(restored, specs)
