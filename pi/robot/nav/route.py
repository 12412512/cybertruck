"""冻结正式RefSpec和参考轨迹；同一格式用于模拟、预览与执行准备。"""

import ctypes as c
import hashlib
import json
import math

from ..device.execution import trial
from ..device.tracking import Spec
from .frames import Pose
from .reference import NativeReference

CASES = ('straight', 'diagonal', 'arc', 'pose')


def digest(plan):
    data = {key: value for key, value in plan.items() if key != 'sha256'}
    return hashlib.sha256(json.dumps(data, sort_keys=True, separators=(',', ':'),
                                     allow_nan=False).encode()).hexdigest()


def short_route(library):
    """参考只取正式C采样，不另算弧线；实际每段锚定新鲜的实测起点。"""
    origin = Pose(0, 0, 0)
    trace, goals, boundaries = [], [], []
    for index, name in enumerate(CASES):
        payload = trial(name, index + 1)
        ref = NativeReference(library, origin, Spec.from_buffer_copy(payload[8:]))
        trace.extend(pose.values() for _, pose in ref.samples(.1))
        goals.append(payload.hex())
        origin = ref.end
        boundaries.append(origin.values())
    plan = dict(schema=1, id='short_combo_v1', domain='commissioning', frame_id='odom',
                cases=list(CASES), goals=goals, reference_trace=trace, boundaries=boundaries,
                executable=False, anchor_policy='measured_segment_start',
                map_registered=False, segment_handoff='stopped_and_settled')
    plan['sha256'] = digest(plan)
    validate(plan)
    return plan


def validate(plan):
    """先于打开串口校验；整场规划只能预览，不能借改标签绕过白名单。"""
    if (plan.get('schema') != 1 or plan.get('id') != 'short_combo_v1'
            or plan.get('domain') != 'commissioning' or plan.get('frame_id') != 'odom'
            or tuple(plan.get('cases', ())) != CASES or plan.get('executable') is not False
            or plan.get('map_registered') is not False
            or plan.get('anchor_policy') != 'measured_segment_start'
            or plan.get('segment_handoff') != 'stopped_and_settled'
            or plan.get('sha256') != digest(plan)):
        raise ValueError('不是本阶段冻结短路线；整图/已修改路线不能执行')
    expected = [trial(name, i + 1).hex() for i, name in enumerate(CASES)]
    if plan.get('goals') != expected:
        raise ValueError('路线目标与本阶段正式短段参数不符')
    points = plan.get('reference_trace')
    if not isinstance(points, list) or not 10 <= len(points) <= 2000:
        raise ValueError('路线缺少有界参考轨迹')
    for point in points:
        if (len(point) != 3 or any(type(v) not in (int, float) or not math.isfinite(v) for v in point)
                or math.hypot(*point[:2]) > .5 or abs(point[2]) > .22):
            raise ValueError('参考轨迹超出短组合范围')


def export_navigation(simulation):
    """直接导出8000规划器已构建的Spec，保留完整场地的未部署边界。"""
    goals = [bytes(segment.reference.spec).hex() for _, segment in simulation.actions]
    result = dict(schema=1, id='field_' + simulation.scope, frame_id='map', domain='simulation',
                  executable=False, goals=goals, goal_encoding='RefSpec88',
                  blockers=['完整外形与地图配准未验收', '比赛参数超出短段权限',
                            '连续非零速度交接未实测'])
    result['sha256'] = digest(result)
    return result
