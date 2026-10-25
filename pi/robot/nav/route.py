"""冻结正式RefSpec和参考轨迹；同一格式用于模拟、预览与执行准备。"""

import ctypes as c
import hashlib
import json
import math

from ..device.execution import trial
from ..device.tracking import Spec
from .frames import Pose, relative, angle
from .reference import NativeReference

CASES = ('straight', 'diagonal', 'arc', 'pose')


def anchor_spec(spec, planned_start, desired, current):
    """复用离线规划器的局部目标转换；只允许小幅纠偏，不重新规划。"""
    result = Spec.from_buffer_copy(bytes(spec))
    if math.dist(current.values()[:2], planned_start.values()[:2]) > .01 or abs(angle(current.yaw_rad-planned_start.yaw_rad)) > .03:
        raise ValueError('整圈段间累计误差超限，停止而不是放宽目标')
    if spec.kind in (1, 5, 6):
        local = relative(current, desired)
        result.x_m, result.y_m = local.x_m, local.y_m
    if spec.kind in (4, 5, 6):
        result.yaw_rad = spec.yaw_rad + angle(desired.yaw_rad-current.yaw_rad-spec.yaw_rad)
    if spec.kind == 4:
        result.path_heading_rad = angle(planned_start.yaw_rad+spec.path_heading_rad-current.yaw_rad)
    elif spec.kind == 2:
        result.yaw_rad = angle(desired.yaw_rad-current.yaw_rad)
    if (abs(result.x_m-spec.x_m) > .03 or abs(result.y_m-spec.y_m) > .03
            or abs(result.yaw_rad-spec.yaw_rad) > .04
            or abs(angle(result.path_heading_rad-spec.path_heading_rad)) > .04):
        raise ValueError(f'局部目标修正超限: dx={result.x_m-spec.x_m},dy={result.y_m-spec.y_m},yaw={result.yaw_rad-spec.yaw_rad},bearing={angle(result.path_heading_rad-spec.path_heading_rad)}')
    return result


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
    if plan.get('id') == 'start2_workbench_v1':
        if (plan.get('sha256') != 'f565c61af4576b3473fde21d95a542398ac18519d4b3e3d51c093d6c298c862b'
                or digest(plan) != plan['sha256']
                or plan.get('cases') != [f'start2_{i+1}' for i in range(10)]
                or plan.get('map_origin') != [2.25, 2.25, math.pi]):
            raise ValueError('不是本次保存的启停区2冻结路线')
        encoded = b''.join(bytes.fromhex(value) for value in plan['goals'])
        if hashlib.sha256(encoded).hexdigest() != '2029dea003aa80ae5091301106dcd1373beb227583de3ec644096db140432fa2':
            raise ValueError('启停区2目标摘要不一致')
        return
    if plan.get('id') in ('start1_qr_raw_v1', 'start1_tour_v1', 'start1_two_batch_v1'):
        batch = plan['id'] == 'start1_two_batch_v1'
        tour = plan['id'] in ('start1_tour_v1', 'start1_two_batch_v1')
        cases = [f'batch_{i+1}' for i in range(9)] if batch else [f'tour_{i+1}' for i in range(6)] if tour else ['qr_approach', 'raw_approach']
        expected_hash = '467916755fd05712813ea7c5a66da4ee092fbe796a00e81185b84afe304d4abf' if batch else '86737aad38dd12d6531c3f0d14eca02747b4c829710f56653167a82e9b6bd66b' if tour else '7aff8ec9099ed15cd4a7015eb30c0f6ac0f4b5e0db8b67f1ef89df39ead3272a'
        if (plan.get('schema') != 1 or plan.get('domain') != 'commissioning'
                or plan.get('frame_id') != 'odom' or plan.get('executable') is not False
                or plan.get('map_registered') is not False
                or plan.get('cases') != cases
                or plan.get('segment_handoff') != 'stopped_and_settled'
                or plan.get('anchor_policy') != 'measured_segment_start'
                or plan.get('sha256') != digest(plan)):
            raise ValueError('原料路线包格式或摘要不匹配')
        try:
            goals = b''.join(bytes.fromhex(v) for v in plan['goals'])
        except (ValueError, KeyError, TypeError) as exc:
            raise ValueError('原料路线目标编码无效') from exc
        if hashlib.sha256(goals).hexdigest() != expected_hash:
            raise ValueError('不是冻结的两段原料目标')
        points = plan.get('reference_trace')
        if (not isinstance(points, list) or not 10 <= len(points) <= 2000
                or any(not isinstance(point, list) or len(point) != 3
                       or any(type(v) not in (int, float) or not math.isfinite(v) or abs(v) > (16 if batch else 8 if tour else 4) for v in point)
                       for point in points)):
            raise ValueError('原料路线缺少有界显示参考')
        return
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
