"""地图多段闭环模型；复用正式C跟踪器和独立Plant，全程不重建odom。"""

import argparse
import ctypes as c
import json
import math
from pathlib import Path
import time

from .track_sim import ClosedLoop, settings
from .odom_sim import fixture
from .device.tracking import Goal, Spec
from .nav.frames import Pose, compose, relative, angle
from .nav.planner import Planner, view
from .nav.geometry import simplify
from .nav.reference import CRUISE_M_S


def odom_pose(values):
    return Pose(values['x_m'], values['y_m'], values['yaw_rad'])


class RouteSimulation:
    def __init__(self, library, *, blocked=(), start='start_1', flow=True, scope='two_batch',
                 cruise_m_s=CRUISE_M_S, return_heading_free=True, return_region=True, compare_baseline=True):
        preparation_started = time.monotonic()
        if any(type(value) is not bool for value in (return_heading_free,return_region,compare_baseline)):
            raise ValueError('返航朝向及基准比较开关必须为布尔值')
        config = settings(fixture())
        config.position_tol_m, config.yaw_tol_rad = .003, .012
        self.model = ClosedLoop(library, config=config, auto_start=False)
        self.planner = Planner(self.model.native.lib, self.model.calibration.geometry,
                               max_rpm=self.model.config.max_rpm,cruise_m_s=cruise_m_s)
        self.field = self.planner.field
        self.start = start
        self.return_heading_free = return_heading_free
        self.return_region = return_region
        if start not in ('start_1', 'start_2'):
            raise ValueError('起点只能为启停区1/2')
        self.flow, self.scope = flow, scope
        if scope not in ('tour','two_batch'):
            raise ValueError('导航范围无效')
        initial_heading=self.planner.data['station_heading_quarters'][start]
        self.map_to_odom = self.planner.pose(start, initial_heading)
        self.roads = {row[0]: 'OPEN' for row in self.planner.data['edges']}
        if set(blocked) - set(self.roads):
            raise ValueError('封路编号无效')
        self.roads.update({key: 'BLOCKED' for key in blocked})
        self.legs = []
        self.actions = []
        node, heading = start, initial_heading
        targets=('qr','raw','rough','staging',start) if scope=='tour' else (
            'qr','raw','rough','staging','raw','rough','staging',start)
        for target in targets:
            free = return_heading_free and target == start
            park = return_region and target == start
            plan = self.planner.plan(node,target,start_heading=heading,roads=self.roads,
                                     assigned_start=start,flow=flow,free_heading=free,park_region=park)
            if flow and compare_baseline:
                baseline=self.planner.plan(node,target,start_heading=heading,roads=self.roads,
                                           assigned_start=start,flow=False,free_heading=free,park_region=park)
                plan['before_s']=baseline.get('estimate_s',0)
            plan['assumed_roads'] = True
            self.legs.append(plan)
            if plan['status'] != 'OK':
                break
            for segment in plan['segments']:
                self.actions.append((len(self.legs)-1, segment))
            node = target
            heading = plan['goal_heading']
        self.status = 'READY' if len(self.legs) == len(targets) and all(p['status'] == 'OK' for p in self.legs) else 'NO_PATH'
        self.index = 0
        self.active = False
        self.reason = ''
        self.raw_trace = []
        self.trace = []
        self.trace_count = -1
        self.results = []
        self.last_truth_ms = -1
        self.previous_truth = None
        self.inplace_turn_ms = 0
        self.moving_turn_ms = 0
        self.travel_m = 0.
        self.peak_actual_rpm = 0.
        self.origin_ms = None
        self.min_origin_consistent = True
        self.started = time.monotonic()
        self.reference_trace = []
        for _, segment in self.actions:
            self.reference_trace.extend(p.values() for _, p in segment.reference.samples(.12))
        self.view_legs = [view(p) for p in self.legs]
        points=[self.planner.data['nodes'][name] for name in (start,*targets)]
        self.minimum_distance_m=sum(math.dist(a,b) for a,b in zip(points,points[1:]))
        if return_region:
            zone = next(box for key,box,_ in self.field.zones if key == start)
            previous = points[-2]
            nearest = (min(zone[2],max(zone[0],previous[0])),min(zone[3],max(zone[1],previous[1])))
            self.minimum_distance_m += math.dist(previous,nearest)-math.dist(previous,points[-1])
        self.preparation_s = time.monotonic()-preparation_started

    def _begin(self):
        leg, segment = self.actions[self.index]
        current = odom_pose(self.model.pose)
        desired = relative(self.map_to_odom, segment.reference.end)
        spec = Spec.from_buffer_copy(bytes(segment.reference.spec))
        if spec.kind in (1,5,6):
            local = relative(current, desired)
            spec.x_m, spec.y_m = local.x_m, local.y_m
            if spec.kind in (5,6):
                planned=segment.reference.spec.yaw_rad
                spec.yaw_rad=planned+angle(desired.yaw_rad-current.yaw_rad-planned)
        elif spec.kind==4:
            actual_map=compose(self.map_to_odom,current)
            bearing=segment.reference.start.yaw_rad+spec.path_heading_rad
            spec.path_heading_rad=angle(bearing-actual_map.yaw_rad)
            planned=segment.reference.spec.yaw_rad
            spec.yaw_rad=planned+angle(desired.yaw_rad-current.yaw_rad-planned)
        elif spec.kind == 2:
            spec.yaw_rad = angle(desired.yaw_rad - current.yaw_rad)
        self.model.begin(Goal(self.index + 1, 30000, spec))
        self.model.kind = segment.kind.lower()
        self.active = True
        self.status = 'RUNNING'
        self.action_started_ms = self.model.elapsed_ms

    def _truth(self):
        allowed = self.start if self.legs[self.actions[self.index][0]]['start'] == self.start or self.legs[self.actions[self.index][0]]['goal'] == self.start else None
        # 2ms独立Plant积分，逐个覆盖真实模拟姿态；不以参考轨迹替代。
        speeds = {item[0]: item[1] for item in self.model.plant.history}
        for ms, x, y, yaw in self.model.plant.positions:
            if ms <= self.last_truth_ms:
                continue
            pose = compose(self.map_to_odom, Pose(x, y, yaw))
            if self.previous_truth:
                old_ms,old_x,old_y,old_yaw=self.previous_truth
                dt=ms-old_ms
                distance=math.hypot(x-old_x,y-old_y)
                self.travel_m+=distance
                if dt>0 and abs(yaw-old_yaw)*1000/dt>.05:
                    if distance*1000/dt<.006:
                        self.inplace_turn_ms+=dt
                    else:
                        self.moving_turn_ms+=dt
            self.previous_truth=(ms,x,y,yaw)
            # 该2ms模型区间轮速单调趋近固定命令，两端最大值界定实际位移。
            rpm = max(map(abs, (*speeds[ms], *speeds.get(ms-2, speeds[ms]))))
            self.peak_actual_rpm = max(self.peak_actual_rpm,rpm)
            wheel_m_s = rpm * 2 * math.pi * .05 / 60
            movement = (math.sqrt(2) * wheel_m_s + (self.field.radius + .04) * wheel_m_s / .22) * .002
            errors = self.field.errors(pose, allowed, movement, tracking=False)
            self.last_truth_ms = ms
            if errors:
                self.model.tracker.cancel()
                self.status, self.reason = 'FAILED', '模拟车体扫掠异常：' + ','.join(errors)
                return False
        return True

    def advance(self, report=True):
        if self.status in ('COMPLETE', 'FAILED', 'NO_PATH'):
            return self.snapshot() if report else None
        if self.model.elapsed_ms >= 240000:
            self.status, self.reason = 'FAILED', '导航模拟总期限到达'
            self.model.tracker.cancel()
            return self.snapshot() if report else None
        self.model.advance(report=False)
        if self.model.pose['valid']:
            point = compose(self.map_to_odom, odom_pose(self.model.pose)).values()
            if not self.raw_trace or math.dist(point[:2],self.raw_trace[-1][:2]) >= .001:
                self.raw_trace.append(point)
            origin = self.model.pose['origin_ms']
            if self.origin_ms is None:
                self.origin_ms = origin
            if origin != self.origin_ms:
                self.min_origin_consistent = False
                self.status, self.reason = 'FAILED', '连续里程计原点改变'
        if not self._truth():
            return self.snapshot() if report else None
        out = self.model.tracker.output()
        if self.active and out['state'] in (5, 6):
            self.status, self.reason = 'FAILED', f'跟踪器状态{out["state"]}，故障{out["fault"]}'
        elif self.active and out['state'] == 4:
            leg, segment = self.actions[self.index]
            mapped = compose(self.map_to_odom, odom_pose(self.model.pose))
            error = math.dist(mapped.values()[:2], segment.reference.end.values()[:2])
            if error > .01 or abs(angle(mapped.yaw_rad-segment.reference.end.yaw_rad)) > .04:
                self.status, self.reason = 'FAILED', '实际模拟终点与冻结地图目标不一致'
                return self.snapshot() if report else None
            if self.return_region and self.index == len(self.actions)-1:
                truth = compose(self.map_to_odom,Pose(self.model.plant.x,self.model.plant.y,self.model.plant.yaw))
                if not all(self.field.parked(pose,self.start) for pose in (mapped,truth)):
                    self.status,self.reason = 'FAILED','返航模拟主体投影未完整进入所选启停区'
                    return self.snapshot() if report else None
            self.results.append(dict(action=self.index+1, leg=leg, kind=segment.kind,
                                     elapsed_ms=self.model.elapsed_ms-self.action_started_ms,
                                     map_error_m=error, zero_ack_ms=out['zero_ack_ms'],
                                     stable_ms=out['stable_elapsed_ms'], origin_ms=self.origin_ms))
            self.raw_trace.append(mapped.values())
            self.index += 1
            self.active = False
            if self.index == len(self.actions):
                self.status = 'COMPLETE'
        if not self.active and self.status not in ('FAILED', 'COMPLETE') and self.model.elapsed_ms >= 120:
            try:
                self._begin()
            except ValueError as exc:
                self.status, self.reason = 'FAILED', str(exc)
        return self.snapshot() if report else None

    def snapshot(self):
        pose = compose(self.map_to_odom, odom_pose(self.model.pose))
        truth = compose(self.map_to_odom, Pose(self.model.plant.x, self.model.plant.y, self.model.plant.yaw))
        if self.trace_count != len(self.raw_trace):
            self.trace = simplify(self.raw_trace)
            self.trace_count = len(self.raw_trace)
        return dict(domain='simulation', frame_id='map', start=self.start, status=self.status, executable=False,
                    budget_scope=self.scope,flow=self.flow,budget_s=60,
                    return_heading_free=self.return_heading_free,
                    return_region=self.return_region,
                    return_target=self.actions[-1][1].reference.end.values() if self.actions else None,
                    return_heading_rad=self.actions[-1][1].reference.end.yaw_rad if self.actions else None,
                    preparation_s=self.preparation_s,
                    budget_complete=False,
                    missing_navigation=['工位内三个目标间移位', '视觉精对准修正'],
                    actual_pose=None, initial_pose_source='启停区1名义中心，朝向按人工确认' if self.start=='start_1'
                    else '启停区2名义中心，朝向按人工确认',
                    initial_pose=self.map_to_odom.values(), map_pose=pose.values(), truth_pose=truth.values(),
                    action=self.index+1 if self.index<len(self.actions) else self.index,
                    actions=len(self.actions), completed=len(self.results), elapsed_ms=self.model.elapsed_ms,
                    leg=self.actions[self.index][0] if self.index<len(self.actions) else len(self.legs)-1,
                    legs=self.view_legs, results=list(self.results), reason=self.reason,
                    estimate_s=sum(p.get('estimate_s', 0) for p in self.legs),
                    cruise_m_s=self.planner.cruise_m_s,minimum_distance_m=self.minimum_distance_m,
                    ideal_minimum_s=self.minimum_distance_m/self.planner.cruise_m_s,
                    peak_rpm=self.model.peak_rpm,max_rpm=self.model.config.max_rpm,
                    peak_actual_rpm=self.peak_actual_rpm,
                    inplace_actions=sum(s.kind=='TURN' for _,s in self.actions),
                    inplace_turn_ms=self.inplace_turn_ms,moving_turn_ms=self.moving_turn_ms,travel_m=self.travel_m,
                    before_s=sum(p.get('before_s', 0) for p in self.legs),
                    origin_ms=self.origin_ms, origin_consistent=self.min_origin_consistent,
                    tracking=self.model.tracker.output(), field=self.field.data, road_states=self.roads,
                    reference_trace=self.reference_trace, trace=list(self.trace), trace_tolerance_m=.0005,
                    assumptions=self.planner.data['unverified'], simulation_roads_assumed_open=True,
                    scope='两批跨区导航小计；尚缺工位内移位与视觉修正，不代表完整60秒验收' if self.scope=='two_batch'
                    else '单次空载巡回对照，不作为两批导航预算成绩')

    def run(self, limit_ms=240000):
        while self.model.elapsed_ms < limit_ms and self.status not in ('COMPLETE', 'FAILED', 'NO_PATH'):
            self.advance(report=False)
        if self.status not in ('COMPLETE', 'FAILED', 'NO_PATH'):
            self.status, self.reason = 'FAILED', '导航模拟总期限到达'
            self.model.tracker.cancel()
        return self.snapshot()


class RouteDemo:
    def __init__(self, library, cruise_m_s=CRUISE_M_S):
        self.model = RouteSimulation(library,cruise_m_s=cruise_m_s)
        self.started = time.monotonic()

    def step(self, now):
        # 演示按5倍逻辑时间推进；耗时一律展示模拟时间，非墙钟性能。
        target = int((now-self.started)*5000)
        for _ in range(16):
            if self.model.model.elapsed_ms >= target or self.model.status in ('COMPLETE', 'FAILED', 'NO_PATH'):
                break
            self.model.advance(report=False)

    def snapshot(self):
        return self.model.snapshot()


def main():
    parser = argparse.ArgumentParser(description='仅条件地图闭环模拟，不连接设备')
    parser.add_argument('--library', type=Path, required=True)
    parser.add_argument('--output', type=Path, default=Path('output/navigation/report.json'))
    parser.add_argument('--blocked', action='append', default=[])
    parser.add_argument('--start',choices=('start_1','start_2'),default='start_1')
    parser.add_argument('--scope',choices=('tour','two_batch'),default='two_batch')
    parser.add_argument('--cruise-m-s',type=float,default=CRUISE_M_S,help='仅模拟巡航档，不写实车参数')
    parser.add_argument('--fixed-return-heading',action='store_true',help='历史固定朝向对照')
    parser.add_argument('--fixed-return-position',action='store_true',help='历史固定中心点对照')
    parser.add_argument('--stopped',action='store_true',help='同朝向同速度的逐段停车基准')
    args = parser.parse_args()
    report = RouteSimulation(args.library,blocked=args.blocked,start=args.start,scope=args.scope,flow=not args.stopped,
                             cruise_m_s=args.cruise_m_s,return_heading_free=not args.fixed_return_heading,
                             return_region=not args.fixed_return_position).run()
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(report, ensure_ascii=False, indent=2), encoding='utf-8')
    print(json.dumps({key:report[key] for key in ('status','reason','completed','actions','elapsed_ms','estimate_s','origin_consistent')},ensure_ascii=False))


if __name__ == '__main__':
    main()
