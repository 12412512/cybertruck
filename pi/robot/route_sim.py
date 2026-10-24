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


def odom_pose(values):
    return Pose(values['x_m'], values['y_m'], values['yaw_rad'])


class RouteSimulation:
    def __init__(self, library, *, blocked=(), start='start_1'):
        config = settings(fixture())
        config.position_tol_m, config.yaw_tol_rad = .003, .012
        self.model = ClosedLoop(library, config=config, auto_start=False)
        self.planner = Planner(self.model.native.lib, self.model.calibration.geometry, max_rpm=self.model.config.max_rpm)
        self.field = self.planner.field
        self.start = start
        if start not in ('start_1', 'start_2'):
            raise ValueError('起点只能为启停区1/2')
        self.map_to_odom = self.planner.pose(start, 1)
        self.roads = {row[0]: 'OPEN' for row in self.planner.data['edges']}
        if set(blocked) - set(self.roads):
            raise ValueError('封路编号无效')
        self.roads.update({key: 'BLOCKED' for key in blocked})
        self.legs = []
        self.actions = []
        node, heading = start, 1
        for target in ('qr', 'raw', 'rough', 'staging', start):
            plan = self.planner.plan(node, target, start_heading=heading, roads=self.roads, assigned_start=start)
            plan['assumed_roads'] = True
            self.legs.append(plan)
            if plan['status'] != 'OK':
                break
            for segment in plan['segments']:
                self.actions.append((len(self.legs)-1, segment))
            node = target
            heading = self.planner.data['station_heading_quarters'][target]
        self.status = 'READY' if len(self.legs) == 5 and all(p['status'] == 'OK' for p in self.legs) else 'NO_PATH'
        self.index = 0
        self.active = False
        self.reason = ''
        self.raw_trace = []
        self.trace = []
        self.trace_count = -1
        self.results = []
        self.last_truth_ms = -1
        self.origin_ms = None
        self.min_origin_consistent = True
        self.started = time.monotonic()
        self.reference_trace = []
        for _, segment in self.actions:
            self.reference_trace.extend(p.values() for _, p in segment.reference.samples(.12))
        self.view_legs = [view(p) for p in self.legs]

    def _begin(self):
        leg, segment = self.actions[self.index]
        current = odom_pose(self.model.pose)
        desired = relative(self.map_to_odom, segment.reference.end)
        spec = Spec.from_buffer_copy(bytes(segment.reference.spec))
        if spec.kind == 1:
            local = relative(current, desired)
            spec.x_m, spec.y_m = local.x_m, local.y_m
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
            # 该2ms模型区间轮速单调趋近固定命令，两端最大值界定实际位移。
            rpm = max(map(abs, (*speeds[ms], *speeds.get(ms-2, speeds[ms]))))
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
                    actual_pose=None, initial_pose_source='启停区1名义中心，朝向按人工确认' if self.start=='start_1'
                    else '启停区2位置与朝向均为独立合成假设',
                    initial_pose=self.map_to_odom.values(), map_pose=pose.values(), truth_pose=truth.values(),
                    action=self.index+1 if self.index<len(self.actions) else self.index,
                    actions=len(self.actions), completed=len(self.results), elapsed_ms=self.model.elapsed_ms,
                    leg=self.actions[self.index][0] if self.index<len(self.actions) else len(self.legs)-1,
                    legs=self.view_legs, results=list(self.results), reason=self.reason,
                    estimate_s=sum(p.get('estimate_s', 0) for p in self.legs),
                    before_s=sum(p.get('before_s', 0) for p in self.legs),
                    origin_ms=self.origin_ms, origin_consistent=self.min_origin_consistent,
                    tracking=self.model.tracker.output(), field=self.field.data, road_states=self.roads,
                    reference_trace=self.reference_trace, trace=list(self.trace), trace_tolerance_m=.0005,
                    assumptions=self.planner.data['unverified'], simulation_roads_assumed_open=True,
                    scope='仅导航巡回，不推进扫码/机构任务或物理成功账本')

    def run(self, limit_ms=240000):
        while self.model.elapsed_ms < limit_ms and self.status not in ('COMPLETE', 'FAILED', 'NO_PATH'):
            self.advance(report=False)
        if self.status not in ('COMPLETE', 'FAILED', 'NO_PATH'):
            self.status, self.reason = 'FAILED', '导航模拟总期限到达'
            self.model.tracker.cancel()
        return self.snapshot()


class RouteDemo:
    def __init__(self, library):
        self.model = RouteSimulation(library)
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
    args = parser.parse_args()
    report = RouteSimulation(args.library, blocked=args.blocked).run()
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(report, ensure_ascii=False, indent=2), encoding='utf-8')
    print(json.dumps({key:report[key] for key in ('status','reason','completed','actions','elapsed_ms','estimate_s','origin_consistent')},ensure_ascii=False))


if __name__ == '__main__':
    main()
