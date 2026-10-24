"""带朝向道路A*及局部弧段优化。默认未知道路不通；仅条件模拟可给开放场景。"""

from dataclasses import dataclass
import heapq
import itertools
import math

from .field import Field
from .frames import Pose, angle
from .reference import NativeReference, specification, CRUISE_M_S, DOCK_M_S, SHORT_MOVE_M, SETTLE_ESTIMATE_S


@dataclass
class Segment:
    kind: str
    reference: NativeReference
    road: str
    reverse: bool = False

    @property
    def cost_s(self):
        return self.reference.duration_s + SETTLE_ESTIMATE_S  # 停稳/反馈/修正预留，非实测耗时

    def view(self):
        return dict(kind=self.kind, start=self.reference.start.values(), end=self.reference.end.values(),
                    reference_s=self.reference.duration_s, estimate_s=self.cost_s, road=self.road,
                    reverse=self.reverse)


class Planner:
    def __init__(self, library, geometry, field=None, max_rpm=100):
        self.library, self.geometry = library, geometry
        self.field = field or Field()
        self.data = self.field.data
        self.nodes = self.data['nodes']
        self.max_rpm = max_rpm
        if (geometry.layout != 1 or any(not math.isfinite(v) or v <= 0 for v in
                (geometry.radius_m, geometry.half_length_m, geometry.half_width_m, max_rpm))):
            raise ValueError('规划器需要显式有效的合成轮参数')
        self.cache = {}

    def pose(self, node, heading):
        return Pose(*self.nodes[node], heading * math.pi / 2)

    def make(self, kind, start, end, road, allowed_start, reverse=False, radius=.1):
        key = (kind, start, end, road, allowed_start, reverse, radius)
        if key in self.cache:
            return self.cache[key]
        try:
            terminal = any(math.dist(end.values()[:2], self.nodes[node]) < 1e-6
                           for node in self.field.terminals(allowed_start))
            velocity = DOCK_M_S if terminal and kind in ('LINE', 'DIAGONAL') else CRUISE_M_S
            spec = specification(kind, start, end, radius, reverse, velocity)
            reference = NativeReference(self.library, start, spec)
            # 正式C多项式变化率界推导最坏理论轮速；无法容纳的候选直接排除。
            wheel_bound = (math.sqrt(2) * reference.velocity_bound +
                           (self.geometry.half_length_m + self.geometry.half_width_m) * reference.yaw_bound)
            wheel_bound *= 60 / (2 * math.pi * self.geometry.radius_m)
            errors = ['WHEEL_LIMIT'] if wheel_bound > self.max_rpm else self.field.check(reference, allowed_start)
            value = (None, errors) if errors else (Segment(kind, reference, road, reverse), [])
        except ValueError as exc:
            value = None, [str(exc)]
        self.cache[key] = value
        return value

    def plan(self, start, goal, *, start_heading=1, goal_heading=None, roads=None,
             assigned_start='start_1', mode='simulation', smooth=True):
        result = dict(status='NO_PATH', frame_id='map', domain='simulation', executable=False,
                      deployment_blockers=list(self.data['unverified']), start=start, goal=goal,
                      segments=[], nodes=[], rejected={}, assumed_roads=False)
        if mode != 'simulation':
            return dict(result, status='NOT_READY')
        if (start not in self.nodes or goal not in self.nodes or type(start_heading) is not int
                or start_heading not in range(4) or assigned_start not in ('start_1', 'start_2')):
            return dict(result, status='INVALID_REQUEST')
        goal_heading = self.data['station_heading_quarters'].get(goal, start_heading) if goal_heading is None else goal_heading
        if type(goal_heading) is not int or goal_heading not in range(4):
            return dict(result, status='INVALID_REQUEST')
        edges = self.data['edges']
        names = {row[0] for row in edges}
        roads = {} if roads is None else dict(roads)
        if set(roads) - names or any(v not in ('OPEN', 'UNKNOWN', 'BLOCKED') for v in roads.values()):
            return dict(result, status='INVALID_REQUEST')
        allowed = tuple(n for n in (start, goal) if n.startswith('start_')) or None
        for node, heading in ((start, start_heading), (goal, goal_heading)):
            errors = self.field.errors(self.pose(node, heading), allowed)
            if errors:
                return dict(result, status='INVALID_POSE', rejected={node: errors})
        adjacency = {n: [] for n in self.nodes}
        rejected = {}
        for road, a, b, policy in edges:
            state = roads.get(road, 'UNKNOWN')
            if state != 'OPEN':
                rejected[road] = [state]
                continue
            adjacency[a].append((b, road, policy))
            adjacency[b].append((a, road, policy))
        initial = (start, start_heading)
        target = (goal, goal_heading)
        counter = itertools.count()
        frontier = [(0., 0., next(counter), initial)]
        best, came = {initial: 0.}, {}
        while frontier:
            _, cost, _, key = heapq.heappop(frontier)
            if cost > best[key] + 1e-9:
                continue
            if key == target:
                break
            node, heading = key
            pose = self.pose(node, heading)
            candidates = []
            for turn in (-1, 1):
                end = Pose(pose.x_m, pose.y_m, pose.yaw_rad + turn * math.pi / 2)
                segment, errors = self.make('TURN', pose, end, f'{node}:turn{turn}', allowed)
                if segment:
                    candidates.append(((node, (heading + turn) % 4), segment, segment.cost_s))
                else:
                    rejected[f'{node}@{heading}:turn{turn}'] = errors
            for nxt, road, policy in adjacency[node]:
                end = self.pose(nxt, heading)
                dx, dy = end.x_m - pose.x_m, end.y_m - pose.y_m
                bearing = math.atan2(dy, dx)
                forward = math.cos(angle(bearing - pose.yaw_rad))
                aligned = abs(math.sin(bearing - pose.yaw_rad)) < 1e-6
                if not aligned and policy != 'short':
                    continue
                if not aligned and math.hypot(dx, dy) > SHORT_MOVE_M:
                    rejected[road] = ['LATERAL_TOO_LONG']
                    continue
                segment, errors = self.make('LINE' if aligned else 'DIAGONAL', pose, end, road,
                                            allowed, reverse=forward < 0)
                if segment:
                    penalty = 1.0 if aligned and forward >= 0 else 1.08 if aligned else 1.25
                    candidates.append(((nxt, heading), segment, segment.cost_s * penalty))
                else:
                    rejected[f'{road}@{heading}'] = errors
            for nxt, segment, edge_cost in candidates:
                new = cost + edge_cost
                if new + 1e-9 < best.get(nxt, math.inf):
                    best[nxt], came[nxt] = new, (key, segment)
                    heuristic = math.dist(self.nodes[nxt[0]], self.nodes[goal]) / CRUISE_M_S
                    heapq.heappush(frontier, (new + heuristic, new, next(counter), nxt))
        result['rejected'] = rejected
        if target not in best:
            return result
        key, chain, node_chain = target, [], [goal]
        while key != initial:
            previous, segment = came[key]
            chain.append(segment)
            if previous[0] != key[0]:
                node_chain.append(previous[0])
            key = previous
        chain.reverse()
        node_chain.reverse()
        before = sum(segment.cost_s for segment in chain)
        # 同一节点的±2pi表示先统一为连续角，再交给冻结参考；不重置odom。
        chain = self.continuous(chain, allowed)
        chain = self.merge(chain, allowed)
        reasons = []
        if smooth:
            chain, reasons = self.smooth(chain, allowed)
        return dict(result, status='OK', nodes=node_chain, segments=chain, rejected=rejected,
                    estimate_s=sum(s.cost_s for s in chain), before_s=before, smoothing=reasons,
                    road_states=roads)

    def merge(self, chain, allowed):
        result = []
        for segment in chain:
            if result:
                last = result[-1]
                first, end = last.reference.start, segment.reference.end
                dx, dy = end.x_m - first.x_m, end.y_m - first.y_m
                if (last.kind == segment.kind == 'LINE' and last.reverse == segment.reverse
                        and abs(angle(last.reference.end.yaw_rad - end.yaw_rad)) < 1e-7
                        and abs(dx * math.sin(first.yaw_rad) - dy * math.cos(first.yaw_rad)) < 1e-6):
                    merged, _ = self.make('LINE', first, end, last.road + '+' + segment.road, allowed, last.reverse)
                    if merged:
                        result[-1] = merged
                        continue
            result.append(segment)
        return result

    def continuous(self, chain, allowed):
        result = []
        for segment in chain:
            start = segment.reference.start
            if result:
                start = result[-1].reference.end
            end = segment.reference.end
            if segment.kind == 'TURN':
                end = Pose(end.x_m, end.y_m, start.yaw_rad + angle(end.yaw_rad - segment.reference.start.yaw_rad))
            else:
                end = Pose(end.x_m, end.y_m, start.yaw_rad)
            rebuilt, errors = self.make(segment.kind, start, end, segment.road, allowed, segment.reverse)
            if not rebuilt:
                raise ValueError(f'连续角转换失效：{errors}')
            result.append(rebuilt)
        return result

    def smooth(self, chain, allowed):
        result, reasons = [], []
        index = 0
        while index < len(chain):
            group = chain[index:index+3]
            best = None
            errors_seen = set()
            if (len(group) == 3 and [s.kind for s in group] == ['LINE', 'TURN', 'LINE']
                    and group[0].reverse == group[2].reverse):
                a, turn, b = group
                for radius in (.04, .06, .08, .10, .12, .16, .20):
                    direction = turn.reference.end.yaw_rad - turn.reference.start.yaw_rad
                    spec = specification('ARC', turn.reference.start, turn.reference.end, radius, a.reverse)
                    try:
                        local = NativeReference(self.library, Pose(0, 0, 0), spec)
                    except ValueError as exc:
                        errors_seen.add(str(exc))
                        continue
                    distance = abs(local.end.x_m)
                    if min(math.dist(a.reference.start.values()[:2], a.reference.end.values()[:2]),
                           math.dist(b.reference.start.values()[:2], b.reference.end.values()[:2])) <= distance + .02:
                        continue
                    sign = -1 if a.reverse else 1
                    corner = turn.reference.start
                    entry = Pose(corner.x_m - sign * distance * math.cos(corner.yaw_rad),
                                 corner.y_m - sign * distance * math.sin(corner.yaw_rad), corner.yaw_rad)
                    exit = Pose(corner.x_m + sign * distance * math.cos(corner.yaw_rad + direction),
                                corner.y_m + sign * distance * math.sin(corner.yaw_rad + direction), corner.yaw_rad + direction)
                    proposals = [self.make('LINE', a.reference.start, entry, a.road, allowed, a.reverse),
                                 self.make('ARC', entry, exit, turn.road, allowed, a.reverse, radius),
                                 self.make('LINE', exit, b.reference.end, b.road, allowed, b.reverse)]
                    errors_seen.update(e for _, errors in proposals for e in errors)
                    if any(s is None for s, _ in proposals):
                        continue
                    candidates = [s for s, _ in proposals]
                    cost = sum(s.cost_s for s in candidates)
                    if cost < sum(s.cost_s for s in group) and (best is None or cost < best[0]):
                        best = cost, candidates, radius
                if best:
                    result.extend(best[1])
                    reasons.append(dict(node=turn.road, selected_radius_m=best[2], reason='扫掠/轮速通过且预计更快'))
                    index += 3
                    continue
                reasons.append(dict(node=turn.road, selected_radius_m=None,
                                    reason='保留停车转向', rejected=sorted(errors_seen)))
            result.append(chain[index])
            index += 1
        return result, reasons


def view(plan):
    return {key: [s.view() for s in value] if key == 'segments' else value for key, value in plan.items()}
