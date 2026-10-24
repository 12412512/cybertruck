"""带朝向道路A*及局部弧段优化。默认未知道路不通；仅条件模拟可给开放场景。"""

from dataclasses import dataclass, replace
import heapq
import itertools
import math

from .field import Field
from .frames import Pose, angle
from .reference import NativeReference, specification, CRUISE_M_S, SETTLE_ESTIMATE_S, flow_specification


@dataclass
class Segment:
    kind: str
    reference: NativeReference
    road: str
    reverse: bool = False
    corner: Pose | None = None

    @property
    def cost_s(self):
        return self.reference.duration_s + SETTLE_ESTIMATE_S  # 停稳/反馈/修正预留，非实测耗时

    def view(self):
        return dict(kind=self.kind, start=self.reference.start.values(), end=self.reference.end.values(),
                    reference_s=self.reference.duration_s, estimate_s=self.cost_s, road=self.road,
                    reverse=self.reverse,cruise_m_s=self.reference.spec.linear.velocity)


class Planner:
    def __init__(self, library, geometry, field=None, max_rpm=100, cruise_m_s=CRUISE_M_S):
        self.library, self.geometry = library, geometry
        self.field = field or Field()
        self.data = self.field.data
        self.nodes = self.data['nodes']
        self.max_rpm = max_rpm
        if not math.isfinite(cruise_m_s) or not 0 < cruise_m_s <= 2:
            raise ValueError('模拟巡航速度必须为(0,2]m/s；不代表实车许可值')
        self.cruise_m_s = cruise_m_s
        if (geometry.layout != 1 or any(not math.isfinite(v) or v <= 0 for v in
                (geometry.radius_m, geometry.half_length_m, geometry.half_width_m, max_rpm))):
            raise ValueError('规划器需要显式有效的合成轮参数')
        self.cache = {}

    def wheel_bound(self, reference):
        """同一时段的保守轮速界；转姿窗口外wz为零，不叠加不同时的峰值。"""
        linear = math.sqrt(2) * reference.velocity_bound
        rotating = math.sqrt(2) * reference.turning_velocity_bound
        radius = self.geometry.half_length_m + self.geometry.half_width_m
        return max(linear, rotating + radius * reference.yaw_bound) * 60 / (
            2 * math.pi * self.geometry.radius_m)

    def pose(self, node, heading):
        return Pose(*self.nodes[node], heading * math.pi / 2)

    def make(self, kind, start, end, road, allowed_start, reverse=False, radius=.1, corner=None, window_m=.08):
        key = (kind, start, end, allowed_start, reverse, radius, corner, window_m)
        if key in self.cache:
            segment, errors = self.cache[key]
            return (replace(segment,road=road) if segment else None), errors
        try:
            errors = self.field.errors(start,allowed_start) + self.field.errors(end,allowed_start)
            if errors:
                self.cache[key] = None, sorted(set(errors))
                return self.cache[key]
            terminal = self.field.compact(end,allowed_start)
            if kind in ('POSE','FLOW','WINDOW') or (terminal and kind in ('LINE','DIAGONAL')):
                spec = flow_specification(self.library,kind,start,end,corner,radius,terminal,window_m,
                                          velocity=self.cruise_m_s)
            else:
                spec = specification(kind,start,end,radius,reverse,self.cruise_m_s)
            for attempt in range(4):
                reference = NativeReference(self.library,start,spec)
                wheel_bound = self.wheel_bound(reference)
                if wheel_bound <= self.max_rpm or spec.kind == 2 or attempt == 3:
                    break
                # 请求值是上限，不是每段必须达到的速度。保留几何候选并降档，
                # 避免提高全局档位反而删掉原本可行的连续弯道、增加停车。
                wheel_m_s = self.max_rpm * 2 * math.pi * self.geometry.radius_m / 60
                radius = self.geometry.half_length_m + self.geometry.half_width_m
                remaining = wheel_m_s-radius*reference.yaw_bound
                overall = math.sqrt(2)*reference.velocity_bound/spec.linear.velocity
                active = math.sqrt(2)*reference.turning_velocity_bound/spec.linear.velocity
                cap = wheel_m_s/overall if overall else spec.linear.velocity
                if active:
                    cap = min(cap,remaining/active)
                cap *= .995
                if cap <= 0 or cap >= spec.linear.velocity*(1-1e-6):
                    break
                spec.linear.velocity = cap
                spec.curve_velocity_m_s = min(spec.curve_velocity_m_s,cap)
                spec.tail_velocity_m_s = min(spec.tail_velocity_m_s,cap)
            errors = ['WHEEL_LIMIT'] if wheel_bound > self.max_rpm else self.field.check(reference, allowed_start)
            value = (None, errors) if errors else (Segment(kind, reference, road, reverse, corner), [])
        except ValueError as exc:
            value = None, [str(exc)]
        self.cache[key] = value
        return value

    def plan(self, start, goal, *, start_heading=None, goal_heading=None, roads=None,
             assigned_start='start_1', mode='simulation', smooth=True, flow=True,
             free_heading=False,park_region=False):
        result = dict(status='NO_PATH', frame_id='map', domain='simulation', executable=False,
                      deployment_blockers=list(self.data['unverified']), start=start, goal=goal,
                      segments=[], nodes=[], rejected={}, assumed_roads=False)
        if start_heading is None:
            start_heading = self.data['station_heading_quarters'].get(start,0)
        if mode != 'simulation':
            return dict(result, status='NOT_READY')
        if (start not in self.nodes or goal not in self.nodes or type(start_heading) is not int
                or start_heading not in range(4) or assigned_start not in ('start_1', 'start_2')):
            return dict(result, status='INVALID_REQUEST')
        goal_heading = self.data['station_heading_quarters'].get(goal, start_heading) if goal_heading is None else goal_heading
        if (type(goal_heading) is not int or goal_heading not in range(4) or
                type(free_heading) is not bool or type(park_region) is not bool):
            return dict(result, status='INVALID_REQUEST')
        goal_headings = set(range(4)) if free_heading else {goal_heading}
        edges = self.data['edges']
        names = {row[0] for row in edges}
        roads = {} if roads is None else dict(roads)
        if set(roads) - names or any(v not in ('OPEN', 'UNKNOWN', 'BLOCKED') for v in roads.values()):
            return dict(result, status='INVALID_REQUEST')
        goal_poses = {h:self.pose(goal,h) for h in goal_headings}
        if park_region:
            entrances = [b if a == goal else a for _,a,b,_ in edges if goal in (a,b)]
            if not goal.startswith('start_') or not entrances or start == goal:
                return dict(result,status='INVALID_REQUEST')
            parking = self.field.parking(goal,self.nodes[entrances[0]])
            goal_poses = {h:pose for h,pose in parking.items() if h in goal_headings}
            goal_headings = set(goal_poses)

        def pose_at(node,heading):
            return goal_poses.get(heading,self.pose(node,heading)) if node == goal else self.pose(node,heading)

        def ends_at(node,start_pose,heading):
            point = pose_at(node,heading)
            delta = angle(point.yaw_rad-start_pose.yaw_rad)
            yield Pose(point.x_m,point.y_m,start_pose.yaw_rad+delta)
            if abs(abs(delta)-math.pi) < 1e-6:
                yield Pose(point.x_m,point.y_m,start_pose.yaw_rad-delta)
        # 重叠路段共享通行证据，不能借另一条边的名字绕过封路或未知状态。
        for group in self.data.get('shared_roads', []):
            state = 'BLOCKED' if any(roads.get(r, 'UNKNOWN') == 'BLOCKED' for r in group) else (
                'UNKNOWN' if any(roads.get(r, 'UNKNOWN') != 'OPEN' for r in group) else 'OPEN')
            roads.update({road: state for road in group})
        allowed = tuple(n for n in (start, goal) if n.startswith('start_')) or None
        errors = self.field.errors(self.pose(start,start_heading),allowed)
        if errors:
            return dict(result,status='INVALID_POSE',rejected={start:errors})
        goal_headings = {h for h in goal_headings if not self.field.errors(goal_poses[h],allowed)}
        if not goal_headings:
            return dict(result,status='INVALID_POSE',rejected={goal:['NO_SAFE_HEADING']})
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
        target = None
        counter = itertools.count()
        frontier = [(0., 0., next(counter), initial)]
        best, came = {initial: 0.}, {}
        while frontier:
            _, cost, _, key = heapq.heappop(frontier)
            if cost > best[key] + 1e-9:
                continue
            if key[0] == goal and key[1] in goal_headings:
                target = key
                break
            node, heading = key
            pose = pose_at(node, heading)
            candidates = []
            for turn in (-1, 1):
                end = Pose(pose.x_m, pose.y_m, pose.yaw_rad + turn * math.pi / 2)
                segment, errors = self.make('TURN', pose, end, f'{node}:turn{turn}', allowed)
                if segment:
                    candidates.append(((node, (heading + turn) % 4), segment, segment.cost_s))
                else:
                    rejected[f'{node}@{heading}:turn{turn}'] = errors
            for nxt, road, policy in adjacency[node]:
                headings = {heading} | goal_headings if flow else {heading}
                for final_heading in headings:
                    if park_region and nxt == goal and final_heading not in goal_headings:
                        continue
                    for end in ends_at(nxt,pose,final_heading):
                        aligned = abs(math.sin(math.atan2(end.y_m-pose.y_m,end.x_m-pose.x_m)-pose.yaw_rad)) < 1e-6
                        kind = 'POSE' if final_heading != heading else 'LINE' if aligned else 'DIAGONAL'
                        segment, errors = self.make(kind,pose,end,road,allowed)
                        if segment:
                            candidates.append(((nxt,final_heading),segment,segment.cost_s))
                        else:
                            rejected[f'{road}@{heading}->{final_heading}'] = errors
                if flow:
                    for final, second_road, _ in adjacency[nxt]:
                        if final == node:
                            continue
                        middle = pose_at(nxt,heading)
                        roads_used = road+'+'+second_road
                        headings = {heading} | goal_headings if final == goal or nxt == 'center' else {heading}
                        for final_heading in headings:
                            if park_region and final == goal and final_heading not in goal_headings:
                                continue
                            for end in ends_at(final,pose,final_heading):
                                direct, _ = self.make('POSE' if final_heading != heading else 'DIAGONAL',
                                                      pose,end,roads_used,allowed)
                                if direct:
                                    candidates.append(((final,final_heading),direct,direct.cost_s))
                                first_bearing = math.atan2(middle.y_m-pose.y_m,middle.x_m-pose.x_m)
                                last_bearing = math.atan2(end.y_m-middle.y_m,end.x_m-middle.x_m)
                                bend = angle(last_bearing-first_bearing)
                                if abs(bend) < 1e-5 and final_heading != heading and nxt == 'center':
                                    for width in (.08,.04,.02,.01,.006):
                                        window, _ = self.make('WINDOW',pose,end,roads_used,allowed,corner=middle,window_m=width)
                                        if window:
                                            candidates.append(((final,final_heading),window,window.cost_s))
                                elif abs(abs(bend)-math.pi/2) < 1e-5:
                                    best_curve = None
                                    for radius in (.04,.08,.12,.16,.20):
                                        curve, _ = self.make('FLOW',pose,end,roads_used,allowed,
                                                             radius=radius,corner=middle)
                                        if curve and (best_curve is None or curve.cost_s < best_curve.cost_s):
                                            best_curve = curve
                                    if best_curve:
                                        candidates.append(((final,final_heading),best_curve,best_curve.cost_s))
            for nxt, segment, edge_cost in candidates:
                new = cost + edge_cost
                if new + 1e-9 < best.get(nxt, math.inf):
                    best[nxt], came[nxt] = new, (key, segment)
                    heuristic = 0 if nxt[0] == goal else min(math.dist(self.nodes[nxt[0]],
                        goal_poses[h].values()[:2]) for h in goal_headings)/self.cruise_m_s
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
        # 跨多条边的自由捷径只用于全部道路已开放的条件模型。
        # 存在封路/未知时保留搜索得到的局部连接，不能用几何可见性跨回关闭走廊。
        if flow and all(roads.get(name,'UNKNOWN') == 'OPEN' for name in names):
            chain = self.shortcut(chain,allowed)
        reasons = []
        if smooth and not flow:
            # 基准保留工位间停车，避免把非法的侧移当成切线弯道。
            reasons = []
        return dict(result, status='OK', nodes=node_chain, segments=chain, rejected=rejected,
                    free_heading=free_heading,goal_heading=target[1],cruise_m_s=self.cruise_m_s,
                    park_region=park_region,goal_pose=chain[-1].reference.end.values() if chain else pose_at(*target).values(),
                    estimate_s=sum(s.cost_s for s in chain), before_s=before, smoothing=reasons,
                    road_states=roads)

    def shortcut(self,chain,allowed):
        """只消除同一工位间的非必要中间点；真实任务停靠点不跨越。"""
        result=[]
        index=0
        while index<len(chain):
            selected=None
            for last in range(index+1,len(chain)):
                group=chain[index:last+1]
                roads='+'.join(dict.fromkeys('+'.join(s.road for s in group).split('+')))
                move,_=self.make('POSE',group[0].reference.start,group[-1].reference.end,roads,allowed)
                start,end=group[0].reference.start,group[-1].reference.end
                if abs(start.x_m-end.x_m)>.08 and abs(start.y_m-end.y_m)>.08:
                    for corner in (Pose(start.x_m,end.y_m,start.yaw_rad),Pose(end.x_m,start.y_m,start.yaw_rad)):
                        for radius in (.04,.08,.12,.16,.20):
                            curve,_=self.make('FLOW',start,end,roads,allowed,corner=corner,radius=radius)
                            if curve and (move is None or curve.cost_s<move.cost_s):
                                move=curve
                gain=sum(s.cost_s for s in group)-move.cost_s if move else 0
                if gain>1e-6 and (selected is None or gain>selected[0]):
                    selected=gain,last,move
            if selected:
                result.append(selected[2])
                index=selected[1]+1
            else:
                result.append(chain[index])
                index+=1
        return result

    def merge(self, chain, allowed):
        result = []
        for segment in chain:
            if result:
                last = result[-1]
                first, end = last.reference.start, segment.reference.end
                dx, dy = end.x_m - first.x_m, end.y_m - first.y_m
                first_dx = last.reference.end.x_m-first.x_m
                first_dy = last.reference.end.y_m-first.y_m
                if (last.kind in ('LINE','DIAGONAL') and segment.kind in ('LINE','DIAGONAL')
                        and abs(angle(last.reference.end.yaw_rad-end.yaw_rad)) < 1e-7
                        and abs(first_dx*dy-first_dy*dx) < 1e-6
                        and first_dx*dx+first_dy*dy > first_dx*first_dx+first_dy*first_dy):
                    merged, _ = self.make('LINE', first, end, last.road + '+' + segment.road, allowed, last.reverse)
                    if merged:
                        result[-1] = merged
                        continue
            result.append(segment)
        return result

    def ends(self,node,start,heading):
        delta = angle(heading*math.pi/2-start.yaw_rad)
        yield Pose(*self.nodes[node],start.yaw_rad+delta)
        if abs(abs(delta)-math.pi)<1e-6:
            yield Pose(*self.nodes[node],start.yaw_rad-delta)

    def continuous(self, chain, allowed):
        result = []
        for segment in chain:
            start = result[-1].reference.end if result else segment.reference.start
            reference = NativeReference(self.library,start,segment.reference.spec)
            errors = self.field.check(reference,allowed)
            if errors:
                raise ValueError(f'连续角转换失效：{errors}')
            result.append(Segment(segment.kind,reference,segment.road,segment.reverse,segment.corner))
        return result



def view(plan):
    return {key: [s.view() for s in value] if key == 'segments' else value for key, value in plan.items()}
