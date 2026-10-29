"""名义场地条件模型。未知尺寸和道路不会被转成已验证的实车配置。"""

import json
import copy
import math
from pathlib import Path

from .geometry import bounds, footprint, intersects, rectangle, circle_intersects, hull

DEFAULT_FIELD = Path(__file__).resolve().parents[3] / 'config/field_preview.json'


class Field:
    def __init__(self, path=DEFAULT_FIELD, *, data=None):
        self.data = copy.deepcopy(data) if data is not None else json.loads(Path(path).read_text(encoding='utf-8'))
        if self.data.get('frame_id') != 'map' or self.data.get('domain') != 'simulation':
            raise ValueError('场地坐标或数据域不匹配，不能隐式配准')
        self.body = self.data['body']
        values = [*self.data['bounds_m'], self.data['margin_m'], self.data['tracking_margin_m'],
                  *(self.body[key] for key in ('front_m', 'rear_m', 'left_m', 'right_m'))]
        if any(type(v) not in (int, float) or not math.isfinite(v) or v < 0 for v in values):
            raise ValueError('场地/包络参数无效')
        x0, y0, x1, y1 = self.data['bounds_m']
        if x0 >= x1 or y0 >= y1 or any(self.body[k] <= 0 for k in ('front_m','rear_m','left_m','right_m')):
            raise ValueError('场地或车体范围为空')
        self.zones = [(z['id'], z['bounds_m'], rectangle(z['bounds_m'])) for z in self.data['zones']]
        self.obstacles = self.data.get('obstacles', [])
        for item in self.obstacles:
            if any(type(item.get(key)) not in (int, float) or not math.isfinite(item[key])
                   for key in ('x_m', 'y_m', 'diameter_m')) or not .01 <= item['diameter_m'] <= .3:
                raise ValueError('圆形障碍物数据无效')
        self.radius = math.hypot(max(self.body['front_m'], self.body['rear_m']),
                                 max(self.body['left_m'], self.body['right_m']))

    @staticmethod
    def terminals(allowed_start):
        return (allowed_start,) if isinstance(allowed_start, str) else tuple(allowed_start or ())

    def parking(self, key, approach, clearance_m=.004):
        """名义主体完整投影停入蓝区；4mm含3mm到位容差，不代表完整突出物验收。"""
        from .frames import Pose
        zone = next((box for name,box,_ in self.zones if name == key and name.startswith('start_')),None)
        if zone is None:
            return {}
        result = {}
        for heading in range(4):
            shape = bounds(footprint(Pose(0,0,heading*math.pi/2),self.body))
            low_x,low_y = zone[0]+clearance_m-shape[0],zone[1]+clearance_m-shape[1]
            high_x,high_y = zone[2]-clearance_m-shape[2],zone[3]-clearance_m-shape[3]
            if low_x > high_x or low_y > high_y:
                continue
            result[heading] = Pose(min(high_x,max(low_x,approach[0])),
                                   min(high_y,max(low_y,approach[1])),heading*math.pi/2)
        return result

    def parked(self, pose, key):
        zone = next((box for name,box,_ in self.zones if name == key and name.startswith('start_')),None)
        if zone is None:
            return False
        shape = bounds(footprint(pose,self.body))
        return (shape[0] >= zone[0]-1e-9 and shape[1] >= zone[1]-1e-9 and
                shape[2] <= zone[2]+1e-9 and shape[3] <= zone[3]+1e-9)

    def compact(self, pose, allowed_start):
        return any(key in self.terminals(allowed_start) and b[0] <= pose.x_m <= b[2] and b[1] <= pose.y_m <= b[3]
                   for key, b, _ in self.zones)

    def errors(self, pose, allowed_start=None, movement=0, compact_end=None, tracking=True):
        compact = self.compact(pose, allowed_start)
        if compact_end is not None:
            compact = compact and self.compact(compact_end, allowed_start)
        margin = self.data['margin_m'] + (self.data['tracking_margin_m'] if tracking else 0)
        polygon = footprint(pose, self.body, margin, movement)
        # 仅蓝区内部起步/返回允许名义主体贴近场界，明确不证明20mm余量。
        boundary = footprint(pose, self.body, 0, movement) if compact else polygon
        a, b = bounds(boundary), self.data['bounds_m']
        errors = ['FIELD_BOUNDARY'] if a[0] < b[0]-1e-9 or a[1] < b[1]-1e-9 or a[2] > b[2]+1e-9 or a[3] > b[3]+1e-9 else []
        # 蓝色是可通行的地面标记；仅黄区、工作台包络参与禁入碰撞。
        # 同一车体包围盒只算一次；静态工作区的包围盒在加载时已知。
        # 仅跳过确定分离的候选，重叠时仍做原SAT检查，不降低转弯扫掠精度。
        box = bounds(polygon) if compact else a
        errors.extend(key for key, zone_box, zone in self.zones if not key.startswith('start_')
                      and intersects(polygon, zone, boxes=(box, zone_box)))
        errors.extend(f'obstacle:{item["id"]}' for item in self.obstacles
                      if circle_intersects(polygon, item['x_m'], item['y_m'], item['diameter_m'] / 2))
        return errors

    def _clear_envelope(self, polygon):
        box, field = bounds(polygon), self.data['bounds_m']
        return (box[0] >= field[0] and box[1] >= field[1] and box[2] <= field[2] and box[3] <= field[3]
                and not any(intersects(polygon, zone, boxes=(box, zone_box))
                            for key, zone_box, zone in self.zones if not key.startswith('start_'))
                and not any(circle_intersects(polygon, item['x_m'], item['y_m'], item['diameter_m']/2)
                            for item in self.obstacles))

    def _proven_clear(self, reference, allowed_start):
        """只提前接受可证明无碰撞的完整包络；证明失败仍走原自适应细分。"""
        spec = reference.spec
        margin = self.data['margin_m'] + self.data['tracking_margin_m']
        extra = .00152  # 不小于原细分的末级包络，保留数值余量。
        if spec.kind in (1, 5, 6) and abs(reference.end.yaw_rad-reference.start.yaw_rad) < 1e-10:
            if self.compact(reference.start, allowed_start) or self.compact(reference.end, allowed_start):
                return False
            polygon = hull(footprint(reference.start,self.body,margin,extra)
                           + footprint(reference.end,self.body,margin,extra))
            return self._clear_envelope(polygon)
        if spec.kind == 2:
            x, y = reference.start.x_m, reference.start.y_m
            radius = math.hypot(max(self.body['front_m'],self.body['rear_m'])+margin,
                                max(self.body['left_m'],self.body['right_m'])+margin)+math.sqrt(2)*extra
            box = self.data['bounds_m']
            return (x-radius >= box[0] and y-radius >= box[1] and x+radius <= box[2] and y+radius <= box[3]
                    and not any(circle_intersects(zone,x,y,radius) for key,_,zone in self.zones
                                if not key.startswith('start_'))
                    and not any(math.hypot(x-item['x_m'],y-item['y_m']) <= radius+item['diameter_m']/2
                                for item in self.obstacles))
        return False

    def check(self, reference, allowed_start=None, *, fast=True):
        if fast and self._proven_clear(reference, allowed_start):
            return []
        margin = self.data['margin_m'] + self.data['tracking_margin_m']
        radius = math.hypot(max(self.body['front_m'], self.body['rear_m']) + margin,
                            max(self.body['left_m'], self.body['right_m']) + margin)
        rate = reference.velocity_bound + radius * reference.yaw_bound
        errors=self.errors(reference.end,allowed_start)
        if errors:
            return errors
        pending=[(0.,reference.start,reference.duration_s,reference.end)]
        while pending:
            at,pose,next_at,next_pose=pending.pop()
            movement=rate*(next_at-at)+.00002
            errors=self.errors(pose,allowed_start,movement,next_pose)
            if not errors:
                continue
            if movement<=.00152 or self.errors(pose,allowed_start):
                return errors
            middle=(at+next_at)/2
            point=reference.at(middle)
            pending.extend(((middle,point,next_at,next_pose),(at,pose,middle,point)))
        return []
