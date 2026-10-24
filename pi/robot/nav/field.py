"""名义场地条件模型。未知尺寸和道路不会被转成已验证的实车配置。"""

import json
import math
from pathlib import Path

from .geometry import bounds, footprint, intersects, rectangle

DEFAULT_FIELD = Path(__file__).resolve().parents[3] / 'config/field_preview.json'


class Field:
    def __init__(self, path=DEFAULT_FIELD):
        self.data = json.loads(Path(path).read_text(encoding='utf-8'))
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
        errors.extend(key for key, _, zone in self.zones if not key.startswith('start_') and intersects(polygon, zone))
        return errors

    def check(self, reference, allowed_start=None):
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
