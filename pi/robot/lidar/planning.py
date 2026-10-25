"""完整扫描转模拟圆障碍，复用唯一工作台规划器；未知道路不开放。"""

import copy
import math

from ..workbench.contracts import PlanInput
from ..workbench.scene import make_field


def distance(point, first, last):
    dx, dy = last[0]-first[0], last[1]-first[1]
    ratio = max(0., min(1., ((point[0]-first[0])*dx+(point[1]-first[1])*dy)/(dx*dx+dy*dy))) if dx or dy else 0.
    return math.hypot(point[0]-first[0]-ratio*dx, point[1]-first[1]-ratio*dy)


def parameters(scan, request):
    if (not scan['usable'] or scan['domain'] != 'simulation' or not scan['synthetic'] or
            not scan['simulation_registered'] or scan['start'] != request.start):
        raise ValueError(scan.get('reason') or '扫描不是当前起点的完整模拟观测')
    scene = make_field(request)
    road_states = {}
    for road, first, last, _ in scene.data['edges']:
        known = road in scan['coverage']
        occupied = any(distance((item['x_m'],item['y_m']),scene.data['nodes'][first],scene.data['nodes'][last])
            <= scene.radius + scene.data['margin_m'] + item['diameter_m']/2 for item in scan['obstacles'])
        road_states[road] = 'UNKNOWN' if not known else 'OBSTACLE' if occupied else 'OBSERVED'
    values = request.model_dump()
    # 只替换雷达命名空间，保留人工黑圈和用户封路；圆障碍另由完整车体扫掠判定。
    values['obstacles'] = [item for item in values['obstacles'] if not item['id'].startswith('lidar_')]+copy.deepcopy(scan['obstacles'])
    values['blocked'] = list(dict.fromkeys(values['blocked']+[road for road,state in road_states.items() if state == 'UNKNOWN']))
    return PlanInput.model_validate(values), road_states


def source(scan, states):
    return dict(domain='simulation', scan_id=scan['id'], scan_sha256=scan['sha256'],
        raw_sha256=scan['raw_sha256'], calibration_version=scan['calibration_version'],
        synthetic=True, real_map_registered=False, authorizes_motion=False, road_states=states)
