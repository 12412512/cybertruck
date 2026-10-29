"""场景的唯一几何来源：工位间隙、圆形障碍物和名义地图。"""

import hashlib
import json

from ..nav.field import Field

PRESETS = dict(raw=['qr', 'raw'], tour=['qr', 'raw', 'rough', 'staging', 'return'],
               batch=['qr', 'raw', 'rough', 'staging', 'raw', 'rough', 'staging', 'return'])
NAMES = dict(start_1='启停区1', start_2='启停区2', qr='扫码', raw='原料',
             rough='粗加工', staging='暂存', north='上方预停点')


def start_zones(parameters):
    """起点及用户明确访问的蓝区采用同一包络规则，保存/执行不能漏掉另一蓝区。"""
    return tuple(dict.fromkeys(name for name in [parameters['start'], *parameters['targets']]
                               if name in ('start_1', 'start_2')))


def normalize(request):
    if request.source != 'custom':
        request = request.model_copy(update={'targets': [request.start if name == 'return' else name
                                                        for name in PRESETS[request.source]]})
    return request


def make_field(request):
    data = Field().data
    work = {zone['id']: zone['bounds_m'] for zone in data['zones']}
    gap = request.docking
    # 三处作业朝向固定，左侧机构朝工位；left_m是朝向工位的车体外缘偏置。
    side = data['body']['left_m']
    # 保留原料道路交点，另建靠台点，避免把整条车道拉斜。
    data['nodes']['raw_road'] = list(data['nodes']['raw'])
    data['edges'] = [[road, 'raw_road' if a == 'raw' else a,
                      'raw_road' if b == 'raw' else b, policy]
                     for road, a, b, policy in data['edges']]
    data['edges'].append(['raw_dock', 'raw_road', 'raw', 'short'])
    data['nodes']['raw'][0] = work['raw_surface'][2] + side + gap.raw_m
    data['nodes']['rough'][0] = work['rough_surface'][0] - side - gap.rough_m
    data['nodes']['staging'][1] = work['staging_surface'][3] + side + gap.staging_m
    data['obstacles'] = [item.model_dump() for item in request.obstacles]
    data['docking'] = gap.model_dump()
    key = json.dumps(dict(docking=data['docking'], obstacles=data['obstacles']), sort_keys=True)
    data['version'] += '-' + hashlib.sha256(key.encode()).hexdigest()[:12]
    return Field(data=data)
