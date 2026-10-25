"""实体PC1选区只选择已冻结路线，不把网页草稿直接提升为实车目标。"""

import json
from pathlib import Path

from .route import validate

ROOT = Path(__file__).resolve().parents[3]


def routes():
    result = {}
    for zone, name, identity in ((1, 'start1_route.json', 'start1_dock_v1'),
                                 (2, 'start2_route.json', 'start2_reverse_v1')):
        value = json.loads((ROOT / 'config' / name).read_text(encoding='utf-8'))
        validate(value)
        if value['id'] != identity or value['map_origin'] != ([.15, 2.25, 0.] if zone == 1 else [2.25, 2.25, 0.]):
            raise ValueError('选区路线起点/朝向不匹配')
        result[zone] = value
    return result


def choice(event, age_s):
    """过期、抖动或旧协议均不给出选区，不回退到默认区1。"""
    if not event or age_s is None or not 0 <= age_s < .25 or event.get('schema') != 3:
        return None
    zone, epoch = event.get('select_zone'), event.get('select_epoch')
    if event.get('select_valid') != 1 or zone not in (1, 2) or type(epoch) is not int or not epoch:
        return None
    return zone, epoch
