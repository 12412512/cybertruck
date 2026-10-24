"""单段现场候选的离线闭环；无设备接口，不授予实车执行权限。"""

import math
import ctypes
import argparse
import json
from pathlib import Path

from ..track_sim import ClosedLoop, settings
from ..odom_sim import fixture
from ..device.tracking import Goal, Limit
from .frames import Pose, compose
from .reference import specification, NativeReference
from .geometry import footprint, hull, intersects, bounds
from .field import Field
from .site import site_digest, read_site


def simulate(library, site):
    """以待摆放标记为条件；不将该姿态写入真实里程计或现场状态。"""
    proposed = site['proposed_start']
    start = Pose(proposed['from_left_m'], site['field']['height_m']-proposed['from_top_m'], 0)
    target = Pose(site['target']['map_x_m'], site['target']['map_y_m'], 0)
    spec = specification('DIAGONAL', start, target, velocity=.018)
    spec.linear = Limit(.018, .05, .2)
    reference = NativeReference(ctypes.CDLL(str(library)), start, spec)
    # 固定朝向直线平移的矩形扫掠恰为两端外形凸包，不依赖稀疏采样。
    sweep = hull(footprint(start, site['body']) + footprint(target, site['body']))
    field = Field()
    collisions = [key for key, _, polygon in field.zones
                  if not key.startswith('start_') and intersects(sweep, polygon)]
    box = bounds(sweep)
    clearance = min(box[0], box[1], site['field']['width_m']-box[2], site['field']['height_m']-box[3])
    config = settings(fixture())
    config.max_rpm = 6
    model = ClosedLoop(library, config=config)
    model.goal = Goal(1, 120000, spec)
    result = model.run(limit_ms=120000)
    truth = compose(start, Pose(**{k: result['plant'][k] for k in ('x_m', 'y_m', 'yaw_rad')}))
    actual = [compose(start, Pose(**p)).values() for p in result['trace']]
    return dict(domain='simulation', executable=False, placement_confirmed=False,
                site_sha256=site_digest(site),
                start=start.values(), target=target.values(), kind='DIAGONAL',
                distance_m=math.dist(start.values()[:2], target.values()[:2]),
                reference_s=reference.duration_s, elapsed_s=result['elapsed_ms']/1000,
                state=result['state'], peak_rpm=result['peak_rpm'],
                truth=truth.values(), terminal_error_m=math.dist(truth.values()[:2], target.values()[:2]),
                reference=[pose.values() for _, pose in reference.samples(.2)], trace=actual,
                sweep=sweep, collisions=collisions, boundary_clearance_m=clearance,
                uncertainty_validated=False, firmware_compatible=False,
                blockers=['待实际按标记摆放并核对位置及航向误差',
                          '当前固件150毫米短段与20秒期限不接受本米级目标',
                          '合成电机参数并非实车距离标定，不证明实车跟踪误差'],
                geometry_scope='固定朝向直线扫掠，仅检查未膨胀静态包络，未证明动态误差余量')


def main():
    parser = argparse.ArgumentParser(description='启停区1到north离线模型，不连接设备')
    parser.add_argument('--library', type=Path, required=True)
    parser.add_argument('--output', type=Path, default=Path('output/start1-qr/preview.json'))
    args = parser.parse_args()
    report = simulate(args.library.resolve(), read_site())
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(report, ensure_ascii=False, indent=2), encoding='utf-8')
    print(json.dumps({key: report[key] for key in ('state', 'elapsed_s', 'distance_m', 'executable')}))


if __name__ == '__main__':
    main()
