"""冻结当前离线方案前两站，复用正式C闭环；不导入串口或执行实车。"""

import argparse
import json
import math
from pathlib import Path
import sys

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'pi'))
from robot.track_sim import ClosedLoop, settings
from robot.odom_sim import fixture
from robot.device.tracking import Goal, Spec
from robot.nav.frames import Pose, compose
from robot.nav.reference import NativeReference
from robot.nav.route import digest, anchor_spec


def prepare(snapshot, library, targets=('qr', 'raw'), name='start1_qr_raw_candidate', anchor=False):
    nav = snapshot['navigation']
    package = nav['execution_package']
    if (package['sha256'] != digest(package) or nav['start'] != 'start_1'
            or [p['goal'] for p in nav['legs'][:len(targets)]] != list(targets)):
        raise ValueError('不是完整的启停区1扫码后到原料区离线方案')
    count = sum(len(p['segments']) for p in nav['legs'][:len(targets)])
    specs = package['goals'][:count]
    config = settings(fixture())
    config.position_tol_m, config.yaw_tol_rad = .003, .012
    model = ClosedLoop(library, config=config, auto_start=False)
    origin = Pose(.15, 2.25, 0)
    references, results, expected = [], [], origin
    while model.elapsed_ms < 150:
        model.advance(report=False)
    for index, encoded in enumerate(specs):
        spec = Spec.from_buffer_copy(bytes.fromhex(encoded))
        planned_start = expected
        reference = NativeReference(model.native.lib, expected, spec)
        references.extend(p.values() for _, p in reference.samples(.05))
        expected = reference.end
        began = model.elapsed_ms
        if anchor:
            actual = compose(origin, Pose(model.pose['x_m'], model.pose['y_m'], model.pose['yaw_rad']))
            spec = anchor_spec(spec, planned_start, expected, actual)
        model.begin(Goal(index+1, 30000, spec))
        result = model.run(limit_ms=began+30000)
        if result['state'] != 'COMPLETE':
            raise ValueError(f'第{index+1}段闭环未完成：{result["state"]}')
        truth = compose(origin, Pose(model.plant.x, model.plant.y, model.plant.yaw))
        error = math.dist(truth.values()[:2], expected.values()[:2])
        if error > .01 or abs(truth.yaw_rad-expected.yaw_rad) > .04:
            raise ValueError('模型累计终点误差超限')
        results.append(dict(action=index+1, elapsed_s=(model.elapsed_ms-began)/1000,
                            truth=truth.values(), expected=expected.values(), error_m=error))
    plan = dict(schema=1, id=name, domain='simulation', executable=False,
                frame_id='map', source_sha256=package['sha256'], goals=specs,
                goal_encoding='RefSpec88', cruise_m_s=nav['cruise_m_s'],
                max_rpm=config.max_rpm, peak_rpm=model.peak_rpm,
                results=results, reference_trace=references,
                elapsed_s=sum(v['elapsed_s'] for v in results), physical_verified=False,
                blockers=['0.3.7固件不接受此两段参数', '离线速度与动态姿态尚未实车验证'])
    plan['sha256'] = digest(plan)
    return plan


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--snapshot', type=Path, default=ROOT/'output/preview/state.json')
    parser.add_argument('--library', type=Path, default=ROOT/'output/raw-route/native.dll')
    parser.add_argument('--output', type=Path, default=ROOT/'output/raw-route/candidate.json')
    args = parser.parse_args()
    plan = prepare(json.loads(args.snapshot.read_text(encoding='utf-8')), args.library.resolve())
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(plan, ensure_ascii=False, indent=2), encoding='utf-8')
    print(json.dumps({key: plan[key] for key in ('id','cruise_m_s','elapsed_s','peak_rpm','results')}))


if __name__ == '__main__':
    main()
