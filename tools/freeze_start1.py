"""复用工作台规划启停区1的50mm停靠路线；仅生成离线候选，不连接设备。"""

import argparse
import json
from pathlib import Path
import struct
import sys

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'pi'))
from robot.device.tracking import Spec
from robot.nav.frames import Pose, relative
from robot.nav.reference import NativeReference
from robot.nav.route import digest
from robot.workbench.contracts import PlanInput
from robot.workbench.planning import make_plan
from robot.workbench.simulation import Simulation


def freeze(library):
    saved = json.loads((ROOT / 'output/workbench/workspace.json').read_text(encoding='utf-8'))
    parameters = dict(saved['draft'], start='start_1', source='batch',
                      targets=['qr', 'raw', 'rough', 'staging', 'raw', 'rough', 'staging', 'start_1'],
                      speed_m_s=.30, max_rpm=100,
                      docking=dict(raw_m=.05, rough_m=.05, staging_m=.05))
    # 保留工作台障碍/封路及其他草稿，不回写用户保存的区2路线。
    request = PlanInput.model_validate(parameters)
    plan = make_plan(library, request)
    simulation = Simulation(library)
    simulation.run(plan)
    while simulation.mode == 'RUNNING' and simulation.model.elapsed_ms - simulation.run_ms < 180000:
        simulation.step()
    if simulation.mode != 'COMPLETE':
        raise ValueError('区1闭环或车体扫掠失败：' + str(simulation.view()))
    origin = Pose(*plan.view['origin'])
    current = origin
    boundaries = []
    for raw in plan.specs:
        current = NativeReference(simulation.model.native.lib, current, Spec.from_buffer_copy(raw)).end
        boundaries.append(relative(origin, current).values())
    route = dict(schema=1, id='start1_dock_v1', domain='commissioning', frame_id='odom',
        cases=[f'start1_{i+1}' for i in range(len(plan.specs))],
        goals=[(struct.pack('<II', i+1, 30000) + raw).hex() for i, raw in enumerate(plan.specs)],
        boundaries=boundaries, reference_trace=[relative(origin, Pose(*p)).values() for p in plan.view['reference']],
        executable=False, map_registered=False, anchor_policy='measured_segment_start',
        segment_handoff='stopped_and_settled', map_origin=origin.values(),
        field=plan.view['field'], parameters=request.model_dump(), legs=plan.view['legs'])
    route['sha256'] = digest(route)
    return route, simulation.view(), plan


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--library', type=Path, default=ROOT / 'output/workbench/native.dll')
    args = parser.parse_args()
    route, result, plan = freeze(args.library.resolve())
    output = ROOT / 'output/start-switch'
    output.mkdir(parents=True, exist_ok=True)
    (output / 'start1_candidate.json').write_text(json.dumps(route, ensure_ascii=False, indent=2), encoding='utf-8')
    (output / 'start1_replay.json').write_text(json.dumps(result, ensure_ascii=False, indent=2), encoding='utf-8')
    print(json.dumps(dict(route_sha256=route['sha256'], segments=len(plan.specs),
                         seconds=result['elapsed_s'], origin=route['map_origin'],
                         nodes=route['field']['nodes']), ensure_ascii=False, indent=2))


if __name__ == '__main__':
    main()
