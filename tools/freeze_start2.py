"""从已保存工作台读取启停区2轨迹，先验证再导出；不访问设备或修改草稿。"""

import argparse
import hashlib
import json
import math
from pathlib import Path
import struct
import sys

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'pi'))
from robot.device.tracking import Spec
from robot.nav.frames import Pose, relative
from robot.nav.reference import NativeReference
from robot.nav.route import digest, validate
from robot.workbench.contracts import PlanInput
from robot.workbench.planning import restore_plan
from robot.workbench.simulation import Simulation


def freeze(workspace, library):
    saved = workspace['last_plan']
    plan = restore_plan(saved, library)
    parameters = PlanInput.model_validate(plan.view['parameters'])
    if PlanInput.model_validate(workspace['draft']).model_dump() != parameters.model_dump():
        raise ValueError('草稿与保存路线不同，先在工作台确认并保存路线')
    if (parameters.start != 'start_2' or parameters.source != 'batch' or
            parameters.speed_m_s != .30 or parameters.max_rpm != 100 or
            parameters.dynamics.lag_ff_s != 0 or parameters.dynamics.wheel_slew_rpm_s != 180):
        raise ValueError('仅接受启停区2两批0.30m/s、100RPM及已验证控制档')
    origin = Pose(*plan.view['origin'])
    if origin != Pose(2.25, 2.25, math.pi) or len(plan.specs) != 10:
        raise ValueError('起点、车头或段数与本次受限入口不匹配')
    simulation = Simulation(library)
    simulation.run(plan)
    while simulation.mode == 'RUNNING' and simulation.model.elapsed_ms-simulation.run_ms < 180000:
        simulation.step()
    if simulation.mode != 'COMPLETE':
        raise ValueError('保存路线闭环/实际扫掠失败：' + str(simulation.view()))
    current = origin
    boundaries = []
    for raw in plan.specs:
        current = NativeReference(simulation.model.native.lib, current, Spec.from_buffer_copy(raw)).end
        boundaries.append(relative(origin, current).values())
    value = dict(schema=1, id='start2_workbench_v1', domain='commissioning', frame_id='odom',
        cases=[f'start2_{index+1}' for index in range(len(plan.specs))],
        goals=[(struct.pack('<II', index+1, 30000) + raw).hex() for index, raw in enumerate(plan.specs)],
        reference_trace=[relative(origin, Pose(*point)).values() for point in plan.view['reference']],
        boundaries=boundaries, executable=False, map_registered=False,
        anchor_policy='measured_segment_start', segment_handoff='stopped_and_settled',
        map_origin=origin.values(), field=plan.view['field'], parameters=parameters.model_dump(),
        source_plan=dict(id=saved['id'], specs_sha256=saved['sha256'],
                         artifact_sha256=saved['artifact_sha256']),
        legs=plan.view['legs'])
    value['sha256'] = digest(value)
    return value, simulation.view()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--library', type=Path, default=ROOT/'output/workbench/native.dll')
    parser.add_argument('--write', action='store_true')
    args = parser.parse_args()
    workspace_path = ROOT/'output/workbench/workspace.json'
    before = workspace_path.read_bytes()
    value, result = freeze(json.loads(before), args.library.resolve())
    if workspace_path.read_bytes() != before:
        raise ValueError('工作台草稿在验证期间改变，拒绝发布旧版本')
    output = ROOT/'output/start2'
    output.mkdir(parents=True, exist_ok=True)
    report = dict(domain='simulation', physical_validated=False, result=result,
                  route_sha256=value['sha256'], source_plan=value['source_plan'])
    (output/'workbench_replay.json').write_text(json.dumps(report, ensure_ascii=False, indent=2), encoding='utf-8')
    if args.write:
        validate(value)
        (ROOT/'config/start2_route.json').write_text(json.dumps(value, ensure_ascii=False, indent=2), encoding='utf-8')
        profile = json.loads((ROOT/'config/batch_trials.json').read_text(encoding='utf-8'))
        profile.update(version=7, source='仅启停区2保存路线10段；候选，尚待本版烧录后实测')
        (ROOT/'config/start2_trials.json').write_text(json.dumps(profile, ensure_ascii=False, indent=2), encoding='utf-8')
    print(json.dumps(dict(status=result['mode'], seconds=result['elapsed_s'],
        route_sha256=value['sha256'], goals_sha256=hashlib.sha256(b''.join(bytes.fromhex(g) for g in value['goals'])).hexdigest()), indent=2))


if __name__ == '__main__':
    main()
