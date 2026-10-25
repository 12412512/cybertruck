"""同一场景的速度与加减速联合对照；不打开真实设备，不改用户草稿。"""

import argparse
import json
from pathlib import Path
import sys
import time
import hashlib

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'pi'))
from robot.workbench.contracts import PlanInput
from robot.workbench.planning import make_plan
from robot.workbench.simulation import Simulation


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--profile', choices=('balanced', 'fast', 'fast_safe', 'target', 'target_segmented', 'target_safe', 'target_slew', 'target_ff', 'target_ff_high'), required=True)
    parser.add_argument('--start', choices=('start_1', 'start_2'))
    parser.add_argument('--library', type=Path, default=ROOT / 'output/workbench/native.dll')
    args = parser.parse_args()
    source = json.loads((ROOT / 'output/workbench/workspace.json').read_text(encoding='utf-8'))['draft']
    profiles = json.loads((ROOT / 'config/navigation_candidates.json').read_text(encoding='utf-8'))
    source.update(profiles['profiles'][args.profile])
    source['source'] = 'batch'
    if args.start:
        source['start'] = args.start
    request = PlanInput.model_validate(source)
    library = args.library.resolve()
    started = time.monotonic()
    plan = make_plan(library, request, progress=lambda value: print(value['completed'], flush=True))
    print('reference', plan.view['seconds'], 'segments', len(plan.specs), flush=True)
    simulation = Simulation(library)
    simulation.run(plan)
    while simulation.mode == 'RUNNING' and simulation.model.elapsed_ms-simulation.run_ms < 240000:
        simulation.step()
    result = dict(profile=args.profile, domain='simulation', physical_validated=False,
                  parameters=request.model_dump(), plan=plan.view,
                  specs_hex=[value.hex() for value in plan.specs], result=simulation.view(),
                  calculation_s=time.monotonic()-started)
    result['library_sha256'] = hashlib.sha256(library.read_bytes()).hexdigest()
    result['source_sha256'] = {name: hashlib.sha256((ROOT/name).read_bytes()).hexdigest() for name in (
        'pi/robot/nav/planner.py', 'pi/robot/nav/reference.py', 'pi/robot/workbench/planning.py',
        'pi/robot/workbench/simulation.py', 'firmware/User/Control/track.c', 'firmware/User/Control/track.h')}
    folder = ROOT / 'output/speed-comparison'
    folder.mkdir(parents=True, exist_ok=True)
    name = f'{args.profile}_{request.start}_{time.time_ns()}.json'
    (folder / name).write_text(json.dumps(result, ensure_ascii=False, indent=2), encoding='utf-8')
    print(result['result']['mode'], result['result']['elapsed_s'], result['result']['reason'], flush=True)


if __name__ == '__main__':
    main()
