"""同一版本、同一速度下比较停车路线与连续路线；仅运行正式离线模型。"""

import argparse
import hashlib
import json
from pathlib import Path
import subprocess
import sys

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'pi'))
from robot.route_sim import RouteSimulation


def main():
    parser = argparse.ArgumentParser(description='离线导航对照，不连接任何设备')
    parser.add_argument('--library', required=True, type=Path)
    parser.add_argument('--output', type=Path, default=ROOT / 'output/flow')
    args = parser.parse_args()
    output = args.output.resolve()
    output.mkdir(parents=True, exist_ok=True)
    paths = [ROOT / 'config/field_preview.json', *sorted((ROOT / 'firmware/User/Control').glob('*.[ch]')),
             *sorted((ROOT / 'pi/robot/nav').glob('*.py')), ROOT / 'pi/robot/route_sim.py',
             ROOT / 'pi/robot/device/tracking.py', args.library.resolve()]
    sources = {str(path.relative_to(ROOT)): hashlib.sha256(path.read_bytes()).hexdigest() for path in paths}
    results = {}
    for name, options in (
            ('start1_stopped', dict(start='start_1', flow=False)),
            ('start1_flow', dict(start='start_1', flow=True)),
            ('start2_flow', dict(start='start_2', flow=True)),
            ('start1_tour', dict(start='start_1', flow=True, scope='tour'))):
        report = RouteSimulation(args.library.resolve(), **options).run()
        (output / f'{name}.json').write_text(json.dumps(report, ensure_ascii=False, indent=2), encoding='utf-8')
        fields = ('status', 'reason', 'elapsed_ms', 'actions', 'inplace_actions', 'inplace_turn_ms',
                  'moving_turn_ms', 'travel_m', 'cruise_m_s', 'minimum_distance_m', 'ideal_minimum_s',
                  'initial_pose', 'origin_consistent', 'budget_scope', 'budget_complete', 'missing_navigation')
        results[name] = {key: report[key] for key in fields}
        print(name, report['status'], report['elapsed_ms'], 'ms', report['actions'], 'actions', flush=True)
    before, after = [results[key] for key in ('start1_stopped', 'start1_flow')]
    results['comparison'] = dict(saved_ms=before['elapsed_ms']-after['elapsed_ms'],
                                 saved_percent=100*(1-after['elapsed_ms']/before['elapsed_ms']))
    comparable = (before['cruise_m_s'] == after['cruise_m_s'] and
                  before['minimum_distance_m'] == after['minimum_distance_m'] and
                  before['budget_scope'] == after['budget_scope'] == 'two_batch')
    results['comparison']['same_scope_and_speed'] = comparable
    evidence = dict(domain='simulation', executable=False,
                    base_commit=subprocess.check_output(['git', 'rev-parse', 'HEAD'], cwd=ROOT, text=True).strip(),
                    sources=sources, results=results)
    (output / 'comparison.json').write_text(json.dumps(evidence, ensure_ascii=False, indent=2), encoding='utf-8')
    if any(row['status'] != 'COMPLETE' for key,row in results.items() if key != 'comparison'):
        raise SystemExit('存在失败场景，保留报告，不能作为全通过验收')
    if not comparable or after['elapsed_ms'] >= before['elapsed_ms'] or after['actions'] >= before['actions']:
        raise SystemExit('同口径优化未通过，保留报告供排查')


if __name__ == '__main__':
    main()
