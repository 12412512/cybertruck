"""离线巡航档与返航朝向对照；各场景独立留证，支持并行运行后汇总。"""

import argparse
import hashlib
import json
from pathlib import Path
import subprocess
import sys

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'pi'))
from robot.route_sim import RouteSimulation

CASES = {
    'fixed_014': dict(cruise_m_s=.14,return_heading_free=False,return_region=False),
    'free_014': dict(cruise_m_s=.14),
    'free_022': dict(cruise_m_s=.22),
    'free_030': dict(cruise_m_s=.30),
    'free_036': dict(cruise_m_s=.36),
    'start2_030': dict(cruise_m_s=.30,start='start_2'),
}
FIELDS = ('status','reason','elapsed_ms','actions','cruise_m_s','return_heading_free',
          'return_heading_rad','return_region','return_target','inplace_actions','inplace_turn_ms','travel_m','peak_rpm',
          'peak_actual_rpm','max_rpm','origin_consistent','ideal_minimum_s','preparation_s',
          'budget_complete','missing_navigation')


def sources(library):
    paths = [ROOT/'config/field_preview.json',library.resolve(),
             *sorted((ROOT/'firmware/User/Control').glob('*.[ch]')),
             *sorted((ROOT/'pi/robot/nav').glob('*.py')),
             *(ROOT/'pi/robot'/name for name in ('route_sim.py','track_sim.py','odom_sim.py'))]
    return {str(path.relative_to(ROOT)):hashlib.sha256(path.read_bytes()).hexdigest() for path in paths}


def main():
    parser = argparse.ArgumentParser(description='仅离线速度对照，不接真实设备')
    parser.add_argument('--library',required=True,type=Path)
    parser.add_argument('--output',type=Path,default=ROOT/'output/speed/final')
    parser.add_argument('--case',choices=tuple(CASES),action='append')
    parser.add_argument('--collect',action='store_true',help='核对六场景及源码版本，汇总而不重复运行')
    args = parser.parse_args()
    args.output.mkdir(parents=True,exist_ok=True)
    version = sources(args.library)
    if not args.collect:
        for name in args.case or CASES:
            report = RouteSimulation(args.library.resolve(),compare_baseline=False,**CASES[name]).run()
            evidence = dict(case=name,base_commit=subprocess.check_output(
                ['git','rev-parse','HEAD'],cwd=ROOT,text=True).strip(),sources=version,result=report)
            (args.output/f'{name}.json').write_text(json.dumps(evidence,ensure_ascii=False,indent=2),encoding='utf-8')
            print(name,report['status'],report['elapsed_ms'],'ms',report['reason'],flush=True)
        return
    rows = {}
    for name in CASES:
        evidence = json.loads((args.output/f'{name}.json').read_text(encoding='utf-8'))
        if evidence['sources'] != version:
            raise SystemExit(f'{name}版本不一致，不能混用旧结果')
        report = evidence['result']
        if report['domain'] != 'simulation' or report['executable'] or report['actual_pose'] is not None:
            raise SystemExit('数据域错误，不能作为离线对照')
        rows[name] = {key:report[key] for key in FIELDS}
    summary = dict(domain='simulation',executable=False,sources=version,cases=rows,
                   note='失败档保留原因；通过仅代表合成车体，不是实车速度许可')
    (args.output/'summary.json').write_text(json.dumps(summary,ensure_ascii=False,indent=2),encoding='utf-8')
    print(json.dumps(rows,ensure_ascii=False,indent=2))


if __name__ == '__main__':
    main()
