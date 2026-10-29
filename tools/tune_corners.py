"""固定0.30m/s比较原弯道策略与前后衔接；不调用任何真实设备。"""

import argparse
import json
from pathlib import Path
import subprocess

from tune_nav import ROOT, sources, FIELDS
from robot.route_sim import RouteSimulation

CASES = {
    'start1_before': dict(start='start_1',lookahead=False),
    'start1_window': dict(start='start_1',lookahead=True,turn_rate_rad_s=.7),
    'start1_after': dict(start='start_1',lookahead=True),
    'start1_rate15': dict(start='start_1',lookahead=True,turn_rate_rad_s=1.5),
    'start2_before': dict(start='start_2',lookahead=False),
    'start2_after': dict(start='start_2',lookahead=True),
}


def main():
    parser = argparse.ArgumentParser(description='同速弯道/转姿离线对照')
    parser.add_argument('--library',type=Path,required=True)
    parser.add_argument('--output',type=Path,default=ROOT/'output/corners/final')
    parser.add_argument('--case',choices=tuple(CASES),action='append')
    parser.add_argument('--collect',action='store_true')
    args = parser.parse_args()
    args.output.mkdir(parents=True,exist_ok=True)
    version = sources(args.library)
    if not args.collect:
        for name in args.case or CASES:
            report = RouteSimulation(args.library,cruise_m_s=.30,compare_baseline=False,**CASES[name]).run()
            evidence = dict(case=name,sources=version,base_commit=subprocess.check_output(
                ['git','rev-parse','HEAD'],cwd=ROOT,text=True).strip(),result=report)
            (args.output/f'{name}.json').write_text(json.dumps(evidence,ensure_ascii=False,indent=2),encoding='utf-8')
            print(name,report['status'],report['elapsed_ms'],'ms',report['reason'],flush=True)
        return
    rows = {}
    for name in CASES:
        evidence = json.loads((args.output/f'{name}.json').read_text(encoding='utf-8'))
        if evidence['sources'] != version:
            raise SystemExit(f'{name}源版本不一致')
        report = evidence['result']
        if report['domain'] != 'simulation' or report['executable'] or report['actual_pose'] is not None or report['cruise_m_s'] != .30:
            raise SystemExit(f'{name}数据域或同速口径错误')
        rows[name] = {key:report[key] for key in (*FIELDS,'lookahead','turn_rate_rad_s')}
    (args.output/'summary.json').write_text(json.dumps(dict(sources=version,cases=rows),
        ensure_ascii=False,indent=2),encoding='utf-8')
    print(json.dumps(rows,ensure_ascii=False,indent=2))
    if any(row['status']!='COMPLETE' for row in rows.values()):
        raise SystemExit('存在失败场景，不能作为整轮通过')


if __name__=='__main__':
    main()
