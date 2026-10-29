"""雷达报文→障碍→规划→执行链离线留证；没有真实设备选项。"""

import argparse
import json
from pathlib import Path
import sys

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'pi'))
from robot.lidar.fixtures import CASES, fixture
from robot.lidar.replay import replay
from robot.lidar.planning import parameters, source
from robot.workbench.contracts import PlanInput
from robot.workbench.planning import make_plan
from robot.workbench.scene import make_field
from robot.workbench.storage import write_json, payload
from robot.nav.bridge import prepare
from robot.device.preview import execute


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--case',choices=CASES,default='detour')
    parser.add_argument('--start',choices=('start_1','start_2'),default='start_1')
    parser.add_argument('--target',help='只规划此站点；省略则两批完整路线')
    parser.add_argument('--library',type=Path,default=ROOT/'output/workbench/native.dll')
    parser.add_argument('--output',type=Path,required=True)
    parser.add_argument('--execute-model',action='store_true',help='仅运行内存协议与独立Plant，不连接设备')
    args = parser.parse_args()
    if args.output.exists():
        parser.error('输出目录已存在，不覆盖旧证据')
    args.output.mkdir(parents=True)
    request = PlanInput(start=args.start,source='custom',targets=[args.target]) if args.target else PlanInput(start=args.start)
    wire, profile = fixture(args.case,args.start,[edge[0] for edge in make_field(request).data['edges']])
    scan = replay(wire,profile,args.start)
    write_json(args.output/'scan.json',dict(scan=scan,wire_hex=wire.hex(),profile=profile))
    result = dict(domain='simulation',device_access=False,programmed=False,scan_status=scan['status'],
                  physical_validated=False,authorizes_motion=False)
    if scan['usable']:
        try:
            request, roads = parameters(scan,request)
            plan = make_plan(args.library,request)
            plan.view['lidar_source'] = source(scan,roads)
            write_json(args.output/'plan.json',payload(plan))
            bundle = prepare(plan,args.library)
            write_json(args.output/'bundle.json',bundle)
            result.update(plan_id=plan.view['id'],segments=len(plan.specs),reference_s=plan.view['seconds'])
            if args.execute_model:
                report = execute(args.library,bundle,args.output/'execution')
                result.update(execution_status=report['status'],elapsed_s=report['elapsed_s'],
                              stop_confirmed=report['stop_confirmed'],sweep_samples=report['sweep_samples'])
        except ValueError as exc:
            result['planning_error'] = str(exc)
    else:
        result['planning_error'] = scan['reason']
    write_json(args.output/'summary.json',result)
    print(json.dumps(result,ensure_ascii=False,indent=2))
    if result.get('planning_error') or result.get('execution_status','COMPLETE') != 'COMPLETE':
        raise SystemExit(1)


if __name__ == '__main__':
    main()
