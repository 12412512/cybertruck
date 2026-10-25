"""换板集中验收：默认仅列计划；读取旧记录离线分析，显式--collect才可能打开串口。"""

import argparse
import json
from pathlib import Path
import subprocess
import sys

ROOT = Path(__file__).resolve().parents[1]


def analyze(data):
    summary, states = data['summary'], data['states']
    execution = [row['execution'] for row in states if row.get('execution')]
    switch = [row['physical_start'] for row in states if row.get('physical_start')]
    flags = [row['board']['flags'] for row in states if row.get('board')]
    zones = sorted({v['select_zone'] for v in switch if v.get('select_valid')})
    keys = sorted({bool(v & 8) for v in flags})
    # 摘要之外再核对原始记录，后续故障不能被旧的passed字段遮住。
    from accept_base import READONLY_KINDS
    requests = summary.get('requests', {})
    checks = dict(
        base=summary['passed'],
        readonly_commands=bool(requests) and all(int(kind) in READONLY_KINDS for kind in requests),
        stopped_disabled=bool(execution) and all(
            not (v['mode'] or v['fault'] or v['can_errors'] or v['action_id'] or
                 v['action_state'] or v['config_version']) and v['stop_confirmed'] for v in execution),
        pc1_both_zones=zones == [1, 2],
        pc0_press_release=keys == [False, True],
        no_preparation=bool(switch) and all(not (v['ticket'] or v['fired'] or v['beep']) for v in switch))
    return dict(schema=1,domain='hardware_acceptance',live=False,authorizes_motion=False,
        scope='base_and_switch_inputs_only',full_hardware_accepted=False,
        passed=all(checks.values()),checks=checks,zones_seen=zones,key_states_seen=keys,
        firmware=summary.get('firmware'),mechanism_state=summary.get('mechanism_state'),
        pending=['ID5/ID6主动轮询需另行受控附着，本次58仅读软件状态，不证明两轴CAN反馈正常',
                 'PG11实际电平及短鸣、PE14波形、实物位置与停车距离另行验收',
                 '历史记录不作为当前动作许可'],source_summary=summary)


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    mode=parser.add_mutually_exclusive_group()
    mode.add_argument('--analyze',type=Path,help='只分析已有采集JSON，不接设备')
    mode.add_argument('--collect',action='store_true',help='现场授权后显式采集，只发送只读白名单')
    parser.add_argument('--port')
    parser.add_argument('--output',type=Path)
    parser.add_argument('--seconds',type=float,default=30)
    args=parser.parse_args()
    if not args.analyze and not args.collect:
        print(json.dumps(dict(domain='preparation',device_access=False,programmed=False,
            firmware='GC-BASE-0.3.21',pins=dict(can_rx='PD0',can_tx='PD1',buzzer='PG11',reserved_pwm='PE14'),
            steps=['烧录须另获授权；本工具不烧录','只读检查版本、身份、启动停车、四轮及IMU、CAN首次错误/历史、PWM关闭',
                   'DISABLED且未准备PC0时，人工拨PC1两档、松开/按PC0，仅记录电平不触发动作',
                   '未覆盖的ID5/ID6、蜂鸣器实测及PE14波形保持待测'],
            collector='--collect --port <已核对串口> --output <新目录>/acceptance.json'),ensure_ascii=False,indent=2))
        return
    if not args.output or args.output.exists():parser.error('必须提供尚不存在的输出文件')
    source=args.analyze
    if args.collect:
        if not args.port or not 5<=args.seconds<=60:parser.error('采集需明确串口及5～60秒窗口')
        args.output.parent.mkdir(parents=True,exist_ok=True)
        source=args.output.with_name(args.output.stem+'_raw.json')
        if source.exists():parser.error('原始采集文件已存在，禁止覆盖')
        result=subprocess.run([sys.executable,str(ROOT/'tools/accept_base.py'),'--port',args.port,
            '--seconds',str(args.seconds),'--output',str(source),'--expect-firmware','GC-BASE-0.3.21'])
        if not source.exists():raise SystemExit(result.returncode or 1)
    report=analyze(json.loads(source.read_text(encoding='utf-8')))
    args.output.parent.mkdir(parents=True,exist_ok=True)
    with args.output.open('x',encoding='utf-8') as file:json.dump(report,file,ensure_ascii=False,indent=2)
    print(json.dumps(report,ensure_ascii=False,indent=2))
    if not report['passed']:raise SystemExit(1)


if __name__=='__main__':main()
