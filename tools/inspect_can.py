"""经既有所有者GET只读采样CAN诊断，保留前后证据；不打开串口。"""

import argparse
import json
from pathlib import Path
import sys
import time

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'pi'))
from robot.device.observer import OwnerClient, number
from robot.device.can_diag import diagnose, compare


def main():
    parser = argparse.ArgumentParser(description='只读CAN故障取证；不发STOP、不清故障')
    parser.add_argument('--owner-url', required=True)
    parser.add_argument('--seconds', type=float, default=10)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    if not .5 <= args.seconds <= 60:
        parser.error('窗口必须为0.5～60秒')
    if args.output.exists():
        parser.error('保留已有证据，请指定新输出文件')
    client = OwnerClient(args.owner_url)
    rows, failures = [], []
    deadline = time.monotonic() + args.seconds
    keys = ('firmware', 'connected', 'state_age_ms', 'board', 'can_error_event',
            'can_diagnostics', 'wheels', 'imu', 'imu_diagnostics', 'stopped', 'ready')
    while time.monotonic() < deadline:
        try:
            source, sent, received = client.read()
            report = diagnose(source)
            rows.append(dict(request_s=sent, response_s=received, wall_s=time.time(),
                             state={key: source.get(key) for key in keys}, report=report))
        except (OSError, ValueError, KeyError, TypeError) as exc:
            failures.append(dict(at_s=time.monotonic(), reason=type(exc).__name__))
        time.sleep(.1)
    fresh = sum(row['state'].get('connected') is True
                and number(row['state'].get('state_age_ms'))
                and row['state']['state_age_ms'] + (row['response_s'] - row['request_s']) * 1000 < 250
                for row in rows)
    summary = dict(samples=len(rows), failures=len(failures), fresh_samples=fresh,
                   control_requests_sent=0)
    if rows:
        summary.update(compare(rows[0]['state'], rows[-1]['state']))
        summary['first'] = rows[0]['report']
        summary['last'] = rows[-1]['report']
    args.output.parent.mkdir(parents=True, exist_ok=True)
    with args.output.open('x', encoding='utf-8') as output:
        json.dump(dict(summary=summary, failures=failures, samples=rows), output,
                  ensure_ascii=False, indent=2, allow_nan=False)
    print(json.dumps({key: value for key, value in summary.items() if key not in ('first', 'last')}))
    if not rows:
        raise SystemExit('没有有效诊断样本，保留失败记录')


if __name__ == '__main__':
    main()
