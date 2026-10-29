"""将本次已结束的UART/SWD证据生成只读Web摘要；不连接设备。"""
import argparse
import json
from pathlib import Path
import sys

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'pi'))
from robot.device.acceptance import build_report
from robot.workbench.storage import write_json


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--source', type=Path, required=True)
    parser.add_argument('--observed-at', type=float, required=True)
    parser.add_argument('--ram', type=Path)
    parser.add_argument('--swd', type=Path)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    load = lambda path: json.loads(path.read_text(encoding='utf-8')) if path else None
    report = build_report(args.source, args.observed_at, load(args.ram), load(args.swd))
    write_json(args.output, report)
    print(json.dumps(dict(passed=report['passed'], firmware=report['firmware'],
                          fault=report['last_execution']['fault'], live=False, authorizes_motion=False)))


if __name__ == '__main__':
    main()
