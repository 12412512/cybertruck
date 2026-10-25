"""核对实测点的轴向假设；只读本地样本，不写标定、不访问设备。"""
import argparse
import json
from pathlib import Path
import sys

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'pi'))
from robot.lidar.calibration import audit_axes


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--sample', type=Path, action='append', required=True)
    parser.add_argument('--independent', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    if args.output.exists():
        parser.error('证据文件已存在，不能覆盖')
    if not 3 <= len(args.sample) <= 12:
        parser.error('拟合样本限定3～12点')
    read = lambda path: json.loads(path.read_text(encoding='utf-8'))
    report = audit_axes([read(path) for path in args.sample], read(args.independent))
    report['source_files'] = dict(samples=[str(path.resolve()) for path in args.sample],
        independent=str(args.independent.resolve()))
    args.output.write_text(json.dumps(report, ensure_ascii=False, indent=2), encoding='utf-8')
    print(json.dumps({key:report[key] for key in ('hypotheses', 'applied',
        'real_map_registered', 'authorizes_motion')}, ensure_ascii=True))


if __name__ == '__main__':
    main()
