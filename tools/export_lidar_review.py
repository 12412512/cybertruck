"""将本地实扫证据导出为原Web的只读核对页；不打开设备或重启服务。"""
import argparse
import json
from pathlib import Path
import sys

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT/'pi'))
from robot.lidar.review import build_review


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--source', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    read = lambda path: json.loads(path.read_text(encoding='utf-8'))
    installation = read(ROOT/'config/lidar_installation.json')
    site = read(ROOT/'config/start1_qr_site.json')
    body = [site['start']['from_left_m'],
        site['field']['height_m']-site['start']['from_top_m'], site['start']['heading_rad']]
    records = []
    for name, label in (('p1_fixed', 'P1复测'), ('p2', 'P2'), ('p3', 'P3'), ('p4', 'P4独立点')):
        folder = args.source/name
        records.append(dict(label=label, sample=read(folder/'measured_sample.json'),
            capture=read(folder/'capture.json')))
    review = build_review(installation, body, records)
    first = read(args.source/'p1/measured_sample.json')
    repeat = read(args.source/'p1_repeat/comparison.json')
    fixed = read(args.source/'p1_fixed/comparison.json')
    review['preserved_first_p1'] = dict(scan_id=first['scan_id'], source_m=first['source_m'],
        difference_after_reposition_m=repeat['difference_mm']/1000,
        fixed_repeat_difference_m=fixed['difference_mm']/1000,
        reason='首次读数差异未定因，原始记录保留；使用固定同点复测作为当前诊断参考')
    from robot.lidar.calibration import digest
    review['version'] = digest({key:value for key, value in review.items() if key != 'version'})
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(review, ensure_ascii=False, indent=2), encoding='utf-8')
    print(json.dumps(dict(version=review['version'], live=False,
        independent_error_m=review['independent_error_m'], device_commands=0,
        auto_execution_ready=False, authorizes_motion=False)))


if __name__ == '__main__':
    main()
