"""通过唯一雷达核心重新取得校准备份；没有09提交或清故障选项。"""
import argparse
import json
from pathlib import Path
import time
from urllib.error import HTTPError
from urllib.request import Request, urlopen


def read(port):
    with urlopen(f'http://127.0.0.1:{port}/state', timeout=3) as response:
        return json.load(response)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--zone', type=int, choices=(1, 2), required=True)
    parser.add_argument('--placement-id', required=True)
    parser.add_argument('--request-id', required=True)
    parser.add_argument('--manual-fixed-deployed', action='store_true')
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--execute', action='store_true')
    args = parser.parse_args()
    body = dict(request_id=args.request_id, zone=args.zone, placement_id=args.placement_id,
        manual_fixed_deployed=args.manual_fixed_deployed)
    if not args.execute:
        print(json.dumps(dict(preview=True, request=body, device_commands=0, calibration_write=False)))
        return
    if not args.manual_fixed_deployed:
        parser.error('只读准备需要本轮人工展开固定确认；不输出舵机PWM')
    args.output.mkdir(parents=True, exist_ok=False)
    def save(name, value):
        (args.output/name).write_text(json.dumps(value, ensure_ascii=False, indent=2), encoding='utf-8')
    before = read(8093); save('lidar_before.json', before); save('board_before.json', read(8091))
    caps = before.get('calibration_capabilities') or {}
    if not caps.get('prepare') or caps.get('write') is not False:
        raise RuntimeError('核心没有明确处于校准只读准备模式；未发送请求')
    save('request.json', body)
    accepted = None
    for _ in range(10):
        request = Request('http://127.0.0.1:8093/calibration/prepare', data=json.dumps(body).encode(),
            headers={'Content-Type':'application/json', 'X-Lidar-Workbench':'1'}, method='POST')
        try:
            with urlopen(request, timeout=3) as response:
                accepted = json.load(response)
            break
        except HTTPError as error:
            detail = json.loads(error.read()).get('detail', '')
            if error.code != 409 or '事务未结束' not in detail:
                save('rejected.json', dict(code=error.code, detail=detail))
                raise RuntimeError(detail) from error
            time.sleep(.1)
    if not accepted:
        raise RuntimeError('只读握手事务未结束，没有接纳校准准备')
    save('accepted.json', accepted)
    samples = []
    for _ in range(40):
        value = read(8093); samples.append(value); current = value.get('calibration') or {}
        if current.get('id') == accepted['id'] and current['status'] in ('PREPARED', 'FAILED', 'CANCELLED'):
            if current['status'] == 'PREPARED' and not current.get('backup_saved'):
                time.sleep(.1); continue
            break
        time.sleep(.1)
    save('states.json', samples); save('board_after.json', read(8091))
    current = samples[-1].get('calibration') or {}; save('result.json', current)
    if current.get('id') != accepted['id'] or current.get('status') != 'PREPARED' or not current.get('backup_saved'):
        raise RuntimeError('未完成本轮只读备份；不自动提交校准')
    if current['write_issued'] or any(row['command'] not in (1, 2, 0x13) for row in current['commands']):
        raise RuntimeError('准备阶段出现非只读命令，需保留现场检查')
    print(json.dumps(dict(id=current['id'], status=current['status'], backup_version=current['backup_version'],
        backup_saved=True, fresh_read=True, calibration_write=False, device_motion=False)))


if __name__ == '__main__':
    main()
