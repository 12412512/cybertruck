"""从已完成的五站离线模型冻结整圈；不连接设备，不生成搬运动作。"""

import argparse
import hashlib
import json
from pathlib import Path
import struct

from prepare_raw_route import ROOT, prepare
from robot.nav.route import digest


def freeze(report, library, two_batch=False):
    targets = ('qr', 'raw', 'rough', 'staging', 'raw', 'rough', 'staging', 'start_1') if two_batch else ('qr', 'raw', 'rough', 'staging', 'start_1')
    if report.get('status') != 'COMPLETE':
        raise ValueError('完整离线跑图尚未通过')
    replay = prepare({'navigation': report}, library, targets, 'tour_candidate', anchor=True)
    if replay['results'][-1]['error_m'] > .01:
        raise ValueError('返航累计误差超限')
    plan = dict(schema=1, id='start1_two_batch_v1' if two_batch else 'start1_tour_v1', domain='commissioning', frame_id='odom',
                cases=[f'batch_{i+1}' if two_batch else f'tour_{i+1}' for i in range(len(replay['goals']))],
                goals=[(struct.pack('<II', i+1, 30000)+bytes.fromhex(spec)).hex()
                       for i, spec in enumerate(replay['goals'])],
                reference_trace=[[x-.15, y-2.25, yaw] for x, y, yaw in replay['reference_trace']],
                boundaries=[[p['expected'][0]-.15, p['expected'][1]-2.25, p['expected'][2]]
                            for p in replay['results']],
                executable=False, map_registered=False, anchor_policy='measured_segment_start',
                segment_handoff='stopped_and_settled')
    plan['sha256'] = digest(plan)
    return plan, replay


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--report', type=Path, default=ROOT/'output/full-route/offline.json')
    parser.add_argument('--library', type=Path, default=ROOT/'output/full-route/native.dll')
    parser.add_argument('--two-batch', action='store_true')
    args = parser.parse_args()
    scope = 'two-batch' if args.two_batch else 'full-route'
    config_name = 'batch' if args.two_batch else 'tour'
    symbol = 'Batch' if args.two_batch else 'Tour'
    macro = 'BATCH' if args.two_batch else 'TOUR'
    (ROOT/'output'/scope).mkdir(parents=True, exist_ok=True)
    plan, replay = freeze(json.loads(args.report.read_text(encoding='utf-8')), args.library.resolve(), args.two_batch)
    (ROOT/'output'/scope/'replay.json').write_text(json.dumps(replay, indent=2), encoding='utf-8')
    (ROOT/'config'/f'{config_name}_route.json').write_text(json.dumps(plan, indent=2)+'\n', encoding='utf-8')
    profile = json.loads((ROOT/'config/raw_trials.json').read_text(encoding='utf-8'))
    profile.update(version=5 if args.two_batch else 4, source='仅限冻结五站整圈；复用已测速度档，新增路段尚未实测')
    (ROOT/'config'/f'{config_name}_trials.json').write_text(json.dumps(profile, ensure_ascii=False, indent=2)+'\n', encoding='utf-8')
    rows = [f'#ifndef {macro}_ROUTE_H', f'#define {macro}_ROUTE_H', '',
            '/* 冻结五站RefSpec88；仅按动作顺序执行，不接受任意整场目标。 */',
            f'#define {macro}_COUNT {len(plan["goals"])}U',
            f'static const unsigned char {symbol}Refs[{macro}_COUNT][88] =', '{']
    for goal in plan['goals']:
        data = bytes.fromhex(goal)[8:]
        rows.append('    {')
        rows.extend('        '+', '.join(f'0x{x:02x}' for x in data[i:i+11])+',' for i in range(0,88,11))
        rows.append('    },')
    rows.extend(['};', '', '#endif', ''])
    (ROOT/'firmware/User/Config'/f'{config_name}_route.h').write_text('\n'.join(rows), encoding='utf-8')
    print(json.dumps(dict(actions=len(plan['goals']), replay_s=replay['elapsed_s'],
                          goals_sha256=hashlib.sha256(b''.join(bytes.fromhex(g) for g in plan['goals'])).hexdigest())))


if __name__ == '__main__':
    main()
