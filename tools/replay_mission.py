"""B端任务合同合成回放；只写独立output，不启动服务、相机或串口。"""

import argparse
import hashlib
import json
from pathlib import Path
import sys

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'pi'))

from robot.ledger import Ledger
from robot.task import phases, reduce
from robot.task_replay import OfflineMission


class Clock:
    def __init__(self):
        self.value = 1.0

    def __call__(self):
        return self.value


def receipt(request, status, clock):
    value = dict(domain='offline_fixture', source_kind='synthetic_execution',
                 request=request, status=status, observed_s=clock())
    if status == 'COMPLETE':
        value['result'] = dict(ok=True, stopped=True, claw_empty=True, transport_safe=True,
                               station=request['goal'].get('station'), complete=True, coverage_valid=True)
    return value


def context(view):
    return dict(camera_id='fixture-camera', camera_config_sha256='a' * 64, image_size=[1280, 720],
                pose_id='fixture:' + view, pose_revision=1, pose_sha256='b' * 64,
                target_plane='fixture:' + view, view=view)


def evidence(mission, clock, rule, sequence, code):
    phase, view, observed, color, identity = rule
    mission.pose(phase, context(view))
    request = mission.pending_request()
    pose = mission.task.state['pending']['pose']
    frames = []
    for offset in range(3):
        clock.value += .05
        sequence += 1
        frames.append(dict(domain='offline_fixture', source_kind='synthetic_assertion',
                           request_sha256=request['request_sha256'], context=context(view),
                           phase=phase, pose_epoch=pose['epoch'], view=view, frame_id=sequence,
                           evidence_id=f'synthetic:{sequence}', captured_s=clock(), quality=1.0,
                           state=observed, color=color, item=identity, code=code,
                           ring=request['goal'].get('ring') if view.startswith(('rough:', 'stage:')) else None,
                           claw_empty=True, placement_valid=True, stable=True))
    return frames, sequence


def run(runtime, code='136+321+631+312', zone=1):
    runtime = Path(runtime)
    # 独立回放目录有旧账本时直接拒绝，不覆写或自动恢复。
    if (runtime / 'task.sqlite3').exists():
        raise ValueError('回放目录已有账本；请显式选择新的输出目录')
    ledger = Ledger(runtime / 'task.sqlite3')
    clock = Clock()
    try:
        mission = OfflineMission.create(ledger, 'fixture-owner', clock,
                                        dict(owner='fixture-core', boot_id='fixture-boot', session_id='fixture-session'),
                                        dict(revision=1, sha256=hashlib.sha256(b'fixture-parameters-v1').hexdigest()), zone)
        mission.task.start()
        sequence, locations = 0, []
        while mission.task.state['status'] != 'COMPLETE':
            request = mission.request()
            for status in ('ACCEPTED', 'RUNNING'):
                mission.receive(receipt(request, status, clock))
            for rule in phases(request['goal'], mission.task.state):
                frames, sequence = evidence(mission, clock, rule, sequence, code)
                mission.checkpoint(frames)
            mission.receive(receipt(request, 'COMPLETE', clock))
            mission.confirm()
            state = mission.task.state
            if request['goal'].get('item') == 'b1-1':
                locations.append(state['items']['b1-1']['location'])
        ledger.audit(mission.task.run_id, reduce)
        report = dict(mission.snapshot(), source_kind='synthetic_fixture', code=code, start_zone=zone,
                      single_item_locations=locations, ledger_audit=True,
                      limitations=['合成断言不是视觉实测', '不批准自动对中或实物抓放', '接口待A审查接入'])
        runtime.mkdir(parents=True, exist_ok=True)
        (runtime / 'report.json').write_text(json.dumps(report, ensure_ascii=False, indent=2), encoding='utf-8')
        return report
    finally:
        ledger.close()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--runtime', type=Path, required=True)
    parser.add_argument('--code', default='136+321+631+312')
    parser.add_argument('--start-zone', type=int, choices=(1, 2), default=1)
    args = parser.parse_args()
    report = run(args.runtime, args.code, args.start_zone)
    print(json.dumps(dict(status=report['simulation']['status'], domain=report['domain'],
                          simulation_counts=report['simulation']['simulation_counts'],
                          physical_acceptance=False), ensure_ascii=False))


if __name__ == '__main__':
    main()
