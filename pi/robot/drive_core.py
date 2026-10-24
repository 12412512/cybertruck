"""独立短段执行核心：默认只准备，不自动部署、接管串口或执行整图。"""

import argparse
import json
from pathlib import Path
import time
from urllib.error import URLError
from urllib.request import ProxyHandler, build_opener
import uuid

from .device.execution import MotionLink, CONFIG, ARM, GOAL, config_payload, trial, trials
from .device.observer import NoRedirect, OwnerClient
from .main import open_transport
from .runtime import ProcessLock, save_snapshot


def require_owner_stopped(origin):
    """旧服务仍可访问就拒绝接管，connected=false也不表示串口已释放。"""
    client = OwnerClient(origin)
    try:
        build_opener(ProxyHandler({}), NoRedirect()).open(client.url, timeout=.5).close()
    except URLError as exc:
        if isinstance(exc.reason, ConnectionRefusedError):
            return
        raise ValueError('无法确认旧所有者已停止；不打开串口') from exc
    raise ValueError('旧串口所有者仍在运行；不得抢占或自动停止它')


def run_trial(transport, profile, case, runtime, record, clock=time.monotonic, sleep=time.sleep):
    """只执行一个明确短段；任何不确定结果均停止，不重发目标。"""
    config = config_payload(profile)
    link = MotionLink()
    stage = 'connect'
    deadline = clock() + 40
    arm_attempted = stop_requested = False
    action_id = None
    completed = False
    goal_accepted = False
    identity = uuid.uuid4().hex
    sequence = 0
    written_at = -1e9

    def pump():
        nonlocal sequence, written_at
        now = clock()
        link.feed(transport.read(4096), now)
        wire = link.step(now)
        if wire and transport.write(wire) != len(wire):
            raise OSError('串口写入不完整，不能重发运动')
        state = link.snapshot(now)
        sequence += 1
        state.update(mode='serial', core_id=identity, core_sequence=sequence,
                     snapshot_at_s=now, snapshot_wall_s=time.time())
        if now - written_at >= .1:
            save_snapshot(Path(runtime) / 'state.json', state)
            record('state', execution=state.get('execution'), stage=stage,
                   connected=state['connected'], state_age_ms=state['state_age_ms'])
            written_at = now
        sleep(.005)
        return state.get('execution')

    try:
        while clock() < deadline:
            state = pump()
            if link.failed:
                raise ValueError('应答丢失、会话变化或命令拒绝，结果需核对')
            if state and (state['fault'] or state['can_errors'] or state['mode'] >= 5):
                record('abort', execution=state, stage=stage)
                raise ValueError('设备报告故障或动作中断')
            if stage == 'connect' and link.bound and state and (state['mode'] != 0 or state['stop_confirmed']):
                if link.firmware != 'GC-BASE-0.3.3' or state['mode'] not in (0, 1):
                    raise ValueError('固件或使能状态不符合冷启动短段入口')
                record('intent', operation='config', profile=profile)
                link.submit(CONFIG, config)
                stage = 'config'
            elif stage == 'config' and link.result == dict(kind=CONFIG, accepted=True):
                record('accepted', operation='config')
                stage = 'observe'
            elif stage == 'observe' and state and state['mode'] == 1 and state['odom_valid']:
                record('intent', operation='arm')
                arm_attempted = True
                link.submit(ARM)
                stage = 'arm'
            elif stage == 'arm' and link.result == dict(kind=ARM, accepted=True):
                record('accepted', operation='arm')
                stage = 'ready'
            elif stage == 'ready' and state and state['mode'] == 3:
                action_id = state['action_id'] + 1
                payload = trial(case, action_id)
                record('intent', operation='goal', case=case, action_id=action_id,
                       boot_id=link.boot, payload_hex=payload.hex())
                link.submit(GOAL, payload)
                stage = 'running'
            elif stage == 'running' and not goal_accepted and link.result == dict(kind=GOAL, accepted=True):
                record('accepted', operation='goal', action_id=action_id)
                goal_accepted = True
            elif (stage == 'running' and state and state['action_id'] == action_id
                  and state['action_state'] == 4 and state['mode'] == 3):
                completed = True
                record('controller_complete', action_id=action_id,
                       external_measurement='待独立尺量/角度确认，不增加任务成功计数')
                break
        else:
            raise TimeoutError('短段整体期限超时')
    finally:
        if arm_attempted:
            record('intent', operation='stop')
            link.stop()
            stop_requested = True
            stopped = False
            end = clock() + 4
            while clock() < end:
                try:
                    state = pump()
                except (OSError, ValueError):
                    break
                if state and state['stop_confirmed']:
                    stopped = True
                    break
            record('stop_result', requested=stop_requested, confirmed=stopped)
            if not stopped:
                raise RuntimeError('停止尚未确认，请现场核对并立即停机；禁止自动续跑')
    return dict(controller_complete=completed, case=case, action_id=action_id,
                external_measurement_verified=False, motion_replayed=False)


def observe(transport, runtime, seconds, clock=time.monotonic, sleep=time.sleep):
    """新固件只读验收；未知IMU符号时也可看原始值，不配置或使能。"""
    link = MotionLink()
    identity = uuid.uuid4().hex
    deadline = clock() + seconds
    sequence = 0
    while clock() < deadline:
        now = clock()
        link.feed(transport.read(4096), now)
        wire = link.step(now)
        if wire and transport.write(wire) != len(wire):
            raise OSError('只读查询写入不完整')
        state = link.snapshot(now)
        sequence += 1
        state.update(mode='serial', core_id=identity, core_sequence=sequence,
                     snapshot_at_s=now, snapshot_wall_s=time.time())
        if sequence % 20 == 0:
            save_snapshot(Path(runtime) / 'state.json', state)
        sleep(.005)


def main():
    parser = argparse.ArgumentParser(description='默认离线准备；显式执行仅一个短段')
    parser.add_argument('--case', choices=('straight', 'side', 'turn'), default='straight')
    parser.add_argument('--profile', type=Path, default=Path('config/base_trials.json'))
    parser.add_argument('--runtime', type=Path, default=Path('output/base-trials'))
    parser.add_argument('--owner-url', default='http://172.20.10.4:8080')
    parser.add_argument('--port', help='旧所有者已释放后，明确指定唯一串口')
    parser.add_argument('--execute', action='store_true')
    parser.add_argument('--observe', action='store_true', help='新固件只读查询，不配置或使能')
    parser.add_argument('--seconds', type=float, default=30)
    parser.add_argument('--accept-candidate', action='store_true', help='仅试验：接受尚未实测的声明参数')
    parser.add_argument('--site-ready', action='store_true', help='本次现场已确认，不从历史快照继承')
    args = parser.parse_args()
    profile = json.loads(args.profile.read_text(encoding='utf-8'))
    if args.execute and args.observe:
        parser.error('只读与执行不能同时选择')
    if not 1 <= args.seconds <= 3600:
        parser.error('只读窗口须为1～3600秒')
    if not args.execute and not args.observe:
        preparation = trials()
        preparation.update(selected_case=args.case, profile=profile, command_sent=False)
        print(json.dumps(preparation, ensure_ascii=False, indent=2))
        return
    if not args.port or (args.execute and (not args.accept_candidate or not args.site_ready)):
        parser.error('执行需要显式串口、候选参数确认及本次现场条件；默认不连接设备')
    if args.execute:
        try:
            config_payload(profile)  # 执行参数未齐全时，在任何连接之前退出。
        except ValueError as exc:
            parser.error(str(exc))
    require_owner_stopped(args.owner_url)
    args.runtime.mkdir(parents=True, exist_ok=True)
    lock = ProcessLock(args.runtime / 'core.lock')
    transport = None
    evidence = args.runtime / f'trial-{uuid.uuid4().hex}.jsonl'
    try:
        with evidence.open('x', encoding='utf-8') as file:
            def record(kind, **fields):
                file.write(json.dumps(dict(kind=kind, mono_s=time.monotonic(), **fields),
                                      ensure_ascii=False, allow_nan=False) + '\n')
                file.flush()
            transport = open_transport('serial', args.port)
            try:
                if args.observe:
                    observe(transport, args.runtime, args.seconds)
                else:
                    result = run_trial(transport, profile, args.case, args.runtime, record)
                    record('result', **result)
                    print(json.dumps(result, ensure_ascii=False))
            except BaseException as exc:
                record('failed', reason=type(exc).__name__, detail=str(exc))
                raise
    finally:
        if transport is not None:
            transport.close()
        lock.close()


if __name__ == '__main__':
    main()
