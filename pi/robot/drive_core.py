"""单段与冻结短组合执行核心：默认仅准备，不自动接管串口或执行整图。"""

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


def run_trial(transport, profile, case, runtime, record, clock=time.monotonic, sleep=time.sleep, route=None, simulation=False):
    """同一会话执行冻结目标；异常停止，不重发或重连续跑。"""
    if route is not None:
        from .nav.route import validate
        validate(route)
    cases = tuple(route['cases']) if route else (case,)
    config = config_payload(profile)
    link = MotionLink()
    stage = 'connect'
    deadline = clock() + (90 if route else 40)
    arm_attempted = stop_requested = False
    action_id = None
    completed = False
    goal_accepted = False
    identity = uuid.uuid4().hex
    sequence = 0
    written_at = -1e9
    index = 0
    completed_segments = []
    trace = []
    pinned_session = None
    route_origin = None
    route_reference = route['reference_trace'] if route else None

    def pump(force=False):
        nonlocal sequence, written_at
        now = clock()
        link.feed(transport.read(4096), now)
        wire = link.step(now)
        if wire and transport.write(wire) != len(wire):
            raise OSError('串口写入不完整，不能重发运动')
        state = link.snapshot(now)
        sequence += 1
        state.update(mode='sim' if simulation else 'serial', core_id=identity, core_sequence=sequence,
                     snapshot_at_s=now, snapshot_wall_s=time.time())
        if route:
            execution = state.get('execution')
            if execution and execution['odom_valid'] and (not trace or trace[-1][0] != execution['board_ms']):
                trace.append([execution['board_ms'], execution['x_m'], execution['y_m'], execution['yaw_rad']])
                if len(trace) > 2000:
                    trace.pop(0)
            state['route_execution'] = dict(domain='simulation' if simulation else 'commissioning',
                id=route['id'], plan_sha256=route['sha256'], frame_id='odom', status=stage,
                index=index, total=len(cases), completed=len(completed_segments),
                cases=list(cases), reference_trace=route_reference, origin=route_origin,
                trace=[p[1:] for p in trace], pose=trace[-1][1:] if trace else None,
                session_continuous=pinned_session is not None and (link.boot, link.session) == pinned_session,
                anchor_policy='每段实际起点；蓝线为名义累计参考',
                reason='段间完成停车稳定屏障；尚非跨段非零速度衔接',
                map_registered=False, physical_precision_verified=False)
        if force or now - written_at >= .1:
            save_snapshot(Path(runtime) / 'state.json', state)
            record('state', execution=state.get('execution'), stage=stage,
                   connected=state['connected'], state_age_ms=state['state_age_ms'],
                   route_segment=index if route else None, boot_id=link.boot, session_id=link.session)
            written_at = now
        sleep(.005)
        return state.get('execution')

    try:
        while clock() < deadline:
            state = pump()
            if pinned_session is not None and (link.boot, link.session) != pinned_session:
                raise ValueError('路线启动身份或会话变化，禁止自动续跑')
            if link.failed:
                raise ValueError('应答丢失、会话变化或命令拒绝，结果需核对')
            if state and (state['fault'] or state['can_errors'] or state['mode'] >= 5):
                record('abort', execution=state, stage=stage)
                raise ValueError('设备报告故障或动作中断')
            if stage == 'connect' and link.bound and state and (state['mode'] != 0 or state['stop_confirmed']):
                if link.firmware != 'GC-BASE-0.3.6' or state['mode'] not in (0, 1):
                    raise ValueError('固件或使能状态不符合冷启动短段入口')
                pinned_session = (link.boot, link.session)
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
                if route and route_origin is None:
                    from .nav.frames import Pose, compose
                    route_origin = [state['x_m'], state['y_m'], state['yaw_rad']]
                    origin = Pose(*route_origin)
                    route_reference = [compose(origin, Pose(*point)).values() for point in route['reference_trace']]
                    record('route_origin', odom=route_origin, boot_id=link.boot, session_id=link.session)
                action_id = state['action_id'] + 1
                payload = trial(cases[index], action_id)
                record('intent', operation='goal', case=cases[index], segment=index, action_id=action_id,
                       boot_id=link.boot, payload_hex=payload.hex())
                link.submit(GOAL, payload)
                stage = 'running'
            elif stage == 'running' and not goal_accepted and link.result == dict(kind=GOAL, accepted=True):
                record('accepted', operation='goal', action_id=action_id)
                goal_accepted = True
            elif (stage == 'running' and state and state['action_id'] == action_id
                  and state['action_state'] == 4 and state['mode'] == 3):
                record('controller_complete', action_id=action_id, execution=state,
                       external_measurement='待独立尺量/角度确认，不增加任务成功计数')
                completed_segments.append(dict(segment=index, case=cases[index], execution=state))
                index += 1
                if index == len(cases):
                    completed = True
                    stage = 'finishing'
                    break
                goal_accepted = False
                stage = 'ready'
        else:
            raise TimeoutError('路线或短段整体期限超时')
    finally:
        if arm_attempted:
            record('intent', operation='stop')
            link.stop()
            stop_requested = True
            stopped = False
            end = clock() + 4
            while clock() < end:
                try:
                    state = pump(force=True)
                except (OSError, ValueError):
                    break
                if state and state['stop_confirmed']:
                    stopped = True
                    break
            # 最终状态无论限频均落盘；异常不能留下仍在运行的路线状态。
            stage = 'complete' if completed and stopped else 'failed'
            if route:
                snapshot = link.snapshot(clock())
                snapshot.update(mode='sim' if simulation else 'serial', core_id=identity,
                                core_sequence=sequence + 1, snapshot_at_s=clock(), snapshot_wall_s=time.time())
                previous = json.loads((Path(runtime) / 'state.json').read_text(encoding='utf-8'))
                snapshot['route_execution'] = dict(previous['route_execution'], status=stage,
                    index=index, completed=len(completed_segments))
                save_snapshot(Path(runtime) / 'state.json', snapshot)
            record('stop_result', requested=stop_requested, confirmed=stopped, execution=state)
            if not stopped:
                raise RuntimeError('停止尚未确认，请现场核对并立即停机；禁止自动续跑')
    return dict(controller_complete=completed, case='short_route' if route else case, action_id=action_id,
                completed_segments=completed_segments,
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
    parser = argparse.ArgumentParser(description='默认仅准备；显式执行单段或冻结短组合')
    parser.add_argument('--case', choices=('straight', 'side', 'turn', 'turn90', 'diagonal', 'arc', 'pose'), default='straight')
    parser.add_argument('--route', type=Path, help='仅接受已冻结的短组合路线；不接受场地整图')
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
    route = None
    if args.route:
        from .nav.route import validate
        route = json.loads(args.route.read_text(encoding='utf-8'))
        validate(route)
        if args.observe:
            parser.error('只读观察不能选择执行路线')
    if args.execute and args.observe:
        parser.error('只读与执行不能同时选择')
    if not 1 <= args.seconds <= 3600:
        parser.error('只读窗口须为1～3600秒')
    if not args.execute and not args.observe:
        preparation = trials()
        preparation.update(selected_case='short_route' if route else args.case, route=route, profile=profile, command_sent=False)
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
                    result = run_trial(transport, profile, args.case, args.runtime, record, route=route)
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
