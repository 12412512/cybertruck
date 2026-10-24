"""树莓派唯一通信核心。默认离线；真实串口必须显式指定。"""

import argparse
import json
import logging
from logging.handlers import RotatingFileHandler
import os
from pathlib import Path
import signal
import time
import uuid

from .device.link import Link
from .runtime import ProcessLock, save_snapshot

DEFAULT_RUNTIME = Path(__file__).resolve().parents[2] / 'output/runtime'


def open_transport(mode, port):
    if mode == 'sim':
        from .device.sim import SimTransport
        return SimTransport(time.monotonic)
    import serial
    options = dict(port=None, baudrate=115200, timeout=0, write_timeout=.05)
    if os.name != 'nt':
        options['exclusive'] = True
    device = serial.Serial(**options)
    # 禁止主动切换DTR/RTS复位外设；USB驱动自身枚举行为仍需实机验证。
    device.dtr = device.rts = False
    device.port = port
    device.open()
    return device


def run(mode='offline', port=None, runtime=DEFAULT_RUNTIME, stop=None, task_code=None, start_zone=1,
        owner_url=None, odom_library=None, track_library=None, route_library=None, route_speed=None):
    if mode not in ('offline', 'sim', 'serial', 'observer'):
        raise ValueError('未知运行模式')
    if task_code is not None and mode != 'sim':
        raise ValueError('任务演示仅允许sim模式')
    if odom_library is not None and mode != 'sim':
        raise ValueError('里程计模型仅允许sim模式')
    if track_library is not None and mode != 'sim':
        raise ValueError('闭环模型仅允许sim模式')
    if track_library is not None and odom_library is not None:
        raise ValueError('闭环模型已包含里程计，不能重复加载独立回放')
    if route_library is not None and mode != 'sim':
        raise ValueError('导航模型仅允许sim模式')
    if route_library is not None and (track_library is not None or odom_library is not None):
        raise ValueError('导航模型已包含跟踪器和里程计，不能重复加载')
    if route_speed is not None and (mode != 'sim' or route_library is None):
        raise ValueError('导航速度只允许显式导航模拟，不修改真实控制参数')
    observer = client = None
    if mode == 'observer':
        if port is not None:
            raise ValueError('观察模式不能指定串口')
        from .device.observer import Observer, OwnerClient
        observer, client = Observer(), OwnerClient(owner_url)
    runtime = Path(runtime)
    runtime.mkdir(parents=True, exist_ok=True)
    lock = ProcessLock(runtime / 'core.lock')
    logger = logging.getLogger('robot.core')
    logger.setLevel(logging.INFO)
    handler = RotatingFileHandler(runtime / 'events.jsonl', maxBytes=1024 * 1024,
                                 backupCount=3, encoding='utf-8')
    logger.addHandler(handler)
    link = Link()
    transport = None
    retry_at = written_at = 0.0
    sequence = 0
    identity = uuid.uuid4().hex
    previous = None
    task_demo = None
    odom_demo = None
    track_demo = None
    route_demo = None
    try:
        if route_library is not None:
            from .route_sim import RouteDemo
            route_demo = RouteDemo(route_library,**({'cruise_m_s':route_speed} if route_speed is not None else {}))
        if track_library is not None:
            from .track_sim import TrackDemo
            track_demo = TrackDemo(track_library)
        if odom_library is not None:
            from .odom_sim import OdomDemo
            odom_demo = OdomDemo(odom_library)
        if mode != 'observer' and (task_code is not None or (runtime / 'task.sqlite3').exists()):
            from .task_sim import TaskDemo
            task_demo = TaskDemo(runtime, identity, task_code, start_zone)
        while stop is None or not stop():
            now = time.monotonic()
            sequence += 1
            if mode == 'observer':
                try:
                    source, sent, received = client.read()
                    observer.feed(source, sent, received)
                except (OSError, ValueError) as exc:
                    observer.reset(f'观察连接异常：{type(exc).__name__}')
                now = time.monotonic()
            elif mode != 'offline':
                try:
                    if transport is None and now >= retry_at:
                        transport = open_transport(mode, port)
                        link.reset('传输已连接，核对新协议')
                    if transport is not None:
                        link.feed(transport.read(4096), now)
                        wire = link.step(now)
                        if wire and transport.write(wire) != len(wire):
                            raise OSError('串口写入不完整')
                except (OSError, RuntimeError) as exc:
                    if transport is not None:
                        transport.close()
                    transport = None
                    link.reset(f'连接中断：{exc}')
                    retry_at = now + 1
            state = observer.snapshot(now) if observer is not None else link.snapshot(now)
            if mode == 'offline':
                state['message'] = '离线模式，未打开设备'
            health = (state['message'], state['boot_id'],
                      None if state['board'] is None else (
                          state['board'].get('fault_bits'), state['board'].get('state'),
                          state['board'].get('bus_error')))
            if health != previous:
                previous = health
                logger.info(json.dumps(dict(time_s=time.time(), message=state['message'],
                                            boot_id=state['boot_id'], board=state['board'],
                                            diagnostics=state['diagnostics']), ensure_ascii=False))
            if now - written_at >= .1:
                if task_demo is not None:
                    state['task'] = task_demo.snapshot()
                if odom_demo is not None:
                    state['odometry'] = odom_demo.snapshot()
                if track_demo is not None:
                    state['tracking'] = track_demo.snapshot()
                if route_demo is not None:
                    state['navigation'] = route_demo.snapshot()
                    state['tracking'] = route_demo.model.model.snapshot()
                state.update(mode=mode, core_id=identity, core_sequence=sequence,
                             snapshot_at_s=now, snapshot_wall_s=time.time())
                save_snapshot(runtime / 'state.json', state)
                written_at = now
            if task_demo is not None:
                task_demo.step(now, link_ok=state['connected'])
            if odom_demo is not None:
                odom_demo.step(now)
            if track_demo is not None:
                track_demo.step(now)
            if route_demo is not None:
                route_demo.step(now)
            time.sleep(.1 if mode == 'observer' else .005)
    finally:
        if task_demo is not None:
            task_demo.close()
        if transport is not None:
            transport.close()
        logger.removeHandler(handler)
        handler.close()
        lock.close()


def main():
    parser = argparse.ArgumentParser(description='只读通信核心；无运动接口')
    parser.add_argument('--mode', choices=('offline', 'sim', 'serial', 'observer'), default='offline')
    parser.add_argument('--port', help='显式指定真实串口；不自动猜测或扫描')
    parser.add_argument('--owner-url', help='observer模式：已核对的既有串口所有者http地址')
    parser.add_argument('--odom-library', type=Path, help='仅sim：正式C观察器的主机模型库')
    parser.add_argument('--track-library', type=Path, help='仅sim：正式C跟踪器与独立车体闭环模型')
    parser.add_argument('--route-library', type=Path, help='仅sim：坐标、名义地图与多段闭环巡回')
    parser.add_argument('--route-speed', type=float, help='仅导航sim：巡航m/s，不写实车参数')
    parser.add_argument('--runtime', type=Path, default=DEFAULT_RUNTIME)
    parser.add_argument('--task-demo', help='仅sim：模拟扫码、两批取放和返航，不执行实车动作')
    parser.add_argument('--start-zone', type=int, choices=(1, 2), default=1)
    args = parser.parse_args()
    if args.mode == 'serial' and not args.port:
        parser.error('serial模式必须提供--port')
    if args.mode == 'observer' and not args.owner_url:
        parser.error('observer模式必须提供--owner-url')
    if args.mode != 'observer' and args.owner_url:
        parser.error('--owner-url仅适用于observer模式')
    if args.mode == 'observer' and args.port:
        parser.error('observer模式不打开串口，不能提供--port')
    if args.task_demo and args.mode != 'sim':
        parser.error('--task-demo仅允许--mode sim')
    if args.odom_library is not None and args.mode != 'sim':
        parser.error('--odom-library仅允许--mode sim')
    if args.track_library is not None and args.mode != 'sim':
        parser.error('--track-library仅允许--mode sim')
    if args.track_library is not None and args.odom_library is not None:
        parser.error('--track-library已包含里程计，不能同时指定--odom-library')
    if args.route_library is not None and args.mode != 'sim':
        parser.error('--route-library仅允许--mode sim')
    if args.route_library is not None and (args.odom_library is not None or args.track_library is not None):
        parser.error('--route-library已包含里程计和跟踪器，不能重复加载')
    if args.route_speed is not None and (args.mode != 'sim' or args.route_library is None):
        parser.error('--route-speed只允许显式导航模拟')
    stopped = False

    def finish(*_):
        nonlocal stopped
        stopped = True

    signal.signal(signal.SIGINT, finish)
    signal.signal(signal.SIGTERM, finish)
    run(args.mode, args.port, args.runtime, lambda: stopped, args.task_demo, args.start_zone,
        args.owner_url, args.odom_library, args.track_library, args.route_library,args.route_speed)


if __name__ == '__main__':
    main()
