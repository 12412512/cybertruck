"""树莓派连接准备：仅管理已核对的服务，不拥有串口、不执行运动或清故障。"""

import argparse
from datetime import datetime
import json
import os
from pathlib import Path
import signal
import subprocess
import time
from urllib.request import build_opener, ProxyHandler
import uuid


def read_state(port):
    try:
        with build_opener(ProxyHandler({})).open(f'http://127.0.0.1:{port}/state', timeout=1) as reply:
            return json.load(reply)
    except (OSError, ValueError):
        return None


def device_live(state):
    return bool(state and state.get('domain') == 'physical' and state.get('connected') is True
                and type(state.get('state_age_ms')) in (int, float) and 0 <= state['state_age_ms'] < 250)


def decide(component, state, present, confirmed=False):
    """在线设备一律复用，包含故障、控制权和活动动作；重连不是清故障入口。"""
    if component == 'device':
        if device_live(state):
            return 'reuse'
        if not present:
            raise ValueError('STM32 USB串口未识别，请检查数据线；未启动第二个核心')
        if not confirmed:
            raise ValueError('串口重开可能复位STM32，需确认设备静止、有支撑及旧基准失效')
        return 'start'
    if component == 'vision':
        if not any(present.values()):
            raise ValueError('没有识别到配置中的USB相机')
        cameras = (state or {}).get('cameras', {})
        if state and all(not exists or cameras.get(role, {}).get('available') for role, exists in present.items()):
            return 'reuse'
        return 'start'
    raise ValueError('未知连接组件')


def find_processes(root, module):
    found = []
    for directory in Path('/proc').glob('[0-9]*'):
        try:
            if directory.stat().st_uid != os.getuid():
                continue
            argv = [v.decode() for v in (directory / 'cmdline').read_bytes().split(b'\0') if v]
            if '-m' in argv and argv[argv.index('-m') + 1] == module:
                cwd = (directory / 'cwd').resolve()
                if cwd != root.resolve():
                    raise ValueError(f'{module}存在其他工程的所有者，拒绝接管')
                found.append((int(directory.name), argv))
        except (FileNotFoundError, ProcessLookupError, PermissionError, IndexError):
            continue
    if len(found) > 1:
        raise ValueError('同类服务不唯一，保持现状，请检查进程')
    return found


def check_serial_users(device):
    real = str(device.resolve())
    for fd in Path('/proc').glob('[0-9]*/fd/*'):
        try:
            if os.readlink(fd) == real:
                raise ValueError('串口仍被其他进程占用，拒绝重复打开')
        except (FileNotFoundError, PermissionError, ProcessLookupError):
            pass


def prepare(profile, component, confirmed=False):
    root = Path(profile[f'{component}_root'])
    if not root.is_dir():
        raise ValueError('已部署的服务目录不存在，不自动下载或替换固件')
    module = 'robot.mech_core' if component == 'device' else 'robot.vision.camera'
    port = 8091 if component == 'device' else 8092
    processes = find_processes(root, module)
    state = read_state(port)
    if state is not None and not processes:
        raise ValueError('端口服务身份与工程不符，拒绝接管')
    if component == 'device':
        present = Path(profile['serial']).exists()
    else:
        cameras = json.loads((root / 'config/vision/cameras.json').read_text())
        present = {role: Path(cameras[role]).exists() for role in ('material', 'qr')}
    action = decide(component, state, present, confirmed)
    if action == 'reuse':
        return dict(action='reused', component=component, pid=processes[0][0], state=state)

    # 唯一目录保留全部旧状态及用户保存版本；不覆盖旧故障或示教。
    backup = root / 'output/connections' / (datetime.now().strftime('%Y%m%d_%H%M%S_') + uuid.uuid4().hex[:8])
    backup.mkdir(parents=True, exist_ok=False)
    (backup / 'before.json').write_text(json.dumps(state, ensure_ascii=False, indent=2), encoding='utf8')
    for path in (root / 'runtime').glob('*.json'):
        (backup / path.name).write_bytes(path.read_bytes())
    for pid, argv in processes:
        # 再核对一次PID身份，防止进程退出后的PID复用。
        current = [v.decode() for v in Path(f'/proc/{pid}/cmdline').read_bytes().split(b'\0') if v]
        if current != argv:
            raise ValueError('旧进程身份已变化，未结束进程')
        os.kill(pid, signal.SIGTERM)
        for _ in range(60):
            if not Path(f'/proc/{pid}').exists():
                break
            time.sleep(.1)
        else:
            raise ValueError('旧服务未正常退出，不强杀、不重复启动')
    if component == 'device':
        check_serial_users(Path(profile['serial']))
    env = dict(os.environ, PYTHONPATH=str(root if component == 'device' else root / 'pi'))
    argv = [profile['python'], '-B', '-m', module]
    if component == 'device':
        argv += ['--serial', profile['serial'], '--runtime', str(root / 'runtime'),
                 '--site-ready', '--mechanism-commissioning']
    with (backup / 'service.log').open('ab') as log:
        process = subprocess.Popen(argv, cwd=root, env=env, stdin=subprocess.DEVNULL,
                                   stdout=log, stderr=log, start_new_session=True)
    deadline = time.monotonic() + 10
    while time.monotonic() < deadline:
        state = read_state(port)
        if state and (device_live(state) if component == 'device' else state.get('available')):
            return dict(action='started', component=component, pid=process.pid, backup=str(backup), state=state)
        if process.poll() is not None:
            raise ValueError(f'服务启动退出；日志在{backup}/service.log')
        time.sleep(.1)
    raise ValueError(f'服务尚未就绪，不自动重试；日志在{backup}/service.log')


def main():
    import fcntl
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--component', choices=('device', 'vision'), required=True)
    parser.add_argument('--confirmed-reopen', action='store_true')
    args = parser.parse_args()
    profile = json.loads((Path(__file__).parent / 'profile.json').read_text())
    try:
        with (Path(__file__).parent / f'{args.component}.lock').open('a') as guard:
            fcntl.flock(guard, fcntl.LOCK_EX | fcntl.LOCK_NB)
            result = prepare(profile, args.component, args.confirmed_reopen)
        print(json.dumps(dict(ok=True, **result), ensure_ascii=False))
    except (ValueError, OSError) as error:
        print(json.dumps(dict(ok=False, reason=str(error)), ensure_ascii=False))


if __name__ == '__main__':
    main()
