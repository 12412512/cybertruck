"""经既有串口所有者执行一次架空单轮测试；不接管串口，不重试运动。"""

import argparse
import hashlib
from http.cookiejar import CookieJar
import json
from pathlib import Path
import sys
import time
from urllib.request import HTTPCookieProcessor, ProxyHandler, Request, build_opener

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'pi'))
from robot.device.observer import NoRedirect, OwnerClient


class BenchError(RuntimeError):
    pass


class Client:
    def __init__(self, origin, record):
        OwnerClient(origin)  # 复用正式接口的地址限制。
        self.origin = origin.rstrip('/')
        self.record = record
        self.csrf = None
        self.opener = build_opener(ProxyHandler({}), NoRedirect(),
                                   HTTPCookieProcessor(CookieJar()))

    def call(self, path, body=None):
        if path not in ('/api/bootstrap', '/api/state', '/api/control/acquire',
                        '/api/control/heartbeat', '/api/control/release',
                        '/api/motion', '/api/stop'):
            raise BenchError('接口不在单轮测试白名单')
        headers = {'Accept': 'application/json'}
        payload = None
        if body is not None:
            if not self.csrf:
                raise BenchError('控制会话未建立')
            headers.update({'Content-Type': 'application/json', 'X-CSRF-Token': self.csrf})
            payload = json.dumps(body).encode('utf-8')
            # 发出前留证；超时也不重发运动，Cookie和CSRF不落盘。
            self.record('request', path=path, body=body)
        started = time.monotonic()
        with self.opener.open(Request(self.origin + path, data=payload, headers=headers),
                              timeout=.4) as response:
            raw = response.read(65537)
        if len(raw) > 65536:
            raise BenchError('响应超出容量')
        value = json.loads(raw)
        if path == '/api/bootstrap':
            self.csrf = value.get('csrf')
            self.record('bootstrap', limits=value.get('limits'))
        else:
            self.record('response', path=path, elapsed_s=time.monotonic() - started, data=value)
        return value


def check(state, baseline=None, feedback=True):
    """板端保护仍负责执行；此处额外拒绝失效观察、复位及新增CAN错误。"""
    board = state.get('board', {})
    if (state.get('firmware') != 'CARCAN13' or state.get('connected') is not True
            or not isinstance(state.get('state_age_ms'), (int, float))
            or not 0 <= state['state_age_ms'] < 250):
        raise BenchError('固件或状态时效不符合要求')
    stopping = board.get('state') == 0 and board.get('reason') in (2, 5)
    if board.get('state') == 5 or board.get('bus_error', 0) & ~(1 if stopping else 0):
        raise BenchError('板端故障，结束测试')
    if baseline is not None:
        if (board.get('can_errors') != baseline['can_errors']
                or board.get('uptime_ms', -1) < baseline['uptime_ms']
                or board.get('action') not in (baseline['action'], baseline['action'] + 1)):
            raise BenchError('CAN错误、复位或动作编号异常')
    if not feedback:
        return board
    wheels = state.get('wheels', [])
    if sorted(w.get('address') for w in wheels) != [1, 2, 3, 4]:
        raise BenchError('四轮反馈不完整')
    for wheel in wheels:
        # CARCAN13运行时轮询速度/位置，状态仅保留使能阶段的读取结果。
        # 不把该状态冒充实时值；仅600ms测试内允许其年龄到600+250ms。
        status_recent = (type(wheel.get('status_age_ms')) is int
                         and 0 <= wheel['status_age_ms'] <= 850)
        status_ok = (status_recent if board.get('state') == 4 else wheel.get('fresh'))
        if (not wheel.get('speed_fresh') or not wheel.get('position_fresh')
                or not status_ok or wheel.get('status', 12) & 12):
            raise BenchError('轮反馈过期或驱动器错误')
    return board


def idle(state):
    board = check(state)
    if (state.get('ready') is not True or state.get('stopped') is not True
            or state.get('control_busy') or state.get('action')
            or board.get('ready_mask') != 15 or board.get('stop_observed_mask') != 15
            or board.get('state') != 1 or board.get('pending_stops')
            or any(abs(w['rpm']) > 1 for w in state['wheels'])):
        raise BenchError('未取得四轮停车或控制空闲证据')
    return board


def run(client, address, record, clock=time.monotonic, sleep=time.sleep):
    client.call('/api/bootstrap')
    first = idle(client.call('/api/state'))
    sleep(.16)
    baseline = idle(client.call('/api/state'))
    if (baseline['uptime_ms'] <= first['uptime_ms']
            or baseline['can_rx_sequence'] <= first['can_rx_sequence']
            or baseline['action'] != first['action']
            or baseline['can_errors'] != first['can_errors']):
        raise BenchError('板时或CAN反馈未推进，或基线发生变化')
    targets = [0, 0, 0, 0]
    targets[address - 1] = 6
    attempted = False
    acquired = False
    outcome = 'unknown'
    try:
        # acquire响应丢失时也要撤销可能已取得的租约。
        acquired = True
        client.call('/api/control/acquire', {})
        idle(client.call('/api/state'))
        attempted = True
        accepted = client.call('/api/motion', {'wheel_rpm': targets, 'duration_ms': 600})
        token = accepted.get('token')
        if (accepted.get('accepted') is not True or not token
                or accepted.get('wheel_rpm') != targets or accepted.get('duration_ms') != 600):
            raise BenchError('动作接受信息不一致；不重发')
        deadline, last_heartbeat = clock() + 7, -1e9
        while clock() < deadline:
            if clock() - last_heartbeat >= .2:
                client.call('/api/control/heartbeat', {})
                last_heartbeat = clock()
            state = client.call('/api/state')
            board = check(state, baseline)
            if any(abs(w['rpm']) > (8 if w['address'] == address else 1)
                   for w in state['wheels']):
                raise BenchError('轮速超出单轮低速测试范围')
            last = state.get('last_action') or {}
            active = state.get('action')
            if active and active.get('token') != token:
                raise BenchError('既有所有者动作身份不一致')
            if last.get('result') == 'cancelled' and not active:
                raise BenchError('所有者取消动作，保留失败证据')
            if (last.get('result') == 'completed' and last.get('action') == baseline['action'] + 1
                    and last.get('targets') == targets and last.get('duration_ms') == 600
                    and board.get('reason') == 5):
                idle(state)
                outcome = 'timed_completed_stop_reported'
                break
            sleep(.08)
        else:
            raise BenchError('动作结果等待超时，不重发运动')
    finally:
        if acquired:
            # 正常释放及异常撤销都只发一次；异常时STOP是停止请求，不是清错重试。
            path = '/api/control/release' if outcome != 'unknown' else '/api/stop'
            try:
                client.call(path, {})
            except Exception as exc:
                record('stop_request_failed', reason=type(exc).__name__)
                outcome = 'stop_request_unconfirmed'
    if outcome != 'timed_completed_stop_reported':
        raise BenchError('没有确认动作和停止结果')
    deadline = clock() + 4
    stable = None
    while clock() < deadline:
        state = client.call('/api/state')
        # 停止已请求，等新反馈刷新；不能把停止过渡中的旧缓存当到位证据。
        check(state, baseline, feedback=False)
        if state.get('stopped') and not state.get('control_mine'):
            board = idle(state)
            if stable and board['uptime_ms'] > stable:
                record('result', outcome=outcome, address=address, targets=targets,
                       action=board['action'], motion_posts=int(attempted),
                       physical_wheel='待用户目视确认', mapping_verified=False)
                return
            stable = board['uptime_ms']
        sleep(.16)
    raise BenchError('释放后的新四轮停车反馈未确认')


def main():
    parser = argparse.ArgumentParser(description='四轮可靠离地且固定后，一次单轮6RPM/600ms测试')
    parser.add_argument('--owner-url', required=True)
    parser.add_argument('--address', type=int, choices=range(1, 5), required=True)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--execute', action='store_true', help='显式执行已确认现场条件的实测')
    args = parser.parse_args()
    if not args.execute:
        parser.error('未指定--execute，不连接设备')
    args.output.parent.mkdir(parents=True, exist_ok=True)
    with args.output.open('x', encoding='utf-8') as file:
        def record(kind, **fields):
            file.write(json.dumps(dict(kind=kind, mono_s=time.monotonic(), **fields),
                                  ensure_ascii=False, allow_nan=False) + '\n')
            file.flush()
        record('setup', address=args.address, rpm=6, duration_ms=600,
               wheels_suspended_confirmed=True, firmware_expected='CARCAN13',
               tool_sha256=hashlib.sha256(Path(__file__).read_bytes()).hexdigest())
        client = Client(args.owner_url, record)
        try:
            run(client, args.address, record)
        except Exception as exc:
            record('failed', reason=type(exc).__name__, detail=str(exc))
            # 异常后只留只读反馈，不重发STOP或动作，不自动清锁存。
            for _ in range(12):
                try:
                    client.call('/api/state')
                except Exception:
                    pass
                time.sleep(.16)
            raise SystemExit(f'测试中止：{exc}；不重发运动，请核对车轮静止及证据') from exc
    print(f'单次结果已保存：{args.output}；仍需目视确认实际轮位和方向')


if __name__ == '__main__':
    main()
