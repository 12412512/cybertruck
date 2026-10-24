"""观察既有CAR服务。唯一网络操作是GET状态，不打开或写入串口。"""

from copy import deepcopy
import json
import math
import time
from urllib.parse import urlsplit
from urllib.request import HTTPRedirectHandler, ProxyHandler, Request, build_opener


class NoRedirect(HTTPRedirectHandler):
    def redirect_request(self, req, fp, code, msg, headers, newurl):
        return None


class OwnerClient:
    def __init__(self, origin):
        address = urlsplit(origin)
        if (address.scheme != 'http' or not address.hostname or address.username
                or address.password or address.path not in ('', '/')
                or address.query or address.fragment):
            raise ValueError('只允许明确的http服务地址，不含凭据、路径或查询参数')
        # 固定路径、禁止重定向/环境代理；不接受任意动作URL。
        self.url = origin.rstrip('/') + '/api/state'
        self.opener = build_opener(ProxyHandler({}), NoRedirect())

    def read(self):
        started = time.monotonic()
        request = Request(self.url, headers={'Accept': 'application/json'}, method='GET')
        with self.opener.open(request, timeout=.25) as response:
            if response.status != 200:
                raise ValueError('既有服务状态码异常')
            raw = response.read(65537)
        if len(raw) > 65536:
            raise ValueError('既有服务状态超过容量')
        state = json.loads(raw, parse_constant=lambda _: None)
        return state, started, time.monotonic()


def number(value, minimum=0):
    return (type(value) in (int, float) and math.isfinite(value) and value >= minimum)


class Observer:
    def __init__(self):
        self.reset('等待既有串口所有者的状态')

    def reset(self, reason):
        self.source = None
        self.received = self.rtt_ms = None
        self.uptime = self.advanced = None
        self.verified = False
        self.message = reason

    def feed(self, state, sent, received):
        if not number(sent) or not number(received) or received < sent:
            raise ValueError('观察时间无效')
        if not isinstance(state, dict):
            raise ValueError('既有服务状态必须为对象')
        board = state.get('board')
        if (state.get('firmware') != 'CARCAN13' or not isinstance(board, dict)
                or type(board.get('uptime_ms')) is not int
                or not 0 <= board['uptime_ms'] <= 0xffffffff
                or type(board.get('state')) is not int or not 0 <= board['state'] <= 5
                or any(type(board.get(key)) is not int or board[key] < 0
                       for key in ('reason', 'bus_error', 'can_errors', 'can_last_error'))
                or not number(state.get('state_age_ms'))
                or not isinstance(state.get('wheels', []), list)):
            raise ValueError('固件或状态结构不符合已核对的CARCAN13')
        # 同一板上时间不能因HTTP重复响应变得更新。重启/乱序后重新核对两次。
        uptime = board['uptime_ms']
        delta = None if self.uptime is None else (uptime - self.uptime) & 0xffffffff
        if delta is None or delta >= 0x80000000:
            self.verified = False
            self.advanced = received
        elif delta > 0:
            self.verified = True
            self.advanced = received
        self.uptime = uptime
        self.source = deepcopy(state)
        self.received = received
        self.rtt_ms = (received - sent) * 1000
        self.message = '既有服务只读观察；运动接口关闭'

    def snapshot(self, now):
        source = self.source or {}
        local_ms = 0 if self.received is None else max(0, (now - self.received) * 1000)
        age = None if self.received is None else source['state_age_ms'] + self.rtt_ms + local_ms
        fresh = (self.verified and source.get('connected') is True and age is not None
                 and age < 250 and now >= self.received and now - self.advanced < .25)
        board = deepcopy(source['board']) if fresh else None
        wheels = []
        for raw in source.get('wheels', [])[:4] if fresh else []:
            if not isinstance(raw, dict):
                continue
            wheel = {key: raw.get(key) for key in ('address', 'rpm', 'position', 'status')}
            for field in ('speed', 'position', 'status'):
                old_age = raw.get(f'{field}_age_ms')
                # 旧API只更新fresh布尔值，age数值没加其内部缓存停留时间。
                # 只能显示源报告，不能产生可用于闭环/时间对齐的精确有效反馈。
                wheel[f'{field}_reported_age_ms'] = old_age if number(old_age) else None
                wheel[f'{field}_age_ms'] = None
                wheel[f'{field}_valid'] = False
                value_key = 'rpm' if field == 'speed' else field
                value = wheel[value_key]
                flag = raw.get('fresh') if field == 'status' else raw.get(f'{field}_fresh')
                wheel[f'{field}_reported'] = (number(old_age) and old_age < 250
                                              and self.rtt_ms + local_ms < 250
                                              and number(value, -1e15) and flag is True)
            wheels.append(wheel)
        imu = deepcopy(source.get('imu')) if fresh and isinstance(source.get('imu'), dict) else None
        if imu:
            for field in ('angle', 'gyro'):
                old_age = imu.get(f'{field}_age_ms')
                value_keys = ('yaw_deg', 'pitch_deg', 'roll_deg') if field == 'angle' else ('gx_dps', 'gy_dps', 'gz_dps')
                imu[f'{field}_reported'] = (imu.get('supported') is True
                                            and imu.get(f'{field}_fresh') is True
                                            and number(old_age) and old_age < 250
                                            and self.rtt_ms + local_ms < 250
                                            and all(number(imu.get(key), -1e15) for key in value_keys))
                imu[f'{field}_reported_age_ms'] = old_age if number(old_age) else None
                imu[f'{field}_age_ms'] = None
                imu[f'{field}_fresh'] = False
        message = self.message if fresh else '既有设备反馈未确认或已过期'
        if board and (board['state'] == 5 or board['bus_error']):
            message = '实车报告故障；只读观察，禁止运动'
        events = [message]
        if fresh and isinstance(source.get('events'), list):
            for event in source['events'][-8:]:
                if isinstance(event, dict) and isinstance(event.get('message'), str):
                    events.append(f"旧服务 {str(event.get('time', ''))[:40]} {event['message'][:200]}")
        return dict(connected=bool(fresh), readonly=True, ready=False,
                    message=message, firmware=source.get('firmware', ''),
                    boot_id=None, session_id=None, state_age_ms=age, board=board,
                    source_protocol='CARCAN13-owner-http', gc_verified=False,
                    feedback_timing_verified=False,
                    wheels=wheels, imu=imu, diagnostics=dict(
                        source_stopped=source.get('stopped') if fresh else None,
                        can_error_event=source.get('can_error_event') if fresh else None,
                        uart_bad_crc=source.get('uart_bad_crc') if fresh else None,
                        uart_bad_line=source.get('uart_bad_line') if fresh else None,
                        owner_busy=source.get('control_busy') if fresh else None,
                        imu=source.get('imu_diagnostics') if fresh else None,
                        rtt_ms=self.rtt_ms),
                    pose=None, pose_valid=False, zero_valid=False,
                    physical_stop_confirmed=False, task=None, grabs=None, places=None,
                    rx_good=None, rx_bad=None, events=events)
