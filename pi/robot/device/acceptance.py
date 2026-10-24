"""已结束的实车只读验收记录；不打开设备，不产生实时或动作许可状态。"""

import hashlib
import json
from pathlib import Path


READONLY_KINDS = {1, 2, 3, 4, 5, 16, 35, 36, 37, 38}


def build_report(source, observed_at_s, ram=None, swd=None):
    raw = Path(source).read_bytes()
    data = json.loads(raw)
    summary = data['summary']
    rows = [row['execution'] for row in data['states'] if row.get('execution')]
    if not rows:
        raise ValueError('没有有效板端执行状态，不能生成停车验收结论')
    requests = summary['requests']
    readonly = bool(requests) and all(int(kind) in READONLY_KINDS for kind in requests)
    no_actions = all(not row['action_id'] and not row['action_state'] and not row['config_version'] for row in rows)
    passed = bool(summary['passed'] and readonly and no_actions
                  and all(not row['fault'] and not row['can_errors'] and row['stop_confirmed'] for row in rows))
    transition = next((index for index, row in enumerate(rows) if row['fault']), None)
    feedback = []
    for kind, name in [(1, '轮速'), (2, '轮位置'), (3, '轮状态'), (4, 'IMU姿态'), (5, 'IMU角速度')]:
        samples = [row for row in data['feedback'] if row['kind'] == kind]
        feedback.append(dict(name=name, samples=len(samples),
                             device_age_max_ms=max(((row['report_ms'] - row['last_ms']) & 0xFFFFFFFF for row in samples), default=None),
                             rtt_max_ms=max((row['rtt_ms'] for row in samples), default=None)))
    value = dict(schema=1, domain='hardware_acceptance', live=False, connected=False,
                 authorizes_motion=False, observed_at_s=observed_at_s,
                 source_sha256=hashlib.sha256(raw).hexdigest(), passed=passed,
                 firmware=summary['firmware'], boot_id=summary['boot_id'],
                 first_execution=rows[0], last_execution=rows[-1],
                 transition=[] if transition is None else rows[max(0, transition - 1):transition + 1],
                 readonly_commands=readonly, no_actions=no_actions, requests=requests,
                 checks=summary['checks'], wheels=summary['wheels'], feedback=feedback,
                 can_rx_delta=summary['can_rx_delta'], uart_diag=summary['uart_diag'],
                 odom_fault_record=summary['odom_fault_record'],
                 can_fault_record=summary.get('can_fault_record'),
                 can_capture_error=summary.get('can_capture_error'),
                 restrictions=['当前板端锁存故障未清除，禁止底盘及机构实车动作' if not passed else '本记录不提供动作许可',
                               '本次读取已结束、串口已释放；当前状态需重新只读核对，不能把本记录当实时停车保证',
                               f'首个执行状态在启动约{rows[0]["board_ms"] / 1000:.2f}秒读到；打开串口可能复位，不能据此证明打开前无故障',
                               'ID5/6机构反馈、人工零点、舵机真实角度均未接入；不使能、不建零、不输出PWM',
                               '闭环配置版本为0，odom无效；本记录不提供地图位姿，模拟位姿不能代替',
                               'Web方向、路线、速度和示教仍仅作用于模拟域；未开启实车控制'],
                 fault_explanation='0x800对应Base_Run中只读CAN查询未正常发送结束；不是里程计子故障。')
    if ram and swd:
        snapshots = ram['snapshots']
        can = snapshots[-1]['can_diag']
        value['ram'] = dict(observed_at_s=swd['finished_wall_s'],
                            running_before=swd['state_before'], running_after=swd['state_after'],
                            approved_0311_equal=swd['approved_0311_equal'], verified_bytes=swd['approved_0311_bytes'],
                            errors=can['errors'], tx_failed=can['tx_failed'], arbitration_lost=can['arbitration_lost'],
                            tx_timeouts=can['tx_timeouts'], rx_overflow=can['rx_overflow'],
                            last_error=can['last_error'], error_tick=can['error_tick'],
                            error_tx_id=can['error_tx_id'], error_tx_length=can['error_tx_length'],
                            error_tx_data0=can['error_tx_data0'], error_esr=can['error_esr'],
                            lec=(can['error_esr'] >> 4) & 7, tec=(can['error_esr'] >> 16) & 255,
                            bus_locked=snapshots[-1]['bus_diag']['locked'],
                            note='最近一次冻结错误，非首次错误；运行中非原子快照。不能仅据此认定接线或软件根因。')
        if value['ram']['errors']:
            value['passed'] = False
    return value


def read_report(path):
    try:
        path = Path(path)
        if path.stat().st_size > 65536:
            raise ValueError('验收摘要超过容量')
        value = json.loads(path.read_text(encoding='utf-8'))
        if (value['schema'] != 1 or value['domain'] != 'hardware_acceptance'
                or value['live'] is not False or type(value['passed']) is not bool):
            raise ValueError('验收摘要格式无效')
        # 历史记录无论来源字段怎样填写，都不能提升实时状态或动作权限。
        return dict(value, available=True, live=False, connected=False, authorizes_motion=False)
    except (OSError, ValueError, TypeError, KeyError) as exc:
        return dict(available=False, live=False, connected=False, authorizes_motion=False,
                    reason='没有可用的实车验收记录：' + str(exc))
