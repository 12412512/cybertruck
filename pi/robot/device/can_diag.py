"""只解析CARCAN13诊断快照，不查询设备、不清寄存器或软件故障。"""

LEC = ('无错误', '位填充错误', '格式错误', 'ACK应答错误',
       '隐性位错误', '显性位错误', 'CRC错误', '软件设定值')
DIAG_KEYS = ('esr', 'tsr', 'btr', 'error_esr', 'error_tsr', 'transport_state',
             'tx_pending', 'tx_isolated', 'tx_failed', 'tx_timeouts', 'arbitration_lost')


def unsigned(value):
    if type(value) is not int or not 0 <= value <= 0xffffffff:
        raise ValueError('诊断数值不是uint32')
    return value


def esr(value):
    value = unsigned(value)
    code = (value >> 4) & 7
    return dict(raw=value, warning=bool(value & 1), passive=bool(value & 2),
                bus_off=bool(value & 4), lec=code, lec_name=LEC[code],
                tec_low8=(value >> 16) & 255, rec=(value >> 24) & 255)


def diagnose(state):
    if not isinstance(state, dict) or state.get('firmware') != 'CARCAN13':
        raise ValueError('仅支持已核对CARCAN13诊断字段')
    board = state.get('board')
    words = state.get('can_diagnostics')
    if not isinstance(board, dict) or not isinstance(words, list) or len(words) != 11:
        raise ValueError('CAN诊断缺字段')
    values = [unsigned(int(value, 16 if index < 5 else 10))
              for index, value in enumerate(words)]
    current = dict(zip(DIAG_KEYS, values))
    current['decoded_esr'] = esr(current['esr'])
    btr = current['btr']
    current['bit_timing'] = dict(prescaler=(btr & 1023) + 1, segment1=((btr >> 16) & 15) + 1,
                                 segment2=((btr >> 20) & 7) + 1, sjw=((btr >> 24) & 3) + 1,
                                 loopback=bool(btr & (1 << 30)), silent=bool(btr & (1 << 31)))
    event = state.get('can_error_event')
    latched = None
    if isinstance(event, dict) and event.get('valid') is True:
        keys = ('sequence', 'tick_ms', 'hal_error', 'esr', 'tsr', 'tx_id', 'tx_length',
                'tx_data0', 'tx_data1', 'mailbox', 'transport_state', 'tx_pending', 'tx_index', 'tx_count')
        latched = {key: unsigned(event[key]) for key in keys}
        if latched['tx_length'] > 8:
            raise ValueError('冻结CAN帧长度无效')
        payload = (latched['tx_data0'].to_bytes(4, 'little') +
                   latched['tx_data1'].to_bytes(4, 'little'))[:latched['tx_length']]
        latched['payload_hex'] = payload.hex(' ').upper()
        latched['address'] = latched['tx_id'] >> 8
        latched['packet_index'] = latched['tx_id'] & 255
        latched['is_stop_frame'] = payload == bytes.fromhex('FE 98 00 6B') and latched['packet_index'] == 0
        latched['decoded_esr'] = esr(latched['esr'])
        latched['pure_arbitration'] = bool(latched['hal_error']) and not latched['hal_error'] & ~0xA800
        latched['mailbox0_transmit_error'] = bool(latched['hal_error'] & 0x1000)
        latched['mailbox0_tsr'] = dict(completed=bool(latched['tsr'] & 1),
                                      tx_ok=bool(latched['tsr'] & 2),
                                      arbitration_lost=bool(latched['tsr'] & 4),
                                      transmit_error=bool(latched['tsr'] & 8))
    bus_error = unsigned(board['bus_error'])
    return dict(current=current, latched=latched,
                software=dict(state=board['state'], reason=board['reason'], bus_error=bus_error,
                              stop_latched=bool(bus_error & 1), can_latched=bool(bus_error & 2),
                              rx_latched=bool(bus_error & 4), driver_latched=bool(bus_error & 8),
                              stop_observed_mask=board.get('stop_observed_mask'),
                              pending_stops=board.get('pending_stops')),
                conclusions=dict(current_esr_zero=current['esr'] == 0,
                                 latest_error_was_stop=bool(latched and latched['is_stop_frame']),
                                 fault_cleared=False, physical_cause_confirmed=False,
                                 note='当前寄存器、冻结错误、软件锁存分别判断；不能从地址3帧失败推断3号驱动器损坏'))


def compare(first, last):
    """同一观察窗口的前后比较，不将新上电窗口与历史累计计数相减。"""
    before, after = first['board'], last['board']
    uptime_delta = (unsigned(after['uptime_ms']) - unsigned(before['uptime_ms'])) & 0xffffffff
    if uptime_delta >= 0x80000000:
        raise ValueError('观察窗口可能跨复位或乱序')
    error_delta = (unsigned(after['can_errors']) - unsigned(before['can_errors'])) & 0xffffffff
    action_delta = (unsigned(after['action']) - unsigned(before['action'])) & 0xffffffff
    rx_delta = (unsigned(after['can_rx_sequence']) - unsigned(before['can_rx_sequence'])) & 0xffffffff
    if any(delta >= 0x80000000 for delta in (error_delta, action_delta, rx_delta)):
        raise ValueError('观察计数倒退，不能作增长结论')
    return dict(window_board_ms=uptime_delta, can_error_delta=error_delta,
                action_delta=action_delta, can_rx_delta=rx_delta,
                reset_identity_available=False)
