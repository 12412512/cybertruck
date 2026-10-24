"""首次CAN错误分页的纯解析；没有串口、清故障或执行权限。"""
import struct

CAN_CAPTURE = 38
HEADER = struct.Struct('<8I')
FIRST = ('tick_ms', 'hal_error', 'tsr', 'esr', 'tx_sequence', 'rx_sequence',
         'tx_id', 'tx_length', 'data0', 'data1', 'state', 'mailbox',
         'tx_pending', 'tx_index', 'tx_count', 'error_count')
EVENT = ('sequence', 'tick_ms', 'kind', 'id', 'length', 'data0', 'data1',
         'tx_sequence', 'tsr', 'esr')
EVENT_NAMES = ('', 'TX', 'RX', 'TXOK', 'ALST', 'ERROR', 'CANCEL', 'TIMEOUT')


def decode_page(payload, expected_page):
    if len(payload) < HEADER.size:
        raise ValueError('CAN记录页长度不足')
    version, valid, generation, count, capacity, page, entry_size, reserved = HEADER.unpack_from(payload)
    if (version != 1 or valid not in (0, 1) or capacity != 64 or count > capacity
            or entry_size != 40 or reserved or page != expected_page or page > count
            or (not valid and (page or count))):
        raise ValueError('CAN记录页头无效')
    size = 32 if not valid else 96 if not page else 72
    if len(payload) != size:
        raise ValueError('CAN记录页长度不符')
    result = dict(version=version, valid=bool(valid), generation=generation, count=count, page=page)
    if valid:
        names = FIRST if not page else EVENT
        result['record'] = dict(zip(names, struct.unpack_from(f'<{len(names)}I', payload, 32)))
        if page:
            event = result['record']
            if (event['sequence'] != (generation - count + page) & 0xffffffff
                    or event['kind'] not in range(1, 8) or event['length'] > 8):
                raise ValueError('CAN历史事件无效或顺序不连续')
            event['kind_name'] = EVENT_NAMES[event['kind']]
    return result


class CaptureReader:
    """按页收集同一boot/session的冻结记录；完整前不对外冒充完整现场。"""
    def __init__(self):
        self.pages = []
        self.result = None
        self.header = None

    @property
    def next_page(self):
        return len(self.pages)

    def feed(self, payload, boot, session):
        page = decode_page(payload, self.next_page)
        if not page['page']:
            self.header = {key: page[key] for key in ('version', 'valid', 'generation', 'count')}
        if not page['valid']:
            self.pages.clear()
            return
        if self.pages and any(page[k] != self.pages[0][k] for k in ('generation', 'count')):
            raise ValueError('CAN分页现场发生变化')
        identity = (boot, session)
        if self.pages and self.identity != identity:
            raise ValueError('CAN分页启动或会话改变')
        self.identity = identity
        self.pages.append(page)
        if page['page'] == page['count']:
            first = self.pages[0]['record']
            history = [p['record'] for p in self.pages[1:]]
            if not history or history[-1]['kind_name'] != 'ERROR' or history[-1]['tick_ms'] != first['tick_ms']:
                raise ValueError('CAN首次错误与历史末帧不一致')
            self.result = dict(version=1, valid=True, complete=True, boot_id=f'{boot:016x}',
                               session_id=f'{session:016x}', generation=page['generation'],
                               first=first, history=history)
