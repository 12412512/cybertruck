"""复用旧工程CYQM v6纯协议；坐标毫米，无ROS、串口或设备访问。"""
import binascii
import struct
from collections import deque

EVENTS = {0x81, 0x82, 0x83, 0x84, 0x85, 0x86, 0x87,
          0x88, 0x89, 0x8A, 0x8D, 0x8E, 0x8F}
# 沿用旧白名单，不增加未验证的扫描/规划命令或标定写入。
COMMANDS = {1, 2, 3, 4, 5, 0x13, 0x19}


def cobs_encode(data):
    result = bytearray([0])
    pos, code = 0, 1
    for value in data:
        if value == 0:
            result[pos] = code
            pos, code = len(result), 1
            result.append(0)
        else:
            result.append(value)
            code += 1
            if code == 255:
                result[pos] = code
                pos, code = len(result), 1
                result.append(0)
    result[pos] = code
    return bytes(result)


def cobs_decode(data):
    result, pos = bytearray(), 0
    while pos < len(data):
        code = data[pos]
        pos += 1
        if not code or pos + code - 1 > len(data):
            raise ValueError('invalid COBS')
        result.extend(data[pos:pos + code - 1])
        pos += code - 1
        if code != 255 and pos < len(data):
            result.append(0)
    return bytes(result)


def frame(node, command, sequence, payload=b''):
    if command not in COMMANDS:
        raise ValueError('unsupported command/node/payload')
    return _frame(node, command, sequence, payload)


def calibration_frame(node, sequence, zone):
    """仅受控校准事务使用；不扩展默认frame白名单，也不代表获得写入许可。"""
    if type(zone) is not int or zone not in (1, 2):
        raise ValueError('calibration zone')
    return _frame(node, 9, sequence, bytes([zone]))


def _frame(node, command, sequence, payload):
    if not 1 <= node <= 254 or not 0 <= sequence <= 65535 or len(payload) > 224:
        raise ValueError('unsupported command/node/payload')
    raw = struct.pack('<2sBBBHH', b'LR', 6, node, command, sequence, len(payload)) + payload
    return cobs_encode(raw + struct.pack('<H', binascii.crc_hqx(raw, 0xFFFF))) + b'\0'


class Decoder:
    def __init__(self):
        self.partial = bytearray()
        self.discarding = False
        self.good = self.bad = self.unknown = 0
        self.last_unknown = None
        self.unknown_frames = deque(maxlen=16)

    def feed(self, data):
        events = []
        for value in data:
            if value:
                if not self.discarding:
                    self.partial.append(value)
                    if len(self.partial) > 236:
                        self.partial.clear()
                        self.discarding = True
                        self.bad += 1
                continue
            if self.discarding:
                self.discarding = False
                continue
            segment = bytes(self.partial)
            self.partial.clear()
            if not segment:
                continue
            try:
                raw = cobs_decode(segment)
                if not 11 <= len(raw) <= 235:
                    raise ValueError('frame size')
                magic, version, node, typ, seq, count = struct.unpack('<2sBBBHH', raw[:9])
                if magic != b'LR' or version != 6 or not 1 <= node <= 254:
                    raise ValueError('header')
                if count > 224 or len(raw) != 11 + count:
                    raise ValueError('payload size')
                if binascii.crc_hqx(raw[:-2], 0xFFFF) != int.from_bytes(raw[-2:], 'little'):
                    raise ValueError('CRC')
                self.good += 1
                if typ not in EVENTS:
                    self.unknown += 1
                    # 仅保留有界诊断，不将扩展事件接纳为扫描或标定结果。
                    self.last_unknown = dict(node=node, type=typ, sequence=seq, payload_hex=raw[9:-2].hex())
                    self.unknown_frames.append(self.last_unknown)
                    continue
                events.append((node, typ, seq, raw[9:-2]))
            except ValueError:
                self.bad += 1
        return events


def hello(payload):
    if len(payload) != 13:
        raise ValueError('HELLO size')
    major, minor, protocol, zone, x, y, heading, running, capacity = struct.unpack('<BBBBhhHBH', payload)
    if protocol != 6 or zone not in (0, 1, 2) or running not in (0, 1) or heading >= 23040:
        raise ValueError('HELLO fields')
    return dict(firmware=f'{major}.{minor}', protocol=protocol, zone=zone,
                x_mm=x, y_mm=y, heading_deg=heading / 64,
                running=running, max_scan_points=capacity)


def persisted(payload):
    if len(payload) < 33 or payload[9] not in (0, 1):
        raise ValueError('PERSISTED size/validity')
    count = payload[10]
    ids = list(payload[33:])
    if count > 25 or len(payload) != 33 + count or len(set(ids)) != count or any(v > 24 for v in ids):
        raise ValueError('PERSISTED grids')
    def pose(offset, zone):
        # 厂家C客户端忽略第七存储字节；它不是区号。
        x, y, heading = struct.unpack_from('<hhH', payload, offset)
        return dict(x_mm=x, y_mm=y, heading_deg=heading / 64, zone=zone)
    return dict(storage_version=struct.unpack_from('<H', payload)[0],
                record_sequence=struct.unpack_from('<I', payload, 2)[0],
                source_zone=payload[6], grid_result_valid=bool(payload[9]),
                grid_ids=ids, zone1_pose=pose(11, 1), zone2_pose=pose(18, 2))


def scan_status(payload):
    if len(payload) != 5:
        raise ValueError('STATUS size')
    status, mode, running, count, overflow = payload
    if running not in (0, 1) or overflow:
        raise ValueError('STATUS flags/overflow')
    return dict(status=status, mode=mode, running=running, frame_count=count)


def isolated_extensions(decoder, before, information, node):
    """仅隔离实测1.37的0x90附加参数，绝不解释为扫描或几何数据。"""
    count = decoder.unknown-before
    values = list(decoder.unknown_frames)[-count:] if count else []
    if count > 16 or any(item['node'] != node or item['type'] != 0x90 or
            len(item['payload_hex']) != 8 or not information or
            information['firmware'] != '1.37' for item in values):
        raise ValueError('未知协议事件：'+str(decoder.last_unknown))
    return values


def scan_grid(payload):
    if len(payload) < 5:
        raise ValueError('GRID size')
    sequence, count = struct.unpack_from('<IB', payload)
    ids = list(payload[5:])
    if count > 25 or len(ids) != count or any(v > 24 and v != 255 for v in ids):
        raise ValueError('GRID count/ids')
    return sequence, ids


def scan_result(payload, sequence, ids):
    if len(payload) < 8:
        raise ValueError('RESULT size')
    current, count, locked, overflow, number = struct.unpack_from('<IBBBB', payload)
    if (current != sequence or number > 25 or len(payload) != 8+number*8 or
            locked not in (0, 1) or overflow or not 1 <= count <= 14 or len(ids) != number):
        raise ValueError('RESULT sequence/count/overflow')
    objects = []
    for index, grid_id in enumerate(ids):
        x, y, hits, confidence, points = struct.unpack_from('<hhBBH', payload, 8+index*8)
        if hits > 14 or confidence > 100:
            raise ValueError('RESULT quality')
        objects.append(dict(x_mm=x, y_mm=y, frame_hits=hits, confidence_percent=confidence,
                            point_count=points, grid_id=grid_id))
    return dict(scan_sequence=current, frame_count=count, locked=locked, objects=objects)


class CloudAssembler:
    """只发布完整、一致且未溢出的抽样点云。"""
    def __init__(self):
        self.current = None
        self.complete = self.dropped = 0

    def discard(self):
        if self.current is not None:
            self.dropped += 1
        self.current = None

    def feed(self, typ, payload):
        try:
            if typ == 0x84:
                self.discard()
                if len(payload) != 14:
                    raise ValueError('SCAN_BEGIN size')
                seq, stamp, frequency, count, source = struct.unpack('<IIHHH', payload)
                if count > 120 or count > source:
                    raise ValueError('display cloud capacity')
                self.current = dict(scan_sequence=seq, device_timestamp_ms=stamp,
                                    frequency_hz=frequency / 10, source_point_count=source,
                                    points=[None] * count)
            elif typ == 0x85:
                if len(payload) < 7:
                    raise ValueError('SCAN_POINTS size')
                seq, offset, count = struct.unpack_from('<IHB', payload)
                if len(payload) != 7 + count * 5:
                    raise ValueError('SCAN_POINTS length')
                if self.current is None:
                    return None
                points = self.current['points']
                if seq != self.current['scan_sequence'] or offset + count > len(points):
                    raise ValueError('SCAN_POINTS range/sequence')
                for i in range(count):
                    point = struct.unpack_from('<hhB', payload, 7 + i * 5)
                    previous = points[offset + i]
                    if previous is not None and previous != point:
                        raise ValueError('conflicting duplicate point')
                    points[offset + i] = point
            elif typ == 0x86:
                if len(payload) != 9:
                    raise ValueError('SCAN_END size')
                seq, count, overflow, nearest = struct.unpack('<IHBH', payload)
                if self.current is None:
                    return None
                cloud = self.current
                if seq != cloud['scan_sequence'] or count != len(cloud['points']) or overflow:
                    raise ValueError('SCAN_END mismatch/overflow')
                if any(point is None for point in cloud['points']):
                    raise ValueError('missing point block')
                cloud['nearest_mm'] = nearest
                self.current = None
                self.complete += 1
                return cloud
        except (ValueError, struct.error):
            self.discard()
            raise
        return None
