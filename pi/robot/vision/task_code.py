"""四组任务码解析复用旧qr_camera/task_codec.py；锁存补齐新帧与时效。"""

import math
import re

COLORS = {1: '红色', 2: '黄色', 3: '蓝色', 4: '绿色', 5: '黑色', 6: '浅蓝'}
TASK_PATTERN = re.compile(r'([1-6]{3})\+([1-3]{3})\+([1-6]{3})\+([1-3]{3})', re.ASCII)


def parse_task(raw):
    """第二组定义首批暂存；第四组不覆盖第二批同色码垛位置。"""
    if not isinstance(raw, str) or len(raw) > 64:
        raise ValueError('任务码必须是长度不超过64的文本')
    code = raw.strip()
    match = TASK_PATTERN.fullmatch(code)
    if match is None:
        raise ValueError('任务码格式须为CCC+PPP+CCC+PPP')
    first_colors, first_positions, second_colors, second_positions = match.groups()
    if len(set(first_colors)) != 3 or len(set(second_colors)) != 3:
        raise ValueError('每批必须包含三种不同颜色')
    if set(first_colors) != set(second_colors):
        raise ValueError('两批颜色集合必须相同')
    if set(first_positions) != set('123') or set(second_positions) != set('123'):
        raise ValueError('位置组必须是123的排列')
    storage_by_color = dict(zip(first_colors, first_positions))

    def batch(colors, positions, first):
        items = []
        for order, (color, position) in enumerate(zip(colors, positions), start=1):
            items.append(dict(order=order, color_id=int(color), color_name=COLORS[int(color)],
                              rough_processing_ring=int(position),
                              temporary_storage_ring=int(position if first else storage_by_color[color])))
        return items

    return dict(valid=True, raw=code, batch1=batch(first_colors, first_positions, True),
                batch2=batch(second_colors, second_positions, False))


class TaskLatch:
    """三个不同新帧且跨越100毫秒后锁存；锁存后不随普通识别变化。"""

    def __init__(self, required=3, max_gap=1.5, max_age=.25, min_span=.1):
        if required < 3 or max_gap <= 0 or max_age <= 0 or min_span < .1:
            raise ValueError('任务码确认参数不能弱于三个新帧/100毫秒')
        self.required, self.max_gap = required, max_gap
        self.max_age, self.min_span = max_age, min_span
        self.reset()

    def reset(self):
        self.accepted = self.pending = None
        self.count = 0
        self.last_frame = -1
        self.first_seen = self.last_seen = None
        self.stream = None

    def observe(self, codes, now, *, frame_id, captured_s, stream):
        if self.accepted is not None:
            return 'ACCEPTED', None
        if (not all(isinstance(value, (int, float)) and math.isfinite(value)
                    for value in (now, captured_s)) or not 0 <= now - captured_s <= self.max_age):
            return 'STALE', None
        if type(frame_id) is not int or frame_id < 0 or not isinstance(stream, str) or not stream:
            return 'INVALID_FRAME', None
        if self.stream is None:
            self.stream = stream
        if stream != self.stream or frame_id <= self.last_frame:
            return 'STALE', None
        if self.last_seen is not None and captured_s <= self.last_seen:
            return 'STALE', None
        self.last_frame = frame_id
        if not isinstance(codes, (list, tuple)) or len(codes) > 16:
            raise ValueError('单帧任务码集合格式错误')
        codes = list(dict.fromkeys(code for code in codes if isinstance(code, str) and code))
        if len(codes) != 1:
            self.pending, self.count = None, 0
            return ('AMBIGUOUS' if codes else 'SEARCHING'), None
        try:
            task = parse_task(codes[0])
        except ValueError:
            self.pending, self.count = None, 0
            return 'INVALID_CODE', None
        if (task['raw'] != self.pending or self.last_seen is None or
                captured_s - self.last_seen > self.max_gap or
                (self.first_seen is not None and now - self.first_seen > self.max_age)):
            self.pending, self.count = task['raw'], 0
            self.first_seen = captured_s
        self.count += 1
        self.last_seen = captured_s
        if self.count >= self.required and captured_s - self.first_seen >= self.min_span - 1e-9:
            self.accepted = task
            return 'ACCEPTED', task
        return 'CONFIRMING', None
