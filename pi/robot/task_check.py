"""迁用旧GrabCheck的新帧、姿态代次、同视图和分阶段确认约束。"""

import math

from .vision.task_code import TaskLatch


class TaskError(ValueError):
    pass


def require(condition, message):
    if not condition:
        raise TaskError(message)


def phases(goal, state):
    kind = goal['kind']
    if kind == 'QR':
        return [('QR', 'qr', 'code', None, None)]
    if kind not in ('PICK', 'PLACE'):
        return []
    item = state['items'][goal['item']]
    color, identity = item['color'], goal['item']
    if kind == 'PICK':
        return [('EMPTY', goal['destination'], 'empty', None, None),
                ('SOURCE', goal['source'], 'present', color, identity),
                ('HELD', 'gripper', 'held', color, identity),
                ('STORED', goal['destination'], 'stored', color, identity)]
    bottom = state['lower'].get(str(goal.get('ring')))
    stacked = goal.get('layer') == 2
    return [('TARGET', goal['destination'], 'occupied' if stacked else 'empty',
             color if stacked else None, bottom if stacked else None),
            ('HELD', 'gripper', 'held', color, identity),
            ('RELEASED', goal['destination'], 'released', color, identity),
            ('PLACED', goal['destination'], 'placed', color, identity)]


def check_frames(state, action, frames, at_s):
    rules = phases(action['goal'], state)
    phase_index = len(action['checks'])
    require(phase_index < len(rules), '当前动作无需更多视觉确认')
    phase, view, observed, color, item = rules[phase_index]
    pose = action.get('pose')
    require(pose is not None and pose['phase'] == phase, '缺少本阶段停车/姿态代次')
    require(isinstance(frames, list) and 3 <= len(frames) <= 16, '需要3～16个新帧')
    previous_id = state['last_frames'].get('qr' if phase == 'QR' else 'material', -1)
    previous_time = None
    camera = 'qr' if phase == 'QR' else 'material'
    latch = TaskLatch() if phase == 'QR' else None
    code = None
    for frame in frames:
        require(isinstance(frame, dict), '观测必须为结构化数据')
        context = (frame.get('domain'), frame.get('run_id'), frame.get('session_id'),
                   frame.get('action_id'), frame.get('pose_epoch'), frame.get('phase'))
        require(context == ('simulation', state['run_id'], state['owner'], action['id'],
                            pose['epoch'], phase), '观测任务/动作/会话/姿态不匹配')
        require(frame.get('camera') == camera and frame.get('view') == view and
                frame.get('calibration') == 'sim-v1', '相机、视图或标定版本不匹配')
        frame_id, captured, quality = frame.get('frame_id'), frame.get('captured_s'), frame.get('quality')
        require(type(frame_id) is int and frame_id > previous_id, '重复或乱序帧')
        require(type(captured) in (int, float) and math.isfinite(captured) and
                pose['settled_s'] <= captured <= at_s and at_s - captured <= .25,
                '旧帧、未来帧或动作前帧')
        require(previous_time is None or captured > previous_time, '采集时间未前进')
        require(type(quality) in (int, float) and math.isfinite(quality) and .7 <= quality <= 1,
                '观测质量不足')
        require(frame.get('state') == observed, '未知/错误物料状态不能确认成功')
        if phase == 'QR':
            _, accepted = latch.observe([frame.get('code')], at_s, frame_id=frame_id,
                                        captured_s=captured, stream='sim-qr')
            code = accepted or code
        else:
            require(frame.get('item') == item and frame.get('color') == color, '颜色或物料身份错误')
            if color is not None:
                require(type(frame.get('color')) is int, '颜色编号类型错误')
            if phase in ('RELEASED', 'PLACED', 'STORED'):
                require(frame.get('claw_empty') is True, '尚未确认夹爪与物料分离')
            if phase == 'PLACED':
                require(frame.get('placement_valid') is True and frame.get('stable') is True,
                        '放置位置或稳定性未确认')
        previous_id, previous_time = frame_id, captured
    require(frames[-1]['captured_s'] - frames[0]['captured_s'] >= .1 - 1e-9,
            '新帧稳定窗口不足100毫秒')
    if phase == 'QR':
        require(code is not None, '任务码未锁存一致结果')
    if phase == 'STORED':
        require(action['checks'][0]['view'] == view, '存车必须使用空槽确认时的视图')
    return dict(phase=phase, view=view, camera=camera, last_frame=previous_id,
                captured_s=previous_time, code=None if code is None else code['raw'])
