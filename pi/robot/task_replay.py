"""唯一设备核心接口的离线合同演练；无传输实现，绝不打开设备或建立真实账本。"""

import copy
import hashlib
import math
import uuid

from .ledger import canonical
from .task import Task, reduce
from .task_check import require
from .vision.task_evidence import fixture_frames, observation_context, sha256_text, same_json


def digest(value):
    return hashlib.sha256(canonical(value).encode('utf-8')).hexdigest()


class OfflineMission:
    """消费显式夹具收据与证据；接口字段为提议，等待A接入唯一核心。"""

    def __init__(self, task, core, parameters):
        require(task.state['domain'] == 'simulation', '离线适配器只允许模拟账本')
        require(isinstance(core, dict) and set(core) == {'owner', 'boot_id', 'session_id'} and
                all(isinstance(v, str) and 0 < len(v) <= 128 for v in core.values()),
                '必须绑定唯一核心及启动/会话身份')
        require(isinstance(parameters, dict) and type(parameters.get('revision')) is int and
                parameters['revision'] > 0 and sha256_text(parameters.get('sha256')),
                '参数版本或摘要无效')
        self.task = task
        self.core = copy.deepcopy(core)
        self.parameters = copy.deepcopy(parameters)
        task.ledger.audit(task.run_id, reduce)

    @classmethod
    def create(cls, ledger, owner, clock, core, parameters, start_zone=1):
        # 先验证合同，再创建账本，避免无效输入留下活动轮次。
        require(isinstance(core, dict) and set(core) == {'owner', 'boot_id', 'session_id'} and
                all(isinstance(v, str) and 0 < len(v) <= 128 for v in core.values()), '核心身份无效')
        require(isinstance(parameters, dict) and type(parameters.get('revision')) is int and
                parameters['revision'] > 0 and sha256_text(parameters.get('sha256')), '参数版本无效')
        return cls(Task.create(ledger, owner, clock, start_zone), core, parameters)

    def _duplicate(self, event_id, field, value):
        for event in self.task.ledger.events(self.task.run_id):
            if event['event_id'] == event_id:
                require(same_json(event['payload'].get(field), value), '重复事件内容冲突')
                return True
        return False

    def request(self):
        """只返回待审合同，不发送；未决动作必须查询，不能重复生成动作。"""
        state = self.task.state
        require(state['status'] == 'RUNNING' and state['step'] < len(state['plan']), '任务不可发起新动作')
        require(state['pending'] is None, '已有未决动作；应查询，不得重发')
        action_id = uuid.uuid4().hex
        request = dict(schema_version=1, domain='offline_fixture', transport='none',
                       run_id=self.task.run_id, action_id=action_id, session_id=self.task.owner,
                       core=copy.deepcopy(self.core), parameters=copy.deepcopy(self.parameters),
                       goal=copy.deepcopy(state['plan'][state['step']]))
        request['request_sha256'] = digest(request)
        self.task.emit('INTENT', dict(action_id=action_id, replay_request=request), action_id + ':INTENT')
        return copy.deepcopy(request)

    def pending_request(self):
        action = self.task.state['pending']
        require(action is not None, '没有待查询动作')
        events = self.task.ledger.events(self.task.run_id)
        request = next((e['payload'].get('replay_request') for e in events if
                        e['kind'] == 'INTENT' and e['payload'].get('action_id') == action['id']), None)
        require(request is not None, '动作未经过离线合同适配器')
        require(same_json(request['core'], self.core) and same_json(request['parameters'], self.parameters),
                '核心会话或参数已变化；停止并对账')
        return copy.deepcopy(request)

    def receive(self, receipt):
        """接受/运行/完成/故障/未知分开；完成收据本身不增加成功计数。"""
        require(isinstance(receipt, dict), '收据格式错误')
        request = receipt.get('request')
        require(isinstance(request, dict) and request.get('run_id') == self.task.run_id and
                same_json(request.get('core'), self.core) and same_json(request.get('parameters'), self.parameters),
                '收据任务/核心/版本不匹配')
        status = receipt.get('status')
        require(status in ('ACCEPTED', 'RUNNING', 'COMPLETE', 'FAULT', 'UNKNOWN'), '收据状态无效')
        require(receipt.get('domain') == 'offline_fixture' and
                receipt.get('source_kind') == 'synthetic_execution', '仅接受合成执行收据')
        action_id = request.get('action_id')
        require(isinstance(action_id, str), '收据动作编号无效')
        event_id = action_id + ':CORE:' + status
        if self._duplicate(event_id, 'replay_receipt', receipt):
            return self.task.state
        require(same_json(request, self.pending_request()), '完整请求合同不匹配')
        stamp, now = receipt.get('observed_s'), self.task.clock()
        require(type(stamp) in (int, float) and math.isfinite(stamp) and
                self.task.state['pending']['started_s'] <= stamp <= now and now - stamp <= .25,
                '执行收据已过期或时间无效')
        payload = dict(action_id=action_id, replay_receipt=copy.deepcopy(receipt))
        if status in ('FAULT', 'UNKNOWN'):
            payload['reason'] = '离线执行' + status + '：' + str(receipt.get('reason', '结果未确定'))
            return self.task.emit('UNKNOWN' if status == 'UNKNOWN' else 'FAILED', payload, event_id)
        kind = {'ACCEPTED': 'ACCEPTED', 'RUNNING': 'RUNNING', 'COMPLETE': 'EXECUTED'}[status]
        if status == 'COMPLETE':
            require(isinstance(receipt.get('result'), dict), '完成收据缺少结果条件')
            result = copy.deepcopy(receipt['result'])
            result.update(domain='simulation', run_id=request['run_id'], session_id=request['session_id'],
                          action_id=action_id, observed_s=stamp)
            payload['result'] = result
        return self.task.emit(kind, payload, event_id)

    def pose(self, phase, context):
        request = self.pending_request()
        context = observation_context(context)
        event_id = request['action_id'] + ':POSE:' + phase
        if self._duplicate(event_id, 'replay_context', context):
            return self.task.state
        return self.task.emit('POSE', dict(action_id=request['action_id'], phase=phase,
                                           replay_context=context), event_id)

    def checkpoint(self, frames):
        request = self.pending_request()
        pose = self.task.state['pending']['pose']
        require(pose is not None, '缺少观察姿态')
        phase = frames[0].get('phase') if isinstance(frames, list) and frames and isinstance(frames[0], dict) else None
        require(isinstance(phase, str), '缺少帧阶段')
        event_id = request['action_id'] + ':CHECK:' + phase
        if self._duplicate(event_id, 'replay_frames', frames):
            return self.task.state
        context = next(e['payload']['replay_context'] for e in self.task.ledger.events(self.task.run_id)
                       if e['event_id'] == request['action_id'] + ':POSE:' + pose['phase'])
        mapped = fixture_frames(frames, request, pose, context)
        return self.task.emit('CHECK', dict(action_id=request['action_id'], frames=mapped,
                                            replay_frames=copy.deepcopy(frames)), event_id)

    def confirm(self):
        request = self.pending_request()
        result = self.task.state['pending'].get('result')
        require(result is not None and 0 <= self.task.clock() - result['observed_s'] <= .25,
                '完成结果缺失或已过期；不能仅凭新视觉帧确认')
        return self.task.action('CONFIRM', request['action_id'])

    def snapshot(self):
        return dict(domain='offline_rehearsal', simulation=self.task.snapshot(),
                    physical_counts=None, physical_acceptance=False, motion_authority=False,
                    transport='none', interface_status='PROPOSED_FOR_A_REVIEW')
