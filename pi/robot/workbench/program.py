"""保存工位姿态流程并编译正式格式候选载荷；联调只复用机构模型。"""

import copy
import ctypes as c
import json
import math
import uuid

from .contracts import MechGoal
from .storage import write_json, payload
from ..device.mechanism import pose_body
from ..device.tracking import Spec
from ..nav.frames import Pose, angle
from ..nav.reference import NativeReference
from ..nav.bridge import prepare
from ..nav.route import digest


class Program:
    def __init__(self, mechanism, library):
        self.mech, self.library = mechanism, library
        self.folder = mechanism.runtime/'mechanism_programs'
        self.folder.mkdir(exist_ok=True)
        self.records, self.arrival_cache = [], {}
        self.active, self.child_id, self.index = None, None, 0
        self.mode, self.reason, self.export = 'IDLE','只保存和联调离线流程',None
        self.error = ''
        for path in sorted(self.folder.glob('*.json'),key=lambda p:p.stat().st_mtime,reverse=True)[:30]:
            try:
                value = json.loads(path.read_text(encoding='utf-8'))
                self._validate(value)
                self.records.append(value)
            except (OSError,ValueError,KeyError,TypeError) as exc:
                self.error = '存在未加载的无效流程：'+str(exc)
        try:
            previous = json.loads((self.folder/'last.state').read_text(encoding='utf-8'))
            if previous['mode'] == 'RUNNING':
                self.mode,self.reason = 'INTERRUPTED','核心重启，不恢复基准或续跑流程'
        except (OSError,ValueError,KeyError):
            pass

    @property
    def busy(self):
        return self.mode == 'RUNNING'

    def _validate(self, record):
        if (record.get('schema') != 1 or record.get('domain') != 'simulation' or
                record.get('physical_validated') is not False or record.get('sha256') != digest(record) or
                record['config_hash'] != self.mech.config_hash or not 1 <= len(record['steps']) <= 32):
            raise ValueError('流程版本、配置或校验不匹配')
        for step in record['steps']:
            pose = step['pose']
            self.mech._targets(pose['targets'])
            if pose['sha256'] != self.mech._hash(pose['targets']) or pose['config_hash'] != self.mech.config_hash:
                raise ValueError('流程内姿态校验失败')

    def arrivals(self, plan):
        if plan is None:
            return []
        if plan.view['id'] not in self.arrival_cache:
            current = Pose(*plan.view['origin'])
            native = c.CDLL(str(self.library))
            result = []
            for index,raw in enumerate(plan.specs,1):
                current = NativeReference(native,current,Spec.from_buffer_copy(raw)).end
                for station,label in [('qr','扫码'),('raw','原料'),('rough','粗加工'),('staging','暂存')]:
                    expected = plan.view['field']['nodes'][station]
                    heading = plan.view['field']['station_heading_quarters'][station]*math.pi/2
                    if math.dist(current.values()[:2],expected) <= .003 and abs(angle(current.yaw_rad-heading)) <= .01:
                        result.append(dict(after_segment=index,station=station,label=f'段{index}到位后 · {label}'))
            self.arrival_cache = {plan.view['id']:result}
        return copy.deepcopy(self.arrival_cache[plan.view['id']])

    def selected(self, args):
        record = next((value for value in self.records if value['id']==args.id and value['revision']==args.revision),None)
        if record is None:
            raise ValueError('流程版本不存在')
        self._validate(record)
        for step in record['steps']:
            pose = step['pose']
            current = next((p for p in self.mech.pose_versions if p['id']==pose['id'] and p['revision']==pose['revision']),None)
            if current is None or current['sha256'] != pose['sha256']:
                raise ValueError('原姿态版本不可读或已变化，不能沿用旧流程')
        return record

    def save(self, args, plan):
        if plan is None or args.plan_id != plan.view['id'] or not args.steps:
            raise ValueError('先生成路线，并至少加入一个已保存姿态')
        previous = max((r['revision'] for r in self.records if r['id']==args.id),default=0)
        if args.revision != previous:
            raise ValueError('流程版本已更新，先读取最新记录')
        arrivals = {item['after_segment']:item for item in self.arrivals(plan)}
        steps, last = [], 0
        for item in args.steps:
            if item.after_segment not in arrivals or item.after_segment < last:
                raise ValueError('步骤只能插在已到位工位，且导航段顺序不能倒退')
            pose = next((p for p in self.mech.pose_versions if p['id']==item.pose_id and p['revision']==item.revision),None)
            if pose is None:
                raise ValueError('先保存要加入流程的姿态版本')
            snapshot = {key:copy.deepcopy(pose[key]) for key in ('id','name','revision','targets','sha256','config_hash')}
            steps.append(dict(after_segment=item.after_segment,station=arrivals[item.after_segment]['station'],
                              pose=snapshot,release_confirmed=item.release_confirmed))
            last = item.after_segment
        record = dict(schema=1,id=args.id,name=args.name,revision=previous+1,domain='simulation',
            physical_validated=False,authorizes_motion=False,config_hash=self.mech.config_hash,
            plan_id=plan.view['id'],plan_artifact_sha256=payload(plan)['artifact_sha256'],steps=steps)
        record['sha256'] = digest(record)
        self._validate(record)
        path = self.folder/f'{args.id}-v{previous+1}.json'
        if path.exists():
            raise ValueError('该版本文件已存在，禁止覆盖')
        write_json(path,record)
        self.records.insert(0,record)
        self.records = self.records[:30]
        return record

    def shadow(self, canonical=False):
        shadow = copy.copy(self.mech)
        shadow.positions = dict(self.mech.positions)
        shadow.results, shadow.queue, shadow.active = [], [], None
        if canonical:
            shadow.positions = dict(lift=0.,reach=0.,**{key:math.radians(value['anchor_deg']) for key,value in shadow.cfg['servos'].items()})
            shadow.zero_valid, shadow.zero_version, shadow.hold_possible = True,1,False
            shadow.fault = ''
        return shadow

    @staticmethod
    def advance(shadow, step):
        pose = step['pose']
        transitions = shadow._pose_steps(pose['targets'])
        shadow._start(transitions,'pose',step['release_confirmed'])
        shadow.positions = dict(pose['targets'])
        shadow._finish('COMPLETE','仅编译/预检中的模型终态')
        return transitions

    def compile(self, record, plan):
        if (plan is None or plan.view['id'] != record['plan_id'] or
                payload(plan)['artifact_sha256'] != record['plan_artifact_sha256']):
            raise ValueError('路线版本已变化；请明确重新绑定工位并保存流程新版本')
        navigation = prepare(plan,self.library)
        shadow, timeline, operation = self.shadow(canonical=True),[],0
        for index,goal in enumerate(navigation['goals'],1):
            if not shadow.transport_ready():
                raise ValueError(f'进入导航段{index}前必须有收拢姿态：升降/伸缩回0、转臂180°')
            timeline.append(dict(kind='navigation',command=34,payload_hex=goal,
                                 guards=['mechanism_transport','material_transport_confirmed']))
            for step in [v for v in record['steps'] if v['after_segment']==index]:
                operation += 1
                transitions = self.advance(shadow,step)
                pose = step['pose']
                body = pose_body(operation,pose['revision'],pose['targets'],True,step['release_confirmed'])
                timeline.append(dict(kind='mechanism',command=55,station=step['station'],
                    after_segment=index,pose_id=pose['id'],revision=pose['revision'],pose_sha256=pose['sha256'],
                    payload_template_hex=(b'\0'*8+body).hex(),safe_transitions=transitions,
                    runtime_fields=dict(token=0,zero_version=4,action_id=8),
                    guards=['base_stopped','station_pose_valid','fresh_feedback','valid_zero','no_fault','physical_taught','servo_confirmed']))
        if not shadow.transport_ready():
            raise ValueError('流程结束前需加入收拢姿态，不自动回零或收臂')
        artifact = dict(schema=1,id=uuid.uuid4().hex,kind='mission_candidate_v1',domain='simulation',
            executable=False,physical_validated=False,authorizes_motion=False,
            program_id=record['id'],program_revision=record['revision'],program_sha256=record['sha256'],
            plan_id=plan.view['id'],navigation=navigation,timeline=timeline,
            initial_model='最高/最近端，转臂180°、料盘30°、夹爪0°，非实物状态',
            runtime_binding_required=True,physical_success_counting=False)
        artifact['sha256'] = digest(artifact)
        folder = self.mech.runtime/'mission_candidates'
        folder.mkdir(exist_ok=True)
        path = folder/f'{artifact["id"]}.json'
        write_json(path,artifact)
        self.export = dict(id=artifact['id'],path=str(path.resolve()),sha256=artifact['sha256'],
                           navigation_steps=len(navigation['goals']),mechanism_steps=operation,
                           domain='simulation',executable=False)
        return self.export

    def run(self, record):
        self.mech._ready()
        if any(value is None for value in self.mech.positions.values()):
            raise ValueError('先锚定模拟舵机估计')
        shadow = self.shadow()
        for step in record['steps']:
            self.advance(shadow,step)
        self.active = copy.deepcopy(record)
        self.index,self.child_id = 0,None
        self.zero_version = self.mech.zero_version
        self.mode,self.reason = 'RUNNING','逐姿态联调，不移动底盘，不增加真实计数'
        self._save_state()

    def cancel(self, reason):
        if self.busy:
            if self.child_id is not None:
                self.mech.stop(reason)
            self.mode,self.reason = 'CANCELLED',reason+'；部分完成不重放'
            self._save_state()

    def tick(self, base_stopped):
        if not self.busy:
            return
        if (not base_stopped or self.mech.fault or not self.mech.zero_valid or self.mech.zero_version != self.zero_version):
            self.cancel('底盘/机构故障或基准失效')
            return
        if self.child_id is not None:
            if self.mech.busy:
                return
            result = next((r for r in self.mech.results if r['action_id']==self.child_id),None)
            if result is None or result['status'] != 'COMPLETE':
                self.cancel('机构动作未完成')
                return
            self.index += 1
            self.child_id = None
        if self.index == len(self.active['steps']):
            self.mode,self.reason = 'COMPLETE','各姿态模型完成；不是整图或实物验收'
            self._save_state()
            return
        step = self.active['steps'][self.index]
        pose = step['pose']
        try:
            result = self.mech.command(MechGoal(action='pose',pose_id=pose['id'],revision=pose['revision'],
                zero_version=self.zero_version,release_confirmed=step['release_confirmed']))
            self.child_id = result['action_id']
        except ValueError as exc:
            self.cancel(str(exc))

    def _save_state(self):
        write_json(self.folder/'last.state',dict(mode=self.mode,index=self.index,reason=self.reason))

    def view(self, plan=None):
        return dict(domain='simulation',busy=self.busy,mode=self.mode,reason=self.reason,index=self.index,
            total=len(self.active['steps']) if self.active else 0,error=self.error,export=self.export,
            authorizes_motion=False,arrivals=self.arrivals(plan),
            versions=[{key:record[key] for key in ('id','name','revision','plan_id','sha256')} |
                      dict(step_count=len(record['steps'])) for record in self.records])
