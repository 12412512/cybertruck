"""单件转存离线编排：复用唯一Mechanism，不导入设备或真实任务账本。"""
import copy
import hashlib
import json
import math
from pathlib import Path
from .contracts import MechGoal
from .storage import write_json


class Transfer:
    def __init__(self, mechanism):
        self.mech = mechanism
        self.path = mechanism.runtime / 'transfer_last.json'
        self.id = 0
        self.mode, self.reason = 'IDLE', '仅离线；先预览再显式运行'
        self.index, self.child_id = 0, None
        self.steps, self.events = [], []
        self.ledger = dict(source='UNSET', gripper='UNSET', slots=['UNSET']*3, destination='UNSET')
        self.frozen = None
        self.started = None
        try:
            previous = json.loads(self.path.read_text(encoding='utf-8'))
            self.id = previous['id']
            if previous['mode'] not in ('IDLE', 'COMPLETE'):
                self.mode, self.reason = 'INTERRUPTED', '上次流程未闭合；不续跑，先重置离线夹具'
                self.ledger = dict(source='UNKNOWN',gripper='UNKNOWN',slots=['UNKNOWN']*3,destination='UNKNOWN')
        except FileNotFoundError:
            pass
        except (ValueError,KeyError,TypeError):
            self.mode,self.reason='INTERRUPTED','离线流程记录无效，不自动恢复'

    @property
    def busy(self):
        return self.mode in ('RUNNING','WAIT_OBSERVATION')

    def preview(self, args):
        self.mech._ready()
        if self.mech.busy or any(v is None for v in self.mech.positions.values()):
            raise ValueError('先等待机构模型空闲并锚定模拟舵机')
        shadow=copy.copy(self.mech)
        shadow.positions=dict(self.mech.positions)
        steps=[]
        seconds=0.
        # 三料位角度从用户确认的安装记录读取；不按等分猜测。
        angles=json.loads((Path(__file__).resolve().parents[3]/'config/mechanism_installation.json').read_text(encoding='utf-8'))['tray_slot_deg']
        tray=math.radians(angles[args.slot])
        def move(axis,target,label):
            nonlocal seconds
            shadow._check(axis,target,shadow.positions)
            if abs(shadow.positions[axis]-target)>1e-9:
                steps.append(dict(kind='move',axis=axis,target=target,label=label))
                distance=abs(shadow.positions[axis]-target)
                if axis in ('lift','reach'):
                    item=self.mech.cfg['axes'][axis]
                    seconds+=max(.1,distance*1.875/item['speed_m_s'],math.sqrt(distance*5.774/item['accel_m_s2']))+.1+.04
                else:
                    seconds+=max(.1,distance*1.875/math.radians(self.mech.cfg['servos'][axis]['rate_deg_s']))+self.mech.cfg['settle_s']+.08
            shadow.positions[axis]=target
        def pose(lift,reach,arm,label):
            target=dict(shadow.positions,lift=lift,reach=reach,arm=math.radians(arm),tray=tray)
            for axis,value in shadow._pose_steps(target):
                move(axis,value,label)
        def observe(expected,label):
            steps.append(dict(kind='observe',expected=expected,label=label))
        move('gripper',math.radians(60),'取料前张爪（仅模型）')
        pose(args.raw_lift_m,args.raw_reach_m,180,'接近原料候选姿态')
        move('gripper',math.radians(98),'取料夹持')
        observe('PICKED','确认模型已持料')
        pose(args.slot_lift_m,0.,83,'撤离、旋转、到车载料位')
        move('gripper',math.radians(60),'释放到车载料位')
        observe('STORED','确认模型已存车')
        pose(0.,0.,180,'存车后收拢观察')
        pose(args.slot_lift_m,0.,83,'到同一车载料位取出')
        move('gripper',math.radians(98),'夹持车载物料')
        observe('RETRIEVED','确认模型已取出')
        pose(args.place_lift_m,args.place_reach_m,180,'到放置候选姿态（高度待测）')
        move('gripper',math.radians(60),'释放到目标位置')
        observe('PLACED','确认模型已放置')
        pose(0.,0.,180,'放置后收拢')
        value=dict(domain='simulation',physical_validated=False,configuration=args.model_dump(),steps=steps,
                   model_seconds=round(seconds,3),deadline_s=math.ceil(seconds*1.3+120),
                   config_hash=self.mech.config_hash,zero_version=self.mech.zero_version,
                   initial=dict(self.mech.positions),slot_angle_deg=angles[args.slot])
        value['configuration']['action']='run'
        for key in ('action_id','step_index','verdict','confirmed'):
            value['configuration'].pop(key,None)
        value['sha256']=hashlib.sha256(json.dumps(value,sort_keys=True).encode()).hexdigest()
        return value

    def command(self,args):
        if args.action=='stop':
            self.cancel('操作员停止离线转存流程')
            return self.view()
        if args.action=='reset':
            if not args.confirmed or self.busy or self.mech.busy or self.mech.hold_possible:
                raise ValueError('先停止并处理模型可能持料；重置夹具需明确确认，不会移动机构')
            self.mode,self.reason='IDLE','离线物料夹具已显式重置；未重建机构零点'
            self.ledger=dict(source='UNSET',gripper='UNSET',slots=['UNSET']*3,destination='UNSET')
            self._save()
            return self.view()
        if args.action=='confirm':
            if (self.mode!='WAIT_OBSERVATION' or args.action_id!=self.id or args.step_index!=self.index):
                raise ValueError('确认不属于当前流程和观察步骤')
            if args.verdict=='UNKNOWN':
                self.cancel('模型观察未知，不重复抓放',failed=True)
            elif args.verdict!=self.steps[self.index]['expected']:
                raise ValueError('观察类型与当前步骤不匹配')
            else:
                self._observe(args.verdict,'operator_model')
            return self.view()
        if self.busy:
            raise ValueError('离线转存运行中，先停止再修改')
        plan=self.preview(args)
        if args.action=='preview':
            return plan
        if not args.confirmed or self.mode not in ('IDLE','COMPLETE') or self.mech.hold_possible:
            raise ValueError('需确认只使用合成物料；中断/可能持料状态不能直接重跑')
        if any(abs(self.mech.positions[a])>.001 for a in ('lift','reach')):
            raise ValueError('流程起点须先回到模型收拢位置，禁止假清零')
        self.id+=1
        self.frozen=plan
        self.steps=copy.deepcopy(plan['steps'])
        self.index,self.child_id=0,None
        self.mode,self.reason='RUNNING','离线单件闭环已接受；全部物料观测仅为模型'
        self.started=self.mech.clock()
        self.ledger=dict(source=args.color,gripper='EMPTY',slots=['EMPTY']*3,destination='EMPTY')
        self.events=[]
        self._save()
        return self.view()

    def _observe(self,expected,source):
        slot=self.frozen['configuration']['slot']
        color=self.frozen['configuration']['color']
        if expected=='PICKED': self.ledger.update(source='EMPTY',gripper=color)
        elif expected=='STORED': self.ledger['slots'][slot]=color; self.ledger['gripper']='EMPTY'
        elif expected=='RETRIEVED': self.ledger['slots'][slot]='EMPTY';self.ledger['gripper']=color
        elif expected=='PLACED': self.ledger.update(gripper='EMPTY',destination=color)
        self.events.append(dict(step=self.index,event=expected,evidence=source,domain='simulation'))
        self.index+=1
        self.mode='RUNNING'
        self._save()

    def cancel(self,reason,failed=False):
        if not self.busy:return
        self.mech.stop(reason,failed=failed)
        self.mode,self.reason=('FAILED' if failed else 'CANCELLED'),reason
        self.ledger.update(source='UNKNOWN',gripper='UNKNOWN',destination='UNKNOWN')
        self.ledger['slots'][self.frozen['configuration']['slot']]='UNKNOWN'
        self.child_id=None
        self._save()

    def tick(self,base_stopped=True):
        if not self.busy:return
        if self.started is None or self.mech.clock()-self.started>self.frozen['deadline_s']:
            self.cancel('离线转存超过模型动作及观察等待期限',failed=True)
            return
        if (not base_stopped or not self.mech.zero_valid or self.mech.fault or
                self.mech.zero_version!=self.frozen['zero_version'] or self.mech.config_hash!=self.frozen['config_hash']):
            self.cancel('底盘/零点/配置/机构故障变化，停止离线转存',failed=True)
            return
        if self.child_id is not None:
            if self.mech.active:
                if self.mech.active['id']!=self.child_id:self.cancel('子动作身份变化',failed=True)
                return
            result=next((r for r in self.mech.results if r['action_id']==self.child_id),None)
            if not result or result['status']!='COMPLETE' or result['domain']!='simulation':
                self.cancel('子动作未按模型确认完成',failed=True)
                return
            self.child_id=None
            self.index+=1
            self._save()
        if self.index>=len(self.steps):
            self.mode,self.reason='COMPLETE','取料、存车、取出、放置及收拢模型闭环完成；不是实物验收'
            self._save()
            return
        step=self.steps[self.index]
        if step['kind']=='observe':
            self.mode='WAIT_OBSERVATION'
            if self.frozen['configuration']['auto_model']:
                self._observe(step['expected'],'synthetic_fixture')
            return
        try:
            result=self.mech.command(MechGoal(action='move',axis=step['axis'],zero_version=self.mech.zero_version,
                release_confirmed=True,**{'target_m' if step['axis'] in ('lift','reach') else 'target_rad':step['target']}))
            self.child_id=result['action_id']
            self._save()
        except ValueError as exc:
            self.cancel(str(exc),failed=True)

    def view(self):
        return copy.deepcopy(dict(id=self.id,mode=self.mode,reason=self.reason,index=self.index,total=len(self.steps),
            current=self.steps[self.index] if self.index<len(self.steps) else None,child_id=self.child_id,
            ledger=self.ledger,events=self.events,plan=self.frozen,domain='simulation',physical_validated=False))

    def _save(self):
        write_json(self.path,self.view())
