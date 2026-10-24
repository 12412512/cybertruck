"""独立物料模型驱动任务联调；不导入串口、相机或任何执行器。"""

import argparse
import json
from pathlib import Path
import uuid

from .ledger import Ledger
from .runtime import ProcessLock
from .task import Task, TERMINAL, phases, reduce
from .task_check import TaskError, require
from .vision.task_code import parse_task


class SimClock:
    def __init__(self):
        self.value = 0.0

    def __call__(self):
        return self.value

    def advance(self, seconds=.01):
        self.value += seconds
        return self.value


class World:
    """物料状态独立于账本；观测来自这里，不能由计数反推成功。"""

    def __init__(self, code):
        task = parse_task(code)
        self.code = task['raw']
        self.items = {f'b{batch}-{item["order"]}': dict(color=item['color_id'], location='raw')
                      for batch in (1, 2) for item in task[f'batch{batch}']}
        self.tray = {'1': None, '2': None, '3': None}
        self.rough, self.lower, self.upper = {}, {}, {}
        self.held = None
        self.frames = dict(qr=0, material=0)
        self.station = None

    def observe(self, goal, phase):
        identity, slot = goal.get('item'), str(goal.get('slot'))
        target = goal.get('destination')
        if phase == 'QR':
            return dict(state='code', code=self.code)
        if phase == 'EMPTY':
            found = self.tray[slot]
            return dict(state='empty' if found is None else 'occupied', item=found,
                        color=None if found is None else self.items[found]['color'])
        if phase == 'SOURCE':
            item = self.items[identity]
            return dict(state='present' if item['location'] == goal['source'] else 'unknown',
                        item=identity, color=item['color'])
        if phase == 'TARGET':
            ring = str(goal['ring'])
            if target.startswith('rough:'):
                found = self.rough.get(ring)
            else:
                found = self.upper.get(ring) or self.lower.get(ring)
            return dict(state='empty' if found is None else 'occupied', item=found,
                        color=None if found is None else self.items[found]['color'])
        if phase == 'HELD':
            require(self.held is None and self.items[identity]['location'] == goal['source'], '模拟物料不在抓取来源')
            if goal['kind'] == 'PLACE':
                require(self.tray[slot] == identity, '模拟料位身份不符')
                self.tray[slot] = None
            elif goal['source'].startswith('rough:'):
                require(self.rough.get(str(goal['ring'])) == identity, '模拟粗加工身份不符')
                del self.rough[str(goal['ring'])]
            self.held = identity
            self.items[identity]['location'] = 'held'
        elif phase == 'STORED':
            require(self.held == identity and self.tray[slot] is None, '模拟存车条件不符')
            self.tray[slot] = identity
            self.items[identity]['location'] = target
            self.held = None
        elif phase == 'RELEASED':
            require(self.held == identity, '模拟释放前未持有正确物料')
            ring = str(goal['ring'])
            destination = self.rough if target.startswith('rough:') else (
                self.lower if goal['layer'] == 1 else self.upper)
            require(ring not in destination, '模拟目标已占用')
            if goal.get('layer') == 2:
                bottom = self.lower.get(ring)
                require(bottom is not None and self.items[bottom]['color'] == self.items[identity]['color'],
                        '模拟码垛底层不同色')
            destination[ring] = identity
            self.items[identity]['location'] = target
            self.held = None
        result = dict(state=phase.lower(), item=identity, color=self.items[identity]['color'])
        if phase in ('STORED', 'RELEASED', 'PLACED'):
            result['claw_empty'] = self.held is None
        if phase == 'PLACED':
            result.update(stable=True, placement_valid=self.items[identity]['location'] == target)
        return result

    def frames_for(self, task, clock, phase):
        state = task.state
        action = state['pending']
        rule = next(rule for rule in phases(action['goal'], state) if rule[0] == phase)
        camera = 'qr' if phase == 'QR' else 'material'
        observation = self.observe(action['goal'], phase)
        frames = []
        for _ in range(3):
            self.frames[camera] += 1
            frames.append(dict(observation, domain='simulation', run_id=task.run_id,
                               session_id=state['owner'], action_id=action['id'],
                               pose_epoch=action['pose']['epoch'], phase=phase,
                               camera=camera, calibration='sim-v1', view=rule[1],
                               frame_id=self.frames[camera], captured_s=clock.advance(.05), quality=.95))
        return frames

    def result(self, task):
        state = task.state
        action = state['pending']
        goal = action['goal']
        require(self.held is None, '模拟夹爪尚有物料，动作不能完成')
        if goal['kind'] == 'NAV':
            self.station = goal['station']
        return dict(domain='simulation', run_id=task.run_id, session_id=state['owner'],
                    action_id=action['id'], ok=True, stopped=True, claw_empty=True,
                    transport_safe=True, station=goal.get('station'), complete=True,
                    coverage_valid=True, observed_s=task.clock())


class TaskDemo:
    def __init__(self, runtime, owner, code=None, start_zone=1, *, recover=True):
        if code is not None:
            parse_task(code)
        self.clock = SimClock()
        self.ledger = Ledger(Path(runtime) / 'task.sqlite3')
        self.enabled = False
        self.started = False
        self.next_at = 0.0
        self.world = None
        previous = self.ledger.latest()
        if previous:
            self.ledger.audit(previous['run_id'], reduce)
        if previous and previous['status'] not in TERMINAL:
            self.task = Task(self.ledger, previous['run_id'], owner if recover else previous['owner'], self.clock)
            if recover:
                self.task.recover()
        elif code is not None:
            self.task = Task.create(self.ledger, owner, self.clock, start_zone)
            self.world = World(code)
            self.enabled = True
        elif previous:
            self.task = Task(self.ledger, previous['run_id'], previous['owner'], self.clock)
        else:
            self.task = None

    def step(self, now=0.0, *, force=False, link_ok=True):
        if not self.enabled or (not force and now < self.next_at):
            return
        self.next_at = now + .05
        state = self.task.tick()
        if state['status'] in TERMINAL or state['status'] == 'RECONCILE_REQUIRED':
            self.enabled = False
            return
        if not link_ok:
            if self.started:
                self.task.abort('模拟通信中断，结束本轮；不自动续跑')
                self.enabled = False
            return
        self.clock.advance()
        try:
            if state['status'] == 'READY':
                self.task.start()
                self.started = True
            elif state['pending'] is None:
                self.task.intent()
            else:
                action = state['pending']
                identity = action['id']
                if action['status'] == 'INTENT':
                    self.task.action('ACCEPTED', identity)
                elif action['status'] == 'ACCEPTED':
                    self.task.action('RUNNING', identity)
                elif action['status'] == 'EXECUTED':
                    self.task.action('CONFIRM', identity)
                else:
                    rules = phases(action['goal'], state)
                    if len(action['checks']) < len(rules):
                        phase = rules[len(action['checks'])][0]
                        self.task.pose(identity, phase)
                        frames = self.world.frames_for(self.task, self.clock, phase)
                        self.task.checkpoint(identity, phase, frames)
                    else:
                        self.task.action('EXECUTED', identity, result=self.world.result(self.task))
        except TaskError as exc:
            state = self.task.state
            if state['pending']:
                self.task.action('FAILED', state['pending']['id'], reason=str(exc))
            else:
                self.task.abort(str(exc))
            self.enabled = False

    def snapshot(self):
        return None if self.task is None else self.task.snapshot()

    def close(self):
        self.ledger.close()


def main():
    parser = argparse.ArgumentParser(description='任务离线模拟；无设备接口')
    parser.add_argument('--code', help='四组任务码；不填则只查看已有账本')
    parser.add_argument('--start-zone', type=int, choices=(1, 2), default=1)
    parser.add_argument('--runtime', type=Path, default=Path(__file__).resolve().parents[2] / 'output/task-demo')
    parser.add_argument('--abort-run', help='停止核心后显式终止待对账模拟任务')
    args = parser.parse_args()
    if args.code and args.abort_run:
        parser.error('--code与--abort-run不能同时使用')
    lock = ProcessLock(args.runtime / 'core.lock')
    demo = None
    try:
        demo = TaskDemo(args.runtime, uuid.uuid4().hex, args.code, args.start_zone,
                        recover=args.code is not None)
        if args.abort_run:
            state = demo.ledger.load(args.abort_run)
            task = Task(demo.ledger, args.abort_run, state['owner'], demo.clock)
            task.abort('操作者显式终止离线模拟任务')
        for _ in range(1024):
            if not demo.enabled:
                break
            demo.step(force=True)
        report = demo.snapshot()
        text = json.dumps(report, ensure_ascii=False, indent=2)
        (args.runtime / 'task-report.json').write_text(text, encoding='utf-8')
        print(text)
    finally:
        if demo is not None:
            demo.close()
        lock.close()


if __name__ == '__main__':
    main()
