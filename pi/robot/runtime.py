"""同机原子快照与单进程锁；Web读取快照，不继承设备对象。"""

import json
import os
import time
from pathlib import Path


class ProcessLock:
    def __init__(self, path):
        Path(path).parent.mkdir(parents=True, exist_ok=True)
        self.file = open(path, 'a+b')
        try:
            if os.name == 'nt':
                import msvcrt
                self.file.seek(0)
                if not self.file.read(1):
                    self.file.write(b'0')
                    self.file.flush()
                self.file.seek(0)
                msvcrt.locking(self.file.fileno(), msvcrt.LK_NBLCK, 1)
            else:
                import fcntl
                fcntl.flock(self.file.fileno(), fcntl.LOCK_EX | fcntl.LOCK_NB)
        except OSError:
            self.file.close()
            raise RuntimeError('该运行目录已有控制核心，拒绝重复启动') from None

    def close(self):
        self.file.close()


def save_snapshot(path, state):
    target = Path(path)
    temporary = target.with_suffix('.tmp')
    temporary.write_text(json.dumps(state, ensure_ascii=False, allow_nan=False), encoding='utf-8')
    # Windows网页读取时可能短暂占用目标文件；有界重试，不无限阻塞保活。
    for attempt in range(3):
        try:
            os.replace(temporary, target)
            return
        except PermissionError:
            if attempt == 2:
                raise
            time.sleep(.005)


def load_snapshot(path, now=None):
    now = time.monotonic() if now is None else now
    try:
        # 同机单调时钟；重启后旧快照禁止凭文件mtime继续显示在线。
        state = json.loads(Path(path).read_text(encoding='utf-8'))
        age = now - state['snapshot_at_s']
        if not 0 <= age < .5:
            raise ValueError('控制核心快照过期')
        if not 0 <= time.time() - state['snapshot_wall_s'] < 1:
            raise ValueError('快照不属于当前时间窗口')
        if state['mode'] not in ('offline', 'sim', 'serial') or state['readonly'] is not True:
            raise ValueError('快照阶段或模式不符')
        state['core_fresh'] = True
        state['core_age_ms'] = int(age * 1000)
        return state
    except (OSError, ValueError, TypeError, KeyError):
        return dict(core_fresh=False, connected=False, readonly=True, ready=False,
                    mode='unknown', message='控制核心未运行或状态已过期', board=None,
                    pose=None, pose_valid=False, zero_valid=False,
                    physical_stop_confirmed=False, task=None, grabs=None, places=None,
                    events=[], core_age_ms=None, state_age_ms=None, firmware='')
