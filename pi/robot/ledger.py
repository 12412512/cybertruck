"""SQLite持久事件账本。每次事件和状态投影在同一事务内提交。"""

import copy
import json
import math
from pathlib import Path
import sqlite3


class LedgerError(ValueError):
    pass


def canonical(value):
    return json.dumps(value, ensure_ascii=False, sort_keys=True, separators=(',', ':'), allow_nan=False)


class Ledger:
    def __init__(self, path):
        path = Path(path)
        path.parent.mkdir(parents=True, exist_ok=True)
        self.db = sqlite3.connect(path, timeout=2, isolation_level=None)
        version = self.db.execute('PRAGMA user_version').fetchone()[0]
        if version not in (0, 1):
            self.db.close()
            raise LedgerError('不支持的账本版本，保留原文件')
        self.db.execute('PRAGMA foreign_keys=ON')
        self.db.execute('PRAGMA journal_mode=WAL')
        self.db.execute('PRAGMA synchronous=FULL')
        self.db.executescript('''
            CREATE TABLE IF NOT EXISTS runs (
                id TEXT PRIMARY KEY,
                state TEXT NOT NULL,
                revision INTEGER NOT NULL
            );
            CREATE TABLE IF NOT EXISTS events (
                sequence INTEGER PRIMARY KEY,
                run_id TEXT NOT NULL REFERENCES runs(id),
                event_id TEXT NOT NULL,
                revision INTEGER NOT NULL,
                kind TEXT NOT NULL,
                at_s REAL NOT NULL,
                payload TEXT NOT NULL,
                UNIQUE(run_id,event_id),
                UNIQUE(run_id,revision)
            );
            PRAGMA user_version=1;
        ''')

    def create(self, state, at_s):
        encoded = canonical(state)
        if state.get('domain') != 'simulation' or not math.isfinite(at_s):
            raise LedgerError('当前节点只允许模拟任务账本')
        self.db.execute('BEGIN IMMEDIATE')
        try:
            rows = self.db.execute('SELECT state FROM runs').fetchall()
            if any(json.loads(row[0])['status'] not in ('COMPLETE', 'ABORTED') for row in rows):
                raise LedgerError('已有未结束任务，禁止另开活动账本')
            self.db.execute('INSERT INTO runs VALUES (?,?,0)', (state['run_id'], encoded))
            self.db.execute('INSERT INTO events(run_id,event_id,revision,kind,at_s,payload) VALUES (?,?,0,?,?,?)',
                            (state['run_id'], 'created', 'CREATED', at_s, encoded))
            self.db.execute('COMMIT')
        except BaseException:
            self.db.execute('ROLLBACK')
            raise
        return self.load(state['run_id'])

    def load(self, run_id):
        row = self.db.execute('SELECT state,revision FROM runs WHERE id=?', (run_id,)).fetchone()
        if row is None:
            raise LedgerError('任务不存在')
        state = json.loads(row[0])
        state['revision'] = row[1]
        return state

    def latest(self):
        row = self.db.execute('SELECT id FROM runs ORDER BY rowid DESC LIMIT 1').fetchone()
        return None if row is None else self.load(row[0])

    def append(self, run_id, event_id, kind, payload, at_s, reducer):
        encoded = canonical(payload)
        if not event_id or len(event_id) > 128 or len(encoded.encode('utf-8')) > 65536 or not math.isfinite(at_s):
            raise LedgerError('事件编号、大小或时间无效')
        self.db.execute('BEGIN IMMEDIATE')
        try:
            old = self.db.execute('SELECT kind,payload FROM events WHERE run_id=? AND event_id=?',
                                  (run_id, event_id)).fetchone()
            state = self.load(run_id)
            if old:
                if old != (kind, encoded):
                    raise LedgerError('同一事件编号内容冲突')
                self.db.execute('COMMIT')
                return state
            if state['revision'] >= 4096:
                raise LedgerError('本轮事件超过上限，停止推进')
            candidate = reducer(copy.deepcopy(state), kind, copy.deepcopy(payload), at_s)
            if candidate['run_id'] != run_id or candidate['domain'] != state['domain']:
                raise LedgerError('不允许改变任务身份或证据域')
            revision = state['revision'] + 1
            candidate['revision'] = revision
            self.db.execute('INSERT INTO events(run_id,event_id,revision,kind,at_s,payload) VALUES (?,?,?,?,?,?)',
                            (run_id, event_id, revision, kind, at_s, encoded))
            self.db.execute('UPDATE runs SET state=?,revision=? WHERE id=?',
                            (canonical(candidate), revision, run_id))
            self.db.execute('COMMIT')
            return candidate
        except BaseException:
            self.db.execute('ROLLBACK')
            raise

    def events(self, run_id):
        rows = self.db.execute('SELECT event_id,kind,at_s,payload FROM events WHERE run_id=? ORDER BY revision',
                               (run_id,)).fetchall()
        return [dict(event_id=row[0], kind=row[1], at_s=row[2], payload=json.loads(row[3])) for row in rows]

    def replay(self, run_id, reducer):
        events = self.events(run_id)
        if not events or events[0]['kind'] != 'CREATED':
            raise LedgerError('缺少创建事件')
        state = events[0]['payload']
        state['revision'] = 0
        for revision, event in enumerate(events[1:], start=1):
            state = reducer(state, event['kind'], event['payload'], event['at_s'])
            state['revision'] = revision
        return state

    def audit(self, run_id, reducer):
        if canonical(self.load(run_id)) != canonical(self.replay(run_id, reducer)):
            raise LedgerError('事件重放与状态投影不一致，保留证据并停止')

    def close(self):
        self.db.close()
