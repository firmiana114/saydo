"""Small durable run journal. Never auto-resumes an interrupted run."""

import json
import os
import pathlib
import sqlite3
import time
import uuid
import contextlib


class Store:
    def __init__(self, directory, secrets=()):
        self.directory = pathlib.Path(directory)
        self.directory.mkdir(parents=True, exist_ok=True, mode=0o700)
        self.directory.chmod(0o700)
        self.path = self.directory / 'runs.sqlite'
        self.secrets = tuple(s for s in secrets if s)
        with self.connect() as db:
            db.execute(
                'CREATE TABLE IF NOT EXISTS runs (id TEXT PRIMARY KEY, body '
                'TEXT NOT NULL)'
            )
            db.execute(
                'CREATE TABLE IF NOT EXISTS calls (run_id TEXT, call_id TEXT, '
                'body TEXT NOT NULL, PRIMARY KEY(run_id,call_id))'
            )
            for key, body in db.execute('SELECT id,body FROM runs').fetchall():
                run = json.loads(body)
                if run['state'] == 'running':
                    calls = [
                        json.loads(r[0])
                        for r in db.execute(
                            'SELECT body FROM calls WHERE run_id=?', (key,)
                        )
                    ]
                    unknown = any(
                        (
                            (
                                c['name']
                                in (
                                    'grasp',
                                    'place',
                                    'reset_arm',
                                    'reorient',
                                    'execute_plan',
                                    'stop_robot',
                                )
                            )
                            and c['state'] != 'completed'
                        )
                        for c in calls
                    )
                    run.update(
                        state='interrupted',
                        phase='进程重启；未自动恢复',
                        outcome_unknown=(
                            unknown or run.get('outcome_unknown', False)
                        ),
                        updated_at=time.time(),
                    )
                    if run.get('timeline_version') == 1:
                        events = run.setdefault('events', [])
                        events.append(
                            {
                                'event': 'final',
                                'seq': len(events) + 1,
                                'at': time.time(),
                                'state': 'interrupted',
                                'text': '进程重启，任务已中断；未自动恢复。',
                            }
                        )
                    db.execute(
                        'UPDATE runs SET body=? WHERE id=?',
                        (self.encode(run), key),
                    )
        self.path.chmod(0o600)

    @contextlib.contextmanager
    def connect(self):
        db = sqlite3.connect(self.path, timeout=5)
        db.execute('PRAGMA synchronous=FULL')
        try:
            with db:
                yield db
        finally:
            db.close()

    def has_uncertain_runs(self):
        with self.connect() as db:
            return any(
                json.loads(row[0]).get('outcome_unknown')
                for row in db.execute('SELECT body FROM runs')
            )

    def encode(self, value):
        text = json.dumps(value, ensure_ascii=False, allow_nan=False)
        for secret in self.secrets:
            text = text.replace(secret, '[REDACTED]')
        return text

    def create(self, instruction, model, mode, reasoning_effort=None):
        run = {
            'run_id': uuid.uuid4().hex,
            'instruction': instruction,
            'model': model,
            'mode': mode,
            'reasoning_effort': reasoning_effort,
            'state': 'running',
            'phase': '读取初始状态',
            'created_at': time.time(),
            'updated_at': time.time(),
            'timeline_version': 1,
            'events': [],
        }
        with self.connect() as db:
            db.execute(
                'INSERT INTO runs VALUES (?,?)',
                (run['run_id'], self.encode(run)),
            )
        return run

    def save(self, run):
        run['updated_at'] = time.time()
        with self.connect() as db:
            db.execute(
                'UPDATE runs SET body=? WHERE id=?',
                (self.encode(run), run['run_id']),
            )

    def get(self, run_id):
        with self.connect() as db:
            row = db.execute(
                'SELECT body FROM runs WHERE id=?', (run_id,)
            ).fetchone()
            calls = db.execute(
                'SELECT body FROM calls WHERE run_id=? ORDER BY rowid',
                (run_id,),
            ).fetchall()
        if row is None:
            raise KeyError('任务不存在')
        return {
            **json.loads(row[0]), 'calls': [json.loads(c[0]) for c in calls]
        }

    def recent(self):
        with self.connect() as db:
            runs = [
                json.loads(row[0])
                for row in db.execute(
                    'SELECT body FROM runs ORDER BY rowid DESC LIMIT 20'
                )
            ]
        # The history list needs summaries only; fetch a selected run's full
        # timeline separately.
        return [
            {key: value for key, value in run.items() if key != 'events'}
            for run in runs
        ]

    def prepare(self, run_id, call_id, name, args, step):
        record = {
            'call_id': call_id,
            'name': name,
            'arguments': args,
            'request_id': f'run-{run_id}-step-{step:03}',
            'state': 'prepared',
            'created_at': time.time(),
        }
        with self.connect() as db:
            db.execute(
                'INSERT INTO calls VALUES (?,?,?)',
                (run_id, call_id, self.encode(record)),
            )
        return record

    def save_call(self, run_id, record):
        with self.connect() as db:
            db.execute(
                'UPDATE calls SET body=? WHERE run_id=? AND call_id=?',
                (self.encode(record), run_id, record['call_id']),
            )
