"""Immutable, single-use plans scoped to one controller process/session."""

import contextlib
import hashlib
import json
import pathlib
import re
import sqlite3
import time
import uuid


def fingerprint(value):
    return hashlib.sha256(
        json.dumps(value, sort_keys=True, allow_nan=False).encode()
    ).hexdigest()


class ObjectPlanStore:
    """持物移动方案的仓库（SQLite 文件），每台臂一个。

    方案先规划并保存，之后凭编号领取执行。方案只在生成它的那次后端
    运行期间有效，有时限，而且只能被领取一次，避免同一条路线被重复
    执行或在现场已经变化之后执行。
    """
    def __init__(
        self, root: pathlib.Path, state_directory: str = 'work/grasp'
    ) -> None:
        """打开一台臂的方案库，文件不存在时创建。

        方案里的路径是按某一台臂的关节算出来的，所以每台臂一个库，
        一台臂规划的方案不会被另一台臂取去执行。

        Args:
            root: 项目根目录。
            state_directory: 该臂的运行记录目录，相对项目根目录；
                缺省是 1 臂的目录。
        """
        self.session_id = uuid.uuid4().hex
        self.path = (
            pathlib.Path(root)
            / state_directory
            / 'object_motion_plans.sqlite'
        )
        self.path.parent.mkdir(parents=True, exist_ok=True)
        with contextlib.closing(self.connect()) as db, db:
            db.execute(
                'CREATE TABLE IF NOT EXISTS plans (id TEXT PRIMARY KEY, '
                'session TEXT, body TEXT, digest TEXT, state TEXT, request_id '
                'TEXT)'
            )

    def connect(self):
        return sqlite3.connect(self.path, timeout=3)

    def save(self, plan):
        plan = {
            **plan,
            'plan_id': uuid.uuid4().hex,
            'session_id': self.session_id,
            'created_at': time.time(),
            'expires_at': time.time() + 300,
        }
        with contextlib.closing(self.connect()) as db, db:
            db.execute(
                'INSERT INTO plans VALUES (?,?,?,?,?,NULL)',
                (
                    plan['plan_id'],
                    self.session_id,
                    json.dumps(plan, allow_nan=False),
                    fingerprint(plan),
                    'ready',
                ),
            )
        return plan

    def get(self, plan_id):
        if (
            not isinstance(plan_id, str)
            or not re.fullmatch('[a-f0-9]{32}', plan_id)
        ):
            raise ValueError('方案编号无效')
        with contextlib.closing(self.connect()) as db:
            row = db.execute(
                'SELECT session,body,digest,state,request_id FROM plans WHERE '
                'id=?',
                (plan_id,),
            ).fetchone()
        if row is None:
            raise ValueError('方案不存在')
        plan = json.loads(row[1])
        if row[0] != self.session_id:
            raise ValueError('后端已重启，旧方案不可执行，请重新规划')
        if fingerprint(plan) != row[2]:
            raise ValueError('方案内容校验失败')
        if time.time() > plan['expires_at']:
            raise ValueError('方案超过5分钟有效期，请重新规划')
        if row[3] != 'ready':
            raise ValueError('方案已被领取或执行，禁止再次执行；请查询原任务')
        return plan

    def claim(self, plan_id, request_id):
        plan = self.get(plan_id)
        with contextlib.closing(self.connect()) as db, db:
            changed = db.execute(
                "UPDATE plans SET state='executing',request_id=? WHERE id=? "
                "AND state='ready' AND session=?",
                (request_id, plan_id, self.session_id),
            ).rowcount
            if changed != 1:
                raise ValueError('方案已被其他任务领取')
            if time.time() > plan['expires_at']:
                raise ValueError('方案领取时已过期，请重新规划')
        return plan

    def finish(self, plan_id, state):
        with contextlib.closing(self.connect()) as db, db:
            db.execute(
                'UPDATE plans SET state=? WHERE id=? AND session=?',
                (state, plan_id, self.session_id),
            )
