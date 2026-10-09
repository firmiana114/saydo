"""按机械臂分发 Agent 请求。

每台臂有自己的 `agent_api.AgentAPI`（各自的任务状态和动作控制器）。
本模块站在它们前面，只做一件事：看请求指的是哪台臂，把请求交给对应的
那一个。它自己不规划、不连接控制器。

请求用 `arm` 指定机械臂：查询写在查询串里（`?arm=arm2`），提交写在请求
体里（`"arm": "arm2"`）。不写时指 1 臂，所以只有一台臂时的用法不变。
带任务编号的请求不需要写 `arm`，按任务记录里的臂处理。

只有“停止”例外：不指定臂时停止全部机械臂，因为喊停的人通常来不及分辨
是哪一台。
"""

from __future__ import annotations

import contextlib
import json
import logging
import pathlib
import threading
from typing import Any, Dict, Mapping, Optional

from saydo.robot import arms
from saydo.workbench import agent_api

LOG = logging.getLogger(__name__)


def controllers_of(tracker: Any) -> Dict[str, Any]:
    """取出一个跟踪器名下的全部动作控制器。

    Args:
        tracker: 跟踪器。较早的写法只有 `grasp` 一个控制器，没有 `arms`。

    Returns:
        臂编号到动作控制器的映射；只有 `grasp` 时把它当作 1 臂。
    """
    controllers = getattr(tracker, 'arms', None)
    if controllers:
        return dict(controllers)
    return {arms.DEFAULT_ARM_ID: tracker.grasp}


class ArmRouter:
    """把 Agent 请求交给对应机械臂的接口。

    Attributes:
        apis: 臂编号到该臂接口的映射，顺序与登记表一致。
        default: 1 臂的接口；不指定臂的请求交给它。
    """

    def __init__(self, tracker: Any, token_path: pathlib.Path) -> None:
        """为每台已配置的臂创建一个接口。

        Args:
            tracker: 跟踪器，名下有各臂的动作控制器。
            token_path: 访问令牌文件，各臂共用。

        Raises:
            ValueError: 令牌文件权限过宽或令牌过短。
        """
        self.apis: Dict[str, agent_api.AgentAPI] = {
            arm_id: agent_api.AgentAPI(
                controller.tracker, token_path, controller
            )
            for arm_id, controller in controllers_of(tracker).items()
        }
        self.default = self.apis[arms.DEFAULT_ARM_ID]
        # 任务编号在各臂之间共用一个数据库。两台臂的“开始任务”逐个处理，
        # 保证同一个编号不会被两台臂同时登记。
        self._start_lock = threading.Lock()

    def authorized(self, header: Optional[str]) -> bool:
        """判断请求头里的令牌是否正确。

        Args:
            header: `Authorization` 请求头的值，没有时为 None。

        Returns:
            令牌是否正确。
        """
        return self.default.authorized(header)

    def api_for(self, arm_id: Any) -> agent_api.AgentAPI:
        """返回一台臂的接口。

        Args:
            arm_id: 臂编号。

        Returns:
            该臂的接口。

        Raises:
            agent_api.AgentError: 没有这个编号，或这台臂在本机没有配置。
        """
        if not isinstance(arm_id, str) or arm_id not in self.apis:
            available = '、'.join(self.apis)
            raise agent_api.AgentError(
                'UNKNOWN_ARM',
                f'未知或未配置的机械臂 {arm_id!r}，可用的编号：{available}。',
            )
        return self.apis[arm_id]

    def _task_arm(self, task_id: str) -> Optional[str]:
        """查出一个任务属于哪台臂；没有这个任务时返回 None。"""
        with contextlib.closing(self.default.connect()) as db:
            row = db.execute(
                'SELECT body FROM tasks WHERE id=?', (task_id,)
            ).fetchone()
        if row is None:
            return None
        # 引入多臂之前的任务记录没有 arm 字段，它们都是 1 臂的。
        arm_id = json.loads(row[0]).get('arm', arms.DEFAULT_ARM_ID)
        return arm_id if arm_id in self.apis else arms.DEFAULT_ARM_ID

    def summary(self) -> dict:
        """列出各臂的身份和当前状态；不连接控制器。

        Returns:
            含 `default_arm` 和 `arms` 列表。每一项有臂编号、名字、左右、
            是否有任务在进行、持物记录、正在处理的任务编号、这台臂是否
            已接入视觉，以及实时跟踪现在是不是接在它的相机上
            （`live_tracking`；不是的臂只能先采集场景快照再动作）。
        """
        rows = []
        for arm_id, api in self.apis.items():
            controller = api.grasp
            with controller.lock:
                status = dict(controller.status)
                held = controller.held_snapshot()
            rows.append(
                {
                    'arm': arm_id,
                    'label': controller.arm.label,
                    'side': controller.arm.side,
                    'state': status.get('state'),
                    'busy': bool(status.get('busy')),
                    'mode': status.get('mode'),
                    'held_object': held,
                    'active_task_id': api.active,
                    'vision_available': (
                        api.tracker.state.get('state') != 'vision_unavailable'
                    ),
                    'live_tracking': bool(getattr(api.tracker, 'live', True)),
                }
            )
        return {'default_arm': arms.DEFAULT_ARM_ID, 'arms': rows}

    def get(self, path: str, params: Mapping[str, list]) -> dict:
        """处理一次只读查询；不发送任何指令。

        Args:
            path: 请求路径，不含查询串。
            params: 查询参数，每个值是字符串列表（`parse_qs` 的结果）。

        Returns:
            可直接序列化为 JSON 的应答。

        Raises:
            agent_api.AgentError: 臂编号或路径未知，或查询的对象不存在。
        """
        remaining = dict(params)
        requested = remaining.pop('arm', [arms.DEFAULT_ARM_ID])[0]
        if path == '/api/agent/arms':
            return self.summary()
        task_id = None
        if path.startswith('/api/agent/tasks/'):
            task_id = path.rsplit('/', 1)[1]
        elif path == '/api/agent/task-diagnostics':
            task_id = remaining.get('task_id', [''])[0]
        if task_id is not None:
            requested = self._task_arm(task_id) or requested
        return self.api_for(requested).get(path, remaining)

    def post(self, path: str, value: Any) -> dict:
        """处理一次提交。开始任务和停止可能发送运动或夹爪指令。

        Args:
            path: 请求路径。
            value: 请求体（已解析的 JSON）。

        Returns:
            可直接序列化为 JSON 的应答。

        Raises:
            agent_api.AgentError: 臂编号、路径或参数无效，或请求被拒绝。
        """
        if (
            path == '/api/agent/stop'
            and isinstance(value, dict)
            and set(value) <= {'arm'}
        ):
            return self.stop(value.get('arm'))
        requested = arms.DEFAULT_ARM_ID
        if isinstance(value, dict) and 'arm' in value:
            value = dict(value)
            requested = value.pop('arm')
        if path == '/api/agent/tasks':
            return self._start(requested, value)
        if (
            path == '/api/agent/cancel-planning'
            and isinstance(value, dict)
            and isinstance(value.get('task_id'), str)
        ):
            requested = self._task_arm(value['task_id']) or requested
        return self.api_for(requested).post(path, value)

    def _start(self, arm_id: Any, request: Any) -> dict:
        """在指定的臂上开始一个任务；同一个编号不能用于两台臂。"""
        api = self.api_for(arm_id)
        with self._start_lock:
            task_id = (
                request.get('request_id') if isinstance(request, dict) else None
            )
            if isinstance(task_id, str):
                owner = self._task_arm(task_id)
                if owner is not None and owner != arm_id:
                    raise agent_api.AgentError(
                        'REQUEST_ID_CONFLICT',
                        f'request_id 已用于 {owner} 的任务，'
                        f'不能再用于 {arm_id}。',
                    )
            return api.start(request)

    def stop(self, arm_id: Optional[str] = None) -> dict:
        """停止机械臂；会向控制器发送停止指令，不松开夹爪。

        不指定臂时同时停止全部已配置的臂，各臂并行发送，互不等待。

        Args:
            arm_id: 只停止这一台；None 表示全部。

        Returns:
            只有一台臂时是该臂的停止记录。多台臂时含总的
            `stop_acknowledged`、`stationary_observed` 和各臂的记录
            `arms`。

        Raises:
            agent_api.AgentError: 有臂的停止未得到确认（`STOP_UNCONFIRMED`），
                `arms` 里写明各臂的情况。
        """
        if arm_id is not None:
            return self.api_for(arm_id).stop_robot()
        if len(self.apis) == 1:
            return self.default.stop_robot()
        records: Dict[str, dict] = {}
        failures: Dict[str, dict] = {}

        def stop_one(name: str, api: agent_api.AgentAPI) -> None:
            try:
                records[name] = api.stop_robot()
            except agent_api.AgentError as error:
                failures[name] = error.payload
            except Exception as error:
                # 线程入口：一台臂的意外错误不能让另一台臂的停止结果
                # 丢失，记录后按“未确认”上报。
                LOG.exception('%s 停止时出现未预期的错误', name)
                failures[name] = {
                    'code': 'STOP_UNCONFIRMED',
                    'message': f'{type(error).__name__}: {error}',
                    'stop_acknowledged': False,
                }

        threads = [
            threading.Thread(target=stop_one, args=item, daemon=True)
            for item in self.apis.items()
        ]
        for thread in threads:
            thread.start()
        for thread in threads:
            thread.join()
        if failures:
            names = '、'.join(failures)
            LOG.warning('停止未全部确认：%s', names)
            raise agent_api.AgentError(
                'STOP_UNCONFIRMED',
                f'以下机械臂的停止未得到确认：{names}。',
                stop_acknowledged=all(
                    failure.get('stop_acknowledged', False)
                    for failure in failures.values()
                ),
                stationary_observed=False,
                arms={**records, **failures},
                action_required='使用现场急停并核对；不要开始新动作。',
            )
        return {
            'requested_at': min(
                record['requested_at'] for record in records.values()
            ),
            'stop_acknowledged': True,
            'stationary_observed': True,
            'arms': records,
            'note': '软件停止，不替代现场急停；未松爪。',
        }
