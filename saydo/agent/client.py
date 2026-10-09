"""Allowlisted HTTP adapter; never retries an ambiguous motion submission."""

import json
import http.client
import pathlib
import re
import urllib.error
import urllib.parse
import urllib.request
from typing import Optional


class BackendError(Exception):
    def __init__(self, payload):
        self.payload = payload
        super().__init__(
            payload.get('message', payload.get('code', 'Backend error'))
        )


class NoRedirect(urllib.request.HTTPRedirectHandler):
    def redirect_request(self, *args, **kwargs):
        return None


class RobotBackend:
    """8089 的 `/api/agent/` 接口客户端；只走这一组带认证的接口。"""

    def __init__(
        self,
        base_url: str,
        token_file: pathlib.Path,
        timeout: float = 15,
        arm: Optional[str] = None,
    ) -> None:
        """创建客户端；不发起连接。

        Args:
            base_url: 后端地址，只含协议、主机和端口。
            token_file: 访问令牌文件，权限必须是仅属主可读写。
            timeout: 每次请求的超时，单位秒。
            arm: 这个客户端操作哪台机械臂（如 `arm2`）；None 表示不
                指定，由后端按缺省的 1 臂处理。

        Raises:
            ValueError: 地址带有凭据、路径、查询串，或臂编号含非法字符。
        """
        if arm is not None and not re.fullmatch('[a-z0-9_]{1,16}', arm):
            raise ValueError(f'机械臂编号无效：{arm!r}')
        self.arm = arm
        parsed = urllib.parse.urlsplit(base_url)
        if (
            parsed.scheme not in ('http', 'https')
            or not parsed.hostname
            or parsed.username
            or parsed.password
            or parsed.query
            or parsed.fragment
            or parsed.path not in ('', '/')
        ):
            raise ValueError(
                'Backend URL must be a fixed HTTP(S) origin without '
                'credentials or a path'
            )
        self.base_url, self.token_file, self.timeout = (
            base_url.rstrip('/'), pathlib.Path(token_file), timeout
        )
        self.opener = urllib.request.build_opener(
            urllib.request.ProxyHandler({}), NoRedirect()
        )

    def request(self, path: str, value: Optional[dict] = None) -> dict:
        """发一次请求并返回后端的应答。

        `value` 为 None 时是只读查询，否则是提交。提交“开始任务”可能
        让后端发送运动或夹爪指令。创建客户端时指定了机械臂的，每个
        请求都带上它；只有“停止”例外，始终停止全部机械臂，因为出了
        问题需要喊停时不应该还去区分是哪一台。

        Args:
            path: 以 `/api/agent/` 开头的路径，可带查询串。
            value: 提交的内容；None 表示只读查询。

        Returns:
            后端应答（已解析的 JSON）。

        Raises:
            ValueError: 路径不属于 Agent 接口，或令牌文件不合要求。
            BackendError: 后端拒绝、不可连接或应答无效。提交类请求
                连接失败时错误码为 `REQUEST_UNCERTAIN`：后端可能已经
                收到，不得换编号重发。
        """
        if not path.startswith('/api/agent/'):
            raise ValueError('Only dedicated Agent API routes are allowed')
        if self.arm is not None and path != '/api/agent/stop':
            if value is None:
                separator = '&' if '?' in path else '?'
                path = f'{path}{separator}arm={self.arm}'
            elif isinstance(value, dict):
                value = {**value, 'arm': self.arm}
        if self.token_file.stat().st_mode & 0o077:
            raise ValueError(
                'Backend token file must have owner-only permissions'
            )
        token = self.token_file.read_text().strip()
        if len(token) < 32:
            raise ValueError('Backend token is invalid')
        data = (
            None
            if value is None
            else json.dumps(value, allow_nan=False).encode()
        )
        request = urllib.request.Request(
            self.base_url + path,
            data=data,
            headers={
                'Authorization': 'Bearer ' + token,
                'Content-Type': 'application/json',
            },
        )
        try:
            with self.opener.open(request, timeout=self.timeout) as response:
                raw = response.read(4 * 1024 * 1024 + 1)
                if len(raw) > 4 * 1024 * 1024:
                    raise BackendError(
                        {
                            'code': 'RESPONSE_TOO_LARGE',
                            'message': '后端响应超出上限。',
                        }
                    )
                return json.loads(raw)
        except urllib.error.HTTPError as error:
            try:
                payload = json.loads(error.read(65536)).get('error')
            except (ValueError, OSError):
                payload = None
            if not isinstance(payload, dict):
                payload = {
                    'code': 'BACKEND_REJECTED',
                    'message': str(payload or '后端拒绝请求。'),
                }
            raise BackendError(payload) from error
        except (
            urllib.error.URLError,
            TimeoutError,
            ConnectionError,
            OSError,
            ValueError,
            http.client.HTTPException,
        ) as error:
            payload = {
                'code': (
                    'REQUEST_UNCERTAIN'
                    if value is not None
                    else 'BACKEND_UNAVAILABLE'
                ),
                'message': (
                    '后端不可连接或响应超时；写入请求可能已被接收，'
                    '禁止换编号直接重试。'
                ),
            }
            if isinstance(value, dict) and 'request_id' in value:
                payload.update(
                    task_id=value['request_id'],
                    recovery=(
                        'get_task；需要重发时必须复用同一个 request_id 和参数。'
                    ),
                )
            raise BackendError(payload) from error

    def action(self, operation, request_id, arguments):
        return self.request(
            '/api/agent/tasks',
            {
                'operation': operation,
                'request_id': request_id,
                'arguments': arguments,
            },
        )
