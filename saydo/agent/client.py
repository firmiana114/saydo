"""Allowlisted HTTP adapter; never retries an ambiguous motion submission."""

import json
import http.client
import pathlib
import urllib.error
import urllib.parse
import urllib.request


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
    def __init__(self, base_url, token_file, timeout=15):
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

    def request(self, path, value=None):
        if not path.startswith('/api/agent/'):
            raise ValueError('Only dedicated Agent API routes are allowed')
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
