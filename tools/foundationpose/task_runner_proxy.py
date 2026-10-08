"""Fixed loopback proxy for the fifth workspace page; no robot commands."""
import http.client
import json
import re


class TaskRunnerProxy:
    def __init__(self, port=8091):
        self.port = port

    @staticmethod
    def matches(path):
        return path.startswith('/task-runner/') or path.startswith('/api/task-runs')

    @staticmethod
    def target(method, path):
        if method == 'GET' and path in ('/task-runner/', '/task-runner/app.js'):
            return path.removeprefix('/task-runner')
        if method == 'GET' and re.fullmatch(r'/api/task-runs(?:/[a-f0-9]{32})?', path):
            return path
        if method == 'POST' and (path in ('/api/task-runs', '/api/task-runs/stop') or re.fullmatch(r'/api/task-runs/[a-f0-9]{32}/cancel', path)):
            return path
        raise ValueError('未知的任务执行器路由')

    def request(self, method, path, value=None):
        target = self.target(method, path)
        data = json.dumps(value, ensure_ascii=False, allow_nan=False).encode() if method == 'POST' else None
        connection = http.client.HTTPConnection('127.0.0.1', self.port, timeout=75 if path.endswith(('/cancel', '/stop')) else 8)
        try:
            # Do not forward browser cookies, credentials, or arbitrary target URLs.
            connection.request(method, target, body=data, headers={'Content-Type': 'application/json',
                'Origin': f'http://127.0.0.1:{self.port}'})
            response = connection.getresponse()
            body = response.read(4*1024*1024+1)
            if len(body) > 4*1024*1024:
                raise ValueError('任务执行器响应超出限制')
            return response.status, body, response.getheader('Content-Type', 'application/json; charset=utf-8')
        except (OSError, http.client.HTTPException):
            if path == '/task-runner/':
                body = '<!doctype html><meta charset="utf-8"><h2>任务服务暂不可用</h2><p>请稍后刷新此页面。</p>'.encode()
                return 503, body, 'text/html; charset=utf-8'
            return 503, json.dumps({'error': '任务服务未连接或请求未确认；请查询原任务，不要重复提交。'}, ensure_ascii=False).encode(), 'application/json; charset=utf-8'
        finally:
            connection.close()
