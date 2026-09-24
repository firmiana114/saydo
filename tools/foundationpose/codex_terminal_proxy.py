"""Allowlisted same-origin proxy to the dedicated loopback Codex terminal bridge."""
import http.client
import json
import re
from urllib.parse import parse_qs, urlsplit


class CodexTerminalProxy:
    def __init__(self, port=8092):self.port=port

    @staticmethod
    def matches(path):
        return path.startswith('/codex-terminal/') or path.startswith('/api/codex-terminal')

    @staticmethod
    def target(method,path):
        assets=('/', '/app.js', '/style.css', '/vendor/xterm.js', '/vendor/xterm.css', '/vendor/addon-fit.js')
        if method=='GET' and path.startswith('/codex-terminal/') and path.removeprefix('/codex-terminal') in assets:
            return path.removeprefix('/codex-terminal')
        if method=='POST' and path in ['/api/codex-terminal/'+n for n in ('connect','input','resize','close')]:
            return path
        if method=='GET':
            parsed=urlsplit(path)
            if parsed.path=='/api/codex-terminal/output' and not parsed.fragment:
                query=parse_qs(parsed.query,strict_parsing=True)
                if (set(query)=={'session_id','offset'} and all(len(v)==1 for v in query.values())
                        and re.fullmatch('[a-f0-9]{32}',query['session_id'][0])
                        and re.fullmatch('[0-9]{1,16}',query['offset'][0])):
                    return path
        raise ValueError('未知的Codex终端路由')

    def request(self,method,path,value=None,authorization=None):
        target=self.target(method,path)
        if authorization is not None and (len(authorization)>256 or '\r' in authorization or '\n' in authorization):
            raise ValueError('终端认证格式无效')
        data=json.dumps(value,ensure_ascii=False,allow_nan=False).encode() if method=='POST' else None
        connection=http.client.HTTPConnection('127.0.0.1',self.port,timeout=8)
        try:
            headers={'Content-Type':'application/json','Origin':f'http://127.0.0.1:{self.port}'}
            if authorization:headers['Authorization']=authorization
            connection.request(method,target,body=data,headers=headers)
            response=connection.getresponse();body=response.read(2*1024*1024+1)
            if len(body)>2*1024*1024:raise ValueError('终端响应超出限制')
            return response.status,body,response.getheader('Content-Type','application/json; charset=utf-8')
        except (OSError,http.client.HTTPException):
            if path=='/codex-terminal/':
                return 503,'<!doctype html><meta charset="utf-8"><h2>Codex终端服务尚未启动</h2><p>请启动MSI上的终端服务后刷新本页。</p>'.encode(),'text/html; charset=utf-8'
            return 503,json.dumps({'error':'终端连接中断；请重新连接，勿重复提交任务。'},ensure_ascii=False).encode(),'application/json; charset=utf-8'
        finally:connection.close()
