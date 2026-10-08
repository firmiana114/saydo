"""Loopback HTTP/PTY bridge for the sixth page; browser access needs a separate token."""
import argparse
import fcntl
import hmac
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
import json
import os
from pathlib import Path
import secrets
import signal
import stat
import sys
import threading
from urllib.parse import parse_qs, urlsplit

from .session import SessionManager, TerminalError

ROOT = Path(__file__).resolve().parents[2]
ASSETS = {'/': ('index.html','text/html; charset=utf-8'),
          '/app.js': ('app.js','application/javascript; charset=utf-8'),
          '/style.css': ('style.css','text/css; charset=utf-8'),
          '/vendor/xterm.js': ('vendor/xterm.js','application/javascript; charset=utf-8'),
          '/vendor/xterm.css': ('vendor/xterm.css','text/css; charset=utf-8'),
          '/vendor/addon-fit.js': ('vendor/addon-fit.js','application/javascript; charset=utf-8')}


def access_token(path):
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True, mode=0o700)
    try:
        fd = os.open(path, os.O_WRONLY | os.O_CREAT | os.O_EXCL, 0o600)
    except FileExistsError:
        pass
    else:
        with os.fdopen(fd, 'w') as output:
            output.write(secrets.token_urlsafe(32)+'\n')
    if path.is_symlink() or not stat.S_ISREG(path.stat().st_mode) or path.stat().st_mode & 0o077:
        raise ValueError('终端访问码文件必须为普通文件，且仅当前用户可读写（600）')
    value = path.read_text().strip()
    if len(value) < 32 or not value.isascii():
        raise ValueError('终端访问码格式无效')
    return value


def make_handler(manager, token):
    class Handler(BaseHTTPRequestHandler):
        def log_message(self, *args):pass

        def reply(self, status, value, mime='application/json; charset=utf-8'):
            body = value if isinstance(value, bytes) else json.dumps(value, ensure_ascii=False).encode()
            self.send_response(status)
            for key, value in {'Content-Type':mime, 'Cache-Control':'no-store', 'X-Content-Type-Options':'nosniff',
                'Content-Security-Policy':"default-src 'self'; script-src 'self'; style-src 'self' 'unsafe-inline'; connect-src 'self'; frame-ancestors 'self'; object-src 'none'; base-uri 'none'",
                'Content-Length':str(len(body))}.items():self.send_header(key, value)
            self.end_headers()
            try:self.wfile.write(body)
            except (BrokenPipeError, ConnectionResetError):pass

        def trusted(self):
            try:
                host = urlsplit('http://'+self.headers.get('Host',''))
                return host.hostname in ('127.0.0.1','localhost') and not host.username and not host.password
            except ValueError:return False

        def authorized(self):
            value = self.headers.get('Authorization','')
            return len(value) <= 256 and hmac.compare_digest(value.encode(), ('Bearer '+token).encode())

        def do_GET(self):
            if not self.trusted():return self.reply(403, {'error':'不允许的主机'})
            if self.path in ASSETS:
                name, mime = ASSETS[self.path]
                return self.reply(200, (Path(__file__).parent / name).read_bytes(), mime)
            if not self.authorized():return self.reply(401, {'error':'请填写正确的终端访问码'})
            try:
                url=urlsplit(self.path);query=parse_qs(url.query,strict_parsing=True)
                if url.path != '/api/codex-terminal/output' or set(query)!={'session_id','offset'} or any(len(v)!=1 for v in query.values()):
                    raise TerminalError('未知的终端接口')
                result=manager.session(query['session_id'][0]).output(int(query['offset'][0]))
                self.reply(200,result)
            except (ValueError,TerminalError) as error:self.reply(getattr(error,'status',400),{'error':str(error)})

        def do_POST(self):
            if not self.trusted():return self.reply(403, {'error':'不允许的主机'})
            origin=self.headers.get('Origin')
            if origin and origin!='http://'+self.headers.get('Host',''):
                return self.reply(403, {'error':'不允许跨站请求'})
            if not self.authorized():return self.reply(401, {'error':'请填写正确的终端访问码'})
            try:
                size=int(self.headers.get('Content-Length','0'))
                if self.headers.get('Content-Type','').split(';')[0]!='application/json' or not 0<size<=26000:
                    raise TerminalError('请求格式或大小无效')
                data=json.loads(self.rfile.read(size))
                if not isinstance(data,dict):raise TerminalError('请求必须为对象')
                if self.path=='/api/codex-terminal/connect' and set(data)=={'request_id','cols','rows'}:
                    result=manager.connect(**data)
                elif self.path=='/api/codex-terminal/input' and set(data)=={'session_id','request_id','data'}:
                    result=manager.session(data['session_id']).send(data['request_id'],data['data'])
                elif self.path=='/api/codex-terminal/resize' and set(data)=={'session_id','cols','rows'}:
                    result=manager.session(data['session_id']).resize(data['cols'],data['rows'])
                elif self.path=='/api/codex-terminal/close' and set(data)=={'session_id'}:
                    result=manager.session(data['session_id']).close()
                else:raise TerminalError('未知的终端接口或参数')
                self.reply(200,result)
            except (ValueError,TypeError,TerminalError) as error:self.reply(getattr(error,'status',400),{'error':str(error)})
            except OSError:self.reply(503,{'error':'终端服务未就绪，请检查服务日志并重新连接'})
    return Handler


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--port',type=int,default=8092)
    parser.add_argument('--state-dir',type=Path,default=ROOT/'work/codex_terminal')
    args=parser.parse_args()
    args.state_dir.mkdir(parents=True,exist_ok=True,mode=0o700)
    lock=(args.state_dir/'server.lock').open('a');fcntl.flock(lock,fcntl.LOCK_EX|fcntl.LOCK_NB)
    token=access_token(args.state_dir/'access_token')
    launcher=ROOT/'tools/robot_agent/codex_launcher.py'
    manager=SessionManager([sys.executable,str(launcher)],str(ROOT))
    server=ThreadingHTTPServer(('127.0.0.1',args.port),make_handler(manager,token));server.daemon_threads=True
    def shutdown(*unused):threading.Thread(target=server.shutdown,daemon=True).start()
    signal.signal(signal.SIGTERM,shutdown);signal.signal(signal.SIGINT,shutdown)
    print(f'Codex terminal bridge http://127.0.0.1:{args.port}; access code file: {args.state_dir / "access_token"}',flush=True)
    try:server.serve_forever()
    finally:server.server_close();manager.close();lock.close()


if __name__=='__main__':main()
