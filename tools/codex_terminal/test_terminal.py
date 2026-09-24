"""Real PTY and HTTP tests, using a harmless echo program (no Codex/hardware)."""
import base64
import http.client
from http.server import ThreadingHTTPServer
import json
import os
from pathlib import Path
import sys
import tempfile
import threading
import time
import unittest
import uuid

from tools.codex_terminal.api import access_token, make_handler
from tools.codex_terminal.session import SessionManager, TerminalError, TerminalSession
from tools.foundationpose.codex_terminal_proxy import CodexTerminalProxy

ECHO = '''import os, sys, tty
print("READY", flush=True)
tty.setraw(0)
while True:
    data = os.read(0, 4096)
    if data == b"\\x04": break
    if data == b"SIZE":
        size = os.get_terminal_size(0)
        data = f"{size.columns}x{size.lines}".encode()
    os.write(1, data)
'''


def uid():return uuid.uuid4().hex

def encoded(value):return base64.b64encode(value).decode()

def wait_for(predicate):
    deadline = time.monotonic()+4
    while time.monotonic()<deadline:
        if predicate():return
        time.sleep(.02)
    raise AssertionError('Timed out waiting for PTY')


class TerminalTests(unittest.TestCase):
    def setUp(self):
        self.manager=SessionManager([sys.executable,'-u','-c',ECHO],os.getcwd())

    def tearDown(self):self.manager.close()

    def connect(self, request_id=None):
        result=self.manager.connect(request_id or uid(),80,24)
        session=self.manager.session(result['session_id'])
        wait_for(lambda:b'READY' in base64.b64decode(session.output(0)['data']))
        # READY is emitted before raw mode; wait for the child to enter its loop.
        time.sleep(.05)
        return session

    def test_unicode_input_deduplicated_resize_and_reconnect(self):
        first_id=uid();session=self.connect(first_id)
        self.assertEqual(session.id,self.manager.connect(uid(),90,30)['session_id'])
        self.assertEqual(session.id,self.manager.connect(first_id,80,24)['session_id'])
        start=session.output(0)['offset'];request_id=uid();message='你好终端'.encode()
        self.assertFalse(session.send(request_id,encoded(message))['duplicate'])
        self.assertTrue(session.send(request_id,encoded(message))['duplicate'])
        wait_for(lambda:message in base64.b64decode(session.output(start)['data']))
        time.sleep(.05)
        self.assertEqual(base64.b64decode(session.output(start)['data']),message)
        with self.assertRaises(TerminalError):session.send(request_id,encoded(b'other'))
        session.resize(100,40);start=session.output(0)['offset']
        session.send(uid(),encoded(b'SIZE'))
        wait_for(lambda:b'100x40' in base64.b64decode(session.output(start)['data']))

    def test_exit_no_retry_respawn_and_stale_session(self):
        request_id=uid();session=self.connect(request_id)
        session.send(uid(),encoded(b'\x04'))
        wait_for(lambda:not session.running)
        self.assertEqual(session.exit_code,0)
        self.assertFalse(self.manager.connect(request_id,80,24)['running'])
        new=self.connect()
        self.assertNotEqual(session.id,new.id)
        with self.assertRaises(TerminalError):self.manager.session(session.id)
        with self.assertRaises(TerminalError):self.manager.connect(request_id,80,24)
        self.assertTrue(new.close()['closed'])

    def test_exited_parent_with_child_holding_tty(self):
        code='import os,time; pid=os.fork(); time.sleep(30) if pid==0 else None'
        session=TerminalSession([sys.executable,'-c',code],os.getcwd(),80,24)
        try:
            wait_for(lambda:not session.running)
            self.assertEqual(session.exit_code,0)
        finally:session.close()

    def test_bounded_output_and_bad_inputs(self):
        session=self.connect();session.buffer_limit=128
        session.send(uid(),encoded(b'a'*1000))
        wait_for(lambda:session.base>0)
        result=session.output(0)
        self.assertTrue(result['reset']);self.assertLessEqual(len(base64.b64decode(result['data'])),128)
        for offset in (-1,True,10**9):
            with self.assertRaises(TerminalError):session.output(offset)
        for value in ('!',encoded(b''),encoded(b'x'*16385)):
            with self.assertRaises(TerminalError):session.send(uid(),value)
        for cols,rows in [(0,24),(80,0),(True,24),(241,24)]:
            with self.assertRaises(TerminalError):session.resize(cols,rows)


class HTTPTests(unittest.TestCase):
    def setUp(self):
        self.manager=SessionManager([sys.executable,'-u','-c',ECHO],os.getcwd())
        self.token='test-only-terminal-token-never-a-production-secret'
        self.server=ThreadingHTTPServer(('127.0.0.1',0),make_handler(self.manager,self.token))
        self.thread=threading.Thread(target=self.server.serve_forever,daemon=True);self.thread.start()
        self.port=self.server.server_port
        self.proxy=CodexTerminalProxy(self.port)

    def tearDown(self):
        self.server.shutdown();self.server.server_close();self.thread.join();self.manager.close()

    def request(self,method,path,body=None,**headers):
        conn=http.client.HTTPConnection('127.0.0.1',self.port,timeout=5)
        try:
            if body is not None:
                body=json.dumps(body);headers.setdefault('Content-Type','application/json')
            conn.request(method,path,body,headers)
            response=conn.getresponse()
            return response.status,response.read(),dict(response.getheaders())
        finally:conn.close()

    def test_no_start_on_assets_and_authentication(self):
        for asset in ('/','/app.js','/style.css','/vendor/xterm.js','/vendor/xterm.css','/vendor/addon-fit.js'):
            status,body,headers=self.request('GET',asset)
            self.assertEqual(status,200);self.assertTrue(body)
            self.assertIn('Content-Security-Policy',headers)
        self.assertIsNone(self.manager.current)
        body={'request_id':uid(),'cols':80,'rows':24}
        self.assertEqual(self.request('POST','/api/codex-terminal/connect',body)[0],401)
        auth={'Authorization':'Bearer '+self.token}
        self.assertEqual(self.request('POST','/api/codex-terminal/connect',body,Origin='http://evil.test',**auth)[0],403)
        self.assertEqual(self.request('POST','/api/codex-terminal/connect',body,Host='evil.test',**auth)[0],403)
        self.assertEqual(self.request('POST','/api/codex-terminal/connect',{**body,'command':'sh'},**auth)[0],400)
        self.assertEqual(self.request('POST','/api/codex-terminal/connect',{'large':'x'*27000},**auth)[0],400)
        self.assertIsNone(self.manager.current)

    def test_proxy_routes_and_authenticated_lifecycle(self):
        status,body,_=self.proxy.request('GET','/codex-terminal/')
        self.assertEqual(status,200);self.assertIn('Codex'.encode(),body)
        self.assertEqual(self.proxy.request('POST','/api/codex-terminal/connect',{'request_id':uid(),'cols':80,'rows':24})[0],401)
        auth='Bearer '+self.token
        status,body,_=self.proxy.request('POST','/api/codex-terminal/connect',{'request_id':uid(),'cols':80,'rows':24},auth)
        self.assertEqual(status,200);sid=json.loads(body)['session_id']
        status,body,_=self.proxy.request('GET',f'/api/codex-terminal/output?session_id={sid}&offset=0',authorization=auth)
        self.assertEqual(status,200);self.assertEqual(json.loads(body)['session_id'],sid)
        status,body,_=self.proxy.request('POST','/api/codex-terminal/close',{'session_id':sid},auth)
        self.assertEqual(status,200);self.assertTrue(json.loads(body)['closed'])
        for method,path in [('GET','/codex-terminal/../../.env'),('POST','/api/codex-terminal/exec'),('GET','http://evil.test/'),('GET',f'/api/codex-terminal/output?session_id={sid}&offset=0&url=http://evil.test')]:
            with self.assertRaises(ValueError):self.proxy.request(method,path)
        with self.assertRaises(ValueError):self.proxy.request('GET','/codex-terminal/',authorization='Bearer x\r\nCookie: bad')

    def test_token_permissions_and_reuse(self):
        with tempfile.TemporaryDirectory() as directory:
            path=Path(directory)/'private'/'token'
            token=access_token(path)
            self.assertEqual(token,access_token(path))
            self.assertEqual(path.stat().st_mode&0o777,0o600)
            self.assertEqual(path.parent.stat().st_mode&0o777,0o700)
            path.chmod(0o644)
            with self.assertRaises(ValueError):access_token(path)


if __name__=='__main__':unittest.main()
