"""One bounded, resumable PTY running a fixed command, never a shell endpoint."""
import base64
from collections import OrderedDict
import errno
import fcntl
import hashlib
import os
import pty
import re
import select
import signal
import struct
import termios
import threading
import time
import uuid


class TerminalError(Exception):
    def __init__(self, message, status=400):
        super().__init__(message)
        self.status = status


def dimensions(cols, rows):
    if type(cols) is not int or type(rows) is not int or not 20 <= cols <= 240 or not 8 <= rows <= 120:
        raise TerminalError('终端尺寸无效')
    return cols, rows


def identifier(value):
    if not isinstance(value, str) or not re.fullmatch(r'[a-f0-9]{32}', value):
        raise TerminalError('会话或请求编号无效')
    return value


class TerminalSession:
    def __init__(self, command, cwd, cols, rows, buffer_limit=1024*1024):
        dimensions(cols, rows)
        self.id = uuid.uuid4().hex
        self.lock = threading.RLock()
        self.input_lock = threading.Lock()
        self.buffer = bytearray()
        self.base = 0
        self.buffer_limit = buffer_limit
        self.exit_code = None
        self.running = True
        self.inputs = OrderedDict()
        env = os.environ.copy()
        env.update(TERM='xterm-256color', COLORTERM='truecolor')
        # Resolve everything before forking; the child only establishes its TTY
        # and execs the fixed program. No shell or user-supplied argv is involved.
        pid, fd = pty.fork()
        if pid == 0:
            try:
                os.chdir(cwd)
                fcntl.ioctl(0, termios.TIOCSWINSZ, struct.pack('HHHH', rows, cols, 0, 0))
                os.execve(command[0], command, env)
            except BaseException:
                os.write(2, '无法启动 Codex，请检查服务器上的专用入口。\r\n'.encode())
                os._exit(127)
        self.pid, self.fd = pid, fd
        os.set_blocking(fd, False)
        self.reader = threading.Thread(target=self._read, daemon=True)
        self.reader.start()

    def _read(self):
        status = None
        try:
            while True:
                if status is None:
                    exited, result = os.waitpid(self.pid, os.WNOHANG)
                    if exited:
                        status = result
                        # MCP children may retain the PTY after the CLI exits.
                        # Reap our CLI and close only its remaining process group.
                        try:
                            os.killpg(self.pid, signal.SIGKILL)
                        except ProcessLookupError:
                            pass
                ready, _, _ = select.select([self.fd], [], [], .2)
                if not ready:
                    if status is not None:
                        break
                    continue
                try:
                    chunk = os.read(self.fd, 65536)
                except BlockingIOError:
                    continue
                except OSError as error:
                    if error.errno == errno.EIO:
                        break
                    raise
                if not chunk:
                    break
                with self.lock:
                    self.buffer.extend(chunk)
                    excess = len(self.buffer)-self.buffer_limit
                    if excess > 0:
                        del self.buffer[:excess]
                        self.base += excess
        finally:
            # EOF can precede process exit; do not expose a new session while
            # the old one is still shutting down or leave a zombie process.
            if status is None:
                _, status = os.waitpid(self.pid, 0)
            with self.lock:
                self.exit_code = os.waitstatus_to_exitcode(status)
                self.running = False
            with self.input_lock:
                os.close(self.fd)

    def output(self, offset):
        if type(offset) is not int or offset < 0:
            raise TerminalError('输出游标无效')
        with self.lock:
            end = self.base+len(self.buffer)
            if offset > end:
                raise TerminalError('输出游标超出当前会话')
            start = max(offset, self.base)
            data = bytes(self.buffer[start-self.base:start-self.base+65536])
            return {'session_id': self.id, 'running': self.running, 'exit_code': self.exit_code,
                    'offset': start+len(data), 'end_offset': end, 'reset': offset < self.base,
                    'data': base64.b64encode(data).decode()}

    def send(self, request_id, encoded):
        identifier(request_id)
        if not isinstance(encoded, str) or len(encoded) > 24000:
            raise TerminalError('单次输入过长')
        try:
            data = base64.b64decode(encoded, validate=True)
        except (ValueError, TypeError):
            raise TerminalError('输入编码无效')
        if not 0 < len(data) <= 16384:
            raise TerminalError('单次输入须为1–16384字节')
        digest = hashlib.sha256(data).digest()
        with self.input_lock:
            if request_id in self.inputs:
                old_digest, done = self.inputs[request_id]
                if old_digest != digest:
                    raise TerminalError('重复请求的输入内容不同', 409)
                if not done:
                    raise TerminalError('上次输入发送未确认，请检查终端；不会重复发送', 409)
                return {'accepted': True, 'duplicate': True}
            if not self.running:
                raise TerminalError('Codex会话已结束', 410)
            self.inputs[request_id] = (digest, False)
            while len(self.inputs) > 512:
                self.inputs.popitem(last=False)
            sent, deadline = 0, time.monotonic()+3
            try:
                while sent < len(data):
                    remaining = deadline-time.monotonic()
                    if remaining <= 0 or not select.select([], [self.fd], [], remaining)[1]:
                        raise OSError('PTY write timed out')
                    try:
                        sent += os.write(self.fd, data[sent:])
                    except BlockingIOError:
                        continue
            except OSError:
                raise TerminalError('终端输入发送未确认，请重新连接查看；不会自动重发', 409)
            self.inputs[request_id] = (digest, True)
            return {'accepted': True, 'duplicate': False}

    def resize(self, cols, rows):
        dimensions(cols, rows)
        with self.input_lock:
            if self.running:
                try:
                    fcntl.ioctl(self.fd, termios.TIOCSWINSZ, struct.pack('HHHH', rows, cols, 0, 0))
                    os.killpg(self.pid, signal.SIGWINCH)
                except ProcessLookupError:
                    pass
        return {'accepted': True}

    def close(self):
        # Close is an explicit user action, not a tab-unload handler. It ends the
        # CLI process group, but is not a robot stop command.
        if self.running:
            try:
                os.killpg(self.pid, signal.SIGTERM)
            except ProcessLookupError:
                pass
            self.reader.join(2)
            if self.reader.is_alive():
                try:
                    os.killpg(self.pid, signal.SIGKILL)
                except ProcessLookupError:
                    pass
                self.reader.join(2)
        return {'closed': not self.running}


class SessionManager:
    def __init__(self, command, cwd):
        self.command, self.cwd = command, cwd
        self.lock = threading.RLock()
        self.current = None
        self.starts = OrderedDict()

    def connect(self, request_id, cols, rows):
        identifier(request_id);dimensions(cols, rows)
        with self.lock:
            created = False
            if request_id in self.starts:
                if self.current is None or self.starts[request_id] != self.current.id:
                    raise TerminalError('该连接请求已过期，请重新连接', 410)
            elif self.current is None or not self.current.running:
                self.current = TerminalSession(self.command, self.cwd, cols, rows)
                created = True
            self.starts[request_id] = self.current.id
            while len(self.starts) > 512:
                self.starts.popitem(last=False)
            result = self.current.output(0)
            # The browser explicitly requests output from zero on attachment.
            return {**{k: result[k] for k in ('session_id', 'running', 'exit_code')},
                    'replay_offset': 0 if created else result['end_offset']}

    def session(self, value):
        identifier(value)
        with self.lock:
            if self.current is None or self.current.id != value:
                raise TerminalError('终端会话已失效，请重新连接', 410)
            return self.current

    def close(self):
        with self.lock:
            if self.current is not None:
                self.current.close()
