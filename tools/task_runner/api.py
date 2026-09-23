"""Loopback task UI/API, one async orchestration loop and a separate stop path."""
import argparse
import asyncio
import fcntl
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
import json
from pathlib import Path
import signal
import threading
from urllib.parse import urlsplit
from .config import Config, ROOT
from .runner import Runner
from .store import Store


def make_handler(runner, store, loop, host, port):
    class Handler(BaseHTTPRequestHandler):
        def log_message(self, *args):
            pass

        def reply(self, status, value, mime='application/json; charset=utf-8'):
            raw = value if isinstance(value, bytes) else json.dumps(value, ensure_ascii=False).encode()
            self.send_response(status)
            self.send_header('Content-Type', mime)
            self.send_header('Cache-Control', 'no-store')
            self.send_header('X-Content-Type-Options', 'nosniff')
            self.send_header('Content-Security-Policy', "default-src 'self'; img-src 'self' data:; style-src 'self' 'unsafe-inline'; script-src 'self'; frame-ancestors 'self'")
            self.send_header('Content-Length', str(len(raw)))
            self.end_headers()
            try:
                self.wfile.write(raw)
            except (BrokenPipeError, ConnectionResetError):
                pass

        def trusted(self):
            parsed = urlsplit('http://'+self.headers.get('Host', ''))
            # SSH forwards may expose a different local port, but not an arbitrary hostname.
            return parsed.hostname in ('127.0.0.1', 'localhost') and parsed.username is None and parsed.password is None

        def do_GET(self):
            if not self.trusted():
                return self.reply(403, {'error': '不允许的 Host'})
            try:
                if self.path in ('/', '/app.js'):
                    name = 'index.html' if self.path == '/' else 'app.js'
                    mime = 'text/html; charset=utf-8' if name.endswith('html') else 'application/javascript; charset=utf-8'
                    return self.reply(200, Path(__file__).with_name(name).read_bytes(), mime)
                if self.path == '/api/task-runs':
                    return self.reply(200, {'runs': store.recent(), 'mode': runner.config.mode,
                        'model': runner.config.model, 'fallback_model': runner.config.fallback_model,
                        'stopping': runner.stopping,
                        'blocked_by_uncertainty': runner.uncertain})
                if self.path.startswith('/api/task-runs/') and self.path.count('/') == 3:
                    return self.reply(200, store.get(self.path.rsplit('/', 1)[1]))
                return self.reply(404, {'error': '接口不存在'})
            except KeyError:
                return self.reply(404, {'error': '任务不存在'})
            except Exception:
                return self.reply(500, {'error': '读取失败'})

        def do_POST(self):
            if not self.trusted() or self.headers.get('Content-Type', '').split(';')[0] != 'application/json':
                return self.reply(403, {'error': '请求来源或类型不允许'})
            origin = self.headers.get('Origin')
            if origin is not None and origin != 'http://'+self.headers.get('Host', ''):
                return self.reply(403, {'error': '跨站请求被拒绝'})
            try:
                size = int(self.headers.get('Content-Length', '0'))
                if not 0 < size <= 20000:
                    raise ValueError('请求大小无效')
                data = json.loads(self.rfile.read(size))
                if self.path == '/api/task-runs/stop' and data == {}:
                    result = asyncio.run_coroutine_threadsafe(runner.stop_current(), loop).result(70)
                    return self.reply(200, result)
                if self.path == '/api/task-runs' and isinstance(data, dict) and set(data) == {'instruction'}:
                    async def submit():
                        return runner.submit(data['instruction'])
                    run_id = asyncio.run_coroutine_threadsafe(submit(), loop).result(5)
                    return self.reply(202, {'run_id': run_id})
                if self.path.startswith('/api/task-runs/') and self.path.endswith('/cancel') and self.path.count('/') == 4 and data == {}:
                    run_id = self.path.split('/')[3]
                    future = asyncio.run_coroutine_threadsafe(runner.cancel(run_id), loop)
                    # Do not cancel a physical stop if the HTTP client disconnects.
                    result = future.result(70)
                    return self.reply(200, result)
                return self.reply(404, {'error': '接口不存在或参数无效'})
            except (ValueError, KeyError):
                return self.reply(400, {'error': '请求参数无效或任务不存在'})
            except Exception as error:
                from .robot_adapter import Blocked
                return self.reply(409 if isinstance(error, Blocked) else 503,
                    {'error': str(error) if isinstance(error, Blocked) else '请求未确认；请查询原任务，不要重复提交'})
    return Handler


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--env', type=Path, default=ROOT/'.env')
    parser.add_argument('--port', type=int, default=8091)
    args = parser.parse_args()
    config = Config.load(args.env)
    config.state_dir.mkdir(parents=True, exist_ok=True, mode=0o700)
    lock = (config.state_dir/'runner.lock').open('a')
    fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
    store = Store(config.state_dir, secrets=(config.api_key, config.token_file.read_text().strip()))
    loop = asyncio.new_event_loop()
    runner = Runner(config, store)
    server = ThreadingHTTPServer(('127.0.0.1', args.port), make_handler(runner, store, loop, '127.0.0.1', args.port))
    thread = threading.Thread(target=loop.run_forever)
    thread.start()
    server.daemon_threads = True
    def shutdown(*unused):
        threading.Thread(target=server.shutdown, daemon=True).start()
    signal.signal(signal.SIGTERM, shutdown)
    signal.signal(signal.SIGINT, shutdown)
    print(f'Saydo task runner http://127.0.0.1:{args.port} mode={config.mode}', flush=True)
    try:
        server.serve_forever()
    finally:
        async def finish():
            if runner.active:
                if runner.adapter.motion_inflight:
                    await runner.cancel(runner.active['run_id'])
                else:
                    runner.cancelled.set()
                if runner.worker:
                    await asyncio.wait_for(asyncio.shield(runner.worker), timeout=75)
            async with runner.stop_lock:
                pass
        try:
            asyncio.run_coroutine_threadsafe(finish(), loop).result(80)
        finally:
            server.server_close()
            loop.call_soon_threadsafe(loop.stop)
            thread.join()
            loop.close()
            lock.close()


if __name__ == '__main__':
    main()
