#!/usr/bin/env python3
"""Local-only RealMan web pendant gateway via the existing msi SSH host.
Run locally: python3 scripts/teach_pendant_gateway.py ; stop with Ctrl-C. No controller files are modified.
"""

import http.client
import http.server
import signal
import socket
import subprocess
import threading
import time

WEB_PORT = 18081
UPSTREAM_PORT = 18082
PATCHES = {
    '/js/rm_web-lock.a77e3490-1741610866799.js': (
        (
            'baseURL:window.location.href.slice(0,window.location.href.indexOf('
            '"/#"))+":8090"'
        ),
        (
            'baseURL:window.location.protocol+"//"+window.location.hostname+":8'
            '090"'
        ),
    ),
    '/js/rm_web-index.0dcbf496-1741610866799.js': (
        '"ws"+localStorage.getItem("baseUrl").slice(4)+":8060"',
        (
            '(window.location.protocol==="https:"?"wss://":"ws://")+window.loca'
            'tion.hostname+":8060"'
        ),
    ),
}


class Handler(http.server.BaseHTTPRequestHandler):
    def do_GET(self):
        self.forward(False)

    def do_HEAD(self):
        self.forward(True)

    def forward(self, head):
        if not self.path.startswith('/') or self.path.startswith('//'):
            self.send_error(400)
            return
        connection = http.client.HTTPConnection(
            '127.0.0.1', UPSTREAM_PORT, timeout=15
        )
        try:
            # This listener only serves vendor web assets; credentials go
            # directly
            # through the SSH tunnel on 8090, never into proxy logs.
            connection.request(
                'GET',
                self.path,
                headers={'Host': '192.168.1.19', 'Accept-Encoding': 'identity'},
            )
            response = connection.getresponse()
            body = response.read()
            patch = PATCHES.get(self.path.split('?')[0])
            if patch and response.status == 200:
                before, after = (v.encode() for v in patch)
                if body.count(before) != 1:
                    self.send_error(
                        502,
                        'Vendor asset changed; gateway patch requires review',
                    )
                    return
                body = body.replace(before, after)
            self.send_response(response.status)
            for key, value in response.getheaders():
                if (
                    key.lower()
                    not in {
                        'content-length',
                        'transfer-encoding',
                        'connection',
                        'etag',
                        'last-modified',
                        'cache-control',
                        'server',
                        'date',
                    }
                ):
                    self.send_header(key, value)
            self.send_header('Content-Length', str(len(body)))
            self.send_header('Cache-Control', 'no-store')
            self.end_headers()
            if not head:
                self.wfile.write(body)
        except (OSError, http.client.HTTPException):
            self.send_error(502, 'Robot web tunnel unavailable')
        finally:
            connection.close()

    def log_message(self, *args):
        pass


def main():
    server = http.server.ThreadingHTTPServer(('127.0.0.1', WEB_PORT), Handler)
    command = [
        'ssh',
        '-N',
        '-T',
        '-o',
        'BatchMode=yes',
        '-o',
        'ExitOnForwardFailure=yes',
        '-o',
        'ServerAliveInterval=30',
        '-o',
        'ServerAliveCountMax=3',
    ]
    for local, remote in ((UPSTREAM_PORT, 80), (8090, 8090), (8060, 8060)):
        command += ['-L', f'127.0.0.1:{local}:192.168.1.19:{remote}']
    tunnel = subprocess.Popen(command + ['msi'])

    def stop(*_):
        raise KeyboardInterrupt

    signal.signal(signal.SIGTERM, stop)
    try:
        for _ in range(50):
            if tunnel.poll() is not None:
                raise RuntimeError(
                    'SSH tunnel could not start; check SSH and local ports'
                )
            try:
                with socket.create_connection(
                    ('127.0.0.1', UPSTREAM_PORT), timeout=.1
                ):
                    break
            except OSError:
                time.sleep(.1)
        else:
            raise RuntimeError('SSH tunnel startup timed out')

        def watch_tunnel():
            tunnel.wait()
            server.shutdown()

        threading.Thread(target=watch_tunnel, daemon=True).start()
        print(
            f'Open http://127.0.0.1:{WEB_PORT}/#/login ; Ctrl-C stops gateway.',
            flush=True,
        )
        server.serve_forever()
    except KeyboardInterrupt:
        pass
    finally:
        server.server_close()
        tunnel.terminate()
        tunnel.wait(timeout=5)


if __name__ == '__main__':
    main()
