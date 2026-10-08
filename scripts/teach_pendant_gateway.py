#!/usr/bin/env python3
"""经 msi 的 SSH 转发，在本机浏览器打开两台机械臂的示教器网页。

在运行浏览器的电脑上执行 `python3 scripts/teach_pendant_gateway.py`，按 Ctrl-C
停止。只用 Python 标准库和系统自带的 ssh，所有监听地址都是 127.0.0.1。

本脚本只负责把示教器网页搬到本机：转发网页资源，并改写其中两处拼接接口地址的
写法。它不修改控制器上的任何文件，不经手账号和口令，也不发送机械臂指令。
"""

from __future__ import annotations

import dataclasses
import http.client
import http.server
import logging
import signal
import socket
import subprocess
import sys
import threading
import time

LOG = logging.getLogger('teach_pendant_gateway')

SSH_HOST = 'msi'
# 控制器上三个服务的端口：网页与静态资源、登录及业务接口、实时通信。
CONTROLLER_WEB_PORT = 80
CONTROLLER_API_PORT = 8090
CONTROLLER_REALTIME_PORT = 8060

# 原厂前端里拼接地址的两处写法。它在网页地址后面直接接 ":8090" 和 ":8060"，
# 经端口转发打开时会拼出 http://127.0.0.1:18081:8090 这样的无效地址，登录请求
# 在校验口令之前就失败。两台机械臂的前端文件完全相同（2026-10-08 核对过摘要）。
API_ASSET_PATH = '/js/rm_web-lock.a77e3490-1741610866799.js'
API_ORIGINAL = (
    'baseURL:window.location.href.slice(0,window.location.href.indexOf('
    '"/#"))+":8090"'
)
REALTIME_ASSET_PATH = '/js/rm_web-index.0dcbf496-1741610866799.js'
REALTIME_ORIGINAL = '"ws"+localStorage.getItem("baseUrl").slice(4)+":8060"'

# 不原样转给浏览器的响应头：长度会因改写而变，其余是缓存和连接相关的头，
# 留着会让浏览器继续使用改写前的旧文件。
DROPPED_RESPONSE_HEADERS = frozenset({
    'content-length',
    'transfer-encoding',
    'connection',
    'etag',
    'last-modified',
    'cache-control',
    'server',
    'date',
})

UPSTREAM_TIMEOUT_S = 15
TUNNEL_START_TIMEOUT_S = 5
# 机械臂断电时浏览器会连续请求几十个文件，同一原因不必每次都记一行。
FAILURE_LOG_INTERVAL_S = 30


@dataclasses.dataclass(frozen=True)
class Arm:
    """一台机械臂的示教器在本机占用的端口。

    Attributes:
        name: 给操作者看的名称。
        controller_ip: 控制器在 msi 一侧的地址。
        web_port: 浏览器打开的本机端口，由本脚本监听。
        upstream_port: SSH 把控制器网页端口转发到的本机端口，仅供本脚本取
            原始文件，浏览器不直接访问。
        api_port: SSH 把登录及业务接口转发到的本机端口。
        realtime_port: SSH 把实时通信转发到的本机端口。
    """

    name: str
    controller_ip: str
    web_port: int
    upstream_port: int
    api_port: int
    realtime_port: int


# 机械臂1的三个对外端口保持最初的取值，已有的书签和文档继续有效。两台臂共用
# 127.0.0.1 这一个主机名，接口和实时通信端口必须错开，否则会登录到另一台臂上。
ARMS = (
    Arm(
        name='机械臂1',
        controller_ip='192.168.1.19',
        web_port=18081,
        upstream_port=18181,
        api_port=8090,
        realtime_port=8060,
    ),
    Arm(
        name='机械臂2',
        controller_ip='192.168.1.18',
        web_port=18082,
        upstream_port=18182,
        api_port=18090,
        realtime_port=18060,
    ),
)


class AssetChangedError(Exception):
    """原厂前端文件与本脚本适配的版本不同，不能按原规则改写。"""


def asset_replacements(arm: Arm) -> dict[str, tuple[bytes, bytes]]:
    """给出一台机械臂需要改写的前端文件及替换内容。

    Args:
        arm: 机械臂的端口安排。

    Returns:
        以文件路径为键，值为（原文，替换后的文字）。替换后接口地址由浏览器
        当前的协议、主机名加这台臂自己的本机端口组成。
    """
    api = (
        'baseURL:window.location.protocol+"//"+window.location.hostname+'
        f'":{arm.api_port}"'
    )
    realtime = (
        '(window.location.protocol==="https:"?"wss://":"ws://")+'
        f'window.location.hostname+":{arm.realtime_port}"'
    )
    return {
        API_ASSET_PATH: (API_ORIGINAL.encode(), api.encode()),
        REALTIME_ASSET_PATH: (REALTIME_ORIGINAL.encode(), realtime.encode()),
    }


def patch_asset(arm: Arm, path: str, body: bytes) -> bytes:
    """按需改写一个前端文件里拼接接口地址的写法。

    Args:
        arm: 机械臂的端口安排。
        path: 请求路径，不含查询参数。
        body: 控制器返回的原始内容。

    Returns:
        需要改写的文件返回改写后的内容，其余文件原样返回。

    Raises:
        AssetChangedError: 待替换的原文不是恰好出现一次，通常是控制器前端
            升级过，需要人工重新核对替换规则。
    """
    replacement = asset_replacements(arm).get(path)
    if replacement is None:
        return body
    original, patched = replacement
    count = body.count(original)
    if count != 1:
        raise AssetChangedError(
            f'{arm.name} 的 {path} 中待替换的原文出现 {count} 次，应为 1 次'
        )
    return body.replace(original, patched)


class GatewayServer(http.server.ThreadingHTTPServer):
    """一台机械臂的本机网页入口，只监听 127.0.0.1。

    Attributes:
        arm: 本入口对应的机械臂。
    """

    daemon_threads = True

    def __init__(self, arm: Arm) -> None:
        """绑定这台机械臂的入口端口。

        Args:
            arm: 机械臂的端口安排。

        Raises:
            OSError: 入口端口已被占用。
        """
        super().__init__(('127.0.0.1', arm.web_port), Handler)
        self.arm = arm
        self._last_failure_log_s = {}
        self._failure_log_lock = threading.Lock()

    def log_failure(self, reason: str, detail: str) -> None:
        """记录一次转发失败；同一原因在短时间内只记一次。

        Args:
            reason: 失败的类别，用于合并重复记录。
            detail: 给操作者看的说明。
        """
        now_s = time.monotonic()
        with self._failure_log_lock:
            last_s = self._last_failure_log_s.get(reason)
            if last_s is not None and now_s - last_s < FAILURE_LOG_INTERVAL_S:
                return
            self._last_failure_log_s[reason] = now_s
        LOG.warning('%s：%s', self.arm.name, detail)


class Handler(http.server.BaseHTTPRequestHandler):
    """把浏览器对网页资源的请求转给控制器，并改写两处接口地址。"""

    def do_GET(self) -> None:
        """处理 GET 请求。"""
        self._forward(send_body=True)

    def do_HEAD(self) -> None:
        """处理 HEAD 请求，只回响应头。"""
        self._forward(send_body=False)

    def log_message(self, format: str, *args: object) -> None:
        """不逐条记录访问；失败由 GatewayServer.log_failure 记录。"""

    def _forward(self, send_body: bool) -> None:
        if not self.path.startswith('/') or self.path.startswith('//'):
            self.send_error(400)
            return
        arm = self.server.arm
        try:
            status, headers, body = _fetch(arm, self.path)
            if status == 200:
                body = patch_asset(arm, self.path.split('?')[0], body)
        except AssetChangedError as error:
            self.server.log_failure('asset', f'{error}；已拒绝返回该文件')
            self.send_error(
                502, 'Vendor asset changed; gateway patch requires review'
            )
            return
        except (OSError, http.client.HTTPException) as error:
            self.server.log_failure(
                'upstream',
                f'取不到控制器 {arm.controller_ip} 的网页'
                f'（{type(error).__name__}）；请检查 SSH 连接、msi 到该控制器'
                '的网络以及机械臂是否通电',
            )
            self.send_error(502, 'Robot web tunnel unavailable')
            return
        self.send_response(status)
        for key, value in headers:
            if key.lower() not in DROPPED_RESPONSE_HEADERS:
                self.send_header(key, value)
        self.send_header('Content-Length', str(len(body)))
        self.send_header('Cache-Control', 'no-store')
        self.end_headers()
        if send_body:
            self.wfile.write(body)


def _fetch(arm: Arm, path: str) -> tuple[int, list[tuple[str, str]], bytes]:
    # 这里只取原厂的网页资源。账号和口令由浏览器经 SSH 转发直接发往接口端口，
    # 不经过本进程，也就不会进入本进程的日志。
    connection = http.client.HTTPConnection(
        '127.0.0.1', arm.upstream_port, timeout=UPSTREAM_TIMEOUT_S
    )
    try:
        connection.request(
            'GET',
            path,
            headers={
                'Host': arm.controller_ip,
                'Accept-Encoding': 'identity',
            },
        )
        response = connection.getresponse()
        return response.status, response.getheaders(), response.read()
    finally:
        connection.close()


def tunnel_command(arms: tuple[Arm, ...]) -> list[str]:
    """组出一条同时转发全部机械臂端口的 ssh 命令。

    Args:
        arms: 要转发的机械臂。

    Returns:
        ssh 的参数列表。不分配终端、不执行远端命令、不交互索取口令；任何一个
        本机端口绑定失败时 ssh 直接退出。
    """
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
    for arm in arms:
        forwards = (
            (arm.upstream_port, CONTROLLER_WEB_PORT),
            (arm.api_port, CONTROLLER_API_PORT),
            (arm.realtime_port, CONTROLLER_REALTIME_PORT),
        )
        for local_port, controller_port in forwards:
            command += [
                '-L',
                f'127.0.0.1:{local_port}:{arm.controller_ip}:{controller_port}',
            ]
    return command + [SSH_HOST]


def _local_ports(arms: tuple[Arm, ...]) -> list[int]:
    ports = []
    for arm in arms:
        ports += [
            arm.web_port,
            arm.upstream_port,
            arm.api_port,
            arm.realtime_port,
        ]
    return ports


def _wait_for_tunnel(tunnel: subprocess.Popen, arms: tuple[Arm, ...]) -> None:
    # ssh 一建立连接就开始监听本机端口，与控制器是否在线无关，所以这里只能
    # 确认 SSH 本身可用；某台臂断电要到浏览器访问时才会表现为 502。
    deadline_s = time.monotonic() + TUNNEL_START_TIMEOUT_S
    pending = [arm.upstream_port for arm in arms]
    while pending:
        if tunnel.poll() is not None:
            raise RuntimeError(
                f'SSH 转发启动失败（退出码 {tunnel.returncode}）。请先确认 '
                f'`ssh -o BatchMode=yes {SSH_HOST} true` 可用，且本机端口 '
                f'{_local_ports(arms)} 没有被占用（旧的入口可能还在运行）'
            )
        if time.monotonic() > deadline_s:
            raise RuntimeError(
                f'SSH 转发在 {TUNNEL_START_TIMEOUT_S} 秒内没有就绪'
            )
        try:
            with socket.create_connection(
                ('127.0.0.1', pending[0]), timeout=.1
            ):
                pending.pop(0)
        except OSError:
            time.sleep(.1)


def _open_servers(arms: tuple[Arm, ...]) -> list[GatewayServer]:
    servers = []
    try:
        for arm in arms:
            servers.append(GatewayServer(arm))
    except OSError as error:
        for server in servers:
            server.server_close()
        raise RuntimeError(
            f'本机端口 {arm.web_port}（{arm.name} 的网页入口）无法监听：'
            f'{error}。旧的入口可能还在运行'
        ) from error
    return servers


def _raise_keyboard_interrupt(signum: int, frame: object) -> None:
    raise KeyboardInterrupt


def _serve_until_tunnel_exits(
    servers: list[GatewayServer],
    serving: list[GatewayServer],
    tunnel: subprocess.Popen,
) -> int:
    # serving 由调用方传入并在这里填写，这样即使中途被 Ctrl-C 打断，调用方
    # 也知道哪些入口已经开始服务、需要停止。
    for server in servers:
        threading.Thread(target=server.serve_forever, daemon=True).start()
        serving.append(server)
        LOG.info(
            '%s（%s）示教器：http://127.0.0.1:%d/#/login',
            server.arm.name,
            server.arm.controller_ip,
            server.arm.web_port,
        )
    LOG.info('按 Ctrl-C 停止全部入口和 SSH 转发')
    returncode = tunnel.wait()
    LOG.error('SSH 连接已断开（退出码 %d），入口停止', returncode)
    return 1


def main() -> int:
    """启动全部机械臂的示教器入口，直到 Ctrl-C 或 SSH 连接断开。

    Returns:
        进程退出码：正常停止为 0，启动失败或 SSH 连接意外断开为 1。
    """
    logging.basicConfig(
        level=logging.INFO,
        format='%(asctime)s %(levelname)s %(message)s',
        datefmt='%H:%M:%S',
    )
    servers = []
    serving = []
    tunnel = None
    signal.signal(signal.SIGTERM, _raise_keyboard_interrupt)
    try:
        servers = _open_servers(ARMS)
        tunnel = subprocess.Popen(tunnel_command(ARMS))
        _wait_for_tunnel(tunnel, ARMS)
        return _serve_until_tunnel_exits(servers, serving, tunnel)
    except RuntimeError as error:
        LOG.error('%s', error)
        return 1
    except KeyboardInterrupt:
        LOG.info('已收到停止请求')
        return 0
    finally:
        # shutdown() 会一直等到 serve_forever 退出，对尚未开始服务的入口调用
        # 它会永远卡住，所以只对已经开始服务的入口调用。
        for server in serving:
            server.shutdown()
        for server in servers:
            server.server_close()
        if tunnel is not None and tunnel.poll() is None:
            tunnel.terminate()
            tunnel.wait(timeout=5)


if __name__ == '__main__':
    sys.exit(main())
