"""示教器本机入口的测试：用本机假控制器代替真实控制器，不建立 SSH 连接。"""

import http.client
import http.server
import threading
import unittest
from unittest import mock

from scripts import teach_pendant_gateway as gateway

PLAIN_PATH = '/index.html'
PLAIN_BODY = b'<title>pendant</title>'
VENDOR_API_ASSET = b'x={' + gateway.API_ORIGINAL.encode() + b',timeout:9}'
VENDOR_REALTIME_ASSET = (
    b'new WebSocket(' + gateway.REALTIME_ORIGINAL.encode() + b')'
)


class FakeController(http.server.ThreadingHTTPServer):
    """系统边界上的替身：扮演控制器 80 端口，按路径返回预设内容。"""

    daemon_threads = True

    def __init__(self):
        super().__init__(('127.0.0.1', 0), _FakeControllerHandler)
        self.assets = {
            PLAIN_PATH: PLAIN_BODY,
            gateway.API_ASSET_PATH: VENDOR_API_ASSET,
            gateway.REALTIME_ASSET_PATH: VENDOR_REALTIME_ASSET,
        }
        self.host_headers = []


class _FakeControllerHandler(http.server.BaseHTTPRequestHandler):
    def do_GET(self):
        self.server.host_headers.append(self.headers.get('Host'))
        body = self.server.assets.get(self.path.split('?')[0])
        if body is None:
            self.send_error(404)
            return
        self.send_response(200)
        self.send_header('Content-Type', 'application/javascript')
        self.send_header('ETag', '"vendor"')
        self.send_header('Cache-Control', 'max-age=3600')
        self.send_header('Content-Length', str(len(body)))
        self.end_headers()
        self.wfile.write(body)

    def log_message(self, format, *args):
        pass


def start(server):
    threading.Thread(target=server.serve_forever, daemon=True).start()
    return server


class PortPlanTests(unittest.TestCase):
    def test_second_arm_page_is_on_18082_and_first_arm_ports_are_unchanged(
        self,
    ):
        first, second = gateway.ARMS
        self.assertEqual(
            (first.controller_ip, first.web_port), ('192.168.1.19', 18081)
        )
        self.assertEqual((first.api_port, first.realtime_port), (8090, 8060))
        self.assertEqual(
            (second.controller_ip, second.web_port), ('192.168.1.18', 18082)
        )

    def test_no_local_port_is_shared_between_arms(self):
        ports = gateway._local_ports(gateway.ARMS)
        self.assertEqual(len(ports), 4 * len(gateway.ARMS))
        self.assertEqual(len(set(ports)), len(ports))

    def test_tunnel_forwards_three_controller_ports_per_arm(self):
        command = gateway.tunnel_command(gateway.ARMS)
        forwards = [
            command[i + 1] for i, word in enumerate(command) if word == '-L'
        ]
        self.assertEqual(
            forwards,
            [
                '127.0.0.1:18181:192.168.1.19:80',
                '127.0.0.1:8090:192.168.1.19:8090',
                '127.0.0.1:8060:192.168.1.19:8060',
                '127.0.0.1:18182:192.168.1.18:80',
                '127.0.0.1:18090:192.168.1.18:8090',
                '127.0.0.1:18060:192.168.1.18:8060',
            ],
        )
        self.assertEqual(command[-1], 'msi')
        self.assertIn('BatchMode=yes', command)
        self.assertIn('ExitOnForwardFailure=yes', command)


class PatchAssetTests(unittest.TestCase):
    def test_each_arm_gets_its_own_api_and_realtime_ports(self):
        first, second = gateway.ARMS
        patched = gateway.patch_asset(
            first, gateway.API_ASSET_PATH, VENDOR_API_ASSET
        )
        self.assertIn(b'window.location.hostname+":8090"', patched)
        self.assertIn(
            b'window.location.hostname+":8060"',
            gateway.patch_asset(
                first, gateway.REALTIME_ASSET_PATH, VENDOR_REALTIME_ASSET
            ),
        )
        patched = gateway.patch_asset(
            second, gateway.API_ASSET_PATH, VENDOR_API_ASSET
        )
        self.assertIn(b'window.location.hostname+":18090"', patched)
        self.assertNotIn(b':8090', patched)
        self.assertNotIn(gateway.API_ORIGINAL.encode(), patched)
        patched = gateway.patch_asset(
            second, gateway.REALTIME_ASSET_PATH, VENDOR_REALTIME_ASSET
        )
        self.assertIn(b'window.location.hostname+":18060"', patched)
        self.assertNotIn(b':8060"', patched)

    def test_other_files_are_returned_untouched(self):
        body = gateway.API_ORIGINAL.encode()
        self.assertIs(
            gateway.patch_asset(gateway.ARMS[1], '/js/other.js', body), body
        )

    def test_missing_or_repeated_target_is_refused_with_the_count(self):
        arm = gateway.ARMS[1]
        for body, count in ((b'upgraded', 0), (VENDOR_API_ASSET * 2, 2)):
            with self.subTest(count=count):
                with self.assertRaisesRegex(
                    gateway.AssetChangedError, f'出现 {count} 次'
                ):
                    gateway.patch_asset(arm, gateway.API_ASSET_PATH, body)


class GatewayRequestTests(unittest.TestCase):
    """浏览器 → 本机入口 → 假控制器，走真实的转发与改写代码。"""

    def setUp(self):
        self.controller = start(FakeController())
        self.addCleanup(self.controller.server_close)
        self.addCleanup(self.controller.shutdown)
        self.arm = gateway.Arm(
            name='机械臂2',
            controller_ip='192.168.1.18',
            web_port=0,
            upstream_port=self.controller.server_address[1],
            api_port=18090,
            realtime_port=18060,
        )
        self.server = start(gateway.GatewayServer(self.arm))
        self.addCleanup(self.server.server_close)
        self.addCleanup(self.server.shutdown)

    def request(self, method, path):
        connection = http.client.HTTPConnection(
            '127.0.0.1', self.server.server_address[1], timeout=5
        )
        self.addCleanup(connection.close)
        connection.request(method, path)
        response = connection.getresponse()
        return response, response.read()

    def test_asset_is_fetched_from_this_arm_and_patched_with_its_ports(self):
        response, body = self.request('GET', gateway.API_ASSET_PATH + '?v=1')
        self.assertEqual(response.status, 200)
        self.assertIn(b'window.location.hostname+":18090"', body)
        self.assertEqual(self.controller.host_headers, ['192.168.1.18'])
        self.assertEqual(response.getheader('Content-Length'), str(len(body)))
        self.assertEqual(response.getheader('Cache-Control'), 'no-store')
        self.assertIsNone(response.getheader('ETag'))
        self.assertEqual(
            response.getheader('Content-Type'), 'application/javascript'
        )

    def test_plain_file_passes_through_and_head_has_no_body(self):
        response, body = self.request('GET', PLAIN_PATH)
        self.assertEqual((response.status, body), (200, PLAIN_BODY))
        response, body = self.request('HEAD', PLAIN_PATH)
        self.assertEqual((response.status, body), (200, b''))
        self.assertEqual(
            response.getheader('Content-Length'), str(len(PLAIN_BODY))
        )

    def test_controller_status_is_relayed(self):
        response, _ = self.request('GET', '/missing.js')
        self.assertEqual(response.status, 404)

    def test_changed_vendor_asset_is_refused_and_logged_once(self):
        self.controller.assets[gateway.API_ASSET_PATH] = b'upgraded frontend'
        with self.assertLogs(gateway.LOG, 'WARNING') as logs:
            for _ in range(2):
                response, _ = self.request('GET', gateway.API_ASSET_PATH)
                self.assertEqual(response.status, 502)
        self.assertEqual(len(logs.output), 1)
        self.assertIn('机械臂2', logs.output[0])
        self.assertIn('出现 0 次', logs.output[0])

    def test_unreachable_controller_gives_502_and_names_the_arm(self):
        self.controller.shutdown()
        self.controller.server_close()
        with self.assertLogs(gateway.LOG, 'WARNING') as logs:
            response, _ = self.request('GET', PLAIN_PATH)
        self.assertEqual(response.status, 502)
        self.assertIn('192.168.1.18', logs.output[0])

    def test_double_slash_path_never_leaves_this_arm(self):
        # 较新的 Python 会先把开头的多个斜杠并成一个，请求按普通路径转给本臂
        # 控制器；较旧的版本由入口直接拒绝。两种情况都不会访问其他主机。
        response, _ = self.request('GET', '//example.com/x')
        self.assertIn(response.status, (400, 404))
        self.assertIn(self.controller.host_headers, ([], ['192.168.1.18']))


class StartupFailureTests(unittest.TestCase):
    def test_busy_entry_port_is_reported_without_starting_ssh(self):
        blocker = gateway.GatewayServer(
            gateway.Arm('占用者', '192.168.1.19', 0, 1, 2, 3)
        )
        self.addCleanup(blocker.server_close)
        busy = gateway.Arm(
            '机械臂2', '192.168.1.18', blocker.server_address[1], 4, 5, 6
        )
        with mock.patch.object(gateway, 'ARMS', (busy,)), mock.patch.object(
            gateway.subprocess, 'Popen'
        ) as popen, self.assertLogs(gateway.LOG, 'ERROR') as logs:
            self.assertEqual(gateway.main(), 1)
        popen.assert_not_called()
        self.assertIn(str(blocker.server_address[1]), logs.output[0])
        self.assertIn('机械臂2', logs.output[0])

    def test_ssh_exiting_at_startup_is_reported_and_entries_are_closed(self):
        tunnel = mock.Mock()
        tunnel.poll.return_value = 255
        tunnel.returncode = 255
        arm = gateway.Arm('机械臂2', '192.168.1.18', 0, 1, 2, 3)
        with mock.patch.object(gateway, 'ARMS', (arm,)), mock.patch.object(
            gateway.subprocess, 'Popen', return_value=tunnel
        ), self.assertLogs(gateway.LOG, 'ERROR') as logs:
            self.assertEqual(gateway.main(), 1)
        self.assertIn('退出码 255', logs.output[0])
        tunnel.terminate.assert_not_called()


if __name__ == '__main__':
    unittest.main()
