import io
import json
import pathlib
import tempfile
import unittest
from unittest import mock
import urllib.error

from saydo.agent import client


class ClientTests(unittest.TestCase):
    def setUp(self):
        self.directory = tempfile.TemporaryDirectory()
        self.addCleanup(self.directory.cleanup)
        self.token = pathlib.Path(self.directory.name) / 'token'
        self.token.write_text('a' * 40)
        self.token.chmod(0o600)
        self.client = client.RobotBackend('http://127.0.0.1:8089', self.token)

    def test_uncertain_submission_returns_same_task_id_without_retry(self):
        self.client.opener.open = mock.Mock(side_effect=TimeoutError('timeout'))
        with self.assertRaises(client.BackendError) as context:
            self.client.action('grasp', 'request-001', {'object_id': 'bottle'})
        self.assertEqual(context.exception.payload['code'], 'REQUEST_UNCERTAIN')
        self.assertEqual(context.exception.payload['task_id'], 'request-001')
        self.client.opener.open.assert_called_once()

    def test_authentication_error_is_structured_and_not_retried(self):
        payload = {'error': {'code': 'UNAUTHORIZED', 'message': 'bad token'}}
        self.client.opener.open = mock.Mock(
            side_effect=urllib.error.HTTPError(
                'http://127.0.0.1:8089',
                401,
                'Unauthorized',
                {},
                io.BytesIO(json.dumps(payload).encode()),
            )
        )
        with self.assertRaises(client.BackendError) as context:
            self.client.request('/api/agent/status')
        self.assertEqual(context.exception.payload['code'], 'UNAUTHORIZED')
        self.client.opener.open.assert_called_once()

    def test_credentials_are_header_only_and_response_is_decoded(self):
        response = mock.Mock()
        response.read.return_value = b'{"ok":true}'
        context = mock.Mock()
        context.__enter__ = mock.Mock(return_value=response)
        context.__exit__ = mock.Mock(return_value=False)
        self.client.opener.open = mock.Mock(return_value=context)
        self.assertTrue(self.client.request('/api/agent/status')['ok'])
        request = self.client.opener.open.call_args.args[0]
        self.assertEqual(
            request.get_header('Authorization'), 'Bearer ' + 'a' * 40
        )
        self.assertNotIn('a' * 40, request.full_url)

    def sent(self, backend, path, value=None):
        response = mock.Mock()
        response.read.return_value = b'{}'
        context = mock.Mock()
        context.__enter__ = mock.Mock(return_value=response)
        context.__exit__ = mock.Mock(return_value=False)
        backend.opener.open = mock.Mock(return_value=context)
        backend.request(path, value)
        request = backend.opener.open.call_args.args[0]
        body = None if request.data is None else json.loads(request.data)
        return request.full_url.split('8089', 1)[1], body

    def test_client_for_one_arm_names_it_in_every_request(self):
        second = client.RobotBackend(
            'http://127.0.0.1:8089', self.token, arm='arm2'
        )
        self.assertEqual(
            self.sent(second, '/api/agent/status'),
            ('/api/agent/status?arm=arm2', None),
        )
        self.assertEqual(
            self.sent(second, '/api/agent/object-geometry?object_id=tube')[0],
            '/api/agent/object-geometry?object_id=tube&arm=arm2',
        )
        task = {
            'operation': 'reset', 'request_id': 'r-0000001', 'arguments': {}
        }
        self.assertEqual(
            self.sent(second, '/api/agent/tasks', task),
            ('/api/agent/tasks', {**task, 'arm': 'arm2'}),
        )
        # 传入的字典不被改动，重发时内容不变。
        self.assertNotIn('arm', task)

    def test_stop_is_never_narrowed_to_one_arm(self):
        second = client.RobotBackend(
            'http://127.0.0.1:8089', self.token, arm='arm2'
        )
        self.assertEqual(
            self.sent(second, '/api/agent/stop', {}), ('/api/agent/stop', {})
        )

    def test_client_without_an_arm_sends_requests_unchanged(self):
        self.assertEqual(
            self.sent(self.client, '/api/agent/status'),
            ('/api/agent/status', None),
        )
        self.assertEqual(
            self.sent(self.client, '/api/agent/vision', {'operation': 'x'})[1],
            {'operation': 'x'},
        )

    def test_invalid_arm_name_is_refused(self):
        for arm in ('arm2&x=1', 'Arm 2', ''):
            with self.subTest(arm=arm), self.assertRaises(ValueError):
                client.RobotBackend(
                    'http://127.0.0.1:8089', self.token, arm=arm
                )

    def test_private_token_and_restricted_routes(self):
        with self.assertRaises(ValueError):
            self.client.request('/api/grasp')
        self.token.chmod(0o644)
        with self.assertRaises(ValueError):
            self.client.request('/api/agent/status')
        with self.assertRaises(ValueError):
            client.RobotBackend('http://name:password@host', self.token)


if __name__ == '__main__':
    unittest.main()
