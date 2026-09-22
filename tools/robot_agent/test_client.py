import io
import json
from pathlib import Path
import tempfile
import unittest
from unittest.mock import Mock
import urllib.error

from client import BackendError, RobotBackend


class ClientTests(unittest.TestCase):
    def setUp(self):
        self.directory=tempfile.TemporaryDirectory();self.addCleanup(self.directory.cleanup)
        self.token=Path(self.directory.name)/'token';self.token.write_text('a'*40);self.token.chmod(0o600)
        self.client=RobotBackend('http://127.0.0.1:8089',self.token)

    def test_uncertain_submission_returns_same_task_id_without_retry(self):
        self.client.opener.open=Mock(side_effect=TimeoutError('timeout'))
        with self.assertRaises(BackendError) as context:self.client.action('grasp','request-001',{'object_id':'bottle'})
        self.assertEqual(context.exception.payload['code'],'REQUEST_UNCERTAIN')
        self.assertEqual(context.exception.payload['task_id'],'request-001')
        self.client.opener.open.assert_called_once()

    def test_authentication_error_is_structured_and_not_retried(self):
        payload={'error':{'code':'UNAUTHORIZED','message':'bad token'}}
        self.client.opener.open=Mock(side_effect=urllib.error.HTTPError('http://127.0.0.1:8089',401,'Unauthorized',{},io.BytesIO(json.dumps(payload).encode())))
        with self.assertRaises(BackendError) as context:self.client.request('/api/agent/status')
        self.assertEqual(context.exception.payload['code'],'UNAUTHORIZED')
        self.client.opener.open.assert_called_once()

    def test_credentials_are_header_only_and_response_is_decoded(self):
        response=Mock();response.read.return_value=b'{"ok":true}'
        context=Mock();context.__enter__=Mock(return_value=response);context.__exit__=Mock(return_value=False)
        self.client.opener.open=Mock(return_value=context)
        self.assertTrue(self.client.request('/api/agent/status')['ok'])
        request=self.client.opener.open.call_args.args[0]
        self.assertEqual(request.get_header('Authorization'),'Bearer '+'a'*40)
        self.assertNotIn('a'*40,request.full_url)

    def test_private_token_and_restricted_routes(self):
        with self.assertRaises(ValueError):self.client.request('/api/grasp')
        self.token.chmod(0o644)
        with self.assertRaises(ValueError):self.client.request('/api/agent/status')
        with self.assertRaises(ValueError):RobotBackend('http://name:password@host',self.token)


if __name__=='__main__':unittest.main()
