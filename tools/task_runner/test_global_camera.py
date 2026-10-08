import asyncio
import json
import unittest
from unittest.mock import Mock, patch
from tools.robot_agent.global_camera import CameraUnavailable
from tools.robot_agent.client import BackendError
from .config import Config
from .robot_adapter import RobotAdapter, Cancelled
from .runner import Runner
from .tool_registry import schemas, validate


class GlobalObservationTests(unittest.IsolatedAsyncioTestCase):
    async def test_all_modes_and_no_robot_http_requests(self):
        for mode in ('read_only','preview','execute'):
            self.assertIn('observe_global',{s['name'] for s in schemas(mode)})
            validate({'type':'function_call','call_id':'global-1','name':'observe_global',
                      'arguments':json.dumps({'progress_message':'观察全局'})},mode)
        backend=Mock();adapter=RobotAdapter(Config(api_key='test'),backend)
        frame={'camera_role':'movable_global','image_base64':'AA==','mime_type':'image/jpeg','captured_at':123}
        with patch('tools.task_runner.robot_adapter.observe_global',return_value=frame):
            result=await adapter.execute('observe_global',{},'r',asyncio.Event(),lambda _:None)
        self.assertEqual(result['image_base64'],frame['image_base64']);self.assertEqual(result['camera_label'],'不固定的全局相机');backend.request.assert_not_called()
        history=[];cleaned=Runner.append_result(history,'global-1','observe_global',result)
        self.assertNotIn('image_base64',cleaned)
        self.assertEqual(history[-1]['content'][-1]['type'],'input_image')
        self.assertIn('movable_global',history[-1]['content'][0]['text'])

    async def test_failure_is_read_failure_and_cancelled_result_is_not_delivered(self):
        adapter=RobotAdapter(Config(api_key='test'),Mock());cancelled=asyncio.Event()
        with patch('tools.task_runner.robot_adapter.observe_global',side_effect=CameraUnavailable('missing')):
            with self.assertRaises(BackendError) as error:
                await adapter.execute('observe_global',{},'r',cancelled,lambda _:None)
        self.assertEqual(error.exception.payload['code'],'CAMERA_UNAVAILABLE')
        cancelled.set()
        with patch('tools.task_runner.robot_adapter.observe_global') as capture:
            with self.assertRaises(Cancelled):await adapter.execute('observe_global',{},'r',cancelled,lambda _:None)
            capture.assert_not_called()
        adapter.backend.request.assert_not_called()
