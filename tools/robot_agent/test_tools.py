"""Real MCP SDK protocol/serialization against a simulated backend."""
import base64
import json
import unittest

from mcp import Client
from client import BackendError
from server import create_server


class FakeBackend:
    def __init__(self):self.calls=[];self.fail=False
    def request(self,path,value=None):
        self.calls.append((path,value))
        if self.fail:raise BackendError({'code':'REQUEST_UNCERTAIN','task_id':'fixed-id','message':'query original task'})
        if path.endswith(('/frame','/selection-frame')):return {'image_base64':base64.b64encode(b'image-fixture').decode(),'mime_type':'image/jpeg','captured_at':123.,'sequence':4}
        return {'path':path,'value':value,'task_id':(value or {}).get('request_id'),'terminal':False}


class ToolsTests(unittest.IsolatedAsyncioTestCase):
    async def test_complete_tool_catalog_and_annotations(self):
        async with Client(create_server(FakeBackend())) as client:
            tools=await client.list_tools()
            tools=tools.tools if hasattr(tools,'tools') else tools
            names={t.name:t for t in tools}
            self.assertEqual(len(names),19)
            for name in ['get_camera_frame','get_tracking_candidates_frame','get_object_pose','get_robot_state','get_task','preview_grasp','preview_place','preview_reorient']:
                self.assertTrue(names[name].annotations.read_only_hint)
            for name in ['grasp','place','reorient','reset_arm','stop_robot']:
                self.assertFalse(names[name].annotations.read_only_hint)

    async def test_image_block_and_metadata_are_native_mcp_content(self):
        async with Client(create_server(FakeBackend())) as client:
            result=await client.call_tool('get_camera_frame',{})
            self.assertFalse(result.is_error)
            self.assertEqual(result.content[0].type,'image')
            self.assertEqual(result.content[0].mime_type,'image/jpeg')
            self.assertEqual(result.structured_content['sequence'],4)
            self.assertNotIn('image_base64',result.structured_content)

    async def test_frozen_candidates_image_uses_separate_route(self):
        backend=FakeBackend()
        async with Client(create_server(backend)) as client:
            result=await client.call_tool('get_tracking_candidates_frame',{})
            self.assertFalse(result.is_error);self.assertEqual(result.content[0].type,'image')
            self.assertEqual(backend.calls[-1][0],'/api/agent/selection-frame')

    async def test_motion_arguments_ids_and_stop_route(self):
        backend=FakeBackend()
        async with Client(create_server(backend)) as client:
            for name,args,operation in [('grasp',{'object_id':'bottle','request_id':'fixed-id'},'grasp'),
                ('place',{'hold_id':'h','orientation_version':2,'placement_revision':'v','request_id':'place-id'},'place'),
                ('reset_arm',{'target':'zero','request_id':'reset-id'},'reset')]:
                result=await client.call_tool(name,args);self.assertFalse(result.is_error)
                path,payload=backend.calls[-1];self.assertEqual(path,'/api/agent/tasks')
                self.assertEqual(payload['operation'],operation);self.assertEqual(payload['request_id'],args['request_id'])
            await client.call_tool('stop_robot',{})
            self.assertEqual(backend.calls[-1],('/api/agent/stop',{}))
            self.assertFalse(any(path=='/api/stop' for path,_ in backend.calls))

    async def test_errors_do_not_retry_or_claim_completion(self):
        backend=FakeBackend();backend.fail=True
        async with Client(create_server(backend)) as client:
            result=await client.call_tool('grasp',{'object_id':'bottle','request_id':'fixed-id'})
            self.assertTrue(result.is_error);self.assertEqual(len(backend.calls),1)
            self.assertEqual(json.loads(result.content[0].text)['task_id'],'fixed-id')

    async def test_invalid_reset_target_never_reaches_backend(self):
        backend=FakeBackend()
        async with Client(create_server(backend)) as client:
            result=await client.call_tool('reset_arm',{'target':'arbitrary','request_id':'fixed-id'})
            self.assertTrue(result.is_error);self.assertEqual(backend.calls,[])

    async def test_reorientation_contract_and_strict_parameters(self):
        backend=FakeBackend()
        args={'object_id':'bottle','hold_id':'h','orientation_version':2,'target_rpy_deg':[0,0,90],'position_mode':'adaptive','request_id':'reorient-001'}
        async with Client(create_server(backend)) as client:
            for name in ('preview_reorient','reorient'):
                result=await client.call_tool(name,args)
                self.assertFalse(result.is_error)
                self.assertEqual(backend.calls[-1][1],{'operation':name,'request_id':args['request_id'],'arguments':{k:v for k,v in args.items() if k!='request_id'}})
            count=len(backend.calls)
            for override in ({'target_rpy_deg':[0,0]}, {'target_rpy_deg':[0,0,181]}, {'target_rpy_deg':[0,True,0]}, {'orientation_version':True}, {'position_mode':'anywhere'}):
                result=await client.call_tool('reorient',{**args,**override})
                self.assertTrue(result.is_error)
            self.assertEqual(len(backend.calls),count)


if __name__=='__main__':unittest.main()
