import asyncio
import copy
import json
import unittest
from unittest import mock
from saydo.task_runner import config
from saydo.task_runner import robot_adapter
from saydo.task_runner import runner
from saydo.task_runner import tool_registry


class ContextTests(unittest.IsolatedAsyncioTestCase):
    async def test_batch_only_reads_and_returns_image_for_existing_pipeline(
        self
    ):
        adapter = robot_adapter.RobotAdapter(config.Config(api_key='test'))
        adapter.request = mock.AsyncMock(return_value={'ok': True})
        args = {
            'object_ids': ['tube', 'rack'],
            'include_profiles': True,
            'include_robot': True,
            'include_frame': True,
            'scene_id': 'scene',
        }
        tool_registry.validate(
            {
                'type': 'function_call',
                'call_id': 'c',
                'name': 'get_task_context',
                'arguments': json.dumps(dict(args, progress_message='读取')),
            },
            'read_only',
        )
        result = await adapter.execute(
            'get_task_context', args, 'r', asyncio.Event(), lambda _: None
        )
        self.assertEqual(len(result), 7)
        self.assertIn('candidate_frame', result)
        self.assertTrue(
            all(
                c.args[0].startswith('/api/agent/') and len(c.args) == 1
                for c in adapter.request.call_args_list
            )
        )

    def test_latest_image_only_preserves_text_evidence_and_original_result(
        self
    ):
        history = []
        for i in range(3):
            frame = {
                'image_base64': str(i),
                'mime_type': 'image/jpeg',
                'captured_at': i,
            }
            runner.Runner.append_result(
                history,
                str(i),
                'get_task_context',
                {'candidate_frame': frame, 'robot': {'pose': i}},
            )
            self.assertEqual(frame['image_base64'], str(i))
        images = [
            p
            for item in history
            if isinstance(item.get('content'), list)
            for p in item['content']
            if p['type'] == 'input_image'
        ]
        self.assertEqual(len(images), 1)
        self.assertTrue(images[0]['image_url'].endswith(',2'))
        self.assertEqual(
            sum(x.get('type') == 'function_call_output' for x in history), 3
        )

    def test_plan_and_feature_compaction_preserves_goal_and_all_ids(self):
        plan = {
            'plan_id': 'p',
            'goal': {'reference_point_object_m': [0, 0, -.075]},
            'waypoints': [{'x': 1}],
            'validation_scope': {'environment_collision_checked': False},
        }
        value = {
            'task': {'result': {'object_motion_plan': plan}},
            'status': {'grasp': {'object_motion_plan': plan}},
            'features': [
                {
                    'id': str(i),
                    'object_T_feature': [
                        [1, 0, 0, i], [0, 1, 0, 0], [0, 0, 1, 0], [0, 0, 0, 1]
                    ],
                }
                for i in range(120)
            ],
        }
        before = copy.deepcopy(value)
        result = robot_adapter.model_result('plan_held_object_motion', value)
        self.assertEqual(value, before)
        self.assertNotIn('object_motion_plan', result['status']['grasp'])
        p = result['task']['result']['object_motion_plan']
        self.assertNotIn('waypoints', p)
        self.assertEqual(p['goal'], plan['goal'])
        self.assertEqual(len(result['features']), 120)
        self.assertEqual(
            result['features'][-1]['position_object_m'], [119, 0, 0]
        )
        self.assertEqual(
            robot_adapter.model_result('get_task_diagnostics', value), value
        )
