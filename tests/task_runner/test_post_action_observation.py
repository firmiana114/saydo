import asyncio
import json
import time
import unittest
from tests.task_runner import test_runner as base
from saydo.task_runner import runner
from saydo.task_runner import observations
from saydo.agent import client


class ObservationFlowTests(unittest.IsolatedAsyncioTestCase):
    asyncSetUp = base.RunnerTests.asyncSetUp
    asyncTearDown = base.RunnerTests.asyncTearDown
    make = base.RunnerTests.make
    execute = base.RunnerTests.execute

    async def test_executor_takes_global_image_after_each_action_and_logs_origin(
        self
    ):
        llm = base.FakeLLM(
            base.call('grasp', {'object_id': 'bottle'}, 'g'),
            base.call(
                'place',
                {
                    'hold_id': 'h',
                    'orientation_version': 0,
                    'placement_revision': 'v',
                },
                'p',
            ),
            base.end('succeeded', ['g', 'p']),
        )
        run, _ = await self.execute(llm)
        self.assertEqual(run['state'], 'succeeded')
        self.assertEqual(self.adapter.global_requests, 2)
        records = [r for r in run['calls'] if r['name'] == 'observe_global']
        self.assertEqual([r['after_action'] for r in records], ['g', 'p'])
        self.assertTrue(all(r['initiated_by'] == 'executor' for r in records))
        self.assertEqual(len(run['global_observations']), 2)
        self.assertNotIn('image_base64', json.dumps(run))
        for history in llm.histories[1:]:
            images = [
                p
                for item in history
                for p in item.get('content', [])
                if isinstance(p, dict) and p.get('type') == 'input_image'
            ]
            self.assertEqual(len(images), 1)
            self.assertIn('movable_global', json.dumps(history))
        # Program observations are explicitly labelled user-context data, not
        # fabricated model function calls or orphan function_call_outputs.
        self.assertFalse(
            any(
                (
                    i.get('type') == 'function_call_output'
                    and i.get('call_id', '').startswith('auto-global-')
                )
                for i in llm.histories[-1]
            )
        )

    async def test_missing_global_image_blocks_next_motion_without_replaying_grasp(
        self
    ):
        self.adapter.global_error = client.BackendError(
            {'code': 'CAMERA_UNAVAILABLE', 'message': 'camera disconnected'}
        )
        llm = base.FakeLLM(
            base.call('grasp', {'object_id': 'bottle'}, 'g'),
            base.call(
                'place',
                {
                    'hold_id': 'h',
                    'orientation_version': 0,
                    'placement_revision': 'v',
                },
                'p',
            ),
        )
        run, _ = await self.execute(llm)
        self.assertEqual(run['state'], 'blocked')
        self.assertEqual([r[0] for r in self.adapter.sent], ['grasp'])
        self.assertEqual(self.adapter.stops, 0)

    async def test_model_can_repair_failed_automatic_observation_before_finishing(
        self
    ):
        original = self.adapter.execute

        async def execute(name, *args):
            if name == 'observe_global' and self.adapter.global_requests == 0:
                self.adapter.global_requests += 1
                raise client.BackendError(
                    {'code': 'CAMERA_UNAVAILABLE', 'message': 'temporary'}
                )
            return await original(name, *args)

        self.adapter.execute = execute
        final = base.end('succeeded', ['g'])
        final['output'][0]['call_id'] = 'finish-second'
        llm = base.FakeLLM(
            base.call('grasp', {'object_id': 'bottle'}, 'g'),
            base.end('succeeded', ['g']),
            base.call('observe_global', {}, 'o'),
            final,
        )
        run, _ = await self.execute(llm)
        self.assertEqual(run['state'], 'succeeded')
        self.assertEqual(
            run['global_observations'][-1]['initiated_by'], 'model'
        )
        self.assertEqual([r[0] for r in self.adapter.sent], ['grasp'])

    async def test_stale_or_wrong_camera_observation_cannot_unlock_completion(
        self
    ):
        original = self.adapter.execute

        async def execute(name, *args):
            if name == 'observe_global':
                return {
                    'camera_role': 'eye_in_hand',
                    'camera_serial': 'wrist',
                    'captured_at': time.time(),
                    'image_base64': 'AA==',
                    'mime_type': 'image/png',
                }
            return await original(name, *args)

        self.adapter.execute = execute
        llm = base.FakeLLM(
            base.call('grasp', {'object_id': 'bottle'}, 'g'),
            base.end('blocked', ['g']),
        )
        run, _ = await self.execute(llm)
        self.assertEqual(run['state'], 'blocked')
        self.assertNotIn('global_observations', run)
        frame = {
            'camera_role': 'movable_global',
            'camera_serial': 'fixed',
            'captured_at': time.time() - 20,
            'image_base64': 'AA==',
            'mime_type': 'image/png',
        }
        self.assertFalse(observations.valid_global(frame, time.time() - 1))

    def test_context_keeps_one_image_per_camera_and_explicit_provenance(self):
        h = []

        def frame(serial):
            return {
                'camera': {'serial': serial},
                'image_base64': 'AA==',
                'mime_type': 'image/png',
                'captured_at': time.time(),
            }

        runner.Runner.append_result(
            h, 'eye-1', 'get_camera_frame', frame('wrist')
        )
        runner.Runner.append_result(
            h,
            'global-1',
            'observe_global',
            {
                **frame('fixed'),
                'camera_role': 'movable_global',
                'camera_serial': 'fixed',
            },
        )
        runner.Runner.append_result(
            h, 'eye-2', 'get_camera_frame', frame('wrist')
        )
        items = [
            i
            for i in h
            if isinstance(i.get('content'), list) and any(
                p.get('type') == 'input_image' for p in i['content']
            )
        ]
        self.assertEqual(len(items), 2)
        self.assertIn('movable_global', items[0]['content'][0]['text'])
        self.assertIn('eye_in_hand', items[1]['content'][0]['text'])
