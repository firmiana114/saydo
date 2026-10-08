import copy
import unittest
import numpy as np
import test_agent_api as base
from agent_api import AgentError


class AgentObjectMotionTests(unittest.TestCase):
    request = base.AgentAPITests.request
    wait = base.AgentAPITests.wait

    def setUp(self):
        base.AgentAPITests.setUp(self)
        self.grasp.held_object = {
            'hold_id': 'h',
            'object_id': 'bottle',
            'model_sha256': 'mesh',
            'orientation_version': 0,
            'object_T_tcp': np.eye(4).tolist(),
            'current_object_rotation': np.eye(3).tolist(),
        }

        def plan(held, goal, release, retreat, request_id):
            value = self.grasp.object_plan_store().save(
                {
                    'kind': 'release' if release else 'held_object_motion',
                    'hold': held,
                    'goal': goal,
                    'waypoints': [],
                    'base_T_object_target': np.eye(4).tolist(),
                }
            )
            self.calls.append(('plan', release))
            self.grasp.update(
                state='object_motion_planned',
                object_motion_plan=value,
                busy=False,
            )

        def execute(value, request_id):
            self.calls.append(('execute', value['plan_id']))
            if value['kind'] == 'release':
                self.grasp.held_object = None
            else:
                self.grasp.held_object['orientation_version'] += 1
            self.grasp.object_plan_store().finish(value['plan_id'], 'completed')
            self.grasp.update(
                state='completed',
                execution_result={
                    'kind': value['kind'], 'plan_id': value['plan_id']
                },
                busy=False,
            )

        self.grasp.run_object_plan = plan
        self.grasp.run_object_execution = execute
        self.args = {
            'object_id': 'bottle',
            'hold_id': 'h',
            'orientation_version': 0,
            'goal': {'target_position_m': [.3, 0, .2]},
        }

    def test_plan_execute_dedup_and_second_request_cannot_replay_plan(self):
        self.request('plan_held_object_motion', arguments=self.args)
        planned = self.wait()
        self.assertEqual(planned['state'], 'object_motion_planned')
        pid = planned['result']['object_motion_plan']['plan_id']
        self.request(
            'execute_plan', task_id='execute-0001', arguments={'plan_id': pid}
        )
        done = self.wait('execute-0001')
        self.assertEqual(done['state'], 'completed')
        self.assertEqual(
            done['result']['held_object']['orientation_version'], 1
        )
        self.assertTrue(
            self.request(
                'execute_plan',
                task_id='execute-0001',
                arguments={'plan_id': pid},
            )['deduplicated']
        )
        other = self.request(
            'execute_plan', task_id='execute-0002', arguments={'plan_id': pid}
        )
        self.assertEqual(other['state'], 'rejected')
        self.assertEqual(len([c for c in self.calls if c[0] == 'execute']), 1)

    def test_release_planning_allowed_in_preview_but_execution_blocked(self):
        self.grasp.mode = 'preview'
        args = {**self.args, 'goal': None, 'retreat_vector_base_m': [0, 0, .05]}
        self.request('plan_release', arguments=args)
        plan = self.wait()['result']['object_motion_plan']
        self.assertEqual(plan['kind'], 'release')
        self.assertIsNotNone(self.grasp.held_object)
        with self.assertRaises(AgentError):
            self.request(
                'execute_plan',
                task_id='execute-disabled',
                arguments={'plan_id': plan['plan_id']},
            )
        self.grasp.mode = 'execute'
        self.request(
            'execute_plan',
            task_id='release-001',
            arguments={'plan_id': plan['plan_id']},
        )
        self.assertIsNone(self.wait('release-001')['result']['held_object'])

    def test_malformed_requests_and_hold_changes_do_not_launch_worker(self):
        for override in [
            {'orientation_version': True},
            {'goal': {'target_position_m': [True, 0, 0]}},
            {'goal': {'target_rpy_deg': [0, 0, 181]}},
            {'goal': {'reference_frame': 'file'}},
        ]:
            with self.assertRaises(AgentError):
                self.request(
                    'plan_held_object_motion',
                    arguments={**self.args, **override},
                )
        result = self.request(
            'plan_held_object_motion', arguments={**self.args, 'hold_id': 'old'}
        )
        self.assertEqual(result['state'], 'rejected')
        self.assertEqual(self.calls, [])
        for args in [
            {'plan_id': '../../file'},
            {'plan_id': 'f' * 32, 'target_position_m': [0, 0, 0]},
        ]:
            with self.assertRaises(AgentError):
                self.request('execute_plan', arguments=args)

    def test_diagnostics_old_task_and_model_geometry_are_read_only(self):
        self.request('plan_held_object_motion', arguments=self.args)
        self.wait()
        result = self.api.get(
            '/api/agent/task-diagnostics', {'task_id': ['request-0001']}
        )
        self.assertFalse(result['diagnostics_available'])
        self.grasp.object_geometry_info = lambda oid: {
            'object_id': oid, 'units': 'metres'
        }
        self.assertEqual(
            self.api.get(
                '/api/agent/object-geometry', {'object_id': ['cup']}
            )['object_id'],
            'cup',
        )


if __name__ == '__main__':
    unittest.main()
