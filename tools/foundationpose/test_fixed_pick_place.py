import copy
import json
import unittest
from unittest.mock import Mock, patch
import numpy as np
from fixed_pick_place import checked_request, placement_goals, joint_targets
import test_object_motion_control as control
import test_agent_api as api_base
from agent_api import AgentError


def fixtures():
    frame = np.eye(4)
    frame[:3, 3] = [.35, -.1, .02]
    obj = np.eye(4)
    obj[:3, 3] = [.3, -.1, .1]
    source = {
        'instance_id': 'tube',
        'object_id': 'sample',
        'role': 'source',
        'state': 'static',
        'pose_mode': '5d',
        'base_T_object': obj.tolist(),
    }
    features = []
    for slot, x in [('A01', 0), ('A02', .025)]:
        for part, z in [('entry', .085), ('support', .012)]:
            f = np.eye(4)
            f[:3, 3] = [x, 0, z]
            features.append(
                {
                    'id': slot + '.' + part,
                    'object_T_feature': f.tolist(),
                    'diameter_m': .02,
                }
            )
    rack = {
        'instance_id': 'rack',
        'object_id': 'rack_model',
        'role': 'reference',
        'state': 'static',
        'pose_mode': '6d',
        'base_T_object': frame.tolist(),
        'features': features,
    }
    scene = {'scene_id': 'a' * 32, 'instances': [source, rack]}
    request = {
        'scene_id': scene['scene_id'],
        'source_instance_id': 'tube',
        'reference_instance_id': 'rack',
        'free_slots': ['A01', 'A02'],
        'release_gap_m': .003,
    }
    vertices = np.array(
        [[x, y, z] for x in [-.007, .007] for y in [0] for z in [-.075, .075]]
    )
    return scene, request, {'vertices': vertices}


class GeometryTests(unittest.TestCase):
    def test_slots_are_ordered_nearest_to_the_source_object_first(self):
        scene, req, geometry = fixtures()
        checked_request(req)
        # The tube lies at x=0.30; slot A01 is at x=0.35 and A02 at x=0.375 in
        # Base.
        for listed in (['A01', 'A02'], ['A02', 'A01']):
            _, _, goals = placement_goals(
                scene, {**req, 'free_slots': listed}, geometry
            )
            self.assertEqual([x[0] for x in goals], ['A01', 'A02'])
            np.testing.assert_allclose(
                [x[3] for x in goals], [.05, .075], atol=1e-9
            )
        beyond = copy.deepcopy(scene)
        beyond['instances'][0]['base_T_object'][0][3] = .42
        self.assertEqual(
            [x[0] for x in placement_goals(beyond, req, geometry)[2]],
            ['A02', 'A01'],
        )
        # Height is ignored and an exact tie falls back to the slot name.
        level = copy.deepcopy(scene)
        level['instances'][0]['base_T_object'][0][3] = .3625
        level['instances'][0]['base_T_object'][2][3] = .5
        tie = placement_goals(
            level, {**req, 'free_slots': ['A02', 'A01']}, geometry
        )[2]
        self.assertEqual([x[0] for x in tie], ['A01', 'A02'])
        self.assertAlmostEqual(tie[0][3], tie[1][3])

    def test_bottom_reference_and_support_height(self):
        scene, req, geometry = fixtures()
        _, _, goals = placement_goals(scene, req, geometry)
        for _, goal, _, _ in goals:
            self.assertEqual(goal['reference_point_object_m'], [0, 0, -.075])
            self.assertAlmostEqual(goal['target_position_m'][2], .015)
            self.assertEqual(goal['target_rpy_deg'], [0, 0, 0])

    def test_inverted_rack_occupied_source_missing_slots_and_bad_fit_rejected(
        self
    ):
        scene, req, geometry = fixtures()
        variants = []
        inverted = copy.deepcopy(scene)
        inverted['instances'][1]['base_T_object'][1][1] = -1
        inverted['instances'][1]['base_T_object'][2][2] = -1
        variants.append((inverted, req))
        moved = copy.deepcopy(scene)
        moved['instances'][0]['state'] = 'held'
        variants.append((moved, req))
        variants.append((scene, {**req, 'free_slots': ['unknown']}))
        tilted = copy.deepcopy(scene)
        angle = np.deg2rad(3)
        tilted['instances'][1]['base_T_object'][0][:3] = [
            np.cos(angle), 0, np.sin(angle)
        ]
        tilted['instances'][1]['base_T_object'][2][:3] = [
            -np.sin(angle), 0, np.cos(angle)
        ]
        variants.append((tilted, req))
        small = copy.deepcopy(scene)
        for f in small['instances'][1]['features']:
            f['diameter_m'] = .005
        variants.append((small, req))
        for value, request in variants:
            with self.assertRaises(ValueError):
                placement_goals(value, request, geometry)
        for change in [
            {'free_slots': []},
            {'free_slots': [{}]},
            {'seed': 7},
            {'release_gap_m': float('nan')},
            {'release_gap_m': .01},
        ]:
            with self.assertRaises(ValueError):
                checked_request({**req, **change})

    def test_waypoint_joint_mapping_uses_own_endpoint_not_retreat(self):
        points = [
            {'name': 'grasp_sample_1'},
            {'name': 'lift_sample_1'},
            {'name': 'lift_sample_2'},
            {'name': 'release_retreat_sample_1'},
        ]
        q = [[i] * 7 for i in range(4)]
        result = joint_targets(points, q, [{'name': 'grasp'}, {'name': 'lift'}])
        self.assertEqual(result['grasp'], [0] * 7)
        self.assertEqual(result['lift'], [2000] * 7)


class PlanningTests(unittest.TestCase):
    def setUp(self):
        control.ObjectControlTests.setUp(self)
        self.service.held_object = None
        self.scene, self.request, self.geom = fixtures()
        self.service.validate_scene = Mock(return_value=self.scene)
        self.service.fixed_binding_check = Mock()
        self.service.scene_observation = Mock(return_value={})
        self.service.stationary_scene_arm = Mock(
            return_value=copy.deepcopy(self.arm)
        )
        self.service.arm_state = Mock(return_value=copy.deepcopy(self.arm))
        source = self.scene['instances'][0]
        self.grasp = {
            'object_id': 'sample',
            'object_name': 'tube',
            'model_sha256': 'mesh',
            'pose_mode': '5d',
            'pose_mode_revision': 'v',
            'tool_response': {},
            'scene_id': self.scene['scene_id'],
            'instance_id': 'tube',
            'base_T_object': source['base_T_object'],
            'flange_T_grip_tcp': np.eye(4).tolist(),
            'grasp_library_revision': 'v',
            'execution_blockers': [],
            'effective_parameters': {'min_table_clearance_m': .01},
            'current_arm': copy.deepcopy(self.arm),
            'waypoints': [
                {
                    'name': 'pregrasp',
                    'controller_pose': [300000, -100000, 200000, 0, 0, 0],
                },
                {
                    'name': 'grasp',
                    'controller_pose': [300000, -100000, 100000, 0, 0, 0],
                },
                {
                    'name': 'lift',
                    'controller_pose': [300000, -100000, 250000, 0, 0, 0],
                },
            ],
        }
        self.service.prepare_grasp_plan = Mock(
            return_value=(self.grasp, self.config)
        )

    def check(self, checker, tool, arm, points, timeout=3):
        self.assertEqual(arm, self.arm)
        self.assertTrue(points[0]['name'].startswith('pregrasp_'))
        return (
            True,
            {
                'verified': True,
                'joint_solutions_deg': [[0] * 7 for _ in points],
            },
        )

    def test_full_plan_includes_grasp_and_release_before_any_write(self):
        with patch(
            'fixed_pick_place.model_geometry', return_value=self.geom
        ), patch('fixed_pick_place.RobotClient') as robot, patch(
            'fixed_pick_place.check_waypoints', side_effect=self.check
        ) as check:
            plan = self.service.plan_fixed_pick_place(self.request, {})
        self.assertTrue(plan['complete_path_prechecked'])
        self.assertEqual(plan['selected_slot'], 'A01')
        self.assertEqual(plan['slot_selection'], 'nearest_feasible_to_source')
        self.assertAlmostEqual(plan['selected_slot_distance_m'], .05)
        self.assertEqual(plan['nearer_slots_rejected'], [])
        self.assertEqual(
            set(plan['grasp']['fixed_joint_targets']),
            {'pregrasp', 'grasp', 'lift'},
        )
        points = check.call_args.args[3]
        self.assertTrue(
            any(p['name'].startswith('release_descent') for p in points)
        )
        self.assertTrue(
            any(p['name'].startswith('release_retreat') for p in points)
        )
        robot.assert_called_once_with(
            self.config['robot_ip'], self.config['robot_port'], writes=False
        )
        self.assertEqual(self.calls, [])

    def test_failure_in_descent_never_saves_or_executes_a_prefix(self):
        def reject(checker, tool, arm, points, timeout=3):
            if any(p['name'].startswith('release_descent') for p in points):
                return False, {'reason': 'descent failure'}
            return self.check(checker, tool, arm, points, timeout)

        store = Mock()
        self.service.object_plan_store = Mock(return_value=store)
        with patch(
            'fixed_pick_place.model_geometry', return_value=self.geom
        ), patch('fixed_pick_place.RobotClient'), patch(
            'fixed_pick_place.check_waypoints', side_effect=reject
        ):
            with self.assertRaisesRegex(ValueError, '全程'):
                self.service.plan_fixed_pick_place(self.request, {})
        store.save.assert_not_called()
        self.assertEqual(self.calls, [])

    def test_start_shift_blocks_before_gripper_or_replanning(self):
        self.service.stationary_scene_arm.return_value = {
            **self.arm, 'joint': [1000] * 7
        }
        self.service.execute = Mock()
        plan = {
            'effective_config': self.config,
            'grasp': self.grasp,
            'current_arm': self.arm,
        }
        with patch('fixed_pick_place.RobotClient'):
            with self.assertRaisesRegex(ValueError, '起点'):
                self.service.execute_fixed_pick_place(plan, {}, 'id')
        self.service.execute.assert_not_called()
        self.service.prepare_grasp_plan.assert_not_called()

    def test_execution_uses_frozen_targets_and_rejects_changed_attachment(self):
        with patch(
            'fixed_pick_place.model_geometry', return_value=self.geom
        ), patch('fixed_pick_place.RobotClient'), patch(
            'fixed_pick_place.check_waypoints', side_effect=self.check
        ):
            plan = self.service.plan_fixed_pick_place(self.request, {})
        self.service.verify_motion_targets = Mock(
            side_effect=lambda p, c, r, a, points: {
                'joint_solutions_deg': [[0] * 7 for _ in points]
            }
        )
        self.service.update_scene_instance = Mock()
        order = []

        def grasp(p, c, r):
            order.append('grasp')
            self.assertEqual(p['waypoints'], plan['grasp']['waypoints'])
            self.service.held_object = {
                **copy.deepcopy(plan['predicted_hold']), 'hold_id': 'actual'
            }

        def release(p, request_id, nested=False):
            order.append('release')
            self.assertTrue(nested)
            self.assertEqual(p['waypoints'], plan['release']['waypoints'])
            self.assertEqual(p['hold']['hold_id'], 'actual')
            self.service.status['state'] = 'completed'
            # The complete hold record is required by the release hardware
            # checks.
            self.assertEqual(p['hold'], self.service.held_object)
            self.assertIn('tool_response', p['hold'])
            self.assertGreater(p['max_execution_duration_s'], 90)

        self.service.execute = Mock(side_effect=grasp)
        self.service.run_object_execution = Mock(side_effect=release)
        with patch(
            'fixed_pick_place.model_geometry', return_value=self.geom
        ), patch('fixed_pick_place.RobotClient'):
            self.service.execute_fixed_pick_place(plan, {}, 'test')
        self.assertEqual(order, ['grasp', 'release'])

        def changed(p, c, r):
            grasp(p, c, r)
            self.service.held_object['object_T_tcp'][0][3] += .003

        self.service.execute.side_effect = changed
        self.service.run_object_execution.reset_mock()
        with patch(
            'fixed_pick_place.model_geometry', return_value=self.geom
        ), patch('fixed_pick_place.RobotClient'):
            with self.assertRaisesRegex(ValueError, '夹持关系'):
                self.service.execute_fixed_pick_place(plan, {}, 'test')
        self.service.run_object_execution.assert_not_called()


class GraspSegmentVerificationTests(unittest.TestCase):
    """Each grasp segment re-solves the remaining path and is driven through its own samples."""

    def setUp(self):
        control.ObjectControlTests.setUp(self)
        names = [
            'pregrasp_sample_0',
            'pregrasp_sample_1',
            'grasp_sample_0',
            'grasp_sample_1',
            'lift_sample_0',
            'release_descent_1_sample_0',
        ]
        self.points = [{'name': n, 'controller_pose': [0] * 6} for n in names]
        self.service.arm_state = Mock(return_value=copy.deepcopy(self.arm))
        self.seen = []

        def solve(plan, config, report, arm, points):
            self.seen.append([p['name'] for p in points])
            # Path samples solve to [k]*7 for sample k; a lone waypoint (not a
            # sample) solves differently.
            if points[0]['name'] not in names:
                return {'joint_solutions_deg': [[99] * 7]}
            return {
                'joint_solutions_deg': [
                    [names.index(p['name'])] * 7 for p in points
                ]
            }

        self.service.verify_motion_targets = Mock(side_effect=solve)
        self.plan = {
            'fixed_joint_targets': {
                'pregrasp': [1000] * 7, 'grasp': [3000] * 7, 'lift': [4000] * 7
            }
        }

    def test_each_segment_solves_the_remaining_fixed_path_and_returns_its_own_samples(
        self
    ):
        self.service._fixed_path_points = self.points
        report = {}
        for name, own in [
            ('pregrasp', [0, 1]), ('grasp', [2, 3]), ('lift', [4])
        ]:
            with self.subTest(name=name):
                solved = self.service.verify_waypoint(
                    self.plan,
                    {},
                    report,
                    {'name': name, 'controller_pose': [0] * 6},
                )
                self.assertEqual(solved, [[k] * 7 for k in own])
                self.assertEqual(
                    self.seen[-1], [p['name'] for p in self.points[own[0]:]]
                )
        self.assertNotIn('joint_configuration_changes', report)

    def test_changed_solution_replaces_the_planned_set_and_is_recorded(self):
        self.service._fixed_path_points = self.points
        report = {}
        self.plan['fixed_joint_targets']['grasp'] = [3600] * 7
        solved = self.service.verify_waypoint(
            self.plan, {}, report, {'name': 'grasp', 'controller_pose': [0] * 6}
        )
        self.assertEqual(solved[-1], [3] * 7)
        change = report['joint_configuration_changes'][0]
        self.assertEqual(
            (change['stage'], change['basis']),
            ('grasp', 'resolved_remaining_path_vs_plan'),
        )
        self.assertEqual(change['reference_joint_raw'], [3600] * 7)
        self.assertAlmostEqual(change['max_deviation_deg'], .6)

    def test_unsolvable_remaining_path_and_missing_samples_stop_before_motion(
        self
    ):
        self.service._fixed_path_points = self.points
        self.service.verify_motion_targets.side_effect = ValueError(
            '没有连续逆解'
        )
        with self.assertRaisesRegex(ValueError, '没有连续逆解'):
            self.service.verify_waypoint(
                self.plan, {}, {}, {'name': 'lift', 'controller_pose': [0] * 6}
            )
        self.service._fixed_path_points = None
        with self.assertRaisesRegex(ValueError, '缺少航点采样'):
            self.service.verify_waypoint(
                self.plan,
                {},
                {},
                {'name': 'pregrasp', 'controller_pose': [0] * 6},
            )

    def test_ordinary_grasp_without_fixed_targets_still_checks_only_its_waypoint(
        self
    ):
        waypoint = {'name': 'pregrasp', 'controller_pose': [0] * 6}
        self.assertIsNone(self.service.verify_waypoint({}, {}, {}, waypoint))
        self.assertEqual(self.seen, [['pregrasp']])


class SampledJointSegmentTests(unittest.TestCase):
    """A fixed plan drives straight segments as one joint move per verified sample, never movel."""

    def setUp(self):
        control.ObjectControlTests.setUp(self)

    def test_samples_are_sent_in_order_and_each_arrival_is_checked(self):
        self.service.robot = self.robot
        sent = []
        state = {'joint': [0] * 7}

        def joints(values, speed, cancelled, watchdog):
            sent.append((list(values), speed))
            watchdog()
            state['joint'] = list(values)

        self.robot.move_joints.side_effect = joints
        self.service.arm_state = Mock(side_effect=lambda: copy.deepcopy(state))
        watchdog = Mock()
        report = {}
        last = self.service.follow_joint_samples(
            [[1, 0, 0, 0, 0, 0, 0], [2.0004, 0, 0, 0, 0, 0, 0]],
            25,
            watchdog,
            report,
            'lift',
        )
        self.assertEqual(
            sent,
            [([1000, 0, 0, 0, 0, 0, 0], 25), ([2000, 0, 0, 0, 0, 0, 0], 25)],
        )
        self.assertEqual(last, [2000, 0, 0, 0, 0, 0, 0])
        self.assertEqual(
            report['sampled_joint_segments'],
            [{'stage': 'lift', 'joint_moves': 2, 'final_joint_raw': last}],
        )
        self.assertEqual(watchdog.call_count, 2)
        self.robot.move_linear.assert_not_called()

    def test_unreached_sample_or_invalid_solution_stops_the_segment(self):
        self.service.robot = self.robot
        sent = []
        self.robot.move_joints.side_effect = lambda values, *rest: sent.append(
            list(values)
        )
        self.service.arm_state = Mock(return_value={'joint': [0] * 7})
        with self.assertRaisesRegex(ValueError, '未到达所选关节解'):
            self.service.follow_joint_samples(
                [[1] * 7, [2] * 7], 25, Mock(), {}, 'lift'
            )
        self.assertEqual(len(sent), 1)
        for bad in ([[1] * 6], [[float('nan')] * 7], []):
            with self.subTest(bad=bad), self.assertRaisesRegex(
                ValueError, '缺少已验证的关节解'
            ):
                self.service.follow_joint_samples(bad, 25, Mock(), {}, 'lift')
        self.assertEqual(len(sent), 1)

    def test_fixed_release_uses_sampled_joint_moves_and_completes_with_different_planned_joints(
        self
    ):
        args = control.ObjectControlTests.args(self)
        args.update(goal=None, retreat_vector_base_m=[0, 0, .05])
        plan = copy.deepcopy(
            control.ObjectControlTests.planned(self, args, True)
        )
        names = (
            [w['name'] for w in plan['waypoints']]
            + [plan['retreat_waypoint']['name']]
        )
        plan['fixed_joint_targets'] = {name: [777000] * 7 for name in names}
        before = control.controller_pose(self.arm['pose'])
        self.service.status.update(busy=True)
        self.service.run_object_execution(plan, 'fixed-release', nested=True)
        self.assertEqual(
            self.service.status['state'],
            'completed',
            self.service.status.get('error'),
        )
        kinds = [c[0] for c in self.calls]
        self.assertNotIn('movel', kinds)
        self.assertEqual(kinds.count('open'), 1)
        opened = kinds.index('open')
        self.assertGreaterEqual(opened, 2)
        self.assertGreater(len(kinds) - opened - 1, 0)
        self.assertEqual(set(kinds) - {'open'}, {'movej'})
        self.assertIsNone(self.service.held_object)
        np.testing.assert_allclose(
            control.controller_pose(self.arm['pose'])[:3, 3],
            before[:3, 3] + [0, 0, .05],
            atol=1e-6,
        )

    def test_nested_release_failing_before_its_context_reports_the_real_error(
        self
    ):
        args = control.ObjectControlTests.args(self)
        args.update(goal=None, retreat_vector_base_m=[0, 0, .05])
        plan = copy.deepcopy(
            control.ObjectControlTests.planned(self, args, True)
        )
        self.service.robot = self.robot
        self.service.actuated = True
        hold = copy.deepcopy(self.service.held_object)
        self.service.motion_context = Mock(
            side_effect=KeyError('tool_response')
        )
        self.service.status.update(busy=True)
        self.service.run_object_execution(plan, 'fixed-release', nested=True)
        self.assertEqual(self.service.status['state'], 'failed')
        self.assertIn('tool_response', self.service.status['error'])
        self.assertNotIn('local variable', self.service.status['error'])
        self.service.retain_reorientation_hold.assert_not_called()
        self.assertEqual(self.service.held_object, hold)
        self.assertEqual(self.calls, [])

    def test_ordinary_release_still_uses_controller_linear_moves(self):
        args = control.ObjectControlTests.args(self)
        args.update(goal=None, retreat_vector_base_m=[0, 0, .05])
        plan = control.ObjectControlTests.planned(self, args, True)
        status = control.ObjectControlTests.execute(self, plan, 'release-001')
        self.assertEqual(status['state'], 'completed', status.get('error'))
        self.assertEqual(
            [c[0] for c in self.calls], ['movel', 'movel', 'open', 'movel']
        )

    def test_fixed_release_budget_and_joint_change_note(self):
        import fixed_pick_place
        from motion_validation import note_joint_change
        self.assertGreater(fixed_pick_place.FIXED_RELEASE_BUDGET_S, 90)
        report = {}
        self.assertAlmostEqual(
            note_joint_change(report, 'lift', 'basis', [1000] * 7, [1400] * 7),
            .4,
        )
        self.assertEqual(report, {})
        self.assertAlmostEqual(
            note_joint_change(
                report,
                'lift',
                'basis',
                [1000] * 7,
                [1000, 1000, 11010.4, 1000, 1000, 1000, 1000],
            ),
            10.0104,
        )
        self.assertEqual(
            report['joint_configuration_changes'][0]['actual_joint_raw'][2],
            11010,
        )


class ApiTests(unittest.TestCase):
    request = api_base.AgentAPITests.request
    wait = api_base.AgentAPITests.wait

    def setUp(self):
        api_base.AgentAPITests.setUp(self)
        self.grasp.validate_scene = Mock()

        def run(args, request_id, plan):
            if plan is None:
                plan = self.grasp.object_plan_store().save(
                    {'kind': 'fixed_pick_place', 'selected_slot': 'A01'}
                )
                self.grasp.update(
                    state='fixed_pick_place_planned',
                    fixed_pick_place_plan=plan,
                    busy=False,
                )
            else:
                self.calls.append('execute')
                self.grasp.object_plan_store().finish(
                    plan['plan_id'], 'completed'
                )
                self.grasp.update(state='completed', busy=False)

        self.grasp.run_fixed_pick_place = run

    def test_fixed_execution_is_mode_gated_single_use_and_deduplicated(self):
        req = fixtures()[1]
        self.request('plan_fixed_pick_place', arguments=req)
        plan = self.wait()['result']['fixed_pick_place_plan']
        args = {'plan_id': plan['plan_id']}
        self.grasp.mode = 'preview'
        with self.assertRaises(AgentError):
            self.request(
                'execute_fixed_pick_place', task_id='exec-001', arguments=args
            )
        self.grasp.mode = 'execute'
        self.request(
            'execute_fixed_pick_place', task_id='exec-001', arguments=args
        )
        self.wait('exec-001')
        self.assertTrue(
            self.request(
                'execute_fixed_pick_place', task_id='exec-001', arguments=args
            )['deduplicated']
        )
        self.assertEqual(
            self.request(
                'execute_fixed_pick_place', task_id='exec-002', arguments=args
            )['state'],
            'rejected',
        )
        self.assertEqual(self.calls, ['execute'])

    def test_cancel_planning_is_id_scoped_and_never_stops_hardware(self):
        self.api.active = 'planning-id'
        self.api.get_task = Mock(
            return_value={'operation': 'plan_fixed_pick_place'}
        )
        self.grasp.actuated = False
        self.grasp.robot = Mock()
        self.assertFalse(
            self.api.post(
                '/api/agent/cancel-planning', {'task_id': 'other'}
            )['cancel_requested']
        )
        self.assertFalse(self.grasp.cancelled.is_set())
        self.assertTrue(
            self.api.post(
                '/api/agent/cancel-planning', {'task_id': 'planning-id'}
            )['cancel_requested']
        )
        self.assertTrue(self.grasp.cancelled.is_set())
        self.grasp.robot.stop.assert_not_called()
        self.grasp.actuated = True
        with self.assertRaises(AgentError):
            self.api.post(
                '/api/agent/cancel-planning', {'task_id': 'planning-id'}
            )
