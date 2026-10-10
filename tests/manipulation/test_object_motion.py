import copy
import json
import pathlib
import tempfile
import threading
import unittest
from unittest import mock
import numpy as np
from scipy.spatial.transform import Rotation
from saydo.geometry import grasp_geometry
from saydo.manipulation import object_motion_geometry
from saydo.manipulation import object_motion_store
from saydo.contracts import motion_contract


def fixture():
    held = {
        'object_id': 'sample',
        'hold_id': 'hold1',
        'orientation_version': 0,
        'model_sha256': 'mesh1',
        'pose_mode': '6d',
        'tool_response': {},
        'contact_position': 500,
    }
    attachment = np.eye(4)
    attachment[:3, 3] = [0, .02, .04]
    held['object_T_tcp'] = attachment.tolist()
    vertices = np.array(
        [
            [x, y, z]
            for x in [-.015, .015]
            for y in [-.015, .015]
            for z in [-.06, .06]
        ]
    )
    arm = {
        'pose': [300000, -100000, 250000, 0, 0, 0], 'joint': [0] * 7, 'err': [0]
    }
    config = {
        'workspace_reviewed': True,
        'workspace_min_m': [-1, -1, 0],
        'workspace_max_m': [1, 1, 1],
        'table_z_m': 0,
        'min_table_clearance_m': .01,
        'arm_speed_percent': 25,
    }
    return held, vertices, arm, config


class ObjectGeometryTests(unittest.TestCase):
    def test_late_valid_candidate_is_kept_after_twenty_seconds(self):
        held, vertices, arm, config = fixture()
        clock = [0.]
        calls = []

        def reachable(points, timeout):
            # 候选成批并行检查，不能按调用次序决定结果：只让绕行路线通过。
            clock[0] += 15.
            calls.append(timeout)
            detour = any('detour' in point['name'] for point in points)
            return detour, {'verified': detour, 'reason': 'try another route'}

        with mock.patch('time.monotonic', side_effect=lambda: clock[0]):
            plan = object_motion_geometry.plan_object_motion(
                held,
                vertices,
                arm,
                config,
                {'target_position_m': [.4, -.1, .3]},
                np.eye(4),
                reachable,
                lambda: None,
            )
        # 每次检查“耗时”15 秒，找到可行候选时早已超过 20 秒，仍然采用。
        self.assertGreaterEqual(clock[0], 30.)
        self.assertTrue(plan['reachability_check']['verified'])
        self.assertEqual(plan['route_offset_m'], [0., .08])
        self.assertEqual(set(calls), {3.})

    def test_candidate_search_remains_cancellable(self):
        h, v, a, c = fixture()
        calls = []

        def cancelled():
            if calls:
                raise RuntimeError('cancelled')

        def reachable(points, timeout):
            calls.append(1)
            return False, {'reason': 'no route'}

        with self.assertRaisesRegex(RuntimeError, 'cancelled'):
            object_motion_geometry.plan_object_motion(
                h,
                v,
                a,
                c,
                {'target_position_m': [.4, -.1, .3]},
                np.eye(4),
                reachable,
                cancelled,
            )
        self.assertGreaterEqual(len(calls), 1)
        self.assertLessEqual(len(calls), object_motion_geometry.SEARCH_WORKERS)

    def plan(self, goal, **kw):
        held, vertices, arm, config = fixture()
        return object_motion_geometry.plan_object_motion(
            held,
            vertices,
            arm,
            config,
            goal,
            np.eye(4),
            mock.Mock(return_value=(True, {'verified': True})),
            lambda: None,
            **kw,
        )

    def test_translation_rotation_full_pose_and_custom_reference(self):
        held, vertices, arm, config = fixture()
        start = (
            grasp_geometry.controller_pose(arm['pose'])
            @ np.linalg.inv(held['object_T_tcp'])
        )
        plan = self.plan({'target_position_m': [.4, -.2, .3]})
        np.testing.assert_allclose(
            np.asarray(plan['base_T_object_target'])[:3, 3], [.4, -.2, .3]
        )
        np.testing.assert_allclose(
            grasp_geometry.controller_pose(
                plan['waypoints'][-1]['controller_pose']
            )[:3, 3],
            [.4, -.18, .34],
        )
        plan = self.plan(
            {
                'target_rpy_deg': [90, 0, 0],
                'reference_point_object_m': [0, 0, .04],
            }
        )
        reference = start[:3, 3] + start[:3, :3] @ np.array([0, 0, .04])
        self.assertEqual(len(plan['waypoints']), 7)
        last = start[:3, :3]
        for waypoint in plan['waypoints']:
            obj = np.array(waypoint['object_pose'])
            np.testing.assert_allclose(
                obj[:3, 3] + obj[:3, :3] @ [0, 0, .04], reference, atol=1e-12
            )
            self.assertLessEqual(
                np.rad2deg(grasp_geometry.rotation_error(last, obj[:3, :3])),
                15.00001,
            )
            last = obj[:3, :3]
        plan = self.plan(
            {'target_position_m': [.4, 0, .35], 'target_rpy_deg': [90, 0, 0]}
        )
        np.testing.assert_allclose(
            np.asarray(plan['base_T_object_target'])[:3, 3], [.4, 0, .35]
        )

    def test_frozen_frame_and_waypoint_component_preservation(self):
        h, v, a, c = fixture()
        frame = np.eye(4)
        frame[:3, :3] = Rotation.from_euler('z', 90, degrees=True).as_matrix()
        frame[:3, 3] = [.2, .2, 0]
        goal = {
            'reference_frame': 'camera',
            'target_position_m': [.1, 0, .3],
            'target_rpy_deg': [0, 0, 0],
            'waypoints': [{'position_m': [0, 0, .3], 'rpy_deg': None}],
        }
        p = object_motion_geometry.plan_object_motion(
            h,
            v,
            a,
            c,
            goal,
            frame,
            mock.Mock(return_value=(True, {})),
            lambda: None,
        )
        np.testing.assert_allclose(
            p['target_reference_position_base_m'], [.2, .3, .3]
        )
        np.testing.assert_allclose(
            np.asarray(p['base_T_object_target'])[:3, :3], frame[:3, :3]
        )

    def test_linear_samples_and_rejections(self):
        h, v, a, c = fixture()
        reachable = mock.Mock(return_value=(True, {}))
        p = object_motion_geometry.plan_object_motion(
            h,
            v,
            a,
            c,
            {'target_position_m': [.4, -.1, .3], 'path_mode': 'linear'},
            np.eye(4),
            reachable,
            lambda: None,
        )
        self.assertGreater(
            len(reachable.call_args.args[0]), p['waypoint_count']
        )
        for goal in [
            {'target_position_m': [0, 0, -.1]},
            {'target_position_m': [1.5, 0, .2]},
            {
                'target_position_m': [.3, 0, .2],
                'reference_point_object_m': [2, 0, 0],
            },
        ]:
            with self.assertRaises(ValueError):
                self.plan(goal)
        for goal in [
            {'target_position_m': [True, 0, 0]},
            {'target_rpy_deg': [0, 0, 181]},
            {'target_position_m': [0, 0, .2], 'position_mode': 'adaptive'},
            {'reference_frame': 'unknown'},
            {'target_rpy_deg': [0, 0, 0], 'max_rotation_step_deg': 0},
        ]:
            with self.assertRaises(ValueError):
                motion_contract.checked_goal(goal)

    def test_5d_free_spin_and_zero_offset_are_not_reapplied(self):
        h, v, a, c = fixture()
        h['pose_mode'] = '5d'
        blocked = []
        guard = threading.Lock()

        def reachable(points, timeout):
            # 第一个被检查的候选不通过，其余都通过；并行时是哪一个并不
            # 确定，下面的断言对任何转角都应成立。
            with guard:
                first = not blocked
                blocked.append(1)
            return (not first), {'reason': 'first blocked'}

        p = object_motion_geometry.plan_object_motion(
            h,
            v,
            a,
            c,
            {'target_position_m': [.4, 0, .3], 'target_rpy_deg': [0, 90, 0]},
            np.eye(4),
            reachable,
            lambda: None,
        )
        self.assertGreaterEqual(len(blocked), 2)
        np.testing.assert_allclose(
            np.asarray(p['base_T_object_target'])[:3, 2], [1, 0, 0], atol=1e-12
        )
        expected = np.asarray(p['base_T_object_target']) @ h['object_T_tcp']
        np.testing.assert_allclose(
            grasp_geometry.controller_pose(
                p['waypoints'][-1]['controller_pose']
            )[:3, 3],
            expected[:3, 3],
            atol=1e-6,
        )
        with self.assertRaises(ValueError):
            object_motion_geometry.plan_object_motion(
                h,
                v,
                a,
                c,
                {
                    'target_position_m': [.4, 0, .3],
                    'reference_point_object_m': [.01, 0, 0],
                },
                np.eye(4),
                reachable,
                lambda: None,
            )

    def test_generic_release_uses_base_vertical_and_same_reference(self):
        h, v, a, c = fixture()
        frame = np.eye(4)
        frame[:3, :3] = Rotation.from_euler('x', 30, degrees=True).as_matrix()
        for point in ([0, 0, 0], [0, 0, -.06]):
            goal = {
                'target_position_m': [.3, 0, .3],
                'target_rpy_deg': [0, 20, 0],
                'reference_point_object_m': point,
            }
            p = object_motion_geometry.plan_object_motion(
                h,
                v,
                a,
                c,
                goal,
                frame,
                mock.Mock(return_value=(True, {'verified': True})),
                lambda: None,
                release=True,
            )
            pre, last = p['waypoints'][-2:]
            self.assertTrue(pre['name'].startswith('release_preplacement'))
            self.assertEqual(last['method'], 'linear')
            np.testing.assert_allclose(
                (
                    np.array(pre['reference_position_base_m'])
                    - last['reference_position_base_m']
                ),
                [0, 0, .1],
                atol=1e-12,
            )
            np.testing.assert_allclose(
                np.array(pre['object_pose'])[:3, :3],
                np.array(last['object_pose'])[:3, :3],
                atol=1e-12,
            )
            np.testing.assert_allclose(
                last['reference_position_base_m'],
                (frame @ np.r_[goal['target_position_m'], 1])[:3],
            )
            self.assertEqual(p['reference_point_object_m'], point)
        c['release_preplacement_height_m'] = float('nan')
        with self.assertRaises(ValueError):
            object_motion_geometry.plan_object_motion(
                h, v, a, c, goal, frame, mock.Mock(), lambda: None, release=True
            )

    def test_preplacement_workspace_checked_and_height_configurable(self):
        h, v, a, c = fixture()
        c['release_preplacement_height_m'] = .12
        p = object_motion_geometry.plan_object_motion(
            h,
            v,
            a,
            c,
            {'target_position_m': [.4, 0, .3]},
            np.eye(4),
            mock.Mock(return_value=(True, {'verified': True})),
            lambda: None,
            release=True,
        )
        np.testing.assert_allclose(
            (
                np.array(p['waypoints'][-2]['reference_position_base_m'])
                - p['waypoints'][-1]['reference_position_base_m']
            ),
            [0, 0, .12],
        )
        with self.assertRaises(ValueError):
            self.plan({'target_position_m': [.4, 0, .95]}, release=True)

    def test_combined_goal_always_orients_before_translation(self):
        for release in (False, True):
            p = self.plan(
                {
                    'target_position_m': [.4, -.1, .3],
                    'target_rpy_deg': [0, 60, 0],
                    'waypoints': [
                        {'position_m': [.35, -.1, .35], 'rpy_deg': None}
                    ],
                },
                release=release,
            )
            point = np.array(p['reference_point_object_m'])
            start = np.array(p['base_T_object_start'])
            start_ref = start[:3, 3] + start[:3, :3] @ point
            target = np.array(p['base_T_object_target'])[:3, :3]
            translating = False
            orientation_steps = 0
            for w in p['waypoints']:
                obj = np.array(w['object_pose'])
                ref = obj[:3, 3] + obj[:3, :3] @ point
                if w['name'].startswith(('orient_', 'release_orientation')):
                    self.assertFalse(translating)
                    np.testing.assert_allclose(ref, start_ref, atol=1e-9)
                    orientation_steps += 1
                else:
                    translating = True
                    self.assertEqual(w['method'], 'linear')
                    np.testing.assert_allclose(obj[:3, :3], target, atol=1e-9)
            self.assertGreater(orientation_steps, 1)
            self.assertTrue(translating)
        with self.assertRaisesRegex(ValueError, '中间路点不得'):
            self.plan(
                {
                    'target_rpy_deg': [0, 60, 0],
                    'waypoints': [
                        {'position_m': [.3, 0, .3], 'rpy_deg': [0, 20, 0]}
                    ],
                }
            )

    def test_detour_search_preserves_goal_orientation_and_final_descent(self):
        h, v, a, c = fixture()
        proof = {'verified': True}
        def reachable(points, timeout):
            if any('detour' in point['name'] for point in points):
                return True, dict(proof)
            return (
                False,
                {
                    'reason': 'blocked direct',
                    'failure': {
                        'kind': 'ik_failed',
                        'waypoint': 'release_preplacement_1_sample_2',
                    },
                },
            )

        p = object_motion_geometry.plan_object_motion(
            h,
            v,
            a,
            c,
            {'target_position_m': [.4, -.1, .3]},
            np.eye(4),
            reachable,
            lambda: None,
            release=True,
        )
        self.assertEqual(p['route_offset_m'], [0., .08])
        self.assertTrue(any('detour' in w['name'] for w in p['waypoints']))
        pre, last = p['waypoints'][-2:]
        self.assertEqual(last['name'], 'release_descent_1')
        np.testing.assert_allclose(
            (
                np.array(pre['reference_position_base_m'])
                - last['reference_position_base_m']
            ),
            [0, 0, .1],
        )
        final = np.array(p['base_T_object_target'])
        for w in p['waypoints']:
            self.assertEqual(w['method'], 'linear')
            np.testing.assert_allclose(
                np.array(w['object_pose'])[:3, :3], final[:3, :3]
            )
        self.assertEqual(
            p['search_attempts'][0]['failure']['kind'], 'ik_failed'
        )

    def test_release_current_pose_and_retreat(self):
        p = self.plan(None, release=True, retreat=[0, 0, .05])
        self.assertEqual(p['kind'], 'release')
        self.assertEqual(p['waypoint_count'], 2)
        self.assertAlmostEqual(p['retreat_waypoint']['position_m'][2], .30)
        self.assertFalse(p['validation_scope']['object_support_checked'])
        with self.assertRaises(ValueError):
            self.plan(None, release=True, retreat=[.3, .3, 0])
        with self.assertRaises(ValueError):
            self.plan({'position_mode': 'adaptive'}, release=True)


class PlanStoreTests(unittest.TestCase):
    def test_expiry_claim_restart_and_tampering(self):
        with tempfile.TemporaryDirectory() as d:
            store = object_motion_store.ObjectPlanStore(d)
            plan = store.save({'kind': 'held_object_motion'})
            self.assertEqual(store.get(plan['plan_id']), plan)
            with mock.patch(
                'saydo.manipulation.object_motion_store.time.time',
                return_value=plan['expires_at'] + 1,
            ):
                with self.assertRaisesRegex(ValueError, '有效期'):
                    store.get(plan['plan_id'])
            store.claim(plan['plan_id'], 'request1')
            with self.assertRaisesRegex(ValueError, '领取'):
                store.claim(plan['plan_id'], 'request2')
            fresh = store.save({'kind': 'release'})
            with self.assertRaisesRegex(ValueError, '重启'):
                object_motion_store.ObjectPlanStore(d).get(fresh['plan_id'])
            with store.connect() as db:
                db.execute(
                    'UPDATE plans SET body=? WHERE id=?',
                    (json.dumps({'bad': True}), fresh['plan_id']),
                )
            with self.assertRaisesRegex(ValueError, '校验'):
                store.get(fresh['plan_id'])



class SearchOrderTests(unittest.TestCase):
    """候选方案的优先顺序、并行检查和提前结束。"""

    # 物品竖直放置；夹爪轴线垂直于物品轴线（与夹试管时一样），所以夹爪
    # 绕物品转到不同方向时，轴线在水平面里指向不同的方位。
    GOAL_5D = {'target_position_m': [.4, 0, .3], 'target_rpy_deg': [0, 0, 0]}

    def held_sideways(self):
        held, vertices, arm, config = fixture()
        held['pose_mode'] = '5d'
        attachment = np.eye(4)
        attachment[:3, :3] = [[0, 0, 1], [0, 1, 0], [-1, 0, 0]]
        held['object_T_tcp'] = attachment.tolist()
        return held, vertices, arm, config

    def search(self, reachable, goal=None, release=False):
        held, vertices, arm, config = self.held_sideways()
        attempts = []
        try:
            plan = object_motion_geometry.plan_object_motion(
                held,
                vertices,
                arm,
                config,
                goal or self.GOAL_5D,
                np.eye(4),
                reachable,
                lambda: None,
                release=release,
                diagnostics=attempts,
            )
        except ValueError as error:
            return error, attempts
        return plan, attempts

    def test_first_direction_is_checked_alone_and_kept_when_feasible(self):
        calls = []

        def reachable(points, timeout):
            calls.append(1)
            return True, {'verified': True}

        plan, attempts = self.search(reachable)
        # 转动最少的方向可行时只检查它一个，不为其余候选白费处理器。
        self.assertEqual(len(calls), 1)
        self.assertEqual(attempts[0]['spin_candidate'], 0)
        self.assertEqual(plan['route_offset_m'], [0., 0.])
        # 检查器没有做障碍物检查时，方案如实写明没有检查过。
        scope = plan['validation_scope']
        self.assertFalse(scope['table_and_pole_collision_checked'])
        self.assertFalse(scope['gripper_collision_checked'])

    def test_plan_records_that_table_and_pole_were_checked(self):
        def reachable(points, timeout):
            return True, {'verified': True, 'collision_check': {}}

        plan, _ = self.search(reachable)
        scope = plan['validation_scope']
        self.assertTrue(scope['table_and_pole_collision_checked'])
        self.assertTrue(scope['gripper_collision_checked'])
        self.assertTrue(scope['joint_intermediate_path_checked'])
        # 桌上的其他物品和另一台臂仍不在检查之内。
        self.assertFalse(scope['environment_collision_checked'])

    def test_other_directions_follow_outward_first_before_any_detour(self):
        def reachable(points, timeout):
            return False, {'reason': 'blocked'}

        error, attempts = self.search(reachable)
        self.assertIsInstance(error, ValueError)
        held, _, arm, _ = self.held_sideways()
        scores = object_motion_geometry.outward_scores(
            held,
            arm,
            motion_contract.checked_goal(dict(self.GOAL_5D), False),
            np.eye(4),
            24,
        )
        # 记录超过 64 条后会丢掉中间的，前 24 条保持不变。
        first = attempts[:24]
        # 先把 24 个方向各试一遍直接路线，然后才试绕行。
        self.assertTrue(all(a['route_offset_m'] == [0., 0.] for a in first))
        order = [a['spin_candidate'] for a in first]
        self.assertEqual(sorted(order), list(range(24)))
        self.assertEqual(order[0], 0)
        rest = [scores[spin] for spin in order[1:]]
        # 其余方向按“夹爪朝外”的程度从高到低（相差 0.02 以内视为相同）。
        self.assertTrue(
            all(a >= b - .02 for a, b in zip(rest, rest[1:])), rest
        )
        self.assertGreater(rest[0] - rest[-1], 1.)

    def test_orientation_failure_in_every_direction_ends_the_search(self):
        calls = []

        def reachable(points, timeout):
            calls.append(1)
            return (
                False,
                {
                    'reason': 'cannot turn',
                    'failure': {'waypoint': 'release_orientation_1_sample_1'},
                },
            )

        error, attempts = self.search(reachable, release=True)
        self.assertIsInstance(
            error, object_motion_geometry.OrientationUnreachable
        )
        self.assertIn('全部 24 个夹持方向都无法完成调姿', str(error))
        # 每个方向按“原地转、边走边转”各试一次：调姿不通时换转移路线
        # 没有用，所以不会再试绕行。
        turns = object_motion_geometry.TURN_FRACTIONS
        self.assertEqual(len(calls), 24 * len(turns))
        # 搜索记录最多保留 64 条，所以只核对留下来的都在预期范围内。
        self.assertEqual(len(attempts), 64)
        self.assertLessEqual(
            {(a['spin_candidate'], a['turn_at']) for a in attempts},
            {(spin, turn) for spin in range(24) for turn in turns},
        )
        self.assertTrue(all(a['route_offset_m'] == [0., 0.] for a in attempts))

    def release_goal(self):
        return {
            'target_position_m': [.4, -.1, .1],
            'target_rpy_deg': [0, 0, 0],
            'reference_point_object_m': [0, 0, -.06],
        }

    def test_turning_in_place_keeps_the_reference_point_still(self):
        held, vertices, arm, config = self.held_sideways()
        goal = motion_contract.checked_goal(self.release_goal(), True)
        plan = object_motion_geometry.build_candidate(
            held, vertices, arm, config, goal, np.eye(4), 3, np.zeros(3),
            True, np.array([0, 0, .05]),
        )
        turning = [
            w for w in plan['waypoints']
            if w['name'].startswith('release_orientation')
        ]
        self.assertGreater(len(turning), 1)
        first = np.array(turning[0]['reference_position_base_m'])
        for waypoint in turning:
            np.testing.assert_allclose(
                waypoint['reference_position_base_m'], first, atol=1e-9
            )

    def test_turning_while_moving_ends_turned_at_the_preplacement_point(self):
        held, vertices, arm, config = self.held_sideways()
        goal = motion_contract.checked_goal(self.release_goal(), True)

        def build(turn_at):
            return object_motion_geometry.build_candidate(
                held, vertices, arm, config, goal, np.eye(4), 3, np.zeros(3),
                True, np.array([0, 0, .05]), (0., 0.), turn_at,
            )

        in_place, moving = build(0.), build(1.)
        turning = [
            w for w in moving['waypoints']
            if w['name'].startswith('release_orientation')
        ]
        start = np.array(
            in_place['waypoints'][0]['reference_position_base_m']
        )
        preplacement = np.array([.4, -.1, .1 + .1])
        # 参考点沿“起点 → 预放置点”的直线前进，转完时正好到预放置点。
        np.testing.assert_allclose(
            turning[-1]['reference_position_base_m'], preplacement, atol=1e-9
        )
        for waypoint in turning:
            offset = np.array(waypoint['reference_position_base_m']) - start
            along = preplacement - start
            np.testing.assert_allclose(
                np.cross(offset, along), np.zeros(3), atol=1e-9
            )
        # 最终朝向和松爪位置与原地转的方案完全相同。
        np.testing.assert_allclose(
            moving['base_T_object_target'], in_place['base_T_object_target'],
            atol=1e-9,
        )
        self.assertEqual(moving['waypoints'][-1]['name'], 'release_descent_1')
        np.testing.assert_allclose(
            moving['waypoints'][-1]['reference_position_base_m'],
            [.4, -.1, .1],
            atol=1e-9,
        )
        # 走到一半时转完：转完的位置在中点。
        halfway = [
            w for w in build(.5)['waypoints']
            if w['name'].startswith('release_orientation')
        ][-1]
        np.testing.assert_allclose(
            halfway['reference_position_base_m'],
            (start + preplacement) / 2,
            atol=1e-9,
        )

    def test_turning_while_moving_is_used_when_turning_in_place_fails(self):
        real = object_motion_geometry.build_candidate

        def tagged(*args):
            plan = real(*args)
            plan['validation_points'][0]['turn_at'] = args[-1]
            return plan

        def reachable(points, timeout):
            if points[0]['turn_at'] == 0.:
                return (
                    False,
                    {
                        'reason': 'cannot turn here',
                        'failure': {
                            'waypoint': 'release_orientation_1_sample_1'
                        },
                    },
                )
            return True, {'verified': True}

        with mock.patch.object(
            object_motion_geometry, 'build_candidate', side_effect=tagged
        ):
            plan, attempts = self.search(
                reachable, goal=self.release_goal(), release=True
            )
        # 24 个方向原地都转不了之后，才用边走边转；不会去试绕行路线。
        self.assertEqual(plan['turn_at'], 1.)
        self.assertEqual(plan['route_offset_m'], [0., 0.])
        self.assertEqual(
            [a['turn_at'] for a in attempts[:24]], [0.] * 24
        )
        self.assertTrue(all(a['turn_at'] == 1. for a in attempts[24:]))

    def test_scores_are_zero_without_an_orientation_goal(self):
        held, _, arm, _ = fixture()
        goal = motion_contract.checked_goal(
            {'target_position_m': [.4, -.1, .3]}, False
        )
        self.assertEqual(
            object_motion_geometry.outward_scores(
                held, arm, goal, np.eye(4), 1
            ),
            [0.],
        )


class HullVerticesTests(unittest.TestCase):
    def test_hull_keeps_the_bounds_in_every_direction(self):
        random = np.random.default_rng(3)
        points = random.normal(size=(2000, 3)) * [.01, .01, .07]
        hull = object_motion_geometry.hull_vertices(points)
        self.assertLess(len(hull), len(points) // 5)
        for direction in random.normal(size=(50, 3)):
            self.assertAlmostEqual(
                float((points @ direction).max()),
                float((hull @ direction).max()),
                places=12,
            )
        # 绕 Z 轴自由旋转时的包围范围由（轴向位置，离轴距离）决定。
        for a, b in random.normal(size=(50, 2)):
            def support(p):
                return float(
                    (a * p[:, 2] + abs(b) * np.hypot(p[:, 0], p[:, 1])).max()
                )
            self.assertAlmostEqual(support(points), support(hull), places=12)

    def test_too_few_or_flat_points_are_returned_unchanged(self):
        few = np.zeros((8, 3))
        self.assertIs(object_motion_geometry.hull_vertices(few), few)
        flat = np.random.default_rng(1).normal(size=(40, 3)) * [1., 1., 0.]
        np.testing.assert_array_equal(
            object_motion_geometry.hull_vertices(flat), flat
        )


if __name__ == '__main__':
    unittest.main()
