import copy
import json
import pathlib
import tempfile
import time
import types
import unittest
from unittest import mock

import numpy as np

from saydo.geometry import grasp_geometry
from saydo.robot import grasp_robot
from saydo.manipulation import grasp_control
from tests.robot import fake_kinematics


class GraspTests(unittest.TestCase):
    def test_manual_open_releases_contact_and_rejects_conflicts(self):
        import fcntl
        with tempfile.TemporaryDirectory() as directory:
            root = pathlib.Path(directory)
            cfg = root / 'config.json'
            cfg.write_text(json.dumps(self.config()))
            service = grasp_control.GraspController(root, None, cfg)
            robot = self.force_robot()
            with mock.patch(
                'saydo.robot.grasp_robot.RobotClient',
                return_value=robot,
            ) as client:
                service.status['busy'] = True
                with self.assertRaisesRegex(ValueError, '进行中'):
                    service.open_gripper()
                service.status['busy'] = False
                service.mode = 'preview'
                with self.assertRaisesRegex(ValueError, '预览'):
                    service.open_gripper()
                client.assert_not_called()
                service.mode = 'execute'
                lock_path = root / 'work/grasp'
                lock_path.mkdir(parents=True)
                with (lock_path / 'robot.lock').open('a') as lock:
                    fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
                    with self.assertRaises(ValueError):
                        service.open_gripper()
                client.assert_not_called()
                original = robot.query.side_effect

                def holding(command):
                    reply = original(command)
                    if command == 'get_rm_plus_state_info':
                        reply['state_info'].update(
                            dof_state=[3], pos=[500], force=[10]
                        )
                    return reply

                robot.query.side_effect = holding
                self.assertTrue(service.open_gripper()['write_acknowledged'])
                robot.move_gripper.assert_called_once_with(1000)
                robot.move_linear.assert_not_called()
                robot.set_register.assert_not_called()
                robot.move_gripper.side_effect = TimeoutError('no reply')
                with self.assertRaisesRegex(ValueError, 'no reply'):
                    service.open_gripper()
                self.assertEqual(robot.move_gripper.call_count, 2)
                self.assertFalse(
                    json.loads(
                        sorted(
                            (lock_path / 'open_commands').glob('*.json')
                        )[-1].read_text()
                    )['write_acknowledged']
                )

                def moving(command):
                    reply = original(command)
                    if command == 'get_rm_plus_state_info':
                        reply['state_info']['speed'] = [1]
                    return reply

                robot.query.side_effect = moving
                with self.assertRaisesRegex(ValueError, '正在运动'):
                    service.open_gripper()
                self.assertEqual(robot.move_gripper.call_count, 2)

    def test_full_open_reset_is_independent_of_smaller_grasp_opening(self):
        grip = self.config()['gripper']
        grip['open_position'] = 300
        state = {
            'sys_err': 0,
            'sys_state': 0,
            'dof_err': [0],
            'dof_state': [3],
            'pos': [1000],
            'speed': [0],
            'force': [0],
        }
        for position in (997, 1000, 297, 300, 303):
            grasp_control.GraspController.check_gripper_start(
                {**state, 'pos': [position]}, grip
            )
        for position in (0, 296, 304, 500, 996):
            with self.subTest(position=position), self.assertRaisesRegex(
                ValueError, '当前位置'
            ):
                grasp_control.GraspController.check_gripper_start(
                    {**state, 'pos': [position]}, grip
                )
        for change in ({'speed': [1]}, {'dof_err': [1]}, {'sys_err': 1}):
            with self.subTest(change=change), self.assertRaises(ValueError):
                grasp_control.GraspController.check_gripper_start(
                    {**state, **change}, grip
                )

    def test_manual_open_allows_retained_force_stop_but_not_closed_contact(
        self
    ):
        state = {
            'sys_err': 0,
            'sys_state': 0,
            'dof_err': [0],
            'dof_state': [3],
            'pos': [1000],
            'speed': [0],
            'force': [0],
        }
        grip = self.config()['gripper']
        for force in (0, 10):
            for position in (997, 1000):
                grasp_control.GraspController.check_gripper_start(
                    {**state, 'pos': [position], 'force': [force]}, grip
                )
        for changes in (
            {'pos': [996]},
            {'pos': [500]},
            {'pos': [500], 'dof_state': [2], 'force': [10]},
        ):
            with self.subTest(changes=changes), self.assertRaisesRegex(
                ValueError, '人工张开复位'
            ):
                grasp_control.GraspController.check_gripper_start(
                    {**state, **changes}, grip
                )
        for changes in (
            {'speed': [1]},
            {'sys_err': 1},
            {'dof_state': [5]},
            {'pos': []},
            {'pos': [None]},
            {'pos': [float('nan')]},
            {'pos': [1001]},
        ):
            with self.subTest(changes=changes), self.assertRaises(ValueError):
                grasp_control.GraspController.check_gripper_start(
                    {**state, **changes}, grip
                )

    def test_start_freezes_one_observation_before_background_work(self):
        observation = {
            'object_id': 'any_tracked_object',
            'time': time.time(),
            'pose': np.eye(4).tolist(),
            'generation': 1,
        }
        tracker = types.SimpleNamespace(
            grasp_observation=mock.Mock(return_value=observation)
        )
        service = grasp_control.GraspController('.', tracker, 'unused')
        with mock.patch(
            'saydo.manipulation.grasp_control.threading.Thread'
        ) as thread:
            service.start()
            locked = thread.call_args.kwargs['args'][0]
            observation['pose'][0][3] = 10
            observation['object_id'] = 'another_object'
            self.assertEqual(locked['pose'][0][3], 0)
            self.assertEqual(locked['object_id'], 'any_tracked_object')
            self.assertTrue(service.status['busy'])
            tracker.grasp_observation.assert_called_once_with()
            thread.return_value.start.assert_called_once_with()

    def test_missing_initial_tracking_never_starts_job(self):
        tracker = types.SimpleNamespace(
            grasp_observation=mock.Mock(
                side_effect=ValueError('没有可靠跟踪位姿')
            )
        )
        service = grasp_control.GraspController('.', tracker, 'unused')
        with mock.patch(
            'saydo.manipulation.grasp_control.threading.Thread'
        ) as thread:
            with self.assertRaisesRegex(ValueError, '没有可靠跟踪位姿'):
                service.start()
            thread.assert_not_called()
            self.assertFalse(service.status['busy'])

    def test_initial_snapshot_requires_fresh_image_and_stationary_arm(self):
        service = grasp_control.GraspController('.', None, 'unused')
        arm = {'joint': [0] * 7, 'pose': [0, 0, 200000, 0, 0, 0]}
        with mock.patch.object(
            service, 'arm_state', return_value=arm
        ) as read, mock.patch('saydo.manipulation.grasp_control.time.sleep'):
            with self.assertRaisesRegex(ValueError, '过期'):
                service.paired_observation({'time': time.time() - 1})
            read.assert_not_called()
            observation = {'time': time.time()}
            self.assertIs(
                service.paired_observation(observation)[1], observation
            )
            moved = copy.deepcopy(arm)
            moved['pose'][0] = 1000
            read.side_effect = [arm, arm, moved]
            with self.assertRaisesRegex(ValueError, '未停稳'):
                service.paired_observation({'time': time.time()})

    def config(self):
        return {
            'robot_ip': 'test',
            'robot_port': 8080,
            'handeye_file': 'unused',
            'tool_file': 'unused',
            'nominal_tcp_user_accepted': True,
            'action': 'grasp_and_lift',
            'lift_height_m': .10,
            'pregrasp_distance_m': .05,
            'clearance_height_m': .08,
            'max_target_distance_m': .3,
            'arm_speed_percent': 2,
            'workspace_reviewed': True,
            'workspace_min_m': [-1, -1, .05],
            'workspace_max_m': [1, 1, 1],
            'table_z_m': 0,
            'gripper': {
                'vendor': 'ZX',
                'device_id': 1,
                'force_control_verified': True,
                'force_limit_raw': 10,
                'force_target_raw': 10,
                'force_units': 'native',
                'open_position': 1000,
                'closed_position': 0,
                'contact_position_margin': 20,
                'close_timeout_s': 2,
            },
            'grasp_profile': {
                'grasp_point_object_m': [0, 0, 0],
                'orientation': 'keep_current_tcp',
            },
        }

    def test_rotated_tcp_offset_is_inverted_as_transform(self):
        base_flange = grasp_geometry.controller_pose(
            [100000, 200000, 300000, 1571, 0, 0]
        )
        flange_tcp = np.eye(4)
        flange_tcp[2, 3] = .2225
        flange_camera = np.eye(4)
        flange_camera[0, 3] = .07
        camera_object = np.eye(4)
        camera_object[2, 3] = .4
        got = grasp_geometry.object_in_base(
            base_flange @ flange_tcp, flange_tcp, flange_camera, camera_object
        )
        np.testing.assert_allclose(
            got, base_flange @ flange_camera @ camera_object, atol=1e-10
        )

    def test_plan_starts_at_directional_pregrasp_and_lifts_ten_cm(self):
        target = np.eye(4)
        target[:3, 3] = [.1, 0, .2]
        plan = grasp_geometry.plan_grasp(
            [0, 0, 200000, 0, 0, 0],
            target,
            self.config()['grasp_profile'],
            self.config(),
        )
        self.assertTrue(plan['lift_after_grasp'])
        self.assertEqual(
            [w['name'] for w in plan['waypoints']],
            ['pregrasp', 'grasp', 'lift'],
        )
        self.assertEqual(plan['waypoints'][-1]['name'], 'lift')
        self.assertEqual(
            plan['waypoints'][-1]['controller_pose'],
            [100000, 0, 300000, 0, 0, 0],
        )
        self.assertFalse(grasp_geometry.workspace_blockers(plan, self.config()))
        config = self.config()
        config['workspace_max_m'][2] = .29
        # Only the lift exceeds this ceiling.
        self.assertTrue(grasp_geometry.workspace_blockers(plan, config))
        for height in (0, -0.1, float('nan')):
            config = self.config()
            config['lift_height_m'] = height
            with self.assertRaises(ValueError):
                grasp_geometry.plan_grasp(
                    [0, 0, 200000, 0, 0, 0],
                    target,
                    config['grasp_profile'],
                    config,
                )
        config = self.config()
        config['workspace_reviewed'] = False
        self.assertTrue(grasp_geometry.workspace_blockers(plan, config))

    def test_table_clearance_ten_mm_for_start_and_waypoints(self):
        config = self.config()
        config['workspace_min_m'] = [-1, -1, 0]
        for height, blocked in [(0.009, True), (0.010, False), (0.011, False)]:
            for at_start in (True, False):
                with self.subTest(height=height, at_start=at_start):
                    plan = {
                        'current_position_m': [
                            0, 0, height if at_start else .2
                        ],
                        'waypoints': [
                            {'position_m': [0, 0, .2 if at_start else height]}
                        ],
                    }
                    self.assertEqual(
                        bool(grasp_geometry.workspace_blockers(plan, config)),
                        blocked,
                    )

    def test_force_contact_is_distinct_from_empty_close_or_current_protection(
        self
    ):
        state = {
            'sys_err': 0,
            'sys_state': 0,
            'dof_err': [0],
            'dof_state': [3],
            'pos': [500],
            'speed': [0],
            'force': [10],
        }
        grip = self.config()['gripper']
        self.assertEqual(
            grasp_geometry.contact_result(state, grip), 'force_contact'
        )
        self.assertEqual(
            grasp_geometry.contact_result(
                {**state, 'dof_state': [2], 'pos': [0]}, grip
            ),
            'empty_close',
        )
        self.assertEqual(
            grasp_geometry.contact_result({**state, 'dof_state': [5]}, grip),
            'fault',
        )
        self.assertEqual(
            grasp_geometry.contact_result({**state, 'force': [100]}, grip),
            'force_contact',
        )
        self.assertEqual(
            grasp_geometry.contact_result({**state, 'force': [0]}, grip),
            'force_stop',
        )

    def test_acknowledgement_without_register_data_is_rejected(self):
        robot = grasp_robot.RobotClient('test')
        with mock.patch.object(
            robot, 'query', return_value={'get_state': True}
        ):
            with self.assertRaisesRegex(ValueError, '实际数值'):
                robot.register(1220)
        with self.assertRaises(ValueError):
            robot.move_gripper(1000)
        with self.assertRaises(ValueError):
            robot.query('movel')

    def test_verified_legacy_device_executes_without_readback_and_preview_does_not(
        self
    ):
        for mode, preview in [
            ('preview', False), ('execute', False), ('execute', True)
        ]:
            with self.subTest(
                mode=mode, preview=preview
            ), tempfile.TemporaryDirectory() as directory:
                root = pathlib.Path(directory)
                config = self.config()
                config['gripper']['force_control_verified'] = True
                config['grasp_profile'].update(
                    orientation='follow_object', object_R_tcp=np.eye(3).tolist()
                )
                cfg = root / 'grasp.json'
                cfg.write_text(json.dumps(config))
                pose = np.eye(4)
                pose[0, 3] = .1
                observation = {
                    'pose': pose.tolist(),
                    'time': time.time(),
                    'generation': 1,
                    'object_id': 'never_configured_paper_cup',
                    'K': np.diag([.5, .5, 1]).tolist(),
                }
                tracker = types.SimpleNamespace(
                    grasp_observation=mock.Mock(
                        side_effect=AssertionError('must not reread tracking')
                    )
                )
                service = grasp_control.GraspController(
                    root, tracker, cfg, mode
                )
                service.force_override = 17
                arm = {
                    'pose': [0, 0, 200000, 0, 0, 0],
                    'joint': [0] * 7,
                    'err': [0],
                }
                tool_raw = {'tool_name': 'grip_tcp', 'pose': [0] * 6}
                tool = {
                    'controller_readback': {'tool_response_raw': tool_raw},
                    'flange_T_grip_tcp': np.eye(4).tolist(),
                }
                handeye = {
                    'installation': {'controller_firmware': 'V1.7.0'},
                    'flange_T_color_optical_camera': np.eye(4).tolist(),
                    'camera': {'K': np.eye(3).tolist()},
                }
                replies = {
                    'get_arm_software_info': {
                        'Product_version': 'RM75-BI',
                        'ctrl_info': {'version': 'V1.7.0'},
                    },
                    'get_current_tool_frame': tool_raw,
                    'get_current_work_frame': {
                        'frame_name': 'Base', 'pose': [0] * 6
                    },
                    'get_joint_en_state': {'en_state': [1] * 7},
                    'get_joint_err_flag': {'err_flag': [0] * 7},
                    'get_rm_plus_mode': {'mode': 115200},
                    'get_rm_plus_base_info': {
                        'base_info': {
                            'manu': 'ZX',
                            'id': 1,
                            'dof': 1,
                            'type': 1,
                            'force': True,
                            'force_up': [100],
                        }
                    },
                    'get_rm_plus_state_info': {
                        'state_info': {
                            'sys_err': 0,
                            'sys_state': 0,
                            'dof_err': [0],
                            'dof_state': [3],
                            'speed': [0],
                            'force': [0],
                            'pos': [1000],
                        }
                    },
                }
                fake = mock.Mock()
                fake.query.side_effect = lambda command: copy.deepcopy(
                    replies[command]
                )
                fake.register.side_effect = AssertionError(
                    'numeric readback must not be required'
                )
                with mock.patch(
                    'saydo.robot.grasp_robot.RobotClient',
                    return_value=fake,
                ) as client, mock.patch.object(
                    service, 'calibration', return_value=(handeye, tool)
                ), mock.patch.object(
                    service,
                    'paired_observation',
                    return_value=(arm, observation),
                ), mock.patch.object(
                    service,
                    'verify_motion_targets',
                    return_value={'joint_travel_deg': 10},
                ) as verify, mock.patch.object(service, 'execute') as execute:
                    service.run(observation, preview=preview)
                    self.assertEqual(
                        client.call_args.kwargs['writes'],
                        mode == 'execute' and not preview,
                    )
                    self.assertEqual(
                        execute.call_count,
                        int(mode == 'execute' and not preview),
                    )
                    self.assertEqual(
                        service.status['plan']['can_execute'],
                        mode == 'execute' and not preview,
                    )
                    self.assertEqual(
                        service.status['plan']['force_limit_raw'], 10
                    )
                    self.assertEqual(
                        service.status['plan'][
                            'effective_parameters'
                        ]['open_position'],
                        1000,
                    )
                    self.assertFalse(
                        service.status['plan']['motion_validation'][
                            'offline_path_checked'
                        ]
                    )
                    self.assertTrue(
                        service.status['plan'][
                            'motion_validation'
                        ]['ik_fk_checked']
                    )
                    verify.assert_called_once()
                    execute.reset_mock()
                    verify.side_effect = ValueError(
                        '预抓点正解位置误差 200 毫米'
                    )
                    service.run(observation, preview=preview)
                    execute.assert_not_called()
                    self.assertEqual(service.status['state'], 'failed')
                    fake.set_register.assert_not_called()
                    fake.move_gripper.assert_not_called()

    def force_robot(self):
        robot = mock.Mock()
        replies = {
            'get_rm_plus_mode': {'mode': 115200},
            'get_rm_plus_base_info': {
                'base_info': {
                    'manu': 'ZX',
                    'id': 1,
                    'type': 1,
                    'dof': 1,
                    'force': True,
                    'force_up': [100],
                }
            },
            'get_rm_plus_state_info': {
                'state_info': {
                    'sys_err': 0,
                    'sys_state': 0,
                    'dof_err': [0],
                    'dof_state': [2],
                    'speed': [0],
                    'pos': [1000],
                    'force': [0],
                }
            },
        }
        robot.query.side_effect = lambda command: copy.deepcopy(
            replies[command]
        )
        return robot

    def test_force_input_writes_hardware_and_preserves_session_value(self):
        with tempfile.TemporaryDirectory() as directory:
            root = pathlib.Path(directory)
            cfg = root / 'grasp.json'
            original = json.dumps(self.config())
            cfg.write_text(original)
            service = grasp_control.GraspController(root, None, cfg)
            robot = self.force_robot()
            with mock.patch(
                'saydo.robot.grasp_robot.RobotClient',
                return_value=robot,
            ) as client:
                for invalid in (0, 101, -1, True, 1.5, '10', None):
                    with self.assertRaises(ValueError):
                        service.set_force_config(invalid)
                client.assert_not_called()
                for value in (1, 100, 12):
                    result = service.set_force_config(value)
                    self.assertEqual(result['value'], value)
                    self.assertTrue(result['write_acknowledged'])
                    self.assertFalse(result['readback_verified'])
                    robot.set_register.assert_called_with(1220, value)
                self.assertEqual(robot.set_register.call_count, 3)
                robot.move_gripper.assert_not_called()
                robot.move_linear.assert_not_called()
                robot.register.assert_not_called()
                service.update(busy=True)
                with self.assertRaises(ValueError):
                    service.set_force_config(20)
                self.assertEqual(
                    service.snapshot()['force_config']['value'], 12
                )
            self.assertEqual(cfg.read_text(), original)
            self.assertFalse(
                grasp_control.GraspController(root, None, cfg).force_config()[
                    'hardware_applied'
                ]
            )
            self.assertEqual(
                len(list((root / 'work/grasp/force_commands').glob('*.json'))),
                3,
            )

    def test_force_failure_does_not_commit_value_or_retry(self):
        with tempfile.TemporaryDirectory() as directory:
            root = pathlib.Path(directory)
            cfg = root / 'config.json'
            cfg.write_text(json.dumps(self.config()))
            service = grasp_control.GraspController(root, None, cfg)
            robot = self.force_robot()
            with mock.patch(
                'saydo.robot.grasp_robot.RobotClient',
                return_value=robot,
            ):
                service.set_force_config(10)
                robot.set_register.side_effect = TimeoutError('no reply')
                with self.assertRaisesRegex(ValueError, 'no reply'):
                    service.set_force_config(50)
            self.assertEqual(robot.set_register.call_count, 2)
            self.assertEqual(service.force_config()['value'], 10)
            self.assertFalse(service.force_config()['hardware_applied'])
            self.assertIn('no reply', service.force_config()['error'])
            robot.move_gripper.assert_not_called()

    def test_preview_force_update_never_connects(self):
        with tempfile.TemporaryDirectory() as directory:
            root = pathlib.Path(directory)
            cfg = root / 'config.json'
            cfg.write_text(json.dumps(self.config()))
            service = grasp_control.GraspController(root, None, cfg, 'preview')
            with mock.patch('saydo.robot.grasp_robot.RobotClient') as client:
                self.assertEqual(service.set_force_config(50)['value'], 50)
                client.assert_not_called()
            self.assertFalse(service.force_config()['hardware_applied'])

    def test_force_write_is_blocked_by_another_grasp_process(self):
        import fcntl
        with tempfile.TemporaryDirectory() as directory:
            root = pathlib.Path(directory)
            cfg = root / 'config.json'
            cfg.write_text(json.dumps(self.config()))
            path = root / 'work/grasp'
            path.mkdir(parents=True)
            service = grasp_control.GraspController(root, None, cfg)
            with (path / 'robot.lock').open('a') as lock, mock.patch(
                'saydo.robot.grasp_robot.RobotClient'
            ) as client:
                fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
                with self.assertRaises(ValueError):
                    service.set_force_config(10)
                client.assert_not_called()

    def test_gripper_position_transport_and_force_rejection(self):
        robot = grasp_robot.RobotClient('test', writes=True)
        with mock.patch.object(
            robot, '_request', return_value={'set_state': True}
        ) as request:
            robot.move_gripper(1000)
            request.assert_called_with(
                {'command': 'hand_follow_pos', 'hand_pos': [1000]}
            )
            robot.move_gripper(0)
            request.assert_called_with(
                {'command': 'hand_follow_pos', 'hand_pos': [0]}
            )
        with mock.patch.object(
            robot, '_request', return_value={'set_state': False}
        ):
            with self.assertRaises(RuntimeError):
                robot.set_register(1220, 10)

    def test_default_execute_requires_explicit_start_and_preview_remains_available(
        self
    ):
        with tempfile.TemporaryDirectory() as directory:
            root = pathlib.Path(directory)
            cfg = root / 'grasp.json'
            cfg.write_text(json.dumps(self.config()))
            with mock.patch('saydo.robot.grasp_robot.RobotClient') as client:
                service = grasp_control.GraspController(root, None, cfg)
                self.assertEqual(service.snapshot()['mode'], 'execute')
                self.assertTrue(service.snapshot()['motion_enabled'])
                self.assertFalse(service.snapshot()['busy'])
                self.assertIsNone(service.thread)
                preview = grasp_control.GraspController(
                    root, None, cfg, 'preview'
                )
                self.assertFalse(preview.snapshot()['motion_enabled'])
                client.assert_not_called()

    def test_executor_lifts_only_after_contact(self):
        with tempfile.TemporaryDirectory() as directory:
            root = pathlib.Path(directory)
            config = self.config()
            cfg = root / 'config.json'
            cfg.write_text(json.dumps(config))
            observation = {
                'pose': np.eye(4).tolist(),
                'time': time.time(),
                'generation': 1,
                'object_id': 'never_configured_paper_cup',
            }
            service = grasp_control.GraspController(
                root,
                types.SimpleNamespace(
                    grasp_observation=mock.Mock(
                        side_effect=AssertionError('tracking lost')
                    )
                ),
                cfg,
                'execute',
            )
            arm = {
                'pose': [0, 0, 200000, 0, 0, 0], 'joint': [0] * 7, 'err': [0]
            }
            target = np.eye(4)
            target[:3, 3] = [.1, 0, .2]
            plan = grasp_geometry.plan_grasp(
                arm['pose'], target, config['grasp_profile'], config
            )
            plan.update(
                current_arm=copy.deepcopy(arm),
                gripper_base={'force_up': [100], 'speed_up': [500]},
                tool_response={},
                work_response={},
            )

            # 沿线取样“求不出连续解”：接近和抬升退回控制器的直线运动。
            # 沿取样点逐点关节运动的情形见
            # test_approach_and_lift_follow_sampled_joints。
            kinematics = fake_kinematics.PairedKinematics(
                sampled_lines_fail=True
            )

            class FakeRobot:
                def __init__(self):
                    self.calls = []
                    self.registers = {}
                    self.contact = False

                def query(self, command):
                    if command == 'get_current_arm_state':
                        return {'arm_state': copy.deepcopy(arm)}
                    if (
                        command
                        in ('get_current_tool_frame', 'get_current_work_frame')
                    ):
                        return {}
                    return {
                        'state_info': {
                            'sys_err': 0,
                            'sys_state': 0,
                            'dof_err': [0],
                            'dof_state': [3 if self.contact else 2],
                            'pos': [500 if self.contact else 1000],
                            'force': [10 if self.contact else 0],
                            'speed': [0],
                        }
                    }

                def set_register(self, address, value):
                    self.registers[address] = value
                    self.calls.append(('register', address, value))

                def register(self, address):
                    return self.registers[address]

                def move_gripper(self, position):
                    self.calls.append(('position', position))
                    self.contact = position == 0

                def move_joints(self, joints, speed, cancelled, watchdog):
                    watchdog()
                    pose = kinematics.pose_for(joints)
                    self.calls.append(('move', pose))
                    arm.update(pose=list(pose), joint=list(joints))

                def move_linear(self, pose, speed, cancelled, watchdog):
                    watchdog()
                    self.calls.append(('move', pose))
                    arm['pose'] = list(pose)

                def move_joints_through(
                    self, targets, speed, cancelled, watchdog
                ):
                    for target in targets:
                        self.move_joints(target, speed, cancelled, watchdog)

            # 替身没有 move_joint_pose：预抓点若仍用位姿关节运动，
            # 这里会直接报属性错误。
            robot = FakeRobot()
            service.robot = robot
            service.verify_motion_targets = mock.Mock(
                side_effect=kinematics.verify_motion_targets
            )
            report = {}
            service.execute(plan, config, report)
            self.assertEqual(service.status['state'], 'completed')
            moves = [x for x in robot.calls if x[0] == 'move']
            self.assertEqual(len(moves), 3)
            # 预抓点查一次；接近、抬升各先试沿线取样（失败），再查终点。
            self.assertEqual(service.verify_motion_targets.call_count, 5)
            self.assertEqual(
                [row['stage'] for row in report['linear_fallbacks']],
                ['grasp', 'lift'],
            )
            self.assertEqual(
                moves[-1][1], plan['waypoints'][-1]['controller_pose']
            )
            self.assertEqual(
                robot.calls[:2], [('register', 1220, 10), ('position', 1000)]
            )
            self.assertEqual(
                robot.calls[-3:-1], [('register', 1220, 10), ('position', 0)]
            )
            self.assertEqual(
                moves[0][1], plan['waypoints'][0]['controller_pose']
            )
            self.assertEqual(
                len(
                    [
                        x
                        for x in robot.calls[
                            :robot.calls.index(('position', 0))
                        ]
                        if x[0] == 'move'
                    ]
                ),
                2,
            )
            self.assertLess(
                next(
                    i
                    for i, x in enumerate(robot.calls)
                    if x == ('position', 1000)
                ),
                next(i for i, x in enumerate(robot.calls) if x[0] == 'move'),
            )
            robot.calls.clear()
            arm.update(pose=[0, 0, 200000, 0, 0, 0], joint=[0] * 7)
            with mock.patch.object(
                robot, 'set_register', side_effect=RuntimeError('rejected')
            ):
                with self.assertRaisesRegex(RuntimeError, 'rejected'):
                    service.execute(plan, config, {})
            self.assertEqual(robot.calls, [])
            original_query = robot.query
            for stop_state in (2, 5):
                robot.calls.clear()
                arm.update(pose=[0, 0, 200000, 0, 0, 0], joint=[0] * 7)

                def failed_close(command):
                    reply = original_query(command)
                    if command == 'get_rm_plus_state_info' and robot.contact:
                        reply['state_info'].update(
                            dof_state=[stop_state], pos=[0]
                        )
                    return reply

                with mock.patch.object(
                    robot, 'query', side_effect=failed_close
                ):
                    with self.assertRaisesRegex(
                        ValueError, '未确认接触|电流保护'
                    ):
                        service.execute(plan, config, {})
                self.assertEqual(
                    len([c for c in robot.calls if c[0] == 'move']), 2
                )
            robot.calls.clear()
            arm.update(pose=[0, 0, 200000, 0, 0, 0], joint=[0] * 7)
            move = robot.move_linear

            def wrong_orientation(*args):
                move(*args)
                # About 5.7 degrees, above the 2-degree tolerance.
                arm['pose'][3] += 100

            with mock.patch.object(
                robot, 'move_linear', side_effect=wrong_orientation
            ):
                with self.assertRaisesRegex(ValueError, '朝向'):
                    service.execute(plan, config, {})
            self.assertNotIn(('position', 0), robot.calls)

    def solved_joint_fixture(self, root, stop_after_pregrasp=True):
        """页面抓取的最小场景。

        只在两处边界打替身：逆解子进程和控制器。路径重查、取关节解、
        下发、核对到位都是真实代码。`stop_after_pregrasp` 为真时，
        预抓点之后的第一条运动指令由替身抛出异常收尾，供只关心预抓点
        这一段的测试使用；为假时夹爪闭合后报告接触，流程可以走完。
        """
        config = self.config()
        config['ik_checker'] = 'checker'
        (root / 'checker').touch()
        cfg = root / 'config.json'
        cfg.write_text(json.dumps(config))
        service = grasp_control.GraspController(
            root, types.SimpleNamespace(), cfg, 'execute'
        )
        arm = {'pose': [0, 0, 200000, 0, 0, 0], 'joint': [0] * 7, 'err': [0]}
        target = np.eye(4)
        target[:3, 3] = [.1, 0, .2]
        plan = grasp_geometry.plan_grasp(
            arm['pose'], target, config['grasp_profile'], config
        )
        plan.update(
            current_arm=copy.deepcopy(arm),
            gripper_base={'force_up': [100], 'speed_up': [500]},
            tool_response={},
            work_response={},
            flange_T_grip_tcp=np.eye(4).tolist(),
        )
        kinematics = fake_kinematics.PairedKinematics()
        robot = mock.Mock()
        closed = []

        def query(command):
            if command == 'get_current_arm_state':
                return {'arm_state': copy.deepcopy(arm)}
            return {
                'state_info': {
                    'sys_err': 0,
                    'sys_state': 0,
                    'dof_err': [0],
                    'dof_state': [3 if closed else 2],
                    'pos': [500 if closed else 1000],
                    'force': [10 if closed else 0],
                    'speed': [0],
                }
            }

        def move_joints(joints, speed, cancelled, watchdog):
            if stop_after_pregrasp and robot.move_joints.call_count > 1:
                raise RuntimeError('到直线接近为止')
            arm.update(pose=kinematics.pose_for(joints), joint=list(joints))

        def move_gripper(position):
            if position == 0:
                closed.append(True)

        robot.query.side_effect = query
        robot.move_joints.side_effect = move_joints
        robot.move_gripper.side_effect = move_gripper
        fake_kinematics.emulate_move_joints_through(robot)
        robot.move_linear.side_effect = RuntimeError('到直线接近为止')
        service.robot = robot
        return service, config, plan, arm, kinematics, robot

    def test_pregrasp_moves_to_joints_solved_for_the_remaining_path(self):
        with tempfile.TemporaryDirectory() as directory:
            service, config, plan, arm, kinematics, robot = (
                self.solved_joint_fixture(pathlib.Path(directory))
            )
            report = {}
            with mock.patch(
                'saydo.robot.motion_validation.check_waypoints',
                side_effect=kinematics.check_waypoints,
            ) as check:
                with self.assertRaisesRegex(RuntimeError, '到直线接近为止'):
                    service.execute(plan, config, report)
            pregrasp, grasp = plan['waypoints'][:2]
            pregrasp_joint = kinematics.joint_raw_for(
                pregrasp['controller_pose']
            )
            # 预抓点：关节运动，目标是逆解给出的关节角；不再把位姿交给
            # 控制器自己去选手臂摆法。
            self.assertEqual(
                robot.move_joints.call_args_list[0],
                mock.call(
                    pregrasp_joint,
                    config['arm_speed_percent'],
                    service.cancelled,
                    mock.ANY,
                ),
            )
            robot.move_joint_pose.assert_not_called()
            self.assertEqual(arm['pose'], pregrasp['controller_pose'])
            first, second = check.call_args_list
            # 发令前查的是从这一段起的全部路点，且带着障碍物。
            self.assertEqual(
                [w['name'] for w in first.args[3]],
                ['pregrasp', 'grasp', 'lift'],
            )
            self.assertIsNotNone(first.kwargs['environment'])
            # 下一段从实际到达的关节角出发重查。
            self.assertEqual(second.args[2]['joint'], pregrasp_joint)
            # 接近段沿直线取样，后面跟着剩余的路点。
            names = [w['name'] for w in second.args[3]]
            self.assertGreater(len(names), 2)
            self.assertTrue(all(n.startswith('grasp:') for n in names[:-1]))
            self.assertEqual(names[-1], 'lift')
            self.assertEqual(
                report['sampled_joint_segments'][0]['final_joint_raw'],
                pregrasp_joint,
            )

    def test_grasp_stops_when_arm_misses_the_solved_pregrasp_joints(self):
        with tempfile.TemporaryDirectory() as directory:
            service, config, plan, arm, kinematics, robot = (
                self.solved_joint_fixture(pathlib.Path(directory))
            )
            reach = robot.move_joints.side_effect

            def miss(joints, *rest):
                reach(joints, *rest)
                # 0.6 度，超过 0.5 度的到位界限。
                arm['joint'][1] += 600

            robot.move_joints.side_effect = miss
            with mock.patch(
                'saydo.robot.motion_validation.check_waypoints',
                side_effect=kinematics.check_waypoints,
            ):
                with self.assertRaisesRegex(ValueError, '未到达所选关节解'):
                    service.execute(plan, config, {})
            robot.move_linear.assert_not_called()
            robot.move_gripper.assert_called_once_with(1000)

    def test_approach_and_lift_follow_sampled_joints(self):
        with tempfile.TemporaryDirectory() as directory:
            service, config, plan, arm, kinematics, robot = (
                self.solved_joint_fixture(
                    pathlib.Path(directory), stop_after_pregrasp=False
                )
            )
            report = {}
            poses = []
            reach = robot.move_joints.side_effect

            def record(joints, *rest):
                reach(joints, *rest)
                poses.append(np.array(arm['pose'][:3]) / 1e6)

            robot.move_joints.side_effect = record
            with mock.patch(
                'saydo.robot.motion_validation.check_waypoints',
                side_effect=kinematics.check_waypoints,
            ):
                service.execute(plan, config, report)
            self.assertEqual(service.status['state'], 'completed')
            # 接近和抬升都不再交给控制器走直线。
            robot.move_linear.assert_not_called()
            self.assertNotIn('linear_fallbacks', report)
            pregrasp, grasp, lift = (
                np.array(w['controller_pose'][:3]) / 1e6
                for w in plan['waypoints']
            )
            segments = report['sampled_joint_segments']
            self.assertEqual(len(segments), 3)
            approach, raising = (
                segments[1]['joint_moves'], segments[2]['joint_moves']
            )
            # 每 10 毫米一个取样点。
            self.assertEqual(
                approach, int(np.ceil(np.linalg.norm(grasp - pregrasp) / .01))
            )
            self.assertEqual(
                raising, int(np.ceil(np.linalg.norm(lift - grasp) / .01))
            )
            np.testing.assert_allclose(poses[0], pregrasp, atol=1e-6)
            np.testing.assert_allclose(poses[approach], grasp, atol=1e-6)
            np.testing.assert_allclose(poses[-1], lift, atol=1e-6)
            # 取样点都在各自的直线上，相邻两点不超过 10 毫米。
            for start, end, points in (
                (pregrasp, grasp, poses[1:approach + 1]),
                (grasp, lift, poses[approach + 1:]),
            ):
                direction = (end - start) / np.linalg.norm(end - start)
                previous = start
                for point in points:
                    offset = point - start
                    np.testing.assert_allclose(
                        offset, direction * (offset @ direction), atol=2e-6
                    )
                    self.assertLessEqual(
                        np.linalg.norm(point - previous), .01 + 2e-6
                    )
                    previous = point
            # 夹爪在到达夹持点之后、抬升之前闭合。
            self.assertEqual(
                [c.args[0] for c in robot.move_gripper.call_args_list],
                [1000, 0],
            )

    def test_line_without_joint_solution_falls_back_to_controller_line(self):
        with tempfile.TemporaryDirectory() as directory:
            service, config, plan, arm, kinematics, robot = (
                self.solved_joint_fixture(
                    pathlib.Path(directory), stop_after_pregrasp=False
                )
            )
            kinematics.sampled_lines_fail = True
            report = {}
            with mock.patch(
                'saydo.robot.motion_validation.check_waypoints',
                side_effect=kinematics.check_waypoints,
            ), self.assertLogs(
                'saydo.manipulation.grasp_control', 'WARNING'
            ) as logs:
                # 替身的直线运动抛出这个异常：说明确实退回了直线运动。
                with self.assertRaisesRegex(RuntimeError, '到直线接近为止'):
                    service.execute(plan, config, report)
            robot.move_linear.assert_called_once_with(
                plan['waypoints'][1]['controller_pose'],
                config['arm_speed_percent'],
                service.cancelled,
                mock.ANY,
            )
            self.assertEqual(robot.move_joints.call_count, 1)
            (fallback,) = report['linear_fallbacks']
            self.assertEqual(fallback['stage'], 'grasp')
            self.assertIn('直线取样无连续解', fallback['reason'])
            self.assertIn('退回控制器的直线运动', logs.output[0])
            # 退回之前，终点仍按原来的方式查过一遍。
            self.assertEqual(
                report['kinematics_checks'][-1]['waypoints'], ['grasp', 'lift']
            )


if __name__ == '__main__':
    unittest.main()
