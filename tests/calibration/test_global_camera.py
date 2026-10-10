"""全局相机快速标定：相机画面是按针孔模型合成的，其余是真实代码。"""

import http.client
import http.server
import json
import pathlib
import tempfile
import threading
import types
import unittest

import cv2
import numpy as np
import yaml

from saydo.calibration import global_camera
from saydo.camera_monitor import server as monitor
from saydo.robot import arms
from saydo.robot import collision_model
from tests.calibration import board_render
from tests.camera_monitor import fake_realsense

OBSERVATION_DEG = [-115.4, -54.4, 81.5, 57.5, 29.0, 110.5, 195.5]
HAND_K = np.array([[454., 0., 320.], [0., 454., 180.], [0., 0., 1.]])
GLOBAL_K = np.array([[654., 0., 647.], [0., 653., 365.], [0., 0., 1.]])
# 预设的真实位置：全局相机在桌子一侧的高处斜着往下看。
GLOBAL_POSE = board_render.looking_at(
    [.30, -.75, .80], [.35, 0., 0.], [0., 0., 1.]
)
# 手上相机在桌面上方正对着往下看。测试里放得比实际高，视野才容得下
# 标定板在两个方向上挪动。
HAND_POSE = board_render.looking_at(
    [.34, .10, .95], [.34, .10, 0.], [1., 0., 0.]
)
# 第二台臂的基座：与第一台背对背、相距半米。
ARM1_T_ARM2 = np.array(
    [[-1., 0., 0., 0.], [0., -1., 0., -.5], [0., 0., 1., 0.], [0., 0., 0., 1.]]
)
HAND2_POSE = board_render.looking_at(
    [.40, .14, .95], [.34, .10, 0.], [-1., 0., 0.]
)


def board_on_table(x_m, y_m, yaw_deg):
    """标定板平放在桌面（Z=0）上的位姿，(x, y) 是板的中心。

    图案朝上，所以板的 Z 轴朝下。
    """
    pose = np.eye(4)
    pose[:3, :3] = (
        cv2.Rodrigues(np.array([0., 0., np.radians(yaw_deg)]))[0]
        @ np.diag([1., -1., -1.])
    )
    # 板的坐标原点在第一个内角点，中心在 (3.5 格, 5 格) 处。
    pose[:3, 3] = [x_m, y_m, 0.] - pose[:3, :3] @ [.105, .15, 0.]
    return pose


# 八处摆位，在两个方向上都铺开 15 厘米以上。
PLACEMENTS = [
    (.26, -.10, 0), (.42, .00, 20), (.30, .12, -15), (.40, .25, 90),
    (.34, -.22, 35), (.28, .30, 110), (.42, -.14, 60), (.34, .05, -30),
]


class GlobalCameraTestCase(unittest.TestCase):
    def setUp(self):
        directory = tempfile.TemporaryDirectory()
        self.addCleanup(directory.cleanup)
        self.root = pathlib.Path(directory.name)
        self.write_arm('arm1', HAND_POSE)
        self.session = global_camera.Session(self.root, ['arm1'], None)

    def write_arm(self, arm_id, base_T_camera, shift_m=(0., 0., 0.)):
        """写出一台臂的配置，使它在观测姿态下的手上相机正好在给定位置。

        `shift_m` 是相机校正里的平移，用来补偿深度读数的偏差。
        """
        arm = arms.by_id(arm_id)
        base_T_flange = collision_model.link_frames(OBSERVATION_DEG)[7]
        calibration = self.root / 'config/calibration'
        calibration.mkdir(parents=True, exist_ok=True)
        (calibration / f'{arm_id}_handeye.yaml').write_text(
            yaml.safe_dump(
                {
                    'flange_T_color_optical_camera': (
                        np.linalg.inv(base_T_flange) @ base_T_camera
                    ).tolist()
                }
            )
        )
        (calibration / f'{arm_id}_refinement.yaml').write_text(
            yaml.safe_dump(
                {
                    'schema_version': 1,
                    'arm': arm_id,
                    'acceptance_status': 'user_accepted_usable',
                    'correction': {
                        'rotation_vector_rad': [0., 0., 0.],
                        'translation_m': list(shift_m),
                    },
                    'depth_quadratic_per_m': 0.,
                }
            )
        )
        path = arm.config_path(self.root)
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_text(
            json.dumps(
                {
                    'handeye_file': f'config/calibration/{arm_id}_handeye.yaml',
                    'vision_refinement_file': (
                        f'config/calibration/{arm_id}_refinement.yaml'
                    ),
                    'return_after_lift': {'joint_deg': OBSERVATION_DEG},
                }
            )
        )

    def global_frame(self, base_T_board=None, square_m=.03):
        image = (
            np.full((720, 1280, 3), board_render.BACKGROUND, np.uint8)
            if base_T_board is None
            else board_render.render(
                GLOBAL_K, 1280, 720,
                np.linalg.inv(GLOBAL_POSE) @ base_T_board, square_m=square_m,
            )
        )
        return {'bgr': image, 'K': GLOBAL_K, 'distortion': np.zeros(5)}

    def hand_frame(
        self, base_T_board, pose=HAND_POSE, joint_deg=None, depth_bias_m=0.,
        square_m=.03,
    ):
        """手上相机的一帧；`depth_bias_m` 模拟深度读数整体偏近或偏远。"""
        camera_T_board = np.linalg.inv(pose) @ base_T_board
        depth = board_render.plane_depth(HAND_K, 640, 360, camera_T_board)
        return {
            'bgr': board_render.render(
                HAND_K, 640, 360, camera_T_board, square_m=square_m
            ),
            'depth_m': depth + np.float32(depth_bias_m),
            'K': HAND_K.tolist(),
            'joint_deg': joint_deg or OBSERVATION_DEG,
        }

    def show(self, placement, times=2, session=None, square_m=.03):
        """让标定板在某处停住若干轮，返回每一轮的结果。"""
        board = board_on_table(*placement)
        return [
            (session or self.session).step(
                self.global_frame(board, square_m),
                {'arm1': self.hand_frame(board, square_m=square_m)},
            )
            for _ in range(times)
        ]

    def record_all(self):
        for placement in PLACEMENTS:
            self.assertTrue(self.show(placement)[-1][0], placement)


class RecordingTests(GlobalCameraTestCase):
    def test_board_must_rest_for_a_round_before_it_is_recorded(self):
        first, second = self.show(PLACEMENTS[0])
        self.assertEqual(first, (False, '标定板刚放下或还在动，请保持不动'))
        self.assertEqual(second, (True, '已记录第 1 组（1臂（左臂）看到）'))

    def test_board_that_keeps_moving_is_not_recorded(self):
        for step in range(4):
            board = board_on_table(.30 + .004 * step, .05, 0)
            recorded, message = self.session.step(
                self.global_frame(board), {'arm1': self.hand_frame(board)}
            )
            self.assertFalse(recorded)
            self.assertIn('还在动', message)

    def test_same_place_is_reported_as_a_repeat(self):
        self.show(PLACEMENTS[0])
        self.assertEqual(
            self.show(PLACEMENTS[0], times=1)[0],
            (False, '与第 1 组重复，请把标定板挪到新位置'),
        )
        # 挪开 2 厘米仍算同一处；原地转 30 度算新的一组。
        self.assertIn('重复', self.show((.28, -.10, 0))[-1][1])
        self.assertTrue(self.show((.26, -.10, 30))[-1][0])

    def test_missing_board_in_the_global_camera(self):
        board = board_on_table(*PLACEMENTS[0])
        self.assertEqual(
            self.session.step(
                self.global_frame(), {'arm1': self.hand_frame(board)}
            ),
            (False, '全局相机没看到完整的标定板'),
        )

    def test_board_outside_the_hand_camera_view(self):
        # 全局相机看得到，但已经出了手上相机的画面。
        results = self.show((.78, .10, 0))
        self.assertEqual(
            results[-1], (False, '1臂（左臂）的相机没看到完整的标定板')
        )

    def test_arm_away_from_the_observation_pose_is_not_used(self):
        board = board_on_table(*PLACEMENTS[0])
        moved = list(OBSERVATION_DEG)
        moved[0] += 5.
        for _ in range(2):
            result = self.session.step(
                self.global_frame(board),
                {'arm1': self.hand_frame(board, joint_deg=moved)},
            )
        self.assertEqual(result, (False, '1臂（左臂）不在观测姿态'))

    def test_hand_camera_without_a_frame(self):
        board = board_on_table(*PLACEMENTS[0])
        for _ in range(2):
            result = self.session.step(self.global_frame(board), {})
        self.assertEqual(result, (False, '1臂（左臂）的相机没有画面'))

    def test_board_at_the_edge_of_the_picture_is_refused(self):
        # 标定板的一角贴着全局相机画面的右边缘。
        corners = global_camera.detect_corners(
            self.global_frame(board_on_table(*PLACEMENTS[0]))['bgr']
        )
        shifted = corners + [1280 - corners[:, 0].max() - 3, 0]
        pose, reason = global_camera.screen_view(
            shifted, 1280, 720, GLOBAL_K, np.zeros(5)
        )
        self.assertIsNone(pose)
        self.assertEqual(reason, '标定板离画面边缘太近')

    def test_enough_needs_both_the_count_and_the_spread(self):
        for placement in PLACEMENTS[:3]:
            self.show(placement)
        self.assertEqual(self.session.enough(8), (False, '还需要 5 组'))
        # 三组都挤在一条线上时，数量够了也不算采够。
        line = global_camera.Session(self.root, ['arm1'], None)
        for x in (.26, .34, .42):
            self.show((x, -.12, 0), session=line)
        done, missing = line.enough(3)
        self.assertFalse(done)
        self.assertIn('摆得太集中', missing)
        self.record_rest()
        self.assertEqual(self.session.enough(8), (True, ''))

    def record_rest(self):
        for placement in PLACEMENTS[3:]:
            self.show(placement)


class SolveTests(GlobalCameraTestCase):
    def assert_pose_close(self, pose):
        # 各组标定板都在桌面上，相机沿视线方向的位置定得最松，所以相机
        # 中心允许差几毫米；真正要紧的是用它换算物品位置准不准。
        shift_m, turn_deg = global_camera.change_from(GLOBAL_POSE, pose)
        self.assertLess(shift_m, .006)
        self.assertLess(turn_deg, .5)
        # 桌面上方 0～10 厘米、标定板摆过的范围内，换算误差在 2 毫米内。
        camera_T_base = np.linalg.inv(GLOBAL_POSE)
        for point in ([.30, -.10, 0.], [.40, .20, .05], [.34, .05, .10]):
            seen = camera_T_base[:3, :3] @ point + camera_T_base[:3, 3]
            mapped = pose[:3, :3] @ seen + pose[:3, 3]
            self.assertLess(np.linalg.norm(mapped - point), .002)

    def test_recovers_the_camera_pose(self):
        self.record_all()
        result = global_camera.solve(
            self.session.samples, GLOBAL_K, np.zeros(5)
        )
        self.assert_pose_close(result['base_T_camera'])
        self.assertTrue(result['accepted'], result['problems'])
        self.assertEqual(result['used'], list(range(1, 9)))
        self.assertEqual(result['rejected'], [])
        self.assertLess(result['reprojection_rms_px'], .6)
        self.assertLess(result['disagreement_rms_m'], .001)
        self.assertGreater(result['spread_m'][1], .12)

    def test_a_board_printed_smaller_than_nominal_still_calibrates(self):
        # 真实情况（2026-10-10）：标定板打印时缩小了 2.7%，方格实际边长
        # 29.2 毫米而不是标称的 30 毫米。求解和验收都不该依赖标称边长。
        # 这里用 29 毫米：合成图案的分辨率下它正好是整数个像素。
        for placement in PLACEMENTS:
            self.assertTrue(
                self.show(placement, square_m=.029)[-1][0], placement
            )
        result = global_camera.solve(
            self.session.samples, GLOBAL_K, np.zeros(5)
        )
        self.assertTrue(result['accepted'], result['problems'])
        self.assert_pose_close(result['base_T_camera'])
        self.assertLess(result['disagreement_rms_m'], .001)
        self.assertAlmostEqual(result['measured_square_m'], .029, delta=.0001)
        with self.assertLogs(global_camera.LOG, 'INFO') as logs:
            global_camera.report(result, None)
        text = '\n'.join(logs.output)
        self.assertIn('方格边长 29.0', text)
        self.assertRegex(text, r'方格比标称值小 3\.\d%')
        self.assertIn('手眼标定', text)

    def test_nominal_board_raises_no_size_warning(self):
        self.record_all()
        result = global_camera.solve(
            self.session.samples, GLOBAL_K, np.zeros(5)
        )
        self.assertAlmostEqual(result['measured_square_m'], .03, delta=.0001)
        with self.assertLogs(global_camera.LOG, 'INFO') as logs:
            global_camera.report(result, None)
        self.assertNotIn('标称值小', '\n'.join(logs.output))

    def test_space_error_is_the_distance_from_the_sight_line(self):
        self.record_all()
        sample = self.session.samples[0]
        moved = GLOBAL_POSE.copy()
        # 把相机的位置沿它自己的视线挪 20 毫米：角点仍大致在视线上，
        # 偏差很小；横着挪 20 毫米，偏差就接近 20 毫米。
        moved[:3, 3] += .02 * GLOBAL_POSE[:3, 2]
        along = global_camera.sample_errors(
            moved, sample, GLOBAL_K, np.zeros(5)
        )[1]
        moved = GLOBAL_POSE.copy()
        moved[:3, 3] += .02 * GLOBAL_POSE[:3, 0]
        across = global_camera.sample_errors(
            moved, sample, GLOBAL_K, np.zeros(5)
        )[1]
        self.assertLess(along, .004)
        self.assertAlmostEqual(across, .02, delta=.002)

    def test_saved_groups_can_be_solved_again(self):
        self.session = global_camera.Session(
            self.root, ['arm1'], None, self.root / 'work/session'
        )
        self.record_all()
        loaded = global_camera.load_samples(self.root / 'work/session')
        self.assertEqual(len(loaded), 8)
        again = global_camera.solve(loaded, GLOBAL_K, np.zeros(5))
        first = global_camera.solve(
            self.session.samples, GLOBAL_K, np.zeros(5)
        )
        np.testing.assert_allclose(
            again['base_T_camera'], first['base_T_camera'], atol=1e-9
        )
        with self.assertRaisesRegex(ValueError, '没有采集数据'):
            global_camera.load_samples(self.root / 'work/nothing')

    def test_one_bad_group_is_dropped(self):
        self.record_all()
        bad = self.session.samples[2]
        bad['points_base_m'] = (
            np.asarray(bad['points_base_m']) + [.03, 0., 0.]
        ).tolist()
        result = global_camera.solve(
            self.session.samples, GLOBAL_K, np.zeros(5)
        )
        self.assertEqual(result['rejected'], [3])
        self.assert_pose_close(result['base_T_camera'])
        self.assertTrue(result['accepted'])

    def test_clustered_boards_are_not_accepted(self):
        for x in (.26, .34, .42):
            self.show((x, -.12, 0))
        result = global_camera.solve(
            self.session.samples, GLOBAL_K, np.zeros(5)
        )
        self.assertFalse(result['accepted'])
        self.assertIn('摆得太集中', result['problems'][0])

    def test_lens_distortion_is_taken_into_account(self):
        # 用带畸变的相机把空间点投成像素，再反求相机位姿。
        distortion = np.array([-.055, .059, -.0008, .0005, -.02])
        random = np.random.default_rng(5)
        points = np.c_[random.uniform(.1, .6, 60), random.uniform(-.2, .3, 60),
                       random.uniform(0., .02, 60)]
        camera_T_base = np.linalg.inv(GLOBAL_POSE)
        pixels = cv2.projectPoints(
            points, cv2.Rodrigues(camera_T_base[:3, :3])[0],
            camera_T_base[:3, 3], GLOBAL_K, distortion,
        )[0].reshape(-1, 2)
        start = GLOBAL_POSE.copy()
        start[:3, 3] += [.03, -.02, .04]
        solved = global_camera.camera_from_points(
            points, pixels, GLOBAL_K, distortion, start
        )
        np.testing.assert_allclose(solved, GLOBAL_POSE, atol=1e-6)
        # 忽略畸变时结果明显偏离，说明畸变参数确实起了作用。
        ignored = global_camera.camera_from_points(
            points, pixels, GLOBAL_K, np.zeros(5), start
        )
        self.assertGreater(
            np.linalg.norm(ignored[:3, 3] - GLOBAL_POSE[:3, 3]), .001
        )

    def test_no_data_is_refused(self):
        with self.assertRaisesRegex(ValueError, '没有可用的数据'):
            global_camera.solve([], GLOBAL_K, np.zeros(5))


class TwoArmTests(GlobalCameraTestCase):
    def setUp(self):
        super().setUp()
        # 第二台臂的手上相机位置按它自己的基座写入配置。
        self.write_arm('arm2', np.linalg.inv(ARM1_T_ARM2) @ HAND2_POSE)
        self.relation = {'arm1_base_T_arm2_base': ARM1_T_ARM2.tolist()}

    def step_twice(self, relation):
        session = global_camera.Session(
            self.root, ['arm1', 'arm2'], relation
        )
        board = board_on_table(.34, .08, 10)
        for _ in range(2):
            result = session.step(
                self.global_frame(board),
                {
                    'arm1': self.hand_frame(board),
                    'arm2': self.hand_frame(board, pose=HAND2_POSE),
                },
            )
        return result, session

    def test_both_arms_agreeing_are_averaged(self):
        result, session = self.step_twice(self.relation)
        self.assertEqual(
            result, (True, '已记录第 1 组（1臂（左臂）、2臂（右臂）看到）')
        )
        self.assertEqual(session.samples[0]['seen_by'], ['arm1', 'arm2'])
        expected = global_camera.transformed(
            board_on_table(.34, .08, 10), global_camera.board_points()
        )
        # 角点从板的哪一端数起由检测器按黑白图案决定，与合成图案时的
        # 约定正好相反；要紧的是各台相机数法一致。
        np.testing.assert_allclose(
            session.samples[0]['points_base_m'], expected[::-1], atol=.002
        )

    def test_arms_that_disagree_are_refused(self):
        wrong = ARM1_T_ARM2.copy()
        wrong[0, 3] += .02
        result, session = self.step_twice(
            {'arm1_base_T_arm2_base': wrong.tolist()}
        )
        self.assertFalse(result[0])
        self.assertRegex(result[1], r'两台臂量出的标定板位置相差 2\d\.\d 毫米')
        self.assertIn('--arms arm1', result[1])
        self.assertEqual(session.samples, [])

    def test_cameras_are_used_the_way_their_correction_was_fitted(self):
        # 相机校正是按“彩色像素加深度读数”量出的点拟合的。这里假设
        # 两台手上相机的深度读数各有一个固定的偏差，校正里的平移正好
        # 补偿它：角点按深度读数来量、再套校正，两台臂应当对得上。
        bias = {'arm1': -.010, 'arm2': -.014}
        self.write_arm('arm1', HAND_POSE, shift_m=(0., 0., .010))
        self.write_arm(
            'arm2', np.linalg.inv(ARM1_T_ARM2) @ HAND2_POSE,
            shift_m=(0., 0., .014),
        )
        session = global_camera.Session(
            self.root, ['arm1', 'arm2'], self.relation
        )
        board = board_on_table(.34, .08, 10)
        for _ in range(2):
            result = session.step(
                self.global_frame(board),
                {
                    'arm1': self.hand_frame(
                        board, depth_bias_m=bias['arm1']
                    ),
                    'arm2': self.hand_frame(
                        board, pose=HAND2_POSE, depth_bias_m=bias['arm2']
                    ),
                },
            )
        self.assertTrue(result[0], result[1])
        expected = global_camera.transformed(
            board, global_camera.board_points()
        )
        np.testing.assert_allclose(
            session.samples[0]['points_base_m'], expected[::-1], atol=.003
        )

    def test_second_arm_needs_the_base_relation(self):
        session = global_camera.Session(self.root, ['arm2'], None)
        board = board_on_table(.34, .08, 10)
        with self.assertRaisesRegex(ValueError, '缺少两底座的相对位置'):
            for _ in range(2):
                session.step(
                    self.global_frame(board),
                    {'arm2': self.hand_frame(board, pose=HAND2_POSE)},
                )


class DepthCornerTests(GlobalCameraTestCase):
    def frame(self, placement=(.34, .08, 10)):
        frame = self.hand_frame(board_on_table(*placement))
        corners = global_camera.detect_corners(frame['bgr'])
        return frame, corners

    def test_corners_from_depth_match_the_true_positions(self):
        frame, corners = self.frame()
        points, reason = global_camera.corners_from_depth(
            corners, frame['depth_m'], HAND_K
        )
        self.assertEqual(reason, '')
        expected = global_camera.transformed(
            np.linalg.inv(HAND_POSE) @ board_on_table(.34, .08, 10),
            global_camera.board_points(),
        )
        np.testing.assert_allclose(points, expected[::-1], atol=.002)

    def test_noisy_readings_are_averaged_out_by_the_plane(self):
        frame, corners = self.frame()
        rng = np.random.default_rng(5)
        noisy = frame['depth_m'] + rng.normal(
            0., .002, frame['depth_m'].shape
        ).astype(np.float32)
        points, _ = global_camera.corners_from_depth(corners, noisy, HAND_K)
        exact, _ = global_camera.corners_from_depth(
            corners, frame['depth_m'], HAND_K
        )
        self.assertLess(np.abs(points - exact).max(), .0006)

    def test_missing_depth_is_refused(self):
        frame, corners = self.frame()
        holes = frame['depth_m'].copy()
        holes[:, :400] = 0.
        points, reason = global_camera.corners_from_depth(
            corners, holes, HAND_K
        )
        self.assertIsNone(points)
        self.assertIn('深度读数缺得太多', reason)
        # 画面合格但深度图不可用时，这一轮不记录，并说明是哪台臂。
        frame['depth_m'] = holes
        board = board_on_table(.34, .08, 10)
        self.session.step(self.global_frame(board), {'arm1': frame})
        recorded, message = self.session.step(
            self.global_frame(board), {'arm1': frame}
        )
        self.assertFalse(recorded)
        self.assertIn('1臂（左臂）的相机：标定板上的深度读数缺得太多', message)

    def test_depth_that_is_not_a_plane_is_refused(self):
        frame, corners = self.frame()
        bent = frame['depth_m'].copy()
        bent[:, 320:] += .03
        points, reason = global_camera.corners_from_depth(
            corners, bent, HAND_K
        )
        self.assertIsNone(points)
        self.assertRegex(reason, r'不成一个平面（起伏 \d+\.\d 毫米）')


class LoopTests(GlobalCameraTestCase):
    def script(self, placements):
        """按顺序给出各处摆位的画面，每处停两轮，中间夹一轮空画面。"""
        frames = []
        for placement in placements:
            board = board_on_table(*placement)
            frames.append((self.global_frame(), {}))
            for _ in range(2):
                frames.append(
                    (self.global_frame(board), {'arm1': self.hand_frame(board)})
                )
        return frames

    def devices(self, frames):
        state = {'index': 0, 'now': 0.}

        def current():
            return frames[min(state['index'], len(frames) - 1)]

        def hand_frame(arm_id):
            frame = current()[1].get(arm_id)
            state['index'] += 1
            return frame

        def sleep(seconds):
            state['now'] += max(seconds, .01)

        camera = types.SimpleNamespace(frame=lambda: current()[0])
        backend = types.SimpleNamespace(hand_frame=hand_frame)
        return camera, backend, (lambda: state['now']), sleep, state

    def test_loop_records_until_enough(self):
        camera, backend, clock, sleep, state = self.devices(
            self.script(PLACEMENTS)
        )
        with self.assertLogs(global_camera.LOG, 'INFO') as logs:
            done = global_camera.calibrate(
                self.session, camera, backend, ['arm1'], 8, 300.,
                clock, sleep,
            )
        self.assertTrue(done)
        self.assertEqual(len(self.session.samples), 8)
        text = '\n'.join(logs.output)
        self.assertIn('已记录第 1 组（1臂（左臂）看到）。还需要 7 组', text)
        self.assertIn('已记录第 8 组（1臂（左臂）看到）。已采够', text)
        # 每轮间隔一秒。
        self.assertAlmostEqual(clock(), state['index'] - 1, delta=1.)

    def test_loop_gives_up_at_the_time_limit(self):
        camera, backend, clock, sleep, _ = self.devices(
            self.script(PLACEMENTS[:2])
        )
        with self.assertLogs(global_camera.LOG, 'INFO') as logs:
            done = global_camera.calibrate(
                self.session, camera, backend, ['arm1'], 8, 20.,
                clock, sleep,
            )
        self.assertFalse(done)
        self.assertEqual(len(self.session.samples), 2)
        # 标定板停在原处时，同一句提示只打印一次。
        repeats = [line for line in logs.output if '重复' in line]
        self.assertEqual(len(repeats), 1)

    def test_loop_waits_out_rounds_without_a_global_picture(self):
        frames = self.script(PLACEMENTS)
        camera, backend, clock, sleep, state = self.devices(frames)
        steady = camera.frame

        def flaky():
            # 第 3 到 6 轮相机服务给不出画面（例如有人在页面上点了释放）。
            if 3 <= state['index'] < 7:
                state['index'] += 1
                raise global_camera.FrameUnavailable('该相机没有活动预览。')
            return steady()

        camera.frame = flaky
        with self.assertLogs(global_camera.LOG, 'INFO') as logs:
            done = global_camera.calibrate(
                self.session, camera, backend, ['arm1'], 6, 300.,
                clock, sleep,
            )
        self.assertTrue(done)
        missing = [line for line in logs.output if '暂时取不到画面' in line]
        self.assertEqual(len(missing), 1)
        self.assertIn('该相机没有活动预览', missing[0])

    def test_verify_reports_how_far_the_stored_result_is_off(self):
        frames = self.script(PLACEMENTS[:1])
        camera, backend, clock, sleep, _ = self.devices(frames)
        with self.assertLogs(global_camera.LOG, 'INFO'):
            pixel, space = global_camera.verify(
                self.session, camera, backend, ['arm1'], GLOBAL_POSE, 30.,
                clock, sleep,
            )
        self.assertLess(pixel, global_camera.ACCEPT_REPROJECTION_PX)
        self.assertLess(space, .002)
        self.assertEqual(self.session.samples, [])
        # 相机被挪动 2 厘米后，同样的画面给出明显的偏差。
        moved = GLOBAL_POSE.copy()
        moved[:3, 3] += [.02, 0., 0.]
        camera, backend, clock, sleep, _ = self.devices(frames)
        with self.assertLogs(global_camera.LOG, 'INFO'):
            pixel, space = global_camera.verify(
                global_camera.Session(self.root, ['arm1'], None),
                camera, backend, ['arm1'], moved, 30., clock, sleep,
            )
        self.assertGreater(pixel, global_camera.ACCEPT_REPROJECTION_PX)
        self.assertAlmostEqual(space, .02, delta=.003)

    def test_verify_times_out_without_a_usable_view(self):
        camera, backend, clock, sleep, _ = self.devices(
            [(self.global_frame(), {})]
        )
        with self.assertLogs(global_camera.LOG, 'INFO'):
            self.assertIsNone(
                global_camera.verify(
                    self.session, camera, backend, ['arm1'], GLOBAL_POSE, 5.,
                    clock, sleep,
                )
            )


class ServedCameraTests(GlobalCameraTestCase):
    """全局相机的画面经真实的 8088 服务代码取得；假的只有相机驱动。"""

    def setUp(self):
        super().setUp()
        self.board = board_on_table(*PLACEMENTS[0])
        self.rs = fake_realsense.Module(
            self.global_frame(self.board)['bgr'],
            serial='218622270784',
            intrinsics=types.SimpleNamespace(
                fx=654., fy=653., ppx=647., ppy=365.,
                model='distortion.inverse_brown_conrady',
                coeffs=[.01, -.02, 0., 0., .003],
            ),
        )
        for patch in fake_realsense.patches(self.rs, monitor, str(self.root)):
            patch.start()
            self.addCleanup(patch.stop)
        self.server = http.server.ThreadingHTTPServer(
            ('127.0.0.1', 0), monitor.Handler
        )
        self.server.daemon_threads = True
        self.server.cameras = {
            'd435i': monitor.Camera(1280, 720, 15, 'd435i', '405622073572'),
            'd405': monitor.Camera(1280, 720, 15, 'd405', '218622270784'),
        }
        self.worker = threading.Thread(
            target=self.server.serve_forever, daemon=True
        )
        self.worker.start()
        self.addCleanup(self.stop_server)
        self.url = 'http://127.0.0.1:%d' % self.server.server_address[1]

    def stop_server(self):
        for camera in self.server.cameras.values():
            camera.stop()
        self.server.shutdown()
        self.server.server_close()
        self.worker.join()

    def test_pictures_come_through_the_service_unaltered(self):
        camera = global_camera.ServedCamera(self.url, '218622270784')
        frame = camera.frame()
        self.assertTrue(np.array_equal(frame['bgr'], self.rs.image))
        self.assertTrue(np.allclose(frame['K'], GLOBAL_K))
        self.assertTrue(
            np.allclose(frame['distortion'], [.01, -.02, 0., 0., .003])
        )
        self.assertEqual(
            camera.distortion_model, 'distortion.inverse_brown_conrady'
        )
        # 相机只被打开了一次：是相机服务开的，不是标定程序。
        self.assertEqual(self.rs.started, 1)

    def test_preview_stays_on_while_a_sample_is_recorded(self):
        camera = global_camera.ServedCamera(self.url, '218622270784')
        viewer = http.client.HTTPConnection(
            *self.server.server_address, timeout=3
        )
        self.addCleanup(viewer.close)
        viewer.request('GET', '/stream/d405')
        stream = viewer.getresponse()
        self.assertEqual(stream.status, 200)
        hands = {'arm1': self.hand_frame(self.board)}
        self.session.step(camera.frame(), hands)
        recorded, message = self.session.step(camera.frame(), hands)
        self.assertTrue(recorded, message)
        # 预览还在出新的画面。
        self.assertIn(b'--frame', stream.readline())
        self.assertEqual(self.server.cameras['d405'].status()['clients'], 1)
        self.assertEqual((self.rs.started, self.rs.stopped), (1, 0))

    def test_unknown_serial_and_stopped_service_are_explained(self):
        with self.assertRaisesRegex(
            global_camera.FrameUnavailable, '没有序列号为 000000000000'
        ):
            global_camera.ServedCamera(self.url, '000000000000')
        self.stop_server()
        self.addCleanup(self.server.server_close)
        self.stop_server = lambda: None
        with self.assertRaisesRegex(
            global_camera.FrameUnavailable, '连不上相机服务.*run_camera_monitor'
        ):
            global_camera.ServedCamera(self.url, '218622270784')

    def test_released_camera_is_reported_for_that_round_only(self):
        camera = global_camera.ServedCamera(self.url, '218622270784')
        self.rs.device.serial = 'unplugged'
        self.server.cameras['d405'].stop()
        with self.assertRaisesRegex(
            global_camera.FrameUnavailable, '没有找到唯一的指定相机'
        ):
            camera.frame()
        self.rs.device.serial = '218622270784'
        self.assertTrue(
            np.array_equal(camera.frame()['bgr'], self.rs.image)
        )


class ResultFileTests(GlobalCameraTestCase):
    def camera(self):
        return types.SimpleNamespace(
            serial='218622270784',
            camera_matrix=GLOBAL_K,
            distortion=np.zeros(5),
            distortion_model='distortion.inverse_brown_conrady',
        )

    def write(self, base_T_camera):
        directory = self.root / 'work/calibration/global_camera_test'
        directory.mkdir(parents=True, exist_ok=True)
        (directory / 'result.json').write_text('{}')
        result = {
            'base_T_camera': base_T_camera,
            'used': [1, 2, 3],
            'reprojection_rms_px': .21,
            'disagreement_rms_m': .0011,
            'spread_m': [.3, .2],
            'measured_square_m': .03,
        }
        with self.assertLogs(global_camera.LOG, 'INFO') as logs:
            global_camera.LOG.info('开始写入')
            path = global_camera.write_result(
                self.root, result, self.camera(), directory, ['arm1']
            )
        return path, logs.output

    def test_result_is_written_and_read_back(self):
        self.assertIsNone(global_camera.load_previous(self.root))
        path, _ = self.write(GLOBAL_POSE)
        self.assertEqual(
            path, self.root / 'config/calibration/global_camera_extrinsic.yaml'
        )
        record = yaml.safe_load(path.read_text())
        self.assertEqual(record['camera_serial'], '218622270784')
        self.assertEqual(record['samples_used'], 3)
        self.assertEqual(record['hand_camera_arms'], ['arm1'])
        self.assertEqual(len(record['source']['sha256']), 64)
        np.testing.assert_allclose(
            global_camera.load_previous(self.root), GLOBAL_POSE
        )

    def test_previous_result_is_backed_up(self):
        self.write(GLOBAL_POSE)
        moved = GLOBAL_POSE.copy()
        moved[:3, 3] += [.01, 0., 0.]
        path, logs = self.write(moved)
        backups = list(path.parent.glob(path.name + '.before-*'))
        self.assertEqual(len(backups), 1)
        np.testing.assert_allclose(
            yaml.safe_load(backups[0].read_text())['arm1_base_T_camera'],
            GLOBAL_POSE,
        )
        self.assertIn('已备份', '\n'.join(logs))
        shift_m, turn_deg = global_camera.change_from(
            GLOBAL_POSE, global_camera.load_previous(self.root)
        )
        self.assertAlmostEqual(shift_m, .01)
        self.assertAlmostEqual(turn_deg, 0., places=6)

    def test_unreadable_previous_result_counts_as_none(self):
        path = self.root / global_camera.RESULT_FILE
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_text('arm1_base_T_camera: [oops')
        with self.assertLogs(global_camera.LOG, 'WARNING'):
            self.assertIsNone(global_camera.load_previous(self.root))


if __name__ == '__main__':
    unittest.main()
