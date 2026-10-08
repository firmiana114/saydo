import math
import unittest
from unittest.mock import patch
import cv2
import numpy as np
from saydo.calibration.geometry import board_pose, object_points, pose_matrix, stationarity, zero_reference_frames
from saydo.calibration.capture import Robot


class GeometryChecks(unittest.TestCase):
    def test_json_units_and_yaw(self):
        transform = pose_matrix([100000, -200000, 300000, 0, 0, 1571])
        np.testing.assert_allclose(transform[:3, 3], [.1, -.2, .3])
        np.testing.assert_allclose(
            transform[:3, :3] @ [1, 0, 0], [0, 1, 0], atol=.0003
        )

    def test_rotation_wrap_is_not_motion(self):
        states = [
            {'err': [0], 'joint': [0] * 7, 'pose': [0, 0, 0, 0, 0, yaw]}
            for yaw in [3141, -3141, 3141]
        ]
        self.assertLess(stationarity(states)['rotation_span_deg'], .15)

    def test_motion_between_equal_endpoints_is_rejected(self):
        states = [
            {'err': [0], 'joint': [q] + [0] * 6, 'pose': [0] * 6}
            for q in [0, 100, 0]
        ]
        with self.assertRaises(ValueError):
            stationarity(states)

    def test_nonzero_tool_is_not_silently_treated_as_flange(self):
        with self.assertRaises(ValueError):
            zero_reference_frames(
                {'tool_name': 'Arm_Tip', 'pose': [0, 0, 200000, 0, 0, 0]},
                {'frame_name': 'Base', 'pose': [0] * 6},
            )

    def test_he_ref_with_nonzero_payload_and_com_is_flange_reference(self):
        zero_reference_frames(
            {
                'tool_name': 'he_ref',
                'pose': [0] * 6,
                'payload': 1032.6759,
                'position': [-19000, 6000, 110000],
            },
            {'frame_name': 'Base', 'pose': [0] * 6},
        )

    def test_he_ref_still_rejects_tool_offsets(self):
        for pose in ([1000, 0, 0, 0, 0, 0], [0, 0, 0, 0, 100, 0]):
            with self.subTest(pose=pose), self.assertRaises(ValueError):
                zero_reference_frames(
                    {'tool_name': 'he_ref', 'pose': pose},
                    {'frame_name': 'Base', 'pose': [0] * 6},
                )

    def test_unknown_tool_or_nonzero_work_frame_is_rejected(self):
        for tool, work in [
            (
                {'tool_name': 'other', 'pose': [0] * 6},
                {'frame_name': 'Base', 'pose': [0] * 6},
            ),
            (
                {'tool_name': 'he_ref', 'pose': [0] * 6},
                {'frame_name': 'Base', 'pose': [0, 0, 1000, 0, 0, 0]},
            ),
        ]:
            with self.subTest(
                tool=tool, work=work
            ), self.assertRaises(ValueError):
                zero_reference_frames(tool, work)

    def test_error_blocks_acceptance(self):
        states = [{'err': [4116], 'joint': [0] * 7, 'pose': [0] * 6}] * 3
        with self.assertRaises(ValueError):
            stationarity(states)

    def test_known_projected_board_pose_is_recovered(self):
        matrix = np.array([[900., 0, 640], [0, 900., 360], [0, 0, 1.]])
        rv = np.array([.25, -.3, .1])
        tv = np.array([-.1, -.12, .7])
        image = cv2.projectPoints(
            object_points(), rv, tv, matrix, np.zeros(5)
        )[0]
        candidates = board_pose(image, matrix, np.zeros(5))
        best = np.array(candidates[0]['camera_T_board'])
        self.assertLess(candidates[0]['reprojection_rms_px'], 1e-5)
        np.testing.assert_allclose(best[:3, 3], tv, atol=1e-6)
        np.testing.assert_allclose(
            best[:3, :3], cv2.Rodrigues(rv)[0], atol=1e-6
        )

    def test_motion_command_rejected_before_network_access(self):
        with patch(
            'saydo.calibration.capture.socket.create_connection'
        ) as connect:
            with self.assertRaises(ValueError):
                Robot('127.0.0.1').query('movej')
            connect.assert_not_called()


if __name__ == '__main__':
    unittest.main()
