import copy
import json
import pathlib
import tempfile
import unittest
from unittest import mock

import cv2
import numpy as np
from saydo.calibration import collect


def sample(sample_id, x=0, degrees=0):
    t = np.eye(4)
    t[:3, :3] = cv2.Rodrigues(np.array([0., 0., np.radians(degrees)]))[0]
    t[0, 3] = x
    return {
        'sample_id': sample_id,
        'status': 'candidate',
        'base_T_flange': t.tolist(),
        'board': {'inner_corners': [8, 11], 'square_size_m': .03},
        'camera': {'serial': 'test-only', 'size': [1280, 720]},
        'preflight': {
            'get_current_tool_frame': {'tool_name': 'he_ref', 'pose': [0] * 6},
            'get_current_work_frame': {'frame_name': 'Base', 'pose': [0] * 6},
            'get_arm_software_info': {'Product_version': 'RM75-BI'},
        },
        'corners_px': [[1, 1]] * 88,
        'board_pose_candidates': [{'reprojection_rms_px': .5}],
        'stationarity': {},
    }


class CollectionTests(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        self.addCleanup(self.tmp.cleanup)
        self.root = pathlib.Path(self.tmp.name)
        self.reference = sample('fit')
        self.write(self.reference)
        self.plan = {
            'training_candidate_ids': ['fit'],
            'validation_candidate_ids': [],
            'validation_target_count': 5,
        }
        collect.atomic_json(self.root / 'dataset_split.json', self.plan)
        # 相机在系统边界上：枚举和占用查询用替身，缺省“空闲”。
        self.camera = {
            'serial': 'test-only', 'name': 'D435I', 'asic_serial': 'asic'
        }
        self.select = self.patch(
            collect.capture, 'select_camera', return_value=self.camera
        )
        self.users = self.patch(
            collect.capture, 'camera_users', return_value=[]
        )

    def patch(self, target, name, **kwargs):
        patcher = mock.patch.object(target, name, **kwargs)
        self.addCleanup(patcher.stop)
        return patcher.start()

    def write(self, value):
        path = self.root / value['sample_id']
        path.mkdir(exist_ok=True)
        collect.atomic_json(path / 'sample.json', value)
        return path

    def mock_robot(self, x):
        def query(command):
            if command == 'get_current_arm_state':
                return {
                    'reply': {
                        'arm_state': {
                            'err': [0], 'pose': [int(x * 1e6), 0, 0, 0, 0, 0]
                        }
                    }
                }
            return {'reply': self.reference['preflight'][command]}

        return query

    def test_duplicate_needs_both_translation_and_rotation(self):
        ref = {'fit': self.reference}
        self.assertTrue(
            collect.comparisons(
                sample('a', .005, 2)['base_T_flange'], ref
            )[0]['near_duplicate']
        )
        self.assertFalse(
            collect.comparisons(
                sample('b', .02, 2)['base_T_flange'], ref
            )[0]['near_duplicate']
        )
        self.assertFalse(
            collect.comparisons(
                sample('c', .005, 12)['base_T_flange'], ref
            )[0]['near_duplicate']
        )

    def test_duplicate_skips_camera_and_does_not_append_validation(self):
        with mock.patch.object(
            collect.capture.Robot, 'query', side_effect=self.mock_robot(.001)
        ), mock.patch.object(
            collect, 'pause_preview'
        ) as preview, mock.patch.object(collect, 'run_capture') as capture:
            result = collect.collect_once(self.root, settle_seconds=0)
        self.assertEqual(result['state'], 'duplicate')
        preview.assert_not_called()
        capture.assert_not_called()
        self.assertEqual(
            json.loads((self.root / 'dataset_split.json').read_text()),
            self.plan,
        )

    def test_capture_calls_once_and_freezes_fitting_pool(self):
        new = sample('new', .03)

        def capture(*args):
            return self.write(new), copy.deepcopy(new)

        with mock.patch.object(
            collect.capture.Robot, 'query', side_effect=self.mock_robot(.03)
        ), mock.patch.object(
            collect, 'pause_preview', return_value='idle'
        ), mock.patch.object(
            collect, 'run_capture', side_effect=capture
        ) as call:
            result = collect.collect_once(self.root, settle_seconds=0)
        call.assert_called_once()
        self.assertEqual(result['state'], 'captured')
        self.assertEqual(result['role'], 'validation')
        plan = json.loads((self.root / 'dataset_split.json').read_text())
        self.assertEqual(plan['training_candidate_ids'], ['fit'])
        self.assertEqual(plan['validation_candidate_ids'], ['new'])

    def capture_new(self, x, sample_id='new'):
        new = sample(sample_id, x)

        def capture(*args):
            return self.write(new), copy.deepcopy(new)

        self.patch(
            collect.capture.Robot, 'query', side_effect=self.mock_robot(x)
        )
        return self.patch(collect, 'run_capture', side_effect=capture)

    def test_requested_camera_is_the_one_checked_and_captured(self):
        call = self.capture_new(.03)
        preview = self.patch(collect, 'pause_preview')
        result = collect.collect_once(
            self.root, '192.168.1.18', settle_seconds=0, serial='test-only'
        )
        self.select.assert_called_once_with('test-only')
        self.users.assert_called_once_with(self.camera)
        call.assert_called_once_with(self.root, '192.168.1.18', 'test-only')
        # 目标相机空闲时不去打断预览服务。
        preview.assert_not_called()
        self.assertEqual(result['preview_before_capture'], 'camera_free')

    def test_preview_is_paused_only_when_the_target_camera_is_in_use(self):
        self.users.return_value = [4321]
        self.capture_new(.03)
        preview = self.patch(collect, 'pause_preview', return_value='idle')
        result = collect.collect_once(self.root, settle_seconds=0)
        preview.assert_called_once_with()
        self.assertEqual(result['preview_before_capture'], 'idle')

    def test_unknown_camera_stops_before_capture(self):
        self.select.side_effect = ValueError('没有找到序列号为 x 的 D435i')
        call = self.capture_new(.03)
        with self.assertRaisesRegex(ValueError, '没有找到'):
            collect.collect_once(self.root, settle_seconds=0, serial='x')
        call.assert_not_called()

    def test_post_capture_duplicate_is_rejected(self):
        new = sample('new', .001)

        def capture(*args):
            return self.write(new), copy.deepcopy(new)

        with mock.patch.object(
            collect.capture.Robot, 'query', side_effect=self.mock_robot(.05)
        ), mock.patch.object(
            collect, 'pause_preview', return_value='idle'
        ), mock.patch.object(collect, 'run_capture', side_effect=capture):
            result = collect.collect_once(self.root, settle_seconds=0)
        self.assertEqual(result['state'], 'rejected')
        self.assertEqual(
            json.loads((self.root / 'new/sample.json').read_text())['status'],
            'rejected',
        )
        self.assertEqual(
            json.loads((self.root / 'dataset_split.json').read_text()),
            self.plan,
        )

    def test_bad_camera_parameters_are_rejected(self):
        new = sample('new', .03)
        new['camera']['serial'] = 'different-camera'
        with self.assertRaises(ValueError):
            collect.check_compatibility(self.reference, new)

    def test_quality_rejection_is_not_retried_or_added(self):
        new = sample('new', .03)
        new.update(status='rejected', error='not stationary')

        def capture(*args):
            return self.write(new), copy.deepcopy(new)

        with mock.patch.object(
            collect.capture.Robot, 'query', side_effect=self.mock_robot(.03)
        ), mock.patch.object(
            collect, 'pause_preview', return_value='idle'
        ), mock.patch.object(
            collect, 'run_capture', side_effect=capture
        ) as call:
            result = collect.collect_once(self.root, settle_seconds=0)
        call.assert_called_once()
        self.assertEqual(result['state'], 'rejected')
        self.assertEqual(
            json.loads((self.root / 'dataset_split.json').read_text()),
            self.plan,
        )

    def test_interrupted_admission_is_recovered_once(self):
        value = sample('pending', .03)
        value['collection_admission'] = {
            'role': 'validation',
            'passed': True,
            'training_candidate_ids': ['fit'],
        }
        self.write(value)
        for _ in range(2):
            plan = collect.load_plan(self.root, collect.load_samples(self.root))
            self.assertEqual(plan['validation_candidate_ids'], ['pending'])
            self.assertEqual(plan['training_candidate_ids'], ['fit'])

    def test_full_validation_set_does_not_touch_hardware(self):
        self.write(sample('v1', .03))
        self.plan.update(
            validation_candidate_ids=['v1'], validation_target_count=1
        )
        collect.atomic_json(self.root / 'dataset_split.json', self.plan)
        with mock.patch.object(
            collect.capture, 'Robot'
        ) as robot, mock.patch.object(collect, 'run_capture') as capture:
            result = collect.collect_once(self.root)
        robot.assert_not_called()
        capture.assert_not_called()
        self.assertEqual(result['state'], 'complete')

    def test_corrupt_split_does_not_get_silently_recreated(self):
        self.plan['validation_candidate_ids'] = ['missing']
        collect.atomic_json(self.root / 'dataset_split.json', self.plan)
        with self.assertRaises(ValueError):
            collect.load_plan(self.root, collect.load_samples(self.root))
        self.assertEqual(
            json.loads((self.root / 'dataset_split.json').read_text()),
            self.plan,
        )


class FittingPhaseTests(unittest.TestCase):
    """新会话的求解样本阶段：真实的登记、去重和冻结代码。"""

    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        self.addCleanup(self.tmp.cleanup)
        self.root = pathlib.Path(self.tmp.name)
        self.x = 0.
        self.captured = 0
        for target, name, kwargs in (
            (collect.capture, 'select_camera', {
                'return_value': {'serial': 'test-only', 'asic_serial': 'a'}
            }),
            (collect.capture, 'camera_users', {'return_value': []}),
            (collect.capture.Robot, 'query', {'side_effect': self.query}),
            (collect, 'run_capture', {'side_effect': self.capture}),
        ):
            patcher = mock.patch.object(target, name, **kwargs)
            self.addCleanup(patcher.stop)
            patcher.start()

    def query(self, command):
        if command == 'get_current_arm_state':
            return {
                'reply': {
                    'arm_state': {
                        'err': [0],
                        'pose': [int(self.x * 1e6), 0, 0, 0, 0, 0],
                    }
                }
            }
        return {'reply': sample('unused')['preflight'][command]}

    def capture(self, session, robot_ip, serial):
        self.captured += 1
        value = sample(f'{self.captured:06d}_pose', self.x)
        path = session / value['sample_id']
        path.mkdir()
        collect.atomic_json(path / 'sample.json', value)
        return path, copy.deepcopy(value)

    def collect(self, x, **kwargs):
        self.x = x
        return collect.collect_once(
            self.root, settle_seconds=0, **{'fitting_count': 3, **kwargs}
        )

    def split(self):
        return json.loads((self.root / 'dataset_split.json').read_text())

    def test_pool_is_frozen_when_the_target_is_reached(self):
        first = self.collect(.00)
        self.assertEqual(first['state'], 'captured')
        self.assertEqual(first['role'], 'training')
        self.assertEqual(
            (first['training_candidate_count'], first['training_target_count']),
            (1, 3),
        )
        self.collect(.03)
        self.assertFalse((self.root / 'dataset_split.json').exists())
        last = self.collect(.06, validation_count=2)
        self.assertIn('已冻结', last['message'])
        plan = self.split()
        self.assertEqual(
            plan['training_candidate_ids'],
            ['000001_pose', '000002_pose', '000003_pose'],
        )
        self.assertEqual(plan['validation_candidate_ids'], [])
        self.assertEqual(plan['validation_target_count'], 2)

    def test_near_duplicate_pose_is_skipped_without_capturing(self):
        self.collect(.00)
        result = self.collect(.004)
        self.assertEqual(result['state'], 'duplicate')
        self.assertEqual(self.captured, 1)
        self.assertEqual(result['training_candidate_count'], 1)

    def test_frozen_pool_cannot_grow_and_validation_follows(self):
        for x in (.00, .03, .06):
            self.collect(x)
        with self.assertRaisesRegex(ValueError, '已经冻结'):
            self.collect(.09)
        self.assertEqual(self.captured, 3)
        result = self.collect(.09, fitting_count=None)
        self.assertEqual(result['role'], 'validation')
        plan = self.split()
        self.assertEqual(len(plan['training_candidate_ids']), 3)
        self.assertEqual(plan['validation_candidate_ids'], ['000004_pose'])

    def test_samples_from_another_workflow_are_not_adopted(self):
        foreign = sample('legacy')
        (self.root / 'legacy').mkdir()
        collect.atomic_json(self.root / 'legacy/sample.json', foreign)
        with self.assertRaisesRegex(ValueError, 'legacy'):
            self.collect(.03)
        self.assertEqual(self.captured, 0)
        self.assertFalse((self.root / 'dataset_split.json').exists())

    def test_interrupted_freeze_is_completed_without_a_new_sample(self):
        for x in (.00, .03):
            self.collect(x)
        with mock.patch.object(
            collect, 'freeze_plan', side_effect=OSError('disk full')
        ), self.assertRaises(OSError):
            self.collect(.06)
        self.assertFalse((self.root / 'dataset_split.json').exists())
        result = self.collect(.09)
        self.assertEqual(result['state'], 'complete')
        self.assertEqual(self.captured, 3)
        self.assertEqual(len(self.split()['training_candidate_ids']), 3)

    def test_first_sample_with_missing_corners_is_rejected(self):
        original = self.capture

        def partial(session, robot_ip, serial):
            path, value = original(session, robot_ip, serial)
            value['corners_px'] = value['corners_px'][:80]
            return path, value

        with mock.patch.object(collect, 'run_capture', side_effect=partial):
            result = self.collect(.00)
        self.assertEqual(result['state'], 'rejected')
        self.assertEqual(result['training_candidate_count'], 0)


if __name__ == '__main__':
    unittest.main()
