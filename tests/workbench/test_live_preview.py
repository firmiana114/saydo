import pathlib
import json
import tempfile
import time
import types
import unittest
from unittest import mock

import cv2
import numpy as np

from saydo.perception import live_checks
from saydo.robot import arms
from saydo.workbench import live_tracker
from tests.robot import fake_controller


class LivePreviewTests(unittest.TestCase):
    def setUp(self):
        self.directory = tempfile.TemporaryDirectory()
        root = pathlib.Path(self.directory.name)
        mesh = root / 'model.obj'
        mesh.write_text('# fixture\n')
        # 跟踪器按项目根目录找抓取配置和配置里引用的标定文件，所以
        # 在临时根目录下自备一份，不读运行所在目录里本机的 config/。
        fake_controller.write_arm_config(
            root, arms.default(), '10.0.0.1', [0] * 7
        )
        args = types.SimpleNamespace(
            mesh=mesh,
            serial='test',
            detector_class='bottle',
            auto_recover=True,
            recovery_interval=0.,
        )
        with mock.patch.object(live_tracker, 'ROOT', root):
            self.tracker = live_tracker.Tracker(args)

    def tearDown(self):
        self.directory.cleanup()

    def test_automatic_recovery_preview_uses_live_frames_but_selection_stays_frozen(
        self
    ):
        t = self.tracker
        t.frozen = {
            'sequence': 1,
            'time': time.time() - 3,
            'bgr': np.full((64, 64, 3), 20, np.uint8),
        }
        t.latest = {
            'sequence': 2,
            'time': time.time(),
            'bgr': np.full((64, 64, 3), 200, np.uint8),
        }
        for mode in ['detecting', 'initializing', 'catching_up', 'recovering']:
            t.state['state'] = mode
            self.assertEqual(t.preview_key()[2], 2)
            image = cv2.imdecode(
                np.frombuffer(t.image(), np.uint8), cv2.IMREAD_COLOR
            )
            self.assertGreater(int(image[50, 50, 0]), 150)
        for mode in ['selecting', 'choose_candidate']:
            t.state['state'] = mode
            self.assertEqual(t.preview_key()[2], 1)
            image = cv2.imdecode(
                np.frombuffer(t.image(), np.uint8), cv2.IMREAD_COLOR
            )
            self.assertLess(int(image[50, 50, 0]), 50)

    def test_catchup_follows_intermediate_frames_and_returns_fresh_result(self):
        t = self.tracker
        now = time.time()
        frames = [{'sequence': i, 'time': now - 1 + i * .1} for i in range(11)]
        t.history.extend(frames)
        t.latest = frames[-1]
        visited = []

        def advance(frame, pose):
            visited.append(frame['sequence'])
            return pose

        result = t.catch_up(0, frames[0], np.eye(4), advance)
        self.assertGreater(len(visited), 1)
        self.assertLess(
            visited[0], 10, 'must not jump straight to latest frame'
        )
        self.assertLess(time.time() - result[0]['time'], .25)
        self.assertIsNone(t.state['camera_T_object'])

    def test_missing_history_and_user_pause_do_not_publish_old_pose(self):
        now = time.time()
        with self.assertRaises(ValueError):
            live_checks.catchup_frame(
                [{'sequence': 20, 'time': now}], 1, now - 4
            )
        t = self.tracker
        frames = [{'sequence': i, 'time': now - 1 + i * .1} for i in range(11)]
        t.history.extend(frames)

        def advance(frame, pose):
            t.pause()
            return pose

        self.assertIsNone(t.catch_up(0, frames[0], np.eye(4), advance))
        self.assertEqual(t.state['state'], 'paused')

    def test_adaptive_catchup_converges_when_fixed_stride_would_fall_behind(
        self
    ):
        t = self.tracker
        clock = [100.]
        frame = {'sequence': 0, 'time': 94.}
        for i in range(91):
            t.history.append({'sequence': i, 'time': 94. + i / 15})
        visited = []

        def advance(target, pose):
            visited.append(target['time'])
            clock[0] += .16  # Slower than the previous fixed 120 ms stride.
            while t.history[-1]['time'] + 1 / 15 <= clock[0]:
                t.history.append(
                    {
                        'sequence': t.history[-1]['sequence'] + 1,
                        'time': t.history[-1]['time'] + 1 / 15,
                    }
                )
            return pose

        with mock.patch.object(
            live_tracker.time, 'time', side_effect=lambda: clock[0]
        ), mock.patch.object(
            live_tracker.time, 'perf_counter', side_effect=lambda: clock[0]
        ):
            result = t.catch_up(0, frame, np.eye(4), advance)
        self.assertIsNotNone(result)
        self.assertLessEqual(clock[0] - result[0]['time'], .25)
        self.assertLessEqual(result[3], 12.2)
        self.assertTrue(
            all(0 < delta <= .300001 for delta in np.diff([94.] + visited))
        )
        events = [
            json.loads(line)
            for line in (t.output / 'events.jsonl').read_text().splitlines()
        ]
        self.assertIn('catchup_progress', [e['event'] for e in events])
        self.assertEqual(events[-1]['event'], 'catchup_completed')

    def test_adaptive_catchup_still_rejects_quality_failure(self):
        t = self.tracker
        now = time.time()
        frames = [{'sequence': i, 'time': now - 1 + i * .1} for i in range(11)]
        t.history.extend(frames)

        def advance(frame, pose):
            raise ValueError('bad pose')

        with self.assertRaisesRegex(ValueError, 'bad pose'):
            t.catch_up(0, frames[0], np.eye(4), advance)
        self.assertIsNone(t.state['camera_T_object'])

    def test_object_switch_invalidates_pending_inference_and_old_overlay(self):
        t = self.tracker
        target = t.models_root / 'mug_new/model'
        target.mkdir(parents=True)
        (target / 'model.obj').write_text('# fixture')
        t.state.update(
            state='tracking',
            model_ready=True,
            camera_T_object=np.eye(4).tolist(),
            pose_frame_at=time.time(),
        )
        t.jpeg = b'old overlay'
        t.command = (0, {}, [1, 1, 20, 20])
        t.switch_object('mug_new')
        self.assertEqual(t.state['state'], 'switching')
        self.assertIsNone(t.command)
        self.assertIsNone(t.jpeg)
        self.assertIsNone(t.snapshot()['camera_T_object'])
        self.assertFalse(t.state['model_ready'])
        self.assertEqual(t.pending_object[1]['detector_class'], 'cup')
        self.assertIsNone(t.catch_up(0, {}, np.eye(4), lambda *args: None))

    def test_grasp_snapshot_accepts_five_poses_in_one_second(self):
        t = self.tracker
        pose = np.eye(4)
        pose[2, 3] = .4
        t.state.update(
            state='tracking',
            quality={'good': True},
            camera_T_object=pose.tolist(),
            pose_frame_at=100.,
            object_id='cup',
        )
        with mock.patch.object(live_tracker.time, 'time', return_value=100.1):
            for times, accepted in [
                ([99., 99.25, 99.5, 99.75, 100.], True),
                ([99.25, 99.5, 99.75, 100.], False),
                ([98.999, 99.25, 99.5, 99.75, 100.], False),
                ([99.6, 99.7, 99.8, 99.9, 100.], False),
            ]:
                t.pose_history.clear()
                for stamp in times:
                    t.pose_history.append(
                        {
                            'pose': pose.tolist(),
                            'time': stamp,
                            'generation': t.generation,
                        }
                    )
                with self.subTest(times=times):
                    if accepted:
                        self.assertEqual(
                            t.grasp_observation()['object_id'], 'cup'
                        )
                    else:
                        with self.assertRaisesRegex(
                            ValueError, '至少 5 个有效位姿'
                        ):
                            t.grasp_observation()

    def test_grasp_snapshot_rejects_old_or_unstable_target(self):
        t = self.tracker
        now = time.time()
        pose = np.eye(4)
        pose[2, 3] = .4
        t.state.update(
            state='tracking',
            quality={'good': True},
            camera_T_object=pose.tolist(),
            pose_frame_at=now - .1,
            object_id='bottle',
        )
        for i in range(8):
            t.pose_history.append(
                {
                    'pose': pose.tolist(),
                    'time': now - .8 + i * .1,
                    'generation': t.generation,
                }
            )
        self.assertEqual(t.grasp_observation()['object_id'], 'bottle')
        with mock.patch.object(live_tracker.time, 'time', return_value=now):
            for age in (.4, .5, .501):
                t.state['pose_frame_at'] = now - age
                t.pose_history.clear()
                for i in range(8):
                    t.pose_history.append(
                        {
                            'pose': pose.tolist(),
                            'time': now - age - .7 + i * .1,
                            'generation': t.generation,
                        }
                    )
                if age <= .5:
                    self.assertEqual(
                        t.grasp_observation()['object_id'], 'bottle'
                    )
                else:
                    with self.assertRaisesRegex(ValueError, '过期'):
                        t.grasp_observation()
        t.pose_history.clear()
        for i in range(8):
            t.pose_history.append(
                {
                    'pose': pose.tolist(),
                    'time': now - .8 + i * .1,
                    'generation': t.generation,
                }
            )
        t.state['pose_frame_at'] = now - 1
        with self.assertRaisesRegex(ValueError, '过期'):
            t.grasp_observation()
        t.state['pose_frame_at'] = now - .1
        t.pose_history[0]['pose'][0][3] = .1
        with self.assertRaisesRegex(ValueError, '变化'):
            t.grasp_observation()


if __name__ == '__main__':
    unittest.main()
