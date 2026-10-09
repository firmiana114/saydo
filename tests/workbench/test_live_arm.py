"""切换实时跟踪所在的臂：定位模型与相机是替身，其余是真实代码。"""

import collections
import json
import pathlib
import tempfile
import threading
import types
import unittest
from unittest import mock

from saydo.robot import arms
from saydo.workbench import agent_router
from saydo.workbench import live_tracker
from tests.robot import fake_controller

SECOND_SERIAL = '344422070678'


def frame(serial, sequence, time_s=100.):
    """造一帧只含跟踪器会读取的字段的画面。"""
    return {
        'sequence': sequence,
        'time': time_s,
        'camera': {'serial': serial},
        'sensor_age_at_receive_s': .01,
    }


class LiveArmTestCase(unittest.TestCase):
    def setUp(self):
        directory = tempfile.TemporaryDirectory()
        self.addCleanup(directory.cleanup)
        self.root = pathlib.Path(directory.name)
        for arm_id, ip in (('arm1', '10.0.0.1'), ('arm2', '10.0.0.2')):
            fake_controller.write_arm_config(
                self.root, arms.by_id(arm_id), ip, [0] * 7
            )
        self.events = []
        # 跟踪器不走完整的启动流程（那会加载定位模型）；只准备切换臂
        # 用到的成员，控制器和视觉来源由真实代码创建。
        tracker = live_tracker.Tracker.__new__(live_tracker.Tracker)
        tracker.models_root = self.root / 'models'
        tracker.lock = threading.RLock()
        tracker.llm_detector = None
        tracker.history = collections.deque(maxlen=10)
        tracker.pose_history = collections.deque(maxlen=10)
        tracker.latest = tracker.frozen = tracker.command = None
        tracker.jpeg = b'old'
        tracker.generation = 7
        tracker.last_attempt_sequence = 55
        tracker.scene_job = None
        tracker.live_arm_id = 'arm1'
        tracker.state = {
            'state': 'tracking', 'error': None, 'model_ready': True,
            'camera_T_object': [[1]], 'live_arm': 'arm1',
        }
        tracker.event = lambda name, **values: self.events.append(
            (name, values)
        )
        self.tracker = tracker
        self.configure_second_camera()
        with mock.patch.object(live_tracker, 'ROOT', self.root):
            tracker.arms = tracker._create_controllers(
                types.SimpleNamespace()
            )
        tracker.grasp = tracker.arms['arm1']
        self.first = tracker.visions['arm1']
        self.second = tracker.visions['arm2']

    def configure_second_camera(self):
        path = arms.by_id('arm2').config_path(self.root)
        config = json.loads(path.read_text())
        config['camera_serial'] = SECOND_SERIAL
        path.write_text(json.dumps(config))


class SwitchTests(LiveArmTestCase):
    def test_frames_reach_the_tracker_only_from_the_live_arm(self):
        self.second.publish(frame(SECOND_SERIAL, 3))
        self.assertIsNone(self.tracker.latest)
        self.first.publish(frame('first', 4))
        self.assertEqual(self.tracker.latest['camera'], {'serial': 'first'})
        self.assertEqual(self.tracker.state['sequence'], 4)
        self.assertEqual(self.tracker.state['camera'], {'serial': 'first'})
        self.assertEqual(len(self.tracker.history), 1)
        # 没在实时跟踪的臂仍保留自己的最近一帧，供场景快照使用。
        self.assertEqual(self.second.latest['sequence'], 3)

    def test_switch_moves_tracking_and_page_actions_to_the_other_arm(self):
        self.first.publish(frame('first', 4))
        self.second.publish(frame(SECOND_SERIAL, 9, 101.))
        result = self.tracker.set_live_arm('arm2')
        self.assertEqual(
            result, {'live_arm': 'arm2', 'changed': True, 'motion_sent': False}
        )
        self.assertIs(self.tracker.grasp, self.tracker.arms['arm2'])
        # 旧相机算到一半的结果、冻结画面和历史帧全部作废。
        self.assertEqual(self.tracker.generation, 8)
        self.assertIsNone(self.tracker.jpeg)
        self.assertEqual(len(self.tracker.history), 0)
        self.assertEqual(self.tracker.last_attempt_sequence, -1)
        state = self.tracker.state
        self.assertEqual(state['state'], 'paused')
        self.assertIsNone(state['camera_T_object'])
        self.assertEqual(state['live_arm'], 'arm2')
        # 切换后立刻用新相机已有的最近一帧，标定核对读到的是新相机。
        self.assertEqual(state['camera'], {'serial': SECOND_SERIAL})
        self.assertEqual(state['camera_frame_at'], 101.)
        self.assertEqual(self.tracker.latest['sequence'], 9)
        self.assertEqual(
            self.events[-1],
            ('live_arm_changed', {'previous': 'arm1', 'arm': 'arm2'}),
        )
        # 之后 1 臂的画面不再进入跟踪器。
        self.first.publish(frame('first', 5))
        self.assertEqual(self.tracker.latest['sequence'], 9)
        self.second.publish(frame(SECOND_SERIAL, 10, 102.))
        self.assertEqual(self.tracker.latest['sequence'], 10)

    def test_each_controller_sees_live_data_only_while_its_arm_is_live(
        self,
    ):
        self.first.publish(frame('first', 4))
        self.second.publish(frame(SECOND_SERIAL, 9))
        first, second = self.tracker.arms['arm1'], self.tracker.arms['arm2']
        self.assertIs(first.tracker.state, self.tracker.state)
        self.assertIs(first.tracker.lock, self.tracker.lock)
        self.assertEqual(
            second.tracker.state['state'], 'scene_snapshot_only'
        )
        with self.assertRaisesRegex(ValueError, '实时跟踪现在接在1臂'):
            second.tracker.freeze()
        self.tracker.set_live_arm('arm2')
        self.assertIs(second.tracker.state, self.tracker.state)
        self.assertIs(second.tracker.lock, self.tracker.lock)
        self.assertEqual(first.tracker.state['state'], 'scene_snapshot_only')
        self.assertEqual(first.tracker.latest['sequence'], 4)
        self.assertIsNone(first.tracker.frozen)
        # 1 臂不能再用实时跟踪的位姿去抓：否则会拿 2 臂相机看到的
        # 位姿驱动 1 臂。
        with self.assertRaisesRegex(
            ValueError, '实时跟踪现在接在2臂.*1臂.*场景快照'
        ):
            first.tracker.grasp_observation()

    def test_scene_capture_bookkeeping_lands_on_the_right_side(self):
        # 场景采集会把代次加一并清掉待处理的指令。
        self.second.generation += 1
        self.second.command = ('x',)
        self.assertEqual(self.tracker.generation, 7)
        self.assertIsNone(self.tracker.command)
        self.first.generation += 1
        self.first.command = None
        self.assertEqual(self.tracker.generation, 8)

    def test_switching_to_the_live_arm_changes_nothing(self):
        result = self.tracker.set_live_arm('arm1')
        self.assertFalse(result['changed'])
        self.assertEqual(self.tracker.generation, 7)
        self.assertEqual(self.tracker.state['state'], 'tracking')

    def test_switch_without_a_frame_yet_leaves_no_stale_camera(self):
        self.first.publish(frame('first', 4))
        self.tracker.set_live_arm('arm2')
        self.assertIsNone(self.tracker.latest)
        self.assertIsNone(self.tracker.state['camera'])
        self.assertNotIn('camera_frame_at', self.tracker.state)

    def test_switch_is_refused_while_any_arm_is_working(self):
        self.tracker.arms['arm1'].status['busy'] = True
        with self.assertRaisesRegex(ValueError, '1臂.*任务进行中'):
            self.tracker.set_live_arm('arm2')
        self.tracker.arms['arm1'].status['busy'] = False
        self.tracker.arms['arm2'].status['stopping'] = True
        with self.assertRaisesRegex(ValueError, '2臂.*任务进行中'):
            self.tracker.set_live_arm('arm2')
        self.assertEqual(self.tracker.live_arm_id, 'arm1')

    def test_switch_is_refused_during_model_loading_or_scene_capture(self):
        self.tracker.state['state'] = 'switching'
        with self.assertRaisesRegex(ValueError, '模型正在切换'):
            self.tracker.set_live_arm('arm2')
        self.tracker.state['state'] = 'paused'
        self.tracker.scene_job = {'done': None}
        with self.assertRaisesRegex(ValueError, '正在采集场景快照'):
            self.tracker.set_live_arm('arm2')

    def test_unknown_arm_is_refused(self):
        with self.assertRaisesRegex(ValueError, "没有配置机械臂 'arm9'"):
            self.tracker.set_live_arm('arm9')
        with self.assertRaisesRegex(ValueError, '没有配置机械臂 None'):
            self.tracker.set_live_arm(None)

    def test_arm_choices_and_agent_summary_mark_the_live_arm(self):
        self.tracker.set_live_arm('arm2')
        self.assertEqual(
            self.tracker.arm_choices(),
            [
                {
                    'arm': 'arm1', 'label': '1臂（左臂）', 'camera': True,
                    'live': False,
                },
                {
                    'arm': 'arm2', 'label': '2臂（右臂）', 'camera': True,
                    'live': True,
                },
            ],
        )
        token = self.root / 'token'
        token.write_text('t' * 40)
        token.chmod(0o600)
        rows = agent_router.ArmRouter(self.tracker, token).summary()['arms']
        self.assertEqual(
            [(row['arm'], row['live_tracking']) for row in rows],
            [('arm1', False), ('arm2', True)],
        )


class NoCameraTests(LiveArmTestCase):
    def configure_second_camera(self):
        """2 臂的配置里不写相机序列号。"""

    def test_arm_without_a_camera_cannot_become_live(self):
        with self.assertRaisesRegex(ValueError, '2臂.*没有接相机'):
            self.tracker.set_live_arm('arm2')
        self.assertEqual(self.tracker.live_arm_id, 'arm1')
        self.assertEqual(
            [row['camera'] for row in self.tracker.arm_choices()],
            [True, False],
        )


if __name__ == '__main__':
    unittest.main()
