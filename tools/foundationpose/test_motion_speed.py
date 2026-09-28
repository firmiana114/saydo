import copy
import fcntl
import json
from pathlib import Path
import unittest
from unittest.mock import Mock,patch
import numpy as np
from motion_speed import checked_speed,reorientation_speed_settings,placement_speed_settings
import test_operator_controls
import test_reorientation_retry
import test_placement


class SpeedConfigTests(unittest.TestCase):
    setUp=test_operator_controls.OperatorControlsTests.setUp

    def test_atomic_hot_update_preserves_other_config_hold_and_old_task_snapshot(self):
        original=json.loads(self.path.read_text());held={'hold_id':'preserve'};self.service.held_object=held
        revision=self.service.motion_speed_configuration()['revision']
        with patch('grasp_robot.socket.create_connection',side_effect=AssertionError('no hardware')):
            result=self.service.set_arm_speed(20,revision)
        current=json.loads(self.path.read_text());expected=copy.deepcopy(original);expected['arm_speed_percent']=20
        self.assertEqual(current,expected);self.assertIs(self.service.held_object,held)
        self.assertNotEqual(original['arm_speed_percent'],20);self.assertFalse(result['motion_sent'])
        self.assertEqual(reorientation_speed_settings(current,{'arm_speed_percent':5})['arm_speed_percent'],20)
        self.assertEqual(placement_speed_settings(current,{'transfer_speed_percent':10,'descent_speed_percent':5}),{'transfer_speed_percent':20,'descent_speed_percent':20})
        self.assertEqual(self.service.motion_speed_configuration()['value'],20)
        with self.assertRaisesRegex(ValueError,'变化'):self.service.set_arm_speed(21,revision)
        with patch('grasp_control.threading.Thread') as thread:
            self.service.start_reset('zero');self.assertEqual(thread.call_args.kwargs['args'][0]['arm_speed_percent'],20)
        with self.assertRaisesRegex(ValueError,'进行中'):self.service.set_arm_speed(25,result['revision'])

    def test_invalid_values_busy_lock_and_write_failure_preserve_config(self):
        before=self.path.read_bytes();revision=self.service.motion_speed_configuration()['revision']
        for value in [0,26,25.0,True,'25',None,float('nan')]:
            with self.assertRaises(ValueError):self.service.set_arm_speed(value,revision)
        for flag in ['busy','stopping']:
            self.service.status[flag]=True
            with self.assertRaises(ValueError):self.service.set_arm_speed(25,revision)
            self.service.status[flag]=False
        folder=self.root/'work/grasp';folder.mkdir(parents=True,exist_ok=True)
        with (folder/'robot.lock').open('a') as lock:
            fcntl.flock(lock,fcntl.LOCK_EX|fcntl.LOCK_NB)
            with self.assertRaises(BlockingIOError):self.service.set_arm_speed(25,revision)
        with patch.object(Path,'replace',side_effect=OSError('disk full')):
            with self.assertRaises(OSError):self.service.set_arm_speed(25,revision)
        self.assertEqual(self.path.read_bytes(),before);self.assertFalse(list(self.path.parent.glob('.speed-*')))
        self.assertEqual(checked_speed(1),1);self.assertEqual(checked_speed(25),25)


class ReorientationSpeedTests(unittest.TestCase):
    setUp=test_reorientation_retry.ReorientationRetryTests.setUp
    def test_runner_uses_global_speed_instead_of_legacy_reorientation_speed(self):
        self.config['arm_speed_percent']=20;self.path.write_text(json.dumps(self.config))
        self.service.reorientation_hardware=Mock(return_value=(self.arm,{'flange_T_grip_tcp':np.eye(4).tolist()}))
        with patch('reorientation_control.RobotClient'),patch('reorientation_control.model_geometry',return_value=self.mesh),patch('reorientation_control.plan_reorientation',return_value={'offset_distance_m':0,'waypoints':[]}) as plan:
            self.service.run_reorient(self.held,np.eye(3).tolist(),preview=True)
        self.assertEqual(self.service.status['state'],'reorient_preview')
        self.assertEqual(plan.call_args.args[5]['arm_speed_percent'],20)


class PlacementSpeedTests(unittest.TestCase):
    setUp=test_placement.PlacementControlTests.setUp
    def test_global_speed_changes_effective_revision_and_is_used_by_runner(self):
        self.settings.update(transfer_speed_percent=10,descent_speed_percent=5)
        self.service.placement_path.write_text(json.dumps(self.settings))
        previous=self.service.placement_configuration();speed=self.service.motion_speed_configuration()
        self.service.set_arm_speed(20,speed['revision']);current=self.service.placement_configuration()
        self.assertNotEqual(previous['revision'],current['revision'])
        self.assertEqual(current['settings']['descent_speed_percent'],20)
        with self.assertRaisesRegex(ValueError,'变化'):
            self.service.save_placement_config({'revision':previous['revision'],'region_xy_m':self.settings['region_xy_m'],'release_gap_m':.02})
        saved=self.service.save_placement_config({'revision':current['revision'],'region_xy_m':self.settings['region_xy_m'],'release_gap_m':.02})
        self.assertEqual(saved['settings']['transfer_speed_percent'],20)
        self.service.reorientation_hardware=Mock(return_value=(self.arm,{'flange_T_grip_tcp':np.eye(4).tolist()}))
        mesh={'model_sha256':self.held['model_sha256'],'vertices':np.asarray(self.vertices).tolist()}
        with patch('placement_control.RobotClient'),patch('placement_control.model_geometry',return_value=mesh),patch('placement_control.plan_placement',return_value={}) as plan:
            self.service.run_placement(self.held,saved['settings'],preview=True)
        self.assertEqual(self.service.status['state'],'placement_preview')
        self.assertEqual(plan.call_args.args[4]['transfer_speed_percent'],20)
        self.assertEqual(plan.call_args.args[4]['descent_speed_percent'],20)

if __name__=='__main__':unittest.main()
