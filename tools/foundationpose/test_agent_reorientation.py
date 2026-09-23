"""Authenticated reorientation uses simulated workers only."""
import copy
import math
import unittest
from unittest.mock import patch
import numpy as np
from scipy.spatial.transform import Rotation
import test_agent_api as base
from agent_api import AgentError
from reorientation_geometry import rotation_from_rpy_deg


class AgentReorientationTests(unittest.TestCase):
    request=base.AgentAPITests.request
    wait=base.AgentAPITests.wait
    complete=base.AgentAPITests.complete

    def setUp(self):
        base.AgentAPITests.setUp(self)
        self.grasp.held_object={'hold_id':'h','object_id':'bottle','model_sha256':'mesh','orientation_version':0,
            'current_object_rotation':np.eye(3).tolist()}
        self.arguments={'object_id':'bottle','hold_id':'h','orientation_version':0,'target_rpy_deg':[10,20,90],'position_mode':'adaptive'}
        def worker(held,rotation,preview,mode):
            self.calls.append((copy.deepcopy(held),rotation,preview,mode))
            target=np.eye(4);target[:3,:3]=rotation
            if not preview:
                self.grasp.held_object={**held,'orientation_version':held['orientation_version']+1,'current_object_rotation':rotation}
            self.grasp.update(state='reorient_preview' if preview else 'completed',busy=False,
                reorientation_plan={'base_T_object_target':target.tolist(),'orientation_tolerance_deg':1.5})
        self.grasp.run_reorient=worker

    def test_conversion_matches_web_xyz_convention(self):
        for angles in ([0,0,90],[90,0,0],[12,-47,150],[-180,90,180]):
            np.testing.assert_allclose(rotation_from_rpy_deg(list(angles)),Rotation.from_euler('xyz',angles,degrees=True).as_matrix(),atol=1e-12)

    def test_execute_preserves_binding_and_replays_only_original_task(self):
        self.request('reorient',arguments=self.arguments);task=self.wait()
        self.assertEqual(task['state'],'completed')
        self.assertEqual(task['result']['held_object']['orientation_version'],1)
        np.testing.assert_allclose(task['result']['reorientation_plan']['base_T_object_target'][:3],np.c_[rotation_from_rpy_deg(self.arguments['target_rpy_deg']),[0,0,0]])
        self.assertEqual(self.calls[0][0]['model_sha256'],'mesh')
        self.assertEqual(self.calls[0][2:],(False,'adaptive'))
        self.assertTrue(self.request('reorient',arguments=self.arguments)['deduplicated'])
        self.assertEqual(len(self.calls),1)
        with self.assertRaises(AgentError):self.request('reorient',arguments={**self.arguments,'target_rpy_deg':[0,0,0]})
        self.grasp.update(state='failed',reorientation_plan=None)
        self.assertEqual(self.api.get_task(task['task_id']),task)

    def test_preview_keeps_version_and_holding(self):
        self.grasp.mode='preview'
        self.request('preview_reorient',arguments={**self.arguments,'position_mode':'fixed'});task=self.wait()
        self.assertEqual(task['state'],'reorient_preview');self.assertEqual(task['result']['held_object']['orientation_version'],0)
        self.assertEqual(self.calls[0][2:],(True,'fixed'))
        with self.assertRaises(AgentError) as error:self.request('reorient',task_id='execute-disabled',arguments=self.arguments)
        self.assertEqual(error.exception.payload['code'],'EXECUTION_DISABLED')

    def test_missing_hold_or_wrong_identity_rejected_before_worker(self):
        for i,override in enumerate([{'object_id':'cup'},{'hold_id':'old'},{'orientation_version':1}]):
            task=self.request('reorient',task_id=f'bad-held-{i}',arguments={**self.arguments,**override})
            self.assertEqual(task['state'],'rejected');self.assertEqual(task['error']['code'],'HOLD_CHANGED')
        self.grasp.held_object=None
        self.assertEqual(self.request('reorient',arguments=self.arguments)['state'],'rejected')
        self.assertEqual(self.calls,[])

    def test_bad_angles_versions_and_mode_never_reach_worker(self):
        bad=[{'target_rpy_deg':v} for v in ([0,0],[0,0,181],[0,True,0],[0,'1',0],[0,float('nan'),0],[0,float('inf'),0])]
        bad += [{'orientation_version':v} for v in (True,-1,'0')]+[{'position_mode':'anywhere'},{'model_sha256':'override'}]
        for i,override in enumerate(bad):
            with self.assertRaises(AgentError):self.request('reorient',task_id=f'invalid-args-{i}',arguments={**self.arguments,**override})
        self.assertEqual(self.calls,[])

    def test_web_entry_requires_current_version_too(self):
        value={'object_id':'bottle','hold_id':'h','model_sha256':'mesh','rotation':np.eye(3).tolist()}
        with patch('reorientation_control.threading.Thread') as thread:
            for version in (None,True,1):
                with self.assertRaisesRegex(ValueError,'版本'):self.grasp.start_reorient({**value,'orientation_version':version})
            thread.assert_not_called()
            self.grasp.start_reorient({**value,'orientation_version':0},preview=True)
            thread.assert_called_once()
