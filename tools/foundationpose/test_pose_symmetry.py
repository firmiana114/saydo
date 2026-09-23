import copy
from collections import deque
import json
from pathlib import Path
import tempfile
import threading
import time
from types import SimpleNamespace
import unittest
from unittest.mock import Mock

import numpy as np
from scipy.spatial.transform import Rotation

from object_catalog import scan_objects, save_pose_mode
from pose_symmetry import (canonical_pose, object_rotation_error, checked_axis_profile,
                           effective_profile, grasp_variants, object_bounds, z_rotation,
                           orientation_variants)
from live_tracker import Tracker
from reorientation_geometry import plan_reorientation
from placement_geometry import plan_placement
import test_reorientation
import test_placement
import test_grasp_library


class AxisGeometryTests(unittest.TestCase):
    def test_object_spin_is_free_but_axis_flip_and_tilt_are_not(self):
        rotation = Rotation.from_euler('xyz',[25,39,-70],degrees=True).as_matrix()
        pose = np.eye(4);pose[:3,:3] = rotation;pose[:3,3] = [.1,.2,.3]
        spun = pose.copy();spun[:3,:3] = rotation @ z_rotation(2.1)
        np.testing.assert_allclose(canonical_pose(pose,'5d'),canonical_pose(spun,'5d'),atol=1e-12)
        self.assertAlmostEqual(object_rotation_error(rotation,spun[:3,:3],'5d'),0,places=7)
        self.assertGreater(object_rotation_error(rotation,spun[:3,:3],'6d'),2)
        flipped = rotation @ np.diag([1,-1,-1])
        self.assertAlmostEqual(object_rotation_error(rotation,flipped,'5d'),np.pi)

    def test_grasp_candidates_keep_center_axis_and_angle_under_any_gauge(self):
        base = np.eye(4);base[:3,:3] = Rotation.from_euler('xyz',[20,40,30],degrees=True).as_matrix()
        current = Rotation.from_euler('xyz',[-5,30,75],degrees=True).as_matrix()
        for angle in (0.,37.,90.,180.):
            profile = checked_axis_profile({'grasp_point_object_m':[0,0,.035],'approach_angle_deg':angle})
            one = grasp_variants(base,profile,current)
            spun = base.copy();spun[:3,:3] = base[:3,:3] @ z_rotation(.71)
            two = grasp_variants(spun,profile,current)
            self.assertEqual(len(one),48)
            for (a,_),(b,_) in zip(one,two):
                world = base[:3,:3] @ a['object_R_tcp']
                np.testing.assert_allclose(world,spun[:3,:3]@b['object_R_tcp'],atol=1e-12)
                self.assertAlmostEqual(np.dot(world[:,2],base[:3,2]),np.cos(np.deg2rad(angle)))
                self.assertAlmostEqual(np.dot(world[:,0],base[:3,2]),0)
                self.assertAlmostEqual(np.linalg.det(world),1)
                self.assertEqual(a['grasp_point_object_m'],[0.,0.,.035])
        for value in ({'grasp_point_object_m':[.001,0,0],'approach_angle_deg':90},
                      {'grasp_point_object_m':[0,0,0],'approach_angle_deg':True},
                      {'grasp_point_object_m':[0,0,0],'approach_angle_deg':181},None):
            with self.assertRaises(ValueError):checked_axis_profile(value)

    def test_nearest_free_spin_preserves_current_tcp_when_axis_already_matches(self):
        held={'pose_mode':'5d','object_T_tcp':np.eye(4).tolist()}
        for angles in ([31,58,130],[-170,0,20],[0,180,0]):
            current=np.eye(4);current[:3,:3]=Rotation.from_euler('xyz',angles,degrees=True).as_matrix()
            requested=current[:3,:3]@z_rotation(1.4)
            variants=orientation_variants(requested,held,current)
            np.testing.assert_allclose(variants[0],current[:3,:3],atol=1e-12)

    def test_rotational_sweep_bounds_cover_all_mesh_spins(self):
        points=np.array([[.03,.01,-.07],[-.02,.04,.06],[.02,-.01,.02]])
        pose=np.eye(4);pose[:3,:3]=Rotation.from_euler('xyz',[17,53,29],degrees=True).as_matrix();pose[:3,3]=[.3,0,.2]
        low,high=object_bounds(pose,points,'5d')
        for angle in np.linspace(0,2*np.pi,360):
            spun=pose.copy();spun[:3,:3]=pose[:3,:3]@z_rotation(angle)
            np.testing.assert_allclose(object_bounds(spun,points,'5d'),[low,high],atol=1e-12)
            vertices=points@spun[:3,:3].T+spun[:3,3]
            self.assertTrue(np.all(vertices>=low-1e-12) and np.all(vertices<=high+1e-12))

    def test_reorientation_searches_multiple_twists_before_declaring_failure(self):
        config,settings,held,vertices,arm=test_reorientation.fixture();held['pose_mode']='5d'
        target=Rotation.from_euler('xyz',[35,50,20],degrees=True).as_matrix()
        reachable=Mock(side_effect=[(False,{'reason':'blocked'}),(False,{}),(True,{})])
        plan=plan_reorientation(held,vertices,target,arm,config,settings,reachable,lambda:None,'fixed')
        self.assertEqual(reachable.call_count,3)
        self.assertEqual(plan['orientation_goal'],'object_z_axis')
        np.testing.assert_allclose(np.asarray(plan['base_T_object_target'])[:3,2],target[:,2],atol=1e-12)
        held['pose_mode']='6d';reachable=Mock(return_value=(False,{}))
        with self.assertRaises(ValueError):plan_reorientation(held,vertices,target,arm,config,settings,reachable,lambda:None,'fixed')
        self.assertEqual(reachable.call_count,1)

    def test_placement_is_invariant_to_the_unobservable_spin(self):
        config,settings,held,vertices,arm=test_placement.fixture();held['pose_mode']='5d'
        first=plan_placement(held,vertices,arm,config,settings,Mock(return_value=(True,{})),lambda:None)
        for angle in (.6,1.7,3.2):
            changed=copy.deepcopy(held);spin=np.eye(4);spin[:3,:3]=z_rotation(angle)
            changed['object_T_tcp']=(spin@np.asarray(held['object_T_tcp'])).tolist()
            other=plan_placement(changed,vertices,arm,config,settings,Mock(return_value=(True,{})),lambda:None)
            for key in ['target_tcp_position_m','footprint_min_xy_m','footprint_max_xy_m','object_bottom_z_m']:
                np.testing.assert_allclose(first[key],other[key],atol=1e-10)


class ModelModeTests(unittest.TestCase):
    def setUp(self):
        self.temp=tempfile.TemporaryDirectory();self.addCleanup(self.temp.cleanup);self.root=Path(self.temp.name)
        for name in ('bottle','other'):
            folder=self.root/name;folder.mkdir();(folder/'model.obj').write_text('# test mesh')
            (folder/'tracking.json').write_text(json.dumps({'name':name,'detector_class':'bottle','custom':{'keep':1}}))

    def test_per_model_atomic_update_preserves_other_settings_and_versions(self):
        before=scan_objects(self.root);other=(self.root/'other/tracking.json').read_bytes()
        after=save_pose_mode(self.root,'bottle','5d',before[0]['pose_mode_revision'])
        self.assertEqual(after['pose_mode'],'5d')
        self.assertEqual(json.loads((self.root/'bottle/tracking.json').read_text())['custom'],{'keep':1})
        self.assertEqual((self.root/'other/tracking.json').read_bytes(),other)
        self.assertTrue(list((self.root/'bottle/tracking_backups').glob('*.json')))
        with self.assertRaises(ValueError):save_pose_mode(self.root,'bottle','6d',before[0]['pose_mode_revision'])
        self.assertEqual(save_pose_mode(self.root,'bottle','6d',after['pose_mode_revision'])['pose_mode'],'6d')

    def test_cannot_change_mode_while_busy_or_holding(self):
        tracker=Tracker.__new__(Tracker);tracker.lock=threading.RLock();tracker.models_root=self.root
        tracker.grasp=SimpleNamespace(lock=threading.RLock(),root=self.root,status={'busy':True},held_object=None)
        tracker.state={'object_id':'bottle'};tracker.event=Mock();tracker.switch_object=Mock()
        row=scan_objects(self.root)[0];request={'id':'bottle','pose_mode':'5d','revision':row['pose_mode_revision']}
        with self.assertRaises(ValueError):tracker.set_pose_mode(request)
        tracker.grasp.status['busy']=False;tracker.grasp.held_object={'hold_id':'h'}
        with self.assertRaises(ValueError):tracker.set_pose_mode(request)
        tracker.grasp.held_object=None;result=tracker.set_pose_mode(request)
        self.assertFalse(result['motion_sent']);tracker.switch_object.assert_called_once_with('bottle')

    def test_tracking_stability_uses_axis_only_for_5d(self):
        now=time.time();tracker=Tracker.__new__(Tracker);tracker.lock=threading.RLock();tracker.generation=1
        pose=np.eye(4);pose[:3,3]=[0,0,.4]
        tracker.state={'state':'tracking','quality':{'good':True},'camera_T_object':pose.tolist(),
                       'pose_frame_at':now,'pose_mode':'5d','object_id':'bottle'}
        tracker.pose_history=deque()
        for i in range(7):
            p=pose.copy();p[:3,:3]=z_rotation(i*.5)
            tracker.pose_history.append({'pose':p.tolist(),'time':now-.9+i*.15,'generation':1})
        self.assertEqual(tracker.grasp_observation()['pose_mode'],'5d')
        tracker.state['pose_mode']='6d'
        with self.assertRaisesRegex(ValueError,'朝向不稳定'):tracker.grasp_observation()
        tracker.state['pose_mode']='5d';tracker.pose_history[0]['pose']=np.diag([1,-1,-1,1]).tolist()
        tracker.pose_history[0]['pose'][2][3]=.4
        with self.assertRaisesRegex(ValueError,'朝向不稳定'):tracker.grasp_observation()


class FiveDLibraryTests(unittest.TestCase):
    setUp=test_grasp_library.GraspLibraryTests.setUp
    entry=test_grasp_library.GraspLibraryTests.entry
    save=test_grasp_library.GraspLibraryTests.save
    candidate_args=test_grasp_library.GraspLibraryTests.candidate_args

    def request(self,action='save',**extra):
        request=test_grasp_library.GraspLibraryTests.request(self,action,**extra)
        request.update(pose_mode=self.mesh.get('pose_mode','6d'),pose_mode_revision=self.mesh.get('pose_mode_revision'))
        return request

    def test_existing_6d_grasp_retained_and_5d_saved_separately(self):
        self.profile['grasp_point_object_m']=[.01,.02,.03]
        self.profile['object_R_tcp']=Rotation.from_euler('y',63,degrees=True).as_matrix().tolist()
        result=self.save();ident=result['saved_id'];original=copy.deepcopy(self.library.object('cup')['grasps'][0]['profile'])
        self.mesh.update(pose_mode='5d',pose_mode_revision='v1')
        display=self.service.grasp_profiles('cup')['grasps'][0]['profile']
        self.assertEqual(display['grasp_point_object_m'],[0,0,.03]);self.assertAlmostEqual(display['approach_angle_deg'],63)
        entry=self.entry();entry['profile']={'grasp_point_object_m':[0,0,.025],'approach_angle_deg':80}
        self.service.edit_grasp_profiles(self.request(id=ident,grasp=entry))
        stored=self.library.object('cup')['grasps'][0]
        self.assertEqual(stored['profile'],original);self.assertEqual(stored['profile_5d']['approach_angle_deg'],80)
        self.mesh['pose_mode']='6d';self.assertEqual(self.service.grasp_profiles('cup')['grasps'][0]['profile'],original)

    def test_rejected_azimuth_is_replaced_during_planning_only(self):
        self.profile['object_R_tcp']=Rotation.from_euler('y',90,degrees=True).as_matrix().tolist();self.save()
        args=self.candidate_args();args[1]['pose_mode']='5d'
        self.service.verify_motion_targets=Mock(side_effect=[ValueError('first azimuth unreachable'),{'joint_travel_deg':3}])
        plan,profile,_=self.service.select_grasp_candidate(*args)
        self.assertEqual(self.service.verify_motion_targets.call_count,2)
        self.assertEqual(plan['orientation_policy'],'axisymmetric_z')
        self.assertEqual(plan['pose_mode'],'5d');self.assertEqual(plan['grasp_point_object_m'][:2],[0,0])
        self.assertEqual(len(plan['grasp_candidates'][0]['azimuth_attempts']),2)

    def test_mode_changes_invalidate_old_editor_requests(self):
        request=self.request(grasp=self.entry());self.mesh.update(pose_mode='5d',pose_mode_revision='v2')
        with self.assertRaisesRegex(ValueError,'位姿模式'):self.service.edit_grasp_profiles(request)


if __name__=='__main__':unittest.main()
