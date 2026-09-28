"""TCP arrival and IK->FK regression tests; no robot connections."""
import copy
import json
import os
from pathlib import Path
from tempfile import TemporaryDirectory
from types import SimpleNamespace
import unittest
from unittest.mock import Mock,patch

import numpy as np
from grasp_control import GraspController
from motion_validation import parse_kinematics,check_waypoints,wait_for_pose
import test_grasp


def protocol(pose='0 0 0 0 0 0',index=0):
    return f'PROTOCOL IK_FK_V1\nFK 0 0 0 0 0 0\nIK 0 0 0 0 0 0 0 0 0\nVERIFY {index} {pose}\nRESIDUAL {index} 0 0\n'


class Clock:
    def __init__(self):self.t=0.;self.sleeps=[]
    def now(self):return self.t
    def sleep(self,seconds):self.sleeps.append(seconds);self.t+=seconds


class MotionValidationTests(unittest.TestCase):
    def setUp(self):
        self.arm={'joint':[0]*7,'pose':[0]*6,'err':[0]}
        self.points=[{'name':'pregrasp','controller_pose':[0]*6}]

    def parse(self,stdout,code=0):
        return parse_kinematics(SimpleNamespace(stdout=stdout,stderr='',returncode=code),self.arm,self.points)

    def test_verified_success(self):
        ok,detail=self.parse(protocol());self.assertTrue(ok);self.assertTrue(detail['verified'])
        self.assertEqual(detail['fk_checks'][0]['position_error_mm'],0)

    def test_linear_branch_jump_rejected_but_joint_endpoint_allowed(self):
        data=protocol().replace('IK 0 0 0 0 0 0 0 0 0','IK 0 0 38.3 0 0 0 0 0 0')
        self.points[0]['method']='linear'
        ok,detail=self.parse(data)
        self.assertFalse(ok);self.assertFalse(detail['verified'])
        self.assertEqual(detail['continuity_failure']['joint_index'],1)
        self.points[0]['method']='joint_pose'
        self.assertTrue(self.parse(data)[0])
        self.points[0]['method']='linear'
        self.assertTrue(self.parse(data.replace('38.3','9.9'))[0])

    def test_ik_success_and_false_residual_cannot_hide_wrong_fk(self):
        for pose in ['.16 0 0 0 0 0','0 0 0 0 0 .1']:
            ok,detail=self.parse(protocol(pose))
            self.assertFalse(ok);self.assertIn('逆解返回成功',detail['reason'])

    def test_rejects_legacy_missing_duplicate_and_nonfinite_proof(self):
        samples=[protocol().replace('PROTOCOL IK_FK_V1\n',''),protocol().replace('VERIFY 0 0 0 0 0 0 0\n',''),
                 protocol()+ 'VERIFY 0 0 0 0 0 0 0\n',protocol(index=1),protocol('nan 0 0 0 0 0'),
                 protocol().replace('IK 0 0','IK 0 1'),protocol().replace('IK 0 0 0 0 0 0 0 0 0','IK 0 0 nan 0 0 0 0 0 0')]
        for data in samples:
            with self.subTest(data=data):self.assertFalse(self.parse(data)[0])

    def test_initial_fk_must_match_controller_feedback(self):
        ok,detail=self.parse(protocol().replace('FK 0 0 0 0 0 0','FK .01 0 0 0 0 0'))
        self.assertFalse(ok);self.assertIn('当前机械臂反馈不一致',detail['reason'])

    def test_bad_fk_diagnostic_survives_nonzero_exit_and_incomplete_path(self):
        self.points.append({'name':'grasp','controller_pose':[0]*6})
        ok,detail=self.parse(protocol('.2 0 0 0 0 0'),3)
        self.assertFalse(ok);self.assertIn('200.00',detail['reason'])

    def test_valid_residual_still_requires_success_exit(self):
        self.assertFalse(self.parse(protocol(),1)[0])

    def test_joint_continuity_retained_after_fk_validation(self):
        ok,detail=self.parse(protocol().replace('IK 0 0 0 0 0 0 0 0 0','IK 0 0 30 0 0 0 0 0 0'))
        self.assertTrue(ok);self.assertEqual(detail['max_joint_step_deg'],30)

    def wait(self,poses):
        clock=Clock();record={};rows=iter(poses);last=[poses[-1]]
        def read():
            last[0]=next(rows,last[0]);return {'pose':last[0],'joint':[0]*7,'err':[0]}
        read=Mock(side_effect=read)
        return clock,record,read

    def test_small_transient_settles_without_loosening_final_tolerance(self):
        clock,record,read=self.wait([[0,0,5690,0,0,0],[0,0,2080,0,0,0],[0,0,900,0,0,0],[0,0,850,0,0,0],[0,0,810,0,0,0]])
        result=wait_for_pose(read,[0]*6,lambda:None,record,clock=clock.now,sleep=clock.sleep)
        self.assertTrue(record['settled']);self.assertEqual(result['pose'][2],810)
        self.assertEqual(len(record['samples']),5);self.assertAlmostEqual(clock.t,.2)

    def test_large_error_fails_on_first_read(self):
        clock,record,read=self.wait([[160000,0,0,0,0,0]])
        with self.assertRaisesRegex(ValueError,'立即停止'):wait_for_pose(read,[0]*6,lambda:None,record,clock=clock.now,sleep=clock.sleep)
        self.assertEqual(read.call_count,1);self.assertEqual(clock.sleeps,[])

    def test_persistent_millimetre_error_is_not_accepted(self):
        clock,record,read=self.wait([[0,0,2080,0,0,0]])
        with self.assertRaisesRegex(ValueError,'位置与目标不符'):wait_for_pose(read,[0]*6,lambda:None,record,clock=clock.now,sleep=clock.sleep)
        self.assertFalse(record['settled']);self.assertLessEqual(len(record['samples']),20)

    def test_in_tolerance_but_oscillating_feedback_is_rejected(self):
        clock,record,read=self.wait([[(-1)**i*1800,0,0,0,0,0] for i in range(25)])
        with self.assertRaisesRegex(ValueError,'稳定窗口'):wait_for_pose(read,[0]*6,lambda:None,record,clock=clock.now,sleep=clock.sleep)

    def test_orientation_grace_and_gross_rejection(self):
        for angle,word in [(100,'朝向'),(300,'立即停止')]:
            clock,record,read=self.wait([[0,0,0,angle,0,0]])
            with self.assertRaisesRegex(ValueError,word):wait_for_pose(read,[0]*6,lambda:None,record,clock=clock.now,sleep=clock.sleep)
            if angle==300:self.assertEqual(read.call_count,1)

    def test_cancel_during_settling_does_not_read_again(self):
        clock,record,read=self.wait([[0,0,1000,0,0,0]])
        def cancel():
            if clock.t>=.05:raise ValueError('cancel')
        with self.assertRaisesRegex(ValueError,'cancel'):wait_for_pose(read,[0]*6,cancel,record,clock=clock.now,sleep=clock.sleep)
        self.assertEqual(read.call_count,1)

    def test_fk_failure_at_stage_never_sends_arm_command(self):
        with TemporaryDirectory() as directory:
            root=Path(directory);config=test_grasp.GraspTests().config();config['ik_checker']='checker'
            (root/'checker').touch();cfg=root/'config.json';cfg.write_text(json.dumps(config))
            service=GraspController(root,None,cfg);service.robot=Mock();report={}
            plan={'flange_T_grip_tcp':np.eye(4).tolist()}
            with patch('grasp_control.check_waypoints',return_value=(False,{'reason':'正解误差 160 毫米'})):
                with self.assertRaisesRegex(ValueError,'正解误差'):service.verify_motion_targets(plan,config,report,self.arm,self.points)
            service.robot.move_joint_pose.assert_not_called();service.robot.move_linear.assert_not_called();service.robot.move_gripper.assert_not_called()
            self.assertEqual(report['kinematics_checks'][0]['reason'],'正解误差 160 毫米')

    @unittest.skipUnless(os.environ.get('SAYDO_IK_CHECKER'),'Set SAYDO_IK_CHECKER to the newly built offline verifier')
    def test_native_historical_false_successes_and_valid_target(self):
        tool=np.eye(4);tool[2,3]=.1905
        cases=[
            ([-43681,-71383,763,132967,-2416,67925,120557],[135992,-147311,209749,2315,-356,440],[458760,-317868,90712,2323,-393,2414],False),
            ([-84386,-71385,90103,59959,20346,94939,200354],[300868,139500,154484,2276,-385,301],[463924,-61884,55889,2218,-432,-811],False),
            ([-84387,-71385,90104,59962,20348,94941,200352],[300853,139481,154464,2276,-385,301],[463300,-60614,54824,2189,-415,-762],False),
            ([-43683,-71382,763,132968,-2415,67928,120559],[135983,-147300,209720,2315,-356,440],[363798,-267227,6124,2205,-117,463],True)]
        for joints,pose,target,expected in cases:
            with self.subTest(target=target):
                valid,detail=check_waypoints(os.environ['SAYDO_IK_CHECKER'],{'flange_T_grip_tcp':tool.tolist()},
                    {'joint':joints,'pose':pose},[{'name':'pregrasp','controller_pose':target}])
                self.assertEqual(valid,expected,detail)
                if not expected:self.assertGreater(detail['fk_checks'][0]['position_error_mm'],150)

if __name__=='__main__':unittest.main()
