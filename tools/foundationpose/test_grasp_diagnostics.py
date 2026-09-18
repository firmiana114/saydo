"""Detailed blocking diagnostics without hardware commands."""
from pathlib import Path
import tempfile
from types import SimpleNamespace
import unittest
from unittest.mock import patch

from grasp_control import GraspController
from grasp_geometry import workspace_blockers, minimum_table_clearance
import test_grasp


class DiagnosticTests(unittest.TestCase):
    def test_configurable_table_clearance_and_validation(self):
        config=test_grasp.GraspTests().config();config['workspace_min_m'][2]=0
        plan={'current_position_m':[0,0,.2], 'waypoints':[{'name':'grasp','position_m':[0,0,.006]}]}
        for minimum, blocked in [(0,False),(.005,False),(.006,False),(.01,True),(.02,True)]:
            config['min_table_clearance_m']=minimum
            reasons=workspace_blockers(plan,config)
            self.assertEqual(bool(reasons),blocked)
            if blocked:self.assertIn(f'要求至少 {minimum*1000:.3f}',reasons[0])
        for invalid in (-.01,True,'0.01',None,float('nan'),float('inf')):
            config['min_table_clearance_m']=invalid
            with self.assertRaises(ValueError):minimum_table_clearance(config)
            self.assertIn('min_table_clearance_m',workspace_blockers(plan,config)[0])
        config.pop('min_table_clearance_m')
        self.assertEqual(minimum_table_clearance(config),.01)

    def test_workspace_reports_all_points_axes_and_clearance(self):
        config=test_grasp.GraspTests().config()
        config['workspace_min_m']=[-1,-1,0]
        plan={'current_position_m':[0,0,.2],'waypoints':[
            {'name':'grasp','position_m':[1.1,0,.006457]},
            {'name':'lift','position_m':[0,1.2,.2]}]}
        reasons=workspace_blockers(plan,config)
        self.assertEqual(len(reasons),3)
        self.assertIn('夹持点：X=1100.000',reasons[0])
        self.assertIn('6.457 毫米',reasons[1])
        self.assertIn('差 3.543 毫米',reasons[1])
        self.assertIn('抬升点：Y=1200.000',reasons[2])

    def test_inverse_failure_collision_and_jump_are_not_conflated(self):
        with tempfile.TemporaryDirectory() as directory:
            root=Path(directory);(root/'checker').touch()
            service=GraspController(root,None,'unused')
            plan={'waypoints':[{'name':'pregrasp','controller_pose':[0,0,200000,0,0,0]}]}
            arm={'joint':[0]*7,'pose':[0,0,200000,0,0,0]}
            tool={'controller_readback':{'tool_response_raw':{'pose':[0]*6}}}
            for code,collision,joint,expected in [(1,-1,0,'未求得关节解'),(-1,-1,0,'输入关节角度'),(-2,-1,0,'四元数无效'),(0,1,0,'合并返回'),(0,0,11,'关节 1 跳变')]:
                output=f'FK 0 0 0.2 0 0 0\nIK {code} {collision} {joint} 0 0 0 0 0 0\n'
                result=SimpleNamespace(returncode=1 if code or collision else 0,stdout=output,stderr='')
                with patch('grasp_control.subprocess.run',return_value=result):
                    check=service.check_ik(plan,arm,tool,{'ik_checker':'checker'})
                self.assertFalse(check['ok'])
                self.assertIn(expected,check['reason'])
                self.assertIn('前往预抓点',check['reason'])
                if code:self.assertNotIn('max_joint_delta_deg',check)

    def test_return_reports_coordinates_and_table_constraint(self):
        service=GraspController('.',None,'unused');config=test_grasp.GraspTests().config();config['ik_checker']='unused'
        result=SimpleNamespace(returncode=1,stdout='RETURN_FAIL 4 0 0 0 0.005\n',stderr='')
        with patch('grasp_control.subprocess.run',return_value=result):
            check=service.check_return([0]*7,[0]*7,{'pose':[0]*6},config)
        self.assertIn('第 4 个采样点',check['reason'])
        self.assertIn('5.000 毫米',check['reason'])
        self.assertNotIn('碰撞',check['reason'])


if __name__=='__main__':unittest.main()
