"""Detailed blocking diagnostics without hardware commands."""
import unittest

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



if __name__=='__main__':unittest.main()
