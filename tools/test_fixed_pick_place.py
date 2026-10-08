import argparse
import tempfile
import unittest
from unittest.mock import Mock,patch
from tools.fixed_pick_place import workflow,wait_task,box,main
from tools.robot_agent.client import BackendError


class WorkflowTests(unittest.TestCase):
    def test_fixed_planner_wait_has_no_elapsed_deadline(self):
        backend=Mock();backend.action.return_value={'terminal':False}
        backend.request.side_effect=[{'terminal':False},{'terminal':True,'state':'fixed_pick_place_planned','result':{'ok':True}}]
        with patch('tools.fixed_pick_place.time.monotonic',side_effect=[0,10000,20000]),patch('tools.fixed_pick_place.time.sleep'):
            self.assertTrue(wait_task(backend,'plan_fixed_pick_place',{},'id',timeout=1)['ok'])
        backend.action.assert_called_once()

    def setUp(self):
        self.temp=tempfile.TemporaryDirectory();self.addCleanup(self.temp.cleanup)
        self.args=argparse.Namespace(tube='tube',rack='rack',tube_box=[0,0,10,10],rack_box=[20,20,50,50],
            free_slots='A01,A02',empty_rack=False,release_gap_mm=3,execute=False,output=self.temp.name)
        self.backend=Mock();self.backend.request.return_value={'grasp':{'mode':'execute'}}
        self.plan={'plan_id':'p','selected_slot':'A02','selected_slot_distance_m':.0831,'nearer_slots_rejected':['A01'],
                   'complete_path_prechecked':True,'validation_scope':{}}

    def test_default_plan_only_and_explicit_execute_preserve_plan_id(self):
        results=[{'scene_id':'scene'},{'fixed_pick_place_plan':self.plan},{}]
        with patch('tools.fixed_pick_place.wait_task',side_effect=results) as wait:
            workflow(self.backend,self.args)
        self.assertEqual([c.args[1] for c in wait.call_args_list],['capture_scene_snapshot','plan_fixed_pick_place'])
        self.args.execute=True
        with patch('tools.fixed_pick_place.wait_task',side_effect=results) as wait:
            workflow(self.backend,self.args)
        self.assertEqual(wait.call_args_list[-1].args[1:3],('execute_fixed_pick_place',{'plan_id':'p'}))

    def test_failed_planning_never_dispatches_execute(self):
        self.args.execute=True
        with patch('tools.fixed_pick_place.wait_task',side_effect=[{'scene_id':'s'},RuntimeError('no path')]) as wait:
            with self.assertRaises(RuntimeError):workflow(self.backend,self.args)
        self.assertEqual(wait.call_count,2)

    def test_keyboard_interrupt_cancels_only_original_readonly_planning(self):
        with patch('tools.fixed_pick_place.wait_task',side_effect=[{'scene_id':'s'},KeyboardInterrupt()]):
            with self.assertRaises(KeyboardInterrupt):workflow(self.backend,self.args)
        path,args=self.backend.request.call_args.args
        self.assertEqual(path,'/api/agent/cancel-planning');self.assertTrue(args['task_id'].endswith('-plan'))

    def test_omitted_tube_box_requests_automatic_scene_detection(self):
        self.args.tube_box=None
        with patch('tools.fixed_pick_place.wait_task',side_effect=[{'scene_id':'s'},{'fixed_pick_place_plan':self.plan}]) as wait:
            workflow(self.backend,self.args)
        self.assertIsNone(wait.call_args_list[0].args[2]['targets'][0]['box'])
        self.assertEqual(wait.call_args_list[0].args[2]['targets'][1]['box'],self.args.rack_box)

    def test_backend_rejecting_the_request_format_explains_the_version_mismatch(self):
        import contextlib,io
        rejected=BackendError({'code':'INVALID_ARGUMENTS','message':'未知的动作或参数。'});output=io.StringIO()
        with patch('sys.argv',['fixed_pick_place','--empty-rack']),patch('tools.fixed_pick_place.RobotBackend'),\
             patch('tools.fixed_pick_place.workflow',side_effect=rejected),contextlib.redirect_stderr(output),\
             self.assertRaises(SystemExit) as stopped:main()
        self.assertEqual(stopped.exception.code,2)
        self.assertIn('restart_tracker.py',output.getvalue());self.assertIn('未发送机械臂或夹爪动作',output.getvalue())
        other=BackendError({'code':'REQUEST_UNCERTAIN'})
        with patch('sys.argv',['fixed_pick_place','--empty-rack']),patch('tools.fixed_pick_place.RobotBackend'),\
             patch('tools.fixed_pick_place.workflow',side_effect=other),self.assertRaises(BackendError):main()

    def test_omitted_rack_box_is_accepted_and_requests_automatic_detection(self):
        with patch('sys.argv',['fixed_pick_place','--empty-rack']),patch('tools.fixed_pick_place.RobotBackend'),\
             patch('tools.fixed_pick_place.workflow') as run:main()
        self.assertIsNone(run.call_args.args[1].rack_box);self.assertIsNone(run.call_args.args[1].tube_box)
        self.args.tube_box=self.args.rack_box=None
        with patch('tools.fixed_pick_place.wait_task',side_effect=[{'scene_id':'s'},{'fixed_pick_place_plan':self.plan}]) as wait:
            workflow(self.backend,self.args)
        self.assertEqual([t['box'] for t in wait.call_args_list[0].args[2]['targets']],[None,None])

    def test_ambiguous_submission_queries_same_id_without_resending(self):
        self.backend.action.side_effect=BackendError({'code':'REQUEST_UNCERTAIN'})
        self.backend.request.return_value={'terminal':True,'state':'completed','result':{'ok':True}}
        self.assertTrue(wait_task(self.backend,'execute_fixed_pick_place',{},'fixed-id')['ok'])
        self.backend.action.assert_called_once();self.backend.request.assert_called_once_with('/api/agent/tasks/fixed-id')

    def test_execute_failure_stops_and_does_not_retry(self):
        self.args.execute=True
        with patch('tools.fixed_pick_place.wait_task',side_effect=[{'scene_id':'s'},{'fixed_pick_place_plan':self.plan},RuntimeError('failed')]):
            with self.assertRaises(RuntimeError):workflow(self.backend,self.args)
        self.backend.request.assert_called_with('/api/agent/stop',{})

    def test_boxes_use_eye_in_hand_dimensions(self):
        self.assertEqual(box('0,0,640,360'),[0,0,640,360])
        with self.assertRaises(argparse.ArgumentTypeError):box('0,0,1280,720')
