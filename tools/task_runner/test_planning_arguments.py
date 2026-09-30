from .test_runner import mock_global_camera
import asyncio
import copy
import tempfile
import unittest
from .config import Config
from .robot_adapter import RobotAdapter,Blocked,Cancelled
from .runner import Runner
from .store import Store
from .tool_registry import validate
from .planning_validation import PlanningArgumentError
from .test_runner import FakeLLM,call,end
from .test_object_motion import ObjectBackend
from tools.robot_agent.client import BackendError
from tools.robot_agent.motion_contract import DEFAULT_GOAL,checked_goal,GoalArgumentError

GOAL={**copy.deepcopy(DEFAULT_GOAL),'reference_frame':'scene_object','scene_id':'a'*32,
      'reference_instance_id':'rack_1','reference_feature_id':'A01.support','reference_object_id':'rack_model',
      'target_position_m':[0,0,.075],'target_rpy_deg':[0,0,0]}
ARGS={'object_id':'b','hold_id':'h','orientation_version':0,'goal':GOAL,'retreat_vector_base_m':[0,0,.1]}


class PlanningArgumentsTests(unittest.IsolatedAsyncioTestCase):
    async def asyncSetUp(self):mock_global_camera(self)
    def make(self,backend=None):
        return RobotAdapter(Config(api_key='test',poll_interval=.001),backend or ObjectBackend())

    async def test_external_release_without_position_is_correctable_and_not_submitted(self):
        adapter=self.make();args=copy.deepcopy(ARGS)
        args['goal'].update(reference_object_id=None,reference_feature_id='A06.entry',target_position_m=None)
        result=await adapter.execute('plan_release',args,'missing-position',asyncio.Event(),lambda _:None)
        self.assertTrue(result['argument_correction_required']);self.assertFalse(result['backend_submitted'])
        self.assertEqual(result['error']['field'],'goal.target_position_m')
        self.assertFalse(any(path=='/api/agent/tasks' for path,_ in adapter.backend.requests))
        args['goal']['target_position_m']=[0,0,.002]
        checked_goal(args['goal'],release=True)
        args['goal']['target_position_m']=None
        checked_goal(args['goal'],release=False)  # Orientation-only held motion remains valid.
        checked_goal(None,release=True)  # Explicit current-position release remains separate.

    async def test_real_failed_scene_arguments_correct_then_release_without_regrasp(self):
        adapter=self.make();fixed=copy.deepcopy(ARGS);fixed['goal']['reference_object_id']=None
        llm=FakeLLM(call('plan_release',ARGS,'bad'),call('plan_release',fixed,'fixed'),call('execute_plan',{'plan_id':'b'*32},'release'),end('succeeded',['release']))
        with tempfile.TemporaryDirectory() as d:
            runner=Runner(adapter.config,Store(d),llm,lambda:adapter);ident=runner.submit('继续放置已夹持物品');await runner.worker;r=runner.store.get(ident)
            self.assertEqual(r['state'],'succeeded',r.get('summary'))
            bad=r['calls'][0]['result'];self.assertTrue(bad['argument_correction_required']);self.assertFalse(bad['motion_sent']);self.assertFalse(bad['backend_submitted'])
            self.assertEqual(bad['error']['field'],'goal.reference_object_id');self.assertIsNone(bad['error']['expected'])
            self.assertEqual(bad['retry_budget']['planning_attempts'],1)
            self.assertEqual(r['calls'][1]['result']['retry_budget']['planning_attempts'],2)
            self.assertIn('goal.reference_object_id',str(llm.histories[1]))
            self.assertTrue(any(e['event']=='planning_argument_error' for e in r['events']))
            writes=[v['operation'] for p,v in adapter.backend.requests if p=='/api/agent/tasks']
            self.assertEqual(writes,['plan_release','execute_plan'])
            self.assertEqual(ARGS['goal']['reference_object_id'],'rack_model','preflight must not normalize/mutate arguments')

    async def test_semantic_schema_and_backend_errors_share_five_attempt_budget(self):
        adapter=self.make();cancel=asyncio.Event()
        for n in range(4):
            result=await adapter.execute('plan_release',copy.deepcopy(ARGS),'bad-'+str(n),cancel,lambda _:None)
            self.assertEqual(result['retry_budget']['planning_attempts'],n+1)
        invalid=copy.deepcopy(ARGS);invalid['goal']['max_rotation_step_deg']=16
        with self.assertRaises(PlanningArgumentError) as caught:validate(call('plan_release',invalid)['output'][0],'execute')
        e=caught.exception
        result=await adapter.planning_argument_error(e.name,e.arguments,e.detail,cancel)
        self.assertEqual(result['retry_budget']['planning_attempts'],5)
        with self.assertRaisesRegex(Blocked,'规划5次'):
            await adapter.execute('plan_held_object_motion',{k:v for k,v in ARGS.items() if k!='retreat_vector_base_m'},'sixth',cancel,lambda _:None)
        self.assertFalse(any(v is not None for _,v in adapter.backend.requests))

    async def test_schema_error_is_returned_to_model_without_backend_submission(self):
        adapter=self.make();invalid=copy.deepcopy(ARGS);invalid['goal']['target_position_m']=['bad',0,0]
        fixed=copy.deepcopy(ARGS);fixed['goal']['reference_object_id']=None
        llm=FakeLLM(call('plan_release',invalid,'bad'),call('plan_release',fixed,'good'),end('answered',['bad','good']))
        with tempfile.TemporaryDirectory() as d:
            runner=Runner(adapter.config,Store(d),llm,lambda:adapter);ident=runner.submit('仅规划');await runner.worker;r=runner.store.get(ident)
            self.assertEqual(r['state'],'succeeded',r.get('summary'));self.assertEqual(r['completion_kind'],'answered')
            result=r['calls'][0]['result'];self.assertEqual(result['error']['validation_source'],'local_schema')
            self.assertEqual(result['error']['fields'][0]['field'],'goal.target_position_m.0')
            self.assertEqual(sum(p=='/api/agent/tasks' for p,_ in adapter.backend.requests),1)

    async def test_backend_invalid_arguments_remain_actionable_but_not_auth_or_unknown(self):
        for code,unknown in [('INVALID_ARGUMENTS',False),('UNAUTHORIZED',False),('INVALID_ARGUMENTS',True),('REQUEST_UNCERTAIN',False)]:
            with self.subTest(code=code,unknown=unknown):
                adapter=self.make();backend=adapter.backend;original=backend.request
                def request(path,value=None):
                    if value is not None:
                        backend.requests.append((path,copy.deepcopy(value)))
                        raise BackendError({'code':code,'message':'goal字段需要修正','outcome_unknown':unknown})
                    if path.startswith('/api/agent/tasks/'):raise BackendError({'code':'TASK_NOT_FOUND'})
                    return original(path,value)
                backend.request=request;fixed=copy.deepcopy(ARGS);fixed['goal']['reference_object_id']=None
                if code=='INVALID_ARGUMENTS' and not unknown:
                    result=await adapter.execute('plan_release',fixed,'rejected',asyncio.Event(),lambda _:None)
                    self.assertTrue(result['argument_correction_required']);self.assertEqual(result['error']['message'],'goal字段需要修正')
                    self.assertEqual(result['retry_budget']['planning_attempts'],1);self.assertIsNone(adapter.active_request)
                else:
                    with self.assertRaises((Blocked,BackendError)):await adapter.execute('plan_release',fixed,'rejected',asyncio.Event(),lambda _:None)
                self.assertEqual(sum(v is not None for _,v in backend.requests),1,'must never automatically resend')

    async def test_motion_argument_error_is_fatal_and_stop_prevents_correction(self):
        adapter=self.make();original=adapter.backend.request
        def reject(path,value=None):
            if value is not None:raise BackendError({'code':'INVALID_ARGUMENTS','message':'actual motion rejected'})
            return original(path,value)
        adapter.backend.request=reject
        with self.assertRaises(BackendError):await adapter.execute('execute_plan',{'plan_id':'a'*32},'motion',asyncio.Event(),lambda _:None)
        cancel=asyncio.Event();cancel.set();count=len(adapter.backend.requests)
        with self.assertRaises(Cancelled):await adapter.planning_argument_error('plan_release',ARGS,{'message':'error'},cancel)
        self.assertEqual(len(adapter.backend.requests),count)

    def test_frame_rules_remain_strict_with_specific_hints(self):
        with self.assertRaises(GoalArgumentError) as e:checked_goal(GOAL,True)
        self.assertEqual(e.exception.field,'goal.reference_object_id');self.assertIsNone(e.exception.expected)
        self.assertEqual(checked_goal({**GOAL,'reference_object_id':None},True)['reference_frame'],'scene_object')
        with self.assertRaises(GoalArgumentError):checked_goal({**DEFAULT_GOAL,'reference_frame':'tracked_object','target_position_m':[0,0,.3]})
        with self.assertRaises(GoalArgumentError):checked_goal({**DEFAULT_GOAL,'reference_instance_id':'stray','target_position_m':[0,0,.3]})
