"""Bounded serial decision loop; cancellation bypasses the model."""
import asyncio
import hashlib
import json
from pathlib import Path
import time
from .llm_client import LLMClient, ModelUnavailable, LLMError, ConnectionNotSent, ResponseTimeout, ResponseFailure
from .robot_adapter import RobotAdapter, Blocked, Cancelled, compact, BackendError, model_result
from .tool_registry import schemas, validate, MOTIONS, READS
from .config import UI_MODELS, UI_REASONING_EFFORTS
from .planning_validation import PlanningArgumentError
from .observations import append_image,valid_global


CONNECTION_ATTEMPTS = 5
CONNECTION_RETRY_DELAYS = (1, 2, 4, 8)


class Runner:
    def __init__(self, config, store, llm=None, adapter_factory=None):
        self.config, self.store = config, store
        self.llm = llm or LLMClient(config)
        self.adapter_factory = adapter_factory or (lambda: RobotAdapter(config))
        self.active = None
        self.worker = None
        self.cancelled = asyncio.Event()
        self.stop_lock = asyncio.Lock()
        self.adapter = None
        self.stopping = False
        self.uncertain = store.has_uncertain_runs()
        self.prompt = Path(__file__).with_name('prompts').joinpath('system.txt').read_text()

    def submit(self, instruction, model=None, reasoning_effort=None):
        if self.active is not None or self.uncertain or self.stopping:
            raise Blocked('当前任务未结束或存在未核对的中断记录')
        if not isinstance(instruction, str) or not instruction.strip() or len(instruction) > 4000:
            raise ValueError('任务指令长度应为1–4000字符')
        if model is not None and model not in UI_MODELS:
            raise ValueError('不支持的模型选择')
        if reasoning_effort is not None and reasoning_effort not in UI_REASONING_EFFORTS:
            raise ValueError('不支持的推理强度')
        chosen_model = self.config.model if model is None else model
        effort = self.config.reasoning_effort if reasoning_effort is None else reasoning_effort
        run = self.store.create(instruction.strip(), chosen_model, self.config.mode, effort)
        run.update(requested_model=chosen_model, fallback_model=self.config.fallback_model if model is None else '')
        self.active = run
        self.emit(run, 'phase', text='任务已接收，正在读取初始状态。')
        self.cancelled = asyncio.Event()
        self.adapter = self.adapter_factory()
        self.worker = asyncio.create_task(self.run(run))
        return run['run_id']

    def emit(self, run, event, **values):
        events = run.setdefault('events', [])
        events.append({'event': event, 'seq': len(events)+1, 'at': time.time(), **values})
        self.store.save(run)

    def progress(self, phase, record_event=True):
        if self.active and self.active['phase'] != phase:
            self.active['phase'] = phase
            if record_event:
                self.emit(self.active, 'phase', text=phase)
            else:
                self.store.save(self.active)

    def finish(self, run, state, summary, **extra):
        phase = {'succeeded': '任务已结束', 'blocked': '需要处理', 'failed': '执行失败', 'cancelled': '已取消'}[state]
        run.update(state=state, phase=phase, summary=summary, **extra)
        self.store.save(run)

    async def cancel(self, run_id):
        if not self.active or self.active['run_id'] != run_id:
            raise KeyError('没有此活动任务')
        self.cancelled.set()
        async with self.stop_lock:
            run = self.active
            if run is None:
                raise KeyError('任务已结束')
            self.progress('停止确认中')
            try:
                result = await self.adapter.stop()
                run['stop_result'] = result
            except Exception:
                self.uncertain = True
                run['outcome_unknown'] = True
                run['stop_result'] = {'stationary_observed': False}
            self.store.save(run)
        return self.store.get(run_id)

    async def stop_current(self):
        if self.active:
            return await self.cancel(self.active['run_id'])
        if self.stopping:
            raise Blocked('已有停止请求正在确认')
        self.stopping = True
        run = self.store.create('操作员请求停止机械臂', self.config.model, self.config.mode, self.config.reasoning_effort)
        record = self.store.prepare(run['run_id'], 'operator-stop', 'stop_robot', {}, 0)
        before = time.monotonic()
        self.emit(run, 'tool_started', name='stop_robot', call_id='operator-stop')
        try:
            async with self.stop_lock:
                result = await self.adapter_factory().stop()
            record.update(state='completed', result=result)
            self.finish(run, 'cancelled', '软件停止已确认；未松爪', stop_result=result)
            return result
        except Exception:
            self.uncertain = True
            record.update(state='blocked', outcome_unknown=True)
            self.finish(run, 'blocked', '停止未确认，需现场核对', outcome_unknown=True)
            raise Blocked('停止未确认，需现场核对', uncertain=True)
        finally:
            record['seconds'] = round(time.monotonic()-before, 3)
            self.store.save_call(run['run_id'], record)
            self.emit(run, 'tool_finished', name='stop_robot', call_id='operator-stop', state=record['state'], seconds=record['seconds'])
            self.emit(run, 'final', text=run['summary'], state=run['state'])
            self.stopping = False

    def check(self):
        if self.cancelled.is_set():
            raise Cancelled()

    async def observe_after_action(self,run,history,action_id,step,after):
        ident='auto-global-'+hashlib.sha256(action_id.encode()).hexdigest()[:16]
        record=self.store.prepare(run['run_id'],ident,'observe_global',{},step)
        record.update(initiated_by='executor',after_action=action_id,state='dispatching')
        record['request_id']+='-global';self.store.save_call(run['run_id'],record)
        self.emit(run,'phase',text='执行器自动获取动作后的不固定的全局相机画面。')
        self.emit(run,'tool_started',name='observe_global',call_id=ident,initiated_by='executor',after_action=action_id)
        started=time.monotonic()
        try:
            self.check()
            data=await self.adapter.execute('observe_global',{},record['request_id'],self.cancelled,self.progress)
            self.check()
            if not valid_global(data,after):raise BackendError({'code':'CAMERA_UNAVAILABLE','message':'全局图像缺少有效机位信息或不是本次动作后的新帧'})
            clean={**compact(data),'evidence_call_id':ident,'after_action':action_id,'initiated_by':'executor'}
            append_image(history,{**data,'after_action':action_id,'initiated_by':'executor'},'observe_global',ident);record.update(state='completed',result=clean)
            run.setdefault('global_observations',[]).append(clean)
            return ident,clean
        except BackendError as error:
            clean={'error':error.payload,'evidence_call_id':ident,'after_action':action_id,'initiated_by':'executor','motion_sent':False}
            record.update(state='unavailable',result=clean)
            history.append({'role':'user','content':'执行器自动全局观察失败，未发送运动。继续动作或报告成功前须用observe_global获取新帧；不能重做刚才已成功的动作。'+json.dumps(clean,ensure_ascii=False)})
            return ident,clean
        except (Cancelled,asyncio.CancelledError):
            record.update(state='cancelled');raise
        finally:
            record['seconds']=round(time.monotonic()-started,3);self.store.save_call(run['run_id'],record)
            self.emit(run,'tool_finished',name='observe_global',call_id=ident,state=record['state'],seconds=record['seconds'],initiated_by='executor',after_action=action_id)

    async def model_response(self, history, run, turn):
        # The 600 s budget covers backend work, not bounded model waiting.
        # Each response attempt retains its own 90 s deadline and cancellation.
        timer=getattr(self,'run_deadline',None)
        deadline=timer.when() if timer is not None else None
        before=time.monotonic()
        if timer is not None:timer.reschedule(None)
        try:
            return await self._model_response(history,run,turn)
        finally:
            if timer is not None and not timer.expired():
                timer.reschedule(deadline+time.monotonic()-before)

    async def _model_response(self, history, run, turn):
        try:
            return await self.model_call(history, run, turn)
        except ModelUnavailable:
            fallback = run.get('fallback_model', '')
            if not fallback or turn != 0 or run['model'] != run['requested_model'] or run['model'] == fallback:
                raise
            self.emit(run, 'model_fallback', **{'from': run['model'], 'to': fallback})
            run['model'] = fallback
            return await self.model_call(history, run, turn)

    async def model_call(self, history, run, turn):
        for attempt in range(1,CONNECTION_ATTEMPTS+1):
            self.check()
            try:
                return await self.model_attempt(history,run,turn,attempt)
            except (ConnectionNotSent,ResponseFailure) as error:
                self.check()
                if attempt==CONNECTION_ATTEMPTS:
                    raise LLMError(f'{error}；模型请求已尝试{CONNECTION_ATTEMPTS}次，达到上限') from error
                delay=CONNECTION_RETRY_DELAYS[attempt-1]
                reason='模型连接失败，请求尚未上传；'
                if isinstance(error,ResponseFailure):
                    reason=('等待模型响应超时；' if isinstance(error,ResponseTimeout) else '模型响应传输失败；')+'未执行本次响应中的任何工具；'
                self.emit(run,'llm_response_retry' if isinstance(error,ResponseFailure) else 'llm_connection_retry',turn=turn+1,failed_attempt=attempt,
                    next_attempt=attempt+1,max_attempts=CONNECTION_ATTEMPTS,delay_s=delay,
                    text=reason+f'{delay}秒后进行第{attempt+1}/{CONNECTION_ATTEMPTS}次尝试。')
                # Cancellable backoff; this never dispatches robot tools.
                try:await asyncio.wait_for(self.cancelled.wait(),timeout=delay)
                except TimeoutError:pass
                self.check()

    async def model_attempt(self, history, run, turn, attempt):
        self.check()
        tools = schemas(self.config.mode)
        if run.get('finalizing'):
            tools = [tool for tool in tools if tool['name'] == 'finish_task']
        model = run['model']
        effort = run['reasoning_effort']
        invocation = 1 + sum(e['event'] == 'llm_started' for e in run['events'])
        self.emit(run, 'llm_started', invocation=invocation, turn=turn+1, model=model, reasoning_effort=effort,
            connection_attempt=attempt,max_connection_attempts=CONNECTION_ATTEMPTS)
        diagnostic_event=run['events'][-1]
        def diagnostics_callback(value):
            diagnostic_event['diagnostics']=value
            self.store.save(run)
        before = time.monotonic()
        outcome = 'failed'
        pending = asyncio.create_task(self.llm.generate(history, tools, model, self.prompt+'\n当前模式：'+self.config.mode,
            reasoning_effort=effort, diagnostics_callback=diagnostics_callback))
        cancel = asyncio.create_task(self.cancelled.wait())
        try:
            done, _ = await asyncio.wait([pending, cancel], return_when=asyncio.FIRST_COMPLETED)
            if cancel in done:
                pending.cancel()
                await asyncio.gather(pending, return_exceptions=True)
                raise Cancelled()
            response = await pending
            outcome = 'completed'
            return response
        except ModelUnavailable:
            outcome = 'unavailable'
            raise
        except (Cancelled, asyncio.CancelledError):
            outcome = 'cancelled'
            raise
        finally:
            if not pending.done():
                pending.cancel()
                await asyncio.gather(pending, return_exceptions=True)
            cancel.cancel()
            await asyncio.gather(cancel, return_exceptions=True)
            self.emit(run, 'llm', invocation=invocation, turn=turn+1, model=model,
                reasoning_effort=effort,
                seconds=round(time.monotonic()-before, 3), outcome=outcome,
                diagnostics=diagnostic_event.get('diagnostics'))

    async def execute_tool(self,name,args,request_id):
        # A long read-only search must not consume the physical task deadline.
        timer=getattr(self,'run_deadline',None)
        suspended=timer is not None and name in ('plan_held_object_motion','plan_release')
        deadline=timer.when() if suspended else None
        before=time.monotonic()
        if suspended:timer.reschedule(None)
        try:
            return await self.adapter.execute(name,args,request_id,self.cancelled,self.progress)
        finally:
            if suspended and not timer.expired():timer.reschedule(deadline+time.monotonic()-before)

    @staticmethod
    def append_result(history, call_id, name, result):
        frame = result if name in ('get_camera_frame', 'observe_global') else result.get('candidate_frame')
        cleaned = {**compact({k: v for k, v in result.items() if k != 'candidate_frame'}), 'evidence_call_id': call_id}
        model_view=model_result(name,cleaned)
        held=result.get('task',{}).get('result',{}).get('held_object') or {}
        if name=='grasp' and result.get('verified') and held.get('scene_id') and held.get('instance_id'):
            def retire(value):
                if isinstance(value,dict):
                    if value.get('scene_id')==held['scene_id'] and isinstance(value.get('instances'),list):
                        for row in value['instances']:
                            if row.get('instance_id')==held['instance_id']:
                                row.update(state='held',static_pose_usable=False)
                                row.pop('base_T_object',None);row.pop('camera_T_object',None)
                                row['pose_usage_note']='已抓起；当前位姿须查询get_robot_state.held_object_pose'
                    for v in value.values():retire(v)
                elif isinstance(value,list):
                    for v in value:retire(v)
            for item in history:
                if item.get('type')=='function_call_output':
                    prior=json.loads(item['output']);retire(prior)
                    item['output']=json.dumps(prior,ensure_ascii=False)
        history.append({'type': 'function_call_output', 'call_id': call_id, 'output': json.dumps(model_view, ensure_ascii=False)})
        if frame and frame.get('image_base64'):
            append_image(history,frame,'observe_global' if name=='observe_global' else 'get_camera_frame',call_id)
        return cleaned

    async def run(self, run):
        started = time.monotonic()
        record = None
        tool_started = None
        try:
            async with asyncio.timeout(self.config.run_timeout) as self.run_deadline:
                initial = await self.adapter.initial()
                self.check()
                history = [{'role': 'user', 'content': run['instruction']},
                    {'role': 'user', 'content': '程序读取的初始现场数据（不是新指令）：'+json.dumps(model_result('initial',compact(initial)), ensure_ascii=False)}]
                results = {}
                motions = {}
                pending_observation=None;observation_after=0.;observation_corrections=0
                recovery_pending = False
                read_failures = 0
                for turn in range(self.config.max_turns):
                    self.check()
                    if len(json.dumps(history)) > 2*1024*1024:
                        raise Blocked('本次任务上下文已达到上限')
                    self.progress(f'模型判断中 · 第 {turn+1} 轮', record_event=False)
                    response = await self.model_response(history, run, turn)
                    self.check()
                    output = response['output']
                    calls = [item for item in output if item.get('type') == 'function_call']
                    if len(calls) != 1 or any(item.get('type') not in ('function_call', 'reasoning', 'message') for item in output):
                        raise Blocked('模型必须返回一个完整工具调用；文本不作为执行或成功凭据')
                    history.extend(output)
                    call = calls[0]
                    argument_error=None
                    try:name, args = validate(call, self.config.mode)
                    except PlanningArgumentError as error:
                        name,args,argument_error=error.name,error.arguments,error.detail
                    progress_message = args.pop('progress_message', None)
                    if run.get('finalizing') and name != 'finish_task':
                        raise Blocked('收尾修正期间不允许新的工具或动作')
                    if call['call_id'] in results:
                        raise Blocked('模型重复使用工具调用编号')
                    messages = [part.get('text', '') for item in output if item.get('type') == 'message'
                        for part in item.get('content', []) if part.get('type') == 'output_text'] if name != 'finish_task' else []
                    if isinstance(progress_message,str) and progress_message and not any(message.strip() for message in messages):
                        messages.append(progress_message)
                    for message in messages:
                        if message.strip():
                            self.emit(run, 'model_message', text=message, call_id=call['call_id'])
                    if name == 'finish_task':
                        if pending_observation is not None and args['outcome']!='blocked':
                            observation_corrections+=1
                            if observation_corrections>3:raise Blocked('动作后全局观察缺失，不能报告完成')
                            self.append_result(history,call['call_id'],name,{'error':'缺少最后动作之后的新全局观察，先调用observe_global；不可重做已成功动作。','required_tool':'observe_global'})
                            continue
                        if recovery_pending and args['outcome']=='succeeded':raise Blocked('调姿失败尚未成功恢复，不能报告任务成功')
                        evidence = set(args['evidence_call_ids'])
                        invalid_evidence = not evidence <= results.keys() or (args['outcome'] == 'succeeded' and not set(motions) <= evidence)
                        if invalid_evidence and results and not run.get('finalizing'):
                            run['finalizing'] = True
                            self.emit(run, 'finish_evidence_correction', text='正在核对并修正总结引用的执行证据。')
                            self.append_result(history, call['call_id'], name, {'error': '引用了错误的证据编号。只修正 finish_task，不得再执行动作。',
                                'available_evidence_call_ids': list(results), 'required_motion_evidence_call_ids': list(motions)})
                            continue
                        if invalid_evidence:
                            raise Blocked('模型引用了不存在或不完整的执行证据')
                        if args['outcome'] == 'succeeded' and (not motions or not set(motions) <= evidence):
                            raise Blocked('缺少本次真实动作成功证据；不能确认实物任务完成')
                        if motions:
                            latest = await self.adapter.status()
                            self.adapter.check_idle(latest)
                            last = list(motions.values())[-1]
                            held = latest['grasp'].get('held_object')
                            if last['name'] == 'place' and held or last['name'] == 'grasp' and (not held or held.get('object_id') != last['arguments']['object_id'] or held.get('hold_id') != last.get('hold_id')):
                                raise Blocked('结束前夹持状态已变化，不能确认任务完成', uncertain=True)
                            if last['name'] == 'reorient' and (not held or held.get('object_id') != last['arguments']['object_id']
                                    or held.get('hold_id') != last.get('hold_id') or held.get('orientation_version') != last.get('orientation_version')):
                                raise Blocked('结束前姿态版本或夹持关系已变化', uncertain=True)
                            if last['name']=='execute_plan':
                                if last.get('plan_kind')=='release' and held is not None:
                                    raise Blocked('结束前释放状态已变化',uncertain=True)
                                if last.get('plan_kind')=='held_object_motion' and (not held or held.get('hold_id')!=last.get('hold_id') or held.get('orientation_version')!=last.get('orientation_version')):
                                    raise Blocked('结束前持物运动版本已变化',uncertain=True)

                        self.check()
                        state = 'blocked' if args['outcome'] == 'blocked' else 'succeeded'
                        self.finish(run, state, args['summary'], completion_kind=args['outcome'],
                            verified_actions=list(motions.values()), evidence_call_ids=list(evidence),
                            visual_verification='post_action_global_images_available_not_proof_of_contact_or_settling' if motions and pending_observation is None else 'not_verified')
                        return
                    self.check()
                    if pending_observation is not None and name in MOTIONS:
                        raise Blocked('动作后全局观察尚未恢复，拒绝继续动作；已完成动作不可重放')
                    if name in MOTIONS and any(m['name'] == name and m['arguments'] == args for m in motions.values()):
                        raise Blocked('本次任务已成功执行相同动作，拒绝重复执行')
                    record = self.store.prepare(run['run_id'], call['call_id'], name, args, turn+1)
                    self.progress('执行工具：'+name, record_event=False)
                    tool_started = time.monotonic()
                    record['state'] = 'dispatching'
                    self.store.save_call(run['run_id'], record)
                    self.emit(run, 'tool_started', name=name, call_id=call['call_id'])
                    if argument_error is not None:
                        result=await self.adapter.planning_argument_error(name,args,argument_error,self.cancelled)
                    elif name == 'stop_robot':
                        result = await self.adapter.stop()
                    else:
                        try:
                            result = await self.execute_tool(name, args, record['request_id'])
                        except BackendError as error:
                            if name not in (set(READS)|{'get_task_context'}) or error.payload.get('code') not in ('POSE_NOT_READY', 'CAMERA_UNAVAILABLE'):
                                detail=str(error.payload.get('message') or '')[:1200]
                                raise Blocked('后端工具拒绝请求：'+str(error.payload.get('code', 'BACKEND_ERROR'))+('：'+detail if detail else '')) from error
                            read_failures += 1
                            if read_failures > 3:
                                raise Blocked('观察连续不可用，已达到有限恢复次数') from error
                            result = {'error': error.payload, 'motion_sent': False}
                    self.check()
                    if hasattr(self.adapter,'scene_view'):result=self.adapter.scene_view(result)
                    cleaned = self.append_result(history, call['call_id'], name, result)
                    results[call['call_id']] = cleaned
                    if name=='observe_global' and pending_observation is not None:
                        if valid_global(result,observation_after):
                            run.setdefault('global_observations',[]).append({**cleaned,'after_action':pending_observation,'initiated_by':'model'})
                            pending_observation=None
                    if result.get('precondition_correction_required'):
                        self.emit(run,'scene_precondition_correction',text=result['error']['message'])
                    if result.get('argument_correction_required'):
                        self.emit(run,'planning_argument_error',text=result['error']['message'],retry_budget=result.get('retry_budget'),field=result['error'].get('field'))
                    if result.get('recoverable_failure'):
                        recovery_pending=True
                        self.emit(run,'motion_recovery',text='调姿失败后已确认停稳和夹持，将诊断返回模型重新规划。',retry_budget=result.get('retry_budget'))
                    if name in MOTIONS and not result.get('recoverable_failure') and not result.get('precondition_correction_required'):
                        if not result.get('verified'):
                            raise Blocked('动作缺少后端核验')
                        if name in ('reorient','execute_plan'):recovery_pending=False
                        held = result.get('task', {}).get('result', {}).get('held_object')
                        motions[call['call_id']] = {'name': name, 'task_id': record['request_id'], 'arguments': args,
                            'hold_id': held.get('hold_id') if held else None,
                            'orientation_version': held.get('orientation_version') if held else None,
                            'plan_kind':result.get('task',{}).get('result',{}).get('execution_result',{}).get('kind') if name=='execute_plan' else None}
                        pending_observation=call['call_id'];observation_after=time.time()
                    record.update(state='recoverable_failure' if result.get('recoverable_failure') else 'unavailable' if 'error' in result else 'completed', result=cleaned, seconds=round(time.monotonic()-tool_started, 3))
                    self.store.save_call(run['run_id'], record)
                    self.emit(run, 'tool_finished', name=name, call_id=call['call_id'], state=record['state'], seconds=record['seconds'])
                    record = None
                    if name in MOTIONS and pending_observation==call['call_id']:
                        ident,observation=await self.observe_after_action(run,history,call['call_id'],turn+1,observation_after)
                        results[ident]=observation
                        if 'error' not in observation:pending_observation=None
                raise Blocked('达到最大模型调用次数')
        except Cancelled:
            async with self.stop_lock:
                pass
            state = 'blocked' if self.uncertain else 'cancelled'
            self.finish(run, state, '停止未确认，需现场核对' if self.uncertain else '已取消；后续模型动作已丢弃')
        except Exception as error:
            uncertain = (isinstance(error, TimeoutError) and self.adapter.motion_inflight) or (isinstance(error, Blocked) and error.uncertain)
            if self.adapter.motion_inflight:
                try:
                    async with self.stop_lock:
                        run['stop_result'] = await self.adapter.stop()
                except Exception:
                    uncertain = True
            self.uncertain |= uncertain
            # Never log raw exception text from HTTP libraries or providers.
            message = str(error) if isinstance(error, (Blocked, LLMError)) else '执行器异常：'+type(error).__name__
            self.finish(run, 'blocked' if isinstance(error, Blocked) or uncertain else 'failed', message, outcome_unknown=uncertain)
        finally:
            if record:
                record.update(state=run['state'], outcome_unknown=run.get('outcome_unknown', False),
                    seconds=round(time.monotonic()-tool_started, 3) if tool_started is not None else 0)
                self.store.save_call(run['run_id'], record)
                self.emit(run, 'tool_finished', name=record['name'], call_id=record['call_id'], state=record['state'], seconds=record['seconds'])
            run['elapsed_s'] = round(time.monotonic()-started, 3)
            self.emit(run, 'final', text=run.get('summary', run['phase']), state=run['state'])
            self.active = None
