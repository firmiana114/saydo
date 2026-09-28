/* Render only user-facing commentary and execution events, never hidden reasoning. */
(function(root){
'use strict';
const tools={capture_scene_snapshot:'采集全部场景目标',get_scene_snapshot:'读取场景快照',get_object_geometry:'读取物品几何特征',plan_held_object_motion:'规划持物运动',plan_release:'规划释放',execute_plan:'执行运动方案',get_status:'读取系统状态',get_camera_frame:'查看相机画面',get_robot_state:'读取机械臂状态',list_objects:'读取物品模型',get_object_pose:'读取物品位姿',get_grasp_profiles:'读取夹法参数',select_object:'选择物品',start_tracking:'初始化跟踪',choose_tracking_target:'选择跟踪目标',preview_grasp:'预览抓取',grasp:'抓取物品',preview_place:'预览放置',place:'放置物品',preview_reorient:'预览物品姿态',reorient:'调整物品姿态',reset_arm:'机械臂复位',stop_robot:'停止机械臂'};
const effortLabels={low:'低',medium:'中',high:'高'};
const outcomes={completed:'完成',failed:'失败',cancelled:'已取消',unavailable:'暂不可用',blocked:'需要处理',interrupted:'已中断'};
const seconds=value=>Number.isFinite(value)?value.toFixed(1)+' 秒':'耗时未记录';
const requestPhases={preparing:'准备请求',connecting:'连接服务',tls:'建立加密连接',uploading:'上传请求',waiting_headers:'等待响应头',reading_body:'读取响应正文',parsing:'校验模型响应',completed:'已完成'};
function requestDetails(d){
 if(!d)return '';
 const parts=[requestPhases[d.phase]||'请求处理中'];
 if(Number.isFinite(d.request_bytes))parts.push(`请求 ${(d.request_bytes/1024).toFixed(1)} KB`);
 if(Number.isFinite(d.request_sent_s))parts.push(`上传完成 ${seconds(d.request_sent_s)}`);
 if(Number.isFinite(d.first_byte_s))parts.push(`首字节 ${seconds(d.first_byte_s)}`);
 if(Number.isFinite(d.http_status))parts.push(`HTTP ${d.http_status}`);
 if(d.usage?.input_tokens!==undefined)parts.push(`输入 ${d.usage.input_tokens} tokens`);
 if(d.usage?.output_tokens!==undefined)parts.push(`输出 ${d.usage.output_tokens} tokens`);
 if(d.exception_type)parts.push(d.exception_type);
 return '\n'+parts.join(' · ');
}
function entries(run,now=Date.now()/1000){
 if(run.timeline_version!==1)return legacy(run);
 const events=run.events||[],ends=new Map(events.filter(e=>e.event==='llm').map(e=>[e.invocation,e]));
 const rows=[];
 for(const event of events){
  let row={id:event.seq,at:event.at,kind:event.event,title:'',text:''};
  if(event.event==='llm_started'){
   const end=ends.get(event.invocation);
   const pending=!end&&run.state==='running';
   row.title=`LLM 调用 #${event.invocation} · 第 ${event.turn} 轮 · ${event.model}${event.reasoning_effort?` · 推理${effortLabels[event.reasoning_effort]||event.reasoning_effort}`:''}`;
   row.text=end?`${outcomes[end.outcome]||end.outcome} · 调用耗时 ${seconds(end.seconds)}`:pending?`等待响应 · 已用 ${seconds(Math.max(0,now-event.at))}`:'已中断 · 耗时未完整记录';
   if(event.connection_attempt>1)row.title+=` · 连接尝试 ${event.connection_attempt}/${event.max_connection_attempts}`;
   row.text+=requestDetails(end?.diagnostics||event.diagnostics);
   row.kind=pending?'llm-pending':'llm';
  }else if(event.event==='llm'){
   continue;
  }else if(event.event==='llm_connection_retry'){
   row.title='连接重试';row.text=event.text;
  }else if(event.event==='model_message'){
   row.title='模型说明';row.text=event.text;
  }else if(event.event==='tool_started'){
   row.title='开始执行';row.text=tools[event.name]||event.name;
  }else if(event.event==='tool_finished'){
   row.title=tools[event.name]||event.name;row.text=`${outcomes[event.state]||event.state} · ${seconds(event.seconds)}`;
  }else if(event.event==='final'){
   row.title=event.state==='succeeded'?'完整总结':event.state==='cancelled'?'取消结果':event.state==='interrupted'?'中断说明':'任务结束说明';row.text=event.text;
  }else if(event.event==='model_fallback'){
   row.title='模型切换';row.text=`${event.from} 不可用，改用 ${event.to}。`;
  }else if(event.event==='phase'||event.event==='finish_evidence_correction'){
   row.title='执行状态';row.text=event.text;
  }else continue;
  rows.push(row);
 }
 return rows;
}
function legacy(run){
 const rows=[{id:'legacy-note',kind:'legacy',title:'历史记录',text:'此任务由旧版执行器完成，未记录过程说明和完整时间线。以下保留已有调用耗时与结果。'}];
 const calls=run.calls||[],used=new Set();
 const addCall=call=>{used.add(call.call_id);rows.push({id:'call-'+call.call_id,kind:'tool_finished',at:call.created_at,title:tools[call.name]||call.name,text:`${outcomes[call.state]||call.state} · ${seconds(call.seconds)}`});};
 for(const event of run.events||[]){
  if(event.event!=='llm')continue;
  rows.push({id:'old-llm-'+event.turn,kind:'llm',title:`LLM 调用 · 第 ${event.turn} 轮`,text:`调用耗时 ${seconds(event.seconds)} · ${event.model||run.model}`});
  for(const call of calls)if(Number(call.request_id?.match(/-step-(\d+)$/)?.[1])===event.turn)addCall(call);
 }
 calls.filter(c=>!used.has(c.call_id)).forEach(addCall);
 if(run.summary)rows.push({id:'old-final',kind:'final',title:'完整总结',text:run.summary});
 return rows;
}
function render(container,run,now){
 const rows=entries(run,now);
 const signature=JSON.stringify([run.run_id,rows]);
 if(container.dataset.signature===signature)return;
 const scroller=container.parentElement?.id==='progress-scroll'?container.parentElement:null;
 const follow=scroller&&scroller.clientHeight>0&&(container._timelineRun!==run.run_id||scroller.scrollHeight-scroller.scrollTop-scroller.clientHeight<=32);
 container.dataset.signature=signature;
 if(container._timelineRun!==run.run_id){
  container.replaceChildren();container._timelineRun=run.run_id;container._timelineNodes=new Map();
 }
 for(const row of rows){
  let entry=container._timelineNodes.get(row.id);
  if(!entry){
   const node=document.createElement('li'),head=document.createElement('div');head.className='timeline-head';
   const title=document.createElement('strong'),stamp=document.createElement('time'),body=document.createElement('div');body.className='timeline-text';
   head.append(title,stamp);node.append(head,body);container.append(node);
   entry={node,title,stamp,body};container._timelineNodes.set(row.id,entry);
  }
  entry.node.className='timeline-item '+row.kind;
  if(entry.title.textContent!==row.title)entry.title.textContent=row.title;
  if(entry.body.textContent!==row.text)entry.body.textContent=row.text;
  entry.stamp.hidden=!Number.isFinite(row.at);
  if(Number.isFinite(row.at)){
   const date=new Date(row.at*1000);entry.stamp.dateTime=date.toISOString();
   const text=date.toLocaleTimeString('zh-CN',{hour12:false});
   if(entry.stamp.textContent!==text)entry.stamp.textContent=text;
  }
 }
 if(follow)scroller.scrollTop=scroller.scrollHeight;
}

root.SaydoTimeline={entries,render};
})(typeof window!=='undefined'?window:globalThis);
