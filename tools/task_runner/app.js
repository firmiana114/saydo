'use strict';
const el=id=>document.getElementById(id);
let selected=null, active=null, polling=false;
const states={running:'执行中',succeeded:'已结束',blocked:'需要处理',failed:'失败',cancelled:'已取消',interrupted:'执行中断'};
const toolNames={get_status:'读取系统状态',get_camera_frame:'查看相机画面',get_robot_state:'读取机械臂状态',list_objects:'读取物品模型',get_object_pose:'读取物品位姿',get_grasp_profiles:'读取夹法参数',select_object:'选择物品',start_tracking:'初始化跟踪',choose_tracking_target:'选择跟踪目标',preview_grasp:'预览抓取',grasp:'抓取物品',preview_place:'预览放置',place:'放置物品',reset_arm:'机械臂复位',stop_robot:'停止机械臂'};
const callStates={prepared:'已记录',dispatching:'执行中',completed:'完成',unavailable:'观测暂不可用',failed:'失败',blocked:'需要处理',cancelled:'已取消'};
async function request(path,data){const response=await fetch(path,{cache:'no-store',...(data===undefined?{}:{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify(data)})});const value=await response.json();if(!response.ok)throw new Error(value.error||'请求失败');return value;}
function show(run){el('state').textContent=states[run.state]||run.state;el('phase').textContent=run.phase;el('meta').textContent=`模型 ${run.model} · ${run.run_id}`;el('summary').textContent=run.summary||'';el('calls').replaceChildren();for(const call of run.calls||[]){const li=document.createElement('li');li.textContent=`${toolNames[call.name]||call.name} · ${callStates[call.state]||call.state}${call.seconds===undefined?'':` · ${call.seconds.toFixed(1)} 秒`}`;el('calls').append(li);}el('evidence').textContent=JSON.stringify(run.verified_actions||[],null,2);}
async function refresh(){if(polling)return;polling=true;try{const data=await request('/api/task-runs');active=data.runs.find(r=>r.state==='running')?.run_id||null;el('mode').textContent={read_only:'只读模式',preview:'预览模式',execute:'实际执行模式'}[data.mode];el('model').textContent=!data.fallback_model||data.model===data.fallback_model?`当前模型 ${data.model}`:`首选 ${data.model} · 不可用时 ${data.fallback_model}`;el('mode-help').textContent=data.mode==='execute'?'此模式允许模型调用抓取、放置和复位。': '任务执行器当前不会发起抓取、放置或复位；工作台的手动控制独立生效。停止按钮会发送软件停止。';el('submit').disabled=Boolean(active)||data.stopping||data.blocked_by_uncertainty;el('cancel').disabled=!active;if(data.blocked_by_uncertainty)el('error').textContent='存在未核对的执行结果，已禁止新任务。请人工核对现场和运行记录。';el('history').replaceChildren();for(const run of data.runs){const button=document.createElement('button');button.textContent=`${states[run.state]||run.state} · ${run.instruction}`;button.onclick=()=>{selected=run.run_id;refresh();};el('history').append(button);}selected=selected||active||data.runs[0]?.run_id;if(selected)show(await request('/api/task-runs/'+selected));}catch(error){el('error').textContent=error.message;}finally{polling=false;}}
el('submit').onclick=async()=>{el('submit').disabled=true;el('error').textContent='';try{const run=await request('/api/task-runs',{instruction:el('instruction').value});selected=run.run_id;}catch(error){el('error').textContent=error.message;}await refresh();};
el('cancel').onclick=async()=>{if(!active)return;el('cancel').disabled=true;el('phase').textContent='正在请求停止并核对停稳…';try{await request('/api/task-runs/'+active+'/cancel',{});}catch(error){el('error').textContent=error.message;}await refresh();};
refresh();setInterval(refresh,1000);

if(window.self!==window.top){
  document.body.classList.add('embedded');
  const notifyHeight=()=>parent.postMessage({type:'saydo-task-height',height:Math.ceil(document.querySelector('main').getBoundingClientRect().height)+24},location.origin);
  new ResizeObserver(notifyHeight).observe(document.querySelector('main'));
  notifyHeight();
}
