'use strict';
const el=id=>document.getElementById(id);
let selected=null, active=null, polling=false;
const states={running:'执行中',succeeded:'已结束',blocked:'需要处理',failed:'失败',cancelled:'已取消',interrupted:'执行中断'};
let preferenceStorage;try{preferenceStorage=window.localStorage;}catch(_){}
const picker=new window.SaydoModelPicker.ModelPicker(el('model-select'),el('effort-select'),preferenceStorage,choice=>{
 el('model').textContent=`新任务：${choice.model} · 推理强度 ${window.SaydoModelPicker.labels[choice.reasoning_effort]||choice.reasoning_effort}`;
});
async function request(path,data){const response=await fetch(path,{cache:'no-store',...(data===undefined?{}:{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify(data)})});const value=await response.json();if(!response.ok)throw new Error(value.error||'请求失败');return value;}
function show(run){
 el('state').textContent=states[run.state]||run.state;
 el('phase').textContent=run.phase;
 el('meta').textContent=`模型 ${run.model} · 推理强度 ${window.SaydoModelPicker.labels[run.reasoning_effort]||run.reasoning_effort||'未记录'} · ${run.run_id}${run.elapsed_s===undefined?'':` · 总用时 ${run.elapsed_s.toFixed(1)} 秒`}`;
 window.SaydoTimeline.render(el('timeline'),run);
 el('evidence').textContent=JSON.stringify(run.verified_actions||[],null,2);
}
function renderHistory(runs){
 const history=el('history'),signature=JSON.stringify(runs.map(run=>[run.run_id,run.state,run.instruction]));
 if(history.dataset.signature===signature)return;
 const top=history.scrollTop;history.dataset.signature=signature;history.replaceChildren();
 for(const run of runs){
  const button=document.createElement('button');button.textContent=`${states[run.state]||run.state} · ${run.instruction}`;
  button.onclick=()=>{selected=run.run_id;refresh();};history.append(button);
 }
 history.scrollTop=top;
}
async function refresh(){if(polling)return;polling=true;try{const data=await request('/api/task-runs');active=data.runs.find(r=>r.state==='running')?.run_id||null;el('mode').textContent={read_only:'只读模式',preview:'预览模式',execute:'实际执行模式'}[data.mode];picker.configure(data);picker.setBusy(Boolean(active)||data.stopping);el('mode-help').textContent=data.mode==='execute'?'此模式允许模型调用抓取、姿态调整、放置和复位。': '任务执行器当前不会发起抓取、姿态调整、放置或复位；工作台的手动控制独立生效。停止按钮会发送软件停止。';el('submit').disabled=!picker.ready||Boolean(active)||data.stopping||data.blocked_by_uncertainty;el('cancel').disabled=!active;if(data.blocked_by_uncertainty)el('error').textContent='存在未核对的执行结果，已禁止新任务。请人工核对现场和运行记录。';renderHistory(data.runs);selected=selected||active||data.runs[0]?.run_id;if(selected)show(await request('/api/task-runs/'+selected));}catch(error){el('error').textContent=error.message;}finally{polling=false;}}
el('submit').onclick=async()=>{el('submit').disabled=true;el('error').textContent='';try{const run=await request('/api/task-runs',{instruction:el('instruction').value,...picker.selection()});selected=run.run_id;}catch(error){el('error').textContent=error.message;}await refresh();};
el('cancel').onclick=async()=>{if(!active)return;el('cancel').disabled=true;el('phase').textContent='正在请求停止并核对停稳…';try{await request('/api/task-runs/'+active+'/cancel',{});}catch(error){el('error').textContent=error.message;}await refresh();};
refresh();setInterval(refresh,1000);

if(window.self!==window.top){
  document.body.classList.add('embedded');
  const notifyHeight=()=>parent.postMessage({type:'saydo-task-height',height:Math.ceil(document.querySelector('main').getBoundingClientRect().height)+24},location.origin);
  new ResizeObserver(notifyHeight).observe(document.querySelector('main'));
  notifyHeight();
}
