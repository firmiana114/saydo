'use strict';
(() => {
  const $=id=>document.getElementById(id),status=$('connection'),feedback=$('feedback');
  const term=new Terminal({cursorBlink:true,fontSize:14,fontFamily:'ui-monospace, SFMono-Regular, Menlo, Consolas, monospace',scrollback:4000,disableStdin:true,theme:{background:'#101722',foreground:'#e2e8f0',cursor:'#a8c3ff'}});
  const fit=new FitAddon.FitAddon();term.loadAddon(fit);term.open($('terminal'));
  let token='',session=null,offset=0,replayOffset=0,replaying=false,generation=0,connected=false,running=false,sending=false,queue=[],timer=null,resizeTimer=null;
  const uuid=()=>Array.from(crypto.getRandomValues(new Uint8Array(16)),v=>v.toString(16).padStart(2,'0')).join('');
  const encode=bytes=>btoa(String.fromCharCode(...bytes)),decode=s=>Uint8Array.from(atob(s),c=>c.charCodeAt(0));
  function controls(){term.options.disableStdin=!connected||!running||offset<replayOffset;$('interrupt').disabled=!connected||!running;$('close').disabled=!connected||!session;}
  function disconnected(message){connected=false;queue=[];generation++;clearTimeout(timer);status.textContent='连接中断';feedback.textContent=message;controls();$('connect').disabled=false;}
  async function request(path,data,retry=false){
    // A retry reuses the request_id; the server deduplicates terminal input.
    for(let attempt=0;;attempt++){
      try{
        const r=await fetch('/api/codex-terminal/'+path,{method:data?'POST':'GET',headers:{Authorization:'Bearer '+token,...(data?{'Content-Type':'application/json'}:{})},body:data?JSON.stringify(data):undefined,signal:AbortSignal.timeout(10000)});
        const value=await r.json();if(!r.ok){const error=Error(value.error||'终端请求失败');error.http=true;throw error;}return value;
      }catch(error){if(!retry||attempt||error.http)throw error;}
    }
  }
  async function flush(){
    if(sending||!connected)return;sending=true;const active=generation;
    try{
      while(queue.length&&connected&&active===generation){const data=queue.shift();await request('input',{session_id:session,request_id:uuid(),data},true);}
    }catch(error){if(active===generation)disconnected(error.message+' 输入未确认，请重连后检查终端，不要直接重发任务。');}
    finally{sending=false;if(queue.length&&connected)flush();}
  }
  function input(bytes){
    if(!connected||!running)return;
    if(queue.length+Math.ceil(bytes.length/8192)>256){disconnected('待发送输入过多，请重新连接检查终端。');return;}
    for(let i=0;i<bytes.length;i+=8192)queue.push(encode(bytes.subarray(i,i+8192)));
    flush();
  }
  // Historical terminal queries must not inject stale device-status replies.
  term.onData(text=>{if(!replaying)input(new TextEncoder().encode(text));});
  term.onBinary(text=>{if(!replaying)input(Uint8Array.from(text,c=>c.charCodeAt(0)&255));});
  async function resize(){
    if($('terminal').clientWidth<100||$('terminal').clientHeight<80)return;
    fit.fit();
    term.resize(Math.max(20,Math.min(240,term.cols)),Math.max(8,Math.min(120,term.rows)));
    if(connected&&running){try{await request('resize',{session_id:session,cols:Math.max(20,Math.min(240,term.cols)),rows:Math.max(8,Math.min(120,term.rows))});}catch(error){feedback.textContent=error.message;}}
  }
  new ResizeObserver(()=>{clearTimeout(resizeTimer);resizeTimer=setTimeout(resize,100);}).observe($('terminal'));
  async function poll(active){
    if(!connected||active!==generation)return;
    try{
      const value=await request('output?session_id='+session+'&offset='+offset);
      if(active!==generation)return;
      if(value.reset){term.reset();feedback.textContent='较早的终端输出已截断，正在恢复当前画面。';}
      replaying=offset<replayOffset;
      try{if(value.data)await new Promise(resolve=>term.write(decode(value.data),resolve));}
      finally{replaying=false;}
      if(active!==generation)return;
      offset=value.offset;running=value.running;controls();
      if(value.reset)await resize();
      if(!running&&offset>=value.end_offset){connected=false;status.textContent='会话已结束';feedback.textContent='Codex已退出（退出码 '+value.exit_code+'）。可启动新会话。';controls();return;}
      timer=setTimeout(()=>poll(active),200);
    }catch(error){if(active===generation)disconnected(error.message);}
  }
  $('connect-form').onsubmit=async event=>{
    event.preventDefault();if($('connect').disabled)return;
    const value=$('token').value.trim()||token;if(!value)return;
    generation++;clearTimeout(timer);queue=[];connected=false;controls();token=value;$('token').value='';$('token').required=false;$('token').placeholder='访问码已保留在本页内存';$('connect').disabled=true;status.textContent='连接中…';
    try{
      fit.fit();const result=await request('connect',{request_id:uuid(),cols:Math.max(20,Math.min(240,term.cols)),rows:Math.max(8,Math.min(120,term.rows))},true);
      session=result.session_id;offset=0;replayOffset=result.replay_offset;running=result.running;connected=true;term.reset();status.textContent='已连接 MSI';feedback.textContent='可直接输入任务，Enter提交。连接的是同一个服务器终端会话。';controls();term.focus();await resize();poll(generation);
    }catch(error){disconnected(error.message);}
    finally{$('connect').disabled=false;}
  };
  $('interrupt').onclick=()=>input(new Uint8Array([3]));
  $('clear').onclick=()=>{term.clear();term.focus();};
  $('close').onclick=async()=>{
    if(!connected||!session)return;
    $('close').disabled=true;
    try{await request('close',{session_id:session});generation++;clearTimeout(timer);queue=[];connected=false;running=false;status.textContent='会话已结束';feedback.textContent='终端会话已结束。机械臂停止需使用右侧“停止运动”。';controls();}
    catch(error){disconnected(error.message);}
  };
  // No unload handler: hiding/reloading the page must not kill an in-flight task.
  resize();
})();
