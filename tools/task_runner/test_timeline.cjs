const fs=require('fs'),vm=require('vm'),assert=require('assert');
class Element{constructor(tag){this.tagName=tag;this.children=[];this.dataset={};this.textContent='';}append(...nodes){this.children.push(...nodes);}replaceChildren(...nodes){this.children=nodes;}}
const document={createElement:tag=>new Element(tag)};const window={};
vm.runInNewContext(fs.readFileSync(__dirname+'/timeline.js','utf8'),{window,document,Date,Map,Set,Number,Math});
const {entries,render}=window.SaydoTimeline;
const events=[
 {seq:1,at:100,event:'phase',text:'读取初始状态'},
 {seq:2,at:101,event:'llm_started',invocation:1,turn:1,model:'gpt-5.6-terra'},
 {seq:3,at:104,event:'llm',invocation:1,turn:1,model:'gpt-5.6-terra',outcome:'completed',seconds:3.125},
 {seq:4,at:104,event:'model_message',text:'我先查看画面，确认目标。'},
 {seq:5,at:104,event:'tool_started',name:'get_camera_frame'},
 {seq:6,at:105,event:'tool_finished',name:'get_camera_frame',state:'completed',seconds:.7},
 {seq:7,at:105,event:'llm_started',invocation:2,turn:2,model:'gpt-5.6-terra'},
 {seq:8,at:107,event:'llm',invocation:2,turn:2,outcome:'completed',seconds:2.2},
 {seq:9,at:107,event:'final',state:'succeeded',text:'完整总结\n'+('核验结果。'.repeat(400))}
];
const run={run_id:'r',timeline_version:1,state:'succeeded',events};
const rows=entries(run,200);
assert.deepEqual(Array.from(rows,r=>r.kind),['phase','llm','model_message','tool_started','tool_finished','llm','final']);
assert(rows[1].text.includes('3.1 秒'));assert(rows[5].text.includes('2.2 秒'));assert.equal(rows.at(-1).text,events.at(-1).text);
const pending=entries({timeline_version:1,state:'running',events:events.slice(0,2)},106);
assert(pending[1].text.includes('5.0 秒'));assert.equal(pending[1].kind,'llm-pending');
const interrupted=entries({timeline_version:1,state:'interrupted',events:events.slice(0,2)},106);
assert(interrupted[1].text.includes('耗时未完整记录'));
const legacy=entries({run_id:'old',state:'succeeded',model:'terra',events:[{event:'llm',turn:1,seconds:4}],calls:[{call_id:'c',request_id:'run-old-step-001',name:'get_status',state:'completed',seconds:.1}],summary:'旧总结'});
assert.equal(legacy[0].kind,'legacy');assert.equal(legacy.at(-1).text,'旧总结');assert(legacy.some(r=>r.text.includes('4.0 秒')));
const container=new Element('ol');render(container,run,200);assert.equal(container.children.length,7);
const children=container.children;render(container,run,200);assert.strictEqual(container.children,children,'unchanged polls must preserve DOM');
const payload='<img src=x onerror=alert(1)>';render(container,{run_id:'x',timeline_version:1,state:'succeeded',events:[{event:'final',seq:1,text:payload,state:'succeeded'}]},200);
assert.equal(container.children[0].children[1].textContent,payload);assert.equal(container.children[0].children[1].children.length,0);
console.log('Timeline passed: chronology, per-call timing, live wait, cancellation/restart, legacy records, full summary, stable DOM and text escaping.');

const live={run_id:'live',timeline_version:1,state:'running',events:events.slice(0,7)};
render(container,live,108);const preservedNote=container.children[2];const pendingRow=container.children.at(-1);
render(container,live,109);assert.strictEqual(container.children[2],preservedNote);assert.strictEqual(container.children.at(-1),pendingRow);
assert(container.children.at(-1).children[1].textContent.includes('4.0 秒'));
console.log('Live counters update in place without replacing prior commentary.');
const scroller={id:'progress-scroll',scrollHeight:1000,clientHeight:200,scrollTop:0};container.parentElement=scroller;
const scrolling={...run,run_id:'scroll'};render(container,scrolling,200);assert.equal(scroller.scrollTop,1000,'new selection follows latest content');
scroller.scrollTop=250;scroller.scrollHeight=1100;render(container,{...scrolling,events:[...events,{seq:10,event:'phase',text:'new status'}]},201);assert.equal(scroller.scrollTop,250,'reading earlier content must not jump');
scroller.scrollTop=900;scroller.scrollHeight=1100;render(container,{...scrolling,events:[...events,{seq:10,event:'phase',text:'updated status'}]},202);assert.equal(scroller.scrollTop,1100,'near-bottom readers follow new content');
console.log('Progress scrolling passed: follow at bottom, preserve manual scroll, reset on task selection.');
const diagnosticRun={run_id:'d',timeline_version:1,state:'failed',events:[
 {event:'llm_started',seq:1,at:1,invocation:1,turn:1,model:'gpt-6-sol'},
 {event:'llm',seq:2,at:91,invocation:1,outcome:'failed',seconds:90,diagnostics:{phase:'waiting_headers',request_bytes:10240,request_sent_s:.5,exception_type:'TimeoutError'}}]};
const diagnosticText=entries(diagnosticRun)[0].text;
assert(diagnosticText.includes('等待响应头'));assert(diagnosticText.includes('10.0 KB'));assert(diagnosticText.includes('TimeoutError'));
assert(!diagnosticText.includes('首字节'),'missing timings must not appear as zero');
console.log('Request diagnostics visible; older events and missing timings remain supported.');
const retryRows=entries({timeline_version:1,state:'running',events:[
 {seq:1,event:'llm_connection_retry',text:'1秒后进行第2/5次尝试。'},
 {seq:2,event:'llm_started',invocation:2,turn:1,model:'gpt-6-sol',at:1,connection_attempt:2,max_connection_attempts:5}]},2);
assert.equal(retryRows[0].title,'连接重试');assert(retryRows[0].text.includes('第2/5次'));assert(retryRows[1].title.includes('连接尝试 2/5'));
const recoveryRows=entries({timeline_version:1,state:'running',events:[{seq:1,event:'motion_recovery',text:'已停稳，重新规划',retry_budget:{planning_attempts:2,execution_attempts:1}},{seq:2,event:'tool_finished',name:'execute_plan',state:'recoverable_failure',seconds:1}]});
assert(recoveryRows[0].text.includes('规划 2/5'));assert(recoveryRows[1].text.includes('允许重新规划'));
