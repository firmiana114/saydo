const fs=require('fs'),vm=require('vm'),assert=require('assert');
const elements={};
for(const id of ['teach-grasp','teach-force','teach-open-position','teach-point','teach-feedback']){
 let value='';elements[id]={disabled:false,checked:false,textContent:'',get value(){return value;},set value(v){value=String(v);},checkValidity(){const n=Number(value);return value!==''&&Number.isInteger(n)&&n>=(id==='teach-force'?1:0)&&n<=(id==='teach-force'?100:1000);}};
}
const context={};vm.createContext(context);vm.runInContext(fs.readFileSync(__dirname+'/teach_grasp_panel.js','utf8'),context);
let calls=[],resolve;
const panel=new context.TeachGraspPanel((path,data)=>{calls.push({path,data});return new Promise(r=>{resolve=r;});},{querySelector:s=>elements[s.slice(1)]});
const state=(object_id,force,opening,busy=false)=>({object_id,state:'tracking',quality:{good:true},grasp:{busy,grasp_configuration:{teaching_defaults:{force_target_raw:force,open_position:opening}}}});
(async()=>{
 panel.update(state('ob_paper_cup_85mm',10,1000));assert.equal(elements['teach-force'].value,'10');assert.equal(elements['teach-open-position'].value,'1000');
 elements['teach-force'].value='15';elements['teach-force'].oninput();panel.update(state('ob_paper_cup_85mm',10,1000));assert.equal(elements['teach-force'].value,'15','polling must preserve draft');assert.equal(calls.length,0,'editing must not send hardware/config requests');
 panel.update(state('mug_iphone_20260917',75,200));assert.equal(elements['teach-force'].value,'75');assert.equal(elements['teach-open-position'].value,'200','object switch must reset object-specific defaults');
 panel.update(state('mug_iphone_20260917',75,200,true));assert(elements['teach-grasp'].disabled);await panel.record();assert.equal(calls.length,0);
 panel.update(state('mug_iphone_20260917',75,200));elements['teach-force'].value='';await panel.record();assert.equal(calls.length,0);
 elements['teach-force'].value='75';elements['teach-point'].checked=true;const pending=panel.record();assert(panel.pending);assert(elements['teach-force'].disabled);
 assert.equal(calls.length,1);assert.equal(calls[0].path,'/api/grasp/teach');assert.deepEqual(JSON.parse(JSON.stringify(calls[0].data)),{object_id:'mug_iphone_20260917',use_current_point:true,gripper_parameters:{force_target_raw:75,open_position:200}});
 resolve({message:'已记录参数'});await pending;assert(!panel.pending);assert.equal(elements['teach-feedback'].textContent,'已记录参数');
 const html=fs.readFileSync(__dirname+'/live_tracker.html','utf8');
 assert(!html.includes('id="gripper-force"'));assert(!html.includes('id="gripper-open-position"'));
 assert(!html.includes("post('/api/gripper/force-config'"));assert(!html.includes("post('/api/gripper/open-position'"));
 assert(html.includes('id="teach-grasp-panel"'));assert(!html.includes('<summary>记录夹法</summary>'));
 console.log('Teaching UI passed: object defaults, draft retention, no automatic writes, bound payload, busy/validation gates and visible layout.');
})().catch(e=>{console.error(e);process.exitCode=1;});
