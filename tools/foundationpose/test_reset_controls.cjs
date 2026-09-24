/* Execute the actual page script against a fake DOM and HTTP service; no robot access. */
const fs=require('fs'),vm=require('vm'),assert=require('assert');
const html=fs.readFileSync(__dirname+'/live_tracker.html','utf8');
const source=html.match(/<script>([\s\S]*?)<\/script>/)[1];
const elements=new Map();
function el(id){
  if(!elements.has(id))elements.set(id,{disabled:false,value:'',style:{},textContent:'',getContext(){return {};},addEventListener(){}});
  return elements.get(id);
}
let state={state:'paused',grasp:{busy:false,motion_enabled:true,reset_available:true,zero_reset_available:true}};
const calls=[];let resolvePost;
const sandbox={document:{querySelector:el},setInterval(){},setTimeout(){},console,
  ReorientationPanel:class{update(){}},PlacementPanel:class{update(){}},GraspEditorPanel:class{update(){}},
  fetch:async(path,options)=>{
    if(path==='/api/status')return {json:async()=>state};
    calls.push(path);
    return new Promise(resolve=>{resolvePost=()=>resolve({ok:true,json:async()=>({message:'模拟复位'})});});
  },
};
vm.createContext(sandbox);vm.runInContext(fs.readFileSync(__dirname+'/teach_grasp_panel.js','utf8'),sandbox);vm.runInContext(fs.readFileSync(__dirname+'/arm_speed_panel.js','utf8'),sandbox);vm.runInContext(source,sandbox);
const tick=()=>new Promise(resolve=>setImmediate(resolve));
async function update(){await tick();await vm.runInContext('update()',sandbox);await tick();}
async function main(){
  await update();assert(!el('#arm-reset').disabled&&!el('#arm-reset-zero').disabled);
  let action=el('#arm-reset-zero').onclick();await tick();
  assert.deepEqual(calls,['/api/arm/reset-zero']);assert(el('#arm-reset').disabled&&el('#arm-reset-zero').disabled);
  await el('#arm-reset').onclick();assert.equal(calls.length,1,'pending zero reset blocks the other reset');
  resolvePost();await action;await update();
  action=el('#arm-reset').onclick();await tick();assert.equal(calls[1],'/api/arm/reset');resolvePost();await action;
  state.grasp.reset_available=false;await update();assert(el('#arm-reset').disabled);assert(!el('#arm-reset-zero').disabled);
  state.grasp.busy=true;await update();assert(el('#arm-reset').disabled&&el('#arm-reset-zero').disabled);assert(!el('#grasp-stop').disabled);
  state.grasp.busy=false;state.grasp.motion_enabled=false;await update();assert(el('#arm-reset').disabled&&el('#arm-reset-zero').disabled);
  assert(html.includes('机械臂复位(观测桌面)')&&html.includes('机械臂复位(零位)'));
  console.log('Reset UI checks passed: names, endpoints, pending/busy/preview gates and independent zero target.');
}
main().catch(error=>{console.error(error);process.exitCode=1;});
