const fs=require('fs'),vm=require('vm'),assert=require('assert');
class Select{constructor(){this.value='';this.events={};this.children=[];}addEventListener(n,f){this.events[n]=f;}replaceChildren(...rows){this.children=rows;this.value=rows[0]?.value||'';}}
const window={},document={createElement:()=>({})};vm.runInNewContext(fs.readFileSync(__dirname+'/model_picker.js','utf8'),{window,document});
const {ModelPicker}=window.SaydoModelPicker;
const options={model:'gpt-6-sol',reasoning_effort:'medium',llm_options:{models:['gpt-5.5','gpt-6-luna','gpt-6-sol','gpt-6-astra'],reasoning_efforts:['low','medium','high']}};
let stored=null;const storage={getItem:()=>stored,setItem:(k,v)=>{stored=v;}};
const model=new Select(),effort=new Select();let chosen;
const picker=new ModelPicker(model,effort,storage,value=>{chosen=value;});picker.configure(options);
assert.equal(model.value,'gpt-6-sol');assert.equal(effort.value,'medium');assert.equal(model.children.length,4);assert.equal(effort.children.length,3);
model.value='gpt-6-sol';effort.value='high';model.events.change();picker.configure(options);
assert.equal(model.value,'gpt-6-sol');assert.equal(effort.value,'high');assert.equal(chosen.reasoning_effort,'high');
picker.setBusy(true);assert(model.disabled&&effort.disabled);picker.setBusy(false);assert(!model.disabled&&!effort.disabled);
const restored=new ModelPicker(new Select(),new Select(),storage,()=>{});restored.configure(options);assert.equal(restored.selection().model,'gpt-6-sol');
stored='{"model":"gpt-6-terra","reasoning_effort":"invalid"}';const invalid=new ModelPicker(new Select(),new Select(),storage,()=>{});invalid.configure(options);assert.equal(invalid.selection().model,'gpt-6-sol');assert.equal(invalid.selection().reasoning_effort,'medium');
const denied=new ModelPicker(new Select(),new Select(),{getItem(){throw Error('disabled');},setItem(){throw Error('disabled');}},()=>{});denied.configure(options);denied.model.value='gpt-5.5';denied.model.events.change();assert.equal(denied.selection().model,'gpt-5.5');
console.log('Model picker passed: allowlist, defaults, saved selection, polling preservation, busy gating and unavailable storage.');

for(const [oldModel,newModel] of [['gpt-5.6-sol','gpt-6-sol'],['gpt-5.6-terra','gpt-6-sol'],['gpt-5.6-luna','gpt-6-luna']]){stored=JSON.stringify({model:oldModel,reasoning_effort:'high'});const migrated=new ModelPicker(new Select(),new Select(),storage,()=>{});migrated.configure(options);assert.equal(migrated.selection().model,newModel);assert.equal(migrated.selection().reasoning_effort,'high');}
