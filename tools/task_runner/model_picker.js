/* Model preferences apply to the next submitted run; they never mutate a running run. */
(function(root){
'use strict';
const labels={low:'低（low）',medium:'中（medium）',high:'高（high）'};
class ModelPicker{
 constructor(model,effort,storage,onChange){
  this.model=model;this.effort=effort;this.storage=storage;this.onChange=onChange;this.ready=false;
  const change=()=>{try{this.storage?.setItem('saydo.task-llm',JSON.stringify(this.selection()));}catch(_){}this.onChange(this.selection());};
  model.addEventListener('change',change);effort.addEventListener('change',change);
 }
 configure(data){
  const options=data.llm_options;
  if(!options?.models?.length||!options?.reasoning_efforts?.length)return;
  const signature=JSON.stringify([options,data.model,data.reasoning_effort]);
  if(this.ready&&signature===this.signature)return;
  let saved=this.ready?this.selection():null;
  if(!saved)try{saved=JSON.parse(this.storage?.getItem('saydo.task-llm')||'null');}catch(_){}
  const fill=(element,values,format)=>element.replaceChildren(...values.map(value=>{const option=document.createElement('option');option.value=value;option.textContent=format(value);return option;}));
  fill(this.model,options.models,value=>value);fill(this.effort,options.reasoning_efforts,value=>labels[value]||value);
  const migratedModel={'gpt-5.6-sol':'gpt-6-sol','gpt-5.6-terra':'gpt-6-sol','gpt-5.6-luna':'gpt-6-luna'}[saved?.model];
  if(migratedModel&&options.models.includes(migratedModel))saved={...saved,model:migratedModel};
  const defaultModel=options.models.includes(data.model)?data.model:options.models[0];
  const defaultEffort=options.reasoning_efforts.includes(data.reasoning_effort)?data.reasoning_effort:'medium';
  this.model.value=options.models.includes(saved?.model)?saved.model:defaultModel;
  this.effort.value=options.reasoning_efforts.includes(saved?.reasoning_effort)?saved.reasoning_effort:defaultEffort;
  this.ready=true;this.signature=signature;this.onChange(this.selection());
 }
 selection(){
  return {model:this.model.value,reasoning_effort:this.effort.value};
 }
 setBusy(busy){this.model.disabled=busy||!this.ready;this.effort.disabled=busy||!this.ready;}
}
root.SaydoModelPicker={ModelPicker,labels};
})(typeof window!=='undefined'?window:globalThis);
