'use strict';
const fs=require('fs'),vm=require('vm'),assert=require('assert');
const elements=new Map();
const element=id=>{
  if(!elements.has(id))elements.set(id,{value:'0',type:'number',checked:true,disabled:false,style:{},classList:{add(){},remove(){}},
    checkValidity(){return this.value!==''&&Number.isFinite(Number(this.value));},addEventListener(){},setAttribute(){},
    getBoundingClientRect(){return {left:0,top:0,width:600,height:400};}});
  return elements.get(id);
};
const context=vm.createContext({console,document:{querySelector:element},post:async()=>({})});
vm.runInContext(fs.readFileSync(__dirname+'/reorientation_viewer.js','utf8')+'\n'+fs.readFileSync(__dirname+'/grasp_editor.js','utf8'),context);
vm.runInContext(`
ReorientationViewer=function(canvas){return {canvas,model:{pose_mode:'5d'},rotation:ReorientationMath.identity(),translation:[0,0,0],draw(){}};};
const panel=new ReorientationPanel();panel.fiveD=true;panel.disabled=false;
panel.axisInputs[0].value='9';panel.axisInputs[0].oninput();
`,context);
const read=s=>JSON.parse(vm.runInContext('JSON.stringify('+s+')',context));
assert.equal(read('panel.axisInputs[0].value'),'9','typing must not reformat or move the cursor after each digit');
vm.runInContext("panel.axisInputs[0].value='90';panel.axisInputs[0].oninput();panel.axisInputs[1].value='45';panel.axisInputs[1].oninput();",context);
const axis=read('panel.viewer.rotation.map(row=>row[2])');
assert(Math.abs(axis[0]-Math.SQRT1_2)<1e-10&&Math.abs(axis[1]-Math.SQRT1_2)<1e-10&&Math.abs(axis[2])<1e-10);
vm.runInContext('panel.viewer.rotation=ReorientationMath.multiply(panel.viewer.rotation,ReorientationMath.axisAngle([0,0,1],1.7));panel.changed();',context);
assert(read('panel.viewer.rotation.map(row=>row[2])').every((v,i)=>Math.abs(v-axis[i])<1e-10),'free spin must not change the directed-axis goal');
const Editor=vm.runInContext('GraspEditorPanel',context),editor=Object.create(Editor.prototype),fields=new Map();
const field=(id,value)=>{const f={value,type:'number',checked:true,checkValidity(){return true;}};fields.set(id,f);return f;};
editor.q=id=>fields.get(id);field('name','5D侧夹').type='text';field('enabled','');field('priority','0');field('opening','800');field('force','30');field('distance','');field('clearance','');
editor.pointInputs=[field('px','123'),field('py','-45'),field('pz','25')];editor.angleInputs=[field('rx',''),field('ry',''),field('rz','')];
editor.fields=[...fields.values()];editor.fiveD=true;editor.approachAngle=field('approach-angle','70');
let value=editor.formValue();assert.deepEqual(JSON.parse(JSON.stringify(value.profile.grasp_point_object_m)),[0,0,.025]);assert.equal(value.profile.approach_angle_deg,70);
assert(Math.abs(value.profile.object_R_tcp[2][2]-Math.cos(70*Math.PI/180))<1e-10);
editor.approachAngle.value='181';assert.throws(()=>editor.formValue(),/夹角/);
const html=fs.readFileSync(__dirname+'/live_tracker.html','utf8');
assert(/id="reorientation-axis-tilt"[^>]*step="any"/.test(html));
assert(html.includes('/api/object/pose-mode'));
console.log('5D UI: axis-only grasp serialization, tilt/heading input, typing retention and free-spin equivalence passed.');
