/* Pure editor geometry and form serialization; no camera or robot needed. */
const fs=require('fs'),vm=require('vm'),assert=require('assert');
const base=fs.readFileSync(__dirname+'/reorientation_viewer.js','utf8');
const source=fs.readFileSync(__dirname+'/grasp_editor.js','utf8');
const context=vm.createContext({console});vm.runInContext(base+'\n'+source,context);
const M=vm.runInContext('ReorientationMath',context),G=vm.runInContext('GraspEditorMath',context),Panel=vm.runInContext('GraspEditorPanel',context);
const plain=value=>JSON.parse(JSON.stringify(value));
let ray=G.ray([200,150],400,300,M.identity(),.2,[.4,.2,.1]);
assert.deepEqual(plain(ray.direction),[0,0,-1]);
const vertices=[[.3,.1,.1],[.5,.1,.1],[.4,.3,.1]],faces=[[0,1,2]];
assert(G.surface(ray,vertices,faces).every((x,i)=>Math.abs(x-[.4,.2,.1][i])<1e-9),'surface pick must retain model coordinates, not recenter the point');
assert.equal(G.surface(G.ray([0,0],400,300,M.identity(),.2,[.4,.2,.1]),vertices,faces),null);
assert.equal(G.segmentDistance([.5,.2],[0,0],[1,0]),.2);
let a={origin:[1,0,2],direction:[0,0,-1]},b={origin:[0,1,2],direction:[0,0,-1]};
assert(Math.abs(G.rotationDrag(M.identity(),2,a,b,[0,0,0])-Math.PI/2)<1e-9,'Z ring quarter turn');
assert.equal(G.rotationDrag(M.identity(),0,a,b,[0,0,0]),null,'edge-on rotation ring requires another view');
const near=(actual,expected)=>assert(actual.flat().every((v,i)=>Math.abs(v-expected.flat()[i])<1e-9));
const tilted=M.euler(31,-24,67),center=[.04,-.03,.02];
const quarters=[[[1,0,0],[0,0,-1],[0,1,0]],[[0,0,1],[0,1,0],[-1,0,0]],[[0,-1,0],[1,0,0],[0,0,1]]];
for(let axis=0;axis<3;axis++){
 near(G.rotationFrame(tilted,'object'),M.identity());
 near(G.rotationFrame(tilted,'tcp'),tilted);
 near(G.rotate(tilted,axis,Math.PI/2,'object'),M.multiply(quarters[axis],tilted));
 near(G.rotate(tilted,axis,Math.PI/2,'tcp'),M.multiply(tilted,quarters[axis]));
 // Exercise the actual drag handler with a non-origin center and tilted TCP.
 const normal=[0,0,0],u=[0,0,0],v=[0,0,0];normal[axis]=1;u[(axis+1)%3]=1;v[(axis+2)%3]=1;
 const makeRay=radial=>({origin:G.add(center,G.add(radial,G.scale(normal,2))),direction:G.scale(normal,-1)});
 const start=makeRay(u),end=makeRay(v);
 assert(Math.abs(G.rotationDrag(tilted,axis,start,end,center,'object')-Math.PI/2)<1e-9);
 for(const fine of [false,true]){
  const canvas={getBoundingClientRect:()=>({left:0,top:0}),setPointerCapture(){},addEventListener(){}},camera={};
  const p=Object.create(Panel.prototype);Object.assign(p,{locked:false,dirty:false,mode:{value:'rotate-object'},q:()=>camera,feedback:{textContent:''},
    draft:{profile:{grasp_point_object_m:center.slice(),object_R_tcp:tilted.map(row=>row.slice())}},writeFields(){},
    viewer:{canvas,handles:[{axis,ring:[[0,0],[1,0]],frame:'object'}],ray:pixel=>pixel[0]===0?start:end}});
  p.bindGestures();canvas.onpointerdown({button:0,pointerId:1,clientX:0,clientY:0,shiftKey:fine,preventDefault(){}});
  canvas.onpointermove({pointerId:1,clientX:1,clientY:0});canvas.onpointerup({pointerId:1});
  near(p.draft.profile.object_R_tcp,G.rotate(tilted,axis,Math.PI/2*(fine?.2:1),'object'));
  assert.deepEqual(p.draft.profile.grasp_point_object_m,center,'object-axis rotation must keep the grasp center fixed');assert(p.dirty);
 }
}
// The rendered object rings stay fixed while TCP rings follow the orientation.
context.document={createElementNS:()=>({children:[],setAttribute(){},appendChild(e){this.children.push(e);}})};
const Viewer=vm.runInContext('GraspEditorViewer',context),viewer=Object.create(Viewer.prototype);
Object.assign(viewer,{grasp:{grasp_point_object_m:center,object_R_tcp:tilted},distance:.05,radius:.1,basis:M.defaultView(),zoom:1,overlay:{replaceChildren(){}}});
const project=p=>[200+p[0]*1000,150-p[1]*1000];
viewer.editMode='rotate-object';viewer.drawOverlay(600,400,[0,0,0],project);const fixed=plain(viewer.handles);
viewer.grasp.object_R_tcp=G.rotate(tilted,0,.8,'object');viewer.drawOverlay(600,400,[0,0,0],project);assert.deepEqual(plain(viewer.handles),fixed);
viewer.editMode='rotate';viewer.drawOverlay(600,400,[0,0,0],project);assert.notDeepEqual(plain(viewer.handles).map(h=>h.ring),fixed.map(h=>h.ring));
assert(fs.readFileSync(__dirname+'/live_tracker.html','utf8').includes('value="rotate-object"'));
for(const basis of [M.defaultView(),M.orbit(M.defaultView(),[73,57],[98,67],146,115)]){
 const point=[.08,-.03,.04],scale=.2,width=400,height=300,pivot=[.1,.1,0],world=G.sub(point,pivot);
 const pixel=[width/2+M.dot(basis[0],world)*height/(2*scale),height/2-M.dot(basis[1],world)*height/(2*scale)];
 const hit=G.plane(G.ray(pixel,width,height,basis,scale,pivot),point,basis[2]);assert(hit.every((v,i)=>Math.abs(v-point[i])<1e-9));
}
const inputs=new Map(),input=(id,value='')=>{const e={value,checked:true,type:'number',checkValidity(){return true;}};inputs.set(id,e);return e;};
const panel=Object.create(Panel.prototype);panel.q=id=>inputs.get(id);
input('name','瓶身侧夹').type='text';input('enabled').type='checkbox';input('priority','7');input('distance','80');input('opening','500');input('force','20');
const original=M.euler(23.456789,37.234567,41.345678),point=[.012345678,.003456789,-.005678912];
panel.draft={profile:{grasp_point_object_m:point,object_R_tcp:original}};
panel.pointInputs=point.map((v,i)=>input('p'+i,(v*1000).toFixed(2)));panel.angleInputs=M.angles(original).map((v,i)=>input('r'+i,v.toFixed(2)));panel.fields=[...inputs.values()];
let result=plain(panel.formValue());assert.deepEqual(result.profile.object_R_tcp,plain(original),'metadata edits must not quantize/rebind the saved orientation');assert.deepEqual(result.profile.grasp_point_object_m,point);
assert.deepEqual(result.overrides,{pregrasp_distance_m:.08,open_position:500,force_target_raw:20});
panel.pointInputs[0].value='42';panel.angleInputs[0].value='90';result=plain(panel.formValue());assert.equal(result.profile.grasp_point_object_m[0],.042);assert(Math.abs(M.angles(result.profile.object_R_tcp)[0]-90)<1e-8);
input('force','');assert(!Object.hasOwn(panel.formValue().overrides,'force_target_raw'));
panel.angleInputs[1].value='';assert.throws(()=>panel.formValue());
console.log('Grasp editor checks passed: ray/surface/plane selection, arbitrary origin, ring rotation, view invariance, unit conversion and lossless metadata edits.');
const unsaved=Object.create(Panel.prototype);let sent=false;
Object.assign(unsaved,{pending:false,locked:false,dirty:true,data:{},feedback:{textContent:''},send:async()=>{sent=true;}});
unsaved.mutate('select',{selection_mode:'auto'}).then(()=>{assert(!sent,'unsaved edits must block changing the grasp selection');assert(unsaved.feedback.textContent.includes('保存草稿'));}).catch(error=>{console.error(error);process.exitCode=1;});
