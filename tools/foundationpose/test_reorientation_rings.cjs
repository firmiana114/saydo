/* Ring geometry and actual panel pointer handlers, without GL or hardware. */
const fs=require('fs'),vm=require('vm'),assert=require('assert');
const source=fs.readFileSync(__dirname+'/reorientation_viewer.js','utf8'),context=vm.createContext({console});
vm.runInContext(source,context);
const M=vm.runInContext('ReorientationMath',context),G=vm.runInContext('ReorientationRings',context),Viewer=vm.runInContext('ReorientationViewer',context);
const plain=x=>JSON.parse(JSON.stringify(x)),near=(a,b)=>assert(a.flat().every((v,i)=>Math.abs(v-b.flat()[i])<1e-9));
const tilted=M.euler(31,-24,67),center=[.03,-.02,.01],pivot=[.07,.02,-.04];
const add=(a,b)=>a.map((v,i)=>v+b[i]),scale=(a,n)=>a.map(v=>v*n);
for(const mode of ['object-ring','base-ring'])for(let axis=0;axis<3;axis++){
 const frame=G.basis(tilted,mode),normal=frame.map(row=>row[axis]),u=frame.map(row=>row[(axis+1)%3]),v=frame.map(row=>row[(axis+2)%3]);
 const ray=p=>({origin:add(center,add(p,scale(normal,2))),direction:scale(normal,-1)});
 near([G.angle(ray(u),ray(v),center,normal)],[Math.PI/2]);
 const unit=[0,0,0];unit[axis]=1;const delta=M.axisAngle(unit,Math.PI/2),result=M.rotate(tilted,delta,mode);
 near(result,mode==='base-ring'?M.multiply(delta,tilted):M.multiply(tilted,delta));
 near(add(M.apply(result,pivot),M.objectOrigin(result,pivot,center)),center);
}
assert.equal(G.angle({origin:[0,0,1],direction:[1,0,0]},{origin:[0,1,1],direction:[1,0,0]},[0,0,0],[0,0,1]),null);
// Ray unprojection follows the same zoom, orbit and translated center as rendering.
for(const basis of [M.defaultView(),M.orbit(M.defaultView(),[73,57],[98,67],146,115)]){
 const p=[.08,-.04,.02],displayCenter=scale(center,.5),extent=.19,relative=p.map((v,i)=>v-displayCenter[i]);
 const pixel=[400+M.dot(basis[0],relative)*400/(2*extent),200-M.dot(basis[1],relative)*400/(2*extent)];
 const ray=G.ray(pixel,800,400,basis,extent,displayCenter),n=basis[2],t=M.dot(p.map((v,i)=>v-ray.origin[i]),n)/M.dot(ray.direction,n);
 near(add(ray.origin,scale(ray.direction,t)),p);
}
context.document={createElementNS:()=>({setAttribute(){},appendChild(){}})};
const viewer=Object.create(Viewer.prototype);Object.assign(viewer,{rotation:tilted,translation:center,basis:M.defaultView(),radius:.12,zoom:1,showAxes:false,model:{holding:true},overlay:{replaceChildren(){}}});
const project=p=>[400+p[0]*1000,200-p[1]*1000];
viewer.dragMode='base-ring';viewer.drawOverlay(800,400,[0,0,0],project);const fixed=plain(viewer.ringHandles);
viewer.rotation=M.euler(47,8,-92);viewer.drawOverlay(800,400,[0,0,0],project);assert.deepEqual(plain(viewer.ringHandles),fixed);
viewer.dragMode='object-ring';viewer.drawOverlay(800,400,[0,0,0],project);assert.notDeepEqual(plain(viewer.ringHandles),fixed);
viewer.dragMode='free';viewer.drawOverlay(800,400,[0,0,0],project);assert.equal(viewer.ringHandles.length,0);

const elements=new Map(),posts=[];
function el(id){if(!elements.has(id))elements.set(id,{value:id==='#reorientation-drag-mode'?'free':'0',checked:true,classList:{add(){},remove(){}},
 focus(){},setAttribute(){},addEventListener(){},getBoundingClientRect(){return {left:0,top:0,width:800,height:400};},clientWidth:800,clientHeight:400,
 setPointerCapture(){},hasPointerCapture(){return false;},checkValidity(){return true;}});return elements.get(id);}
const c=vm.createContext({document:{querySelector:el},post:()=>posts.push(1)});
vm.runInContext(source+`
const ActualViewer=ReorientationViewer;
ReorientationViewer=function(canvas){const v=Object.create(ActualViewer.prototype);Object.assign(v,{canvas,basis:ReorientationMath.defaultView(),rotation:ReorientationMath.identity(),translation:[0,0,0],radius:.12,pivot:[.07,.02,-.04],zoom:1,model:{},draw(){}});return v;};
const panel=new ReorientationPanel();panel.disabled=false;
`,c);
const panel=vm.runInContext('panel',c),canvas=panel.viewer.canvas,modeInput=el('#reorientation-drag-mode');
const event=(x,extra={})=>({button:0,pointerId:1,clientX:x,clientY:0,shiftKey:false,preventDefault(){},...extra});
for(const mode of ['object-ring','base-ring'])for(let axis=0;axis<3;axis++)for(const fine of [false,true]){
 panel.viewer.rotation=plain(tilted);panel.viewer.translation=center.slice();modeInput.value=mode;modeInput.onchange();
 const frame=G.basis(tilted,mode),normal=frame.map(row=>row[axis]),u=frame.map(row=>row[(axis+1)%3]),v=frame.map(row=>row[(axis+2)%3]);
 panel.viewer.ringHandles=[{axis,normal,points:[[0,0],[1,0]]}];
 panel.viewer.ringRay=p=>{const a=p[0]*Math.PI/180,relative=add(scale(u,Math.cos(a)),scale(v,Math.sin(a)));return {origin:add(center,add(relative,scale(normal,2))),direction:scale(normal,-1)};};
 canvas.onpointerdown(event(0,{shiftKey:fine}));
 for(const angle of [90,170,190,270])canvas.onpointermove(event(angle));
 const unit=[0,0,0];unit[axis]=1;near(panel.viewer.rotation,M.rotate(tilted,M.axisAngle(unit,Math.PI*1.5*(fine?.2:1)),mode));
 near(panel.viewer.translation,center);assert(panel.poseDirty);assert.equal(panel.selectedRingAxis,axis);
 canvas.onpointerup(event(270));near(panel.viewer.translation,[0,0,0]);
}
panel.viewer.rotation=plain(tilted);panel.viewer.translation=center.slice();modeInput.value='base-ring';modeInput.onchange();
canvas.onpointerdown(event(400));canvas.onpointermove(event(430));near(panel.viewer.rotation,tilted);near(panel.viewer.translation,center);
// Mode switching only redraws; a robot-busy transition cancels editing.
modeInput.value='object-ring';modeInput.onchange();near(panel.viewer.rotation,tilted);near(panel.viewer.translation,center);
panel.disabled=true;canvas.onpointerdown(event(0));canvas.onpointermove(event(90));near(panel.viewer.rotation,tilted);
assert.equal(posts.length,0);
const html=fs.readFileSync(__dirname+'/live_tracker.html','utf8'),select=html.match(/id="reorientation-drag-mode">([\s\S]*?)<\/select>/)[1];
assert(select.includes('value="free"')&&select.includes('value="object-ring"')&&select.includes('value="base-ring"'));assert(!select.includes('value="x"'));
console.log('Reorientation rings passed: local/base axes, fixed held pivot, ray projection, rendered handles, fine multi-turn drag, miss/busy guards and mode switching.');
