/* Rotations are row-major in the API. The renderer preserves the model origin. */
'use strict';
const PlacementMath={
  identity:()=>[[1,0,0],[0,1,0],[0,0,1]],
  multiply:(a,b)=>a.map(row=>b[0].map((_,j)=>row.reduce((s,v,k)=>s+v*b[k][j],0))),
  apply:(r,p)=>r.map(row=>row.reduce((s,v,k)=>s+v*p[k],0)),
  euler(x,y,z){const [a,b,c]=[x,y,z].map(v=>v*Math.PI/180),cx=Math.cos(a),sx=Math.sin(a),cy=Math.cos(b),sy=Math.sin(b),cz=Math.cos(c),sz=Math.sin(c);
    return [[cz*cy,cz*sy*sx-sz*cx,cz*sy*cx+sz*sx],[sz*cy,sz*sy*sx+cz*cx,sz*sy*cx-cz*sx],[-sy,cy*sx,cy*cx]];},
  angles(r){const y=Math.asin(Math.max(-1,Math.min(1,-r[2][0]))),regular=Math.abs(Math.cos(y))>1e-6;
    return [(regular?Math.atan2(r[2][1],r[2][2]):0),y,(regular?Math.atan2(r[1][0],r[0][0]):Math.atan2(-r[0][1],r[1][1]))].map(v=>v*180/Math.PI);},
  column(r){return new Float32Array([r[0][0],r[1][0],r[2][0],r[0][1],r[1][1],r[2][1],r[0][2],r[1][2],r[2][2]]);}
};
class PlacementViewer{
  constructor(canvas,overlay){
    this.canvas=canvas;this.overlay=overlay;this.rotation=PlacementMath.identity();
    const gl=this.gl=canvas.getContext('webgl',{antialias:true});
    if(!gl)throw Error('浏览器未启用三维绘图，请开启硬件加速后刷新。');
    const shader=(type,source)=>{const s=gl.createShader(type);gl.shaderSource(s,source);gl.compileShader(s);if(!gl.getShaderParameter(s,gl.COMPILE_STATUS))throw Error(gl.getShaderInfoLog(s));return s;};
    const program=this.program=gl.createProgram();
    gl.attachShader(program,shader(gl.VERTEX_SHADER,'attribute vec3 p;attribute vec3 n;uniform mat3 rotation;uniform vec3 origin;uniform mat4 view;varying vec3 normal;void main(){gl_Position=view*vec4(rotation*p+origin,1.0);normal=rotation*n;}'));
    gl.attachShader(program,shader(gl.FRAGMENT_SHADER,'precision mediump float;varying vec3 normal;uniform vec4 color;uniform float shaded;void main(){float light=mix(1.0,0.45+0.55*abs(dot(normalize(normal),normalize(vec3(0.4,-0.5,1.0)))),shaded);gl_FragColor=vec4(color.rgb*light,color.a);}'));
    gl.linkProgram(program);if(!gl.getProgramParameter(program,gl.LINK_STATUS))throw Error(gl.getProgramInfoLog(program));
    this.u={};for(const key of ['rotation','origin','view','color','shaded'])this.u[key]=gl.getUniformLocation(program,key);
    this.p=gl.getAttribLocation(program,'p');this.n=gl.getAttribLocation(program,'n');
    this.buffer=gl.createBuffer();this.lines=gl.createBuffer();
    new ResizeObserver(()=>this.draw()).observe(canvas);
  }
  setModel(model){
    this.model=model;const data=new Float32Array(model.faces.length*18);let index=0;
    for(const face of model.faces){const [a,b,c]=face.map(i=>model.vertices[i]),u=b.map((v,k)=>v-a[k]),v=c.map((x,k)=>x-a[k]);
      const n=[u[1]*v[2]-u[2]*v[1],u[2]*v[0]-u[0]*v[2],u[0]*v[1]-u[1]*v[0]],norm=Math.hypot(...n)||1;
      for(const point of [a,b,c])for(const x of [...point,...n.map(x=>x/norm)])data[index++]=x;
    }
    this.count=data.length/6;const gl=this.gl;gl.bindBuffer(gl.ARRAY_BUFFER,this.buffer);gl.bufferData(gl.ARRAY_BUFFER,data,gl.STATIC_DRAW);
    this.rotation=model.initial_rotation;this.draw();
  }
  bind(buffer){const gl=this.gl;gl.bindBuffer(gl.ARRAY_BUFFER,buffer);gl.enableVertexAttribArray(this.p);gl.enableVertexAttribArray(this.n);gl.vertexAttribPointer(this.p,3,gl.FLOAT,false,24,0);gl.vertexAttribPointer(this.n,3,gl.FLOAT,false,24,12);}
  line(a,b,color){const gl=this.gl;gl.bindBuffer(gl.ARRAY_BUFFER,this.lines);gl.bufferData(gl.ARRAY_BUFFER,new Float32Array([...a,0,0,1,...b,0,0,1]),gl.DYNAMIC_DRAW);this.bind(this.lines);gl.uniform4fv(this.u.color,color);gl.drawArrays(gl.LINES,0,2);}
  draw(){
    if(!this.model)return;
    const gl=this.gl,canvas=this.canvas,ratio=Math.min(window.devicePixelRatio||1,2),width=canvas.clientWidth,height=canvas.clientHeight;
    if(!width||!height)return;
    canvas.width=Math.round(width*ratio);canvas.height=Math.round(height*ratio);
    const rotated=this.model.vertices.map(p=>PlacementMath.apply(this.rotation,p));
    const low=[Infinity,Infinity,Infinity],high=[-Infinity,-Infinity,-Infinity];
    for(const p of rotated)for(let k=0;k<3;k++){low[k]=Math.min(low[k],p[k]);high[k]=Math.max(high[k],p[k]);}
    const z=this.model.release_clearance_m-low[2],center=[(low[0]+high[0])/2,(low[1]+high[1])/2,(high[2]-low[2])/2];
    const scale=Math.max(.18,Math.hypot(...high.map((v,k)=>v-low[k]))*1.15,Math.hypot(...center)*1.3),aspect=width/height;
    const right=[Math.SQRT1_2,Math.SQRT1_2,0],up=[-.408248,.408248,.816497],depth=[.577350,-.577350,.577350];
    const dot=(a,b)=>a.reduce((s,x,k)=>s+x*b[k],0),sx=1/(scale*aspect),sy=1/scale,sz=-1/(scale*8);
    const view=new Float32Array([right[0]*sx,up[0]*sy,depth[0]*sz,0,right[1]*sx,up[1]*sy,depth[1]*sz,0,right[2]*sx,up[2]*sy,depth[2]*sz,0,-dot(right,center)*sx,-dot(up,center)*sy,-dot(depth,center)*sz,1]);
    gl.viewport(0,0,canvas.width,canvas.height);gl.clearColor(.028,.06,.085,1);gl.clear(gl.COLOR_BUFFER_BIT|gl.DEPTH_BUFFER_BIT);gl.enable(gl.DEPTH_TEST);gl.useProgram(this.program);gl.uniformMatrix4fv(this.u.view,false,view);
    gl.uniformMatrix3fv(this.u.rotation,false,PlacementMath.column(PlacementMath.identity()));gl.uniform3f(this.u.origin,0,0,0);gl.uniform1f(this.u.shaded,0);
    const grid=Math.max(.15,scale),step=.05;
    for(let i=-Math.ceil(grid/step);i<=Math.ceil(grid/step);i++){
      this.line([i*step,-grid,0],[i*step,grid,0],[.16,.24,.29,1]);this.line([-grid,i*step,0],[grid,i*step,0],[.16,.24,.29,1]);
    }
    gl.uniformMatrix3fv(this.u.rotation,false,PlacementMath.column(this.rotation));gl.uniform3f(this.u.origin,0,0,z);gl.uniform1f(this.u.shaded,1);gl.uniform4fv(this.u.color,[.64,.79,.87,1]);this.bind(this.buffer);gl.drawArrays(gl.TRIANGLES,0,this.count);
    gl.disable(gl.DEPTH_TEST);gl.uniform1f(this.u.shaded,0);
    const colors=[[1,.30,.30,1],[.30,.91,.46,1],[.35,.62,1,1]],labels=['X','Y','Z'];
    this.overlay.textContent='';
    const project=p=>[(dot(right,p)-dot(right,center))*sx*width/2+width/2,height/2-(dot(up,p)-dot(up,center))*sy*height/2];
    const label=(text,x,y,color)=>{const span=document.createElement('span');span.textContent=text;span.style.cssText=`position:absolute;left:${x}px;top:${y}px;color:${color};font:bold 14px system-ui;pointer-events:none`;this.overlay.appendChild(span);};
    for(let i=0;i<3;i++){const end=[0,0,0];end[i]=.05;this.line([0,0,0],end,colors[i]);const world=PlacementMath.apply(this.rotation,end);world[2]+=z;const [x,y]=project(world);label('物品 '+labels[i],x+4,y-8,['#ff7777','#73e88a','#7caeff'][i]);}
    const [ox,oy]=project([0,0,z]);label('原点',ox+5,oy+4,'#ffffff');
    // The corner triad represents the fixed robot-base directions, not model axes.
    const base=document.createElement('div');base.style.cssText='position:absolute;left:12px;bottom:10px;background:#101820dd;padding:8px;border-radius:6px;font-size:12px';
    base.textContent='基座方向：X ↘　Y ↗　Z ↑　｜桌面网格 5 厘米';this.overlay.appendChild(base);
  }
}
class PlacementPanel{
  constructor(){
    this.key=null;this.pending=false;this.loading=false;this.generation=0;this.held=null;this.disabled=true;
    this.info=document.querySelector('#placement-info');this.feedback=document.querySelector('#placement-feedback');
    this.preview=document.querySelector('#place-preview');this.execute=document.querySelector('#place');this.restore=document.querySelector('#placement-restore');
    this.inputs=['x','y','z'].map(axis=>document.querySelector('#placement-'+axis));
    try{this.viewer=new PlacementViewer(document.querySelector('#placement-view'),document.querySelector('#placement-overlay'));}catch(e){this.info.textContent=e.message;return;}
    for(const input of this.inputs)input.oninput=()=>{if(this.disabled)return;if(!this.anglesValid()){this.preview.disabled=true;this.execute.disabled=true;this.feedback.textContent='请填写三个有效角度，每项在 −180 到 180 度之间。';return;}this.viewer.rotation=PlacementMath.euler(...this.inputs.map(x=>Number(x.value)));this.changed();};
    this.restore.onclick=()=>{this.viewer.rotation=this.viewer.model.initial_rotation.map(row=>row.slice());this.syncAngles();this.changed();};
    let anchor=null;const canvas=this.viewer.canvas;
    canvas.onpointerdown=e=>{if(this.disabled||!this.viewer.model)return;anchor=[e.clientX,e.clientY];canvas.setPointerCapture(e.pointerId);};
    canvas.onpointermove=e=>{if(!anchor||this.disabled)return;const [x,y]=anchor;anchor=[e.clientX,e.clientY];this.viewer.rotation=PlacementMath.multiply(PlacementMath.euler((e.clientY-y)*.4,0,(e.clientX-x)*.4),this.viewer.rotation);this.syncAngles();this.changed();};
    canvas.onpointerup=canvas.onpointercancel=()=>{anchor=null;};
    this.preview.onclick=()=>this.request(true);this.execute.onclick=()=>this.request(false);
  }
  syncAngles(){PlacementMath.angles(this.viewer.rotation).forEach((v,k)=>{this.inputs[k].value=v.toFixed(1);});}
  anglesValid(){return this.inputs.every(x=>x.value!==''&&x.checkValidity());}
  changed(){this.viewer.draw();this.feedback.textContent='朝向已调整；点击“预览放置方案”检查此朝向能否到达。';}
  async load(id,key){
    this.key=key;this.loading=true;const generation=++this.generation;this.info.textContent='正在加载放置模型…';
    try{const response=await fetch('/api/placement/model?object_id='+encodeURIComponent(id)),model=await response.json();if(!response.ok)throw Error(model.error);if(generation!==this.generation)return;
      this.viewer.setModel(model);this.syncAngles();this.info.textContent='模型：'+model.object_name+'。显示朝向相对于机械臂基座；物品原点与三根轴随模型一起旋转。';
      this.feedback.textContent=model.holding?'已关联本次抓取，可以预览或放置。':'可先旋转查看；成功抓取后加载该次抓取朝向，才能放置。';
    }catch(e){if(generation===this.generation){this.viewer.model=null;this.info.textContent='模型加载失败：'+e.message;}}
    finally{if(generation===this.generation)this.loading=false;}
  }
  update(state,otherPending){
    if(!this.viewer)return;
    const g=state.grasp||{};this.held=g.held_object;this.motionEnabled=g.motion_enabled;
    const id=this.held?.object_id||state.object_id,key=[id,this.held?.hold_id||'',this.held?.model_sha256||state.model_sha256].join(':');
    if(id&&key!==this.key)this.load(id,key);
    this.disabled=!!g.busy||otherPending||this.pending||this.loading||!this.viewer.model;
    this.inputs.forEach(x=>x.disabled=this.disabled);this.restore.disabled=this.disabled;
    const ready=!this.disabled&&this.anglesValid()&&this.held&&this.viewer.model?.object_id===this.held.object_id&&this.viewer.model?.model_sha256===this.held.model_sha256;
    this.preview.disabled=!ready;this.execute.disabled=!ready||!g.motion_enabled;
    if(g.operation==='place'&&g.busy)this.feedback.textContent=g.message;
    if(g.operation==='place'&&!g.busy&&g.placement_plan&&this.planShown!==g.placement_plan.hold_id+g.placement_plan.base_T_object_target.flat().join(',')){
      const p=g.placement_plan;this.planShown=p.hold_id+p.base_T_object_target.flat().join(',');
      this.feedback.textContent=g.message+'\n物品原点的固定落点：基座 X＝'+(p.target_xy_m[0]*100).toFixed(1)+' 厘米，Y＝'+(p.target_xy_m[1]*100).toFixed(1)+' 厘米。';
    }else if(g.operation==='place'&&!g.busy&&g.error)this.feedback.textContent=g.error;
  }
  async request(preview){
    if(this.pending||this.disabled||!this.held||!this.anglesValid())return;
    this.pending=true;this.preview.disabled=true;this.execute.disabled=true;
    try{await post(preview?'/api/place/preview':'/api/place',{object_id:this.held.object_id,hold_id:this.held.hold_id,model_sha256:this.viewer.model.model_sha256,rotation:this.viewer.rotation});this.feedback.textContent='正在检查指定朝向与落点…';this.planShown=null;}
    catch(e){this.feedback.textContent=e.message;}
    finally{this.pending=false;}
  }
}
