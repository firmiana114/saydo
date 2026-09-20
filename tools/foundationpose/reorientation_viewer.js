/* API rotations are row-major. Display materials and camera zoom never alter motion geometry. */
'use strict';
const ReorientationMath={
  identity:()=>[[1,0,0],[0,1,0],[0,0,1]],
  multiply:(a,b)=>a.map(row=>b[0].map((_,j)=>row.reduce((s,v,k)=>s+v*b[k][j],0))),
  apply:(r,p)=>r.map(row=>row.reduce((s,v,k)=>s+v*p[k],0)),
  dot:(a,b)=>a.reduce((s,v,k)=>s+v*b[k],0),
  cross:(a,b)=>[a[1]*b[2]-a[2]*b[1],a[2]*b[0]-a[0]*b[2],a[0]*b[1]-a[1]*b[0]],
  unit:p=>{const n=Math.hypot(...p)||1;return p.map(x=>x/n);},
  euler(x,y,z){const [a,b,c]=[x,y,z].map(v=>v*Math.PI/180),cx=Math.cos(a),sx=Math.sin(a),cy=Math.cos(b),sy=Math.sin(b),cz=Math.cos(c),sz=Math.sin(c);
    return [[cz*cy,cz*sy*sx-sz*cx,cz*sy*cx+sz*sx],[sz*cy,sz*sy*sx+cz*cx,sz*sy*cx-cz*sx],[-sy,cy*sx,cy*cx]];},
  angles(r){const y=Math.asin(Math.max(-1,Math.min(1,-r[2][0]))),regular=Math.abs(Math.cos(y))>1e-6;
    return [(regular?Math.atan2(r[2][1],r[2][2]):0),y,(regular?Math.atan2(r[1][0],r[0][0]):Math.atan2(-r[0][1],r[1][1]))].map(v=>v*180/Math.PI);},
  column(r){return new Float32Array([r[0][0],r[1][0],r[2][0],r[0][1],r[1][1],r[2][1],r[0][2],r[1][2],r[2][2]]);},
  rotate(rotation,delta,mode='free'){return mode==='free'?this.multiply(delta,rotation):this.multiply(rotation,delta);},
  objectOrigin(rotation,pivot){return this.apply(rotation,pivot).map(v=>-v);},
  axisAngle(axis,angle){const [x,y,z]=this.unit(axis),c=Math.cos(angle),s=Math.sin(angle),t=1-c;
    return [[t*x*x+c,t*x*y-s*z,t*x*z+s*y],[t*x*y+s*z,t*y*y+c,t*y*z-s*x],[t*x*z-s*y,t*y*z+s*x,t*z*z+c]];},
  ball(x,y,width,height){const radius=Math.min(width,height)*.42,p=[(x-width/2)/radius,(height/2-y)/radius],d=Math.hypot(...p);
    return d<=1?[...p,Math.sqrt(1-d*d)]:[p[0]/d,p[1]/d,0];},
  drag(start,end,width,height,basis,mode='free',fine=false){
    const factor=fine?.2:1;
    if(mode!=='free'){const axis=[0,0,0];axis['xyz'.indexOf(mode)]=1;return this.axisAngle(axis,(end[0]-start[0])*.006*factor);}
    const a=this.ball(...start,width,height),b=this.ball(...end,width,height),cross=this.cross(a,b),length=Math.hypot(...cross),dot=Math.max(-1,Math.min(1,this.dot(a,b)));
    if(length<1e-8&&dot>0)return this.identity();
    const axis=length<1e-8?this.unit(this.cross(a,Math.abs(a[0])<.8?[1,0,0]:[0,1,0])):cross.map(v=>v/length);
    const world=basis[0].map((_,k)=>axis.reduce((s,v,i)=>s+v*basis[i][k],0));
    return this.axisAngle(world,Math.atan2(length,dot)*factor);
  }
};
class ReorientationViewer{
  constructor(canvas,overlay){
    this.canvas=canvas;this.overlay=overlay;this.rotation=ReorientationMath.identity();this.zoom=1;this.version=0;this.textures=[];this.showAxes=true;
    this.basis=[[Math.SQRT1_2,Math.SQRT1_2,0],[-1/Math.sqrt(6),1/Math.sqrt(6),2/Math.sqrt(6)],[1/Math.sqrt(3),-1/Math.sqrt(3),1/Math.sqrt(3)]];
    const gl=this.gl=canvas.getContext('webgl',{antialias:true});
    if(!gl)throw Error('浏览器未启用三维绘图，请开启硬件加速后刷新。');
    const shader=(type,source)=>{const s=gl.createShader(type);gl.shaderSource(s,source);gl.compileShader(s);if(!gl.getShaderParameter(s,gl.COMPILE_STATUS))throw Error(gl.getShaderInfoLog(s));return s;};
    const program=this.program=gl.createProgram();
    gl.attachShader(program,shader(gl.VERTEX_SHADER,'attribute vec3 p;attribute vec3 n;attribute vec2 uv;attribute vec4 tint;uniform mat3 rotation;uniform vec3 origin;uniform mat4 view;varying vec3 normal;varying vec2 texcoord;varying vec4 vertexColor;void main(){gl_Position=view*vec4(rotation*p+origin,1.0);normal=rotation*n;texcoord=uv;vertexColor=tint;}'));
    gl.attachShader(program,shader(gl.FRAGMENT_SHADER,'precision mediump float;varying vec3 normal;varying vec2 texcoord;varying vec4 vertexColor;uniform vec4 color;uniform float shaded;uniform bool textured;uniform sampler2D image;void main(){vec4 base=color*vertexColor;if(textured)base*=texture2D(image,texcoord);if(base.a<0.05)discard;float light=mix(1.0,0.65+0.35*abs(dot(normalize(normal),normalize(vec3(0.4,-0.5,1.0)))),shaded);gl_FragColor=vec4(base.rgb*light,1.0);}'));
    gl.linkProgram(program);if(!gl.getProgramParameter(program,gl.LINK_STATUS))throw Error(gl.getProgramInfoLog(program));
    this.u={};for(const key of ['rotation','origin','view','color','shaded','textured','image'])this.u[key]=gl.getUniformLocation(program,key);
    this.a={};for(const key of ['p','n','uv','tint'])this.a[key]=gl.getAttribLocation(program,key);
    this.buffer=gl.createBuffer();this.lines=gl.createBuffer();
    this.white=gl.createTexture();gl.bindTexture(gl.TEXTURE_2D,this.white);gl.texImage2D(gl.TEXTURE_2D,0,gl.RGBA,1,1,0,gl.RGBA,gl.UNSIGNED_BYTE,new Uint8Array([255,255,255,255]));gl.texParameteri(gl.TEXTURE_2D,gl.TEXTURE_MIN_FILTER,gl.LINEAR);
    new ResizeObserver(()=>this.draw()).observe(canvas);
  }
  setModel(model){
    const gl=this.gl,version=++this.version;for(const texture of this.textures)if(texture)gl.deleteTexture(texture);this.textures=[];
    this.model=model;this.zoom=1;this.pivot=model.pivot_object_m||[0,0,0];this.radius=model.vertices.reduce((r,p)=>Math.max(r,Math.hypot(...p.map((v,k)=>v-this.pivot[k]))),.04);
    const appearance=model.appearance||{},materials=appearance.materials?.length?appearance.materials:[{color:[.64,.79,.87,1]}];
    this.materials=materials;this.groups=[];this.textureState=appearance.status==='unavailable'?'纹理不可用，已显示基础材质':appearance.status==='colored'?'顶点颜色':'基础材质（源模型无可用纹理）';
    const rows=materials.map(()=>[]);
    model.faces.forEach((face,fi)=>{
      const [a,b,c]=face.map(i=>model.vertices[i]),normal=ReorientationMath.unit(ReorientationMath.cross(b.map((v,k)=>v-a[k]),c.map((v,k)=>v-a[k]))),row=rows[appearance.face_materials?.[fi]||0];
      for(const vi of face)row.push(...model.vertices[vi],...normal,...(appearance.uv?.[vi]||[0,0]),...(appearance.vertex_colors?.[vi]||appearance.face_colors?.[fi]||[1,1,1,1]));
    });
    let offset=0;const data=new Float32Array(model.faces.length*3*12);
    rows.forEach((row,i)=>{data.set(row,offset*12);this.groups.push({first:offset,count:row.length/12,material:i});offset+=row.length/12;});
    gl.bindBuffer(gl.ARRAY_BUFFER,this.buffer);gl.bufferData(gl.ARRAY_BUFFER,data,gl.STATIC_DRAW);
    this.rotation=model.initial_rotation.map(row=>row.slice());
    let pending=materials.filter(m=>m.texture).length,failed=0;
    if(pending)this.textureState='正在加载纹理…';
    materials.forEach((material,i)=>{
      if(!material.texture)return;
      const image=new Image();image.onload=()=>{
        if(version!==this.version)return;
        const texture=gl.createTexture();gl.bindTexture(gl.TEXTURE_2D,texture);gl.pixelStorei(gl.UNPACK_FLIP_Y_WEBGL,true);
        gl.texImage2D(gl.TEXTURE_2D,0,gl.RGBA,gl.RGBA,gl.UNSIGNED_BYTE,image);
        gl.texParameteri(gl.TEXTURE_2D,gl.TEXTURE_WRAP_S,gl.CLAMP_TO_EDGE);gl.texParameteri(gl.TEXTURE_2D,gl.TEXTURE_WRAP_T,gl.CLAMP_TO_EDGE);
        gl.texParameteri(gl.TEXTURE_2D,gl.TEXTURE_MIN_FILTER,gl.LINEAR);gl.texParameteri(gl.TEXTURE_2D,gl.TEXTURE_MAG_FILTER,gl.LINEAR);
        this.textures[i]=texture;finish();
      };
      const finish=()=>{pending--;this.textureState=pending?'正在加载纹理…':failed?'部分纹理加载失败，已使用基础材质':'原始模型纹理';this.draw();};
      image.onerror=()=>{if(version!==this.version)return;failed++;finish();};image.src=material.texture;
    });
    this.draw();
  }
  bind(buffer){const gl=this.gl;gl.bindBuffer(gl.ARRAY_BUFFER,buffer);for(const [name,size,offset] of [['p',3,0],['n',3,12],['uv',2,24],['tint',4,32]]){gl.enableVertexAttribArray(this.a[name]);gl.vertexAttribPointer(this.a[name],size,gl.FLOAT,false,48,offset);}}
  line(a,b,color){const gl=this.gl;gl.bindBuffer(gl.ARRAY_BUFFER,this.lines);gl.bufferData(gl.ARRAY_BUFFER,new Float32Array([...a,0,0,1,0,0,1,1,1,1,...b,0,0,1,0,0,1,1,1,1]),gl.DYNAMIC_DRAW);this.bind(this.lines);gl.uniform4fv(this.u.color,color);gl.drawArrays(gl.LINES,0,2);}
  draw(){
    if(!this.model)return;
    const gl=this.gl,canvas=this.canvas,ratio=Math.min(window.devicePixelRatio||1,2),width=canvas.clientWidth,height=canvas.clientHeight;
    if(!width||!height)return;
    const pw=Math.round(width*ratio),ph=Math.round(height*ratio);if(canvas.width!==pw||canvas.height!==ph){canvas.width=pw;canvas.height=ph;}
    const origin=ReorientationMath.objectOrigin(this.rotation,this.pivot),center=[0,0,0],scale=Math.max(.1,this.radius*1.55)/this.zoom,aspect=width/height;
    const [right,up,depth]=this.basis,dot=ReorientationMath.dot,sx=1/(scale*aspect),sy=1/scale,sz=-1/(this.radius*16);
    const view=new Float32Array([right[0]*sx,up[0]*sy,depth[0]*sz,0,right[1]*sx,up[1]*sy,depth[1]*sz,0,right[2]*sx,up[2]*sy,depth[2]*sz,0,-dot(right,center)*sx,-dot(up,center)*sy,-dot(depth,center)*sz,1]);
    gl.viewport(0,0,canvas.width,canvas.height);gl.clearColor(.028,.06,.085,1);gl.clear(gl.COLOR_BUFFER_BIT|gl.DEPTH_BUFFER_BIT);gl.enable(gl.DEPTH_TEST);gl.useProgram(this.program);gl.uniformMatrix4fv(this.u.view,false,view);
    gl.uniformMatrix3fv(this.u.rotation,false,ReorientationMath.column(ReorientationMath.identity()));gl.uniform3f(this.u.origin,0,0,0);gl.uniform1f(this.u.shaded,0);gl.uniform1i(this.u.textured,0);
    const grid=Math.max(.15,scale*2),step=.05;
    for(let i=-Math.ceil(grid/step);i<=Math.ceil(grid/step);i++){this.line([i*step,-grid,-this.radius*1.2],[i*step,grid,-this.radius*1.2],[.12,.20,.25,1]);this.line([-grid,i*step,-this.radius*1.2],[grid,i*step,-this.radius*1.2],[.12,.20,.25,1]);}
    gl.uniformMatrix3fv(this.u.rotation,false,ReorientationMath.column(this.rotation));gl.uniform3fv(this.u.origin,origin);gl.uniform1f(this.u.shaded,1);this.bind(this.buffer);
    for(const group of this.groups){gl.uniform4fv(this.u.color,this.materials[group.material].color);gl.uniform1i(this.u.textured,this.textures[group.material]?1:0);gl.activeTexture(gl.TEXTURE0);gl.bindTexture(gl.TEXTURE_2D,this.textures[group.material]||this.white);gl.uniform1i(this.u.image,0);gl.drawArrays(gl.TRIANGLES,group.first,group.count);}
    this.drawOverlay(width,height,origin,p=>[(dot(right,p)-dot(right,center))*sx*width/2+width/2,height/2-(dot(up,p)-dot(up,center))*sy*height/2]);
  }
  drawOverlay(width,height,offset,project){
    const ns='http://www.w3.org/2000/svg',svg=document.createElementNS(ns,'svg');svg.setAttribute('viewBox',`0 0 ${width} ${height}`);svg.setAttribute('width','100%');svg.setAttribute('height','100%');
    const add=(tag,attrs,text)=>{const el=document.createElementNS(ns,tag);for(const [k,v] of Object.entries(attrs))el.setAttribute(k,v);if(text)el.textContent=text;svg.appendChild(el);return el;};
    const colors=['#ff6b72','#69e795','#69aaff'],names=['X','Y','Z'];
    const label=(text,x,y,color,size=15)=>add('text',{x,y,fill:color,'font-size':size,'font-weight':700,'font-family':'system-ui','paint-order':'stroke','stroke':'#071017','stroke-width':4,'stroke-linejoin':'round'},text);
    const arrow=(a,b,color,dashed=false)=>{add('line',{x1:a[0],y1:a[1],x2:b[0],y2:b[1],stroke:'#071017','stroke-width':7});add('line',{x1:a[0],y1:a[1],x2:b[0],y2:b[1],stroke:color,'stroke-width':3,'stroke-dasharray':dashed?'4 5':'none','stroke-linecap':'round'});
      if(!dashed){const angle=Math.atan2(b[1]-a[1],b[0]-a[0]),ends=[-.5,.5].map(d=>[b[0]-10*Math.cos(angle+d),b[1]-10*Math.sin(angle+d)]);add('path',{d:`M${ends[0]} L${b} L${ends[1]}`,fill:'none',stroke:color,'stroke-width':3,'stroke-linecap':'round','stroke-linejoin':'round'});}};
    if(this.showAxes){
      const origin=project(offset);
      for(let i=0;i<3;i++){const axis=[0,0,0];axis[i]=.05;const vector=ReorientationMath.apply(this.rotation,axis),end=project(vector.map((v,k)=>v+offset[k])),back=project(vector.map((v,k)=>offset[k]-v*.45));arrow(origin,back,colors[i],true);arrow(origin,end,colors[i]);
        label('物品 '+names[i],Math.max(8,Math.min(width-70,end[0]+9)),Math.max(24,Math.min(height-24,end[1]-10-i*3)),colors[i]);}
      add('circle',{cx:origin[0],cy:origin[1],r:5,fill:'#fff',stroke:'#071017','stroke-width':2});label('物品原点',origin[0]+10,origin[1]+23,'#b8c6d0',12);
    }
    const pivot=project([0,0,0]);add('circle',{cx:pivot[0],cy:pivot[1],r:8,fill:'none',stroke:'#ffe19b','stroke-width':2,'stroke-dasharray':'3 2'});add('line',{x1:pivot[0]+8,y1:pivot[1]+8,x2:pivot[0]+48,y2:pivot[1]+45,stroke:'#ffe19b','stroke-width':1,'stroke-dasharray':'3 3'});label(this.model.holding?'固定 TCP':'预览中心（未抓取）',pivot[0]+50,pivot[1]+51,'#ffe19b',12);
    const small=width<450,cardW=small?112:146,cardH=small?98:115,anchor=[small?62:80,height-(small?54:65)],length=small?30:42;
    add('rect',{x:12,y:height-cardH-12,width:cardW,height:cardH,rx:9,fill:'#101c28',stroke:'#385267'});
    label('固定基座坐标',22,height-cardH+6,'#cedce6',small?10:12);
    for(let i=0;i<3;i++){const end=[anchor[0]+this.basis[0][i]*length,anchor[1]-this.basis[1][i]*length];arrow(anchor,end,colors[i]);label(names[i],end[0]+7,end[1]+4,colors[i],small?11:13);}
    label('参考网格 5 cm',22,height-22,'#9aafbf',small?10:11);
    label(this.textureState,16,26,'#cadce8',12);label(`${Math.round(this.zoom*100)}%`,width-62,26,'#cadce8',12);
    this.overlay.replaceChildren(svg);
  }
}
class ReorientationPanel{
  constructor(){
    this.key=null;this.pending=false;this.loading=false;this.generation=0;this.held=null;this.disabled=true;
    this.info=document.querySelector('#reorientation-info');this.feedback=document.querySelector('#reorientation-feedback');
    this.preview=document.querySelector('#reorient-preview');this.execute=document.querySelector('#reorient');this.restore=document.querySelector('#reorientation-restore');
    this.inputs=['x','y','z'].map(axis=>document.querySelector('#reorientation-'+axis));
    try{this.viewer=new ReorientationViewer(document.querySelector('#reorientation-view'),document.querySelector('#reorientation-overlay'));}catch(e){this.info.textContent=e.message;return;}
    for(const input of this.inputs)input.oninput=()=>{if(this.disabled)return;if(!this.anglesValid()){this.preview.disabled=true;this.execute.disabled=true;this.feedback.textContent='请填写三个有效角度，每项在 −180 到 180 度之间。';return;}this.viewer.rotation=ReorientationMath.euler(...this.inputs.map(x=>Number(x.value)));this.changed();};
    this.restore.onclick=()=>{this.viewer.rotation=this.viewer.model.initial_rotation.map(row=>row.slice());this.syncAngles();this.changed();};
    this.dragMode=document.querySelector('#reorientation-drag-mode');
    this.axes=document.querySelector('#reorientation-axes');this.axes.onchange=()=>{this.viewer.showAxes=this.axes.checked;this.viewer.draw();};
    this.viewReset=document.querySelector('#reorientation-view-reset');this.viewReset.onclick=()=>{this.viewer.zoom=1;this.viewer.draw();};
    let anchor=null;const canvas=this.viewer.canvas;
    const point=e=>{const r=canvas.getBoundingClientRect();return [e.clientX-r.left,e.clientY-r.top];};
    canvas.onpointerdown=e=>{if(this.disabled||!this.viewer.model||e.button!==0||anchor)return;e.preventDefault();canvas.focus();anchor={id:e.pointerId,point:point(e),rotation:this.viewer.rotation.map(row=>row.slice()),mode:this.dragMode.value,fine:e.shiftKey};canvas.setPointerCapture(e.pointerId);canvas.classList.add('dragging');};
    const release=()=>{anchor=null;canvas.classList.remove('dragging');};this.cancelDrag=release;
    canvas.onpointermove=e=>{if(!anchor||e.pointerId!==anchor.id)return;if(this.disabled){release();return;}const delta=ReorientationMath.drag(anchor.point,point(e),canvas.clientWidth,canvas.clientHeight,this.viewer.basis,anchor.mode,anchor.fine);this.viewer.rotation=ReorientationMath.rotate(anchor.rotation,delta,anchor.mode);this.syncAngles();this.changed();};
    canvas.onpointerup=canvas.onpointercancel=e=>{if(anchor?.id===e.pointerId)release();};canvas.onlostpointercapture=release;
    canvas.addEventListener('wheel',e=>{if(this.disabled)return;e.preventDefault();this.viewer.zoom=Math.max(.6,Math.min(2.8,this.viewer.zoom*Math.exp(-e.deltaY*.001)));this.viewer.draw();},{passive:false});
    canvas.ondblclick=()=>{if(!this.disabled){this.viewer.zoom=1;this.viewer.draw();}};
    canvas.onkeydown=e=>{if(this.disabled||!['ArrowLeft','ArrowRight','ArrowUp','ArrowDown'].includes(e.key))return;e.preventDefault();const sign=['ArrowLeft','ArrowDown'].includes(e.key)?-1:1,mode=this.dragMode.value,axis=mode==='free'?(e.key==='ArrowUp'||e.key==='ArrowDown'?this.viewer.basis[0]:this.viewer.basis[1]):[0,1,2].map(i=>i==='xyz'.indexOf(mode)?1:0);this.viewer.rotation=ReorientationMath.rotate(this.viewer.rotation,ReorientationMath.axisAngle(axis,sign*(e.shiftKey?1:5)*Math.PI/180),mode);this.syncAngles();this.changed();};
    this.preview.onclick=()=>this.request(true);this.execute.onclick=()=>this.request(false);
  }
  syncAngles(){ReorientationMath.angles(this.viewer.rotation).forEach((v,k)=>{this.inputs[k].value=v.toFixed(1);});}
  anglesValid(){return this.inputs.every(x=>x.value!==''&&x.checkValidity());}
  changed(){this.poseDirty=true;this.viewer.draw();this.feedback.textContent='朝向已调整；点击“预览姿态方案”检查此朝向能否到达。';}
  async load(id,key){
    this.key=key;this.loading=true;const generation=++this.generation;this.info.textContent='正在加载物品模型…';
    try{const response=await fetch('/api/reorient/model?object_id='+encodeURIComponent(id)),model=await response.json();if(!response.ok)throw Error(model.error);if(generation!==this.generation)return;
      this.viewer.setModel(model);this.poseDirty=false;this.syncAngles();this.info.textContent='模型：'+model.object_name+'。'+(model.holding?'金色标记为固定 TCP，物品围绕它旋转。':'尚未抓取，暂以模型原点预览；抓取后使用真实夹持中心。')+'左下角为固定基座坐标。'+(model.appearance?.error?' 纹理不可用：'+model.appearance.error:'');
      this.feedback.textContent=model.holding?'已关联本次抓取，可以预览或调整姿态。':'可先旋转查看；成功抓取后加载该次抓取朝向，才能执行姿态调整。';
    }catch(e){if(generation===this.generation){this.viewer.model=null;this.info.textContent='模型加载失败：'+e.message;}}
    finally{if(generation===this.generation)this.loading=false;}
  }
  update(state,otherPending){
    if(!this.viewer)return;
    const g=state.grasp||{};this.held=g.held_object;this.motionEnabled=g.motion_enabled;
    const id=this.held?.object_id||state.object_id,key=[id,this.held?.hold_id||'',this.held?.model_sha256||state.model_sha256,this.held?.orientation_version||0].join(':');
    if(id&&key!==this.key)this.load(id,key);
    this.disabled=!!g.busy||otherPending||this.pending||this.loading||!this.viewer.model;
    if(this.disabled)this.cancelDrag();this.dragMode.disabled=this.disabled;this.viewReset.disabled=this.disabled;this.viewer.canvas.setAttribute('aria-disabled',String(this.disabled));
    this.inputs.forEach(x=>x.disabled=this.disabled);this.restore.disabled=this.disabled;
    const ready=!this.disabled&&this.anglesValid()&&this.held&&this.viewer.model?.object_id===this.held.object_id&&this.viewer.model?.model_sha256===this.held.model_sha256;
    this.preview.disabled=!ready;this.execute.disabled=!ready||!g.motion_enabled;
    if(g.operation==='reorient'&&g.busy)this.feedback.textContent=g.message;
    if(g.operation==='reorient'&&!g.busy&&!this.poseDirty&&g.reorientation_plan&&this.planShown!==g.reorientation_plan.hold_id+g.reorientation_plan.base_T_object_target.flat().join(',')){
      const p=g.reorientation_plan;this.planShown=p.hold_id+p.base_T_object_target.flat().join(',');
      this.feedback.textContent=g.message+'\n固定末端位置（基座 X / Y / Z）：'+p.fixed_tcp_position_m.map(v=>(v*100).toFixed(1)).join(' / ')+' 厘米。';
    }else if(g.operation==='reorient'&&!g.busy&&!this.poseDirty&&g.error)this.feedback.textContent=g.error;
  }
  async request(preview){
    if(this.pending||this.disabled||!this.held||!this.anglesValid())return;
    this.pending=true;this.preview.disabled=true;this.execute.disabled=true;
    try{await post(preview?'/api/reorient/preview':'/api/reorient',{object_id:this.held.object_id,hold_id:this.held.hold_id,model_sha256:this.viewer.model.model_sha256,rotation:this.viewer.rotation});this.poseDirty=false;this.feedback.textContent='正在检查固定末端位置下的目标姿态…';this.planShown=null;}
    catch(e){this.feedback.textContent=e.message;}
    finally{this.pending=false;}
  }
}
