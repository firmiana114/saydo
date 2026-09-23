/* API rotations are row-major. Display materials and camera zoom never alter motion geometry. */
'use strict';
const ReorientationMath={
  identity:()=>[[1,0,0],[0,1,0],[0,0,1]],
  defaultView:()=>[[Math.SQRT1_2,Math.SQRT1_2,0],[-1/Math.sqrt(6),1/Math.sqrt(6),2/Math.sqrt(6)],[1/Math.sqrt(3),-1/Math.sqrt(3),1/Math.sqrt(3)]],
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
  rotate(rotation,delta,mode='free'){return mode==='free'||mode==='base-ring'?this.multiply(delta,rotation):this.multiply(rotation,delta);},
  orbit(basis,start,end,width,height,fine=false){return this.multiply(basis,this.drag(start,end,width,height,basis,'free',fine));},
  objectOrigin(rotation,pivot,translation=[0,0,0]){return this.apply(rotation,pivot).map((v,k)=>translation[k]-v);},
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
const ReorientationRings={
  basis(rotation,mode){return mode==='base-ring'?ReorientationMath.identity():rotation;},
  ray(pixel,width,height,basis,scale,center){const x=(pixel[0]-width/2)*2*scale/height,y=(height/2-pixel[1])*2*scale/height;
    return {origin:center.map((v,i)=>v+basis[0][i]*x+basis[1][i]*y+basis[2][i]*scale*20),direction:basis[2].map(v=>-v)};},
  angle(start,end,center,normal){const M=ReorientationMath;
    const hit=ray=>{const denominator=M.dot(ray.direction,normal);if(Math.abs(denominator)<.08)return null;
      const t=M.dot(center.map((v,i)=>v-ray.origin[i]),normal)/denominator;return ray.origin.map((v,i)=>v+ray.direction[i]*t-center[i]);};
    const a=hit(start),b=hit(end);if(!a||!b||Math.hypot(...a)<1e-6||Math.hypot(...b)<1e-6)return null;
    return Math.atan2(M.dot(normal,M.cross(a,b)),M.dot(a,b));},
  distance(p,a,b){const d=b.map((v,i)=>v-a[i]),n=ReorientationMath.dot(d,d),t=n?Math.max(0,Math.min(1,ReorientationMath.dot(p.map((v,i)=>v-a[i]),d)/n)):0;
    return Math.hypot(...p.map((v,i)=>v-a[i]-t*d[i]));},
  hit(handles,p){const choices=handles.map(h=>({handle:h,distance:Math.min(...h.points.slice(1).map((b,i)=>this.distance(p,h.points[i],b)))})).sort((a,b)=>a.distance-b.distance);
    return choices[0]?.distance<=12?choices[0].handle:null;}
};
// Depth cues describe direction relative to the view, independently of axis sign.
const AxisDepthCue={
  describe(vector,depth){const n=Math.hypot(...vector)||1,value=ReorientationMath.dot(vector,depth)/n;
    const kind=value>.12?'front':value<-.12?'back':'plane';
    return {value,kind,symbol:kind==='front'?'⊙':kind==='back'?'⊗':'—',text:kind==='front'?'朝向屏幕外':kind==='back'?'朝向屏幕内':'近似平行屏幕',
      width:kind==='front'?3.5:kind==='back'?1.8:2.8,opacity:kind==='back'?.42:1,dash:kind==='back'?'5 4':'none'};
  },
  segment(add,a,b,color,cue,{arrow=true,badge=true,minor=false}={}){
    const opacity=cue.opacity*(minor?.7:1),width=cue.width*(minor?.65:1),distance=Math.hypot(b[0]-a[0],b[1]-a[1]);
    const line={x1:a[0],y1:a[1],x2:b[0],y2:b[1],'stroke-linecap':'round','stroke-dasharray':cue.dash,opacity,'data-depth':cue.kind};
    add('line',{...line,stroke:'#071017','stroke-width':width+3});add('line',{...line,stroke:color,'stroke-width':width});
    if(arrow&&distance>8){const angle=Math.atan2(b[1]-a[1],b[0]-a[0]),ends=[-.5,.5].map(d=>[b[0]-9*Math.cos(angle+d),b[1]-9*Math.sin(angle+d)]);
      add('path',{d:`M${ends[0]} L${b} L${ends[1]}`,fill:'none',stroke:color,'stroke-width':width,opacity,'stroke-linecap':'round'});}
    if(badge&&cue.kind!=='plane'){
      const r=minor?3.5:5.5;
      add('circle',{cx:b[0],cy:b[1],r,fill:'#101820',stroke:color,'stroke-width':1.6,opacity:minor?.7:1,'data-depth':cue.kind});
      if(cue.kind==='front')add('circle',{cx:b[0],cy:b[1],r:minor?1.2:1.8,fill:color});
      else for(const sign of [-1,1])add('line',{x1:b[0]-r*.5,y1:b[1]-sign*r*.5,x2:b[0]+r*.5,y2:b[1]+sign*r*.5,stroke:color,'stroke-width':1.5});
    }
  }
};
class ReorientationViewer{
  constructor(canvas,overlay,viewControl=null){
    this.canvas=canvas;this.overlay=overlay;this.translation=[0,0,0];this.rotation=ReorientationMath.identity();this.zoom=1;this.version=0;this.textures=[];this.showAxes=true;
    this.basis=ReorientationMath.defaultView();this.viewControl=viewControl;
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
  resetView(){this.basis=ReorientationMath.defaultView();this.zoom=1;this.draw();}
  orbitView(basis,start,end,width,height,fine=false){this.basis=ReorientationMath.orbit(basis,start,end,width,height,fine);this.draw();}
  ringRay(point){const center=this.translation.map(v=>v/2),scale=Math.max(.1,this.radius*1.55+Math.hypot(...this.translation)/2)/this.zoom;
    return ReorientationRings.ray(point,this.canvas.clientWidth,this.canvas.clientHeight,this.basis,scale,center);}
  setModel(model){
    const gl=this.gl,version=++this.version;for(const texture of this.textures)if(texture)gl.deleteTexture(texture);this.textures=[];
    this.model=model;this.translation=[0,0,0];this.zoom=1;this.pivot=model.pivot_object_m||[0,0,0];this.radius=model.vertices.reduce((r,p)=>Math.max(r,Math.hypot(...p.map((v,k)=>v-this.pivot[k]))),.04);
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
    const origin=ReorientationMath.objectOrigin(this.rotation,this.pivot,this.translation),center=this.translation.map(v=>v/2),scale=Math.max(.1,this.radius*1.55+Math.hypot(...this.translation)/2)/this.zoom,aspect=width/height;
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
    this.ringHandles=[];
    if(this.dragMode==='object-ring'||this.dragMode==='base-ring'){
      const frame=ReorientationRings.basis(this.rotation,this.dragMode),length=Math.max(.035,Math.min(.09,this.radius*.8)),strokes=[],labels=[];
      for(let axis=0;axis<3;axis++){
        const normal=frame.map(row=>row[axis]),u=frame.map(row=>row[(axis+1)%3]),v=frame.map(row=>row[(axis+2)%3]),points=[],vectors=[];
        for(let k=0;k<=64;k++){const theta=k*Math.PI/32,relative=u.map((value,i)=>(value*Math.cos(theta)+v[i]*Math.sin(theta))*length);vectors.push(relative);points.push(project(relative.map((value,i)=>value+this.translation[i])));}
        this.ringHandles.push({axis,normal,points});
        for(let k=0;k<64;k++){const middle=vectors[k].map((value,i)=>(value+vectors[k+1][i])/2),cue=AxisDepthCue.describe(middle,this.basis[2]);strokes.push({axis,k,a:points[k],b:points[k+1],cue});}
        const p=points[8+axis*5];labels.push({text:'绕'+(this.dragMode==='base-ring'?'基座':'物品')+' '+names[axis],p:[Math.max(8,Math.min(width-90,p[0])),Math.max(66,Math.min(height-22,p[1]))],axis});
      }
      for(const {axis,k,a,b,cue} of strokes.sort((a,b)=>a.cue.value-b.cue.value)){
        if(cue.kind==='back'&&k%2)continue;
        const attrs={x1:a[0],y1:a[1],x2:b[0],y2:b[1],opacity:cue.opacity,'stroke-dasharray':cue.dash};
        add('line',{...attrs,stroke:'#071017','stroke-width':cue.width+2});add('line',{...attrs,stroke:colors[axis],'stroke-width':cue.width,'data-ring-axis':names[axis]});
      }
      for(const item of labels)label(item.text,...item.p,colors[item.axis],12);
    }
    if(this.showAxes){
      const origin=project(offset);
      const axes=[];
      for(let i=0;i<3;i++)for(const sign of [-1,1]){const axis=[0,0,0];axis[i]=.05*sign*(sign<0?.45:1);const vector=ReorientationMath.apply(this.rotation,axis);
        axes.push({i,sign,vector,end:project(vector.map((v,k)=>v+offset[k])),cue:AxisDepthCue.describe(vector,this.basis[2])});}
      axes.sort((a,b)=>a.cue.value-b.cue.value);
      for(const {i,sign,end,cue} of axes){const minor=sign<0;
        AxisDepthCue.segment(add,origin,end,colors[i],cue,{minor,arrow:!minor,badge:!minor});
        if(minor&&Math.hypot(end[0]-origin[0],end[1]-origin[1])<16)continue;
        const text=(minor?'−':'物品 +')+names[i]+' '+cue.symbol;
        const left=end[0]<origin[0],x=Math.max(8,Math.min(width-(minor?45:95),end[0]+(left?-(minor?35:80):10)));
        label(text,x,Math.max(60,Math.min(height-24,end[1]+(minor?14:-12)-i*2)),colors[i],minor?10:13);
      }
      add('circle',{cx:origin[0],cy:origin[1],r:5,fill:'#fff',stroke:'#071017','stroke-width':2});label('物品原点',origin[0]+10,origin[1]+23,'#b8c6d0',12);
    }
    const pivot=project(this.translation);if(Math.hypot(...this.translation)>1e-6){const from=project([0,0,0]);arrow(from,pivot,'#ffe19b',true);add('circle',{cx:from[0],cy:from[1],r:5,fill:'none',stroke:'#b8c6d0'});label('起点',from[0]-25,from[1]-12,'#b8c6d0',12);}add('circle',{cx:pivot[0],cy:pivot[1],r:8,fill:'none',stroke:'#ffe19b','stroke-width':2,'stroke-dasharray':'3 2'});add('line',{x1:pivot[0]+8,y1:pivot[1]+8,x2:pivot[0]+48,y2:pivot[1]+45,stroke:'#ffe19b','stroke-width':1,'stroke-dasharray':'3 3'});label(this.model.holding?(Math.hypot(...this.translation)>1e-6?'目标 TCP':'目标 TCP'):'预览中心（未抓取）',pivot[0]+50,pivot[1]+51,'#ffe19b',12);
    const small=width<450,cardW=small?112:146,cardH=small?98:115,anchor=[small?62:80,height-(small?54:65)],length=small?30:42;
    if(this.viewControl){this.viewControl.style.width=cardW+'px';this.viewControl.style.height=cardH+'px';}
    add('rect',{x:12,y:height-cardH-12,width:cardW,height:cardH,rx:9,fill:'#101c28',stroke:'#385267'});
    label('固定基座坐标',22,height-cardH+6,'#cedce6',small?10:12);
    for(const i of [0,1,2].sort((a,b)=>this.basis[2][a]-this.basis[2][b])){const vector=[0,0,0];vector[i]=1;const cue=AxisDepthCue.describe(vector,this.basis[2]);
      const end=[anchor[0]+this.basis[0][i]*length,anchor[1]-this.basis[1][i]*length];AxisDepthCue.segment(add,anchor,end,colors[i],cue);label('+'+names[i],end[0]+8,end[1]+4,colors[i],small?10:12);}

    label(small?'拖动旋转视角':'拖动视角 · 网格 5 cm',22,height-22,'#9aafbf',small?10:11);
    label('⊙ 朝向你　⊗ 向屏幕内　— 近似平行',16,46,'#acbdca',11);
    label(this.textureState,16,26,'#cadce8',12);label(`${Math.round(this.zoom*100)}%`,width-62,26,'#cadce8',12);
    this.overlay.replaceChildren(svg);
  }
}
class ReorientationPanel{
  constructor(){
    this.key=null;this.pending=false;this.loading=false;this.generation=0;this.held=null;this.disabled=true;
    this.info=document.querySelector('#reorientation-info');this.feedback=document.querySelector('#reorientation-feedback');
    this.preview=document.querySelector('#reorient-preview');this.execute=document.querySelector('#reorient');this.restore=document.querySelector('#reorientation-restore');
    this.positionMode=document.querySelector('#reorientation-position-mode');
    this.inputs=['x','y','z'].map(axis=>document.querySelector('#reorientation-'+axis));
    this.cameraControl=document.querySelector('#reorientation-camera-control');
    try{this.viewer=new ReorientationViewer(document.querySelector('#reorientation-view'),document.querySelector('#reorientation-overlay'),this.cameraControl);}catch(e){this.info.textContent=e.message;return;}
    for(const input of this.inputs)input.oninput=()=>{if(this.disabled)return;if(!this.anglesValid()){this.preview.disabled=true;this.execute.disabled=true;this.feedback.textContent='请填写三个有效角度，每项在 −180 到 180 度之间。';return;}this.viewer.rotation=ReorientationMath.euler(...this.inputs.map(x=>Number(x.value)));this.changed();};
    this.restore.onclick=()=>{this.viewer.rotation=this.viewer.model.initial_rotation.map(row=>row.slice());this.syncAngles();this.changed();};
    this.positionMode.onchange=()=>{this.changed();};
    this.dragMode=document.querySelector('#reorientation-drag-mode');
    this.viewer.dragMode=this.dragMode.value;this.selectedRingAxis=2;
    this.axes=document.querySelector('#reorientation-axes');this.axes.onchange=()=>{this.viewer.showAxes=this.axes.checked;this.viewer.draw();};
    this.viewReset=document.querySelector('#reorientation-view-reset');
    let anchor=null,viewAnchor=null;const canvas=this.viewer.canvas;
    const point=e=>{const r=canvas.getBoundingClientRect();return [e.clientX-r.left,e.clientY-r.top];};
    canvas.onpointerdown=e=>{
      if(this.disabled||!this.viewer.model||e.button!==0||anchor||viewAnchor)return;e.preventDefault();canvas.focus();const p=point(e),mode=this.dragMode.value;
      const ring=mode==='free'?null:ReorientationRings.hit(this.viewer.ringHandles||[],p);
      if(mode!=='free'&&!ring){this.feedback.textContent='请拖动彩色圆环；左下角坐标卡片用于旋转观察视角。';return;}
      if(ring)this.selectedRingAxis=ring.axis;
      anchor={id:e.pointerId,point:p,rotation:this.viewer.rotation.map(row=>row.slice()),mode,fine:e.shiftKey,ring,
        ray:ring?this.viewer.ringRay(p):null,center:this.viewer.translation.slice(),lastAngle:0,totalAngle:0,edited:false};
      canvas.setPointerCapture(e.pointerId);canvas.classList.add('dragging');
    };
    const release=()=>{const reset=anchor?.ring&&anchor.edited;anchor=null;canvas.classList.remove('dragging');if(reset){this.viewer.translation=[0,0,0];this.viewer.draw();}};this.cancelDrag=release;
    this.dragMode.onchange=()=>{release();this.viewer.dragMode=this.dragMode.value;this.viewer.draw();};
    canvas.onpointermove=e=>{
      if(!anchor||e.pointerId!==anchor.id)return;if(this.disabled){release();return;}let delta;
      if(anchor.ring){
        const angle=ReorientationRings.angle(anchor.ray,this.viewer.ringRay(point(e)),anchor.center,anchor.ring.normal);
        if(angle===null){this.feedback.textContent='圆环接近侧面，请先旋转观察视角再拖动。';return;}
        const step=Math.atan2(Math.sin(angle-anchor.lastAngle),Math.cos(angle-anchor.lastAngle));anchor.totalAngle+=step;anchor.lastAngle=angle;
        const axis=[0,0,0];axis[anchor.ring.axis]=1;delta=ReorientationMath.axisAngle(axis,anchor.totalAngle*(anchor.fine?.2:1));
      }else delta=ReorientationMath.drag(anchor.point,point(e),canvas.clientWidth,canvas.clientHeight,this.viewer.basis,'free',anchor.fine);
      this.viewer.rotation=ReorientationMath.rotate(anchor.rotation,delta,anchor.mode);anchor.edited=true;this.syncAngles();this.changed(!!anchor.ring);
    };
    canvas.onpointerup=canvas.onpointercancel=e=>{if(anchor?.id===e.pointerId)release();};canvas.onlostpointercapture=release;
    canvas.addEventListener('wheel',e=>{if(this.disabled)return;e.preventDefault();this.viewer.zoom=Math.max(.6,Math.min(2.8,this.viewer.zoom*Math.exp(-e.deltaY*.001)));this.viewer.draw();},{passive:false});
    canvas.ondblclick=()=>{if(!this.disabled){this.viewer.zoom=1;this.viewer.draw();}};
    canvas.onkeydown=e=>{if(this.disabled||!['ArrowLeft','ArrowRight','ArrowUp','ArrowDown'].includes(e.key))return;e.preventDefault();release();const sign=['ArrowLeft','ArrowDown'].includes(e.key)?-1:1,mode=this.dragMode.value,axis=mode==='free'?(e.key==='ArrowUp'||e.key==='ArrowDown'?this.viewer.basis[0]:this.viewer.basis[1]):[0,1,2].map(i=>i===this.selectedRingAxis?1:0);this.viewer.rotation=ReorientationMath.rotate(this.viewer.rotation,ReorientationMath.axisAngle(axis,sign*(e.shiftKey?1:5)*Math.PI/180),mode);this.syncAngles();this.changed();};
    const camera=this.cameraControl;
    const viewPoint=e=>{const r=camera.getBoundingClientRect();return [e.clientX-r.left,e.clientY-r.top];};
    const releaseView=()=>{const id=viewAnchor?.id;viewAnchor=null;camera.classList.remove('dragging');if(id!==undefined&&camera.hasPointerCapture(id))camera.releasePointerCapture(id);};
    camera.onpointerdown=e=>{
      if(!this.viewer.model||e.button!==0||viewAnchor||anchor)return;
      e.preventDefault();e.stopPropagation();camera.focus();const r=camera.getBoundingClientRect();
      viewAnchor={id:e.pointerId,point:viewPoint(e),basis:this.viewer.basis.map(row=>row.slice()),width:r.width,height:r.height,fine:e.shiftKey};
      camera.setPointerCapture(e.pointerId);camera.classList.add('dragging');
    };
    camera.onpointermove=e=>{if(!viewAnchor||e.pointerId!==viewAnchor.id)return;e.preventDefault();this.viewer.orbitView(viewAnchor.basis,viewAnchor.point,viewPoint(e),viewAnchor.width,viewAnchor.height,viewAnchor.fine);};
    camera.onpointerup=camera.onpointercancel=e=>{if(viewAnchor?.id===e.pointerId)releaseView();};camera.onlostpointercapture=releaseView;
    const resetView=()=>{releaseView();this.viewer.resetView();};
    camera.ondblclick=e=>{e.preventDefault();resetView();};this.viewReset.onclick=resetView;
    camera.onkeydown=e=>{
      if(!this.viewer.model)return;
      if(e.key==='Home'){e.preventDefault();resetView();return;}
      if(!['ArrowLeft','ArrowRight','ArrowUp','ArrowDown'].includes(e.key))return;
      e.preventDefault();e.stopPropagation();releaseView();
      const sign=['ArrowLeft','ArrowDown'].includes(e.key)?-1:1,axis=['ArrowUp','ArrowDown'].includes(e.key)?this.viewer.basis[0]:this.viewer.basis[1];
      const delta=ReorientationMath.axisAngle(axis,sign*(e.shiftKey?1:5)*Math.PI/180);
      this.viewer.basis=ReorientationMath.multiply(this.viewer.basis,delta);this.viewer.draw();
    };
    this.preview.onclick=()=>this.request(true);this.execute.onclick=()=>this.request(false);
  }
  syncAngles(){ReorientationMath.angles(this.viewer.rotation).forEach((v,k)=>{this.inputs[k].value=v.toFixed(1);});}
  anglesValid(){return this.inputs.every(x=>x.value!==''&&x.checkValidity());}
  changed(keepPreviewPosition=false){this.poseDirty=true;if(!keepPreviewPosition)this.viewer.translation=[0,0,0];this.viewer.draw();this.feedback.textContent='目标已调整；预览可查看位移和终点离桌高度，执行时重新检查。';}
  async load(id,key){
    this.key=key;this.loading=true;const generation=++this.generation;this.info.textContent='正在加载物品模型…';
    try{const response=await fetch('/api/reorient/model?object_id='+encodeURIComponent(id)),model=await response.json();if(!response.ok)throw Error(model.error);if(generation!==this.generation)return;
      this.viewer.setModel(model);this.poseDirty=false;this.syncAngles();this.info.textContent='模型：'+model.object_name+'。'+(model.holding?'金色标记为目标 TCP；预览仅显示目标位姿和端点位移。':'尚未抓取，暂以模型原点预览；抓取后使用真实夹持中心。')+'拖动左下角基座坐标可旋转观察视角，不改变目标姿态。'+(model.appearance?.error?' 纹理不可用：'+model.appearance.error:'');
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
    if(this.disabled)this.cancelDrag();this.dragMode.disabled=this.disabled;this.viewReset.disabled=!this.viewer.model;this.cameraControl.disabled=!this.viewer.model;this.viewer.canvas.setAttribute('aria-disabled',String(this.disabled));
    this.positionMode.disabled=this.disabled;
    this.inputs.forEach(x=>x.disabled=this.disabled);this.restore.disabled=this.disabled;
    const ready=!this.disabled&&this.anglesValid()&&this.held&&this.viewer.model?.object_id===this.held.object_id&&this.viewer.model?.model_sha256===this.held.model_sha256;
    this.preview.disabled=!ready;this.execute.disabled=!ready||!g.motion_enabled;
    if(g.operation==='reorient'&&g.busy)this.feedback.textContent=g.message;
    if(g.operation==='reorient'&&!g.busy&&!this.poseDirty&&g.reorientation_plan&&this.planShown!==g.reorientation_plan.hold_id+g.reorientation_plan.base_T_object_target.flat().join(',')){
      const p=g.reorientation_plan;this.planShown=p.hold_id+p.base_T_object_target.flat().join(',');
      this.viewer.translation=p.offset_m||[0,0,0];this.viewer.draw();
      const offset=(p.offset_m||[0,0,0]).map(v=>(v>=0?'+':'')+(v*100).toFixed(1)).join(' / ');
      this.feedback.textContent=g.message+'\n移位 ΔX / ΔY / ΔZ：'+offset+' 厘米（基座方向）\n最终末端 X / Y / Z：'+p.target_tcp_position_m.map(v=>(v*100).toFixed(1)).join(' / ')+' 厘米。'+(p.position_mode==='adaptive'?'\n终点末端离桌高度：'+((p.target_tcp_position_m[2]-p.table_z_m)*100).toFixed(1)+' 厘米。':'');
    }else if(g.operation==='reorient'&&!g.busy&&!this.poseDirty&&g.error)this.feedback.textContent=g.message||g.error;
  }
  async request(preview){
    if(this.pending||this.disabled||!this.held||!this.anglesValid())return;
    this.pending=true;this.preview.disabled=true;this.execute.disabled=true;
    try{await post(preview?'/api/reorient/preview':'/api/reorient',{object_id:this.held.object_id,hold_id:this.held.hold_id,orientation_version:this.held.orientation_version,model_sha256:this.viewer.model.model_sha256,rotation:this.viewer.rotation,position_mode:this.positionMode.value});this.poseDirty=false;this.feedback.textContent='正在按所选位置模式检查目标姿态与离桌条件…';this.planShown=null;}
    catch(e){this.feedback.textContent=e.message;}
    finally{this.pending=false;}
  }
}
