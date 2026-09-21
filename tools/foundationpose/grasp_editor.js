/* Editing is object-relative; camera orbit never changes the saved grasp pose. */
'use strict';
const GraspEditorMath={
  add:(a,b)=>a.map((v,i)=>v+b[i]),sub:(a,b)=>a.map((v,i)=>v-b[i]),scale:(a,n)=>a.map(v=>v*n),
  segmentDistance(p,a,b){const d=this.sub(b,a),n=ReorientationMath.dot(d,d),t=n?Math.max(0,Math.min(1,ReorientationMath.dot(this.sub(p,a),d)/n)):0;return Math.hypot(...this.sub(p,this.add(a,this.scale(d,t))));},
  ray(pixel,width,height,basis,scale,pivot){
    const x=(pixel[0]-width/2)*2*scale/height,y=(height/2-pixel[1])*2*scale/height;
    return {origin:this.add(pivot,this.add(this.scale(basis[0],x),this.add(this.scale(basis[1],y),this.scale(basis[2],scale*20)))),direction:this.scale(basis[2],-1)};
  },
  plane(ray,center,normal){const d=ReorientationMath.dot(ray.direction,normal);if(Math.abs(d)<.08)return null;return this.add(ray.origin,this.scale(ray.direction,ReorientationMath.dot(this.sub(center,ray.origin),normal)/d));},
  surface(ray,vertices,faces){
    let nearest=Infinity,hit=null;const M=ReorientationMath;
    for(const face of faces){const a=vertices[face[0]],e1=this.sub(vertices[face[1]],a),e2=this.sub(vertices[face[2]],a),p=M.cross(ray.direction,e2),det=M.dot(e1,p);if(Math.abs(det)<1e-10)continue;
      const t=this.sub(ray.origin,a),u=M.dot(t,p)/det;if(u<0||u>1)continue;const q=M.cross(t,e1),v=M.dot(ray.direction,q)/det;if(v<0||u+v>1)continue;
      const distance=M.dot(e2,q)/det;if(distance>=0&&distance<nearest){nearest=distance;hit=this.add(ray.origin,this.scale(ray.direction,distance));}}
    return hit;
  },
  rotationDrag(rotation,axis,startRay,endRay,point){const M=ReorientationMath,n=rotation.map(row=>row[axis]),a=this.plane(startRay,point,n),b=this.plane(endRay,point,n);if(!a||!b)return null;
    const x=this.sub(a,point),y=this.sub(b,point);if(Math.hypot(...x)<1e-6||Math.hypot(...y)<1e-6)return null;
    return Math.atan2(M.dot(n,M.cross(x,y)),M.dot(x,y));}
};

class GraspEditorViewer extends ReorientationViewer{
  setGrasp(profile,distance,mode){this.grasp=profile;this.distance=distance;this.editMode=mode;this.draw();}
  ray(point){const scale=Math.max(.1,this.radius*1.55)/this.zoom;return GraspEditorMath.ray(point,this.canvas.clientWidth,this.canvas.clientHeight,this.basis,scale,this.pivot);}
  drawOverlay(width,height,offset,project){
    const ns='http://www.w3.org/2000/svg',svg=document.createElementNS(ns,'svg'),M=ReorientationMath,V=GraspEditorMath;
    svg.setAttribute('viewBox',`0 0 ${width} ${height}`);svg.setAttribute('width','100%');svg.setAttribute('height','100%');
    const add=(tag,attrs,text)=>{const e=document.createElementNS(ns,tag);for(const [k,v] of Object.entries(attrs))e.setAttribute(k,v);if(text)e.textContent=text;svg.appendChild(e);return e;};
    const label=(text,p,color='#d7e7f3',size=12)=>add('text',{x:p[0],y:p[1],fill:color,'font-size':size,'font-family':'system-ui','font-weight':600,'paint-order':'stroke',stroke:'#071017','stroke-width':3},text);
    const colors=['#ff6b72','#69e795','#69aaff'];
    const arrow=(a,b,color,dashed=false)=>{add('line',{x1:a[0],y1:a[1],x2:b[0],y2:b[1],stroke:'#071017','stroke-width':7});add('line',{x1:a[0],y1:a[1],x2:b[0],y2:b[1],stroke:color,'stroke-width':3,'stroke-dasharray':dashed?'5 4':'none'});if(!dashed){const angle=Math.atan2(b[1]-a[1],b[0]-a[0]),ends=[-.5,.5].map(d=>[b[0]-10*Math.cos(angle+d),b[1]-10*Math.sin(angle+d)]);add('path',{d:`M${ends[0]} L${b} L${ends[1]}`,fill:'none',stroke:color,'stroke-width':3});}};
    this.handles=[];
    if(this.grasp){
      const point=this.grasp.grasp_point_object_m,R=this.grasp.object_R_tcp,origin=project(V.add(point,offset)),length=Math.max(.025,Math.min(.07,this.radius*.7));
      for(let axis=0;axis<3;axis++){
        const vector=R.map(row=>row[axis]),end=project(V.add(V.add(point,offset),V.scale(vector,length)));
        if(this.editMode==='rotate'){
          const u=R.map(row=>row[(axis+1)%3]),v=R.map(row=>row[(axis+2)%3]),ring=[];
          for(let k=0;k<=64;k++){const theta=k*Math.PI/32;ring.push(project(V.add(V.add(point,offset),V.add(V.scale(u,Math.cos(theta)*length),V.scale(v,Math.sin(theta)*length)))));}
          add('polyline',{points:ring.map(p=>p.join(',')).join(' '),fill:'none',stroke:colors[axis],'stroke-width':3});this.handles.push({axis,ring});label('旋转 '+'XYZ'[axis],ring[8+axis*5],colors[axis]);
        }else{arrow(origin,end,colors[axis]);label('TCP '+'XYZ'[axis],[end[0]+6,end[1]-5],colors[axis]);this.handles.push({axis,a:origin,b:end,length,vector});}
      }
      const pre=V.sub(point,V.scale(R.map(row=>row[2]),this.distance)),prePixel=project(V.add(pre,offset));arrow(prePixel,origin,'#ffd27c',true);
      add('circle',{cx:origin[0],cy:origin[1],r:5,fill:'#fff',stroke:'#071017','stroke-width':2});label('抓取中心',[origin[0]+9,origin[1]+19]);
      add('circle',{cx:prePixel[0],cy:prePixel[1],r:5,fill:'none',stroke:'#ffd27c','stroke-width':2});label('预抓点 → 工具 +Z',[prePixel[0]+8,prePixel[1]-8],'#ffd27c');
    }
    const small=width<450,w=small?112:146,h=small?98:115,anchor=[small?62:80,height-(small?54:65)],length=small?30:42;
    if(this.viewControl){this.viewControl.style.width=w+'px';this.viewControl.style.height=h+'px';}
    add('rect',{x:12,y:height-h-12,width:w,height:h,rx:9,fill:'#101c28',stroke:'#385267'});label('物品坐标 · 视角',[22,height-h+6],'#cedce6',small?10:12);
    for(let i=0;i<3;i++){const end=[anchor[0]+this.basis[0][i]*length,anchor[1]-this.basis[1][i]*length];arrow(anchor,end,colors[i]);label('XYZ'[i],[end[0]+6,end[1]+4],colors[i]);}
    label('拖动旋转视角',[22,height-22],'#9aafbf',11);label(this.textureState||'物品模型',[14,24]);label(`${Math.round(this.zoom*100)}%`,[width-58,24]);
    this.overlay.replaceChildren(svg);
  }
}

class GraspEditorPanel{
  constructor(send){
    this.send=send;this.pending=false;this.loading=false;this.locked=true;this.objectId=null;this.generation=0;this.dirty=false;this.activeId=null;
    const q=id=>document.querySelector('#grasp-editor-'+id);this.q=q;this.feedback=q('feedback');this.list=q('list');
    this.pointInputs=['px','py','pz'].map(q);this.angleInputs=['rx','ry','rz'].map(q);
    this.fields=[q('name'),q('enabled'),q('priority'),...this.pointInputs,...this.angleInputs,q('distance'),q('opening'),q('force')];
    try{this.viewer=new GraspEditorViewer(q('view'),q('overlay'),q('camera'));}catch(error){this.feedback.textContent=error.message;return;}
    this.mode=q('mode');this.mode.onchange=()=>this.redraw();
    for(const field of this.fields)field.oninput=()=>this.readFields();
    this.list.onchange=()=>this.selectEntry(this.list.value);
    q('new').onclick=()=>this.newEntry();q('save').onclick=()=>this.save();q('reload').onclick=()=>this.load(this.objectId);
    q('duplicate').onclick=()=>this.mutate('duplicate');q('delete').onclick=()=>this.mutate('delete');
    q('use').onclick=()=>this.mutate('select',{selection_mode:'manual'});q('auto').onclick=()=>this.mutate('select',{selection_mode:'auto'});
    q('preview').onclick=async()=>{if(this.pending||this.locked||this.dirty)return;this.pending=true;try{await this.send('/api/grasp/preview');this.feedback.textContent='按已保存的选择检查当前跟踪物品；不会发送动作。';}catch(error){this.feedback.textContent=error.message;}finally{this.pending=false;}};
    this.bindGestures();
  }
  async load(id){
    if(!id)return;const generation=++this.generation;this.objectId=id;this.loading=true;this.feedback.textContent='加载物品夹法…';
    try{
      const fetchJSON=async path=>{const r=await fetch(path);const data=await r.json();if(!r.ok)throw Error(data.error);return data;};
      const [data,model]=await Promise.all([fetchJSON('/api/grasp/profiles?object_id='+encodeURIComponent(id)),fetchJSON('/api/grasp/editor-model?object_id='+encodeURIComponent(id))]);
      if(generation!==this.generation)return;this.data=data;this.changedAt=data.changed_at;this.model=model;this.viewer.setModel(model);this.populate();
      if(data.grasps.length)this.selectEntry(data.selected_id||data.grasps[0].id);else this.newEntry();
    }catch(error){if(generation===this.generation){this.data=null;this.feedback.textContent=error.message;}}
    finally{if(generation===this.generation)this.loading=false;}
  }
  populate(){
    this.list.replaceChildren();for(const row of this.data.grasps){const option=document.createElement('option');option.value=row.id;option.textContent=(row.enabled?'':'[停用] ')+row.name+(row.binding_error?' [绑定失效]':'')+' · 优先级 '+row.priority+(row.tested_successfully?' · 实测成功过':' · 已保存');this.list.appendChild(option);}
    const selected=this.data.grasps.find(g=>g.id===this.data.selected_id);
    this.q('selection').textContent='抓取使用：'+(this.data.selection_mode==='auto'?'自动筛选已启用夹法':selected?.name||'未选择');
    this.q('info').textContent=this.data.object_name+' · 坐标与角度均相对于物品模型';
  }
  newEntry(){
    if(!this.model||!this.data)return;this.activeId=null;
    const low=this.model.point_min_m,high=this.model.point_max_m;
    this.draft={name:'新夹法',enabled:true,priority:0,profile:{grasp_point_object_m:[0,0,0].map((v,i)=>Math.max(low[i],Math.min(high[i],v))),object_R_tcp:ReorientationMath.identity()},overrides:{}};
    this.dirty=true;this.writeFields();this.feedback.textContent='新夹法尚未保存。拖动箭头/圆环，或输入坐标和角度。';
  }
  selectEntry(id){
    const entry=this.data?.grasps.find(g=>g.id===id);if(!entry)return;
    this.activeId=id;this.list.value=id;this.draft=JSON.parse(JSON.stringify(entry));this.dirty=false;this.writeFields();
    this.feedback.textContent=entry.binding_error||'已载入 '+entry.name+'；选择列表只用于编辑，“使用选中夹法”才会改变抓取选择。';
  }
  writeFields(){
    const d=this.draft;this.q('name').value=d.name;this.q('enabled').checked=d.enabled;this.q('priority').value=d.priority;
    this.pointInputs.forEach((el,i)=>{el.value=(d.profile.grasp_point_object_m[i]*1000).toFixed(2);el.min=this.model.point_min_m[i]*1000;el.max=this.model.point_max_m[i]*1000;});
    ReorientationMath.angles(d.profile.object_R_tcp).forEach((v,i)=>this.angleInputs[i].value=v.toFixed(2));
    for(const [id,key,factor] of [['distance','pregrasp_distance_m',1000],['opening','open_position',1],['force','force_target_raw',1]]){
      this.q(id).value=d.overrides?.[key]==null?'':d.overrides[key]*factor;this.q(id).placeholder='沿用 '+this.data.defaults[key]*factor;
    }
    this.redraw();
  }
  readFields(){
    this.dirty=true;
    if(!this.draft)return;
    try{this.draft=this.formValue();this.feedback.textContent='草稿未保存；抓取仍使用已保存的夹法。';this.redraw();}
    catch(error){this.feedback.textContent=error.message;}
  }
  formValue(){
    if(!this.q('name').value.trim()||this.fields.filter(e=>e.type!=='checkbox'&&e.value!=='').some(e=>!e.checkValidity()))throw Error('请检查名称、坐标、角度和参数范围。');
    if([...this.pointInputs,...this.angleInputs,this.q('priority')].some(e=>e.value.trim()===''||!Number.isFinite(Number(e.value))))throw Error('坐标、角度和优先级不能为空。');
    const overrides={};for(const [id,key,factor] of [['distance','pregrasp_distance_m',.001],['opening','open_position',1],['force','force_target_raw',1]])if(this.q(id).value.trim()!=='')overrides[key]=Number(this.q(id).value)*factor;
    const old=this.draft?.profile,angles=old?ReorientationMath.angles(old.object_R_tcp):[];
    const point=old&&this.pointInputs.every((e,i)=>e.value===(old.grasp_point_object_m[i]*1000).toFixed(2))?old.grasp_point_object_m.slice():this.pointInputs.map(e=>Number(e.value)/1000);
    const rotation=old&&this.angleInputs.every((e,i)=>e.value===angles[i].toFixed(2))?old.object_R_tcp.map(row=>row.slice()):ReorientationMath.euler(...this.angleInputs.map(e=>Number(e.value)));
    return {name:this.q('name').value.trim(),enabled:this.q('enabled').checked,priority:Number(this.q('priority').value),profile:{grasp_point_object_m:point,object_R_tcp:rotation},overrides};
  }
  redraw(){if(this.draft&&this.viewer)this.viewer.setGrasp(this.draft.profile,this.draft.overrides?.pregrasp_distance_m??this.data.defaults.pregrasp_distance_m,this.mode.value);}
  async save(){if(this.pending||this.locked||!this.data)return;let grasp;try{grasp=this.formValue();}catch(error){this.feedback.textContent=error.message;return;}await this.mutate('save',{grasp,model_sha256:this.data.model_sha256});}
  async mutate(action,extra={}){
    if(this.pending||this.locked||!this.data)return;if(this.dirty&&['duplicate','select'].includes(action)){this.feedback.textContent='请先保存草稿或重新载入，再复制或更改抓取选择。';return;}this.pending=true;
    try{const data=await this.send('/api/grasp/profiles',{object_id:this.objectId,revision:this.data.revision,id:this.activeId,action,...extra});
      this.data=data;this.changedAt=data.changed_at;this.q('results').textContent='';this.populate();const id=action==='delete'?(data.selected_id||data.grasps[0]?.id):(data.saved_id||this.activeId);
      if(id)this.selectEntry(id);else this.newEntry();this.feedback.textContent='已保存。'+(action==='select'?'抓取选择已更新。':'本次未发送运动。');
    }catch(error){this.feedback.textContent=error.message;}finally{this.pending=false;}
  }
  update(state,otherPending=false){
    if(!this.viewer)return;const id=state.object_id,g=state.grasp||{};
    if(id&&id!==this.objectId){this.changedAt=g.grasp_library_changed_at;this.load(id);}
    if(g.grasp_library_changed_at&&g.grasp_library_changed_at!==this.changedAt&&!this.loading&&!this.pending&&!this.dirty){this.changedAt=g.grasp_library_changed_at;this.load(id);}
    if(this.data){
      const oldDistance=this.data.defaults.pregrasp_distance_m;
      if(g.pregrasp_distance_m!=null)this.data.defaults.pregrasp_distance_m=g.pregrasp_distance_m;
      if(g.force_config)this.data.defaults.force_target_raw=g.force_config.value;
      if(g.open_position_config)this.data.defaults.open_position=g.open_position_config.value;
      for(const [id,key,factor] of [['distance','pregrasp_distance_m',1000],['opening','open_position',1],['force','force_target_raw',1]])this.q(id).placeholder='沿用 '+this.data.defaults[key]*factor;
      if(oldDistance!==this.data.defaults.pregrasp_distance_m)this.redraw();
    }
    this.locked=!!g.busy||otherPending||this.pending||this.loading||!this.data;
    for(const field of [...this.fields,this.list,this.mode])field.disabled=this.locked;
    for(const id of ['new','save','reload'])this.q(id).disabled=this.locked;
    this.q('auto').disabled=this.locked||this.dirty;
    this.q('delete').disabled=this.locked||!this.activeId;this.q('duplicate').disabled=this.locked||!this.activeId||this.dirty;
    const selected=this.data?.grasps.find(e=>e.id===this.activeId);
    this.q('use').disabled=this.locked||this.dirty||!selected?.enabled||!!selected?.binding_error;
    this.q('preview').disabled=this.locked||this.dirty||state.state!=='tracking'||!state.quality?.good;
    this.q('camera').disabled=!this.viewer.model;
    const plan=g.plan;
    if(plan?.grasp_candidates)this.q('results').textContent='本次选择：'+plan.grasp_name+'\n'+plan.grasp_candidates.map(r=>r.name+'：'+(r.passed?'航点检查通过':r.reason)).join('\n');
    else if(g.operation==='grasp'&&g.error)this.q('results').textContent=g.error;
  }
  bindGestures(){
    const viewer=this.viewer,M=ReorientationMath,V=GraspEditorMath,canvas=viewer.canvas,camera=this.q('camera');let drag=null,orbit=null;
    const point=(el,e)=>{const r=el.getBoundingClientRect();return [e.clientX-r.left,e.clientY-r.top];};
    const changed=()=>{this.dirty=true;this.writeFields();this.feedback.textContent='夹法已调整，尚未保存。';};
    canvas.onpointerdown=e=>{
      if(this.locked||!this.draft||e.button!==0||drag||orbit)return;e.preventDefault();const p=point(canvas,e);
      if(this.mode.value==='pick'){const hit=V.surface(viewer.ray(p),this.model.vertices,this.model.faces);if(hit){this.draft.profile.grasp_point_object_m=hit;changed();}else this.feedback.textContent='没有点中模型表面。';return;}
      const choices=viewer.handles.map(h=>({h,d:h.ring?Math.min(...h.ring.slice(1).map((b,i)=>V.segmentDistance(p,h.ring[i],b))):V.segmentDistance(p,h.a,h.b)})).sort((a,b)=>a.d-b.d);
      if(!choices.length||choices[0].d>12){this.feedback.textContent='请拖动彩色箭头或圆环；左下角用于旋转视角。';return;}
      const h=choices[0].h;
      if(!h.ring&&Math.hypot(...V.sub(h.b,h.a))<8){this.feedback.textContent='此轴接近视线方向，请先旋转观察视角。';return;}
      drag={id:e.pointerId,p,h,profile:JSON.parse(JSON.stringify(this.draft.profile)),ray:viewer.ray(p),fine:e.shiftKey};canvas.setPointerCapture(e.pointerId);
    };
    canvas.onpointermove=e=>{
      if(!drag||e.pointerId!==drag.id)return;if(this.locked){drag=null;return;}const p=point(canvas,e),factor=drag.fine?.2:1;
      if(drag.h.ring){const angle=V.rotationDrag(drag.profile.object_R_tcp,drag.h.axis,drag.ray,viewer.ray(p),drag.profile.grasp_point_object_m);if(angle===null){this.feedback.textContent='圆环接近侧面，请先旋转视角再拖动。';return;}const axis=[0,0,0];axis[drag.h.axis]=1;this.draft.profile.object_R_tcp=M.multiply(drag.profile.object_R_tcp,M.axisAngle(axis,angle*factor));}
      else{const axis=V.sub(drag.h.b,drag.h.a),delta=M.dot(V.sub(p,drag.p),axis)/M.dot(axis,axis)*drag.h.length*factor;this.draft.profile.grasp_point_object_m=V.add(drag.profile.grasp_point_object_m,V.scale(drag.h.vector,delta));}
      changed();
    };
    canvas.onpointerup=canvas.onpointercancel=e=>{if(drag?.id===e.pointerId)drag=null;};canvas.onlostpointercapture=()=>drag=null;
    camera.onpointerdown=e=>{if(!viewer.model||e.button!==0||orbit||drag)return;e.preventDefault();const r=camera.getBoundingClientRect();orbit={id:e.pointerId,p:point(camera,e),basis:viewer.basis.map(row=>row.slice()),w:r.width,h:r.height,fine:e.shiftKey};camera.setPointerCapture(e.pointerId);};
    camera.onpointermove=e=>{if(orbit?.id===e.pointerId)viewer.orbitView(orbit.basis,orbit.p,point(camera,e),orbit.w,orbit.h,orbit.fine);};
    camera.onpointerup=camera.onpointercancel=e=>{if(orbit?.id===e.pointerId)orbit=null;};camera.onlostpointercapture=()=>orbit=null;
    camera.ondblclick=()=>{orbit=null;viewer.resetView();};camera.onkeydown=e=>{if(e.key==='Home'){e.preventDefault();viewer.resetView();}};
    canvas.addEventListener('wheel',e=>{e.preventDefault();viewer.zoom=Math.max(.5,Math.min(4,viewer.zoom*Math.exp(-e.deltaY*.001)));viewer.draw();},{passive:false});
  }
}
