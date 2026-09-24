'use strict';
const fs=require('fs'),vm=require('vm'),assert=require('assert');
function element(tag){return {tag,children:[],style:{},append(...items){this.children.push(...items);},appendChild(item){this.children.push(item);},setAttribute(){},showModal(){},close(){}};}
const fixed=new Map(),posts=[],row={id:'cup',name:'纸杯',available:true,pose_mode:'6d',pose_mode_revision:'pose',detection_prompt:'',effective_detection_prompt:'纸杯',detection_prompt_revision:'old'};
const document={querySelector(id){if(!fixed.has(id))fixed.set(id,element(id));return fixed.get(id);},createElement:element,createTextNode:t=>t};
const context=vm.createContext({document,fetch:async()=>({json:async()=>({objects:[row]})}),message(){},post:async(path,body)=>{posts.push({path,body});if(body.prompt==='conflict')throw Error('版本冲突');return {object:{...row,detection_prompt:body.prompt,effective_detection_prompt:body.prompt||row.name,detection_prompt_revision:'new'}};}});
const html=fs.readFileSync(__dirname+'/live_tracker.html','utf8');
vm.runInContext(html.slice(html.indexOf('const chooser='),html.indexOf('let openingGripper=')),context);
(async()=>{
 await fixed.get('#objects-open').onclick();
 const children=fixed.get('#object-list').children[0].children;
 const prompt=children[2].children[0],save=children[3],status=children[4];
 assert.equal(prompt.placeholder,'纸杯');assert.equal(prompt.maxLength,2000);
 prompt.value='白色纸杯，不要带把手的马克杯';await save.onclick();
 assert.equal(posts[0].path,'/api/object/detection-prompt');assert.equal(posts[0].body.revision,'old');assert.equal(posts[0].body.prompt,prompt.value);assert(status.textContent.includes('已保存'));
 prompt.value='conflict';await save.onclick();assert.equal(posts[1].body.revision,'new');assert.equal(status.textContent,'版本冲突');assert.equal(save.disabled,false);
 prompt.value='';await save.onclick();assert.equal(row.effective_detection_prompt,'纸杯');
 console.log('Detection prompt UI: save, revision refresh, conflict feedback and empty-name fallback passed.');
})().catch(e=>{console.error(e);process.exitCode=1;});
