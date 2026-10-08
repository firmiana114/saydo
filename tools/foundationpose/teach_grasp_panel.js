/* Teaching drafts only become per-grasp settings when explicitly recorded. */
(function(global){
  class TeachGraspPanel {
    constructor(send,root=document){
      this.send=send;this.pending=false;this.objectId=null;this.state=null;
      this.button=root.querySelector('#teach-grasp');this.force=root.querySelector('#teach-force');
      this.opening=root.querySelector('#teach-open-position');this.point=root.querySelector('#teach-point');
      this.feedback=root.querySelector('#teach-feedback');
      for(const input of [this.force,this.opening])input.oninput=()=>{this.feedback.textContent='这些值将在记录时保存到新夹法，不会立即下发夹爪。';};
      this.button.onclick=()=>this.record();
    }
    update(state,otherPending=false){
      this.state=state;const id=state.object_id,g=state.grasp||{};
      const defaults=g.grasp_configuration?.teaching_defaults;
      const modeChanged=state.pose_mode!==this.poseMode;this.poseMode=state.pose_mode;
      if(id!==this.objectId){
        this.objectId=id;this.force.value=defaults?.force_target_raw??'';this.opening.value=defaults?.open_position??'';
        this.feedback.textContent=id?'参数随新夹法保存；记录不发送运动。':'先选择物品。';
      }
      if(modeChanged&&id)this.feedback.textContent=state.pose_mode==='5d'?'5D示教记录轴上高度和接近夹角；抓取中心X/Y投影为0，方位由规划选择。':'参数随新夹法保存；记录不发送运动。';
      const disabled=!id||!!g.busy||!!g.stopping||this.pending||otherPending;
      this.force.disabled=this.opening.disabled=this.point.disabled=disabled;
      this.button.disabled=disabled||state.state!=='tracking'||!state.quality?.good;
    }
    async record(){
      if(this.pending||this.button.disabled||!this.objectId)return;
      if([this.force,this.opening].some(el=>el.value.trim()===''||!el.checkValidity())||
         !Number.isInteger(Number(this.force.value))||!Number.isInteger(Number(this.opening.value))){
        this.feedback.textContent='请输入力度 1–100、张开位置 0–1000 的整数原始值。';return;
      }
      const request={object_id:this.objectId,use_current_point:this.point.checked,
        gripper_parameters:{force_target_raw:Number(this.force.value),open_position:Number(this.opening.value)}};
      this.pending=true;this.button.disabled=this.force.disabled=this.opening.disabled=this.point.disabled=true;
      this.feedback.textContent='正在读取姿态并记录夹法…';
      try{const result=await this.send('/api/grasp/teach',request);this.feedback.textContent=result.message;}
      catch(error){this.feedback.textContent=error.message;}
      finally{this.pending=false;}
    }
  }
  global.TeachGraspPanel=TeachGraspPanel;
})(globalThis);
