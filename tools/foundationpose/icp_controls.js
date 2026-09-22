/* Live ICP settings: no service restart and no robot commands. */
(function(global){
  class ICPControls {
    constructor(root,post){
      this.post=post;this.pending=false;this.config=null;this.robotBusy=false;this.error='';
      this.toggle=root.querySelector('#icp-enabled');this.mode=root.querySelector('#icp-mode');
      this.status=root.querySelector('#icp-status');this.lastMode='shadow';this.report=null;
      this.toggle.onchange=()=>this.change(this.toggle.checked?this.lastMode:'off');
      this.mode.onchange=()=>this.change(this.mode.value);
      this.draw();
    }
    update(state){
      this.robotBusy=!!state.grasp?.busy;
      if(!this.pending&&state.icp_config&&(!this.config||state.icp_config.revision>=this.config.revision)){
        this.config=state.icp_config;
        this.report=state.icp?.revision===this.config.revision?state.icp:null;
      }
      this.draw();
    }
    draw(){
      const enabled=!!this.config&&this.config.mode!=='off';
      this.toggle.checked=enabled;this.toggle.disabled=this.pending||this.robotBusy||!this.config;
      if(enabled)this.lastMode=this.config.mode;
      this.mode.value=this.lastMode;this.mode.disabled=this.toggle.disabled||!enabled;
      const names={off:'已关闭',shadow:'仅对比，抓取使用原位姿',apply:'应用通过检查的精修位姿'};
      const reasons={budget_exceeded:'超时，保留原位姿',stale_frame:'画面过旧，跳过精修',degenerate_geometry:'表面几何约束不足',insufficient_depth_support:'有效深度不足',insufficient_visible_surface:'可见表面不足',raw_quality_failed:'原位姿不可靠',correction_limit:'修正幅度过大',correspondence_loss:'匹配点不足',coverage_regressed:'匹配覆盖率下降',no_improvement:'未改善，保留原位姿',render_validation_failed:'深度复核未通过',invalid_input:'输入无效',invalid_input_or_solver_failure:'精修失败'};
      let text=this.pending?'正在切换…':this.config?names[this.config.mode]:'等待服务状态…';
      if(this.report&&enabled){
        text+=' · '+(this.report.applied?'精修已采用':this.report.accepted?'对比结果通过':reasons[this.report.reason]||'等待有效帧');
        if(Number.isFinite(this.report.elapsed_ms))text+=' · '+this.report.elapsed_ms.toFixed(1)+' ms';
        if(Number.isFinite(this.report.initial_rmse_mm)&&Number.isFinite(this.report.final_rmse_mm))text+=' · 拟合偏差 '+this.report.initial_rmse_mm.toFixed(2)+' → '+this.report.final_rmse_mm.toFixed(2)+' mm';
      }
      if(this.robotBusy)text+=' · 任务进行中，暂不可切换';
      this.status.textContent=this.error||text;
    }
    async change(mode){
      if(this.pending||this.robotBusy||!this.config){this.draw();return;}
      this.pending=true;this.error='';this.draw();
      try{this.config=await this.post('/api/icp',{mode,revision:this.config.revision});this.report=null;}
      catch(error){this.error='切换失败：'+error.message;}
      finally{this.pending=false;this.draw();}
    }
  }
  global.ICPControls=ICPControls;
})(globalThis);
