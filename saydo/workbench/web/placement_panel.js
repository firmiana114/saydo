/* Table placement uses the held object's measured orientation, independent of the rotation editor. */
class PlacementPanel {
  constructor(send) {
    this.send = send;
    this.pending = false;
    this.dirty = false;
    this.config = null;
    this.held = null;
    this.state = null;
    this.fields = ['x_min', 'x_max', 'y_min', 'y_max', 'gap'].map(
        key => document.querySelector('#place-' + key)
    );
    this.feedback = document.querySelector('#placement-feedback');
    this.summary = document.querySelector('#placement-summary');
    this.save = document.querySelector('#place-save');
    this.preview = document.querySelector('#place-preview');
    this.execute = document.querySelector('#place-execute');
    this.reload = document.querySelector('#place-reload');
    for (const input of this.fields) {
      input.oninput = () => {
        this.dirty = true;
        this.preview.disabled = true;
        this.execute.disabled = true;
        this.feedback.textContent = '设置尚未保存，请先保存范围与释放间隙。';
      };
    }
    this.reload.onclick = () => {
      this.dirty = false;
      this.load();
      this.feedback.textContent = '已载入保存的放置设置。';
    };
    this.save.onclick = () => this.saveConfig();
    this.preview.onclick = () => this.start(true);
    this.execute.onclick = () => this.start(false);
    document.querySelector('#place-stop').onclick = async() => {
      try {
        await this.send('/api/grasp/stop');
        this.feedback.textContent = '已请求停止放置任务。';
      } catch (error) {
        this.feedback.textContent = error.message;
      }
    };
  }

  load() {
    if (!this.config?.settings) {
      return;
    }
    const settings = this.config.settings;
    for (let i = 0; i < 4; i++) {
      this.fields[i].value =
          settings.region_xy_m ?
          Number(
              (
                  settings.region_xy_m[
                      ['x_min', 'x_max', 'y_min', 'y_max'][i]
                  ] *
                  1000
              ).toFixed(3)
          ) :
          '';
    }
    this.fields[4].value = Number((settings.release_gap_m * 1000).toFixed(3));
  }

  request() {
    if (!this.config?.revision) {
      throw Error('未读取到有效放置配置');
    }
    if (
        this.fields.some(
            input => input.value.trim() === '' || !input.checkValidity()
        )
    ) {
      throw Error('请输入有效的毫米数值；释放间隙范围为 0–60 毫米。');
    }
    const values = this.fields.map(input => Number(input.value) / 1000);
    if (
        values.some(value => !Number.isFinite(value)) ||
        values[0] >= values[1] ||
        values[2] >= values[3]
    ) {
      throw Error('X、Y 最小值必须分别小于最大值。');
    }
    return {
      revision: this.config.revision,
      region_xy_m: {
        x_min: values[0], x_max: values[1], y_min: values[2], y_max: values[3]
      },
      release_gap_m: values[4],
    };
  }

  async saveConfig() {
    if (this.pending || this.state?.busy) {
      return;
    }
    let request;
    try {
      request = this.request();
    } catch (error) {
      this.feedback.textContent = error.message;
      return;
    }
    this.pending = true;
    this.save.disabled = true;
    this.preview.disabled = true;
    this.execute.disabled = true;
    try {
      this.config = await this.send('/api/place/config', request);
      this.dirty = false;
      this.load();
      this.feedback.textContent = '范围与释放间隙已保存；本次不移动机械臂。';
      this.draw(null);
    } catch (error) {
      this.feedback.textContent = '保存失败：' + error.message;
    } finally {
      this.pending = false;
    }
  }

  async start(preview) {
    if (
        this.pending ||
        this.dirty ||
        this.state?.busy ||
        !this.held ||
        !this.config?.configured ||
        (!preview && !this.state?.motion_enabled)
    ) {
      return;
    }
    this.pending = true;
    this.preview.disabled = true;
    this.execute.disabled = true;
    const held = this.held;
    try {
      await this.send(
          preview ? '/api/place/preview' : '/api/place',
          {
            revision: this.config.revision,
            hold_id: held.hold_id,
            object_id: held.object_id,
            model_sha256: held.model_sha256,
            orientation_version: held.orientation_version ?? 0,
          }
      );
      this.feedback.textContent =
          preview ?
          '正在读取当前朝向并计算放置方案，不发送运动…' :
          '正在重新规划并执行放置，可点击“停止放置”中止。';
    } catch (error) {
      this.feedback.textContent = error.message;
    } finally {
      this.pending = false;
    }
  }

  update(status, otherPending = false) {
    const g = status.grasp || {};
    this.state = g;
    this.held = g.held_object || null;
    const previous = this.config?.revision;
    if (!this.pending) {
      this.config = g.placement_config || null;
    }
    if (!this.dirty && !this.pending && previous !== this.config?.revision) {
      this.load();
    }
    const locked = !!g.busy || this.pending || otherPending;
    this.save.disabled = locked || !this.config?.revision;
    this.reload.disabled = locked || !this.config?.settings;
    for (const input of this.fields) {
      input.disabled = locked || !this.config?.settings;
    }
    this.preview.disabled =
        locked || this.dirty || !this.held || !this.config?.configured;
    this.execute.disabled = this.preview.disabled || !g.motion_enabled;
    document.querySelector('#place-stop').disabled = !g.busy;
    document.querySelector('#placement-info').textContent =
        (
            this.held ?
            '当前夹持：' +
            (this.held.object_name || this.held.object_id) +
            '；以执行时的实际朝向为最终放置朝向。' :
            '先成功抓取物品；需要改变朝向时，请切换到“将物品摆至指定姿态”页。'
        ) +
        (
            this.config?.table_z_m != null ?
            ' 桌面 Z：' +
            (this.config.table_z_m * 1000).toFixed(2) +
            ' 毫米（估算）。' :
            ''
        );
    if (g.operation === 'place' && !this.pending && !this.dirty) {
      this.feedback.textContent = g.message || '';
    } else if (this.config?.error) {
      this.feedback.textContent = '配置错误：' + this.config.error;
    }
    const plan = g.operation === 'place' ? g.placement_plan : null;
    if (plan) {
      const mm = value => (value * 1000).toFixed(2);
      const labels = {
        raise: '到达转移高度',
        transfer: '移至上方',
        lower: '下降并松爪',
        withdraw: '沿工具 -Z 后撤',
        retreat: '竖直退出',
      };
      this.summary.textContent = '目标 TCP（Base X/Y/Z）：' + plan.target_tcp_position_m.map(mm).join(' / ') + ' 毫米\n' + '物品最低点相对 TCP：' + mm(plan.lowest_point_offset_z_m) + ' 毫米；释放间隙：' + mm(plan.release_gap_m) + ' 毫米\n' + '路径：' + plan.waypoints.map(
          w =>
          (labels[w.name] || w.name) +
          ' (' +
          (w.method === 'joint_pose' ? 'movej_p' : 'movel') +
          ') [' +
          w.position_m.map(mm).join(', ') +
          ']'
      ).join(
          ' → '
      ) + ' 毫米\n' + '完全张开至 1000 后，沿 grip_tcp 的 -Z 后撤 ' + mm(
          plan.withdraw_distance_m ?? 0
      ) +
          ' 毫米，再竖直退出；执行会重新读取状态并规划。未验证物品松开后的稳定性、夹指与环境碰撞。';
    } else {
      this.summary.textContent =
          this.config?.configured ?
          '预览将显示物品占地范围、释放高度和动作顺序。' :
          '请先输入实际可用桌面范围并保存，单位为毫米。';
    }
    this.draw(plan);
  }

  draw(plan) {
    const canvas = document.querySelector('#placement-map');
    const ctx = canvas.getContext('2d');
    const region = plan?.region_xy_m || this.config?.settings?.region_xy_m;
    ctx.clearRect(0, 0, canvas.width, canvas.height);
    ctx.fillStyle = '#b9cad9';
    ctx.font = '14px system-ui';
    if (!region) {
      ctx.fillText('设置桌面范围后显示俯视预览', 22, 35);
      return;
    }
    const padding = 45;
    const w = canvas.width - 2 * padding;
    const h = canvas.height - padding - 62;
    const scale = Math.min(
        w / (region.x_max - region.x_min), h / (region.y_max - region.y_min)
    );
    const left = (canvas.width - (region.x_max - region.x_min) * scale) / 2;
    const x = value => left + (value - region.x_min) * scale;
    const y = value => canvas.height - padding - (value - region.y_min) * scale;
    ctx.strokeStyle = '#79c9ef';
    ctx.lineWidth = 2;
    ctx.strokeRect(
        x(region.x_min),
        y(region.y_max),
        (region.x_max - region.x_min) * scale,
        (region.y_max - region.y_min) * scale
    );
    ctx.fillText('Base 俯视：X →，Y ↑', padding, 24);
    ctx.fillText(
        'X ' +
            (region.x_min * 1000).toFixed(0) +
            '～' +
            (region.x_max * 1000).toFixed(0) +
            ' mm；Y ' +
            (region.y_min * 1000).toFixed(0) +
            '～' +
            (region.y_max * 1000).toFixed(0) +
            ' mm',
        padding,
        canvas.height - 12
    );
    if (plan) {
      const low = plan.footprint_min_xy_m;
      const high = plan.footprint_max_xy_m;
      ctx.fillStyle = '#70c6a366';
      ctx.strokeStyle = '#70c6a3';
      ctx.fillRect(
          x(low[0]),
          y(high[1]),
          (high[0] - low[0]) * scale,
          (high[1] - low[1]) * scale
      );
      ctx.strokeRect(
          x(low[0]),
          y(high[1]),
          (high[0] - low[0]) * scale,
          (high[1] - low[1]) * scale
      );
      const tcp = plan.target_tcp_position_m;
      ctx.fillStyle = '#ffd27c';
      ctx.beginPath();
      ctx.arc(x(tcp[0]), y(tcp[1]), 4, 0, Math.PI * 2);
      ctx.fill();
      ctx.fillText('绿框：物品投影包围框；黄点：TCP', padding, 44);
    }
  }
}
