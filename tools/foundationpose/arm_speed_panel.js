/* One global speed, applied only to subsequent arm tasks. */
(function(global) {
  class ArmSpeedPanel {
    constructor(send, root = document) {
      this.send = send;
      this.pending = false;
      this.dirty = false;
      this.revision = null;
      this.input = root.querySelector('#arm-speed');
      this.saveButton = root.querySelector('#arm-speed-save');
      this.reloadButton = root.querySelector('#arm-speed-reload');
      this.feedback = root.querySelector('#arm-speed-feedback');
      this.input.oninput = () => {
        this.dirty = true;
        this.feedback.textContent = '速度尚未保存；当前设置保持不变。';
      };
      this.input.onkeydown = event => {
        if (event.key === 'Enter') {
          event.preventDefault();
          this.save();
        }
      };
      this.saveButton.onclick = () => this.save();
      this.reloadButton.onclick = () => {
        if (this.reloadButton.disabled) return;
        this.dirty = false;
        this.revision = null;
        this.update(this.state);
        this.feedback.textContent = '已载入当前速度。';
      };
    }

    update(state, otherPending = false) {
      this.state = state;
      const g = state?.grasp || {}, config = g.motion_speed_config;
      if (
          !this.pending &&
          !this.dirty &&
          config &&
          !config.error &&
          this.revision !== config.revision
      ) {
        this.input.value = String(config.value);
        this.revision = config.revision;
      }
      if (
          config?.error && !this.dirty && !this.pending
      ) this.feedback.textContent = config.error;
      const disabled =
          this.pending ||
          otherPending ||
          !!g.busy ||
          !!g.stopping ||
          !config ||
          !!config.error ||
          !config.revision;
      this.input.disabled =
          this.saveButton.disabled =
          this.reloadButton.disabled =
          disabled;
    }

    async save() {
      if (this.pending || this.saveButton.disabled) return;
      const value = Number(this.input.value);
      if (
          this.input.value.trim() === '' ||
          !Number.isInteger(value) ||
          value < 1 ||
          value > 25
      ) {
        this.feedback.textContent = '请输入1–25的整数百分比。';
        return;
      }
      this.pending = true;
      this.input.disabled =
          this.saveButton.disabled =
          this.reloadButton.disabled =
          true;
      try {
        const result = await this.send(
            '/api/arm/speed', {value, revision: this.revision}
        );
        this.input.value = String(result.value);
        this.revision = result.revision;
        this.dirty = false;
        this.state = {
          ...this.state,
          grasp: {...this.state.grasp, motion_speed_config: result},
        };
        this.feedback.textContent = result.message;
      } catch (error) {
        this.feedback.textContent = error.message;
      } finally {
        this.pending = false;
        this.update(this.state);
      }
    }
  }
  global.ArmSpeedPanel = ArmSpeedPanel;
})(globalThis);
