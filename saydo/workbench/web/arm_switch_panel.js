/* 页头的“当前机械臂”切换：实时跟踪接在哪台臂的相机上，页面上的找框、
 * 跟踪、抓取、示教、复位等操作就作用在哪台臂上。只有一台臂时不显示。 */
(function(global) {
  class ArmSwitchPanel {
    /**
     * @param {function(string, Object): Promise<Object>} send 发送请求。
     * @param {function(): void} onChanged 切换成功后调用；页面用它整页
     *     重新载入，让各面板都换成新这台臂的数据。
     * @param {Document} root 页面文档，测试时传入替身。
     */
    constructor(send, onChanged, root = document) {
      this.send = send;
      this.onChanged = onChanged;
      this.root = root;
      this.pending = false;
      this.container = root.querySelector('#arm-switch');
      this.feedback = root.querySelector('#arm-switch-feedback');
      this.buttons = new Map();
      this.signature = '';
    }

    /**
     * 按后端状态刷新按钮。
     * @param {Object} state `/api/status` 的应答。
     * @param {boolean} otherPending 页面上是否有别的操作正在提交。
     */
    update(state, otherPending = false) {
      const rows = state?.arms || [];
      this.container.hidden = rows.length < 2;
      const signature = rows.map(row => row.arm + ':' + row.label).join('|');
      if (signature !== this.signature) {
        this.signature = signature;
        this.buttons.clear();
        this.container.replaceChildren();
        for (const row of rows) {
          const button = this.root.createElement('button');
          button.type = 'button';
          button.textContent = row.label;
          button.onclick = () => this.choose(row.arm, row.label);
          this.container.appendChild(button);
          this.buttons.set(row.arm, button);
        }
      }
      const busy = !!state?.grasp?.busy || !!state?.grasp?.stopping;
      for (const row of rows) {
        const button = this.buttons.get(row.arm);
        button.setAttribute('aria-pressed', row.live ? 'true' : 'false');
        button.title = row.camera ?
            (row.live ? '页面当前操作的机械臂' : '切换到这台机械臂') :
            '这台机械臂没有接相机，不能做实时跟踪';
        button.disabled =
            this.pending || otherPending || busy || row.live || !row.camera;
      }
    }

    /**
     * 请求把实时跟踪切换到另一台臂；不发送机械臂指令。
     * @param {string} arm 臂编号。
     * @param {string} label 臂的名称，用于提示。
     */
    async choose(arm, label) {
      const button = this.buttons.get(arm);
      if (this.pending || !button || button.disabled) {
        return;
      }
      this.pending = true;
      for (const other of this.buttons.values()) {
        other.disabled = true;
      }
      this.feedback.textContent = '正在切换到' + label + '…';
      try {
        await this.send('/api/live-arm', {arm});
        this.feedback.textContent = '已切换到' + label + '。';
        this.onChanged();
      } catch (error) {
        this.feedback.textContent = error.message;
      } finally {
        this.pending = false;
      }
    }
  }
  global.ArmSwitchPanel = ArmSwitchPanel;
})(globalThis);
