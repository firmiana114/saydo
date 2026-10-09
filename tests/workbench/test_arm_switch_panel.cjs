// 被测页面与脚本所在的目录。
const WEB_DIR = require('path').join(
    __dirname, '..', '..', 'saydo', 'workbench', 'web');
const fs = require('fs');
const vm = require('vm');
const assert = require('assert');

// 页面文档的替身：只实现面板用到的几个方法。
function element() {
  return {
    hidden: false,
    disabled: false,
    textContent: '',
    title: '',
    children: [],
    attributes: {},
    setAttribute(name, value) {
      this.attributes[name] = value;
    },
    appendChild(child) {
      this.children.push(child);
    },
    replaceChildren() {
      this.children = [];
    },
  };
}
const fields = new Map();
const root = {
  querySelector: id => {
    if (!fields.has(id)) {
      fields.set(id, element());
    }
    return fields.get(id);
  },
  createElement: () => element(),
};
const context = vm.createContext({document: root});
vm.runInContext(
    fs.readFileSync(WEB_DIR + '/arm_switch_panel.js', 'utf8'), context
);
const Panel = vm.runInContext('ArmSwitchPanel', context);
const posts = [];
let fail = false;
let reloads = 0;
const panel = new Panel(async(path, body) => {
  posts.push({path, body});
  if (fail) {
    throw Error('1臂任务进行中，不能切换当前机械臂');
  }
  return {live_arm: body.arm, changed: true, motion_sent: false};
}, () => reloads++, root);
const state = (live, busy = false, secondCamera = true) => ({
  grasp: {busy},
  arms: [
    {arm: 'arm1', label: '1臂（左臂）', camera: true, live: live === 'arm1'},
    {
      arm: 'arm2',
      label: '2臂（右臂）',
      camera: secondCamera,
      live: live === 'arm2',
    },
  ],
});
(async() => {
  // 只有一台臂时不显示切换。
  panel.update({grasp: {}, arms: [state('arm1').arms[0]]});
  assert(panel.container.hidden);
  panel.update({grasp: {}});
  assert(panel.container.hidden);

  panel.update(state('arm1'));
  assert(!panel.container.hidden);
  const [first, second] = panel.container.children;
  assert.equal(first.textContent, '1臂（左臂）');
  assert.equal(first.attributes['aria-pressed'], 'true');
  assert.equal(second.attributes['aria-pressed'], 'false');
  // 当前这台不可再点，另一台可点。
  assert(first.disabled && !second.disabled);
  // 轮询刷新不重建按钮。
  panel.update(state('arm1'));
  assert.strictEqual(panel.container.children[1], second);

  // 有任务在进行或别的操作在提交时不能切换。
  panel.update(state('arm1', true));
  assert(second.disabled);
  await second.onclick();
  assert.equal(posts.length, 0);
  panel.update(state('arm1'), true);
  assert(second.disabled);
  // 没接相机的臂不能选，并说明原因。
  panel.update(state('arm1', false, false));
  assert(second.disabled);
  assert(second.title.includes('没有接相机'));

  // 后端拒绝时显示原因，不重新载入页面。
  panel.update(state('arm1'));
  fail = true;
  await second.onclick();
  assert.deepEqual(posts.at(-1), {path: '/api/live-arm', body: {arm: 'arm2'}});
  assert.equal(
      panel.feedback.textContent, '1臂任务进行中，不能切换当前机械臂'
  );
  assert.equal(reloads, 0);
  assert(!panel.pending);

  // 成功后通知页面重新载入。
  panel.update(state('arm1'));
  fail = false;
  await second.onclick();
  assert.equal(reloads, 1);
  assert.equal(panel.feedback.textContent, '已切换到2臂（右臂）。');
  panel.update(state('arm2'));
  assert.equal(second.attributes['aria-pressed'], 'true');
  assert(second.disabled && !first.disabled);

  // 页面里确实接入了这个面板。
  const html = fs.readFileSync(WEB_DIR + '/live_tracker.html', 'utf8');
  assert(html.includes('<script src="/arm_switch_panel.js"></script>'));
  assert(html.includes('id="arm-switch"'));
  assert(html.includes('armSwitchPanel.update(s, actionPending)'));
  console.log('arm switch panel: ok');
})().catch(error => {
  console.error(error);
  process.exit(1);
});
