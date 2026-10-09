// 2 臂相机卡片的显示，以及相机被采集程序借走后自动恢复预览。
const WEB_DIR = require('path').join(
    __dirname, '..', '..', 'saydo', 'camera_monitor', 'web');
const assert = require('node:assert/strict');
const fs = require('node:fs');
const vm = require('node:vm');
const nodes = new Map();
const ARM2 = 'd435i_arm2';

function node(id) {
  if (!nodes.has(id)) {
    nodes.set(id, {
      style: {},
      disabled: false,
      hidden: false,
      textContent: '',
      src: '',
      removeAttribute(key) {
        delete this[key];
      },
    });
  }
  return nodes.get(id);
}

function row(id) {
  return {
    state: 'idle',
    camera: id,
    camera_serial: id,
    width: 1280,
    height: 720,
    target_fps: 15,
    clients: 0,
    sequence: 0,
    held_s: 0,
  };
}

const rows = {d435i: row('d435i'), d405: row('d405')};
const requests = [];
const context = vm.createContext({
  document: {getElementById: node},
  Date,
  console,
  setInterval() {},
  fetch: async(path, options) => {
    requests.push({path, options});
    return {ok: true, json: async() => ({cameras: rows})};
  },
});
vm.runInContext(fs.readFileSync(WEB_DIR + '/app.js', 'utf8'), context);
const refresh = () => vm.runInContext('status()', context);

(async() => {
  // 服务没有提供 2 臂相机：卡片隐藏，“开始全部预览”也不去连它。
  await refresh();
  assert.equal(node(ARM2 + '-card').hidden, true);
  assert.equal(node('d435i-card').hidden, false);
  node('start-all').onclick();
  assert.equal(node(ARM2 + '-feed').src, '');
  await node('stop-all').onclick();

  // 服务提供了 2 臂相机：卡片出现，可以单独预览。
  rows[ARM2] = row(ARM2);
  await refresh();
  assert.equal(node(ARM2 + '-card').hidden, false);
  node(ARM2 + '-start').onclick();
  assert.ok(node(ARM2 + '-feed').src.startsWith('/stream/' + ARM2 + '?'));
  rows[ARM2].state = 'streaming';
  await refresh();
  assert.equal(node(ARM2 + '-feed').style.display, 'block');

  // 采集程序借走相机：画面收起并说明原因，不能手动重连。
  rows[ARM2].state = 'idle';
  rows[ARM2].held_s = 40;
  await refresh();
  assert.equal(node(ARM2 + '-feed').src, undefined);
  assert.match(node(ARM2 + '-title').textContent, /借给采集程序/);
  assert.equal(node(ARM2 + '-start').disabled, true);
  await refresh();
  assert.equal(node(ARM2 + '-feed').src, undefined);

  // 归还后自动恢复预览，不用用户再点。
  rows[ARM2].held_s = 0;
  await refresh();
  assert.ok(node(ARM2 + '-feed').src.startsWith('/stream/' + ARM2 + '?'));
  rows[ARM2].state = 'streaming';
  await refresh();
  assert.equal(node(ARM2 + '-feed').style.display, 'block');

  // 借走期间用户主动释放：归还后不再自动重连。
  rows[ARM2].state = 'idle';
  rows[ARM2].held_s = 40;
  await refresh();
  await node(ARM2 + '-stop').onclick();
  rows[ARM2].held_s = 0;
  await refresh();
  await refresh();
  assert.equal(node(ARM2 + '-feed').src, undefined);

  // 没被借走、只是别处释放了预览：不自动重连，由用户决定。
  node(ARM2 + '-start').onclick();
  rows[ARM2].state = 'streaming';
  await refresh();
  rows[ARM2].state = 'idle';
  node(ARM2 + '-feed').onerror();
  await refresh();
  assert.equal(node(ARM2 + '-feed').src, undefined);
  console.log('2 臂相机卡片与借用后自动恢复检查通过');
})().catch (error => {
  console.error(error);
  process.exitCode = 1;
});
