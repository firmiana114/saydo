// 相机出错后被取图程序重新打开时页面自动接回，以及停帧时的提示。
const WEB_DIR = require('path').join(
    __dirname, '..', '..', 'saydo', 'camera_monitor', 'web');
const assert = require('node:assert/strict');
const fs = require('node:fs');
const vm = require('node:vm');
const nodes = new Map();
let nowMs = 1000000;

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
    stalled: false,
  };
}

// 页面用 Date.now() 计时；这里换成可以拨动的钟。
class Clock extends Date {
  static now() {
    return nowMs;
  }
}

const rows = {d435i: row('d435i'), d405: row('d405')};
const context = vm.createContext({
  document: {getElementById: node},
  Date: Clock,
  console,
  setInterval() {},
  fetch: async() => ({ok: true, json: async() => ({cameras: rows})}),
});
vm.runInContext(fs.readFileSync(WEB_DIR + '/app.js', 'utf8'), context);
const refresh = () => vm.runInContext('status()', context);
const feed = node('d405-feed');

(async() => {
  // 用户开始预览，画面出来。
  node('d405-start').onclick();
  const first = feed.src;
  rows.d405.state = 'streaming';
  await refresh();
  assert.equal(feed.style.display, 'block');

  // 相机停帧：画面留着，说明停了多久、会自动继续。
  rows.d405.stalled = true;
  rows.d405.frame_at = 1000;
  rows.d405.frame_age_ms = 6400;
  await refresh();
  assert.equal(feed.src, first);
  assert.match(node('d405-error').textContent, /已有 6 秒没有出图/);
  assert.match(node('d405-error').textContent, /自动继续/);
  rows.d405.stalled = false;
  rows.d405.frame_age_ms = 40;
  await refresh();
  assert.equal(node('d405-error').textContent, '');

  // 相机出错：画面收起并显示原因。
  rows.d405.state = 'error';
  rows.d405.error = '相机连续 5 次重开后仍不出图';
  await refresh();
  assert.equal(feed.src, undefined);
  assert.match(node('d405-error').textContent, /仍不出图/);

  // 取图程序把相机重新打开了：页面自动接回，不用用户再点。
  nowMs += 5000;
  rows.d405.state = 'streaming';
  rows.d405.error = null;
  await refresh();
  assert.ok(feed.src.startsWith('/stream/d405?'));

  // 接回去又立刻断开时，3 秒内不反复重试。
  feed.onerror();
  nowMs += 1000;
  await refresh();
  assert.equal(feed.src, undefined);
  nowMs += 2500;
  await refresh();
  assert.ok(feed.src.startsWith('/stream/d405?'));

  // 用户自己点了释放：即使相机还在为程序采集，也不再接回。
  await node('d405-stop').onclick();
  nowMs += 10000;
  await refresh();
  assert.equal(feed.src, undefined);

  // 用户没看过的那一路，相机在采集也不会自己打开预览。
  rows.d435i.state = 'streaming';
  await refresh();
  assert.equal(node('d435i-feed').src, '');
  console.log('停帧提示与出错后自动接回检查通过');
})().catch (error => {
  console.error(error);
  process.exitCode = 1;
});
