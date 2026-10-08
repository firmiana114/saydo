// 被测页面与脚本所在的目录。
const WEB_DIR = require('path').join(
    __dirname, '..', '..', 'saydo', 'camera_monitor', 'web');
const assert = require('node:assert/strict');
const fs = require('node:fs');
const vm = require('node:vm');
const nodes = new Map();

function node(id) {
  if (!nodes.has(id)) {
    nodes.set(id, {
      style: {},
      disabled: false,
      textContent: '',
      src: '',
      removeAttribute(key) {
        delete this[key];
      },
    });
  }
  return nodes.get(id);
}

const rows = Object.fromEntries(
    ['d435i', 'd405'].map(
        id =>
        [
          id,
          {
            state: 'idle',
            camera: id,
            camera_serial: id,
            width: 1280,
            height: 720,
            target_fps: 15,
            clients: 0,
            sequence: 0,
          },
        ]
    )
);
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
(async() => {
  await vm.runInContext('status()', context);
  node('start-all').onclick();
  assert.ok(node('d435i-feed').src.startsWith('/stream/d435i?'));
  assert.ok(node('d405-feed').src.startsWith('/stream/d405?'));
  rows.d435i.state = 'streaming';
  rows.d405.state = 'streaming';
  await vm.runInContext('status()', context);
  assert.equal(node('d435i-feed').style.display, 'block');
  assert.equal(node('d405-feed').style.display, 'block');
  rows.d405.state = 'idle';
  await node('d405-stop').onclick();
  assert.equal(
      requests.filter(r => r.options?.method === 'POST').at(-1).path,
      '/api/cameras/d405/pause'
  );
  assert.ok(node('d435i-feed').src);
  assert.equal(node('d405-feed').src, undefined);
  rows.d405.state = 'error';
  rows.d405.error = 'D405 busy';
  await vm.runInContext('status()', context);
  assert.equal(node('d405-error').textContent, 'D405 busy');
  assert.equal(node('d435i-error').textContent, '');
  assert.equal(node('d435i-feed').style.display, 'block');
  rows.d435i.state = 'idle';
  rows.d405.state = 'idle';
  rows.d405.error = null;
  await node('stop-all').onclick();
  assert.equal(
      requests.filter(r => r.options?.method === 'POST').at(-1).path,
      '/api/pause'
  );
  assert.equal(node('d435i-feed').src, undefined);
  console.log('双路显示、独立释放、单路故障隔离和全部释放检查通过');
})().catch (error => {
  console.error(error);
  process.exitCode = 1;
});
