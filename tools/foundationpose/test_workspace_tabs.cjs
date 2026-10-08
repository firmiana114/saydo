/* Page navigation changes visibility only; no robot requests or draft resets. */
const fs = require('fs');
const vm = require('vm');
const assert = require('assert');
const html = fs.readFileSync(__dirname + '/live_tracker.html', 'utf8');
const source = html.match(
    /<script id="workspace-tabs-script">([\s\S]*?)<\/script>/
)[1];
const names = [
  'tracking', 'grasp-editor', 'reorientation', 'placement', 'task-runner'
];
const ids = [...html.matchAll(/\bid="([^"]+)"/g)].map(m => m[1]);
assert.equal(
    new Set(ids).size,
    ids.length,
    'all existing controls must retain unique IDs'
);
const arm = html.match(/<aside class="arm-sidebar"[\s\S]*?<\/aside>/)[0];
assert(arm.includes('id="arm-panel"') && arm.includes('id="grasp-stop"'));
assert(
    !arm.includes('role="tabpanel"'),
    'robot control sidebar must not belong to a tab'
);
assert(html.includes('夹爪的正上方为 <strong>TCP −Y 方向</strong>'));

function run(saved, unavailable = false) {
  const panels = names.map(
      () => ({hidden: false, draft: {point: [.02, 0, 0], dirty: true}})
  );
  const draws = [];
  const buttons = names.map((name, index) => ({
    id: 'tab-' + name,
    attributes: {'aria-controls': name + '-panel'},
    events: {},
    getAttribute(k) {
      return this.attributes[k];
    },
    setAttribute(k, v) {
      this.attributes[k] = v;
    },
    addEventListener(k, fn) {
      this.events[k] = fn;
    },
    focus() {
      this.focused = true;
    },
  }));
  const storage = {
    getItem() {
      if (unavailable) {
        throw Error('disabled');
      }
      return saved;
    },
    setItem(k, v) {
      if (unavailable) {
        throw Error('disabled');
      }
      saved = v;
    },
  };
  vm.runInNewContext(source, {
    document: {
      querySelectorAll: () => buttons,
      getElementById: id => panels[names.indexOf(id.replace('-panel', ''))],
    },
    sessionStorage: storage,
    requestAnimationFrame: fn => fn(),
    fetch() {
      throw Error('tabs must not issue HTTP requests');
    },
    graspEditorPanel: {viewer: {draw: () => draws.push('grasp-editor')}},
    reorientationPanel: {viewer: {draw: () => draws.push('reorientation')}},
    placementPanel: {
      state: {operation: 'place', placement_plan: {id: 1}},
      draw: plan => {
        assert.equal(plan.id, 1);
        draws.push('placement');
      },
    },
  });
  const visible = () => panels.flatMap((p, i) => p.hidden ? [] : [i]);
  return {buttons, panels, visible, draws};
}

let test = run('tab-grasp-editor');
assert.deepEqual(test.visible(), [1]);
const draft = test.panels[1].draft;
for (let i = 0; i < 5; i++) {
  test.buttons[i].events.click();
  assert.deepEqual(test.visible(), [i]);
  assert.equal(test.buttons[i].attributes['aria-selected'], 'true');
  assert.equal(test.buttons[i].tabIndex, 0);
  assert.strictEqual(test.panels[1].draft, draft);
  assert(draft.dirty);
}
test.buttons[4].events.keydown({key: 'ArrowDown', preventDefault() {}});
assert.deepEqual(test.visible(), [0]);
assert(test.buttons[0].focused);
test.buttons[0].events.keydown({key: 'End', preventDefault() {}});
assert.deepEqual(test.visible(), [4]);
test.buttons[4].events.keydown({key: 'Home', preventDefault() {}});
assert.deepEqual(test.visible(), [0]);
assert(
    test.draws.includes('grasp-editor') &&
    test.draws.includes('reorientation') &&
    test.draws.includes('placement')
);
assert.deepEqual(run('invalid').visible(), [0]);
assert.deepEqual(run(null, true).visible(), [0]);
console.log(
    'Workspace tabs passed: panel isolation, persistent sidebar, draft retention, redraw, keyboard and storage fallback.'
);

assert(html.includes('src="/task-runner/"'));
assert.deepEqual(run('tab-task-runner').visible(), [4]);
const sizing = html.match(
    /<script id="task-runner-frame-script">([\s\S]*?)<\/script>/
)[1];
const frame = {contentWindow: {}, style: {}};
let resize;
vm.runInNewContext(sizing, {
  document: {getElementById: () => frame},
  window: {
    addEventListener: (name, fn) => {
      resize = fn;
    },
  },
  location: {origin: 'http://localhost:8089'},
});
resize({
  origin: 'https://untrusted.example',
  source: frame.contentWindow,
  data: {type: 'saydo-task-height', height: 1000},
});
assert.equal(frame.style.height, undefined);
resize({
  origin: 'http://localhost:8089',
  source: frame.contentWindow,
  data: {type: 'saydo-task-height', height: 1200},
});
assert.equal(frame.style.height, '1200px');
console.log('Fifth task page and same-origin frame sizing passed.');

assert(
    !/id="grasp-stop"[^>]*disabled/.test(html),
    'stop must remain available while the model is thinking and the robot is idle'
);
assert(
    html.includes(
        "document.querySelector('#grasp-stop').disabled=taskStopPending;"
    )
);
assert(html.includes("if(taskStopPending)return;"));

assert(
    !/codex-terminal/.test(html),
    'the removed Codex terminal page must not be linked'
);
console.log('Workspace has exactly five pages; no Codex terminal page.');
