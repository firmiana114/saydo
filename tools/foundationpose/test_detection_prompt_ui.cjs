'use strict';
const fs = require('fs');
const vm = require('vm');
const assert = require('assert');

function element(tag) {
  return {
    tag,
    children: [],
    style: {},
    attrs: {},
    open: false,
    disabled: false,
    value: '',
    append(...items) {
      this.children.push(...items);
    },
    appendChild(item) {
      this.children.push(item);
    },
    setAttribute(k, v) {
      this.attrs[k] = v;
    },
    addEventListener(k, v) {
      this[k] = v;
    },
    showModal() {
      this.open = true;
    },
    close() {
      this.open = false;
    },
    set textContent(v) {
      this.text = v;
      this.children = [];
    },
    get textContent() {
      return this.text;
    },
  };
}

const fixed = new Map();
const posts = [];
const rows = [
  {
    id: 'cup',
    name: '纸杯',
    available: true,
    pose_mode: '6d',
    pose_mode_revision: 'pose',
    detection_prompt: '',
    effective_detection_prompt: '纸杯',
    detection_prompt_revision: 'old',
  },
  {
    id: 'bottle',
    name: '水瓶',
    available: true,
    pose_mode: '5d',
    pose_mode_revision: 'pose2',
    detection_prompt: '透明水瓶',
    detection_prompt_revision: 'b1',
  },
  {
    id: 'broken',
    name: '损坏模型',
    available: false,
    reason: '模型文件缺失',
    pose_mode: '6d',
  },
];
const document = {
  querySelector(id) {
    if (!fixed.has(id)) {
      fixed.set(id, element(id));
    }
    return fixed.get(id);
  },
  createElement: element,
};
let failMode = false;
let failSwitch = false;
const context = vm.createContext({
  document,
  fetch: async() => ({ok: true, json: async() => ({objects: rows})}),
  message() {},
  liveView() {},
  selection: 1,
  anchor: 1,
  box: 1,
  candidates: [1],
  post: async(path, body) => {
    posts.push({path, body});
    if (
        body.prompt === 'conflict' || (path.endsWith('pose-mode') && failMode)
    ) {
      throw Error('版本冲突');
    }
    if (path === '/api/object') {
      if (failSwitch) {
        throw Error('忙碌中');
      }
      return {};
    }
    const row = rows.find(x => x.id === body.id);
    return {
      object: {
        ...row,
        ...(
            path.endsWith('pose-mode') ?
            {pose_mode: body.pose_mode, pose_mode_revision: 'pose-new'} :
            {
              detection_prompt: body.prompt,
              effective_detection_prompt: body.prompt || row.name,
              detection_prompt_revision: 'new',
            }
        ),
      }
    };
  },
});
const html = fs.readFileSync(__dirname + '/live_tracker.html', 'utf8');
vm.runInContext(
    html.slice(
        html.indexOf('const chooser ='), html.indexOf('let openingGripper =')
    ),
    context
);
vm.runInContext("currentObjectId='cup'", context);
const byClass =
    (root, cls) =>
    root.className === cls ?
    root :
    root.children.map(x => byClass(x, cls)).find(Boolean);
(async() => {
  await fixed.get('#objects-open').onclick();
  const panels = fixed.get('#object-details').children;
  const buttons = fixed.get('#object-list').children;
  const apply = fixed.get('#objects-apply');
  assert.equal(panels[0].hidden, false);
  assert.equal(panels[1].hidden, true);
  assert(apply.disabled, 'current object is not switched again');
  const prompt = byClass(panels[0], 'object-prompt');
  const save = byClass(panels[0], 'object-save');
  const status = byClass(panels[0], 'object-feedback');
  const mode = byClass(panels[0], 'object-mode-toggle');
  assert.equal(prompt.placeholder, '纸杯');
  assert.equal(prompt.maxLength, 2000);
  prompt.value = '白色纸杯，不要带把手的马克杯';
  prompt.oninput();
  buttons[1].onclick();
  assert.equal(posts.length, 0, 'browsing must not switch or save');
  buttons[0].onclick();
  assert.equal(
      prompt.value,
      '白色纸杯，不要带把手的马克杯',
      'draft retained across detail selection'
  );
  await save.onclick();
  assert.equal(posts[0].path, '/api/object/detection-prompt');
  assert.equal(posts[0].body.revision, 'old');
  assert.equal(posts[0].body.prompt, prompt.value);
  assert(status.textContent.includes('已保存'));
  prompt.value = 'conflict';
  await save.onclick();
  assert.equal(posts[1].body.revision, 'new');
  assert.equal(status.textContent, '版本冲突');
  assert.equal(save.disabled, false);
  prompt.value = '';
  await save.onclick();
  assert.equal(rows[0].effective_detection_prompt, '纸杯');
  mode.checked = true;
  await mode.onchange();
  assert.equal(rows[0].pose_mode, '5d');
  assert.equal(posts.at(-1).body.revision, 'pose');
  failMode = true;
  mode.checked = false;
  await mode.onchange();
  assert.equal(mode.checked, true);
  assert.equal(posts.at(-1).body.revision, 'pose-new');
  buttons[2].onclick();
  assert(apply.disabled);
  assert(byClass(panels[2], 'object-prompt').disabled);
  assert.equal(fixed.get('#objects-hint').textContent, '模型文件缺失');
  buttons[1].onclick();
  failSwitch = true;
  await apply.onclick();
  assert.equal(fixed.get('#objects').open, true);
  assert.equal(fixed.get('#objects-hint').textContent, '忙碌中');
  assert.equal(apply.disabled, false);
  failSwitch = false;
  await apply.onclick();
  assert.equal(posts.at(-1).path, '/api/object');
  assert.equal(posts.at(-1).body.id, 'bottle');
  assert.equal(fixed.get('#objects').open, false);
  assert.equal(vm.runInContext('selection', context), null);
  assert.equal(fixed.get('#start').disabled, true);
  console.log(
      'Object chooser: separate browse/switch, draft retention, revisions, save/conflict feedback, unavailable models and switch recovery passed.'
  );
})().catch (e => {
  console.error(e);
  process.exitCode = 1;
});
