const fs = require('fs');
const vm = require('vm');
const assert = require('assert');
const fields = new Map();
const root = {
  querySelector: id => {
    if (!fields.has(id)) {
      fields.set(id, {value: '', disabled: false, textContent: ''});
    }
    return fields.get(id);
  },
};
const context = vm.createContext({document: root});
vm.runInContext(
    fs.readFileSync(__dirname + '/arm_speed_panel.js', 'utf8'), context
);
const Panel = vm.runInContext('ArmSpeedPanel', context);
let posts = [];
let fail = false;
const panel = new Panel(async(path, body) => {
  posts.push({path, body});
  if (fail) {
    throw Error('配置已变化');
  }
  return {value: body.value, revision: 'r2', message: '已保存', error: null};
}, root);
const state = (value = 25, revision = 'r1', busy = false) => ({
  grasp: {busy, motion_speed_config: {value, revision, error: null}}
});
(async() => {
  panel.update(state());
  assert.equal(panel.input.value, '25');
  panel.input.value = '15';
  panel.input.oninput();
  panel.update(state(20, 'external'));
  assert.equal(panel.input.value, '15');
  assert.equal(
      panel.revision, 'r1', 'polling must not silently rebase a dirty edit'
  );
  fail = true;
  await panel.save();
  assert.equal(panel.feedback.textContent, '配置已变化');
  assert(panel.dirty);
  panel.reloadButton.onclick();
  assert.equal(panel.input.value, '20');
  assert.equal(panel.revision, 'external');
  panel.input.value = '15';
  panel.input.oninput();
  fail = false;
  await panel.save();
  assert.equal(posts.at(-1).path, '/api/arm/speed');
  assert.equal(posts.at(-1).body.value, 15);
  assert.equal(posts.at(-1).body.revision, 'external');
  assert(!panel.dirty);
  const n = posts.length;
  for (const invalid of ['', '0', '26', '1.5', 'NaN']) {
    panel.input.value = invalid;
    await panel.save();
  }
  assert.equal(posts.length, n);
  panel.update(state(15, 'r2', true));
  assert(panel.input.disabled && panel.saveButton.disabled);
  await panel.save();
  assert.equal(posts.length, n);
  panel.update(state(15, 'r2', false));
  assert(!panel.input.disabled);
  assert.equal(panel.revision, 'r2');
  console.log(
      'Unified speed UI: load, draft preservation, conflict/reload, save, range and busy guards passed.'
  );
})().catch (e => {
  console.error(e);
  process.exitCode = 1;
});
