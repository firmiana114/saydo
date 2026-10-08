/* Fake DOM exercises the actual panel's mm conversion, gating and request payloads. */
// 被测页面与脚本所在的目录。
const WEB_DIR = require('path').join(
    __dirname, '..', '..', 'saydo', 'workbench', 'web');
const fs = require('fs');
const vm = require('vm');
const assert = require('assert');
const elements = new Map();
const context = {
  clearRect() {},
  fillText() {},
  strokeRect() {},
  fillRect() {},
  beginPath() {},
  arc() {},
  fill() {},
};

function element(selector) {
  if (!elements.has(selector)) {
    elements.set(selector, {
      _value: '',
      get value() {
        return this._value;
      },
      set value(v) {
        this._value = String(v);
      },
      textContent: '',
      disabled: false,
      width: 650,
      height: 280,
      checkValidity() {
        const n = Number(this.value);
        return Number.isFinite(n) &&
            (
                selector === '#place-gap' ?
                n >= 0 && n <= 60 :
                Math.abs(n) <= 2000
            );
      },
      getContext() {
        return context;
      },
    });
  }
  return elements.get(selector);
}

const Panel = vm.runInNewContext(
    fs.readFileSync(WEB_DIR + '/placement_panel.js', 'utf8') +
        '\nPlacementPanel',
    {document: {querySelector: element}, console,}
);
const plain = x => JSON.parse(JSON.stringify(x));
const saved = {
  configured: true,
  error: null,
  revision: 'r1',
  table_z_m: -.02449,
  settings: {
    region_xy_m: {x_min: .2, x_max: .5, y_min: -.1, y_max: .1},
    release_gap_m: .003,
  },
};
const hold = {
  hold_id: 'h', object_id: 'cup', model_sha256: 'sha', orientation_version: 2
};
const status = {
  grasp: {
    placement_config: saved,
    held_object: hold,
    motion_enabled: true,
    busy: false,
  }
};

async function main() {
  const calls = [];
  const panel = new Panel(async(path, data) => {
    calls.push([path, plain(data)]);
    return {...saved, saved: true};
  });
  panel.update(status);
  assert.equal(element('#place-x_min').value, 200);
  assert.equal(element('#place-gap').value, 3);
  assert(!panel.preview.disabled && !panel.execute.disabled);
  await panel.start(true);
  assert.equal(calls[0][0], '/api/place/preview');
  assert.deepEqual(calls[0][1], {revision: 'r1', ...hold});
  element('#place-x_min').value = '250';
  element('#place-x_min').oninput();
  await panel.start(false);
  assert.equal(calls.length, 1, 'unsaved edits must block motion');
  panel.update(status);
  assert.equal(
      element('#place-x_min').value, '250', 'polling must not clobber edits'
  );
  element('#place-gap').value = '60';
  await panel.saveConfig();
  assert.equal(calls[1][1].region_xy_m.x_min, .25);
  assert.equal(calls[1][1].release_gap_m, .06);
  panel.update({...status, grasp: {...status.grasp, motion_enabled: false}});
  assert(panel.execute.disabled);
  assert(!panel.preview.disabled);
  await panel.start(false);
  assert.equal(calls.length, 2);
  panel.update({...status, grasp: {...status.grasp, busy: true}});
  assert(panel.preview.disabled && panel.save.disabled);
  assert(!element('#place-stop').disabled);
  panel.update({...status, grasp: {...status.grasp, held_object: null}});
  assert(panel.execute.disabled);
  element('#place-x_min').value = '';
  assert.throws(() => panel.request());
  element('#place-x_min').value = '600';
  assert.throws(() => panel.request());
  element('#place-x_min').value = '200';
  element('#place-gap').value = '-1';
  assert.throws(() => panel.request());
  element('#place-gap').value = '60.1';
  assert.throws(() => panel.request());
  console.log(
      'Placement panel checks passed: units, saved settings, stale edits, hold/mode/busy gates, preview payload, invalid ranges.'
  );
}

main().catch (error => {
  console.error(error);
  process.exitCode = 1;
});
