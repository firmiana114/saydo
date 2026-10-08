/* Run with node; no browser or hardware required. */
const fs = require('fs');
const vm = require('vm');
const assert = require('assert');
const source = fs.readFileSync(__dirname + '/reorientation_viewer.js', 'utf8');
const M = vm.runInNewContext(source + '\nReorientationMath');
const basis = [
  [Math.SQRT1_2, Math.SQRT1_2, 0],
  [-1 / Math.sqrt(6), 1 / Math.sqrt(6), 2 / Math.sqrt(6)],
  [1 / Math.sqrt(3), -1 / Math.sqrt(3), 1 / Math.sqrt(3)],
];

function rigid(r) {
  for (let i = 0; i < 3; i++) {
    for (let j = 0; j < 3; j++) {
      assert(Math.abs(M.dot(r[i], r[j]) - (i === j ? 1 : 0)) < 1e-10);
    }
  }
  assert(Math.abs(M.dot(M.cross(r[0], r[1]), r[2]) - 1) < 1e-10);
}

const still = M.drag([400, 200], [400, 200], 800, 400, basis);
rigid(still);
assert(Math.abs(still[0][0] - 1) < 1e-12);
const right = M.drag([400, 200], [460, 200], 800, 400, basis);
rigid(right);
assert(
    M.dot(M.apply(right, basis[2]), basis[0]) > 0,
    'rightward drag must move the front surface right on screen'
);
const up = M.drag([400, 200], [400, 140], 800, 400, basis);
assert(M.dot(M.apply(up, basis[2]), basis[1]) > 0);
for (const axis of ['x', 'y', 'z']) {
  const r = M.drag([0, 0], [150, 0], 800, 400, basis, axis);
  rigid(r);
  const v = [0, 0, 0];
  v['xyz'.indexOf(axis)] = 1;
  assert(M.apply(r, v).every((x, i) => Math.abs(x - v[i]) < 1e-12));
}
const fine = M.drag([400, 200], [460, 200], 800, 400, basis, 'free', true);
assert(
    M.dot(M.apply(fine, basis[2]), basis[0]) <
    M.dot(M.apply(right, basis[2]), basis[0])
);
let r = M.identity();
for (let i = 0; i < 1500; i++) {
  r = M.multiply(
      M.drag(
          [i % 700, i % 300], [(i * 7) % 700, (i * 11) % 300], 800, 400, basis
      ),
      r
  );
}
rigid(r);
for (const angles of [[0, 0, 0], [23, -51, 137], [0, 90, 40], [0, -90, -30]]) {
  const r = M.euler(...angles);
  const back = M.euler(...M.angles(r));
  assert(
      r.every((row, i) => row.every((x, j) => Math.abs(x - back[i][j]) < 1e-8))
  );
}
console.log(
    '7 viewer math checks passed: identity, screen direction, constrained axes, fine drag, rotation stability, Euler round-trip.'
);

const pivot = [.02, -.03, .04];
for (const r of [M.identity(), M.euler(30, 90, 135)]) {
  const o = M.objectOrigin(r, pivot);
  const v = M.apply(r, pivot);
  assert(
      v.every((x, k) => Math.abs(x + o[k]) < 1e-12),
      'held TCP must remain at the fixed preview center'
  );
}
console.log('Fixed-pivot preview checks passed.');

// A tilted object's selected local axis must remain fixed in world space.
const tilted = M.euler(32, 47, 81);
const near = (a, b) => a.every((v, i) => Math.abs(v - b[i]) < 1e-10);
for (const mode of ['x', 'y', 'z']) {
  const axis = [0, 0, 0];
  axis['xyz'.indexOf(mode)] = 1;
  const delta = M.drag([100, 100], [170, 100], 800, 400, basis, mode);
  const rotated = M.rotate(tilted, delta, mode);
  rigid(rotated);
  assert(
      near(M.apply(rotated, axis), M.apply(tilted, axis)),
      'local ' + mode + ' axis must keep its world direction'
  );
  assert(
      !near(M.apply(M.multiply(delta, tilted), axis), M.apply(tilted, axis)),
      'test must distinguish local from base axes'
  );
}
const keyboard = M.rotate(
    M.euler(0, 0, 90), M.axisAngle([1, 0, 0], 5 * Math.PI / 180), 'x'
);
assert(
    near(M.angles(keyboard), [5, 0, 90]),
    'local X keyboard step after Z=90 degrees'
);
assert(
    M.rotate(tilted, right, 'free').every(
        (row, i) => near(row, M.multiply(right, tilted)[i])
    ),
    'free drag remains screen-relative'
);
console.log(
    'Local-axis drag/keyboard and unchanged free-rotation checks passed.'
);

const adaptiveShift = [0, -.035355, .035355];
const adaptiveOrigin = M.objectOrigin(tilted, pivot, adaptiveShift);
assert(
    near(
        M.apply(tilted, pivot).map((v, i) => v + adaptiveOrigin[i]),
        adaptiveShift
    ),
    'adaptive preview places the held TCP at the planned displacement'
);
console.log('Adaptive translated-pivot preview check passed.');

// Orbit changes the display basis, with the same screen direction as the drag.
const orbit = M.orbit(basis, [73, 57], [98, 57], 146, 115);
rigid(orbit);
assert(
    M.dot(orbit[0], basis[2]) > 0,
    'rightward view drag must rotate the visible scene rightward'
);
let orbitBasis = basis;
for (let i = 0; i < 1000; i++) {
  orbitBasis = M.orbit(orbitBasis, [73, 57], [90, 67], 146, 115);
}
rigid(orbitBasis);
console.log(
    'Camera orbit direction and accumulated basis stability checks passed.'
);

// Exercise actual panel event handlers with a lightweight viewer, retaining its
// real orbit/reset methods. No browser GL or robot endpoint is used here.
const elements = new Map();
const posts = [];

function element(id) {
  if (!elements.has(id)) {
    elements.set(id, {
      value: '0',
      disabled: false,
      style: {},
      textContent: '',
      checked: true,
      classList: {add() {}, remove() {}},
      focus() {},
      setAttribute() {},
      checkValidity() {
        return true;
      },
      addEventListener() {},
      getBoundingClientRect() {
        return {
          left: 0,
          top: 0,
          width: id === '#reorientation-camera-control' ? 146 : 800,
          height: id === '#reorientation-camera-control' ? 115 : 400,
        };
      },
      clientWidth: 800,
      clientHeight: 400,
      setPointerCapture(id) {
        this.pointer = id;
      },
      hasPointerCapture(id) {
        return this.pointer === id;
      },
      releasePointerCapture() {
        this.pointer = null;
      },
    });
  }
  return elements.get(id);
}

const cameraContext = vm.createContext({
  document: {querySelector: element},
  post: async(path, payload) => {
    posts.push({path, payload: JSON.parse(JSON.stringify(payload))});
    return {};
  },
  console,
});
vm.runInContext(
    source +
        `
const ActualViewer=ReorientationViewer;
ReorientationViewer=function(canvas,overlay,control){
 const viewer=Object.create(ActualViewer.prototype);
 Object.assign(viewer,{canvas,overlay,viewControl:control,basis:ReorientationMath.defaultView(),rotation:ReorientationMath.euler(20,30,40),translation:[.02,.01,0],zoom:1.4,draw(){this.drawCount=(this.drawCount||0)+1;},model:{object_id:'cup',model_sha256:'sha',initial_rotation:ReorientationMath.euler(20,30,40)}});
 return viewer;
};
const cameraPanel=new ReorientationPanel();cameraPanel.syncAngles();cameraPanel.poseDirty=true;
cameraPanel.key='cup:hold:sha:0';
const cameraStatus={object_id:'cup',grasp:{busy:false,motion_enabled:true,held_object:{object_id:'cup',hold_id:'hold',model_sha256:'sha',orientation_version:0}}};
cameraPanel.update(cameraStatus,false);
`,
    cameraContext
);
const read = expression => JSON.parse(
    vm.runInContext('JSON.stringify(' + expression + ')', cameraContext)
);
const beforeView = read(
    '({rotation:cameraPanel.viewer.rotation,translation:cameraPanel.viewer.translation,angles:cameraPanel.inputs.map(x=>x.value),dirty:cameraPanel.poseDirty})'
);
const camera = element('#reorientation-camera-control');
const event = (x, y, extra = {}) => ({
  pointerId: 1,
  clientX: x,
  clientY: y,
  button: 0,
  shiftKey: false,
  preventDefault() {},
  stopPropagation() {},
  ...extra,
});
camera.onpointerdown(event(73, 57));
camera.onpointermove(event(98, 67));
camera.onpointerup(event(98, 67));
assert.notDeepEqual(
    read('cameraPanel.viewer.basis'),
    JSON.parse(JSON.stringify(M.defaultView()))
);
assert.deepEqual(
    read(
        '({rotation:cameraPanel.viewer.rotation,translation:cameraPanel.viewer.translation,angles:cameraPanel.inputs.map(x=>x.value),dirty:cameraPanel.poseDirty})'
    ),
    beforeView
);
assert.equal(posts.length, 0, 'orbit must not send a control request');
vm.runInContext(
    'cameraStatus.grasp.busy=true;cameraPanel.update(cameraStatus,false)',
    cameraContext
);
assert(!camera.disabled, 'view-only control remains usable during robot tasks');
const beforeKey = read('cameraPanel.viewer.basis');
camera.onkeydown(event(0, 0, {key: 'ArrowUp'}));
assert.notDeepEqual(read('cameraPanel.viewer.basis'), beforeKey);
camera.ondblclick(event(0, 0));
assert.deepEqual(
    read('cameraPanel.viewer.basis'),
    JSON.parse(JSON.stringify(M.defaultView()))
);
assert.equal(read('cameraPanel.viewer.zoom'), 1);
assert.deepEqual(read('cameraPanel.viewer.rotation'), beforeView.rotation);
camera.onpointerdown(event(73, 57));
camera.onpointermove(event(100, 70));
camera.onpointercancel(event(100, 70));
const afterCancel = read('cameraPanel.viewer.basis');
camera.onpointermove(event(20, 10));
assert.deepEqual(read('cameraPanel.viewer.basis'), afterCancel);
element('#reorientation-view-reset').onclick();
assert.deepEqual(
    read('cameraPanel.viewer.basis'),
    JSON.parse(JSON.stringify(M.defaultView()))
);
assert.deepEqual(
    read('cameraPanel.viewer.translation'), beforeView.translation
);
assert.equal(posts.length, 0);
vm.runInContext(
    'cameraStatus.grasp.busy=false;cameraPanel.update(cameraStatus,false)',
    cameraContext
);
vm.runInContext('cameraPanel.request(true)', cameraContext).then(() => {
  assert.deepEqual(
      posts[0].payload.rotation,
      beforeView.rotation,
      'API target rotation must remain unchanged after view operations'
  );
  vm.runInContext(
      "Object.assign(cameraStatus.grasp,{operation:'reorient',state:'failed',error:'motion failed',message:'夹持记录已保留，可重新选择目标姿态后再试。',reorientation_plan:null});cameraPanel.update(cameraStatus,false)",
      cameraContext
  );
  assert(
      !element('#reorient-preview').disabled && !element('#reorient').disabled,
      'a failed motion with a retained hold must permit a new target'
  );
  assert(
      element('#reorientation-feedback').textContent.includes('夹持记录已保留')
  );
  console.log(
      'View widget interaction checks passed: drag, keyboard, reset, cancellation, busy viewing, unchanged pose/translation/angles and API rotation.'
  );
}).catch (error => {
  console.error(error);
  process.exitCode = 1;
});

(async() => {
  const sent = [];
  const Panel = vm.runInNewContext(
      source + '\nReorientationPanel',
      {post: async(path, value) => sent.push({path, value})}
  );
  const panel = Object.create(Panel.prototype);
  Object.assign(
      panel,
      {
        held: {object_id: 'cup', hold_id: 'same-hold', orientation_version: 7},
        anglesValid: () => true,
        preview: {},
        execute: {},
        feedback: {},
        viewer: {rotation: M.identity(), model: {model_sha256: 'mesh'}},
        positionMode: {value: 'adaptive'},
      }
  );
  await panel.request(true);
  assert.equal(sent[0].path, '/api/reorient/preview');
  assert.equal(sent[0].value.orientation_version, 7);
  panel.held.orientation_version = 8;
  await panel.request(false);
  assert.equal(sent[1].path, '/api/reorient');
  assert.equal(sent[1].value.orientation_version, 8);
  panel.disabled = true;
  await panel.request(false);
  assert.equal(sent.length, 2);
  console.log(
      'Reorientation requests preserve current holding version and disabled gate.'
  );
})().catch (error => {
  console.error(error);
  process.exitCode = 1;
});
