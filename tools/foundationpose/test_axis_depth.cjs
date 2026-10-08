const fs = require('fs'), vm = require('vm'), assert = require('assert');
const source = fs.readFileSync(__dirname + '/reorientation_viewer.js', 'utf8');
const ctx = vm.createContext({});
vm.runInContext(source, ctx);
const M = vm.runInContext(
    'ReorientationMath', ctx
), C = vm.runInContext('AxisDepthCue', ctx);
assert.equal(C.describe([0, 0, 1], [0, 0, 1]).kind, 'front');
assert.equal(C.describe([0, 0, -1], [0, 0, 1]).kind, 'back');
assert.equal(C.describe([1, 0, 0], [0, 0, 1]).kind, 'plane');
assert.equal(
    C.describe([0, 0, .001], [0, 0, 1]).symbol,
    '⊙',
    'depth cue must not depend on axis length'
);
assert.equal(
    C.describe([-1, 0, 0], [-1, 0, 0]).kind,
    'front',
    'negative coordinate direction can point towards the viewer'
);
assert.equal(
    C.describe([1, 0, 0], [-1, 0, 0]).kind,
    'back',
    'positive direction is not always the near side'
);
const basis = M.defaultView();
assert.equal(C.describe([1, 0, 0], basis[2]).kind, 'front');
assert.equal(C.describe([0, 1, 0], basis[2]).kind, 'back');
const turned = M.multiply(basis, M.axisAngle(basis[1], Math.PI));
assert.equal(C.describe([1, 0, 0], turned[2]).kind, 'back');
for (const kind of ['front', 'back', 'plane']) {
  const rows = [];
  const vector =
      kind === 'front' ? [0, 0, 1] : kind === 'back' ? [0, 0, -1] : [1, 0, 0];
  C.segment(
      (tag, attrs) => rows.push({tag, attrs}),
      [0, 0],
      [40, 20],
      'red',
      C.describe(vector, [0, 0, 1])
  );
  const line = rows.find(
      row => row.tag === 'line' && row.attrs.stroke === 'red'
  );
  assert.equal(line.attrs['data-depth'], kind);
  assert.equal(
      line.attrs['stroke-dasharray'], kind === 'back' ? '5 4' : 'none'
  );
  if (kind === 'back') assert(line.attrs.opacity < .5);
  if (kind !== 'plane') assert(
      rows.some(row => row.tag === 'circle'),
      'out/in glyph must remain visible at the axis tip'
  );
}
const rows = [];
C.segment(
    (tag, attrs) => rows.push({tag, attrs}),
    [50, 50],
    [50, 50],
    'blue',
    C.describe([0, 0, 1], [0, 0, 1])
);
assert(
    !rows.some(row => row.tag === 'path'),
    'edge-on arrow must not invent a screen-space direction'
);
assert(
    rows.some(row => row.tag === 'circle' && row.attrs.fill === 'blue'),
    'axis pointing directly at camera uses a dot badge'
);
console.log(
    'Axis depth checks passed: front/back/parallel, sign independence, camera rotation, dashed rear and edge-on dot/cross markers.'
);
