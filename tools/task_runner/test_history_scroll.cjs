const fs = require('fs');
const vm = require('vm');
const assert = require('assert');
const source = fs.readFileSync(__dirname + '/app.js', 'utf8');
const block = source.slice(
    source.indexOf('function renderHistory('),
    source.indexOf('async function refresh(')
);
const history = {
  dataset: {},
  scrollTop: 120,
  children: [],
  replaceChildren() {
    this.children = [];
    this.scrollTop = 0;
  },
  append(node) {
    this.children.push(node);
  },
};
const context = {
  el: () => history,
  document: {createElement: () => ({})},
  states: {running: '执行中', succeeded: '已结束'},
  refresh() {},
  selected: null,
};
vm.createContext(context);
vm.runInContext(block, context);
const runs = [
  {run_id: 'one', state: 'running', instruction: '任务一'},
  {run_id: 'two', state: 'succeeded', instruction: '任务二'},
];
context.renderHistory(runs);
assert.equal(history.scrollTop, 120);
const node = history.children[0];
context.renderHistory(runs);
assert.strictEqual(
    history.children[0], node, 'polling unchanged history must not rebuild it'
);
history.scrollTop = 90;
context.renderHistory([{...runs[0], state: 'succeeded'}, runs[1]]);
assert.equal(history.scrollTop, 90);
const html = fs.readFileSync(__dirname + '/index.html', 'utf8');
assert(html.includes('id="progress-scroll"'));
assert(html.includes('#history{max-height:'));
assert(html.includes('overflow-y:auto'));
console.log(
    'History scrolling passed: bounded regions, stable polling and preserved scroll position.'
);
