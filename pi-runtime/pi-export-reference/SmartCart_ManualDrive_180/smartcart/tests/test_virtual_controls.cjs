'use strict';
const assert = require('node:assert/strict');
const V = require('../web/js/virtual_controls.js');
assert.deepEqual(V.compute('forward'), [1,1]);
assert.deepEqual(V.compute('reverse'), [-1,-1]);
assert.deepEqual(V.compute('left'), [-1,1]);
assert.deepEqual(V.compute('right'), [1,-1]);
assert.deepEqual(V.compute('stop'), [0,0]);
assert.deepEqual(V.compute('unknown'), [0,0]);

function target() {
  const handlers = {};
  return {handlers, addEventListener(name, fn) { handlers[name] = fn; },
          fire(name, event = {}) { if (handlers[name]) handlers[name](event); }};
}
const output = {textContent: ''}, status = {textContent: ''}, rpm = {textContent: ''};
const limit = target(); limit.value = '1000';
const doc = target(), win = target();
const buttons = ['forward','reverse','left','right','stop'].map(name => {
  const b = target(); b.dataset = {virtual:name}; b.setPointerCapture = () => {}; return b;
});
doc.getElementById = id => ({virtualControlOutput: output, virtualControlStatus: status,
  virtualRpmOutput: rpm, virtualRpmLimit: limit})[id];
doc.querySelectorAll = () => buttons;
V.install(doc, win);
const event = {pointerId: 3, preventDefault() {}};
buttons[0].fire('pointerdown', event);
assert.equal(output.textContent, '1 / 1');
buttons[0].fire('pointerup', event);
assert.equal(output.textContent, '0 / 0');
buttons[2].fire('pointerdown', event);
assert.equal(output.textContent, '-1 / 1');
win.fire('blur');
assert.equal(output.textContent, '0 / 0');
buttons[3].fire('pointerdown', event);
doc.hidden = true; doc.fire('visibilitychange');
assert.equal(output.textContent, '0 / 0');
buttons[0].fire('pointerdown', {pointerId: 3, preventDefault() {}});
buttons[2].fire('pointerdown', {pointerId: 4, preventDefault() {}});
assert.equal(output.textContent, '0 / 1');
assert.equal(rpm.textContent, '0 / 1000');
buttons[1].fire('pointerup', {pointerId: 3});
assert.equal(output.textContent, '-1 / 1');
buttons[4].fire('pointerdown', {pointerId: 7, preventDefault() {}});
assert.equal(output.textContent, '0 / 0');
assert.equal(rpm.textContent, '0 / 0');
limit.value = '999999';
buttons[0].fire('pointerdown', {pointerId: 8, preventDefault() {}});
assert.equal(rpm.textContent, '10000 / 10000');
win.fire('pagehide');
assert.equal(rpm.textContent, '0 / 0');
console.log('가상 조종 방향·해제·포커스 상실 검증 통과');
