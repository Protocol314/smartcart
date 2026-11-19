'use strict';
const assert = require('assert');
const sensing = require('../web/js/sensing.js');
const core = require('../web/js/core.js');
const autonomy = require('../web/js/virtual_autonomy.js');

const original = {x: 2.5, y: 0.4};
const adjusted = sensing.uwbDisplayPosition(original);
assert.deepStrictEqual(original, {x: 2.5, y: 0.4});
for (const scale of [20, 100]) {
  const before = core.toScreen(original, 200, 200, scale);
  const after = core.toScreen(adjusted, 200, 200, scale);
  assert.equal(after.x, before.x);
  assert.equal(after.y - before.y, 1.25 * scale);
}
const frame = sensing.syntheticUwb(0);
frame.measurement.position = {frame_id:'base_link', x:2.1, y:0};
const shown = sensing.deriveUwb(frame, {mode:'demo', radius:1});
assert(shown.marker, '이동 후 반경에 들어온 마커는 표시해야 한다');
assert.equal(shown.marker.x, 2.1);
assert.equal(shown.measurement.position.x, 2.1);

const elements = {};
function el(id) {
  if (!elements[id]) elements[id] = {
    value: '', textContent:'', attrs:{}, listeners:{}, width:260, height:180,
    addEventListener(event, fn) { this.listeners[event] = fn; },
    setAttribute(key, value) { this.attrs[key] = value; },
    querySelector() { return this.span || (this.span = {textContent:''}); },
    getContext() { return new Proxy({}, {get: (obj, key) => obj[key] || (() => {})}); },
    click() { this.listeners.click(); }
  };
  return elements[id];
}
const doc = {getElementById:el, querySelectorAll:()=>[], addEventListener:()=>{}, hidden:false};
let now=1000, tick;
const win = {performance:{now:()=>now}, setInterval:fn=>{tick=fn;}, addEventListener:()=>{}};
el('simSource').value='live';
autonomy.install(doc,win);
el('simAuto').click();
assert.equal(el('simAuto').attrs['aria-pressed'],'false');
assert.match(el('simHeaderStatus').textContent,/시작 불가.*입력 대기/);
const n=360, inc=2*Math.PI/n;
const input={ready:true,until:1200,scan:{angle_min:-Math.PI,angle_max:Math.PI-inc,angle_increment:inc,range_min:.05,range_max:10,ranges:Array(n).fill(4),truncated:false},track:{state:'tracking',candidateCount:1,confirmationCount:3,selected:{x:3,y:0},uwb:{x:3,y:0},transform:{yaw:0,offsetX:0,offsetY:0}},uwb:{profile:'single_left',valid:true,sensors:{uwb_left:{idValid:true,measurementValid:true}}}};
win.SmartCartVirtualInput(input);
el('simFollow').click(); el('simAuto').click();
assert.equal(el('simAuto').attrs['aria-pressed'],'true');
now=1050; tick();
assert.equal(el('simAuto').attrs['aria-pressed'],'true');
now=1200; tick();
assert.equal(el('simAuto').attrs['aria-pressed'],'false');
assert.match(el('simHeaderStatus').textContent,/만료/);
input.until=1500; win.SmartCartVirtualInput(input); now=1250; tick();
assert.equal(el('simAuto').attrs['aria-pressed'],'false','복구 후 자동으로 다시 시작해서는 안 된다');
el('simAuto').click(); assert.equal(el('simAuto').attrs['aria-pressed'],'true');
console.log('PASS: UWB 표시 이동·원시 좌표 보존·실측 시작 차단 및 만료 후 수동 재시작');
