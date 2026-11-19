'use strict';
// 감독기 조종 화면 연결의 가상 DOM 검사. 실제 서버·모터에 연결하지 않는다.
const assert = require('node:assert/strict');
const D = require('../web/js/drive_link.js');

assert.deepEqual(D.axes(new Set(['forward'])), {forward: 1, turn: 0});
assert.deepEqual(D.axes(new Set(['reverse', 'left'])), {forward: -1, turn: 1});
assert.deepEqual(D.axes(new Set()), {forward: 0, turn: 0});
assert.equal(D.label('estop_feedback_unavailable'), '비상정지 피드백 없음');
assert.equal(D.label('unknown_code'), 'unknown_code');
assert.equal(D.label(null), '—');
assert.match(D.headline({mode: 'manual', active: false, latched_reason: 'obstacle', operator: true}), /수동 · 정지 · 장애물 · 조종권 있음 · 가상 출력/);
for (const code of ['unverified_forward_polarity', 'unverified_independent_cutoff', 'unverified_estop_feedback',
                    'unverified_lidar_geometry', 'clearance_geometry_unverified', 'tracking_ambiguous',
                    'controller_command_timeout', 'uwb_left_id_mismatch', 'operator_disconnected'])
  assert.notEqual(D.label(code), code, code);

function element(tag) {
  const handlers = {};
  const node = {tag, children: [], dataset: {}, attributes: {}, textContent: '', handlers,
    get firstChild() { return this.children[0] || null; },
    appendChild(child) { this.children.push(child); return child; },
    removeChild(child) { this.children.splice(this.children.indexOf(child), 1); },
    setAttribute(name, value) { this.attributes[name] = value; },
    addEventListener(name, fn) { handlers[name] = fn; },
    fire(name, event = {}) { if (handlers[name]) handlers[name](event); }};
  return node;
}
function setup(statusValue) {
  const calls = [];
  const anchor = element('div');
  anchor.insertAdjacentElement = (where, panel) => { anchor.inserted = [where, panel]; };
  const buttons = ['forward', 'reverse', 'left', 'right', 'stop'].map(name => {
    const b = element('button'); b.dataset.virtual = name; return b;
  });
  const doc = element('document');
  doc.createElement = element;
  doc.querySelector = selector => selector === '.drive-status' ? anchor : null;
  doc.querySelectorAll = () => buttons;
  const win = element('window');
  const timers = [];
  const fetch = (url, options = {}) => {
    calls.push({url, options, body: options.body ? JSON.parse(options.body) : null});
    if (url === '/api/drive/status')
      return Promise.resolve({ok: statusValue !== null, json: () => Promise.resolve(statusValue)});
    const drive = options.body && JSON.parse(options.body).type === 'drive';
    const result = drive ? {accepted: false, reason: 'latched'} : {accepted: true, reason: null};
    return Promise.resolve({ok: true, json: () => Promise.resolve({result, status: statusValue})});
  };
  return {calls, anchor, buttons, doc, win, timers,
    install: () => D.install(doc, win, {fetch, setInterval: fn => timers.push(fn), EventSource: null})};
}

(async () => {
  // 8765 센싱 화면: 감독기 API가 없으면 패널·명령·타이머를 만들지 않는다.
  let t = setup(null);
  assert.equal(await t.install(), false);
  assert.equal(t.anchor.inserted, undefined);
  assert.equal(t.timers.length, 0);

  const status = {schema: 'smartcart.drive_supervisor.v1', mode: 'manual', active: true, operator: true,
                  blocking_reasons: ['estop_feedback_unavailable', '<img src=x onerror=alert(1)>'], hardware_output: false};
  t = setup(status);
  const link = await t.install();
  assert.ok(link);
  const panel = t.anchor.inserted[1];
  assert.equal(t.anchor.inserted[0], 'afterend');
  const reasons = panel.children.find(child => child.className === 'supervisor-reasons');
  assert.equal(reasons.children[0].textContent, '비상정지 피드백 없음');
  assert.equal(reasons.children[1].textContent, '<img src=x onerror=alert(1)>');  // 텍스트로만 표시
  assert.equal(t.timers.length, 1);

  // 버튼 응답은 표시하고, 주기 명령 응답(래치 거부)은 그 표시를 덮지 않는다.
  const row = panel.children.find(child => child.className === 'supervisor-row');
  const resultLine = panel.children.find(child => child.className === 'supervisor-result');
  row.children.find(button => button.dataset.drive === 'claim').fire('click');
  await new Promise(resolve => setImmediate(resolve));
  assert.equal(resultLine.textContent, '응답: 수락');
  t.timers[0]();
  await new Promise(resolve => setImmediate(resolve));
  assert.equal(resultLine.textContent, '응답: 수락');

  t.buttons[0].fire('pointerdown', {pointerId: 1});
  t.buttons[2].fire('pointerdown', {pointerId: 2});
  t.timers[0]();
  await new Promise(resolve => setImmediate(resolve));
  let sent = t.calls.filter(call => call.url === '/api/drive/command').pop();
  assert.equal(sent.options.method, 'POST');
  assert.equal(sent.options.credentials, 'same-origin');
  assert.deepEqual([sent.body.type, sent.body.forward, sent.body.turn], ['drive', 1, 1]);
  const firstSeq = sent.body.seq;

  t.buttons[0].fire('pointerup', {pointerId: 1});
  t.buttons[2].fire('pointerup', {pointerId: 2});
  await new Promise(resolve => setImmediate(resolve));
  t.timers[0]();
  await new Promise(resolve => setImmediate(resolve));
  sent = t.calls.filter(call => call.url === '/api/drive/command').pop();
  assert.deepEqual([sent.body.forward, sent.body.turn], [0, 0]);
  assert.ok(sent.body.seq > firstSeq);

  t.buttons[4].fire('pointerdown', {pointerId: 3});
  sent = t.calls.filter(call => call.url === '/api/drive/command').pop();
  assert.equal(sent.body.type, 'stop');
  await new Promise(resolve => setImmediate(resolve));  // 정지 응답이 먼저 반영된다.

  link.render({...status, mode: 'follow'});
  t.timers[0]();
  await new Promise(resolve => setImmediate(resolve));
  sent = t.calls.filter(call => call.url === '/api/drive/command').pop();
  assert.equal(sent.body.type, 'heartbeat');

  t.win.fire('blur');
  sent = t.calls.filter(call => call.url === '/api/drive/command').pop();
  assert.equal(sent.body.type, 'stop');
  assert.equal(sent.options.keepalive, true);
  console.log('drive_link: OK');
})().catch(error => { console.error(error); process.exit(1); });
