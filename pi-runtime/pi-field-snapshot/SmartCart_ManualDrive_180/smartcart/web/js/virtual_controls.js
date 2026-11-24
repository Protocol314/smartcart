/* 브라우저 안의 가상 조종 표시. 네트워크나 GPIO에 명령을 보내지 않는다. */
(function (root, factory) {
  const api = factory();
  if (typeof module === 'object' && module.exports) module.exports = api;
  else { root.VirtualControls = api; api.install(root.document, root); }
})(typeof globalThis !== 'undefined' ? globalThis : this, function () {
  'use strict';
  const outputs = Object.freeze({forward:[1,1], reverse:[-1,-1], left:[-1,1], right:[1,-1], stop:[0,0]});
  function compute(direction) {
    return Object.prototype.hasOwnProperty.call(outputs, direction) ? outputs[direction].slice() : [0, 0];
  }
  function mix(forward, turn) {
    const left = forward - turn, right = forward + turn;
    const scale = Math.max(1, Math.abs(left), Math.abs(right));
    return [left / scale, right / scale];
  }
  function install(document, win) {
    const output = document.getElementById('virtualControlOutput');
    const status = document.getElementById('virtualControlStatus');
    const limit = document.getElementById('virtualRpmLimit');
    const rpmOutput = document.getElementById('virtualRpmOutput');
    const buttons = document.querySelectorAll('[data-virtual]');
    if (!output || !status || !buttons.length) return;
    const pressed = new Map();
    function update() {
      const names = new Set(pressed.values());
      const forward = Number(names.has('forward')) - Number(names.has('reverse'));
      const turn = Number(names.has('left')) - Number(names.has('right'));
      const values = mix(forward, turn);
      output.textContent = values.map(value => Number(value.toFixed(2))).join(' / ');
      status.textContent = values.some(Boolean) ? '가상 방향 계산 중 · 실제 구동 잠금' : '정지 · 실제 구동 잠금';
      if (rpmOutput) {
        const raw = limit ? Number(limit.value) : 0;
        const safe = Number.isFinite(raw) ? Math.max(0, Math.min(10000, raw)) : 0;
        rpmOutput.textContent = values.map(value => Math.round(value * safe)).join(' / ');
      }
    }
    function stop() { pressed.clear(); update(); }
    function begin(id, direction) {
      if (direction === 'stop') { stop(); return; }
      // 한 축에 한 입력만 허용하고 전후진과 회전은 동시에 계산한다.
      const axis = ['forward', 'reverse'].includes(direction) ? ['forward', 'reverse'] : ['left', 'right'];
      if ([...pressed.values()].some(item => axis.includes(item))) return;
      pressed.set(id, direction); update();
    }
    for (const button of buttons) {
      const direction = button.dataset.virtual;
      button.addEventListener('pointerdown', event => {
        event.preventDefault();
        if (pressed.has(event.pointerId)) return;
        begin(event.pointerId, direction);
        if (pressed.has(event.pointerId) && button.setPointerCapture) button.setPointerCapture(event.pointerId);
      });
      for (const name of ['pointerup', 'pointercancel', 'lostpointercapture'])
        button.addEventListener(name, event => { if (pressed.delete(event.pointerId)) update(); });
      button.addEventListener('keydown', event => {
        if (event.key !== ' ' && event.key !== 'Enter') return;
        event.preventDefault();
        const id = 'keyboard-' + direction;
        if (!pressed.has(id)) begin(id, direction);
      });
      button.addEventListener('keyup', event => {
        if ((event.key === ' ' || event.key === 'Enter') && pressed.delete('keyboard-' + direction)) update();
      });
    }
    if (limit) limit.addEventListener('input', update);
    win.addEventListener('blur', stop);
    win.addEventListener('pagehide', stop);
    document.addEventListener('visibilitychange', () => { if (document.hidden) stop(); });
    stop();
  }
  return {compute, mix, install};
});
