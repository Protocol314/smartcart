/* Actual network input for the existing SmartCart controls. */
(function (root, factory) {
  const api = factory();
  if (typeof module === 'object' && module.exports) module.exports = api;
  else { root.ManualControls = api; api.install(root.document, root); }
})(typeof globalThis !== 'undefined' ? globalThis : this, function () {
  'use strict';
  function axes(directions) {
    const names = new Set(directions);
    return {
      forward: Number(names.has('forward')) - Number(names.has('reverse')),
      turn: Number(names.has('left')) - Number(names.has('right'))
    };
  }
  function install(document, win) {
    const $ = id => document.getElementById(id);
    const buttons = Array.from(document.querySelectorAll('[data-virtual]'));
    if (!$('driveConnect') || !buttons.length) return;
    let socket = null, state = null, sequence = 0, lastStateAt = 0;
    let armSequence = null, error = '', lastStopInputAt = -Infinity, staleStopSent = false;
    const pressed = new Map();
    const clock = () => win.performance.now();

    function fresh() { return state && clock() - lastStateAt < 350; }
    function modeAllowsDrive() {
      return state && (!state.hardware || $('source').value === 'live');
    }
    function enabled() {
      return fresh() && modeAllowsDrive() && state.armed && armSequence !== null
        && state.ack_seq >= armSequence;
    }
    function connected() { return socket && socket.readyState === 1 && fresh(); }
    function selectedPwm() {
      const value = Number($('virtualRpmLimit').value);
      return Number.isFinite(value) ? Math.max(0, Math.min(180, value)) : 0;
    }
    function paintPressed() {
      const names = new Set(pressed.values());
      for (const button of buttons) {
        const down = names.has(button.dataset.virtual);
        button.classList.toggle('is-held', down);
        button.setAttribute('aria-pressed', String(down));
      }
    }
    function clear() { pressed.clear(); paintPressed(); }
    function send(type) {
      if (!socket || socket.readyState !== 1) return null;
      if (type === 'stop') {
        socket.send(JSON.stringify({type: 'stop'}));
        return null;
      }
      if (!fresh()) return null;
      if (socket.bufferedAmount > 4096) {
        clear(); armSequence = null; socket.close(); return null;
      }
      const a = type === 'drive' && enabled() ? axes(pressed.values()) : {forward: 0, turn: 0};
      const seq = ++sequence;
      socket.send(JSON.stringify({
        type, seq, epoch: state.epoch, stamp: state.stamp,
        forward: a.forward, turn: a.turn, pwm: selectedPwm()
      }));
      return seq;
    }
    function stop(reason) {
      armSequence = null;
      clear();
      send('stop');
      if (reason) error = reason;
      render();
    }
    function render() {
      const live = connected();
      const ready = enabled();
      $('driveBackend').textContent = state
        ? (state.hardware ? 'GPIO 실제 출력' : 'MOCK · 모터 미연결') : '조종 서버 미연결';
      $('driveBackend').dataset.hardware = String(!!(state && state.hardware));
      const messages = {
        connected: '연결됨 · 방향 버튼을 놓고 활성화',
        disconnected: '연결 끊김', software_stop: 'STOP · 재활성화 필요',
        command_timeout: '통신 만료 · 다시 활성화', loop_delay: '제어 지연 · 다시 활성화',
        invalid_command: '명령 오류 · 다시 연결', gpio_fault: 'GPIO 오류 · 물리 정지 후 서버 확인',
        disarmed: '정지', armed: '주행 활성화', closed: '서버 종료'
      };
      const command = axes(pressed.values());
      $('virtualControlStatus').textContent = error || (
        !live ? '조종 서버 연결 대기' :
        !modeAllowsDrive() ? '센서 SOURCE를 Pi 센싱 서버로 바꾸세요' :
        ready && !command.forward && command.turn ? '제자리 회전 / 방향 전환 시 정지 후 회전' :
        ready ? '활성화 · 누르는 동안 주행 / 놓으면 감속' :
        messages[state.reason] || '정지 · 활성화 필요'
      );
      const output = live ? state.pwm : null;
      $('virtualControlOutput').textContent = output ? output.join(' / ') : '— / —';
      $('virtualRpmOutput').textContent = live ? state.estimated_rpm.join(' / ') : '— / —';
      $('driveEstimatedSpeed').textContent = live ? state.estimated_kmh.toFixed(2) : '—';
      $('driveArm').disabled = !live || !modeAllowsDrive() || !!state.fault || !!state.armed || armSequence !== null;
      $('driveArm').textContent = ready ? '주행 활성화됨' : armSequence !== null ? '활성화 요청 중…' : '주행 활성화';
      for (const button of buttons) {
        if (button.dataset.virtual !== 'stop') button.disabled = !ready;
      }
    }

    $('driveConnect').addEventListener('click', () => {
      if (win.location.protocol === 'file:') {
        error = 'HTML 파일 대신 Pi IP:8080 주소로 여세요'; render(); return;
      }
      const key = $('driveKey').value.trim();
      if (!key) { error = '라즈베리 터미널에 표시된 연결키를 입력하세요'; render(); return; }
      stop();
      if (socket) socket.close();
      state = null; error = ''; armSequence = null;
      const current = new win.WebSocket(
        (win.location.protocol === 'https:' ? 'wss://' : 'ws://') + win.location.host + '/ws/drive');
      socket = current;
      render();
      current.onopen = () => {
        if (current === socket) current.send(JSON.stringify({key}));
      };
      current.onmessage = event => {
        if (current !== socket) return;
        try {
          const value = JSON.parse(event.data);
          if (value.error) { error = value.error; stop(error); return; }
          if (value.schema !== 'smartcart.manual.v1') throw new Error('Invalid status');
          state = value; lastStateAt = clock();
          staleStopSent = false;
          if (!value.armed && armSequence !== null && value.ack_seq >= armSequence) {
            armSequence = null; clear();
          }
          render();
        } catch (_) { stop('조종 상태 오류'); current.close(); }
      };
      current.onclose = () => {
        if (current !== socket) return;
        armSequence = null; state = null; clear();
        if (!error) error = '연결 끊김 · 다시 연결하세요';
        render();
      };
      current.onerror = () => {
        if (current === socket) stop('조종 서버 접속 실패 · 주소와 서버 터미널 확인');
      };
    });
    $('driveArm').addEventListener('click', () => {
      if (!connected() || !modeAllowsDrive() || state.armed || armSequence !== null || pressed.size) return;
      error = '';
      armSequence = send('arm');
      render();
    });
    for (const button of buttons) {
      const direction = button.dataset.virtual;
      button.addEventListener('contextmenu', event => event.preventDefault());
      function begin(id) {
        if (direction === 'stop') {
          lastStopInputAt = clock();
          stop('STOP · 재활성화 필요');
          return;
        }
        if (!enabled() || pressed.has(id)) return;
        const axis = ['forward', 'reverse'].includes(direction)
          ? ['forward', 'reverse'] : ['left', 'right'];
        if ([...pressed.values()].some(value => axis.includes(value))) return;
        pressed.set(id, direction); paintPressed(); send('drive'); render();
      }
      button.addEventListener('pointerdown', event => {
        event.preventDefault(); begin(event.pointerId);
        if (pressed.has(event.pointerId)) button.setPointerCapture(event.pointerId);
      });
      button.addEventListener('pointerup', event => {
        if (pressed.delete(event.pointerId)) { paintPressed(); send('drive'); render(); }
      });
      button.addEventListener('pointercancel', () => stop('터치 취소 · 재활성화 필요'));
      button.addEventListener('lostpointercapture', event => {
        if (pressed.has(event.pointerId)) stop('터치 해제 · 재활성화 필요');
      });
      button.addEventListener('keydown', event => {
        if (event.key === ' ' || event.key === 'Enter') {
          event.preventDefault(); if (!event.repeat) begin('button-' + direction);
        }
      });
      button.addEventListener('keyup', event => {
        if (event.key === ' ' || event.key === 'Enter') {
          event.preventDefault();
          if (pressed.delete('button-' + direction)) { paintPressed(); send('drive'); render(); }
        }
      });
      if (direction === 'stop') button.addEventListener('click', () => {
        // 터치·키보드 입력 후 생성된 클릭은 같은 정지 요청이다.
        if (clock() - lastStopInputAt >= 1000) stop('STOP · 재활성화 필요');
      });
    }
    $('virtualRpmLimit').addEventListener('input', () => {
      const value = Number($('virtualRpmLimit').value);
      if (!Number.isFinite(value) || value < 0 || value > 180) {
        stop('출력은 PWM 0~180 범위로 입력하세요'); return;
      }
      send('drive');
    });
    for (const id of ['source', 'fileInput']) $(id).addEventListener('change', () => stop('입력 소스 변경 · 재활성화 필요'));
    for (const id of ['openSettings', 'pauseView', 'openFile'])
      $(id).addEventListener('click', () => stop('화면 조작 · 재활성화 필요'));
    win.addEventListener('blur', () => stop('창 포커스 해제 · 재활성화 필요'));
    win.addEventListener('pagehide', () => stop());
    document.addEventListener('visibilitychange', () => {
      if (document.hidden) stop('화면 숨김 · 재활성화 필요');
    });
    document.addEventListener('keydown', event => {
      if (event.code === 'Space' && !['INPUT', 'TEXTAREA', 'SELECT'].includes(event.target.tagName)) {
        event.preventDefault(); stop('STOP · 재활성화 필요');
      }
    });
    win.setInterval(() => {
      if (!socket || socket.readyState !== 1) return;
      if (!fresh() || document.hidden) {
        if (!staleStopSent) {
          staleStopSent = true;
          stop(document.hidden ? '화면 숨김 · 재활성화 필요' : '조종 통신 지연 · 재활성화 필요');
        }
        return;
      }
      send('drive');
    }, 100);
    render();
    win.SmartCartManual = {
      stop: () => stop('STOP · 재활성화 필요'),
      get state() { return state; },
      get enabled() { return enabled(); },
      get pressed() { return Array.from(pressed.values()); }
    };
  }
  return {axes, install};
});
