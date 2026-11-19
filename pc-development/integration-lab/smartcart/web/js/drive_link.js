/* 구동 감독기 서버가 제공한 화면에서만 동작한다. 8765 센싱 화면에서는 아무것도 하지 않는다.
 * 서버 응답은 textContent로만 표시한다. 정지는 조종권 없이도 보낸다. */
(function (root, factory) {
  const api = factory();
  if (typeof module === 'object' && module.exports) module.exports = api;
  else { root.DriveLink = api; api.install(root.document, root); }
})(typeof globalThis !== 'undefined' ? globalThis : this, function () {
  'use strict';
  const MODES = Object.freeze({locked: '잠금', manual: '수동', follow: '추종'});
  const REASONS = Object.freeze({
    startup: '시작 후 재무장 필요', mode_change: '모드 변경 · 재무장 필요', mode_locked: '잠금 모드',
    awaiting_inputs: '센서 입력 대기', inputs_expired: '센서 입력 끊김', invalid_inputs: '센서 입력 형식 오류',
    operator_stop: '조종자 정지', operator_disconnected: '조종자 연결 끊김', shutdown: '서버 종료',
    estop_feedback_unavailable: '비상정지 피드백 없음', estop_engaged: '비상정지 눌림',
    unverified_forward_polarity: '전진 극성 미검수', unverified_independent_cutoff: '독립 전원 차단 미검수',
    unverified_estop_feedback: '비상정지 피드백 회로 미검수', unverified_lidar_geometry: 'LiDAR 장착·정지 영역 미검수',
    unverified_uwb_model: 'UWB 거리·각도 모델 미검수', unverified_follow_trial: '저속 추종 실물 시험 전',
    lidar_missing: 'LiDAR 없음', lidar_not_live: 'LiDAR 실측 아님', lidar_invalid: 'LiDAR 무효', lidar_stale: 'LiDAR 만료',
    uwb_status_missing: 'UWB 상태 없음', uwb_left_disconnected: '왼쪽 UWB 단절', uwb_left_id_mismatch: '태그 ID 불일치',
    uwb_left_invalid: '왼쪽 UWB 무효', uwb_left_stale: '왼쪽 UWB 만료', uwb_right_disconnected: '오른쪽 UWB 단절',
    obstacle: '장애물', clearance_unobserved_rays: '미관측 광선(무반사 포함)',
    clearance_geometry_unverified: '정지 영역 미검수', clearance_incomplete_coverage: 'LiDAR 360도 범위 부족',
    tracking_missing: '추적 없음', tracking_ambiguous: '후보 여러 개', tracking_lost: '대상 상실',
    tracking_waiting: '추적 대기', tracking_acquiring: '대상 확인 중', tracking_coasting: '대상 가림',
    tracking_provisional: '추적 임시값', tracking_not_authorized: '추적 구동 미허가', tracking_stale: '추적 만료',
    controller_command_timeout: '조종 명령 만료', controls_not_neutral: '방향 버튼을 놓으세요',
    mode_mismatch: '모드와 다른 입력', latched: '정지 래치 · 재무장 필요', not_operator: '조종권 없음',
    operator_busy: '다른 조종자 사용 중', already_active: '이미 활성', invalid_command: '명령 오류'
  });
  function label(code) {
    if (typeof code !== 'string') return '—';
    return Object.prototype.hasOwnProperty.call(REASONS, code) ? REASONS[code] : code;
  }
  function axes(names) {
    const has = name => names.has(name) ? 1 : 0;
    return {forward: has('forward') - has('reverse'), turn: has('left') - has('right')};
  }
  function headline(status) {
    if (!status || typeof status !== 'object') return '감독기 상태 없음';
    const mode = MODES[status.mode] || '—';
    const state = status.active ? '활성' : status.arming ? '무장 중' : '정지';
    const output = status.hardware_output ? '실제 출력' : '가상 출력';
    const operator = status.operator ? '조종권 있음' : '조종권 없음';
    const latched = status.latched_reason ? ' · ' + label(status.latched_reason) : '';
    return `${mode} · ${state}${latched} · ${operator} · ${output}`;
  }

  function install(document, win, deps) {
    deps = deps || {};
    const fetcher = deps.fetch || (win && win.fetch && win.fetch.bind(win));
    const Source = deps.EventSource || (win && win.EventSource);
    const every = deps.setInterval || (win && win.setInterval && win.setInterval.bind(win));
    if (!document || !fetcher) return Promise.resolve(false);
    const anchor = document.querySelector('.drive-status');
    const buttons = document.querySelectorAll('[data-virtual]');
    if (!anchor || !buttons.length) return Promise.resolve(false);
    const pressed = new Map();
    let status = null, busy = false, seq = Date.now();

    // 버튼 응답만 결과 줄에 표시한다. 50ms 주기 명령의 응답이 거부 사유를 덮지 않게 한다.
    function post(body, options) {
      const {keepalive = false, show = true} = options || {};
      return fetcher('/api/drive/command', {method: 'POST', credentials: 'same-origin', cache: 'no-store',
        keepalive, headers: {'Content-Type': 'application/json'}, body: JSON.stringify(body)})
        .then(response => response.json())
        .then(value => { if (value && value.status) render(value.status, show ? value.result : null); return value; })
        .catch(() => null);
    }
    const panel = document.createElement('section');
    panel.className = 'drive-supervisor';
    panel.setAttribute('aria-label', '구동 감독기');
    const row = document.createElement('div');
    row.className = 'supervisor-row';
    const controls = [['claim', '조종권 받기'], ['release', '반납'], ['mode:locked', '잠금'],
                      ['mode:manual', '수동'], ['mode:follow', '추종'], ['arm', '재무장']];
    for (const [action, text] of controls) {
      const button = document.createElement('button');
      button.type = 'button';
      button.textContent = text;
      button.dataset.drive = action;
      button.addEventListener('click', () => {
        const [type, mode] = action.split(':');
        post(mode ? {type, mode} : {type});
      });
      row.appendChild(button);
    }
    const state = document.createElement('p');
    state.className = 'supervisor-state';
    state.setAttribute('role', 'status');
    const result = document.createElement('p');
    result.className = 'supervisor-result';
    const reasons = document.createElement('ul');
    reasons.className = 'supervisor-reasons';
    panel.appendChild(row); panel.appendChild(state); panel.appendChild(result); panel.appendChild(reasons);

    function render(value, last) {
      status = value;
      state.textContent = headline(value);
      if (last) result.textContent = '응답: ' + (last.accepted ? '수락' : label(last.reason));
      while (reasons.firstChild) reasons.removeChild(reasons.firstChild);
      for (const code of (value && Array.isArray(value.blocking_reasons) ? value.blocking_reasons : []).slice(0, 8)) {
        const item = document.createElement('li');
        item.textContent = label(code);
        reasons.appendChild(item);
      }
    }
    function release() {
      pressed.clear();
      if (status && status.operator) post({type: 'stop'}, {keepalive: true});
    }
    for (const button of buttons) {
      const direction = button.dataset.virtual;
      button.addEventListener('pointerdown', event => {
        if (direction === 'stop') { pressed.clear(); post({type: 'stop'}); return; }
        const axis = ['forward', 'reverse'].includes(direction) ? ['forward', 'reverse'] : ['left', 'right'];
        if ([...pressed.values()].some(item => axis.includes(item))) return;
        pressed.set(event.pointerId, direction);
      });
      for (const name of ['pointerup', 'pointercancel', 'lostpointercapture'])
        button.addEventListener(name, event => { pressed.delete(event.pointerId); });
    }
    function tick() {
      if (busy || !status || !status.operator) return;
      busy = true;
      const body = status.mode === 'manual'
        ? Object.assign({type: 'drive', seq: ++seq}, axes(new Set(pressed.values())))
        : {type: 'heartbeat'};
      post(body, {show: false}).then(() => { busy = false; });
    }
    return fetcher('/api/drive/status', {credentials: 'same-origin', cache: 'no-store'})
      .then(response => response.ok ? response.json() : null).catch(() => null)
      .then(value => {
        if (!value || value.schema !== 'smartcart.drive_supervisor.v1') return false;
        anchor.insertAdjacentElement('afterend', panel);
        render(value);
        if (win && win.addEventListener) { win.addEventListener('blur', release); win.addEventListener('pagehide', release); }
        if (document.addEventListener) document.addEventListener('visibilitychange', () => { if (document.hidden) release(); });
        if (Source) { const events = new Source('/api/drive/events'); events.onmessage = event => { try { render(JSON.parse(event.data)); } catch (_) { /* 무시 */ } }; }
        if (every) every(tick, 50);
        return {tick, render, pressed};
      });
  }
  return {axes, label, headline, install, REASONS};
});
