/* 좌우 BU04의 수신·측정 유효성과 만료를 구분한다. 위치나 구동 허가는 만들지 않는다. */
(function (root, factory) {
  const api = factory();
  if (typeof module === 'object' && module.exports) module.exports = api;
  else root.DualUwbCore = api;
})(typeof globalThis !== 'undefined' ? globalThis : this, function () {
  'use strict';
  const roles = ['uwb_left', 'uwb_right'];
  const labels = { waiting: '첫 수신 대기', live: '유효 수신', invalid: '측정 무효', stale: '수신 시간 초과',
    disconnected: 'USB 연결 끊김', error: '수신 오류', offline: '서버 연결 끊김', paused: '화면 정지' };
  const reasons = {invalid_measurement: '거리 또는 각도 범위 무효', id_mismatch: '허용 태그 ID 불일치',
    checksum: '프레임 검사 오류', disconnected: 'USB 연결 끊김', reconnect: '장치 재연결 중',
    stale: '새 측정 수신 대기'};
  const finite = (v, low, high) => typeof v === 'number' && Number.isFinite(v) && v >= low && v <= high;
  const code = v => v === null || typeof v === 'string' && /^[A-Za-z0-9_.:-]{1,128}$/.test(v);
  const empty = state => ({state, label: labels[state], valid: false, rangeM: null, angleDeg: null,
    rangeOut: false, angleOut: false, measurementValid: null, idValid: null, ageMs: null, reason: null, reasonLabel: null});
  function derive(input, options = {}) {
    const result = {valid: false, received: false, state: 'waiting', label: '좌우 수신 대기',
      sensors: Object.fromEntries(roles.map(role => [role, empty('waiting')]))};
    const fail = state => ({...result, state, label: labels[state],
      sensors: Object.fromEntries(roles.map(role => [role, empty(state)]))});
    if (options.paused) return fail('paused');
    if (options.connected === false) return fail('offline');
    if (!input) return result;
    const elapsed = options.elapsedMs === undefined ? 0 : options.elapsedMs;
    if (!finite(elapsed, 0, 86400000) || input.schema !== 'smartcart.dual_uwb_view.v1' ||
        input.source_mode !== 'live' || input.drive_authorized !== false ||
        !['waiting', 'running', 'stale', 'error'].includes(input.state) ||
        !Number.isSafeInteger(input.sequence) || input.sequence < 0 ||
        typeof input.valid !== 'boolean' || typeof input.received !== 'boolean' ||
        !finite(input.remaining_ms, 0, 2000) || !input.sensors ||
        Object.keys(input.sensors).sort().join(',') !== roles.slice().sort().join(',')) return fail('error');
    if (input.state !== 'running') return fail(input.state);
    if (input.remaining_ms <= elapsed) return fail('stale');
    for (const role of roles) {
      const s = input.sensors[role];
      if (!s || !Object.hasOwn(labels, s.state) || !['waiting','live','invalid','stale','disconnected','error'].includes(s.state) ||
          ['valid','connected','measurement_valid','id_valid','range_out_of_bounds','angle_out_of_bounds'].some(k => typeof s[k] !== 'boolean') ||
          !code(s.invalid_reason) || !finite(s.remaining_ms, 0, 1000) ||
          !(s.age_ms === null || finite(s.age_ms, 0, 86400000)) ||
          !(s.range_m === null || finite(s.range_m, .05, 20)) ||
          !(s.raw_angle_deg === null || finite(s.raw_angle_deg, -180, 180)) ||
          s.valid !== (s.state === 'live') || s.valid && (!s.connected || !s.measurement_valid || !s.id_valid ||
            s.range_m === null || s.raw_angle_deg === null || s.invalid_reason !== null)) return fail('error');
      let state = s.state;
      if (['live', 'invalid'].includes(state) && s.remaining_ms <= elapsed) state = 'stale';
      const display = ['live','invalid'].includes(state) && [null,'invalid_measurement','id_mismatch'].includes(s.invalid_reason);
      result.sensors[role] = {...empty(state), valid: state === 'live',
        rangeM: display ? s.range_m : null, angleDeg: display ? s.raw_angle_deg : null,
        rangeOut: display && s.range_out_of_bounds, angleOut: display && s.angle_out_of_bounds,
        measurementValid: display ? s.measurement_valid : null, idValid: display ? s.id_valid : null,
        ageMs: s.age_ms === null ? null : s.age_ms + elapsed, reason: s.invalid_reason,
        reasonLabel: s.invalid_reason ? reasons[s.invalid_reason] || '수신 상태 확인' : null};
    }
    if (input.profile !== undefined && !['dual', 'single_left'].includes(input.profile)) return fail('error');
    const active = input.profile === 'single_left' ? ['uwb_left'] : roles;
    result.profile = input.profile || 'dual';
    result.valid = active.every(role => result.sensors[role].valid);
    result.received = active.every(role => ['live','invalid'].includes(result.sensors[role].state));
    result.state = result.valid ? 'live' : result.received ? 'invalid' : 'waiting';
    const name = result.profile === 'single_left' ? '왼쪽 BU04' : '좌우 BU04';
    result.label = result.valid ? name + ' 유효 수신' : result.received ? name + ' 수신 중 · 무효 표본 있음' : name + ' 수신 상태 확인';
    return result;
  }
  return {derive};
});
