/* UWB 표시 검증과 출처 격리. 위치를 추정하거나 장치 출력을 제어하지 않는다. */
(function (root, factory) {
  const api = factory();
  if (typeof module === 'object' && module.exports) module.exports = api;
  else root.SensingCore = api;
})(typeof globalThis !== 'undefined' ? globalThis : this, function () {
  'use strict';
  const finite = value => typeof value === 'number' && Number.isFinite(value);
  const boundedText = value => typeof value === 'string' ? value.slice(0, 160) : '';
  const states = Object.freeze({ waiting: '수신 대기', live: '실측 수신', synthetic: '합성 · 비실측', stale: '수신 지연', error: '입력 오류', offline: '연결 끊김', invalid: '유효 위치 없음', disabled: '이 소스에 UWB 없음' });

  function deriveUwb(input, options = {}) {
    const mode = options.mode || 'live';
    const base = { state: 'waiting', label: states.waiting, marker: null, measurement: null,
      ageMs: null, receiveHz: null, sourceLabel: '—', synthetic: false, reason: '', rejectedFrames: 0 };
    const fail = (state, reason = '') => ({ ...base, state, label: states[state], reason,
      measurement: base.measurement ? { ...base.measurement, valid: false, position: null } : null });
    if (mode === 'sample' || mode === 'file' || options.serverMode === 'replay') return fail('disabled', 'LiDAR 단독 자료 · UWB 기록을 합치지 않습니다.');
    if (mode === 'live' && options.connected === false) return fail('offline', '서버 연결을 확인하십시오.');
    if (!input) return base;
    if (input.schema !== 'smartcart.uwb.v1' || !['serial', 'ros', 'demo', 'off'].includes(input.source_mode) ||
        !['waiting', 'live', 'synthetic', 'stale', 'error', 'off'].includes(input.state)) return fail('error', 'UWB 메시지 형식 오류');
    if (input.source_mode === 'off' || input.state === 'off') return fail('disabled', '서버에서 UWB 입력을 사용하지 않도록 설정했습니다.');
    base.synthetic = input.source_mode === 'demo' || input.synthetic === true;
    base.sourceLabel = base.synthetic ? '합성 UWB' : input.source_mode === 'ros' ? 'ROS / UWB' : 'ESP32 / USB';
    base.receiveHz = finite(input.receive_hz) && input.receive_hz >= 0 && input.receive_hz <= 10000 ? input.receive_hz : null;
    base.rejectedFrames = Number.isInteger(input.rejected_frames) && input.rejected_frames >= 0 ? input.rejected_frames : 0;
    const expectedDemo = mode === 'demo' || options.serverMode === 'demo' || options.serverSynthetic === true;
    if (base.synthetic !== expectedDemo || base.synthetic && input.state === 'live' || !base.synthetic && input.state === 'synthetic')
      return fail('error', '실측과 합성 소스가 일치하지 않아 UWB를 숨겼습니다.');
    const elapsed = finite(options.elapsedMs) ? Math.max(0, options.elapsedMs) : 0;
    const age = finite(input.age_ms) && input.age_ms >= 0 ? input.age_ms + elapsed : null;
    const timeout = finite(input.stale_after_ms) && input.stale_after_ms > 0 ? Math.min(1500, input.stale_after_ms) : 500;
    base.ageMs = age;
    if (input.state === 'error') return fail('error', boundedText(input.error) || 'UWB 입력 오류');
    if (input.state === 'waiting') return fail('waiting');
    if (input.state === 'stale' || age === null || age > timeout) return fail('stale', '마지막 UWB 위치의 유효 시간이 지났습니다.');
    const m = input.measurement;
    if (!m || typeof m !== 'object') return fail('invalid');
    const idValid = Number.isInteger(m.tag_id) && m.tag_id >= 0 && m.tag_id <= 255 &&
      Number.isInteger(m.accepted_id) && m.accepted_id >= 0 && m.accepted_id <= 15;
    const addressValid = typeof m.tag_address === 'string' && /^0x[0-9a-f]{4}$/i.test(m.tag_address);
    base.measurement = { tag_address: addressValid ? m.tag_address : '—', tag_id: idValid ? m.tag_id : null,
      accepted_id: idValid ? m.accepted_id : null, authenticated: m.authenticated === true && idValid && m.tag_id === m.accepted_id,
      valid: m.valid === true, raw_distance_m: finite(m.raw_distance_m) && m.raw_distance_m >= 0 && m.raw_distance_m <= 1000 ? m.raw_distance_m : null,
      filtered_distance_m: finite(m.filtered_distance_m) && m.filtered_distance_m >= 0 && m.filtered_distance_m <= 1000 ? m.filtered_distance_m : null,
      angle_deg: finite(m.angle_deg) && Math.abs(m.angle_deg) <= 180 ? m.angle_deg : null,
      firmware_state: boundedText(m.firmware_state), position: null };
    const p = m.position;
    if (!base.measurement.authenticated) return fail('invalid', 'ID 불일치 또는 인증되지 않은 태그');
    if (m.valid !== true || !addressValid || base.measurement.filtered_distance_m === null || base.measurement.angle_deg === null ||
        !p || p.frame_id !== 'base_link' || !finite(p.x) || !finite(p.y) || Math.abs(p.x) > 1000 || Math.abs(p.y) > 1000)
      return fail('invalid', '유효한 base_link 좌표가 없습니다.');
    base.measurement.position = { x: p.x, y: p.y, frame_id: 'base_link' };
    const withinView = !finite(options.radius) || Math.hypot(p.x, p.y) <= options.radius;
    return { ...base, state: base.synthetic ? 'synthetic' : 'live', label: base.synthetic ? states.synthetic : states.live,
      marker: withinView ? { x: p.x, y: p.y, label: `UWB ${m.tag_address}`, synthetic: base.synthetic } : null,
      reason: withinView ? '' : '태그가 표시 반경 밖에 있습니다.' };
  }

  function syntheticUwb(t) {
    const x = 1.7 + .35 * Math.sin(t * .45), y = .7 * Math.sin(t * .32);
    const distance = Math.hypot(x, y);
    return { schema: 'smartcart.uwb.v1', state: 'synthetic', source_mode: 'demo', age_ms: 0,
      stale_after_ms: 1500, receive_hz: 2, rejected_frames: 0, error: null,
      measurement: { tag_address: '0x6E19', tag_id: 0, accepted_id: 0, authenticated: true, valid: true,
        raw_distance_m: distance, filtered_distance_m: distance, angle_deg: -Math.atan2(y, x) * 180 / Math.PI,
        position: { x, y, frame_id: 'base_link' }, firmware_state: '합성 상태' } };
  }

  function integratedState({ mode, lidarUsable, lidarSynthetic, uwb, paused, serverReady }) {
    if (paused) return { state: 'paused', label: '화면 정지', ready: false };
    if (mode === 'sample' || mode === 'file') return { state: 'local', label: 'LiDAR 자료 보기', ready: false };
    if (lidarSynthetic || mode === 'demo' || uwb.synthetic) return { state: 'synthetic', label: '합성 데모 · 비실측', ready: false };
    if (lidarUsable && uwb.state === 'live' && serverReady !== false) return { state: 'ready', label: '두 센서 수신 중', ready: true };
    if (lidarUsable || uwb.state === 'live') return { state: 'degraded', label: '일부 센서 확인 필요', ready: false };
    return { state: 'waiting', label: '센서 연결 확인', ready: false };
  }
  return { deriveUwb, syntheticUwb, integratedState };
});
