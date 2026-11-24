/* 추적 화면의 출처·수명·표시값을 검증한다. 위치를 추정하거나 구동을 승인하지 않는다. */
(function (root, factory) {
  const api = factory();
  if (typeof module === 'object' && module.exports) module.exports = api;
  else root.TrackingCore = api;
})(typeof globalThis !== 'undefined' ? globalThis : this, function () {
  'use strict';
  const finite = value => typeof value === 'number' && Number.isFinite(value);
  const integer = (value, maximum = Number.MAX_SAFE_INTEGER) => Number.isSafeInteger(value) && value >= 0 && value <= maximum;
  const code = value => typeof value === 'string' && /^[A-Za-z0-9_.:-]{1,128}$/.test(value);
  const xy = value => Array.isArray(value) && value.length === 2 && value.every(v => finite(v) && Math.abs(v) <= 100);
  const labels = Object.freeze({ waiting: '입력 대기', acquiring: '대상 확인 중', tracking: '대상 추적 중',
    coasting: '가림 · 예측만 표시', ambiguous: '후보 모호 · 선택 보류', lost: '대상 분실', error: '추적 입력 오류',
    stale: '추적 표시 만료', ended: '서버 재생 종료', off: '추적 사용 안 함', offline: '추적 연결 끊김',
    paused: '화면 정지 · 추적 숨김', local: '로컬 LiDAR 보기', disabled: '추적 수신 대기' });
  function empty(state = 'waiting', reason = '') {
    return { state, label: labels[state] || labels.error, sourceLabel: '—', configured: false,
      clusters: [], selected: null, prediction: null, uwb: null, transform: null,
      trackId: null, confirmationCount: 0, candidateCount: 0, ageMs: null,
      reason, driveAuthorized: false, provisional: true, suppressBackground: false };
  }
  function deriveTracking(input, options = {}) {
    let view = empty();
    const fail = (state, reason, suppressBackground = false) => ({ ...view, state, label: labels[state],
      clusters: [], selected: null, prediction: null, uwb: null,
      trackId: state === 'paused' ? view.trackId : null, reason, suppressBackground });
    if ((options.mode || 'live') !== 'live') return empty('local', '로컬 자료에는 서버 추적을 섞지 않습니다.');
    if (!input) return view;
    if (!input || typeof input !== 'object' || input.schema !== 'smartcart.tracking_view.v1' ||
        !['ros', 'replay', 'off'].includes(input.mode) || !['live', 'replay', 'off'].includes(input.source_mode) ||
        !['waiting', 'running', 'stale', 'ended', 'error', 'off'].includes(input.state) ||
        !integer(input.sequence) || input.drive_authorized !== false || input.provisional !== true ||
        input.frame_id !== 'base_link' || typeof input.valid !== 'boolean') return fail('error', '추적 메시지 형식 오류');
    if (input.mode === 'off' && input.source_mode === 'off') return empty('off');
    if ((input.mode === 'ros' && input.source_mode !== 'live') || (input.mode === 'replay' && input.source_mode !== 'replay') ||
        input.mode === 'off' || input.source_mode === 'off') return fail('error', '추적 출처가 일치하지 않습니다.');
    view.sourceLabel = input.mode === 'ros' ? 'ROS · 검수 전' : '서버 재생 · 비실측';
    view.configured = true;
    if (options.serverSourceMode !== undefined &&
        options.serverSourceMode !== (input.mode === 'ros' ? 'ros' : 'replay'))
      return fail('error', '배경 LiDAR와 추적 결과의 출처가 일치하지 않습니다.', true);
    const transform = input.lidar_transform;
    if (!transform || !finite(transform.yaw_deg) || Math.abs(transform.yaw_deg) > 180 ||
        !Array.isArray(transform.translation_m) || transform.translation_m.length !== 3 ||
        !transform.translation_m.every(v => finite(v) && Math.abs(v) <= 10)) return fail('error', '장착 좌표 형식 오류', true);
    if (Math.abs(transform.translation_m[0]) > 2 || Math.abs(transform.translation_m[1]) > 2)
      return fail('error', '표시 가능한 장착 범위 초과', true);
    view.transform = { yaw: transform.yaw_deg, offsetX: transform.translation_m[0], offsetY: transform.translation_m[1] };
    if (options.connected === false) return fail('offline', '추적 SSE 연결을 확인하십시오.');
    const elapsed = options.elapsedMs === undefined ? 0 : options.elapsedMs;
    if (!finite(elapsed) || elapsed < 0 || !(input.age_ms === null && input.state !== 'running' ||
        finite(input.age_ms) && input.age_ms >= 0 && input.age_ms <= 86400000))
      return fail('error', '추적 경과 시간이 올바르지 않습니다.');
    view.ageMs = input.age_ms === null ? null : input.age_ms + elapsed;
    const remaining = key => input[key] - elapsed;
    for (const key of ['remaining_ms', 'cluster_remaining_ms', 'prediction_remaining_ms', 'uwb_remaining_ms'])
      if (!finite(input[key]) || input[key] < 0 || input[key] > 5000) return fail('error', '추적 표시 수명이 올바르지 않습니다.');
    if (input.state !== 'running') return fail(input.state === 'off' ? 'off' : input.state, labels[input.state] || '추적 입력 대기');
    const result = input.result;
    if (!result || typeof result !== 'object') return fail('error', '추적 결과가 없습니다.');
    const track = result.track, clusters = result.lidar_clusters, uwb = result.uwb_estimate;
    for (const [value, schema] of [[track, 'smartcart.target_track.v1'], [clusters, 'smartcart.lidar_clusters.v1'], [uwb, 'smartcart.uwb_tag_estimate.v1']])
      if (!value || !(value.schema === schema || schema === 'smartcart.uwb_tag_estimate.v1' && value.schema === 'smartcart.single_uwb_estimate.v1') || value.frame_id !== 'base_link' || value.source_mode !== input.source_mode ||
          value.drive_authorized !== false || value.provisional !== true || typeof value.valid !== 'boolean')
        return fail('error', '서버·센서 결과의 출처 또는 구동 금지 표식 불일치');
    if (!Object.hasOwn(labels, track.state) || !['waiting', 'acquiring', 'tracking', 'coasting', 'ambiguous', 'lost', 'error'].includes(track.state) ||
        !integer(track.confirmation_count, 10) || !integer(track.candidate_count, 128) ||
        !(track.track_id === null || integer(track.track_id) && track.track_id > 0) ||
        !(track.invalid_reason === null || code(track.invalid_reason))) return fail('error', '대상 상태 형식 오류');
    if (!Array.isArray(clusters.clusters) || clusters.clusters.length > 128 ||
        !(clusters.invalid_reason === null || code(clusters.invalid_reason))) return fail('error', '군집 수 또는 상태 오류');
    const ids = new Set(), checked = [];
    for (const c of clusters.clusters) {
      if (!c || !integer(c.cluster_id, 2147483647) || ids.has(c.cluster_id) || !xy(c.centroid_xy_m) ||
          !finite(c.radius_m) || c.radius_m < 0 || c.radius_m > 100 || !finite(c.error_radius_m) || c.error_radius_m < 0 || c.error_radius_m > 100 ||
          !finite(c.width_m) || c.width_m < 0 || c.width_m > 10 || !integer(c.point_count, 8192) || c.point_count < 2 ||
          !integer(c.first_ray, 8191) || !integer(c.last_ray, 8191)) return fail('error', '군집 좌표·크기 형식 오류');
      ids.add(c.cluster_id); checked.push({ id: c.cluster_id, x: c.centroid_xy_m[0], y: c.centroid_xy_m[1], radius: c.radius_m });
    }
    if (!clusters.valid && checked.length) return fail('error', '무효 군집에 이전 위치가 남아 있습니다.');
    view.trackId = track.track_id; view.confirmationCount = track.confirmation_count; view.candidateCount = track.candidate_count;
    view.state = track.state; view.label = labels[track.state]; view.reason = track.invalid_reason || '';
    if (input.valid && !track.valid) return fail('error', '추적 유효 표식이 일치하지 않습니다.');
    if (track.valid) {
      if (track.state !== 'tracking' || track.track_id === null || !xy(track.position_xy_m) || !xy(track.velocity_xy_mps) ||
          !finite(track.error_radius_m) || track.error_radius_m < 0 || track.error_radius_m > 100 || track.predicted_position_xy_m !== null ||
          !integer(track.associated_cluster_id, 2147483647) || !ids.has(track.associated_cluster_id)) return fail('error', '유효 추적 위치 형식 오류');
      if (input.valid && remaining('remaining_ms') > 0) view.selected = {
        x: track.position_xy_m[0], y: track.position_xy_m[1], id: track.track_id,
        clusterId: track.associated_cluster_id, error: track.error_radius_m };
      else { view.state = 'stale'; view.label = labels.stale; view.reason = '측정 위치 만료 · 신선한 군집만 표시합니다.'; }
    } else {
      if (track.position_xy_m !== null || track.velocity_xy_mps !== null) return fail('error', '무효 대상에 측정 위치가 남아 있습니다.');
      if (track.predicted_position_xy_m !== null) {
        if (!['coasting', 'acquiring', 'ambiguous'].includes(track.state) || !xy(track.predicted_position_xy_m))
          return fail('error', '예측 표시 형식 오류');
        if (remaining('prediction_remaining_ms') > 0) view.prediction = { x: track.predicted_position_xy_m[0], y: track.predicted_position_xy_m[1], id: track.track_id };
      }
    }
    if (clusters.valid && remaining('cluster_remaining_ms') > 0) view.clusters = checked;
    if (uwb.valid && remaining('uwb_remaining_ms') > 0) {
      if (!xy(uwb.position_xy_m)) return fail('error', '새 UWB 위치 형식 오류');
      view.uwb = { x: uwb.position_xy_m[0], y: uwb.position_xy_m[1] };
    }
    if (options.paused) return fail('paused', '화면 정지 중 선택·예측·군집을 숨깁니다.');
    if (!view.selected && !view.prediction && !view.clusters.length && !view.uwb &&
        Math.max(input.remaining_ms, input.cluster_remaining_ms, input.prediction_remaining_ms, input.uwb_remaining_ms) > 0 &&
        Math.max(remaining('remaining_ms'), remaining('cluster_remaining_ms'), remaining('prediction_remaining_ms'), remaining('uwb_remaining_ms')) <= 0)
      return fail('stale', '추적·군집 표시의 유효 시간이 지났습니다.');
    if (finite(options.radius)) {
      if (view.selected && Math.hypot(view.selected.x, view.selected.y) > options.radius) view.selected = null;
      if (view.prediction && Math.hypot(view.prediction.x, view.prediction.y) > options.radius) view.prediction = null;
      if (view.uwb && Math.hypot(view.uwb.x, view.uwb.y) > options.radius) view.uwb = null;
    }
    return view;
  }
  function acceptEnvelope(previous, payload, receivedAt) {
    const nextSequence = payload && payload.sequence;
    const previousSequence = previous && (previous.lastSequence === undefined ? previous.payload && previous.payload.sequence : previous.lastSequence);
    if (!integer(nextSequence)) return { payload, receivedAt, lastSequence: previousSequence };
    if (!previous || !integer(previousSequence)) return { payload, receivedAt, lastSequence: nextSequence };
    if (nextSequence < previousSequence) return { payload: { schema: 'invalid' }, receivedAt, lastSequence: previousSequence };
    if (nextSequence === previousSequence) {
      if (payload.state !== 'running' || payload.valid === false && previous.payload.valid === true)
        return { payload, receivedAt: previous.receivedAt, lastSequence: previousSequence };
      return previous;
    }
    return { payload, receivedAt, lastSequence: nextSequence };
  }
  return { deriveTracking, acceptEnvelope, labels, empty };
});
