/* 현장 가정을 이용한 화면 전용 비교. 추적 결과와 구동 명령을 만들지 않는다. */
(function (root, factory) {
  const api = factory();
  if (typeof module === 'object' && module.exports) module.exports = api;
  else root.FieldPreview = api;
})(typeof globalThis !== 'undefined' ? globalThis : this, function () {
  'use strict';
  const BASELINE_M = 0.45;
  const FORWARD_FROM_LIDAR_M = 0.10;
  const MAX_SENSOR_AGE_MS = 300;
  const MAX_PAIR_SKEW_MS = 100;
  const MAX_DISAGREEMENT_M = 0.60;
  const ASSOCIATION_DISTANCE_M = 0.60;
  const SMOOTH_WINDOW_MS = 600;
  const SMOOTH_MAX_SAMPLES = 5;
  const SMOOTH_ALPHA = 0.3;
  const finite = (value, low, high) => typeof value === 'number' && Number.isFinite(value) && value >= low && value <= high;
  const empty = (state, label) => ({state, label, left: null, right: null, average: null,
    disagreementAverage: null, residualM: null, candidate: null, candidateCount: null,
    provisional: true, driveAuthorized: false});

  function derive(dual, tracking, options = {}) {
    if (options.enabled !== true) return empty('off', '실험 표시 꺼짐');
    if (options.paused === true) return empty('paused', '화면 정지');
    if (options.sourceMode !== 'ros' || options.connected !== true || options.lidarUsable !== true ||
        options.trackingConnected !== true)
      return empty('waiting', '실측 LiDAR와 UWB 연결 대기');
    if (!dual || dual.valid !== true || !dual.sensors) return empty('waiting', '신선한 UWB 수신 대기');
    if (!tracking || tracking.sourceLabel !== 'ROS · 검수 전' || !tracking.transform || !Array.isArray(tracking.clusters))
      return empty('waiting', 'LiDAR 장착 좌표·군집 대기');
    const transform = tracking.transform;
    if (!finite(transform.yaw, -180, 180) || !finite(transform.offsetX, -2, 2) || !finite(transform.offsetY, -2, 2))
      return empty('error', 'LiDAR 좌표 형식 오류');
    const left = dual.sensors.uwb_left, right = dual.sensors.uwb_right;
    const single = dual.profile === 'single_left';
    for (const sensor of single ? [left] : [left, right]) {
      if (!sensor || sensor.valid !== true || sensor.state !== 'live' ||
          !finite(sensor.rangeM, 0.05, 20) || !finite(sensor.angleDeg, -180, 180) ||
          !finite(sensor.ageMs, 0, MAX_SENSOR_AGE_MS)) return empty('waiting', '좌우 UWB의 새 관측 대기');
    }
    if (!single && Math.abs(left.ageMs - right.ageMs) > MAX_PAIR_SKEW_MS)
      return empty('skew', '좌우 수신 시각 차이 초과');

    // BU04 오른쪽 양수 각도를 좌측 양수로 바꾸고, LiDAR와 평행이라는 가정을 적용한다.
    const yaw = transform.yaw * Math.PI / 180;
    const project = (sensor, lateral) => {
      const angle = -sensor.angleDeg * Math.PI / 180;
      const x = FORWARD_FROM_LIDAR_M + sensor.rangeM * Math.cos(angle);
      const y = lateral + sensor.rangeM * Math.sin(angle);
      return {x: transform.offsetX + x * Math.cos(yaw) - y * Math.sin(yaw),
        y: transform.offsetY + x * Math.sin(yaw) + y * Math.cos(yaw)};
    };
    const l = project(left, BASELINE_M / 2);
    if (single) {
      // 화면 비교 전용 가정이다. 실제 장착·높이·거리 모델과 구동 조건을 검증하지 않는다.
      const result = {...empty('single', '왼쪽 위치 가정 · 수평거리·장착 검수 전'), left: l, average: l};
      if (options.clustersUsable !== true) return result;
      if (tracking.clusters.length > 128) return empty('error', '군집 수 오류');
      const candidates = [];
      for (const cluster of tracking.clusters) {
        if (!cluster || !Number.isSafeInteger(cluster.id) || cluster.id < 0 ||
            !finite(cluster.x, -100, 100) || !finite(cluster.y, -100, 100) || !finite(cluster.radius, 0, 1))
          return empty('error', '군집 형식 오류');
        if (Math.hypot(cluster.x - l.x, cluster.y - l.y) <= ASSOCIATION_DISTANCE_M + cluster.radius)
          candidates.push(cluster);
      }
      result.candidateCount = candidates.length;
      if (candidates.length === 1) {
        const candidate = candidates[0];
        result.candidate = {x: candidate.x, y: candidate.y, clusterId: candidate.id};
        result.label = '왼쪽 가정 위치 · 인접 군집 1개 · 사람 판별 아님';
      } else if (candidates.length > 1) result.label = '왼쪽 가정 위치 · 군집 여러 개 · 선택 보류';
      else result.label = '왼쪽 가정 위치 · 인접 군집 없음';
      return result;
    }
    const r = project(right, -BASELINE_M / 2);
    const residual = Math.hypot(l.x - r.x, l.y - r.y);
    if (!finite(residual, 0, 50)) return empty('error', '가정 위치 계산 오류');
    const result = {...empty('disagreement', '좌우 가정 위치 불일치 · 평균 보류'), left: l, right: r, residualM: residual};
    if (residual > MAX_DISAGREEMENT_M) {
      // 불일치 평균은 화면의 십자 표시만 허용한다. 군집 연관에는 절대 사용하지 않는다.
      result.disagreementAverage = {x: (l.x + r.x) / 2, y: (l.y + r.y) / 2};
      return result;
    }
    result.state = 'position';
    result.label = '좌우 평균 가정 위치 · 검수 전';
    result.average = {x: (l.x + r.x) / 2, y: (l.y + r.y) / 2};
    if (options.clustersUsable !== true) {
      result.label = '좌우 평균 가정 위치 · LiDAR 군집 대기';
      return result;
    }
    if (tracking.clusters.length > 128) return {...empty('error', '군집 수 오류')};
    const candidates = [];
    for (const cluster of tracking.clusters) {
      if (!cluster || !Number.isSafeInteger(cluster.id) || cluster.id < 0 ||
          !finite(cluster.x, -100, 100) || !finite(cluster.y, -100, 100) || !finite(cluster.radius, 0, 1))
        return empty('error', '군집 형식 오류');
      const distance = Math.hypot(cluster.x - result.average.x, cluster.y - result.average.y);
      if (distance <= ASSOCIATION_DISTANCE_M + cluster.radius) candidates.push(cluster);
    }
    result.candidateCount = candidates.length;
    if (candidates.length === 1) {
      const candidate = candidates[0];
      result.candidate = {x: candidate.x, y: candidate.y, clusterId: candidate.id};
      result.label = '인접 군집 1개 · 사람 판별 아님';
    } else if (candidates.length > 1) result.label = '인접 군집 여러 개 · 선택 보류';
    else result.label = '인접 군집 없음 · 위치 비교 중';
    return result;
  }

  function createSmoother() {
    // 필터 이력은 탭 메모리에만 두며 신선도·좌우 불일치·군집 판정을 바꾸지 않는다.
    let samples = [], point = null, kind = null, leftSeq = null, rightSeq = null, lastTime = null, frameKey = null;
    const reset = () => { samples = []; point = kind = leftSeq = rightSeq = lastTime = frameKey = null; };
    const median = values => {
      const sorted = values.slice().sort((a, b) => a - b), mid = Math.floor(sorted.length / 2);
      return sorted.length % 2 ? sorted[mid] : (sorted[mid - 1] + sorted[mid]) / 2;
    };
    function update(preview, meta) {
      if (!preview || !['position', 'disagreement'].includes(preview.state)) { reset(); return preview; }
      const field = preview.state === 'position' ? 'average' : 'disagreementAverage';
      const input = preview[field];
      if (!input || !finite(input.x, -100, 100) || !finite(input.y, -100, 100) || !meta ||
          !finite(meta.nowMs, 0, 1e12) || typeof meta.frameKey !== 'string' || meta.frameKey.length > 128 ||
          !Number.isSafeInteger(meta.leftSeq) || meta.leftSeq < 1 ||
          !Number.isSafeInteger(meta.rightSeq) || meta.rightSeq < 1) { reset(); return preview; }
      if (kind !== preview.state || frameKey !== null && frameKey !== meta.frameKey ||
          lastTime !== null && (meta.nowMs < lastTime || meta.nowMs - lastTime > SMOOTH_WINDOW_MS) ||
          leftSeq !== null && (meta.leftSeq < leftSeq || meta.rightSeq < rightSeq)) reset();
      kind = preview.state; frameKey = meta.frameKey;
      // 같은 관측을 되풀이해도 필터를 갱신하거나 원래 데이터 수명을 늘리지 않는다.
      const nextPair = leftSeq === null || meta.leftSeq > leftSeq && meta.rightSeq > rightSeq;
      if (nextPair) {
        samples = samples.filter(sample => meta.nowMs - sample.time <= SMOOTH_WINDOW_MS);
        samples.push({x: input.x, y: input.y, time: meta.nowMs});
        if (samples.length > SMOOTH_MAX_SAMPLES) samples.shift();
        const centre = {x: median(samples.map(sample => sample.x)), y: median(samples.map(sample => sample.y))};
        point = point === null ? centre : {x: point.x + SMOOTH_ALPHA * (centre.x - point.x),
          y: point.y + SMOOTH_ALPHA * (centre.y - point.y)};
        leftSeq = meta.leftSeq; rightSeq = meta.rightSeq;
      }
      lastTime = meta.nowMs;
      return point === null ? preview : {...preview, [field]: {...point}, displaySmoothed: true};
    }
    return {update, reset};
  }
  return {derive, createSmoother};
});
