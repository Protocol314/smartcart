'use strict';
/* 실제 장치를 열지 않고 표시 가정과 만료·불일치 처리를 검증한다. */
const assert = require('node:assert/strict');
const P = require('../web/js/field_preview.js');
let passed = 0;
function test(name, run) { run(); passed++; console.log('통과', name); }
function pair() {
  const distance = Math.hypot(2, .225);
  const angle = Math.atan2(.225, 2) * 180 / Math.PI;
  return {valid: true, sensors: {
    uwb_left: {valid: true, state: 'live', rangeM: distance, angleDeg: angle, ageMs: 20},
    uwb_right: {valid: true, state: 'live', rangeM: distance, angleDeg: -angle, ageMs: 20}}};
}
function tracking(clusters = []) {
  return {sourceLabel: 'ROS · 검수 전', transform: {yaw: 0, offsetX: 0, offsetY: 0}, clusters};
}
const live = {enabled: true, paused: false, sourceMode: 'ros', connected: true,
  lidarUsable: true, trackingConnected: true, clustersUsable: true};
test('왼쪽 단독 관측은 수평거리 가정의 화면 전용 위치와 인접 군집을 표시', () => {
  const d = pair(); d.profile = 'single_left'; d.sensors.uwb_right = null;
  const out = P.derive(d, tracking([{id: 5, x: 2.1, y: 0, radius: .1}]), live);
  assert.equal(out.state, 'single'); assert.ok(out.average); assert.equal(out.candidate.clusterId, 5);
  assert.equal(out.driveAuthorized, false); assert.equal(out.provisional, true);
  d.sensors.uwb_left.ageMs = 301;
  assert.equal(P.derive(d, tracking(), live).average, null);
});
test('좌우 개별 위치를 계산하고 평균을 LiDAR 앞 10cm 기준으로 표시', () => {
  const out = P.derive(pair(), tracking(), live);
  assert.ok(Math.abs(out.average.x - 2.1) < 1e-10);
  assert.ok(Math.abs(out.average.y) < 1e-10);
  assert.ok(out.residualM < 1e-10);
  assert.equal(out.provisional, true); assert.equal(out.driveAuthorized, false);
});
test('장착 yaw와 LiDAR 기준 이동을 한 번 적용', () => {
  const t = tracking(); t.transform = {yaw: 90, offsetX: 1, offsetY: 2};
  const out = P.derive(pair(), t, live);
  assert.ok(Math.abs(out.average.x - 1) < 1e-10);
  assert.ok(Math.abs(out.average.y - 4.1) < 1e-10);
});
test('2m 시험에서 관측된 형태의 상반된 각도는 평균과 군집을 숨김', () => {
  const d = pair(); d.sensors.uwb_left.rangeM = 1.44; d.sensors.uwb_left.angleDeg = -15;
  d.sensors.uwb_right.rangeM = 1.45; d.sensors.uwb_right.angleDeg = 12;
  const out = P.derive(d, tracking([{id: 1, x: 1.5, y: 0, radius: .1}]), live);
  assert.ok(out.residualM > .6); assert.equal(out.state, 'disagreement');
  assert.equal(out.average, null); assert.equal(out.candidate, null);
  assert.ok(out.left && out.right && out.disagreementAverage);
});
test('유일 인접 군집은 실험 후보이며 사람 판정이나 구동 권한이 아님', () => {
  const out = P.derive(pair(), tracking([{id: 3, x: 2.3, y: .1, radius: .1}]), live);
  assert.equal(out.candidate.clusterId, 3); assert.equal(out.candidateCount, 1);
  assert.match(out.label, /사람 판별 아님/); assert.equal(out.driveAuthorized, false);
});
test('인접 군집 여러 개면 선택하지 않음', () => {
  const out = P.derive(pair(), tracking([{id: 1, x: 2.2, y: 0, radius: .1},
    {id: 2, x: 2.1, y: .1, radius: .1}]), live);
  assert.equal(out.candidateCount, 2); assert.equal(out.candidate, null);
});
test('만료된 LiDAR 군집은 없는 군집으로 해석하지 않음', () => {
  const out = P.derive(pair(), tracking([{id: 1, x: 2.2, y: 0, radius: .1}]), {...live, clustersUsable: false});
  assert.ok(out.average); assert.equal(out.candidate, null); assert.equal(out.candidateCount, null);
});
test('센서 만료·단절·무효·수신 시각 차이는 평균을 지움', () => {
  for (const alter of [s => {s.uwb_left.ageMs = 301;}, s => {s.uwb_right.ageMs = 125;},
    s => {s.uwb_left.valid = false;}, s => {s.uwb_left.rangeM = NaN;},
    s => {s.uwb_right.angleDeg = Infinity;}]) {
    const d = pair(); alter(d.sensors); const out = P.derive(d, tracking(), live);
    assert.equal(out.average, null); assert.equal(out.candidate, null);
  }
});
test('정지·재생·센서 단절·가정 표시 끄기는 이전 위치를 내보내지 않음', () => {
  for (const change of [{enabled: false}, {paused: true}, {sourceMode: 'replay'},
    {connected: false}, {lidarUsable: false}, {trackingConnected: false}]) {
    const out = P.derive(pair(), tracking(), {...live, ...change});
    assert.equal(out.average, null); assert.equal(out.left, null); assert.equal(out.right, null);
  }
});
test('LiDAR 군집·좌표의 잘못된 입력을 거부', () => {
  const t = tracking([{id: 0, x: Infinity, y: 0, radius: .1}]);
  assert.equal(P.derive(pair(), t, live).state, 'error');
  t.transform.yaw = NaN;
  assert.equal(P.derive(pair(), t, live).average, null);
});
function disagreement(x, y = 0) {
  return {state: 'disagreement', left: {x: 0, y: 0}, right: {x: 2, y: 0},
    average: null, disagreementAverage: {x, y}, residualM: 1.2, candidate: null,
    candidateCount: null, provisional: true, driveAuthorized: false};
}
function meta(n, now = n * 100, frameKey = 'frame') {
  return {leftSeq: n, rightSeq: n, nowMs: now, frameKey};
}
test('단발성 큰 표시 튐은 중위값과 평활로 줄이고 원시 불일치 판정을 유지', () => {
  const filter = P.createSmoother(); let out;
  for (const [i, x] of [1, 1.02, 3.5, 1.01, 1].entries()) out = filter.update(disagreement(x), meta(i + 1));
  assert.ok(Math.abs(out.disagreementAverage.x - 1) < .1);
  assert.equal(out.residualM, 1.2); assert.equal(out.average, null); assert.equal(out.candidate, null);
  assert.equal(out.driveAuthorized, false); assert.equal(out.displaySmoothed, true);
});
test('반복되거나 한쪽만 새 관측이면 같은 값을 다시 필터에 넣지 않음', () => {
  const filter = P.createSmoother();
  assert.equal(filter.update(disagreement(1), meta(1)).disagreementAverage.x, 1);
  assert.equal(filter.update(disagreement(4), meta(1, 150)).disagreementAverage.x, 1);
  assert.equal(filter.update(disagreement(4), {leftSeq: 2, rightSeq: 1, nowMs: 200, frameKey: 'frame'}).disagreementAverage.x, 1);
  const updated = filter.update(disagreement(4), meta(2, 250));
  assert.ok(updated.disagreementAverage.x > 1 && updated.disagreementAverage.x < 4);
});
test('무효·단절·기준좌표 변경·시각 공백·순번 역행이면 과거 필터 이력 폐기', () => {
  const filter = P.createSmoother();
  filter.update(disagreement(1), meta(10));
  const invalid = {state: 'waiting', average: null, disagreementAverage: null};
  assert.equal(filter.update(invalid, meta(11)), invalid);
  assert.equal(filter.update(disagreement(4), meta(12)).disagreementAverage.x, 4);
  assert.equal(filter.update(disagreement(2), meta(13, 1300, 'new-frame')).disagreementAverage.x, 2);
  assert.equal(filter.update(disagreement(5), meta(14, 2100, 'new-frame')).disagreementAverage.x, 5);
  assert.equal(filter.update(disagreement(3), meta(1, 2200, 'new-frame')).disagreementAverage.x, 3);
});
test('좌우 일치와 불일치 간 전환은 서로 다른 상태의 과거 점을 섞지 않음', () => {
  const filter = P.createSmoother(); filter.update(disagreement(1), meta(1));
  const position = P.derive(pair(), tracking(), live);
  const out = filter.update(position, meta(2));
  assert.ok(Math.abs(out.average.x - 2.1) < 1e-10);
  assert.equal(out.state, 'position'); assert.equal(out.driveAuthorized, false);
});
test('잘못된 순번은 표시 원본을 반환하고 이전 이력을 지움', () => {
  const filter = P.createSmoother(); filter.update(disagreement(1), meta(1));
  const original = disagreement(4);
  assert.equal(filter.update(original, {...meta(2), leftSeq: null}), original);
  assert.equal(filter.update(disagreement(3), meta(3)).disagreementAverage.x, 3);
});
console.log(`실험 표시 회귀 ${passed}개 통과`);
