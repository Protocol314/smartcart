/* 추적 표시의 출처·수명과 실제 앱 이벤트 경로를 장치 없이 검증한다. */
const assert = require('node:assert/strict');
const fs = require('node:fs');
const path = require('node:path');
const vm = require('node:vm');
const T = require('../web/js/tracking.js');
const C = require('../web/js/core.js');
const S = require('../web/js/sensing.js');
const D = require('../web/js/dual_uwb.js');
const P = require('../web/js/field_preview.js');
let passed = 0;
function test(name, check) { check(); passed++; console.log(`통과 ${name}`); }
const copy = value => JSON.parse(JSON.stringify(value));
function payload() {
  const common = { frame_id: 'base_link', source_mode: 'replay', drive_authorized: false, provisional: true, valid: true, invalid_reason: null };
  return { schema: 'smartcart.tracking_view.v1', mode: 'replay', source_mode: 'replay', state: 'running', sequence: 1,
    age_ms: 0, remaining_ms: 400, cluster_remaining_ms: 300, prediction_remaining_ms: 0, uwb_remaining_ms: 400,
    valid: true, drive_authorized: false, provisional: true, frame_id: 'base_link',
    lidar_transform: { yaw_deg: 0, translation_m: [.817, 0, .2678] }, result: {
      uwb_estimate: { ...common, schema: 'smartcart.uwb_tag_estimate.v1', position_xy_m: [2, .1] },
      lidar_clusters: { ...common, schema: 'smartcart.lidar_clusters.v1', clusters: [
        { cluster_id: 0, centroid_xy_m: [2, .1], radius_m: .1, width_m: .2, point_count: 8, first_ray: 1, last_ray: 8, error_radius_m: .15 }] },
      track: { ...common, schema: 'smartcart.target_track.v1', state: 'tracking', track_id: 7,
        position_xy_m: [2, .1], velocity_xy_mps: [.1, 0], predicted_position_xy_m: null, error_radius_m: .3,
        confirmation_count: 3, candidate_count: 1, associated_cluster_id: 0 } } };
}
function prediction(state = 'coasting') {
  const p = payload(); p.valid = false; p.remaining_ms = 0; p.prediction_remaining_ms = 100;
  Object.assign(p.result.track, { state, valid: false, invalid_reason: state === 'ambiguous' ? 'ambiguous_candidates' : 'no_candidate',
    position_xy_m: null, velocity_xy_mps: null, predicted_position_xy_m: [2.1, .1], associated_cluster_id: null,
    candidate_count: state === 'ambiguous' ? 2 : 0 });
  return p;
}
test('서버 재생·군집·선택 대상·새 UWB를 표시하고 카트 좌표를 재회전하지 않음', () => {
  const view = T.deriveTracking(payload(), { mode: 'live', connected: true, radius: 5 });
  assert.equal(view.state, 'tracking'); assert.match(view.sourceLabel, /재생/);
  assert.deepEqual([view.selected.x, view.selected.y], [2, .1]);
  assert.equal(view.selected.id, 7); assert.equal(view.clusters.length, 1); assert.ok(view.uwb);
  assert.deepEqual(view.transform, { yaw: 0, offsetX: .817, offsetY: 0 });
});
test('왼쪽 단독 UWB 위치와 연관 군집을 유효 시간 안에서만 표시', () => {
  const p = payload(); p.mode = 'ros'; p.source_mode = 'live';
  for (const value of Object.values(p.result)) value.source_mode = 'live';
  p.result.uwb_estimate.schema = 'smartcart.single_uwb_estimate.v1';
  const live = T.deriveTracking(p, {serverSourceMode: 'ros'});
  assert.ok(live.uwb); assert.ok(live.selected);
  const expired = T.deriveTracking(p, {serverSourceMode: 'ros', elapsedMs: 401});
  assert.equal(expired.uwb, null); assert.equal(expired.selected, null);
});
test('각 표시의 독립 TTL 만료와 군집 유지', () => {
  let view = T.deriveTracking(payload(), { elapsedMs: 301 });
  assert.equal(view.clusters.length, 0); assert.ok(view.selected);
  view = T.deriveTracking(payload(), { elapsedMs: 401 });
  assert.equal(view.state, 'stale'); assert.equal(view.selected, null); assert.equal(view.uwb, null); assert.equal(view.trackId, null);
});
test('첫 입력 대기는 표시 만료로 오인하지 않음', () => {
  const p = prediction('acquiring'); p.result.track.state = 'waiting'; p.result.track.predicted_position_xy_m = null;
  p.result.track.invalid_reason = 'waiting_for_inputs';
  Object.assign(p, { remaining_ms: 0, cluster_remaining_ms: 0, prediction_remaining_ms: 0, uwb_remaining_ms: 0 });
  assert.equal(T.deriveTracking(p).state, 'waiting');
});
test('모호함·가림·확인 중은 속빈 예측만 허용', () => {
  for (const state of ['coasting', 'acquiring', 'ambiguous']) {
    const view = T.deriveTracking(prediction(state));
    assert.equal(view.selected, null); assert.ok(view.prediction); assert.equal(view.state, state);
    assert.equal(T.deriveTracking(prediction(state), { elapsedMs: 101 }).prediction, null);
  }
});
test('로컬 sample/file/demo와 화면 정지는 모든 추적 표시를 숨김', () => {
  for (const mode of ['sample', 'file', 'demo']) {
    const view = T.deriveTracking(payload(), { mode });
    assert.equal(view.selected, null); assert.equal(view.clusters.length, 0);
  }
  const view = T.deriveTracking(payload(), { paused: true });
  assert.equal(view.state, 'paused'); assert.equal(view.selected, null); assert.equal(view.prediction, null);
  assert.equal(view.clusters.length, 0);
});
test('추적 전용 연결 끊김은 표시를 즉시 제거', () => {
  const view = T.deriveTracking(payload(), { connected: false });
  assert.equal(view.state, 'offline'); assert.equal(view.selected, null); assert.equal(view.clusters.length, 0);
});
test('끝남·대기·오류·stale 서버 상태에서 이전 좌표 제거', () => {
  for (const state of ['waiting', 'stale', 'error', 'ended']) {
    const view = T.deriveTracking({ ...payload(), state });
    assert.equal(view.selected, null); assert.equal(view.clusters.length, 0);
  }
});
test('ROS와 replay 출처·구동금지·프레임 불일치 거부', () => {
  const cases = [];
  let p = payload(); p.source_mode = 'live'; cases.push(p);
  p = payload(); p.result.track.source_mode = 'live'; cases.push(p);
  p = payload(); p.result.lidar_clusters.source_mode = 'live'; cases.push(p);
  p = payload(); p.result.uwb_estimate.source_mode = 'live'; cases.push(p);
  p = payload(); p.result.track.drive_authorized = true; cases.push(p);
  p = payload(); p.drive_authorized = true; cases.push(p);
  p = payload(); p.result.track.frame_id = 'laser'; cases.push(p);
  for (const input of cases) assert.equal(T.deriveTracking(input).state, 'error');
});
test('ROS live 데이터는 명시적인 ROS 출처로 표시', () => {
  const p = payload(); p.mode = 'ros'; p.source_mode = 'live';
  for (const value of Object.values(p.result)) value.source_mode = 'live';
  assert.match(T.deriveTracking(p).sourceLabel, /ROS/);
});
test('배경 ROS에 재생 군집 또는 재생 배경에 ROS 추적을 섞지 않음', () => {
  assert.equal(T.deriveTracking(payload(), { serverSourceMode: 'ros' }).suppressBackground, true);
  assert.ok(T.deriveTracking(payload(), { serverSourceMode: 'replay' }).selected);
  const p = payload(); p.mode = 'ros'; p.source_mode = 'live';
  for (const value of Object.values(p.result)) value.source_mode = 'live';
  assert.equal(T.deriveTracking(p, { serverSourceMode: 'replay' }).selected, null);
});
test('미래·음수·무한 경과시간과 과대 TTL 거부', () => {
  for (const elapsedMs of [-1, NaN, Infinity]) assert.equal(T.deriveTracking(payload(), { elapsedMs }).state, 'error');
  for (const value of [-1, Infinity, 5001, null]) assert.equal(T.deriveTracking({ ...payload(), remaining_ms: value }).state, 'error');
  assert.equal(T.deriveTracking({ ...payload(), age_ms: -1 }).state, 'error');
});
test('같은 sequence heartbeat가 수명·위치를 갱신하지 않음', () => {
  const first = T.acceptEnvelope(null, payload(), 0);
  const repeated = payload(); repeated.remaining_ms = 4000; repeated.result.track.position_xy_m = [4, 4];
  const second = T.acceptEnvelope(first, repeated, 350);
  assert.equal(second, first);
  assert.equal(T.deriveTracking(second.payload, { elapsedMs: 401 }).selected, null);
  const error = T.acceptEnvelope(second, { ...repeated, state: 'error' }, 402);
  assert.equal(error.receivedAt, 0); assert.equal(T.deriveTracking(error.payload).selected, null);
});
test('순번 역행·오류 뒤 과거 순번으로 위치 복원 금지', () => {
  let inbox = T.acceptEnvelope(null, { ...payload(), sequence: 5 }, 0);
  inbox = T.acceptEnvelope(inbox, { schema: 'invalid' }, 1);
  inbox = T.acceptEnvelope(inbox, { ...payload(), sequence: 4 }, 2);
  assert.equal(T.deriveTracking(inbox.payload).state, 'error');
  inbox = T.acceptEnvelope(inbox, { ...payload(), sequence: 6 }, 3);
  assert.ok(T.deriveTracking(inbox.payload).selected);
});
test('과대 군집·비유한·중복 ID와 무효 측정 좌표 거부', () => {
  const cases = [];
  let p = payload(); p.result.lidar_clusters.clusters = Array.from({ length: 129 }, (_, i) => ({ ...p.result.lidar_clusters.clusters[0], cluster_id: i })); cases.push(p);
  p = payload(); p.result.lidar_clusters.clusters[0].radius_m = Infinity; cases.push(p);
  p = payload(); p.result.lidar_clusters.clusters[0].centroid_xy_m = [NaN, 1]; cases.push(p);
  p = payload(); p.result.lidar_clusters.clusters.push(copy(p.result.lidar_clusters.clusters[0])); cases.push(p);
  p = prediction(); p.result.track.position_xy_m = [1, 1]; cases.push(p);
  p = payload(); p.result.track.associated_cluster_id = 9; cases.push(p);
  for (const input of cases) assert.equal(T.deriveTracking(input).state, 'error');
});
test('지원 밖 장착 좌표는 clamp 없이 추적·배경을 숨김', () => {
  const p = payload(); p.lidar_transform.translation_m[0] = 2.1;
  const view = T.deriveTracking(p); assert.equal(view.suppressBackground, true);
  assert.equal(view.selected, null); assert.match(view.reason, /장착 범위/);
});
test('표시 반경 밖 대상과 예측은 숨기고 군집은 캔버스로 자름', () => {
  const view = T.deriveTracking(payload(), { radius: 1 });
  assert.equal(view.selected, null); assert.equal(view.uwb, null); assert.equal(view.clusters.length, 1);
});
function harness() {
  let now = 0, timerId = 0;
  const timers = new Map(), elements = new Map(), sources = [];
  const html = fs.readFileSync(path.join(__dirname, '../web/index.html'), 'utf8');
  for (const match of html.matchAll(/\bid="([^"]+)"/g)) elements.set(match[1], {
    value: '', textContent: '', hidden: false, disabled: false, checked: false, parentElement: { dataset: {} },
    dataset: {}, listeners: {}, open: false, addEventListener(n, fn) { this.listeners[n] = fn; },
    showModal() { this.open = true; }, close() { this.open = false; } });
  class Renderer {
    constructor() { this.view = {}; this.marker = null; this.tracking = null; this.scanBuilds = 0; }
    setFrame(scan, projection) { this.view.scan = scan; this.view.projection = projection; this.scanBuilds++; }
    setStatus(s) { Object.assign(this.view, s); }
    setUwb(m) { this.marker = m; }
    setTracking(t) { this.tracking = t; }
    configure(s) { this.settings = s; }
  }
  class Events {
    constructor(url) { this.url = url; this.handlers = {}; this.closed = false; sources.push(this); }
    addEventListener(n, fn) { this.handlers[n] = fn; }
    close() { this.closed = true; }
    emit(data, name) { this.handlers[name]({ data: JSON.stringify(data) }); }
  }
  const context = { RadarCore: C, SensingCore: S, TrackingCore: T, DualUwbCore: D, FieldPreview: P, RadarRenderer: Renderer,
    document: { getElementById: id => elements.get(id), documentElement: {}, addEventListener() {} },
    performance: { now: () => now }, location: { protocol: 'http:', search: '' }, localStorage: { getItem: () => null, setItem() {} },
    URLSearchParams, EventSource: Events, addEventListener() {}, setTimeout: () => 1, clearTimeout() {},
    setInterval(fn, period) { const id = ++timerId; timers.set(id, { fn, period }); return id; }, clearInterval(id) { timers.delete(id); } };
  context.window = context; context.SMARTCART_SAMPLE = C.syntheticScan(0);
  vm.runInNewContext(fs.readFileSync(path.join(__dirname, '../web/js/app.js'), 'utf8'), context);
  return { context, elements, sources, get view() { return context.SmartCartDiagnostics; },
    frame(tracking = payload()) { const scan = { ...C.syntheticScan(0), sequence: now, source: { kind: 'replay' } };
      sources.findLast(s => !s.closed && s.url.startsWith('/api/events')).emit({ schema: 'smartcart.envelope.v1',
        source_mode: 'replay', state: 'replay', age_ms: 0, stale_after_ms: 1500, receive_hz: 25, scan, tracking }, 'frame'); },
    tracking(p) { sources.findLast(s => !s.closed && s.url.startsWith('/api/tracking/events')).emit(p, 'tracking'); },
    advance(ms) { now += ms; for (const t of timers.values()) if (t.period === 200) t.fn(); },
    click(id) { elements.get(id).listeners.click({}); },
    mode(next) { const e = elements.get('source'); e.value = next; e.listeners.change({}); } };
}
test('실제 앱: 전용 SSE·장착 변환·고속군집은 배경 재계산 없음', () => {
  const h = harness(); h.frame();
  assert.ok(h.sources.some(s => s.url === '/api/tracking/events?hz=10'));
  assert.equal(h.view.displayProjectionSettings.offsetX, .817);
  assert.equal(h.elements.get('offsetX').disabled, true);
  const builds = h.view.renderer.scanBuilds;
  h.tracking({ ...payload(), sequence: 2 });
  assert.equal(h.view.renderer.scanBuilds, builds); assert.ok(h.view.tracking.selected);
});
test('실제 앱: 연결 끊김·화면 정지·만료·소스 전환에서 추적 제거', () => {
  const h = harness(); h.frame(); h.tracking({ ...payload(), sequence: 2 });
  h.sources.find(s => s.url.startsWith('/api/tracking/events')).onerror();
  assert.equal(h.view.tracking.selected, null);
  h.tracking({ ...payload(), sequence: 3 }); assert.ok(h.view.tracking.selected);
  h.click('pauseView'); assert.equal(h.view.tracking.selected, null);
  h.advance(501); h.click('pauseView'); assert.equal(h.view.tracking.selected, null);
  h.mode('sample'); assert.equal(h.view.tracking.clusters.length, 0);
  assert.ok(h.sources.filter(s => s.url.startsWith('/api/tracking/events')).every(s => s.closed));
});
test('실제 앱: 잘못된 SSE와 지원 밖 transform은 위치를 남기지 않음', () => {
  const h = harness(); h.frame(); h.tracking({ schema: 'bad' }); assert.equal(h.view.tracking.selected, null);
  const p = payload(); p.sequence = 4; p.lidar_transform.translation_m[0] = 3;
  h.tracking(p); assert.equal(h.view.active, false); assert.equal(h.view.tracking.suppressBackground, true);
});
test('텍스트 안전·전용 canvas·PNG 합성·JSON opt-in 표시', () => {
  const app = fs.readFileSync(path.join(__dirname, '../web/js/app.js'), 'utf8');
  const radar = fs.readFileSync(path.join(__dirname, '../web/js/radar.js'), 'utf8');
  const html = fs.readFileSync(path.join(__dirname, '../web/index.html'), 'utf8');
  assert.ok(!/\.innerHTML\s*=|document\.write\s*\(/.test(app + radar + fs.readFileSync(path.join(__dirname, '../web/js/tracking.js'), 'utf8')));
  assert.match(html, /id="radarTracking"/); assert.match(radar, /this\.uwbCanvas, this\.trackingCanvas/);
  assert.match(app, /tracking: trackingView/); assert.match(app, /위치·추적 정보/);
});
test('실제 앱: 실측 좌우와 군집의 실험 표시를 공식 추적·구동과 분리', () => {
  const h = harness(), p = payload();
  h.elements.get('fieldPreviewEnabled').checked = true;
  p.mode = 'ros'; p.source_mode = 'live'; p.valid = false; p.remaining_ms = 0;
  p.lidar_transform.translation_m = [0, 0, 0];
  for (const value of Object.values(p.result)) value.source_mode = 'live';
  Object.assign(p.result.track, {state: 'waiting', valid: false, invalid_reason: 'uwb_invalid',
    track_id: null, position_xy_m: null, velocity_xy_mps: null, associated_cluster_id: null, confirmation_count: 0});
  Object.assign(p.result.uwb_estimate, {valid: false, invalid_reason: 'range_model_unknown', position_xy_m: null});
  p.result.lidar_clusters.clusters[0].centroid_xy_m = [2.2, 0];
  const d = Math.hypot(2, .225), a = Math.atan2(.225, 2) * 180 / Math.PI;
  const sensor = {state: 'live', connected: true, valid: true, invalid_reason: null,
    measurement_valid: true, id_valid: true, range_m: d, raw_angle_deg: a,
    range_out_of_bounds: false, angle_out_of_bounds: false, age_ms: 10, remaining_ms: 490, sample_sequence: 1};
  const dual = {schema: 'smartcart.dual_uwb_view.v1', source_mode: 'live', state: 'running', sequence: 1,
    age_ms: 0, remaining_ms: 750, valid: true, received: true, drive_authorized: false,
    sensors: {uwb_left: {...sensor}, uwb_right: {...sensor, raw_angle_deg: -a}}};
  const scan = {...C.syntheticScan(0), sequence: 1, source: {kind: 'ros'}};
  h.sources.find(s => s.url.startsWith('/api/events')).emit({schema: 'smartcart.envelope.v1', source_mode: 'ros',
    state: 'live', age_ms: 0, stale_after_ms: 1500, receive_hz: 10, scan, tracking: p,
    sensing: {schema: 'smartcart.sensing.v1', uwb_backend: 'dual'}, dual_uwb: dual}, 'frame');
  assert.ok(h.view.renderer.tracking.fieldPreview.average);
  assert.equal(h.view.renderer.tracking.fieldPreview.candidate.clusterId, 0);
  assert.equal(h.view.tracking.selected, null); assert.equal(h.view.renderer.tracking.fieldPreview.driveAuthorized, false);
  assert.equal(h.elements.get('fieldPreviewPanel').hidden, false);
  const dualStream = h.sources.find(s => s.url.startsWith('/api/dual-uwb/events'));
  dualStream.onerror();
  assert.equal(h.view.renderer.tracking.fieldPreview.average, null);
  dualStream.emit(dual, 'dual_uwb');
  assert.ok(h.view.renderer.tracking.fieldPreview.average);
  h.advance(200);
  dualStream.emit(dual, 'dual_uwb');
  h.advance(101); assert.equal(h.view.renderer.tracking.fieldPreview.average, null);
});
console.log(`추적 프런트엔드 회귀 ${passed}개 통과`);
