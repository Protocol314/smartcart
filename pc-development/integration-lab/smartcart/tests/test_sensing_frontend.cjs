'use strict';
/* 브라우저 의존성 없이 표시 판정·출처 격리·화면 정지의 회귀를 검증한다. */
const assert = require('node:assert/strict');
const fs = require('node:fs');
const path = require('node:path');
const vm = require('node:vm');
const S = require('../web/js/sensing.js');
const C = require('../web/js/core.js');
const D = require('../web/js/dual_uwb.js');
let passed = 0;
function test(name, fn) { fn(); passed++; console.log('통과', name); }
function liveUwb(overrides = {}) {
  const demo = S.syntheticUwb(0);
  return { ...demo, source_mode: 'serial', state: 'live', stale_after_ms: 500,
    measurement: { ...demo.measurement, firmware_state: 'UNLOCKED' }, ...overrides };
}
const liveOptions = { mode: 'live', connected: true, radius: 5 };
test('ROS 전방·좌측 좌표를 재회전하지 않음', () => {
  const v = S.deriveUwb(liveUwb(), liveOptions);
  assert.equal(v.state, 'live'); assert.equal(v.marker.x, 1.7); assert.equal(v.marker.y, 0);
});
test('각 센서의 마지막 수신 시각으로 UWB 만료', () => {
  const v = S.deriveUwb(liveUwb({ age_ms: 300 }), { ...liveOptions, elapsedMs: 201 });
  assert.equal(v.state, 'stale'); assert.equal(v.marker, null); assert.equal(v.ageMs, 501);
});
test('무한·음수·누락 경과 시간으로 실측 정상 판정하지 않음', () => {
  for (const age_ms of [NaN, Infinity, -1, null, undefined]) assert.equal(S.deriveUwb(liveUwb({ age_ms }), liveOptions).marker, null);
});
test('서버 오류와 stale은 좌표가 남아 있어도 마커 제거', () => {
  for (const state of ['error', 'stale', 'waiting']) assert.equal(S.deriveUwb(liveUwb({ state }), liveOptions).marker, null);
});
test('서버 연결 중단은 즉시 좌표 제거', () => assert.equal(S.deriveUwb(liveUwb(), { ...liveOptions, connected: false }).marker, null));
test('ID 불일치·인증 실패·무효 위치 거부', () => {
  for (const changes of [{ tag_id: 1 }, { accepted_id: -1 }, { authenticated: false }, { valid: false }, { valid: 'true' }, { tag_address: '<img>' }]) {
    const m = { ...liveUwb().measurement, ...changes };
    const v = S.deriveUwb(liveUwb({ measurement: m }), liveOptions);
    assert.equal(v.marker, null); assert.equal(v.state, 'invalid'); assert.equal(v.measurement.valid, false);
  }
});
test('타 프레임·비유한·과대 좌표 거부', () => {
  for (const p of [{ x: 1, y: 0, frame_id: 'laser' }, { x: Infinity, y: 0, frame_id: 'base_link' }, { x: 1001, y: 0, frame_id: 'base_link' }]) {
    const v = S.deriveUwb(liveUwb({ measurement: { ...liveUwb().measurement, position: p } }), liveOptions);
    assert.equal(v.marker, null);
  }
});
test('표시 반경 밖의 UWB는 수신 패널만 유지', () => {
  const v = S.deriveUwb(liveUwb(), { ...liveOptions, radius: 1 });
  assert.equal(v.state, 'live'); assert.equal(v.marker, null); assert.ok(v.reason.includes('반경 밖'));
});
test('샘플·파일 보기에는 실측 UWB를 섞지 않음', () => {
  for (const mode of ['sample', 'file']) assert.equal(S.deriveUwb(liveUwb(), { ...liveOptions, mode }).state, 'disabled');
  assert.equal(S.deriveUwb(liveUwb(), { ...liveOptions, serverMode: 'replay' }).state, 'disabled');
});
test('데모에는 실측 UWB를 섞지 않음', () => assert.equal(S.deriveUwb(liveUwb(), { ...liveOptions, mode: 'demo' }).marker, null));
test('합성 UWB는 명시한 합성 화면에만 표시', () => {
  const v = S.deriveUwb(S.syntheticUwb(0), { ...liveOptions, mode: 'demo' });
  assert.equal(v.state, 'synthetic'); assert.equal(v.marker.synthetic, true);
  assert.equal(S.deriveUwb(S.syntheticUwb(0), liveOptions).marker, null);
});
test('ROS로 전달된 합성 표식도 실측으로 판정하지 않음', () => {
  const input = { ...S.syntheticUwb(0), source_mode: 'ros', synthetic: true };
  const v = S.deriveUwb(input, { ...liveOptions, serverSynthetic: true });
  assert.equal(v.synthetic, true); assert.equal(v.state, 'synthetic');
  assert.equal(S.deriveUwb(input, liveOptions).marker, null);
});
test('레거시 LiDAR 서버는 UWB 수신 대기', () => assert.equal(S.deriveUwb(undefined, liveOptions).state, 'waiting'));
test('UWB 입력 off는 형식 오류 대신 사용 안 함', () => assert.equal(S.deriveUwb({ schema: 'smartcart.uwb.v1', source_mode: 'off', state: 'off' }, liveOptions).state, 'disabled'));
test('두 센서·서버 준비 상태가 있어야 통합 정상', () => {
  const uwb = S.deriveUwb(liveUwb(), liveOptions);
  assert.equal(S.integratedState({ mode: 'live', lidarUsable: true, uwb }).ready, true);
  assert.equal(S.integratedState({ mode: 'live', lidarUsable: true, uwb, serverReady: false }).ready, false);
  assert.equal(S.integratedState({ mode: 'live', lidarUsable: false, uwb }).state, 'degraded');
  assert.equal(S.integratedState({ mode: 'live', lidarUsable: true, uwb, paused: true }).ready, false);
});

function harness() {
  let now = 0, timerId = 0;
  const timers = new Map(), elements = new Map(), sources = [];
  const html = fs.readFileSync(path.join(__dirname, '../web/index.html'), 'utf8');
  for (const match of html.matchAll(/\bid="([^"]+)"/g)) {
    elements.set(match[1], { value: '', textContent: '', hidden: false, disabled: false, checked: false,
      parentElement: { dataset: {} }, dataset: {}, listeners: {}, open: false,
      addEventListener(name, fn) { this.listeners[name] = fn; },
      showModal() { this.open = true; }, close() { this.open = false; } });
  }
  const document = { getElementById: id => elements.get(id), documentElement: {}, addEventListener() {} };
  class Renderer {
    constructor() { this.view = {}; this.marker = null; this.frameCount = 0; }
    setFrame(scan, projection) { this.view.scan = scan; this.view.projection = projection; }
    setStatus(status) { Object.assign(this.view, status); }
    setUwb(marker) { this.marker = marker; }
    configure() {}
  }
  class Events {
    constructor(url) { this.url = url; this.handlers = {}; this.closed = false; sources.push(this); }
    addEventListener(name, fn) { this.handlers[name] = fn; }
    close() { this.closed = true; }
    emit(value, name = 'frame') { if (!this.closed && this.handlers[name]) this.handlers[name]({ data: JSON.stringify(value) }); }
  }
  const context = { RadarCore: C, SensingCore: S, DualUwbCore: D, RadarRenderer: Renderer, document,
    performance: { now: () => now }, location: { protocol: 'http:', search: '' },
    localStorage: { getItem: () => null, setItem() {} }, URLSearchParams, EventSource: Events,
    addEventListener() {}, setTimeout: () => 1, clearTimeout() {},
    setInterval(fn, period) { const id = ++timerId; timers.set(id, { fn, period }); return id; },
    clearInterval(id) { timers.delete(id); } };
  context.window = context;
  context.SMARTCART_SAMPLE = C.syntheticScan(0);
  vm.runInNewContext(fs.readFileSync(path.join(__dirname, '../web/js/app.js'), 'utf8'), context);
  function emit({ uwb = liveUwb(), state = 'live', source_mode = 'ros', scan = C.syntheticScan(0), sensing = undefined, dual_uwb = undefined } = {}) {
    const sensorScan = { ...scan, source: { kind: source_mode === 'ros' ? 'ros' : 'synthetic' }, sequence: now };
    sources.findLast(s => !s.closed && s.url.startsWith('/api/events')).emit({ schema: 'smartcart.envelope.v1', source_mode, state, age_ms: 0,
      receive_hz: 10, stale_after_ms: 1500, scan: sensorScan, uwb, sensing, dual_uwb });
  }
  return { get diagnostics() { return context.SmartCartDiagnostics; }, get elements() { return elements; },
    emit, sources, emitUwb(value) { sources.findLast(s => !s.closed && s.url.startsWith('/api/uwb/events')).emit(value, 'uwb'); },
    emitDual(value) { sources.findLast(s => !s.closed && s.url.startsWith('/api/dual-uwb/events')).emit(value, 'dual_uwb'); },
    failDual() { sources.findLast(s => !s.closed && s.url.startsWith('/api/dual-uwb/events')).onerror(); },
    failStream(uwb = false) { sources.findLast(s => !s.closed && s.url.startsWith(uwb ? '/api/uwb/events' : '/api/events')).onerror(); },
    advance(ms) { now += ms; for (const t of timers.values()) if (t.period === 200) t.fn(); },
    click(id) { elements.get(id).listeners.click({}); },
    mode(next) { const e = elements.get('source'); e.value = next; e.listeners.change({}); } };
}
test('실제 앱 이벤트 경로: LiDAR 중단과 UWB 연결은 독립 표시', () => {
  const h = harness(); h.emit({ state: 'stale' });
  assert.equal(h.diagnostics.active, false); assert.equal(h.diagnostics.uwb.state, 'live');
  assert.ok(h.diagnostics.renderer.marker); assert.equal(h.diagnostics.sensing.state, 'degraded');
});
test('실제 앱 이벤트 경로: UWB 만료 시 LiDAR를 유지', () => {
  const h = harness(); h.emit(); h.advance(501);
  assert.equal(h.diagnostics.active, true); assert.equal(h.diagnostics.renderer.marker, null);
  assert.equal(h.diagnostics.uwb.state, 'stale');
});
test('화면 정지 시 새 UWB 위치를 숨겨두고 snapshot 자체를 만료', () => {
  const h = harness(); h.emit(); h.click('pauseView');
  const x = h.diagnostics.renderer.marker.x;
  h.advance(200); h.emit({ uwb: liveUwb({ measurement: { ...liveUwb().measurement, position: { x: 3, y: 1, frame_id: 'base_link' } } }) });
  assert.equal(h.diagnostics.renderer.marker.x, x); assert.equal(h.diagnostics.sensing.ready, false);
  assert.equal(h.elements.get('pauseNotice').hidden, false);
  h.advance(301); assert.equal(h.diagnostics.renderer.marker, null);
  h.click('pauseView'); assert.equal(h.diagnostics.renderer.marker.x, 3);
});
test('화면 정지 LiDAR도 최신 서버 이벤트에 의해 영구 유효해지지 않음', () => {
  const h = harness(); h.emit(); h.click('pauseView');
  h.advance(1600); h.emit(); assert.equal(h.diagnostics.active, false);
});
test('로컬 소스 전환 시 실측 UWB 제거와 합성 생성', () => {
  const h = harness(); h.emit(); h.mode('sample');
  assert.equal(h.diagnostics.renderer.marker, null); assert.equal(h.diagnostics.uwb.state, 'disabled');
  h.mode('demo'); assert.equal(h.diagnostics.uwb.state, 'synthetic'); assert.equal(h.diagnostics.renderer.marker.synthetic, true);
  h.mode('live'); assert.equal(h.diagnostics.renderer.marker, null);
});
test('레거시 envelope는 앱 오류 없이 LiDAR 단독 사용', () => {
  const h = harness(); h.emit({ uwb: null });
  assert.equal(h.diagnostics.active, true); assert.equal(h.diagnostics.uwb.state, 'waiting');
});
test('통합 schema 확인 후에만 별도 10Hz UWB 스트림 연결', () => {
  const h = harness(); h.emit(); assert.equal(h.sources.length, 1);
  h.emit({ sensing: { schema: 'smartcart.sensing.v1', ready: true } });
  assert.equal(h.sources.length, 2); assert.equal(h.sources[1].url, '/api/uwb/events?hz=10');
  h.emit({ sensing: { schema: 'smartcart.sensing.v1', ready: true } }); assert.equal(h.sources.length, 2);
});
test('저속 LiDAR 기본 프레임이 고속 UWB 좌표를 되돌리지 않음', () => {
  const h = harness(); const sensing = { schema: 'smartcart.sensing.v1', ready: true };
  h.emit({ sensing }); h.emitUwb(liveUwb({ measurement: { ...liveUwb().measurement, position: { x: 3, y: .2, frame_id: 'base_link' } } }));
  h.advance(300); h.emit({ sensing });
  assert.equal(h.diagnostics.renderer.marker.x, 3);
  h.emitUwb(liveUwb()); h.advance(300); assert.equal(h.diagnostics.uwb.state, 'live');
});
test('UWB 전용 연결 끊김은 LiDAR만 정상 유지', () => {
  const h = harness(); h.emit({ sensing: { schema: 'smartcart.sensing.v1', ready: true } }); h.emitUwb(liveUwb());
  h.failStream(true); assert.equal(h.diagnostics.active, true); assert.equal(h.diagnostics.uwb.state, 'offline');
  assert.equal(h.diagnostics.renderer.marker, null); h.emitUwb(liveUwb()); assert.equal(h.diagnostics.uwb.state, 'live');
});
test('LiDAR 전용 연결 끊김은 UWB 전용 연결과 독립', () => {
  const h = harness(); h.emit({ sensing: { schema: 'smartcart.sensing.v1', ready: true } }); h.emitUwb(liveUwb());
  h.failStream(false); assert.equal(h.diagnostics.active, false); assert.equal(h.diagnostics.uwb.state, 'live');
  assert.ok(h.diagnostics.renderer.marker);
});
test('전용 UWB 스트림도 화면 정지와 만료를 따름', () => {
  const h = harness(); h.emit({ sensing: { schema: 'smartcart.sensing.v1', ready: true } }); h.emitUwb(liveUwb());
  h.click('pauseView'); const x = h.diagnostics.renderer.marker.x;
  h.advance(100); h.emitUwb(liveUwb({ measurement: { ...liveUwb().measurement, position: { x: 4, y: 0, frame_id: 'base_link' } } }));
  assert.equal(h.diagnostics.renderer.marker.x, x); h.advance(401); assert.equal(h.diagnostics.renderer.marker, null);
  h.click('pauseView'); assert.equal(h.diagnostics.renderer.marker.x, 4);
});
test('소스 변경 때 기본·UWB EventSource 모두 종료', () => {
  const h = harness(); h.emit({ sensing: { schema: 'smartcart.sensing.v1', ready: true } }); h.emitUwb(liveUwb());
  h.mode('sample'); assert.equal(h.sources.length, 2); assert.ok(h.sources.every(s => s.closed));
  h.mode('demo'); assert.equal(h.sources.length, 2); assert.equal(h.diagnostics.uwb.state, 'synthetic');
});
test('외부 텍스트는 HTML 렌더링 API를 사용하지 않음', () => {
  for (const filename of ['sensing.js', 'dual_uwb.js', 'app.js', 'radar.js']) {
    const code = fs.readFileSync(path.join(__dirname, '../web/js', filename), 'utf8');
    assert.ok(!/\.innerHTML\s*=|document\.write\s*\(/.test(code));
  }
});
function dualValue() {
  const sensor = {state: 'live', connected: true, valid: true, invalid_reason: null,
    measurement_valid: true, id_valid: true, range_m: 2, raw_angle_deg: -12,
    range_out_of_bounds: false, angle_out_of_bounds: false, age_ms: 10, remaining_ms: 490, sample_sequence: 1};
  return {schema: 'smartcart.dual_uwb_view.v1', source_mode: 'live', state: 'running', sequence: 1,
    age_ms: 0, remaining_ms: 750, valid: true, received: true, drive_authorized: false,
    sensors: {uwb_left: {...sensor}, uwb_right: {...sensor, range_m: 1.8, raw_angle_deg: 8}}};
}
const dualSensing = {schema: 'smartcart.sensing.v1', ready: true, uwb_backend: 'dual'};
test('왼쪽 단독 프로파일은 오른쪽 연결 없이 왼쪽 상태만 사용', () => {
  const h = harness(), v = dualValue();
  v.profile = 'single_left';
  Object.assign(v.sensors.uwb_right, {state: 'disconnected', connected: false, valid: false,
    invalid_reason: 'disabled_single_left', measurement_valid: false, id_valid: false,
    range_m: null, raw_angle_deg: null, age_ms: null, remaining_ms: 0, sample_sequence: null});
  h.emit({sensing: dualSensing, dual_uwb: v});
  assert.equal(h.elements.get('dualRightCard').hidden, true);
  assert.equal(h.elements.get('dualLeftDistance').textContent, '2.00 m');
  assert.equal(h.elements.get('uwbState').textContent, '왼쪽 BU04 유효 수신');
});
test('실제 앱 좌우 경로: ESP32 off를 덮고 거리·각도·유효 상태 표시', () => {
  const h = harness(); h.emit({uwb: {schema: 'smartcart.uwb.v1', source_mode: 'off', state: 'off'},
    sensing: dualSensing, dual_uwb: dualValue()});
  assert.equal(h.sources[1].url, '/api/dual-uwb/events?hz=10');
  assert.equal(h.elements.get('dualUwbPanel').hidden, false);
  assert.equal(h.elements.get('legacyUwbPanel').hidden, true);
  assert.equal(h.elements.get('dualLeftDistance').textContent, '2.00 m');
  assert.equal(h.elements.get('dualRightAngle').textContent, '8.0°');
  assert.equal(h.elements.get('uwbState').textContent, '좌우 BU04 유효 수신');
});
test('한쪽 무효 표본은 다른 쪽과 분리하고 범위 밖 값을 숨김', () => {
  const h = harness(); h.emit({sensing: dualSensing, dual_uwb: dualValue()});
  const v = dualValue(); Object.assign(v.sensors.uwb_left, {state: 'invalid', valid: false,
    invalid_reason: 'invalid_measurement', measurement_valid: false, range_m: null, range_out_of_bounds: true});
  v.valid = false; h.emitDual(v);
  assert.equal(h.elements.get('dualLeftDistance').textContent, '범위 밖');
  assert.equal(h.elements.get('dualLeftState').textContent, '측정 무효');
  assert.equal(h.elements.get('dualRightDistance').textContent, '1.80 m');
});
test('좌우 전용 스트림은 LiDAR 스트림 단절과 독립이며 오래된 관측은 만료', () => {
  const h = harness(); h.emit({sensing: dualSensing, dual_uwb: dualValue()}); h.emitDual(dualValue());
  h.failStream(); assert.equal(h.elements.get('dualLeftDistance').textContent, '2.00 m');
  h.advance(500); assert.equal(h.elements.get('dualLeftState').textContent, '수신 시간 초과');
  assert.equal(h.elements.get('dualLeftDistance').textContent, '—');
  h.emitDual(dualValue()); assert.equal(h.elements.get('dualLeftDistance').textContent, '2.00 m');
  h.failDual(); assert.equal(h.elements.get('dualLeftState').textContent, '서버 연결 끊김');
});
test('좌우 값은 화면 정지·소스 전환 시 숨기고 실측 수신만 허용', () => {
  const h = harness(); h.emit({sensing: dualSensing, dual_uwb: dualValue()}); h.click('pauseView');
  assert.equal(h.elements.get('dualLeftDistance').textContent, '—');
  h.click('pauseView'); h.mode('sample'); assert.equal(h.elements.get('dualUwbPanel').hidden, true);
  const v = dualValue(); v.source_mode = 'replay'; assert.equal(D.derive(v).state, 'error');
});
console.log(`통합 센싱 프런트엔드 회귀 ${passed}개 통과`);
