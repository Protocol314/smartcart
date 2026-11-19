(function () {
  'use strict';
  const C = window.RadarCore, S = window.SensingCore, T = window.TrackingCore, P = window.FieldPreview, $ = id => document.getElementById(id);
  const STORAGE = 'smartcart-radar-v101-settings';
  let settings = { ...C.defaults };
  try { settings = C.validateSettings(JSON.parse(localStorage.getItem(STORAGE) || '{}')); } catch (_) { /* Storage is optional. */ }
  const renderer = new window.RadarRenderer($('radar'));
  const previewSmoother = P ? P.createSmoother() : null;
  let mode = 'live', events = null, timer = null, session = 0, sequence = 0, packets = 0;
  let scan = null, projection = null, frameKey = null, frameAt = 0, frameAge = 0;
  let envelope = null, envelopeAt = 0, connected = false, hadConnection = false, sourceError = '';
  let uwbEnvelope = null, uwbAt = 0, pendingUwb = null, uwbView = S.deriveUwb(null), sensingView = null;
  let uwbEvents = null, uwbStreamState = 'none', uwbDedicatedReceived = false;
  let dualEnvelope = null, dualAt = 0, previewDualInbox = null, uwbStreamKind = 'legacy';
  let trackingEvents = null, trackingStreamState = 'none', trackingDedicatedReceived = false;
  let trackingInbox = null, trackingTransform = null, trackingView = T ? T.empty() : null;
  let paused = false, pending = null, active = false;
  let localFrames = [], replayIndex = 0, replayPlaying = false, replayClock = 0, replayStart = 0;
  let toastTimer = null;

  function text(id, value) { const el = $(id), next = String(value); if (el.textContent !== next) el.textContent = next; }
  function toast(message) {
    const id = $('settingsDialog').open ? 'dialogMessage' : 'toast';
    text(id, message); $(id).hidden = false; clearTimeout(toastTimer);
    toastTimer = setTimeout(() => { $('toast').hidden = true; $('dialogMessage').hidden = true; }, 3200);
  }
  function persist() { try { localStorage.setItem(STORAGE, JSON.stringify(settings)); } catch (_) { /* Session settings remain usable. */ } }
  function syncControls() {
    for (const key of ['radius', 'displayHz', 'yaw', 'offsetX', 'offsetY', 'nearM', 'farM', 'maxGap']) $(key).value = String(settings[key]);
    for (const key of ['contoursEnabled', 'sweepEnabled']) $(key).checked = settings[key];
    text('legend', `RED <${settings.nearM} m · YELLOW ${settings.nearM}–${settings.farM} m · GREEN ≥${settings.farM} m`);
  }
  function projectionSettings() { return trackingTransform && mode === 'live' ? { ...settings, ...trackingTransform } : settings; }
  function applyTracking(next) {
    if (!T) return;
    trackingInbox = T.acceptEnvelope(trackingInbox, next, performance.now());
  }
  function applyFrame(next, key, age = 0, force = false) {
    if (paused && !force) { pending = { next, key, age, at: performance.now() }; return; }
    if (key === frameKey && !force) return;
    scan = next; frameKey = key; frameAt = performance.now(); frameAge = age;
    projection = C.projectScan(scan, projectionSettings());
    renderer.setFrame(scan, projection, key);
  }
  function clearView() {
    scan = null; projection = null; frameKey = null; pending = null; active = false;
    renderer.setFrame(null, null, null); renderer.setStatus({ fresh: false, paused: false });
    uwbEnvelope = null; pendingUwb = null; renderer.setUwb(null);
    dualEnvelope = null; dualAt = 0; previewDualInbox = null;
    if (previewSmoother) previewSmoother.reset();
    trackingInbox = null; trackingTransform = null; trackingView = T ? T.empty() : null;
    if (renderer.setTracking) renderer.setTracking(null);
  }
  function applyUwb(next) {
    if (paused) { pendingUwb = { next, at: performance.now() }; return; }
    uwbEnvelope = next || null; uwbAt = performance.now();
  }
  function applyDual(next) {
    dualEnvelope = next || null; dualAt = performance.now();
    if (T) previewDualInbox = T.acceptEnvelope(previewDualInbox,
      next && typeof next === 'object' ? next : {schema: 'invalid'}, dualAt);
  }
  function stopProducer() {
    if (events) events.close(); events = null;
    closeUwbStream();
    closeTrackingStream();
    if (timer) clearInterval(timer); timer = null;
    session++;
  }
  function closeUwbStream() {
    if (uwbEvents) uwbEvents.close();
    uwbEvents = null; uwbStreamState = 'none'; uwbDedicatedReceived = false;
  }
  function closeTrackingStream() {
    if (trackingEvents) trackingEvents.close();
    trackingEvents = null; trackingStreamState = 'none'; trackingDedicatedReceived = false;
  }
  function connectTrackingStream(token) {
    if (!T || trackingEvents) return;
    trackingStreamState = 'connecting';
    trackingEvents = new EventSource('/api/tracking/events?hz=10');
    trackingEvents.addEventListener('tracking', event => {
      if (mode !== 'live' || token !== session) return;
      trackingStreamState = 'live'; trackingDedicatedReceived = true;
      try {
        if (typeof event.data !== 'string' || event.data.length > 262144) throw new Error('추적 메시지 크기 오류');
        applyTracking(JSON.parse(event.data));
      } catch (_) { applyTracking({ schema: 'invalid' }); }
      updateStatus();
    });
    trackingEvents.onerror = () => {
      if (mode === 'live' && token === session) { trackingStreamState = 'offline'; updateStatus(); }
    };
  }
  function connectUwbStream(token, dual = false) {
    const kind = dual ? 'dual' : 'legacy';
    if (uwbEvents && uwbStreamKind === kind) return;
    if (uwbEvents) closeUwbStream();
    uwbStreamKind = kind;
    uwbStreamState = 'connecting';
    uwbEvents = new EventSource(dual ? '/api/dual-uwb/events?hz=10' : '/api/uwb/events?hz=10');
    uwbEvents.addEventListener(dual ? 'dual_uwb' : 'uwb', event => {
      if (mode !== 'live' || token !== session) return;
      try {
        if (typeof event.data !== 'string' || event.data.length > 16384) throw new Error('UWB 메시지 크기 오류');
        const data = JSON.parse(event.data);
        if (!data || data.schema !== (dual ? 'smartcart.dual_uwb_view.v1' : 'smartcart.uwb.v1')) throw new Error('UWB 메시지 형식 오류');
        uwbStreamState = 'live'; uwbDedicatedReceived = true;
        if (dual) applyDual(data); else applyUwb(data);
      } catch (_) {
        uwbStreamState = 'live'; uwbDedicatedReceived = true;
        if (dual) applyDual({ schema: 'invalid' }); else applyUwb({ schema: 'invalid' });
      }
      updateStatus();
    });
    uwbEvents.onerror = () => {
      if (mode === 'live' && token === session) {
        uwbStreamState = 'offline';
        if (uwbStreamKind === 'dual') { previewDualInbox = null; if (previewSmoother) previewSmoother.reset(); }
        updateStatus();
      }
    };
  }
  function connectServer() {
    if (events) events.close(); closeUwbStream(); closeTrackingStream();
    trackingInbox = null; trackingTransform = null; connected = false; envelope = null; sourceError = '';
    dualEnvelope = null; dualAt = 0; previewDualInbox = null;
    if (previewSmoother) previewSmoother.reset();
    if (location.protocol === 'file:') {
      sourceError = 'ROS 연결: Pi 서버의 http://<Pi IP>:8765에서 여십시오.'; updateStatus(); return;
    }
    const token = session;
    events = new EventSource(`/api/events?hz=${settings.displayHz}`);
    events.addEventListener('frame', event => {
      if (mode !== 'live' || token !== session) return;
      try {
        const data = JSON.parse(event.data);
        if (data.schema !== 'smartcart.envelope.v1') throw new Error('서버 형식 오류');
        if (!['ros', 'demo', 'replay'].includes(data.source_mode)) throw new Error('입력 종류 오류');
        envelope = data; envelopeAt = performance.now(); connected = true; hadConnection = true; sourceError = ''; packets++;
        if (!uwbDedicatedReceived) applyUwb(data.uwb);
        const dual = data.sensing && data.sensing.uwb_backend === 'dual';
        if (dual && !uwbDedicatedReceived) applyDual(data.dual_uwb);
        if (data.sensing && data.sensing.schema === 'smartcart.sensing.v1') connectUwbStream(token, dual);
        if (T && data.tracking && data.tracking.schema === 'smartcart.tracking_view.v1') {
          if (!trackingDedicatedReceived || data.tracking.mode === 'off') applyTracking(data.tracking);
          if (data.tracking.mode !== 'off') connectTrackingStream(token);
          else closeTrackingStream();
        }
        if (data.scan) {
          const key = `${token}:${data.scan.sequence}`;
          if (key !== frameKey) {
            const parsed = C.normaliseScan(data.scan);
            if (data.source_mode !== 'ros' && parsed.source.kind === 'ros') parsed.source.kind = 'replay';
            applyFrame(parsed, key, Math.max(0, Number(data.age_ms) || 0));
          }
        }
      } catch (error) { sourceError = `입력 오류: ${error.message}`; }
      updateStatus();
    });
    events.onerror = () => { if (mode === 'live' && token === session) { connected = false; updateStatus(); } };
  }
  function setMode(next) {
    const previous = mode;
    stopProducer(); mode = next; connected = false; hadConnection = false; envelope = null; sourceError = '';
    paused = false; replayPlaying = false; $('source').value = next; $('fileControls').hidden = next !== 'file';
    text('pauseView', '화면 정지'); clearView();
    // The supplied incomplete scan is entirely near the laser. Explicitly zoom it, never invent a room.
    if (next === 'sample' && settings.radius >= 1) settings.radius = .25;
    else if ((next === 'live' || next === 'demo') && previous === 'sample' && settings.radius === .25) settings.radius = 5;
    syncControls(); renderer.configure(settings);
    if (next === 'live') connectServer();
    else if (next === 'sample') applyFrame(C.normaliseScan(window.SMARTCART_SAMPLE, { kind: 'sample', label: '첨부 부분 스캔' }), `sample:${session}`);
    else if (next === 'demo') {
      const start = performance.now();
      const tick = () => { const t = (performance.now() - start) / 1000; applyFrame(C.syntheticScan(t), `demo:${++sequence}`); applyUwb(S.syntheticUwb(t)); updateStatus(); };
      tick(); timer = setInterval(tick, 1000 / settings.displayHz);
    } else if (next === 'file') { replayIndex = 0; showReplay(0); startReplayTimer(); }
    updateStatus();
  }
  function sourceInfo() {
    if (mode === 'sample') return { badge: 'SAMPLE', label: 'SAMPLE / 첨부 · 비실시간' };
    if (mode === 'file' || envelope && envelope.source_mode === 'replay') {
      const synthetic = scan && (scan.source.kind === 'synthetic' || scan.source.original_kind === 'synthetic');
      return { badge: 'REPLAY', label: synthetic ? 'REPLAY / SYNTHETIC' : 'REPLAY / 비실시간' };
    }
    if (mode === 'demo' || envelope && envelope.source_mode === 'demo' || scan && (scan.source.kind === 'synthetic' || scan.source.original_kind === 'synthetic'))
      return { badge: 'DEMO', label: 'DEMO / 합성 · 비실측' };
    return { badge: 'LIVE', label: 'ROS /scan' };
  }
  function updateStatus() {
    const now = performance.now(), origin = sourceInfo();
    let state = origin.badge, notice = '', usable = Boolean(scan);
    const end = envelope && envelope.state === 'ended';
    if (mode === 'live') {
      let age = envelope ? Math.max(0, Number(envelope.age_ms) || 0) + now - envelopeAt : Infinity;
      if (paused && scan) age = Math.max(age, frameAge + now - frameAt);
      const staleLimit = envelope ? Math.min(1500, Number(envelope.stale_after_ms) || 1500) : 1500;
      if (sourceError) { state = 'ERROR'; usable = false; notice = sourceError; }
      else if (!connected) { state = hadConnection ? 'OFFLINE' : 'WAITING'; usable = false; notice = hadConnection ? '서버 연결 끊김' : ''; }
      else if (envelope.state === 'error') { state = 'ERROR'; usable = false; notice = String(envelope.error || '입력 오류'); }
      else if (envelope.state === 'waiting' || !scan) { state = 'WAITING'; usable = false; }
      else if (end) { state = 'REPLAY END'; }
      else if (envelope.state === 'stale' || age > staleLimit) { state = 'STALE'; usable = false; notice = '새 스캔 없음 · /scan 확인'; }
    } else if (mode === 'file' && !replayPlaying) state = 'REPLAY STOP';
    if (paused && usable) state = 'PAUSED';
    if (T) {
      trackingView = T.deriveTracking(trackingInbox && trackingInbox.payload, { mode, paused,
        serverSourceMode: envelope && envelope.source_mode,
        connected: trackingDedicatedReceived ? trackingStreamState === 'live' : connected && !sourceError,
        elapsedMs: trackingInbox ? now - trackingInbox.receivedAt : 0, radius: settings.radius });
      if (trackingView.transform && JSON.stringify(trackingView.transform) !== JSON.stringify(trackingTransform)) {
        trackingTransform = trackingView.transform;
        if (scan) { projection = C.projectScan(scan, projectionSettings()); renderer.setFrame(scan, projection, frameKey); }
        renderer.configure(projectionSettings());
      }
      if (trackingView.state === 'off' && trackingTransform) {
        trackingTransform = null;
        if (scan) { projection = C.projectScan(scan, settings); renderer.setFrame(scan, projection, frameKey); }
        renderer.configure(settings);
      }
      if (trackingView.suppressBackground) { usable = false; state = 'ERROR'; notice = trackingView.reason; }
      text('trackingState', trackingView.label); $('trackingPanel').dataset.state = trackingView.state;
      text('trackingSource', trackingView.sourceLabel);
      text('trackingId', trackingView.trackId === null ? '—' : `#${trackingView.trackId}`);
      text('trackingConfirm', `${trackingView.confirmationCount} / ${trackingView.candidateCount}`);
      text('trackingCandidates', trackingView.clusters.length);
      text('trackingAge', trackingView.ageMs === null ? '—' : `${Math.round(trackingView.ageMs)} ms`);
      text('trackingReason', trackingView.reason || '유일 후보 연관 · 모든 결과 검수 전 · 구동 금지');
      const serverMount = mode === 'live' && Boolean(trackingTransform);
      for (const key of ['yaw', 'offsetX', 'offsetY']) $(key).disabled = serverMount;
      $('trackingTransformNotice').hidden = !serverMount;
    }
    active = usable;
    const uwbInput = sourceError && !uwbDedicatedReceived ? null : uwbEnvelope;
    const uwbConnected = mode !== 'live' || (uwbDedicatedReceived ? uwbStreamState === 'live' :
      connected && !sourceError && uwbStreamState !== 'offline');
    uwbView = S.deriveUwb(uwbInput, { mode, serverMode: envelope && envelope.source_mode,
      serverSynthetic: origin.badge === 'DEMO', connected: uwbConnected,
      elapsedMs: mode === 'demo' && paused ? 0 : now - uwbAt, radius: settings.radius });
    sensingView = S.integratedState({ mode, lidarUsable: usable, lidarSynthetic: origin.badge === 'DEMO',
      uwb: uwbView, paused, serverReady: envelope && envelope.sensing ? envelope.sensing.ready : undefined });
    const dualMode = mode === 'live' && Boolean(envelope && envelope.sensing && envelope.sensing.uwb_backend === 'dual');
    $('dualUwbPanel').hidden = !dualMode;
    $('legacyUwbPanel').hidden = dualMode;
    let dualView = null;
    if (dualMode && window.DualUwbCore) {
      dualView = window.DualUwbCore.derive(dualEnvelope, {connected: uwbStreamState === 'live' ||
        connected && ['none', 'connecting'].includes(uwbStreamState),
        paused, elapsedMs: now - dualAt});
      for (const [role, suffix] of [['uwb_left','Left'], ['uwb_right','Right']]) {
        const d = dualView.sensors[role], id = 'dual' + suffix;
        $(id + 'Card').hidden = role === 'uwb_right' && dualView.profile === 'single_left';
        $(id + 'Card').dataset.state = d.state;
        text(id + 'State', d.label);
        text(id + 'Distance', d.rangeOut ? '범위 밖' : d.rangeM === null ? '—' : `${d.rangeM.toFixed(2)} m`);
        text(id + 'Angle', d.angleOut ? '범위 밖' : d.angleDeg === null ? '—' : `${d.angleDeg.toFixed(1)}°`);
        text(id + 'Validity', d.measurementValid === null ? '—' : `${d.measurementValid ? '측정 유효' : '측정 무효'} / ${d.idValid ? 'ID 일치' : 'ID 불일치'}`);
        text(id + 'Age', d.ageMs === null ? '—' : `${Math.round(d.ageMs)} ms`);
        text(id + 'Reason', d.reasonLabel || (d.valid ? '최신 관측 · 위치 정확도 검수 전' : '수신 대기'));
      }
      sensingView = {state: paused ? 'paused' : usable && dualView.valid ? 'ready' : 'degraded',
        label: paused ? '화면 정지' : usable && dualView.valid ? (dualView.profile === 'single_left' ? 'LiDAR·왼쪽 UWB 수신 중' : 'LiDAR·좌우 UWB 수신 중') :
          usable && dualView.received ? '센서 수신 중 · UWB 무효 표본 있음' : '센서 수신 상태 확인', ready: false};
    }
    const previewTrackingPayload = trackingInbox && trackingInbox.payload;
    const previewDualView = dualMode && window.DualUwbCore ? window.DualUwbCore.derive(
      previewDualInbox && previewDualInbox.payload, {connected: uwbStreamState === 'live' ||
        connected && ['none', 'connecting'].includes(uwbStreamState), paused,
        elapsedMs: previewDualInbox ? now - previewDualInbox.receivedAt : 0}) : null;
    let preview = P ? P.derive(previewDualView, trackingView, {enabled: $('fieldPreviewEnabled').checked,
      paused, sourceMode: envelope && envelope.source_mode, connected: uwbConnected, lidarUsable: usable,
      trackingConnected: Boolean(previewTrackingPayload && previewTrackingPayload.state === 'running' &&
        trackingView && Number.isFinite(trackingView.ageMs) && trackingView.ageMs < 750 &&
        (trackingDedicatedReceived ? trackingStreamState === 'live' : connected && !sourceError)),
      clustersUsable: Boolean(previewTrackingPayload && previewTrackingPayload.result &&
        previewTrackingPayload.result.lidar_clusters && previewTrackingPayload.result.lidar_clusters.valid === true &&
        Number.isFinite(previewTrackingPayload.cluster_remaining_ms) &&
        previewTrackingPayload.cluster_remaining_ms > now - trackingInbox.receivedAt)}) : null;
    if (previewSmoother && preview) {
      const sensors = previewDualInbox && previewDualInbox.payload && previewDualInbox.payload.sensors;
      preview = previewSmoother.update(preview, {nowMs: now,
        frameKey: trackingView && trackingView.transform ? JSON.stringify(trackingView.transform) : '',
        leftSeq: sensors && sensors.uwb_left && sensors.uwb_left.sample_sequence,
        rightSeq: sensors && sensors.uwb_right && sensors.uwb_right.sample_sequence});
    }
    const singlePreview = dualView && dualView.profile === 'single_left';
    $('fieldPreviewPanel').hidden = !dualMode || !P;
    $('fieldPreviewLegend').hidden = !dualMode || !P || !$('fieldPreviewEnabled').checked;
    text('fieldPreviewTitle', singlePreview ? '왼쪽 위치 가정 · 화면 실험' : '좌우 위치 비교 · 화면 실험');
    if (preview) {
      text('fieldPreviewState', preview.label);
      const previewPosition = preview.average || preview.disagreementAverage;
      text('fieldPreviewPosition', previewPosition ? `${previewPosition.x.toFixed(2)} / ${previewPosition.y.toFixed(2)} m${preview.disagreementAverage ? ' · 불일치' : ''}` : '—');
      text('fieldPreviewResidual', preview.residualM === null ? '—' : `${preview.residualM.toFixed(2)} m`);
      text('fieldPreviewCandidates', preview.candidateCount === null ? '—' : `${preview.candidateCount}개`);
    }
    renderer.setUwb(uwbView.marker);
    if (renderer.setTracking && trackingView) renderer.setTracking({ ...trackingView,
      uwb: uwbView.marker ? null : trackingView.uwb, fieldPreview: preview });
    text('sensingState', sensingView.label); $('sensingState').parentElement.dataset.state = sensingView.state;
    text('lidarState', state); text('uwbState', dualView ? dualView.label : uwbView.label);
    $('pauseNotice').hidden = !paused;
    const lidarAge = envelope && typeof envelope.age_ms === 'number' && Number.isFinite(envelope.age_ms) ?
      Math.max(0, envelope.age_ms) + now - envelopeAt : null;
    text('lidarAge', mode !== 'live' ? '비실시간 자료' : lidarAge === null ? '—' : `${Math.round(paused ? Math.max(lidarAge, frameAge + now - frameAt) : lidarAge)} ms`);
    text('lidarConnection', mode !== 'live' ? '로컬 보기' : !connected ? '서버 연결 대기' : envelope.source_mode === 'ros' ? 'ROS /scan · SSE 연결' : envelope.source_mode === 'demo' ? '서버 합성 소스' : '서버 파일 재생');
    const um = uwbView.measurement, validUwb = uwbView.state === 'live' || uwbView.state === 'synthetic';
    text('uwbConnection', uwbView.sourceLabel);
    text('uwbAddress', um ? um.tag_address : '—');
    text('uwbIds', um && um.tag_id !== null ? `${um.tag_id} / ${um.accepted_id}` : '—');
    text('uwbAuthentication', um ? `${um.authenticated ? 'ID 일치' : 'ID 불일치 / 무효'}${uwbView.synthetic ? ' · 합성' : ''}` : '—');
    text('uwbDistance', validUwb && um && um.filtered_distance_m !== null ? `${um.filtered_distance_m.toFixed(2)} m` : '—');
    text('uwbRawDistance', validUwb && um && um.raw_distance_m !== null ? `${um.raw_distance_m.toFixed(2)} m` : '—');
    text('uwbAngle', validUwb && um && um.angle_deg !== null ? `${um.angle_deg.toFixed(1)}°` : '—');
    text('uwbPosition', validUwb && um && um.position ? `${um.position.x.toFixed(2)} / ${um.position.y.toFixed(2)} m` : '—');
    text('uwbFirmware', um && um.firmware_state ? um.firmware_state : '—');
    text('uwbRxAge', `${uwbView.receiveHz === null ? '—' : `${uwbView.receiveHz.toFixed(1)} Hz`} / ${uwbView.ageMs === null ? '—' : `${Math.round(uwbView.ageMs)} ms`}`);
    text('uwbNotice', uwbView.reason || (paused ? '화면 정지 · 마지막 수신 순간을 표시합니다.' : uwbView.synthetic ? '합성 태그 · 실제 장치의 위치·상태가 아닙니다.' : 'ESP 방위 +는 BU04 오른쪽 · X/Y는 ROS 전방/좌측입니다.'));
    text('statusBadge', state); $('statusBadge').parentElement.dataset.active = String(usable && !paused && origin.badge === 'LIVE');
    text('plotLabel', origin.label);
    $('plotState').hidden = usable && !paused;
    text('plotState', state);
    text('notice', notice); $('notice').hidden = !notice;
    renderer.setStatus({ fresh: usable, paused: paused || mode === 'sample' || mode === 'file' && !replayPlaying || Boolean(end), label: origin.label, state });
    const stats = scan && scan.stats;
    text('coverage', !stats ? 'NO DATA' : `${scan.truncated ? 'PARTIAL ' : ''}${stats.provided_rays} / ${stats.expected_rays} RAYS`);
    text('visibleCount', usable && projection ? projection.points.length.toLocaleString() : '—');
    text('rxHz', mode !== 'live' ? 'LOCAL' : connected && envelope && !sourceError ? `${Number(envelope.receive_hz || 0).toFixed(1)} Hz` : '—');
    for (const [key, suffix] of Object.entries({ front: 'Front', right: 'Right', rear: 'Rear', left: 'Left' })) {
      const s = usable && projection ? projection.sectors[key] : null, d = s && s.min_m;
      text(`sector${suffix}`, d !== null && d !== undefined ? d.toFixed(d < 1 ? 3 : 2) : '—');
      $(`sector${suffix}`).title = s ? `${s.count}개 측정점 · 카트 중심 기준` : '유효 측정 없음';
    }
    const hasMeasurement = Boolean(scan || uwbView.measurement || trackingView && trackingView.configured);
    $('pauseView').disabled = !hasMeasurement; $('savePng').disabled = !hasMeasurement; $('saveJson').disabled = !hasMeasurement;
    if ($('settingsDialog').open) {
      text('frameId', scan ? String(scan.header.frame_id || '—') : '—');
      text('rayCount', stats ? `${stats.provided_rays} / ${stats.expected_rays}` : '—');
      text('metadataHz', scan ? `${(scan.scan_time * 1000).toFixed(1)} ms` : '—');
      text('scanAge', !scan ? '—' : mode === 'sample' || mode === 'file' || end ? 'STORED' : `${Math.round(frameAge + now - frameAt)} ms`);
      text('invalidCount', stats ? stats.no_return_rays : '—'); text('belowCount', stats ? stats.below_min_rays : '—');
      text('aboveCount', stats ? stats.above_max_rays : '—'); text('outsideCount', projection ? projection.outside_view : '—');
    }
  }
  function reconfigure(reconnect = false) {
    settings = C.validateSettings(settings); persist(); syncControls();
    if (scan) { projection = C.projectScan(scan, projectionSettings()); renderer.setFrame(scan, projection, frameKey); }
    renderer.configure(projectionSettings());
    if (reconnect && mode === 'live') { session++; connectServer(); }
    else if (reconnect && mode === 'demo') {
      // Change the timer without unpausing or rewinding the current scan.
      if (timer) clearInterval(timer);
      timer = setInterval(() => { const t = performance.now() / 1000; applyFrame(C.syntheticScan(t), `demo:${++sequence}`); applyUwb(S.syntheticUwb(t)); updateStatus(); }, 1000 / settings.displayHz);
    } else if (reconnect && mode === 'file') startReplayTimer();
    updateStatus();
  }
  function showReplay(index) {
    if (!localFrames.length) return;
    replayIndex = Math.max(0, Math.min(localFrames.length - 1, index));
    applyFrame(localFrames[replayIndex], `file:${session}:${replayIndex}`, 0, true);
    $('replayPosition').max = String(localFrames.length - 1); $('replayPosition').value = String(replayIndex);
    $('replayPlay').disabled = localFrames.length < 2; text('replayLabel', `${replayIndex + 1} / ${localFrames.length}`);
    text('replayPlay', replayPlaying ? '정지' : '재생'); updateStatus();
  }
  function startReplayTimer() {
    if (timer) clearInterval(timer);
    timer = setInterval(() => {
      if (!replayPlaying || paused) return;
      let index = replayStart, elapsed = (performance.now() - replayClock) / 1000;
      while (index < localFrames.length - 1) {
        const interval = Math.max(.01, Math.min(5, localFrames[index].scan_time || .1));
        if (elapsed < interval) break; elapsed -= interval; index++;
      }
      if (index === localFrames.length - 1) replayPlaying = false;
      showReplay(index);
    }, 1000 / settings.displayHz);
  }
  async function loadFile(file) {
    if (!file) return;
    if (file.size > 25 * 1024 * 1024) { toast('25 MiB 초과: Python 서버의 replay 모드를 사용하십시오.'); return; }
    try {
      const raw = (await file.text()).replace(/^\uFEFF/, '').trim(); let objects;
      try { const data = JSON.parse(raw); objects = Array.isArray(data) ? data : [data]; }
      catch (_) { objects = raw.split(/\r?\n/).filter(line => line.trim()).map(line => JSON.parse(line)); }
      if (!objects.length || objects.length > 20000) throw new Error('프레임 한도: 1–20,000');
      localFrames = objects.map(data => C.normaliseScan(data, { kind: 'replay', label: file.name }));
      setMode('file'); $('settingsDialog').close();
    } catch (error) { toast(`파일 오류: ${error.message}`); }
  }
  function download(blob, ext) {
    const url = URL.createObjectURL(blob), link = document.createElement('a');
    link.href = url; link.download = `smartcart-${scan ? scan.source.kind : 'uwb'}-${new Date().toISOString().replace(/[:.]/g, '-')}.${ext}`;
    document.body.appendChild(link); link.click(); link.remove(); setTimeout(() => URL.revokeObjectURL(url), 15000);
  }
  $('source').addEventListener('change', () => {
    const next = $('source').value;
    if (next === 'file') { $('source').value = mode; $('fileInput').click(); }
    else setMode(next);
  });
  $('openFile').addEventListener('click', () => $('fileInput').click());
  $('fileInput').addEventListener('change', event => { loadFile(event.target.files[0]); event.target.value = ''; });
  $('radius').addEventListener('change', () => { settings.radius = Number($('radius').value); reconfigure(); });
  $('displayHz').addEventListener('change', () => { settings.displayHz = Number($('displayHz').value); reconfigure(true); });
  $('pauseView').addEventListener('click', () => {
    paused = !paused; text('pauseView', paused ? '화면 재개' : '화면 정지');
    if (!paused && pending) { const frame = pending; pending = null; applyFrame(frame.next, frame.key, frame.age + performance.now() - frame.at, true); }
    if (!paused && pendingUwb) { uwbEnvelope = pendingUwb.next || null; uwbAt = pendingUwb.at; pendingUwb = null; }
    if (mode === 'file') { replayClock = performance.now(); replayStart = replayIndex; }
    updateStatus();
  });
  $('openSettings').addEventListener('click', () => { syncControls(); $('settingsDialog').showModal(); updateStatus(); });
  $('fieldPreviewEnabled').addEventListener('change', updateStatus);
  $('closeSettings').addEventListener('click', () => $('settingsDialog').close());
  $('settingsDialog').addEventListener('click', event => {
    if (event.target !== $('settingsDialog')) return;
    const r = event.target.getBoundingClientRect();
    if (event.clientX < r.left || event.clientX > r.right || event.clientY < r.top || event.clientY > r.bottom) event.target.close();
  });
  $('settingsForm').addEventListener('submit', event => {
    event.preventDefault();
    const near = Number($('nearM').value), far = Number($('farM').value);
    if (near >= far) { $('farM').setCustomValidity('초록 시작 거리는 빨강 기준보다 커야 합니다.'); $('farM').reportValidity(); return; }
    for (const key of ['yaw', 'offsetX', 'offsetY', 'nearM', 'farM', 'maxGap'])
      if (!trackingTransform || !['yaw', 'offsetX', 'offsetY'].includes(key)) settings[key] = Number($(key).value);
    for (const key of ['contoursEnabled', 'sweepEnabled']) settings[key] = $(key).checked;
    reconfigure(); $('settingsDialog').close();
  });
  for (const key of ['nearM', 'farM']) $(key).addEventListener('input', () => $('farM').setCustomValidity(''));
  $('resetSettings').addEventListener('click', () => { settings = { ...C.defaults }; $('farM').setCustomValidity(''); reconfigure(true); });
  $('replayPlay').addEventListener('click', () => {
    if (replayIndex === localFrames.length - 1) showReplay(0);
    replayPlaying = !replayPlaying; paused = false; text('pauseView', '화면 정지');
    replayClock = performance.now(); replayStart = replayIndex; text('replayPlay', replayPlaying ? '정지' : '재생'); updateStatus();
  });
  $('replayPosition').addEventListener('input', () => { replayPlaying = false; paused = false; text('pauseView', '화면 정지'); showReplay(Number($('replayPosition').value)); });
  $('saveJson').addEventListener('click', () => {
    if (!scan && !uwbView.measurement && !(trackingView && trackingView.configured)) return;
    updateStatus();
    const capture = { schema: 'smartcart.capture.v1', version: '1.0.1', exported_at: new Date().toISOString(),
      integration_version: '0.1.0', view_mode: mode, view_status: $('statusBadge').textContent, display_config: settings, scan,
      sensing: sensingView, uwb: { ...uwbView, snapshot_paused: paused }, tracking: trackingView };
    download(new Blob([JSON.stringify(capture, null, 2)], { type: 'application/json' }), 'json');
    toast('JSON 저장 · 태그·위치·추적 정보가 포함될 수 있습니다.');
  });
  $('savePng').addEventListener('click', () => {
    if (!scan && !uwbView.measurement && !(trackingView && trackingView.configured)) return;
    updateStatus();
    renderer.exportCanvas().toBlob(blob => { if (blob) download(blob, 'png'); else toast('PNG 저장 실패'); }, 'image/png');
    toast('PNG 저장 · 화면의 위치·추적 정보가 포함됩니다.');
  });
  if (!document.documentElement.requestFullscreen) $('fullscreen').hidden = true;
  $('fullscreen').addEventListener('click', async () => {
    try { if (document.fullscreenElement) await document.exitFullscreen(); else await document.documentElement.requestFullscreen(); }
    catch (_) { toast('전체 화면을 사용할 수 없습니다.'); }
  });
  document.addEventListener('fullscreenchange', () => text('fullscreen', document.fullscreenElement ? '전체 화면 종료' : '전체 화면'));
  window.addEventListener('beforeunload', stopProducer);
  const requested = new URLSearchParams(location.search).get('source');
  syncControls(); setMode(['sample', 'demo', 'live'].includes(requested) ? requested : location.protocol === 'file:' ? 'sample' : 'live');
  setInterval(updateStatus, 200);
  window.SmartCartDiagnostics = {
    get mode() { return mode; }, get settings() { return { ...settings }; }, get state() { return $('statusBadge').textContent; },
    get points() { return projection ? projection.points.length : 0; }, get links() { return projection ? projection.links.length : 0; },
    get active() { return active; }, get packets() { return packets; }, get scan() { return scan; }, get projection() { return projection; },
    get renderedFrames() { return renderer.frameCount; }, get renderer() { return renderer; },
    get uwb() { return uwbView; }, get sensing() { return sensingView; }, get paused() { return paused; },
    get tracking() { return trackingView; }, get trackingPayload() { return trackingInbox && trackingInbox.payload; },
    get trackingStream() { return trackingStreamState; }, get displayProjectionSettings() { return { ...projectionSettings() }; }
  };
})();
