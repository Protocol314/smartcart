/* Pure scan geometry. Shared by browser and Node regression tests. */
(function (root, factory) {
  const api = factory();
  if (typeof module === 'object' && module.exports) module.exports = api;
  else root.RadarCore = api;
})(typeof globalThis !== 'undefined' ? globalThis : this, function () {
  'use strict';
  const TAU = Math.PI * 2;
  const num = value => typeof value === 'number' && Number.isFinite(value);
  const defaults = Object.freeze({ radius: 5, displayHz: 2, yaw: 0, offsetX: 0, offsetY: 0,
    nearM: 1, farM: 3, maxGap: 0.2, contoursEnabled: true, sweepEnabled: true });

  function validateSettings(value) {
    const v = value && typeof value === 'object' ? value : {};
    const s = {};
    // Allowlist only. Old persistence/trails must not re-enable accumulated scans.
    for (const key of Object.keys(defaults)) s[key] = v[key] === undefined ? defaults[key] : v[key];
    if (![0.25, 0.5, 1, 2, 3, 5].includes(s.radius)) s.radius = 5;
    if (![1, 2].includes(s.displayHz)) s.displayHz = 2;
    if (!num(s.yaw) || s.yaw < -180 || s.yaw > 180) s.yaw = 0;
    for (const key of ['offsetX', 'offsetY']) if (!num(s[key]) || Math.abs(s[key]) > 2) s[key] = 0;
    if (!num(s.nearM) || !num(s.farM) || s.nearM < 0.05 || s.nearM > 15 || s.farM <= s.nearM || s.farM > 16) {
      s.nearM = 1; s.farM = 3;
    }
    if (!num(s.maxGap) || s.maxGap < 0.02 || s.maxGap > 0.5) s.maxGap = 0.2;
    s.contoursEnabled = s.contoursEnabled !== false;
    s.sweepEnabled = s.sweepEnabled !== false;
    return s;
  }

  function normaliseScan(input, overrideSource) {
    const d = input && input.scan ? input.scan : input;
    if (!d || typeof d !== 'object') throw new Error('스캔 객체가 없습니다.');
    for (const key of ['angle_min', 'angle_max', 'angle_increment', 'range_min', 'range_max']) {
      if (!num(d[key])) throw new Error(`유효한 ${key} 값이 필요합니다.`);
    }
    const expected = Math.round((d.angle_max - d.angle_min) / d.angle_increment) + 1;
    if (!d.angle_increment || (d.angle_max - d.angle_min) / d.angle_increment < 0 || expected < 1 || expected > 100000 || Math.abs(d.angle_max - d.angle_min) > TAU + 0.1) throw new Error('각도 메타데이터가 일치하지 않습니다.');
    if (!(d.range_min >= 0 && d.range_max > d.range_min && d.range_max <= 100000)) throw new Error('센서 거리 범위 오류');
    if (!Array.isArray(d.ranges) || !d.ranges.length || d.ranges.length > 100000) throw new Error('ranges 배열 오류');
    let truncated = Boolean(d.truncated);
    let values = d.ranges;
    const marker = values.indexOf('...');
    if (marker >= 0) {
      if (marker !== values.length - 1) throw new Error('중간 생략 구간의 각도를 복원할 수 없습니다.');
      values = values.slice(0, -1); truncated = true;
    }
    if (!values.length) throw new Error('스캔 항목이 없습니다.');
    for (const v of values) if (v !== null && typeof v !== 'number') throw new Error('ranges에는 숫자 또는 null만 사용할 수 있습니다.');
    const ranges = values.map(v => num(v) ? v : null);
    truncated = truncated || ranges.length < expected;
    let noReturn = 0, below = 0, above = 0;
    for (const v of ranges) {
      if (v === null) noReturn++;
      else if (v < d.range_min) below++;
      else if (v > d.range_max) above++;
    }
    const warnings = [];
    if (truncated) warnings.push('TRUNCATED_SCAN');
    if (expected !== ranges.length) warnings.push('RAY_COUNT_MISMATCH');
    if (below) warnings.push('BELOW_SENSOR_MINIMUM');
    const valid = ranges.length - noReturn - below - above;
    if (!valid) warnings.push('NO_VALID_RETURNS');
    const original = d.source && typeof d.source === 'object' ? d.source : {};
    const source = overrideSource ? { ...original, original_kind: original.original_kind || original.kind || null, ...overrideSource } : { kind: 'replay', label: 'File', ...original };
    const intensities = Array.isArray(d.intensities) ? d.intensities.slice(0, 100000).filter(v => v !== '...').map(v => num(v) ? v : null) : [];
    if (intensities.length && intensities.length !== ranges.length) warnings.push('INTENSITY_COUNT_MISMATCH');
    return { schema: 'smartcart.scan.v1', source, sequence: d.sequence, received_unix: d.received_unix,
      header: d.header || { frame_id: 'laser', stamp: { sec: 0, nanosec: 0 } },
      angle_min: d.angle_min, angle_max: d.angle_max, angle_increment: d.angle_increment,
      scan_time: num(d.scan_time) && d.scan_time > 0 ? d.scan_time : 0,
      time_increment: num(d.time_increment) ? Math.max(0, d.time_increment) : 0,
      range_min: d.range_min, range_max: d.range_max, ranges, intensities, truncated, warnings,
      stats: { provided_rays: ranges.length, expected_rays: expected, valid_sensor_rays: valid,
        no_return_rays: noReturn, below_min_rays: below, above_max_rays: above,
        metadata_scan_hz: d.scan_time > 0 ? 1 / d.scan_time : null,
        provided_span_deg: Math.abs(d.angle_increment) * (ranges.length - 1) * 180 / Math.PI } };
  }

  function projectScan(scan, settings) {
    const s = validateSettings(settings), yaw = s.yaw * Math.PI / 180;
    const sectors = Object.fromEntries(['front', 'left', 'right', 'rear'].map(n => [n, { min_m: null, count: 0 }]));
    const points = [], bins = new Array(36).fill(0);
    let outside = 0;
    scan.ranges.forEach((r, i) => {
      if (!num(r) || r < scan.range_min || r > scan.range_max) return;
      const a = scan.angle_min + i * scan.angle_increment;
      const x = r * Math.cos(a), y = r * Math.sin(a), distance = r;
      if (distance > s.radius) { outside++; return; }
      const bearing = Math.atan2(y, x) * 180 / Math.PI;
      const key = bearing >= -45 && bearing < 45 ? 'front' : bearing >= 45 && bearing < 135 ? 'left' : bearing >= -135 && bearing < -45 ? 'right' : 'rear';
      const sector = sectors[key]; sector.count++;
      sector.min_m = sector.min_m === null ? distance : Math.min(sector.min_m, distance);
      bins[Math.min(35, Math.floor((bearing + 180) / 10))]++;
      points.push({ index: i, x, y, distance, range: r, bearing_deg: bearing });
    });
    const contours = buildContours(points, scan.angle_increment, s);
    return { points, sectors, outside_view: outside, bins, ...contours };
  }

  /** O(N) conservative adjacency, not object classification or a fitted surface.
   * Original beam indices are required: an invalid/cropped ray breaks the chain.
   * Never connect the last ray to the first ray, even on a 360-degree scan.
   */
  function buildContours(points, angleStep, settings) {
    const s = validateSettings(settings), runs = [], links = [];
    let run = null, previous = null;
    const step = Math.abs(angleStep);
    for (const point of points) {
      const angularSpacing = previous ? 2 * Math.min(previous.range, point.range) * Math.sin(step / 2) : 0;
      const gate = Math.min(s.maxGap, 0.035 + 3 * angularSpacing);
      const join = previous && point.index === previous.index + 1 && step <= Math.PI / 90 &&
        Math.hypot(point.x - previous.x, point.y - previous.y) <= gate;
      if (!join) { run = []; runs.push(run); }
      else links.push([previous, point]);
      run.push(point); previous = point;
    }
    return { runs, links };
  }

  function colorBand(distance, settings) {
    return distance < settings.nearM ? 0 : distance < settings.farM ? 1 : 2;
  }

  /** Split a display guide at cart-centered color boundaries, not new returns. */
  function coloredEdge(a, b, settings) {
    const dx = b.x - a.x, dy = b.y - a.y, length2 = dx * dx + dy * dy;
    const cuts = [0, 1];
    if (length2 > 1e-18) {
      for (const radius of [settings.nearM, settings.farM]) {
        const dot = a.x * dx + a.y * dy;
        const disc = dot * dot - length2 * (a.x * a.x + a.y * a.y - radius * radius);
        if (disc >= 0) for (const t of [(-dot - Math.sqrt(disc)) / length2, (-dot + Math.sqrt(disc)) / length2]) {
          if (t > 1e-9 && t < 1 - 1e-9) cuts.push(t);
        }
      }
    }
    cuts.sort((x, y) => x - y);
    const out = [];
    for (let i = 1; i < cuts.length; i++) {
      const lo = cuts[i - 1], hi = cuts[i];
      if (hi - lo < 1e-9) continue;
      const mid = (lo + hi) / 2;
      out.push({ a: { x: a.x + dx * lo, y: a.y + dy * lo },
        b: { x: a.x + dx * hi, y: a.y + dy * hi },
        band: colorBand(Math.hypot(a.x + dx * mid, a.y + dy * mid), settings) });
    }
    return out;
  }

  // 화면만 좌우 반전하며 ROS 좌표와 센서 측정값은 변경하지 않는다.
  function toScreen(point, cx, cy, scale) { return { x: cx + point.y * scale, y: cy - point.x * scale }; }

  // 표기 전용: 원래 좌표의 X 부호를 바꾸고 표시 원점을 1.25m 이동한다.
  function displayX(x) { return -x - 1.25; }

  function syntheticScan(t) {
    const n = 720, ranges = [];
    const circles = [[0.8, 0.45, 0.16], [1.5 + 0.5 * Math.sin(t * 0.7), -1.1 + 0.3 * Math.cos(t * 0.5), 0.24], [-1.9, 1.7, 0.32], [2.7, 2.4, 0.38]];
    const walls = [[[-3.5,-4.2],[4.3,-4.2]],[[4.3,-4.2],[4.3,3.8]],[[4.3,3.8],[-3.5,3.8]],[[-3.5,3.8],[-3.5,-4.2]],[[0.9,2],[1.7,2]],[[1.7,2],[1.7,2.8]],[[1.7,2.8],[0.9,2.8]],[[0.9,2.8],[0.9,2]],[[-1.8,-2.4],[-0.3,-2.4]]];
    for (let i = 0; i < n; i++) {
      const a = -Math.PI + i * TAU / (n - 1), dx = Math.cos(a), dy = Math.sin(a);
      let hit = 16;
      for (const [[ax,ay],[bx,by]] of walls) {
        const ex = bx-ax, ey=by-ay, den=dx*ey-dy*ex;
        if (Math.abs(den)<1e-9) continue;
        const d=(ax*ey-ay*ex)/den, u=(ax*dy-ay*dx)/den;
        if(d>0&&u>=0&&u<=1)hit=Math.min(hit,d);
      }
      for (const [cx,cy,r] of circles) {
        const b=dx*cx+dy*cy, disc=b*b-(cx*cx+cy*cy-r*r);
        if(disc>=0&&b>0){const d=b-Math.sqrt(disc);if(d>0)hit=Math.min(hit,d);}
      }
      ranges.push(i%113===0?null:hit+0.006*Math.sin(i*11.3+t*4));
    }
    return normaliseScan({ header:{frame_id:'synthetic_laser',stamp:{sec:0,nanosec:0}},
      angle_min:-Math.PI,angle_max:Math.PI,angle_increment:TAU/(n-1),scan_time:0.1,time_increment:0.1/(n-1),
      range_min:0.05,range_max:16,ranges,source:{kind:'synthetic',label:'합성 데모 · 실측 아님'} });
  }
  return { TAU, defaults, validateSettings, normaliseScan, projectScan, buildContours, colorBand, coloredEdge, toScreen, displayX, syntheticScan };
});
