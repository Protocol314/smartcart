/* 격자·LiDAR·회전선·UWB를 분리해 고속 UWB 갱신이 LiDAR를 다시 계산하지 않는다. */
(function () {
  'use strict';
  const C = window.RadarCore;
  // LiDAR 원본 180°를 화면 위쪽으로 표시한다.
  const lidarToScreen = (point, cx, cy, scale) => C.toScreen({x: -point.x, y: point.y}, cx, cy, scale);
  const COLORS = ['#ff4b4b', '#ffd34d', '#36dd83'];
  const FONT = 'ui-monospace, "Cascadia Code", Consolas, monospace';
  class RadarRenderer {
    constructor(canvas) {
      this.canvas = canvas;
      this.grid = document.getElementById('radarGrid');
      this.sweep = document.getElementById('radarSweep');
      this.uwbCanvas = document.getElementById('radarUwb');
      this.trackingCanvas = document.getElementById('radarTracking');
      this.ctx = canvas.getContext('2d');
      this.gridCtx = this.grid.getContext('2d', { alpha: false });
      this.sweepCtx = this.sweep.getContext('2d');
      this.uwbCtx = this.uwbCanvas.getContext('2d');
      this.trackingCtx = this.trackingCanvas.getContext('2d');
      if (!this.ctx || !this.gridCtx || !this.sweepCtx || !this.uwbCtx || !this.trackingCtx) throw new Error('Canvas 2D를 사용할 수 없습니다.');
      this.settings = { ...C.defaults };
      this.view = { fresh: false, paused: false, scan: null, projection: null, label: 'NO DATA', state: 'WAITING' };
      this.uwb = null; this.uwbKey = '';
      this.tracking = null; this.trackingKey = ''; this.trackingBuilds = 0;
      this.width = 320; this.height = 320; this.dpr = 1; this.phase = -Math.PI / 2;
      this.selected = null; this.frameCount = 0; this.scanBuilds = 0; this.gridBuilds = 0; this.uwbBuilds = 0;
      this.lastDraw = 0; this.raf = null;
      this.motion = window.matchMedia('(prefers-reduced-motion: reduce)');
      this.tick = this.tick.bind(this);
      this.observer = new ResizeObserver(() => this.resize());
      this.observer.observe(canvas.parentElement);
      canvas.addEventListener('click', event => this.selectPoint(event));
      canvas.addEventListener('keydown', event => { if (event.key === 'Escape') { this.selected = null; this.drawScan(); } });
      document.addEventListener('visibilitychange', () => {
        if (document.hidden && this.raf !== null) { cancelAnimationFrame(this.raf); this.raf = null; }
        else this.wake();
      });
      this.motion.addEventListener('change', () => { this.drawSweep(performance.now()); this.wake(); });
      window.addEventListener('resize', () => this.resize());
      this.resize();
    }
    prepare(ctx) { ctx.setTransform(this.dpr, 0, 0, this.dpr, 0, 0); }
    geometry() {
      const side = Math.min(this.width, this.height);
      return { cx: this.width / 2, cy: this.height / 2, R: Math.max(20, side / 2 - (side < 400 ? 34 : 42)) };
    }
    resize() {
      const box = this.canvas.parentElement.getBoundingClientRect();
      const w = Math.max(1, box.width), h = Math.max(1, box.height);
      const dpr = Math.min(window.devicePixelRatio || 1, 2, 1600 / Math.max(w, h));
      if (this.width === w && this.height === h && this.dpr === dpr && this.gridBuilds) return;
      this.width = w; this.height = h; this.dpr = dpr;
      for (const canvas of [this.grid, this.sweep, this.canvas, this.uwbCanvas, this.trackingCanvas]) {
        canvas.width = Math.round(w * dpr); canvas.height = Math.round(h * dpr);
      }
      this.drawGrid(); this.drawScan(); this.drawUwbOverlay(); this.drawTracking(); this.drawSweep(performance.now()); this.wake();
    }
    configure(settings) {
      this.settings = C.validateSettings(settings);
      this.selected = null; this.drawGrid(); this.drawScan(); this.drawUwbOverlay(); this.drawTracking(); this.drawSweep(performance.now()); this.wake();
    }
    setFrame(scan, projection, key) {
      this.view.scan = scan; this.view.projection = projection; this.uniqueId = key;
      this.selected = null; this.drawScan(); this.wake();
    }
    setUwb(marker) {
      const key = marker ? JSON.stringify(marker) : '';
      if (key === this.uwbKey) return;
      this.uwb = marker; this.uwbKey = key; this.drawUwbOverlay();
    }
    setStatus(status) {
      const redraw = this.view.fresh !== status.fresh;
      const animateChanged = this.view.paused !== status.paused;
      Object.assign(this.view, status);
      if (redraw) this.drawScan();
      if (redraw || animateChanged) this.drawSweep(performance.now());
      this.wake();
    }
    setTracking(view) {
      const next = view ? { clusters: view.clusters, selected: view.selected, prediction: view.prediction,
        uwb: view.uwb, sourceLabel: view.sourceLabel, fieldPreview: view.fieldPreview } : null;
      const key = next ? JSON.stringify(next) : '';
      if (key === this.trackingKey) return;
      this.tracking = next; this.trackingKey = key; this.drawTracking();
    }
    drawTracking() {
      const g = this.trackingCtx; this.prepare(g); g.clearRect(0, 0, this.width, this.height);
      this.trackingBuilds++;
      if (!this.tracking) return;
      const { cx, cy, R } = this.geometry(), scale = R / this.settings.radius;
      g.save(); g.beginPath(); g.arc(cx, cy, R, 0, C.TAU); g.clip();
      for (const cluster of ([] /* cluster overlay hidden */)) {
        const p = C.toScreen(cluster, cx, cy, scale);
        const selected = this.tracking.selected && this.tracking.selected.clusterId === cluster.id;
        g.strokeStyle = selected ? '#d8b4fe' : '#8792a5'; g.lineWidth = selected ? 2 : 1;
        g.beginPath(); g.arc(p.x, p.y, Math.max(3, cluster.radius * scale), 0, C.TAU); g.stroke();
        g.fillStyle = '#bac4d1'; g.beginPath(); g.arc(p.x, p.y, 2, 0, C.TAU); g.fill();
        this.text(g, `C${cluster.id}`, p.x + 6, p.y + 11, 9, 'left', '#bac4d1');
      }
      const target = this.tracking.selected || this.tracking.prediction;
      if (target) {
        const p = C.toScreen(target, cx, cy, scale), measured = Boolean(this.tracking.selected);
        g.strokeStyle = measured ? '#ead5ff' : '#e2bc65'; g.fillStyle = '#d8b4fe'; g.lineWidth = 2;
        g.setLineDash(measured ? [] : [4, 3]);
        g.beginPath(); g.arc(p.x, p.y, measured ? 7 : 9, 0, C.TAU);
        if (measured) g.fill(); g.stroke(); g.setLineDash([]);
        this.text(g, `${measured ? '대상' : '예측'} ${target.id === null ? '—' : '#' + target.id}`,
          p.x + 12, p.y - 11, 10, 'left', measured ? '#ead5ff' : '#e2bc65');
      }
      if (this.tracking.uwb) {
        const p = C.toScreen(this.tracking.uwb, cx, cy, scale), size = 6;
        g.strokeStyle = '#62e4ef'; g.lineWidth = 1.5; g.beginPath();
        g.moveTo(p.x, p.y - size); g.lineTo(p.x + size, p.y); g.lineTo(p.x, p.y + size);
        g.lineTo(p.x - size, p.y); g.closePath(); g.stroke();
      }
      const preview = this.tracking.fieldPreview;
      if (preview) {
        g.strokeStyle = '#eabf7e'; g.lineWidth = 1.5; g.setLineDash([3, 3]);
        if (!preview.average) {
          for (const [label, point] of [['L 가정', preview.left], ['R 가정', preview.right]]) {
            if (!point) continue;
            const p = C.toScreen(point, cx, cy, scale);
            g.beginPath(); g.arc(p.x, p.y, 5, 0, C.TAU); g.stroke();
            this.text(g, label, p.x + 7, p.y + 11, 9, 'left', '#eabf7e');
          }
        }
        g.setLineDash([]);
        if (preview.average) {
          const p = C.toScreen(preview.average, cx, cy, scale);
          g.strokeStyle = '#ffd390'; g.lineWidth = 2; g.beginPath(); g.arc(p.x, p.y, 8, 0, C.TAU); g.stroke();
          this.text(g, preview.state === 'single' ? '왼쪽 가정' : '가정 평균', p.x + 11, p.y - 10, 10, 'left', '#ffd390');
        }
        if (preview.disagreementAverage) {
          const p = C.toScreen(preview.disagreementAverage, cx, cy, scale);
          g.strokeStyle = '#ff8b73'; g.lineWidth = 2; g.beginPath();
          g.moveTo(p.x - 6, p.y - 6); g.lineTo(p.x + 6, p.y + 6);
          g.moveTo(p.x + 6, p.y - 6); g.lineTo(p.x - 6, p.y + 6); g.stroke();
          this.text(g, '불일치 평균', p.x + 10, p.y - 10, 10, 'left', '#ff8b73');
        }
        if (preview.candidate) {
          const p = C.toScreen(preview.candidate, cx, cy, scale);
          g.strokeStyle = '#ffd390'; g.lineWidth = 1.5; g.strokeRect(p.x - 7, p.y - 7, 14, 14);
          this.text(g, `인접 C${preview.candidate.clusterId}`, p.x + 10, p.y + 12, 9, 'left', '#ffd390');
        }
      }
      g.restore();
    }
    text(ctx, text, x, y, size = 11, align = 'left', color = '#b5b5b5') {
      ctx.font = `${size}px ${FONT}`; ctx.fillStyle = color; ctx.textAlign = align; ctx.textBaseline = 'middle';
      ctx.fillText(text, x, y);
    }
    drawGrid() {
      const g = this.gridCtx, { cx, cy, R } = this.geometry(); this.prepare(g);
      g.fillStyle = '#000'; g.fillRect(0, 0, this.width, this.height);
      g.lineWidth = 1;
      // Sparse polar guides, not a Cartesian map or a free-space estimate.
      g.strokeStyle = '#181818'; g.beginPath();
      for (let deg = 0; deg < 360; deg += 30) {
        const a = deg * Math.PI / 180;
        g.moveTo(cx, cy); g.lineTo(cx + Math.cos(a) * R, cy + Math.sin(a) * R);
      }
      g.stroke();
      for (let ring = 1; ring <= 5; ring++) {
        const r = ring * R / 5;
        g.strokeStyle = ring === 5 ? '#777' : '#303030';
        g.beginPath(); g.arc(cx, cy, r, 0, C.TAU); g.stroke();
        const d = this.settings.radius * ring / 5;
        this.text(g, `${Number(d.toFixed(2))} m`, cx + 8, cy - r + 12, this.width < 380 ? 9 : 11);
      }
      g.strokeStyle = '#4c4c4c'; g.beginPath();
      g.moveTo(cx - R, cy); g.lineTo(cx + R, cy); g.moveTo(cx, cy - R); g.lineTo(cx, cy + R); g.stroke();
      g.strokeStyle = '#5b5b5b'; g.beginPath();
      for (let deg = 0; deg < 360; deg += 10) {
        const a = deg * Math.PI / 180;
        const len = deg % 90 === 0 ? 11 : deg % 30 === 0 ? 7 : 3;
        g.moveTo(cx + Math.cos(a) * (R + 1), cy + Math.sin(a) * (R + 1));
        g.lineTo(cx + Math.cos(a) * (R + len), cy + Math.sin(a) * (R + len));
      }
      g.stroke();
      const small = this.width < 380;
      this.text(g, '0°', cx, cy - R - 22, small ? 10 : 12, 'center', '#fff');
      // 각도는 ROS 좌표 기준이며 화면 위치만 좌우 반전한다.
      this.text(g, '−90°', cx - R - 7, cy - 10, small ? 9 : 11, 'right');
      this.text(g, '90°', cx + R + 7, cy - 10, small ? 9 : 11, 'left');
      this.text(g, '180°', cx, cy + R + 23, small ? 10 : 12, 'center');
      this.drawCart(g, cx, cy);
      if (this.settings.offsetX || this.settings.offsetY) {
        const p = C.toScreen({ x: this.settings.offsetX, y: this.settings.offsetY }, cx, cy, R / this.settings.radius);
        if (Math.hypot(p.x - cx, p.y - cy) < R) {
          g.strokeStyle = '#aaa'; g.beginPath(); g.arc(p.x, p.y, 3, 0, C.TAU); g.stroke();
        }
      }
      this.gridBuilds++;
    }
    drawCart(g, cx, cy) {
      const scale = this.geometry().R / this.settings.radius;
      const width = 0.6 * scale;
      const length = 1.0 * scale;

      g.save();
      g.translate(cx, cy);
      g.strokeStyle = '#e8e8e8';
      g.lineWidth = 1.5;
      g.strokeRect(-width / 2, 0, width, length);

      // 레이더 원점 = 차체 앞끝 중앙에 장착한 LiDAR.
      g.fillStyle = '#62e4ef';
      g.beginPath();
      g.arc(0, 0, 3, 0, C.TAU);
      g.fill();
      g.restore();
    }
    drawScan() {
      const g = this.ctx, { cx, cy, R } = this.geometry(); this.prepare(g);
      g.clearRect(0, 0, this.width, this.height);
      document.getElementById('pointReadout').hidden = true;
      if (!this.view.fresh || !this.view.projection) { this.scanBuilds++; return; }
      const { points, links } = this.view.projection, scale = R / this.settings.radius;
      const pointPaths = COLORS.map(() => new Path2D()), linePaths = COLORS.map(() => new Path2D());
      // Every filled dot is one real valid sample. Do not decimate into object centroids.
      const linkedPoints = new Set(links.flat());
      for (const point of points) {
        if (!linkedPoints.has(point)) continue;
        const p = lidarToScreen(point, cx, cy, scale), path = pointPaths[C.colorBand(point.distance, this.settings)];
        const size = this.width < 420 ? 1.05 : 1.3;
        path.moveTo(p.x + size, p.y); path.arc(p.x, p.y, size, 0, C.TAU);
      }
      if (this.settings.contoursEnabled) for (const [a, b] of links) {
        // Thin guides only between conservative neighboring returns. Never fill polygons.
        for (const edge of C.coloredEdge(a, b, this.settings)) {
          const p = lidarToScreen(edge.a, cx, cy, scale), q = lidarToScreen(edge.b, cx, cy, scale);
          const path = linePaths[edge.band]; path.moveTo(p.x, p.y); path.lineTo(q.x, q.y);
        }
      }
      g.save(); g.beginPath(); g.arc(cx, cy, R, 0, C.TAU); g.clip();
      g.lineWidth = 2.5; g.lineCap = 'round'; g.globalAlpha = .95;
      for (let band = 0; band < 3; band++) { g.strokeStyle = COLORS[band]; g.stroke(linePaths[band]); }
      g.globalAlpha = 1;
      for (let band = 0; band < 3; band++) { g.fillStyle = COLORS[band]; g.fill(pointPaths[band]); }
      g.restore();
      if (this.selected) {
        const p = lidarToScreen(this.selected, cx, cy, scale);
        g.strokeStyle = '#fff'; g.lineWidth = 1; g.beginPath(); g.arc(p.x, p.y, 5, 0, C.TAU); g.stroke();
        const out = document.getElementById('pointReadout');
        out.textContent = `#${this.selected.index} · LiDAR 실측 거리 ${this.selected.range.toFixed(3)} m · 표시 X ${(-this.selected.x).toFixed(2)} Y ${this.selected.y.toFixed(2)}`;
        out.hidden = false;
      }
      this.scanBuilds++;
    }
    drawUwbOverlay() {
      const g = this.uwbCtx; this.prepare(g); g.clearRect(0, 0, this.width, this.height);
      this.drawUwb(g); this.uwbBuilds++;
    }
    drawUwb(g) {
      if (!this.uwb) return;
      const { cx, cy, R } = this.geometry();
      const p = C.toScreen(this.uwb, cx, cy, R / this.settings.radius);
      const size = this.width < 420 ? 6 : 8;
      g.save(); g.beginPath(); g.arc(cx, cy, R, 0, C.TAU); g.clip();
      g.fillStyle = '#000'; g.strokeStyle = '#62e4ef'; g.lineWidth = 2;
      if (this.uwb.synthetic) g.setLineDash([3, 2]);
      g.beginPath(); g.moveTo(p.x, p.y - size); g.lineTo(p.x + size, p.y);
      g.lineTo(p.x, p.y + size); g.lineTo(p.x - size, p.y); g.closePath(); g.fill(); g.stroke();
      g.setLineDash([]); g.fillStyle = '#62e4ef'; g.beginPath(); g.arc(p.x, p.y, 1.5, 0, C.TAU); g.fill();
      const label = `${this.uwb.synthetic ? 'DEMO ' : ''}${this.uwb.label}`;
      const width = Math.min(150, label.length * 6.5 + 8);
      const x = Math.max(cx - R + 3, Math.min(cx + R - width - 3, p.x + size + 6));
      const y = Math.max(cy - R + 12, Math.min(cy + R - 12, p.y - size - 10));
      g.fillStyle = '#000'; g.fillRect(x - 3, y - 9, width, 18);
      this.text(g, label, x, y, this.width < 420 ? 9 : 11, 'left', '#62e4ef'); g.restore();
    }
    selectPoint(event) {
      if (!this.view.fresh || !this.view.projection) return;
      const rect = this.canvas.getBoundingClientRect(), { cx, cy, R } = this.geometry();
      const pointer = { x: event.clientX - rect.left, y: event.clientY - rect.top };
      this.selected = null; let nearest = event.pointerType === 'touch' ? 20 : 12;
      for (const point of this.view.projection.points) {
        const p = lidarToScreen(point, cx, cy, R / this.settings.radius);
        const distance = Math.hypot(p.x - pointer.x, p.y - pointer.y);
        if (distance < nearest) { nearest = distance; this.selected = point; }
      }
      this.drawScan();
    }
    animating() { return this.view.fresh && !this.view.paused && this.settings.sweepEnabled && !this.motion.matches && !document.hidden; }
    wake() { if (this.animating() && this.raf === null) this.raf = requestAnimationFrame(this.tick); }
    tick(now) {
      this.raf = null;
      if (!this.animating()) return;
      if (now - this.lastDraw >= 1000 / 30 - .5) { this.drawSweep(now); this.lastDraw = now; }
      this.wake();
    }
    drawSweep(now) {
      const g = this.sweepCtx, { cx, cy, R } = this.geometry(); this.prepare(g); g.clearRect(0, 0, this.width, this.height);
      if (!this.view.fresh || !this.settings.sweepEnabled || this.motion.matches) return;
      if (!this.view.paused) this.phase = (now / 1000 * this.settings.displayHz % 1) * C.TAU - Math.PI / 2;
      // White sweep: distance colors are reserved for measured returns and contour guides.
      for (let i = 0; i < 8; i++) {
        const a = this.phase - (8 - i) * .023;
        g.fillStyle = `rgba(255,255,255,${.004 + i * .0012})`;
        g.beginPath(); g.moveTo(cx, cy); g.arc(cx, cy, R, a, a + .024); g.closePath(); g.fill();
      }
      g.strokeStyle = 'rgba(255,255,255,.38)'; g.lineWidth = 1;
      g.beginPath(); g.moveTo(cx, cy); g.lineTo(cx + Math.cos(this.phase) * R, cy + Math.sin(this.phase) * R); g.stroke();
      this.frameCount++;
    }
    exportCanvas() {
      const out = document.createElement('canvas'); out.width = this.canvas.width; out.height = this.canvas.height;
      const g = out.getContext('2d', { alpha: false });
      for (const layer of [this.grid, this.sweep, this.canvas, this.uwbCanvas, this.trackingCanvas]) g.drawImage(layer, 0, 0);
      g.setTransform(this.dpr, 0, 0, this.dpr, 0, 0);
      this.text(g, this.view.label, 8, 12, 10, 'left', '#fff');
      this.text(g, this.view.state, this.width - 8, this.height - 12, 10, 'right', '#fff');
      return out;
    }
  }
  window.RadarRenderer = RadarRenderer;
})();
