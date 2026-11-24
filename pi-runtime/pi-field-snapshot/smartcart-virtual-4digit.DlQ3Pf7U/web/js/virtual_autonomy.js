/* 합성 환경 전용: 실제 센서·서버·모터 명령과 연결하지 않는다. */
(function (root, factory) {
  const api = factory();
  if (typeof module === 'object' && module.exports) module.exports = api;
  else { root.VirtualAutonomy = api; api.install(root.document, root); }
})(typeof globalThis !== 'undefined' ? globalThis : this, function () {
  'use strict';
  const clamp = (x, a, b) => Math.max(a, Math.min(b, x));
  const wrap = a => Math.atan2(Math.sin(a), Math.cos(a));
  function initial() {
    return {x:0, y:0, yaw:0, tag:[4,0], obstacles:[[2,0],[2.8,1.7]],
      auto:false, follow:false, avoid:false, distance:1.2, speed:0.45, radius:0.65,
      status:'정지', linear:0, angular:0};
  }
  function step(s, dt) {
    if (!Number.isFinite(dt) || dt <= 0 || dt > 0.2) { s.linear=0; s.angular=0; s.auto=false; s.status='시간 오류 · 정지'; return; }
    s.linear=0; s.angular=0;
    if (!s.auto || !s.follow) { s.status='정지'; return; }
    const dx=s.tag[0]-s.x, dy=s.tag[1]-s.y, distance=Math.hypot(dx,dy);
    if (distance <= s.distance) { s.status='추종 거리 유지'; return; }
    let heading=Math.atan2(dy,dx), nearest=Infinity, threat=null;
    for (const p of s.obstacles) {
      const x=p[0]-s.x, y=p[1]-s.y, d=Math.hypot(x,y);
      nearest=Math.min(nearest,d);
      if (d < s.radius) { s.status='장애물 정지'; return; }
      const relative=Math.abs(wrap(Math.atan2(y,x)-heading));
      if (d < 1.8 && relative < 0.55 && (!threat || d < threat.d)) threat={x,y,d};
    }
    if (threat && !s.avoid) { s.status='장애물 정지 · 회피 OFF'; return; }
    if (threat) {
      const side=wrap(Math.atan2(threat.y,threat.x)-heading) >= 0 ? -1 : 1;
      heading=wrap(heading + side * 0.85);
    }
    const error=wrap(heading-s.yaw);
    s.angular=clamp(error*1.8,-1.2,1.2);
    s.linear=Math.abs(error)>0.65 ? 0 : Math.min(s.speed,Math.max(0,(distance-s.distance)*0.6));
    if (nearest < s.radius + 0.35) s.linear=Math.min(s.linear,0.12);
    const nx=s.x+Math.cos(s.yaw)*s.linear*dt, ny=s.y+Math.sin(s.yaw)*s.linear*dt;
    if (s.obstacles.some(p=>Math.hypot(p[0]-nx,p[1]-ny)<s.radius)) { s.linear=0; s.angular=0; s.status='장애물 정지'; return; }
    s.x=nx; s.y=ny; s.yaw=wrap(s.yaw+s.angular*dt); s.status=threat?'장애물 회피':'사용자 추종';
  }
  // 실측점은 기존 장착 변환만 사용한다. 무효·미관측 빔을 빈 공간으로 바꾸지 않는다.
  function liveInput(input, now) {
    const fail = reason => ({ok:false, reason});
    if (!input || !input.ready || !input.scan || !input.track || !input.uwb) return fail('센서·추적 입력 대기');
    if (!Number.isFinite(input.until) || now >= input.until) return fail('센서 데이터 만료');
    const q=input.scan, t=input.track, u=input.uwb, m=t.transform;
    if (!u.valid || u.profile !== 'single_left' || !u.sensors || !u.sensors.uwb_left ||
        !u.sensors.uwb_left.idValid || !u.sensors.uwb_left.measurementValid)
      return fail('BU04 측정 또는 태그 ID 무효');
    if (!t.selected || !t.uwb || t.state !== 'tracking' || t.candidateCount !== 1 || t.confirmationCount < 3 ||
        !m || ![m.yaw,m.offsetX,m.offsetY,t.selected.x,t.selected.y].every(Number.isFinite)) return fail('유효한 단일 추적 위치 없음');
    if (q.truncated || !Array.isArray(q.ranges) || q.ranges.length < 3 || q.ranges.length > 4096 ||
        ![q.angle_min,q.angle_max,q.angle_increment,q.range_min,q.range_max].every(Number.isFinite) ||
        q.angle_increment <= 0 || q.angle_increment > 0.1 || q.range_min < 0 || q.range_max <= q.range_min ||
        Math.abs((q.ranges.length-1)*q.angle_increment-(q.angle_max-q.angle_min)) > 1.5*q.angle_increment ||
        (q.ranges.length-1)*q.angle_increment < 2*Math.PI-1.5*q.angle_increment ||
        (q.ranges.length-1)*q.angle_increment > 2*Math.PI+q.angle_increment ||
        q.ranges.some(r=>!Number.isFinite(r) || r<q.range_min || r>q.range_max)) return fail('LiDAR 미관측 또는 스캔 무효');
    const yaw=m.yaw*Math.PI/180;
    const points=q.ranges.map((r,i)=>[m.offsetX+r*Math.cos(q.angle_min+i*q.angle_increment+yaw),m.offsetY+r*Math.sin(q.angle_min+i*q.angle_increment+yaw)]);
    return {ok:true, until:input.until, tag:[t.selected.x,t.selected.y], obstacles:points};
  }
  function install(document, win) {
    const get=id=>document.getElementById(id), canvas=get('simCanvas');
    if (!canvas) return;
    const s=initial(), ctx=canvas.getContext('2d');
    let live=null;
    const fields=[['simDistance','distance',0.5,3],['simSpeed','speed',0.1,0.8],['simRadius','radius',0.4,1.2]];
    const switches=[['simAuto','auto'],['simAvoid','avoid'],['simFollow','follow']];
    function render() {
      for (const [id,key] of switches) { get(id).setAttribute('aria-pressed',String(s[key])); get(id).querySelector('span').textContent=s[key]?'ON':'OFF'; }
      get('simStatus').textContent=s.status+' · 속도 '+s.linear.toFixed(2)+' m/s · 회전 '+s.angular.toFixed(2)+' rad/s';
      ctx.clearRect(0,0,canvas.width,canvas.height); ctx.fillStyle='#101a23'; ctx.fillRect(0,0,canvas.width,canvas.height);
      const pos=(x,y)=>[20+x*45,canvas.height/2-y*45];
      for (const p of s.obstacles) { const [x,y]=pos(...p);ctx.fillStyle='#f48372';ctx.beginPath();ctx.arc(x,y,8,0,Math.PI*2);ctx.fill(); }
      let [x,y]=pos(...s.tag);ctx.fillStyle='#70e6c8';ctx.beginPath();ctx.arc(x,y,7,0,Math.PI*2);ctx.fill();
      [x,y]=pos(s.x,s.y);ctx.save();ctx.translate(x,y);ctx.rotate(-s.yaw);ctx.fillStyle='#fff';ctx.beginPath();ctx.moveTo(12,0);ctx.lineTo(-8,-7);ctx.lineTo(-8,7);ctx.closePath();ctx.fill();ctx.restore();
    }
    for (const [id,key] of switches) get(id).addEventListener('click',()=>{s[key]=!s[key]; if (!s.auto || !s.follow) {s.linear=0;s.angular=0;s.status='정지';} render();});
    get('simSource').addEventListener('change',()=>{Object.assign(s,initial());live=null;for(const [id,key] of fields){get(id).value=s[key];get(id+'Value').textContent=s[key].toFixed(2);}render();});
    for (const [id,key,min,max] of fields) get(id).addEventListener('input',()=>{const v=Number(get(id).value);if (Number.isFinite(v) && v>=min && v<=max) s[key]=v; get(id+'Value').textContent=s[key].toFixed(2);render();});
    get('simReset').addEventListener('click',()=>{Object.assign(s,initial());for(const [id,key] of fields){get(id).value=s[key];get(id+'Value').textContent=s[key].toFixed(2);}render();});
    function stopSimulation() { s.auto=false; s.linear=0; s.angular=0; s.status='STOP · 가상 주행 정지'; render(); }
    for (const button of document.querySelectorAll('[data-virtual="stop"]')) {
      button.addEventListener('pointerdown',stopSimulation);
      button.addEventListener('click',stopSimulation);
      button.addEventListener('keydown',event=>{if(event.key===' ' || event.key==='Enter') stopSimulation();});
    }
    document.addEventListener('visibilitychange',()=>{if(document.hidden){s.auto=false;s.linear=0;s.angular=0;s.status='화면 숨김 · 정지';render();}});
    win.addEventListener('pagehide',()=>{s.auto=false;s.linear=0;s.angular=0;});
    win.SmartCartVirtualInput = input => {live=liveInput(input,win.performance.now());};
    let last=win.performance.now(); win.setInterval(()=>{const now=win.performance.now(),dt=(now-last)/1000;last=now;
      if(get('simSource').value==='live') {
        if (!live || !live.ok || now>=live.until) {s.linear=0;s.angular=0;s.auto=false;s.status=live && !live.ok ? live.reason : '실측 입력 만료 · 다시 시작';}
        else {s.x=0;s.y=0;s.yaw=0;s.tag=live.tag;s.obstacles=live.obstacles;step(s,dt);s.x=0;s.y=0;s.yaw=0;}
      } else step(s,dt);
      render();},50); render();
  }
  return {initial,step,liveInput,install};
});
