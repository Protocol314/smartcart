/* Existing buttons, one authenticated WebSocket, explicit arming and STOP. */
(()=>{'use strict';
let active=false,follow=false,last=null;
const $=id=>document.getElementById(id);
const Native=window.WebSocket;
function paint(){
 for(const [id,value] of [['simAuto',active],['simFollow',follow],['simAvoid',last?.lidar_guard?.enabled===true]]){
  const b=$(id);if(b){b.setAttribute('aria-pressed',String(value));const span=b.querySelector('span');if(span)span.textContent=value?'ON':'OFF';}
 }
}
function off(){active=false;paint();}
function stop(){off();window.SmartCartManual?.stop();}
function render(state){
 last=state;const a=state.autonomy;if(!a)return;
 if(state.fault || (!state.armed && !window.SmartCartManual?.sessionEnabled))off();
 paint();
 const f=x=>Number.isFinite(x)?x.toFixed(2):'—';
 const names={off:'정지',ready:'UWB 준비',confirming_uwb:'새 UWB 표본 확인 중',distance_hold:'추종거리 유지',following:'UWB 추종',turning_in_place:'제자리 방향 조정',uwb_stale:'UWB 표본 만료',uwb_stale_after_outlier:'정상 UWB 입력 대기',uwb_outlier_ignored:'튀는 UWB 표본 제외',uwb_measurement_or_id_invalid:'UWB 측정·ID 무효',target_outside_forward_trial:'태그가 전방 시험 범위 밖',rearm_required:'주행 재활성화 필요',waiting_for_lidar:'LiDAR 입력 대기',lidar_stale:'LiDAR 표본 만료',lidar_obstacle_stop:'LiDAR 장애물 정지',lidar_unknown_motion_sector:'주행 방향 LiDAR 빔 무효',lidar_confirming_clear:'LiDAR 빈 공간 확인 중',lidar_slowdown:'LiDAR 근접 감속',lidar_clear:'LiDAR 감시 정상'};
 const mode=a.paused?'정지 대기 — '+(names[a.reason]||a.reason):(names[a.reason]||a.reason);
 const laser=state.lidar_guard;
 const laserText=laser?.enabled?' / LiDAR '+(names[laser.reason]||laser.reason)+' / 최근접 '+f(laser.nearest_m)+' m (센서 기준)':' / LiDAR 미연결';
 const text=(state.hardware?'GPIO 실제 출력':'MOCK — 모터 출력 없음')+' / '+mode+' / BU04 거리 '+f(a.range_m)+' m / 표시 각도 '+f(a.heading_deg)+'° / 자동 회전 부호 '+(a.auto_turn_sign??'?')+' / PWM '+state.pwm.join(' / ')+laserText;
 for(const id of ['simStatus','simHeaderStatus'])if($(id))$(id).textContent=text+' / 이상치 제외 '+(a.outlier_count||0);
 if($('simDistanceValue'))$('simDistanceValue').textContent=f(a.stop_distance_m);
 if($('simCanvas')){const c=$('simCanvas').getContext('2d'),w=$('simCanvas').width,h=$('simCanvas').height;
  c.fillStyle='#101a23';c.fillRect(0,0,w,h);c.fillStyle='white';c.fillRect(w/2-5,h-20,10,10);
  if(Number.isFinite(a.range_m)&&Number.isFinite(a.heading_deg)){const t=a.heading_deg*Math.PI/180,x=w/2-a.range_m*Math.sin(t)*30,y=h-15-a.range_m*Math.cos(t)*30;c.strokeStyle='#70e6c8';c.beginPath();c.moveTo(w/2,h-15);c.lineTo(x,y);c.stroke();c.fillStyle='#70e6c8';c.beginPath();c.arc(x,y,5,0,Math.PI*2);c.fill();}
  c.fillStyle='white';c.fillText('UWB follow + LiDAR stop monitor',6,15);
 }
}
window.WebSocket=class extends Native{
 send(raw){let d;try{d=JSON.parse(raw);}catch{}
  if(d?.type==='stop')off();
  if(d?.type==='drive'&&active){
   if(window.SmartCartManual?.pressed.length)off();
   else {d.type='auto';raw=JSON.stringify(d);}
  }
  return super.send(raw);
 }
 constructor(...args){super(...args);this.addEventListener('message',e=>{try{const d=JSON.parse(e.data);if(d.autonomy)render(d);}catch{}});this.addEventListener('close',off);}
};
document.addEventListener('DOMContentLoaded',()=>{
 $('simSource').value='live';$('simSource').disabled=true;
 $('simSource').selectedOptions[0].textContent='실측 BU04 + LiDAR';
 $('source').value='live';
 $('simAvoid').disabled=true;$('simAvoid').firstChild.textContent='LiDAR 경로 회피 ';
 for(const id of ['simDistance','simSpeed','simRadius'])$(id).disabled=true;
 $('simDistance').value=0.8;$('simDistanceValue').textContent='0.80';
 $('simSpeedValue').textContent='미보정';$('simRadiusValue').textContent='미사용';
 $('autonomyHelp').textContent='실차 v1.3 · BU04 추종·이상치 제외 · 1.00m 출발 / 0.80m 정지 · 자동 회전 방향 보정 · LiDAR 근접 감속·정지 감시(자동 모드) · 장애물 우회 경로·엔코더 제어 없음';
 $('simAuto').onclick=()=>{
  if(active){stop();return;}
  if(!follow){$('simStatus').textContent='사용자 추종을 먼저 켜세요';return;}
  if(!window.SmartCartManual?.enabled){$('simStatus').textContent='조이스틱 연결 및 주행 활성화가 필요합니다';return;}
  active=true;paint();
 };
 $('simFollow').onclick=()=>{follow=!follow;if(!follow)stop();paint();};
 $('simReset').onclick=()=>{follow=false;stop();paint();};
 window.addEventListener('blur',off);window.addEventListener('pagehide',off);
 document.addEventListener('visibilitychange',()=>{if(document.hidden)off();});
 paint();if(last)render(last);
});
})();
