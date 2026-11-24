'use strict';
const assert = require('node:assert/strict');
const C = require('../web/js/manual_controls.js');
function target(){
 const handlers={};
 return {
  value:'',textContent:'',dataset:{},disabled:false,tagName:'DIV',
  classList:{toggle(){}},setAttribute(){},setPointerCapture(){},
  addEventListener(name,fn){(handlers[name] ||= []).push(fn);},
  fire(name,e={}){for(const f of handlers[name]||[]) f(e);}
 };
}
let now=0, interval=null;
const ids=Object.fromEntries([
 'driveConnect','driveKey','driveArm','driveBackend','driveEstimatedSpeed',
 'virtualControlStatus','virtualControlOutput','virtualRpmOutput','virtualRpmLimit',
 'source','fileInput','openSettings','pauseView','openFile'
].map(id=>[id,target()]));
ids.driveKey.value='test-key'; ids.virtualRpmLimit.value='50'; ids.source.value='live';
const buttons=['forward','reverse','left','right','stop'].map(name=>{
 const b=target(); b.dataset.virtual=name; return b;
});
const doc=target(),win=target();
doc.getElementById=id=>ids[id]; doc.querySelectorAll=()=>buttons; doc.hidden=false;
win.performance={now:()=>now}; win.location={protocol:'http:',host:'192.168.0.3:8080'};
const sockets=[];
win.WebSocket=class {
 constructor(url){this.url=url;this.readyState=0;this.bufferedAmount=0;this.sent=[];sockets.push(this);}
 send(data){this.sent.push(JSON.parse(data));}
 close(){this.readyState=3;if(this.onclose)this.onclose();}
 open(){this.readyState=1;this.onopen();}
 receive(data){this.onmessage({data:JSON.stringify(data)});}
};
win.setInterval=fn=>{interval=fn;return 1;};
C.install(doc,win);
assert(buttons[0].disabled);
ids.driveConnect.fire('click');
let ws=sockets[0];ws.open();
assert.deepEqual(ws.sent[0],{key:'test-key'});
assert(ws.url.endsWith('/ws/drive'));
let status={schema:'smartcart.manual.v1',hardware:true,armed:false,reason:'connected',
 epoch:1,ack_seq:-1,stamp:100,pwm:[0,0],estimated_rpm:[0,0],estimated_kmh:0,fault:null};
function receive(changes={}){status={...status,...changes};ws.receive(status);}
receive();
ids.driveArm.fire('click');
let cmd=ws.sent.at(-1);
assert.equal(cmd.type,'arm');assert.equal(cmd.forward,0);assert.equal(cmd.turn,0);
assert(ids.driveArm.disabled);
receive({armed:true,reason:'armed',ack_seq:cmd.seq});
assert(win.SmartCartManual.enabled);
assert(!buttons[0].disabled);
const pointer=id=>({pointerId:id,preventDefault(){}});
buttons[0].fire('pointerdown',pointer(1));
assert.equal(ws.sent.at(-1).forward,1);
buttons[2].fire('pointerdown',pointer(2));
assert.equal(ws.sent.at(-1).forward,1);assert.equal(ws.sent.at(-1).turn,1);
buttons[0].fire('pointerup',pointer(1));
assert.equal(ws.sent.at(-1).forward,0);assert.equal(ws.sent.at(-1).turn,1);
buttons[2].fire('pointerup',pointer(2));
assert.equal(ws.sent.at(-1).turn,0);
assert.equal(ws.sent.at(-1).type,'drive','버튼 해제는 중립 명령이어야 한다');
assert(win.SmartCartManual.enabled,'정상 해제 후 활성화가 유지되어야 한다');
buttons[2].fire('lostpointercapture',pointer(2));
assert.equal(ws.sent.at(-1).type,'drive','정상 해제 뒤 포인터 캡처 해제는 정지가 아니다');
assert(win.SmartCartManual.enabled,'정상 포인터 해제 후에도 다시 활성화할 필요가 없어야 한다');
buttons[0].fire('pointerdown',pointer(5));
assert.equal(ws.sent.at(-1).forward,1);
buttons[0].fire('pointerup',pointer(5));
assert(win.SmartCartManual.enabled,'다시 누르고 놓아도 활성화를 유지해야 한다');
receive({pwm:[60,80],estimated_rpm:[37.6,50.2],estimated_kmh:1.26});
assert.equal(ids.virtualControlOutput.textContent,'60 / 80');
assert.equal(ids.driveEstimatedSpeed.textContent,'1.26');
buttons[4].fire('pointerdown',pointer(3));
assert.equal(ws.sent.at(-1).type,'stop');
const countAfterStop = ws.sent.length;
buttons[4].fire('click');
assert.equal(ws.sent.length,countAfterStop,'터치에 뒤따르는 클릭은 정지를 중복 전송하지 않는다');
assert(!win.SmartCartManual.enabled);
receive({armed:true}); // old telemetry must not re-enable movement after STOP
assert(!win.SmartCartManual.enabled);
receive({armed:false,epoch:2,reason:'software_stop'});
ids.driveArm.fire('click');
cmd=ws.sent.at(-1);receive({armed:true,ack_seq:cmd.seq});
buttons[0].fire('pointerdown',pointer(4));
win.fire('blur');
assert.equal(ws.sent.at(-1).type,'stop');
assert.deepEqual(win.SmartCartManual.pressed,[]);
receive({armed:false,epoch:3});
ids.driveArm.fire('click');cmd=ws.sent.at(-1);receive({armed:true,ack_seq:cmd.seq});
ids.source.value='sample';ids.source.fire('change');
assert(!win.SmartCartManual.enabled);
assert.equal(ws.sent.at(-1).type,'stop');
ids.source.value='live';receive({armed:false,epoch:4});
ids.driveArm.fire('click');cmd=ws.sent.at(-1);receive({armed:true,ack_seq:cmd.seq});
now=400;interval();
assert.equal(ws.readyState,1);
assert.equal(ws.sent.at(-1).type,'stop');
const countAfterTimeout=ws.sent.length;
interval();
assert.equal(ws.sent.length,countAfterTimeout);
assert(!win.SmartCartManual.enabled);
assert.equal(ids.virtualControlOutput.textContent,'— / —');
receive({armed:false,epoch:5,reason:'command_timeout'});
assert(!win.SmartCartManual.enabled,'통신 복구만으로 자동 주행하면 안 된다');
ids.driveArm.fire('click');
assert.equal(ws.sent.at(-1).type,'arm');
cmd=ws.sent.at(-1);receive({armed:true,ack_seq:cmd.seq,reason:'armed',pwm:[0,0]});
buttons[0].fire('pointerdown',pointer(10));
const firstDrive=ws.sent.at(-1);
now=500;receive({armed:true,applied_seq:firstDrive.seq,pwm:[2,2]});
for (now=600;now<=2600;now+=100) { receive({armed:true,pwm:[2,2]}); interval(); }
assert(win.SmartCartManual.enabled,'정상 PWM 응답 뒤에는 2초 경과로 멈추지 않는다');
buttons[0].fire('pointerup',pointer(10));
buttons[1].fire('pointerdown',pointer(11));
const reverseStart=now;
for (now=reverseStart+100;now<reverseStart+2000;now+=100) { receive({armed:true,pwm:[0,0]}); interval(); }
now=reverseStart+2000; receive({armed:true,pwm:[0,0]});
assert.equal(ws.sent.at(-1).type,'stop');
assert.match(ids.virtualControlStatus.textContent,/2초 초과/);
assert(!win.SmartCartManual.enabled,'출력 지연 정지 후 자동 재개하지 않는다');
receive({armed:false,epoch:6,reason:'software_stop'});
ids.driveArm.fire('click');cmd=ws.sent.at(-1);receive({armed:true,ack_seq:cmd.seq,reason:'armed',pwm:[0,0]});
ws.bufferedAmount=5000;
buttons[0].fire('pointerdown',pointer(15));
assert.equal(ws.sent.at(-1).type,'stop');
assert.equal(ws.readyState,3);
assert.match(ids.virtualControlStatus.textContent,/전송 정체/);
assert(!win.SmartCartManual.enabled,'전송 정체 시 조종이 잠겨야 한다');
assert.deepEqual(C.axes(['forward','right']),{forward:1,turn:-1});
console.log('PASS: button -> network command, dual touch, release, STOP latch, blur, source switch, timeout, telemetry');

