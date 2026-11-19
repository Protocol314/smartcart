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
receive({pwm:[60,80],estimated_rpm:[37.6,50.2],estimated_kmh:1.26});
assert.equal(ids.virtualControlOutput.textContent,'60 / 80');
assert.equal(ids.driveEstimatedSpeed.textContent,'1.26');
buttons[4].fire('pointerdown',pointer(3));
assert.equal(ws.sent.at(-1).type,'stop');
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
assert.equal(ws.readyState,3);
assert(!win.SmartCartManual.enabled);
assert.equal(ids.virtualControlOutput.textContent,'— / —');
assert.deepEqual(C.axes(['forward','right']),{forward:1,turn:-1});
console.log('PASS: button -> network command, dual touch, release, STOP latch, blur, source switch, timeout, telemetry');
