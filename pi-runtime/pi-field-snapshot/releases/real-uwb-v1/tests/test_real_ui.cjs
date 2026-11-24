'use strict';
const assert=require('node:assert/strict');
const fs=require('node:fs');
const vm=require('node:vm');
const path=require('node:path');
const controls=require('../web/js/manual_controls.js');
function target(){const listeners={};return {
 value:'',textContent:'',dataset:{},disabled:false,tagName:'DIV',attrs:{},width:260,height:180,
 firstChild:{textContent:''},selectedOptions:[{textContent:''}],classList:{toggle(){}},setPointerCapture(){},
 setAttribute(k,v){this.attrs[k]=v;},querySelector(){return this.span||(this.span={textContent:''});},
 getContext(){return new Proxy({}, {get:(o,k)=>o[k]||(()=>{})});},
 addEventListener(k,f){(listeners[k]||=[]).push(f);},
 fire(k,e={}){for(const f of listeners[k]||[])f(e);this['on'+k]?.(e);}
};}
const ids={};const el=id=>ids[id]||(ids[id]=target());
const doc=target(),win=target();doc.hidden=false;doc.getElementById=el;
const buttons=['forward','reverse','left','right','stop'].map(name=>{const b=target();b.dataset.virtual=name;return b;});
doc.querySelectorAll=()=>buttons;
let now=0,tick;win.performance={now:()=>now};win.location={protocol:'https:',host:'172.20.10.4:8081'};
win.setInterval=f=>{tick=f;};
const sockets=[];
win.WebSocket=class{
 constructor(){Object.assign(this,target());this.readyState=1;this.bufferedAmount=0;this.sent=[];sockets.push(this);}
 send(raw){this.sent.push(JSON.parse(raw));}close(){this.readyState=3;this.fire('close');}
 receive(data){this.fire('message',{data:JSON.stringify(data)});}
};
el('virtualRpmLimit').value='50';el('driveKey').value='4321';
vm.runInNewContext(fs.readFileSync(path.join(__dirname,'../web/js/uwb_real.js'),'utf8'),{window:win,document:doc,console});
controls.install(doc,win);doc.fire('DOMContentLoaded');
assert.equal(el('source').value,'live');assert(el('simAvoid').disabled);
el('simFollow').onclick();el('simAuto').onclick();
assert.equal(el('simAuto').attrs['aria-pressed'],'false','Cannot auto-enable before authenticated arming');
el('driveConnect').fire('click');const socket=sockets[0];socket.fire('open');
assert.deepEqual(socket.sent.at(-1),{key:'4321'});
let status={schema:'smartcart.manual.v1',hardware:true,armed:false,reason:'connected',fault:null,
 epoch:1,ack_seq:-1,stamp:100,pwm:[0,0],estimated_rpm:[0,0],estimated_kmh:0,
 autonomy:{enabled:false,reason:'off',range_m:3,heading_deg:0,stop_distance_m:2}};
socket.receive(status);el('driveArm').fire('click');let command=socket.sent.at(-1);
assert.equal(command.type,'arm');assert.equal(command.forward,0);
status={...status,armed:true,ack_seq:command.seq};socket.receive(status);
assert(win.SmartCartManual.enabled);
el('simAuto').onclick();assert.equal(el('simAuto').attrs['aria-pressed'],'true');tick();
assert.equal(socket.sent.at(-1).type,'auto');assert.equal(socket.sent.at(-1).epoch,1);
buttons[0].fire('pointerdown',{pointerId:1,preventDefault(){}});
assert.equal(socket.sent.at(-1).type,'drive','Manual press cancels auto');
assert.equal(el('simAuto').attrs['aria-pressed'],'false');
buttons[0].fire('pointerup',{pointerId:1});
el('simAuto').onclick();tick();assert.equal(socket.sent.at(-1).type,'auto');
buttons[4].fire('pointerdown',{pointerId:2,preventDefault(){}});
assert.equal(socket.sent.at(-1).type,'stop');assert.equal(el('simAuto').attrs['aria-pressed'],'false');
status={...status,armed:false,epoch:2};socket.receive(status);
el('driveArm').fire('click');command=socket.sent.at(-1);
status={...status,armed:true,ack_seq:command.seq};socket.receive(status);
el('simAuto').onclick();win.fire('blur');
assert.equal(socket.sent.at(-1).type,'stop');assert.equal(el('simAuto').attrs['aria-pressed'],'false');
console.log('PASS: real UI arming, UWB auto routing, manual override, STOP and focus-loss stop');
