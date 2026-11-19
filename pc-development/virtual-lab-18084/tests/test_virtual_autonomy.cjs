'use strict';
const assert = require('assert');
const {initial,step} = require('../web/js/virtual_autonomy.js');
let s=initial();
for(let i=0;i<100;i++) step(s,.05);
assert.equal(s.x,0);
s.auto=true; s.follow=true; s.avoid=false;
for(let i=0;i<300;i++) step(s,.05);
assert.match(s.status,/장애물 정지/);
assert(s.x<2-s.radius);
s=initial(); s.auto=s.follow=s.avoid=true;
let closest=Infinity, avoided=false;
for(let i=0;i<2000;i++) { step(s,.05); avoided ||= s.status==='장애물 회피'; for(const p of s.obstacles) closest=Math.min(closest,Math.hypot(p[0]-s.x,p[1]-s.y)); }
assert(avoided); assert(closest>=s.radius); assert(Math.hypot(s.tag[0]-s.x,s.tag[1]-s.y)<s.distance+.02);
s.auto=false; const x=s.x; step(s,.05); assert.equal(s.x,x); assert.equal(s.linear,0);
s.auto=true; s.follow=false; step(s,.05); assert.equal(s.linear,0);
s.follow=true; step(s,1); assert.equal(s.auto,false); assert.equal(s.linear,0);
console.log('PASS: 가상 추종, 회피, 정지 반경, 토글과 시간 오류');
