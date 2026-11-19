'use strict';
const assert = require('node:assert/strict');
const C = require('../web/js/core.js');
let count = 0;
function test(name, fn) { fn(); count++; console.log('PASS', name); }
function scan(ranges, step = .01, start = 0) {
  return C.normaliseScan({angle_min:start,angle_max:start+(ranges.length-1)*step,angle_increment:step,
    range_min:.05,range_max:16,ranges});
}
const project = (ranges, s = {}, step = .01, start = 0) => C.projectScan(scan(ranges,step,start),s);
const wall = Array.from({length:201},(_,i)=>2/Math.cos(-.5+i*.005));
const s = C.validateSettings({});
test('plane wall retains all 201 measured dots and 200 guides',()=>{
  const p=project(wall,{},.005,-.5); assert.equal(p.points.length,201);assert.equal(p.links.length,200);assert.equal(p.runs.length,1);
  assert.ok(p.points.every(q=>Math.abs(q.x-2)<1e-12));
});
test('null breaks both neighbor connections',()=>{
  const a=wall.slice();a[100]=null;const p=project(a,{},.005,-.5);assert.equal(p.points.length,200);assert.equal(p.links.length,198);assert.equal(p.runs.length,2);
});
test('NaN never bridges across missing index',()=>{
  const p=project([1,NaN,1]);assert.equal(p.points.length,2);assert.equal(p.links.length,0);
});
test('Infinity never bridges across missing index',()=>assert.equal(project([1,Infinity,1]).links.length,0));
test('below sensor minimum breaks contour',()=>assert.equal(project([1,.04,1]).links.length,0));
test('above sensor maximum breaks contour',()=>assert.equal(project([1,17,1]).links.length,0));
test('cart-centered crop breaks contour, not just hides dot',()=>assert.equal(project([1,6,1]).links.length,0));
test('depth discontinuity splits foreground from background',()=>{
  const p=project([1,1.01,3,3.01]);assert.equal(p.links.length,2);assert.equal(p.runs.length,2);
});
test('bounded gap will not bridge separate nearby objects',()=>assert.equal(project([1,1.12],{maxGap:.1}).links.length,0));
test('adaptive bound is stricter for near objects',()=>assert.equal(project([.1,.19]).links.length,0));
test('large angular gap is not a wall',()=>assert.equal(project([1,1],{},.1).links.length,0));
test('negative angle increment retains scan ordering',()=>assert.equal(project([1,1.001,1.002],{},-.01,.02).links.length,2));
test('isolated valid return is not enlarged into an object',()=>{
  const p=project([1]);assert.equal(p.points.length,1);assert.equal(p.links.length,0);assert.equal(p.runs[0].length,1);
});
test('all invalid rays produce neither dots nor guides',()=>{
  const p=project([null,NaN,Infinity,.01,18]);assert.equal(p.points.length,0);assert.equal(p.links.length,0);assert.equal(p.runs.length,0);
});
test('360 seam is never forcibly closed',()=>{
  const p=project(Array(721).fill(2),{},C.TAU/720,-Math.PI);assert.equal(p.links.length,720);
  assert.ok(p.links.every(([a,b])=>b.index===a.index+1));
});
test('all guide endpoints are actual unchanged measured positions',()=>{
  const p=project(wall,{},.005,-.5);const positions=new Set(p.points);
  assert.ok(p.links.every(([a,b])=>positions.has(a)&&positions.has(b)));
});
test('rigid transform does not change contour membership',()=>{
  const a=project(wall,{},.005,-.5),b=project(wall,{yaw:73,offsetX:.3,offsetY:.1},.005,-.5);
  assert.equal(a.links.length,b.links.length);assert.equal(b.points.length,a.points.length);
});
test('zoom does not change physical link gate',()=>{
  assert.equal(project([1,1.05],{radius:2}).links.length,project([1,1.05],{radius:5}).links.length);
});
test('sample truncated positions are never padded',()=>{
  const p=project([1,1,1]);assert.equal(p.points.length,3);assert.equal(p.links.length,2);
});
test('near and far defaults are 1 and 3 meters',()=>assert.deepEqual([s.nearM,s.farM],[1,3]));
test('color boundaries are exact half-open intervals',()=>{
  assert.deepEqual([C.colorBand(.99,s),C.colorBand(1,s),C.colorBand(2.99,s),C.colorBand(3,s)],[0,1,1,2]);
});
test('color measures distance from cart, not laser origin',()=>{
  const p=project([.9],{offsetX:.3});assert.equal(C.colorBand(p.points[0].distance,s),1);
});
test('guide color splits at exact range boundaries',()=>{
  const edges=C.coloredEdge({x:.8,y:0},{x:3.2,y:0},s);assert.equal(edges.length,3);
  assert.deepEqual(edges.map(e=>e.band),[0,1,2]);assert.ok(Math.abs(edges[0].b.x-1)<1e-12);assert.ok(Math.abs(edges[1].b.x-3)<1e-12);
});
test('equal endpoint ranges may cross interior threshold',()=>{
  const edges=C.coloredEdge({x:1,y:.1},{x:1,y:-.1},{nearM:1.003,farM:3});
  assert.deepEqual(edges.map(e=>e.band),[1,0,1]);
});
test('zero-length duplicate return is safely colorable',()=>assert.equal(C.coloredEdge({x:1,y:0},{x:1,y:0},s).length,1));
test('invalid threshold order restores defaults',()=>assert.equal(C.validateSettings({nearM:4,farM:1}).farM,3));
test('nonfinite maxGap restores .2m',()=>assert.equal(C.validateSettings({maxGap:Infinity}).maxGap,.2));
test('trail setting removed, cannot retain motion ghosts',()=>assert.equal('trail' in C.validateSettings({trail:1}),false));
test('settings object only exports allowlisted fields',()=>assert.equal('danger' in C.validateSettings({danger:true}),false));
test('single ray mismatch with negative span is rejected',()=>assert.throws(()=>C.normaliseScan({angle_min:0,angle_max:-.00001,angle_increment:.1,range_min:.05,range_max:16,ranges:[1]})));
test('demo contains all three actual geometric distance bands',()=>{
  const p=C.projectScan(C.syntheticScan(0),s);assert.equal(new Set(p.points.map(p=>C.colorBand(p.distance,s))).size,3);
});
test('guide toggle does not change sensor measurement counts',()=>{
  const a=project([1,1,1]),b=project([1,1,1],{contoursEnabled:false});assert.deepEqual(a.points,b.points);
});
console.log(`Contour tests: ${count} passed`);
