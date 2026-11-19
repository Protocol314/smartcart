'use strict';
const assert=require('assert');
const {liveInput,initial,step}=require('../web/js/virtual_autonomy.js');
const n=360, inc=2*Math.PI/n;
function input(){return {ready:true,until:1200,scan:{angle_min:-Math.PI,angle_max:Math.PI-inc,angle_increment:inc,range_min:.05,range_max:10,ranges:Array(n).fill(4),truncated:false},track:{state:'tracking',candidateCount:1,confirmationCount:3,selected:{x:3,y:0},uwb:{x:3,y:0},transform:{yaw:0,offsetX:0,offsetY:0}},uwb:{profile:'single_left',valid:true,sensors:{uwb_left:{idValid:true,measurementValid:true}}}};}
let good=input(), v=liveInput(good,1000);assert(v.ok);assert.equal(v.obstacles.length,360);
let s=initial();s.auto=s.follow=s.avoid=true;s.tag=v.tag;s.obstacles=v.obstacles;step(s,.05);assert(s.linear>0);
assert.equal(liveInput(good,1200).ok,false);
for(const mutate of [a=>a.ready=false,a=>a.uwb.valid=false,a=>a.uwb.sensors.uwb_left.idValid=false,a=>a.track.selected=null,a=>a.track.candidateCount=2,a=>a.track.uwb=null,a=>a.scan.ranges[5]=null,a=>a.scan.ranges.pop(),a=>a.scan.truncated=true]){const a=input();mutate(a);assert.equal(liveInput(a,1000).ok,false);}
const rotated=input();rotated.track.transform.yaw=90;assert(Math.abs(liveInput(rotated,1000).obstacles[180][1]-4)<1e-8);
console.log('PASS: 실측 입력 검증, 만료, ID·추적·스캔 오류 및 장착 변환');
