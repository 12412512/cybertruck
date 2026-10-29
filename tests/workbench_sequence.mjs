import assert from 'node:assert/strict';
import {sequenceRows,sameRows,canDeleteSequence} from '../pi/robot/web/static/workbench-sequence.js';
let count=0;const check=x=>{assert.ok(x);count++;};
const rows=[{order:1,axis:'lift',target:20,speed:5},{order:2,axis:'reach',target:10,speed:5},
 {order:2,axis:'arm',target:359,speed:90},{order:3,axis:'lift',target:0,speed:5}];
const result=sequenceRows(rows);check(result[0].target_m===.02);check(Math.abs(result[2].target_rad-359*Math.PI/180)<1e-10);
check(result[2].speed_rad_s===Math.PI/2);check(result.map(r=>r.order).join(',')==='1,2,2,3');
for(const value of [[],[...rows,...Array(40).fill(rows[0])],[{...rows[0],order:0}],[{...rows[0],order:1.2}],
 [{...rows[0],target:-1}],[{...rows[2],target:360}],[{...rows[0],speed:3001}],
 [rows[0],{...rows[0],target:30}],[{...rows[0],target:''}]]){assert.throws(()=>sequenceRows(value));count++;}
assert.throws(()=>sequenceRows(rows,'together'));count++;
check(sequenceRows(rows.slice(0,3),'together').length===3);
check(sameRows(result,result.map(r=>({...r}))));
check(sameRows(result,result.map(r=>({...r,...(r.target_rad?{target_rad:r.target_rad+1e-14}:{})}))));
check(!sameRows(result,[...result.slice(0,3)]));
check(!sameRows(result,result.map((r,i)=>({...r,...(i===0?{target_m:.03}:{})}))));
check(sequenceRows([{order:1,axis:'gripper',target:60,speed:45}])[0].release_confirmed);
console.log(`${count} sequence table checks PASS; no network or device access`);
const state={connected:true,sequence_management:{schema:1},sequence:{status:'IDLE'},mechanism:{axis_state:6,servo_state:9,pose_state:0},group:{state:0}};
check(canDeleteSequence(state,{id:'test'},false,''));
for(const change of [{connected:false},{sequence_management:null},{owner:'other'},{sequence:{status:'RUNNING'}},{group:{state:1}},{mechanism:{servo_state:3}}])
 check(!canDeleteSequence({...state,...change},{id:'test'},false,''));
check(!canDeleteSequence(state,null,false,''));check(!canDeleteSequence(state,{id:'test'},false,'pending'));
console.log('9 version-delete UI guards PASS; no file or device changes');
