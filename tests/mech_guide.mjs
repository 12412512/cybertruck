import assert from 'node:assert/strict';
import {blockers,canAct,targetError,samePose,procedure,stateName} from '../pi/robot/web/static/workbench-mech-guide.js';
const healthy=()=>({connected:true,site_ready:true,base:{fault:0,stopped:true},mechanism:{fault:0,attached:1,base_stopped:1,config_mask:3,zero_valid:1,pwm_mask:7,axis_state:0,servo_state:0,pose_state:0}});
let checks=0;
const check=(value)=>{assert.ok(value);checks++;};
check(canAct(healthy(),true,'','move'));
for(const action of ['move','next','confirm','pose','zero','enable']){
  for(const fault of ['board','mechanism','stale','site','owner','pending']){
    const state=healthy();state.mechanism.pose_state=4;state.mechanism.servo_state=5;
    if(fault==='board')state.base.fault=2048;
    if(fault==='mechanism')state.mechanism.fault=2;
    if(fault==='stale')state.connected=false;
    if(fault==='site')state.site_ready=false;
    check(!canAct(state,fault!=='owner',fault==='pending'?'r':'',action));
  }
}
for(const axis of ['lift','reach','arm','tray','gripper']){
  check(procedure(axis).length===6);check(targetError(axis,Infinity)!=='');
  check(targetError(axis,-1)!=='');
}
check(targetError('lift',49)==='');check(targetError('lift',49.1)!=='');
check(targetError('reach',50)==='');check(targetError('reach',50.1)!=='');
check(targetError('tray',270)==='');check(targetError('arm',82.9)!=='');
const record={targets:{lift:0,reach:0,arm:Math.PI,tray:0,gripper:1}};
check(samePose({...record.targets},record));check(!samePose({...record.targets,lift:.002},record));
check(!samePose({...record.targets,tray:NaN},record));
check(stateName('servo',5)==='等待人工姿态确认');
let state=healthy();state.mechanism.pose_state=4;check(canAct(state,true,'','next'));
state.mechanism.pose_state=2;check(!canAct(state,true,'','next'));
state=healthy();state.mechanism.servo_state=5;check(canAct(state,true,'','confirm'));
state.mechanism.zero_valid=0;check(!canAct(state,true,'','confirm'));
state=healthy();state.mechanism.pwm_mask=3;check(!canAct(state,true,'','pose'));
state=healthy();state.mechanism.config_mask=1;check(!canAct(state,true,'','enable'));
check(blockers({connected:false},false,'r').length===4);
console.log(`${checks} guide checks PASS; no network or device calls`);
