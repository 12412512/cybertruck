import assert from 'node:assert/strict';
import fs from 'node:fs';
import {actuators,freshDevice,blockedReasons,allowed,moveArgs} from '../pi/robot/web/static/workbench-remote-state.js';

const healthy=()=>({domain:'physical',connected:true,state_age_ms:15,site_ready:true,
    base:{fault:0,stopped:true},mechanism:{attached:1,fault:0,base_stopped:1,config_mask:3,
        zero_valid:1,pwm_mask:7,axis_state:0,servo_state:0,pose_state:0}});
let checks=0;
const check=value=>{assert.ok(value);checks++;};
check(actuators.length===5);
check(actuators.map(a=>a.pin).join(',')==='CAN · ID5,CAN · ID6,PE9,PE11,PE13');
check(freshDevice(healthy(),100,200));
check(!freshDevice(healthy(),100,335));
check(!freshDevice({...healthy(),domain:'simulation'},100,100));
check(!freshDevice({...healthy(),state_age_ms:null},100,100));
check(!freshDevice(healthy(),100,99));
for(const action of ['move','enable','zero','anchor','pose']) {
    check(allowed(healthy(),true,'',action));
    for(const fault of ['base','mech','stale','site','owner','pending','zero','config','parked']) {
        const state=healthy();
        if(fault==='base')state.base.fault=2048;
        if(fault==='mech')state.mechanism.fault=204;
        if(fault==='stale')state.connected=false;
        if(fault==='site')state.site_ready=false;
        if(fault==='zero')state.mechanism.zero_valid=0;
        if(fault==='config')state.mechanism.config_mask=1;
        if(fault==='parked')state.base.stopped=false;
        if(fault==='zero'&&['enable','zero'].includes(action))continue;
        check(!allowed(state,fault!=='owner',fault==='pending'?'waiting':'',action));
    }
}
let state=healthy();state.mechanism.servo_state=5;
check(allowed(state,true,'','confirm'));check(!allowed(state,true,'','move'));
state.mechanism.pose_state=4;check(allowed(state,true,'','next'));
check(allowed({},false,'unknown','stop'));
state=healthy();state.mechanism.attached=0;
check(allowed(state,true,'','attach'));
state.base.fault=2048;check(!allowed(state,true,'','attach'));
check(allowed({...state,site_ready:false},true,'','teach'));
check(blockedReasons({connected:false,reason:'连接断开'},false,'').includes('连接断开'));
check(!blockedReasons({connected:false},false,'').some(x=>x.includes('保持只读')));
assert.deepEqual(moveArgs('lift','2',5),{axis:'lift',target_m:.002,rpm:5,release_confirmed:false});checks++;
check(Math.abs(moveArgs('tray','30').target_rad-Math.PI/6)<1e-10);
check(!('rpm' in moveArgs('tray','0')));
check(moveArgs('gripper','60',5,true).release_confirmed);
for(const [axis,value,rpm] of [['lift','',5],['lift',' ',5],['reach','51',5],['arm','82',5],['tray','271',5],['gripper','99',5],['lift','2',31],['lift','2',1.5]]) {
    assert.throws(()=>moveArgs(axis,value,rpm));checks++;
}
const entry=fs.readFileSync(new URL('../pi/robot/web/static/workbench-mechanism.js',import.meta.url),'utf8');
check(!entry.includes('initTransfer'));check(!entry.includes('initCompact'));check(!entry.includes('initProgram'));
check(!entry.includes("command('mechanism'"));
const ui=fs.readFileSync(new URL('../pi/robot/web/static/workbench-device.js',import.meta.url),'utf8');
check(ui.includes("fetch('/api/device/state'"));check(ui.includes("fetch('/api/device/command'"));
check(!ui.includes('/api/workbench'));check(!ui.includes('localStorage'));
check(!ui.includes('initGuide'));check(!ui.includes('模拟零点'));
check(ui.includes('if(!token)return;'));
check(ui.includes("document.addEventListener('visibilitychange'"));
console.log(`${checks} remote interface checks PASS; no network or device calls`);
