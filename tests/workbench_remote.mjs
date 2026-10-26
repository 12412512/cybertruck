import assert from 'node:assert/strict';
import fs from 'node:fs';
import {actuators,freshDevice,blockedReasons,allowed,moveArgs,manualTuning,armMaximum} from '../pi/robot/web/static/workbench-remote-state.js';

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
state=healthy();state.action_locked=true;
for(const action of ['claim','attach','move','enable','zero','anchor','pose','next','confirm','teach'])check(!allowed(state,true,'',action));
check(allowed(state,false,'','stop'));
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

state=healthy();Object.assign(state,{firmware:'GC-BASE-0.3.26',action_locked:true,latched:false,stop_acknowledged:true,site_ready:true});
Object.assign(state.mechanism,{attached:1,fault:6,config_mask:3,base_stopped:1,zero_valid:0,pwm_mask:0});
check(allowed(state,false,'','prepare'));check(!allowed(state,true,'','prepare'));check(!allowed(state,false,'pending','prepare'));
for(const mutate of [s=>s.base.fault=2048,s=>s.mechanism.pwm_mask=1,s=>s.firmware='GC-BASE-0.3.25',s=>s.connected=false,s=>s.stop_acknowledged=false]){
 const copy=structuredClone(state);mutate(copy);check(!allowed(copy,false,'','prepare'));
}
console.log('8 normal-stop prepare guards PASS');

state=healthy();state.firmware='GC-BASE-0.3.27';
check(allowed(state,true,'','mode'));check(!allowed(state,false,'','mode'));
state.manual_debug=true;state.mechanism.servo_state=9;
check(allowed(state,true,'','move'));check(!allowed(state,true,'','pose'));check(!allowed(state,true,'','next'));
check(allowed(state,true,'','confirm'));
for(const mutate of [s=>s.base.fault=2048,s=>s.site_ready=false,s=>s.connected=false,s=>s.mechanism.zero_valid=0,s=>s.firmware='GC-BASE-0.3.26']) {
 const copy=structuredClone(state);mutate(copy);check(!allowed(copy,true,'','mode'));
}
console.log('11 manual-mode UI guards PASS');

for(const version of ['GC-BASE-0.3.27','GC-BASE-0.3.28','GC-BASE-0.3.29','GC-BASE-0.3.30','GC-BASE-0.3.31']) {
 const live=healthy();live.firmware=version;
 check(allowed(live,true,'','mode'));
 for(const mutate of [s=>s.base.fault=2048,s=>s.site_ready=false,s=>s.connected=false,s=>s.mechanism.zero_valid=0]) {
  const copy=structuredClone(live);mutate(copy);check(!allowed(copy,true,'','mode'));
 }
 Object.assign(live,{action_locked:true,latched:false,stop_acknowledged:true});
 Object.assign(live.mechanism,{fault:6,zero_valid:0,pwm_mask:0});
 check(allowed(live,false,'','prepare'));
 live.base.fault=2048;check(!allowed(live,false,'','prepare'));
}
console.log('35 current-firmware compatibility and fault guards PASS');

state=healthy();state.firmware='GC-BASE-0.3.31';
check(!allowed(state,true,'','move'));check(allowed(state,true,'','mode'));
state.manual_debug=true;check(manualTuning(state));check(allowed(state,true,'','move'));
check(moveArgs('lift','150',1000,false,{manualTuning:true}).target_m===.15);
check(moveArgs('reach','120',3000,false,{manualTuning:true}).rpm===3000);
check(moveArgs('gripper','60').release_confirmed);
check(!moveArgs('gripper','98').release_confirmed);
check(Math.abs(moveArgs('arm','90',5,false,{manualTuning:true,speedDegS:120}).speed_rad_s-2*Math.PI/3)<1e-10);
for(const args of [['lift','-1',5,false,{manualTuning:true}],['lift','10',3001,false,{manualTuning:true}],
 ['arm','90',5,false,{manualTuning:true,speedDegS:0}],['arm','90',5,false,{manualTuning:true,speedDegS:361}]]){
 assert.throws(()=>moveArgs(...args));checks++;
}
for(const mutate of [s=>s.connected=false,s=>s.base.fault=2048,s=>s.mechanism.zero_valid=0]){
 const copy=structuredClone(state);mutate(copy);check(!allowed(copy,true,'','move'));
}
check(!ui.includes('id="real-release"'));check(ui.includes('real-speed-${a.id}'));
console.log('18 manual tuning and explicit gripper target checks PASS');
for(const angle of [0,30,83,180,270])check(Math.abs(moveArgs('arm',String(angle),5,false,{manualTuning:true}).target_rad-angle*Math.PI/180)<1e-10);
for(const angle of [-1,270.001]){assert.throws(()=>moveArgs('arm',String(angle),5,false,{manualTuning:true}));checks++;}
for(const angle of [0,270]){assert.throws(()=>moveArgs('arm',String(angle)));checks++;}
check(ui.includes('el(`${kind}-${a.id}`).max=tuning?armMax:a.max'));
console.log(`10 PE11 direct command angle checks PASS; total ${checks}`);
state=healthy();state.firmware='GC-BASE-0.3.32';state.manual_debug=true;state.group={id:0,state:0};
check(armMaximum(state)===359);check(manualTuning(state));check(allowed(state,true,'','program_run'));
check(Math.abs(moveArgs('arm','359',5,false,{manualTuning:true,armMax:359}).target_rad-359*Math.PI/180)<1e-10);
assert.throws(()=>moveArgs('arm','360',5,false,{manualTuning:true,armMax:359}));checks++;
state.sequence={status:'RUNNING'};for(const action of ['move','pose','program_run','program_save','mode'])check(!allowed(state,true,'',action));
check(allowed(state,true,'','stop'));state.sequence.status='DONE';state.group.state=1;check(!allowed(state,true,'','move'));
console.log(`12 grouped-control gates PASS; total ${checks}`);
