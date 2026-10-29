import assert from 'node:assert/strict';
import {allowed,manualTuning,armMaximum} from '../pi/robot/web/static/workbench-remote-state.js';
import {mechanismNotice,parkingText} from '../pi/robot/web/static/workbench-mech-status.js';
const state={domain:'physical',connected:true,firmware:'GC-BASE-0.3.35',site_ready:true,manual_debug:true,
    base:{fault:0,stopped:true},group:{id:0,state:0},home:{id:0,state:0},
    mechanism:{fault:0,attached:1,config_mask:3,base_stopped:1,zero_valid:1,pwm_mask:7,
        axis_state:0,servo_state:9,pose_state:0}};
assert.ok(manualTuning(state));assert.equal(armMaximum(state),359);
for(const action of ['move','program_run','home'])assert.ok(allowed(state,true,'',action));
state.mechanism.fault=9;state.mechanism.zero_valid=0;
state.park_failure={captured:1,mask:4,wheel:3,action_id:32,wheels:[{},{},{rpm:-3,
    speed_age_ms:20,position_age_ms:30,status_age_ms:40}]};
assert.match(parkingText(state.park_failure),/ID3轮速-3RPM/);
assert.match(mechanismNotice(state).text,/首次停车条件失败 #32/);
assert.match(mechanismNotice(state).text,/20\/30\/40ms/);
for(const action of ['move','program_run','home'])assert.ok(!allowed(state,true,'',action));
assert.ok(allowed(state,true,'','stop'));
state.connected=false;assert.equal(mechanismNotice(state).kind,'muted');
assert.equal(parkingText({captured:0}), '');
state.connected=true;state.firmware='GC-BASE-0.3.36';
state.mechanism.fault=0;state.mechanism.zero_valid=1;
assert.ok(allowed(state,true,'','move'));
assert.match(mechanismNotice(state).text,/已取消车轮1RPM限制/);
state.mechanism.fault=9;
assert.ok(!allowed(state,true,'','move')); // 新策略不能在网页偷偷清除已有锁存。
state.firmware='GC-BASE-0.3.37';state.mechanism.fault=0;
assert.ok(manualTuning(state));assert.equal(armMaximum(state),359);
for(const action of ['move','program_run','home'])assert.ok(allowed(state,true,'',action));
assert.match(mechanismNotice(state).text,/已取消车轮1RPM限制/);
state.mechanism.fault=9;assert.ok(!allowed(state,true,'','move'));
console.log('22 servo-channel firmware / frozen evidence / fault-lock UI checks PASS; no device calls');
