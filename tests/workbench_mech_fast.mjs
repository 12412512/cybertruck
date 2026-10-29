import assert from 'node:assert/strict';
import {mechanismNotice} from '../pi/robot/web/static/workbench-mech-status.js';
import {allowed,gripMaximum,armMaximum} from '../pi/robot/web/static/workbench-remote-state.js';
const state={connected:true,domain:'physical',firmware:'GC-BASE-0.3.39',site_ready:true,manual_debug:true,
    base:{fault:0,stopped:true},group:{state:0},home:{state:0},
    mechanism:{fault:0,zero_valid:1,attached:1,config_mask:3,base_stopped:1,axis_state:0,servo_state:9,pose_state:0}};
assert.equal(gripMaximum(state),100);assert.equal(armMaximum(state),359);
assert.ok(allowed(state,true,'','move'));assert.ok(allowed(state,true,'','program_run'));
state.mechanism.fault=104;
state.mechanism_failure={captured:1,tick_ms:215418,axis:2,position_um:[1,83538],rpm:[0,0],age_ms:[20,30]};
let notice=mechanismNotice(state);
assert.equal(notice.kind,'fault');assert.ok(notice.text.includes('ID6'));
assert.ok(notice.text.includes('83.538'));assert.ok(notice.text.includes('20/30ms'));
assert.ok(!allowed(state,true,'','move'));
state.mechanism_failure.position_um[0]=-9223372036854775808;
notice=mechanismNotice(state);assert.ok(notice.text.includes('未知/83.538'));
console.log('10 fast-mechanism UI and frozen-fault checks PASS; no device calls');
