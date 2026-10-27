import assert from 'node:assert/strict';
import {mechanismNotice, faultText} from '../pi/robot/web/static/workbench-mech-status.js';
import {allowed,blockedReasons} from '../pi/robot/web/static/workbench-remote-state.js';

const state=()=>({domain:'physical',connected:true,firmware:'GC-BASE-0.3.34',site_ready:true,
    manual_debug:true,base:{fault:0,stopped:true},mechanism:{fault:0,attached:1,config_mask:3,
        base_stopped:1,zero_valid:1,axis_state:0,axis_id:4,servo_state:9,servo_id:3,pose_state:0,pwm_mask:7}});
let checks=0;
const check=value=>{assert.ok(value);checks++;};
let live=state();
for(let id=4;id<7;id++) {
    live.mechanism.axis_id=id;live.mechanism.axis_state=3;
    check(mechanismNotice(live).kind==='busy');
    check(!allowed(live,true,'','move'));
    live.mechanism.axis_state=6;
    check(mechanismNotice(live).kind==='ready');
    check(allowed(live,true,'','move')); // 正常结束不留下“只能点一次”的锁。
}
live.mechanism.fault=105;live.mechanism.axis_state=7;live.mechanism.zero_valid=0;
check(mechanismNotice(live).kind==='fault');
check(mechanismNotice(live).text.includes('另一轴'));
check(blockedReasons(live,true,'').some(x=>x.includes('105')&&x.includes('另一轴')));
check(!allowed(live,true,'','move'));
check(allowed(live,true,'','stop'));
check(mechanismNotice({...live,connected:false}).kind==='muted'); // 不拿旧故障当当前反馈。
check(faultText(999).includes('999'));
live=state();check(mechanismNotice(live,'伸缩目标').kind==='busy');
live.manual_debug=false;check(mechanismNotice(live).text.includes('进入手动调试'));
live.mechanism.zero_valid=0;check(mechanismNotice(live).text.includes('尚未建零'));
console.log(`${checks} mechanical progress/fault/next-command checks PASS; no network or device calls`);
