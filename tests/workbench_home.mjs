import assert from 'node:assert/strict';
import fs from 'node:fs';
import {allowed,manualTuning,armMaximum,canRecover} from '../pi/robot/web/static/workbench-remote-state.js';
let count=0;const check=v=>{assert.ok(v);count++;};
const ready=()=>({domain:'physical',firmware:'GC-BASE-0.3.34',connected:true,site_ready:true,manual_debug:true,
    home:{id:0,state:0,zero_version:1},base:{fault:0,stopped:true},
    mechanism:{attached:1,fault:0,config_mask:3,base_stopped:1,zero_valid:1,axis_state:0,servo_state:0,pose_state:0}});
check(allowed(ready(),true,'','home'));check(!allowed(ready(),true,'','home_confirm'));
for(const mutate of [s=>s.firmware='GC-BASE-0.3.33',s=>s.connected=false,s=>s.manual_debug=false,
    s=>s.mechanism.zero_valid=0,s=>s.base.fault=2048,s=>s.mechanism.fault=14,s=>s.home=null]){
    const s=ready();mutate(s);check(!allowed(s,true,'','home'));
}
check(!allowed(ready(),false,'','home'));check(!allowed(ready(),true,'waiting','home'));
for(const stage of [1,2,3]){
    const s=ready();s.home.state=stage;
    for(const action of ['move','mode','pose','program_run','home'])check(!allowed(s,true,'',action));
    check(allowed(s,true,'','home_confirm')===(stage===3));check(allowed(s,true,'','stop'));
}
const s=ready();s.home.state=4;check(allowed(s,true,'','move'));check(manualTuning(s));check(armMaximum(s)===359);
s.action_locked=s.stop_acknowledged=true;check(canRecover(s,false,''));
const source=fs.readFileSync(new URL('../pi/robot/web/static/workbench-device.js',import.meta.url),'utf8');
check(source.includes('id="real-home"'));check(source.includes('id="real-home-confirm"'));
check(source.includes("act('home',{confirmed:true,safe_zero:true})"));
check(source.includes('不能保证按网页速度慢速回零'));check(source.includes('高级 · 手动记录当前舵机参考（可选）'));
console.log(`${count} explicit home UI checks PASS; no device access`);
