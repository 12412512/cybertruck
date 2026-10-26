import assert from 'node:assert/strict';
import fs from 'node:fs';
import {canRecover,allowed,manualTuning,armMaximum} from '../pi/robot/web/static/workbench-remote-state.js';
let count=0;const check=v=>{assert.ok(v);count++;};
const ready=()=>({domain:'physical',firmware:'GC-BASE-0.3.33',connected:true,site_ready:true,
    action_locked:true,stop_acknowledged:true,latched:false,base:{fault:2048,stopped:true},
    mechanism:{attached:1,config_mask:3,fault:9,pwm_mask:3}});
check(canRecover(ready(),false,'')); // 允许处理真实故障，关闭PWM须另有现场确认。
for(const change of [s=>s.domain='simulation',s=>s.firmware='GC-BASE-0.3.32',s=>s.connected=false,
    s=>s.site_ready=false,s=>s.action_locked=false,s=>s.stop_acknowledged=false,s=>s.latched=true,
    s=>s.base.stopped=false,s=>s.mechanism.attached=0,s=>s.mechanism.config_mask=1]){
    const s=ready();change(s);check(!canRecover(s,false,''));
}
check(!canRecover(ready(),true,''));check(!canRecover(ready(),false,'waiting'));
for(const status of ['CAPTURE','REQUEST','REBOOT','VERIFY','ATTACH']){
    const s=ready();s.recovery={status};check(!canRecover(s,false,''));
    for(const action of ['claim','prepare','move','program_run'])check(!allowed(s,false,'',action));
    check(allowed(s,false,'','stop'));
}
const s=ready();s.manual_debug=true;check(manualTuning(s));check(armMaximum(s)===359);
const ui=fs.readFileSync(new URL('../pi/robot/web/static/workbench-device.js',import.meta.url),'utf8');
check(ui.includes('id="real-recover"'));check(ui.includes("await send('recover'"));
check(ui.includes('placeholder="按实物填写"'));check(ui.includes('placeholder="核对后填写"'));
check(!ui.includes('step="1" value="${a.initial}"'));
console.log(`${count} operator recovery checks PASS; no device access`);
