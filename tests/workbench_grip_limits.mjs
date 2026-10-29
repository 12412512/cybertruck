import assert from 'node:assert/strict';
import {gripHold,gripMaximum,moveArgs,allowed} from '../pi/robot/web/static/workbench-remote-state.js';
import {sequenceRows,sameRows} from '../pi/robot/web/static/workbench-sequence.js';
let checks=0;const check=x=>{assert.ok(x);checks++;};
check(gripHold===98);
for(const firmware of ['GC-BASE-0.3.37','GC-BASE-0.3.38','unknown']){
    const state={firmware},max=firmware.endsWith('.38')?100:98;
    check(gripMaximum(state)===max);
    for(const angle of [0,60,98,99,100,100.001,-1]){
        const options={manualTuning:true,gripMax:gripMaximum(state),speedDegS:45};
        const rows=[{order:1,axis:'gripper',target:angle,speed:45}];
        if(angle>=0&&angle<=max){
            const args=moveArgs('gripper',angle,5,false,options);
            check(Math.abs(args.target_rad-angle*Math.PI/180)<1e-12);
            check(args.release_confirmed===(angle<98));
            const sequence=sequenceRows(rows,'ordered',state);
            check(sameRows(sequence,[{order:1,...args}]));
            check(sequenceRows(rows,'together',state).length===1);
        }else{
            assert.throws(()=>moveArgs('gripper',angle,5,false,options));checks++;
            assert.throws(()=>sequenceRows(rows,'ordered',state));checks++;
        }
    }
}
assert.throws(()=>moveArgs('gripper',101,5,false,{manualTuning:true,gripMax:270}));checks++;
const healthy={domain:'physical',connected:true,firmware:'GC-BASE-0.3.38',site_ready:true,manual_debug:true,
    base:{fault:0,stopped:true},group:{state:0},home:{state:0},
    mechanism:{fault:0,base_stopped:1,attached:1,config_mask:3,zero_valid:1,axis_state:0,servo_state:9,pose_state:0}};
check(allowed(healthy,true,'','move'));check(allowed(healthy,true,'','program_run'));
healthy.mechanism.fault=6;check(!allowed(healthy,true,'','move'));check(!allowed(healthy,true,'','program_run'));
console.log(`${checks} gripper target/safety bound/version/fault checks PASS; no network or device calls`);
