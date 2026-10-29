import assert from 'node:assert/strict';
import {setText,setSteps} from '../pi/robot/web/static/workbench-dom.js';
import {mechanismNotice} from '../pi/robot/web/static/workbench-mech-status.js';
import {allowed} from '../pi/robot/web/static/workbench-remote-state.js';
let writes=0,rebuilds=0,text='';
const node={dataset:{},get textContent(){return text;},set textContent(v){text=v;writes++;},
    ownerDocument:{createElement:()=>({})},replaceChildren(...items){this.children=items;rebuilds++;}};
for(let i=0;i<100;i++){setText(node,'状态正常');setSteps(node,[['连接',true],['建零',false]]);}
assert.equal(writes,1);assert.equal(rebuilds,1);
const first=node.children[0];setSteps(node,[['连接',1],['建零',0]]);assert.equal(node.children[0],first);
setSteps(node,[['连接',true],['建零',true]]);assert.equal(rebuilds,2);
let s={connected:true,domain:'physical',firmware:'GC-BASE-0.3.40',site_ready:true,manual_debug:true,
    base:{fault:0,stopped:true},mechanism:{fault:6,attached:1,config_mask:3,base_stopped:1},
    safety_events:[{source:'lease-expired',reason:'续约未到'}]};
assert.ok(mechanismNotice(s).text.includes('续约超时'));
assert.ok(!allowed(s,true,'','move'));
s.connected=false;assert.ok(!allowed(s,true,'','move'));
console.log('DOM重复100轮不重建、停止来源及断线拒绝动作 PASS');
