// 联调轨迹只能替换对应版本的离线视图，不能改变实车数据。
import assert from 'node:assert/strict';
import {modelView} from '../pi/robot/web/static/workbench-execution.js';
const state={simulation:{mode:'IDLE',pose:[0,0,0]},planning:{plan:{id:'one'}},real:{fault:7}};
assert.equal(modelView(state),state.simulation);
state.execution_check={visible:true,source_plan_id:'two',trace:[[1,2,3]],status:'COMPLETE'};
assert.equal(modelView(state),state.simulation);
state.execution_check={...state.execution_check,source_plan_id:'one',id:'sim',elapsed_s:2,total:1,completed:1,stop_confirmed:true,rpm:[0,0,0,0]};
assert.deepEqual(modelView(state).pose,[1,2,3]);
assert.equal(modelView(state).mode,'COMPLETE');
assert.equal(modelView(state).stopped,true);
assert.equal(modelView(state).fault,undefined);
assert.equal(state.real.fault,7);
assert.equal(state.simulation.mode,'IDLE');
state.execution_check.visible=false;
assert.equal(modelView(state),state.simulation);
console.log('9 execution view checks passed');
