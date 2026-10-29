// 地图只能叠加与当前起点匹配的模拟标定；实车/外部未配准数据不能伪装成地图观测。
import assert from 'node:assert/strict';
import {scanMap} from '../pi/robot/web/static/workbench-lidar.js';
const valid={domain:'simulation',live:false,synthetic:true,real_map_registered:false,
    simulation_registered:true,start:'start_1',points:[[.3,2.1]]};
const state={draft:{start:'start_1'},lidar:valid,device:{fault:2048}};
assert.equal(scanMap(state),valid);
for (const change of [{domain:'physical'},{live:true},{synthetic:false},{real_map_registered:true},
                       {simulation_registered:false},{start:'start_2'}]) {
    assert.equal(scanMap({...state,lidar:{...valid,...change}}),null);
}
assert.equal(scanMap({}),null);
assert.equal(state.device.fault,2048);
console.log('9 lidar view checks passed');
