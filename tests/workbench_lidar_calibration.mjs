import assert from 'node:assert/strict';
import fs from 'node:fs';
const source = fs.readFileSync(new URL('../pi/robot/web/static/workbench-lidar-calibration.js',import.meta.url));
const {calibrationLines} = await import('data:text/javascript;base64,'+source.toString('base64'));
const value = {domain:'physical_lidar',authorizes_motion:false};
assert.match(calibrationLines(value)[0],/尚未部署/);
const ready = {...value,calibration_capabilities:{prepare:true,write:false,board_ready:false},
    calibration:{status:'PREPARED',backup_version:'ab'.repeat(32),backup_saved:true,write_issued:false,errors:[]}};
assert.match(calibrationLines(ready)[0],/只读准备/);
assert.match(calibrationLines(ready)[1],/已归档/);
assert.match(calibrationLines(ready)[1],/未发送09/);
assert.match(calibrationLines({...ready,calibration_capabilities:{prepare:true,write:true,board_ready:false}})[0],/拦截/);
assert.match(calibrationLines({...ready,calibration:{...ready.calibration,write_issued:true}})[1],/不宣称成功/);
assert.match(calibrationLines({...ready,calibration:{...ready.calibration,status:'CANCELLED'}})[0],/已取消（未校准）/);
assert.match(calibrationLines({...ready,calibration:{...ready.calibration,status:'FAILED'}})[0],/已结束（未校准）/);
assert.throws(() => calibrationLines({...value,domain:'simulation'}));
assert.throws(() => calibrationLines({...value,authorizes_motion:true}));
console.log('10 calibration view checks PASS');
