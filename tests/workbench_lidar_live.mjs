// 实机视图不将历史配置/扩展参数当作地图障碍或动作许可。
import assert from 'node:assert/strict';
import fs from 'node:fs';
const file = new URL('../pi/robot/web/static/workbench-lidar-live.js',import.meta.url);
const {liveLines,scanLine} = await import('data:text/javascript;base64,'+fs.readFileSync(file).toString('base64'));
assert.match(liveLines(null)[0],/未连接/);
const value={domain:'physical_lidar',authorizes_motion:false,scan_allowed:false,real_map_registered:false,
    connected:true,node:1,decode_bad:0,decode_unknown:1,
    information:{firmware:'1.37',protocol:6,zone:0,running:0},
    persisted:{storage_version:11,record_sequence:22,grid_ids:[9,20,24],
        zone1_pose:{x_mm:156,y_mm:2254,heading_deg:198.4},zone2_pose:{x_mm:137,y_mm:143,heading_deg:21.3}},
    reason:'0x90仅隔离归档'};
assert.match(liveLines(value)[0],/基础通信在线/);
assert.match(liveLines(value)[1],/未选择/);
assert.match(liveLines(value)[2],/不作当前障碍/);
assert.match(liveLines(value)[3],/隔离/);
assert.match(liveLines({...value,scan_allowed:true})[0],/未连接/);
assert.match(liveLines({...value,domain:'simulation'})[0],/未连接/);
assert.match(liveLines({...value,connected:false,status:'FAULT'})[0],/异常/);
assert.match(scanLine(value),/尚未/);
const scan={domain:'physical_lidar_scan',status:'COMPLETE',zone:1,cloud_frames:5,result_frames:5,matched_frames:5,stop_confirmed:true,live:false,errors:[]};
assert.match(scanLine({...value,scan}),/历史样本/);
assert.match(scanLine({...value,scan}),/STOP双确认/);
assert.match(scanLine({...value,scan}),/不能用作空场/);
assert.match(liveLines({...value,scan})[0],/静扫COMPLETE/);
console.log('13 lidar live view checks PASS');
