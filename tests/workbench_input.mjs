// 工作台键盘契约，不连接浏览器或设备。
import assert from 'node:assert/strict';
import {motionDirection, matchesRelease} from '../pi/robot/web/static/workbench-input.js';

const event = (code, extra = {}) => ({code, target:{tagName:'BODY'}, ...extra});
assert.equal(motionDirection(event('KeyW')), '1,0,0');
assert.equal(motionDirection(event('KeyA', {ctrlKey:true})), null);
assert.equal(motionDirection(event('KeyW', {metaKey:true})), null);
assert.equal(motionDirection(event('KeyW', {altKey:true})), null);
assert.equal(motionDirection(event('KeyW', {repeat:true})), null);
for (const tagName of ['INPUT', 'SELECT', 'TEXTAREA']) {
    assert.equal(motionDirection(event('KeyA', {target:{tagName}})), null);
}
assert.equal(motionDirection(event('KeyW', {target:{isContentEditable:true}})), null);
for (const code of ['Tab', 'Return', 'Enter', 'ArrowDown', 'ShiftLeft', 'ControlLeft']) {
    assert.equal(matchesRelease(event(code), undefined), false);
    assert.equal(motionDirection(event(code)), null);
}
assert.equal(matchesRelease(event('KeyW'), '1,0,0'), true);
assert.equal(matchesRelease(event('KeyD'), '1,0,0'), false);
console.log('工作台键盘契约：23项断言通过');
