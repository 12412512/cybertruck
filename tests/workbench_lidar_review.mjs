import assert from 'node:assert/strict';
import fs from 'node:fs';
const base = new URL('../pi/robot/web/static/', import.meta.url);
const source = fs.readFileSync(new URL('workbench-lidar-review.js', base));
const {reviewLines, reviewPoints} = await import('data:text/javascript;base64,'+source.toString('base64'));
const value = JSON.parse(fs.readFileSync(new URL('lidar-review.json', base)));
assert.match(reviewLines(value)[0], /-145 mm/);
assert.match(reviewLines(value)[1], /35\.3 mm/);
assert.match(reviewLines(value)[2], /历史核对/);
assert.equal(reviewPoints(value).length, 4);
for (const key of ['live', 'applied', 'real_map_registered', 'auto_execution_ready', 'authorizes_motion']) {
    assert.throws(() => reviewLines({...value, [key]:true}));
}
const bad = structuredClone(value); bad.points[0].map_m = [3, 0];
assert.throws(() => reviewPoints(bad));
assert.throws(() => reviewLines({...value, domain:'simulation'}));
console.log('11 lidar review checks PASS');
