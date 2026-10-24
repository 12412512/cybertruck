// 场景编辑只修改草稿。坐标按map米，直径与间隙在输入边界换成毫米。
import {onMapPoint, drawObstacles} from './workbench-map.js';

const $ = id => document.getElementById(id);
let obstacles = [];
let changed, notify, isRunning;

function numeric(id, low, high) {
    const value = $(id).valueAsNumber;
    if (!Number.isFinite(value) || value < low || value > high) throw new Error('场景参数超出输入范围');
    return value;
}

export function sceneValues() {
    return {max_rpm:numeric('model-rpm', 20, 350),
        docking:{raw_m:numeric('gap-raw', 0, 250) / 1000,
            rough_m:numeric('gap-rough', 0, 250) / 1000,
            staging_m:numeric('gap-staging', 0, 250) / 1000},
        obstacles:structuredClone(obstacles)};
}

export function applyScene(request) {
    $('model-rpm').value = request.max_rpm;
    $('gap-raw').value = request.docking.raw_m * 1000;
    $('gap-rough').value = request.docking.rough_m * 1000;
    $('gap-staging').value = request.docking.staging_m * 1000;
    $('obstacle-diameter').value = 50;
    $('map-mode').value = 'view';
    $('map').classList.remove('placing');
    obstacles = structuredClone(request.obstacles);
    renderList();
}

function renderList() {
    const body = $('obstacle-list');
    body.replaceChildren();
    for (const obstacle of obstacles) {
        const row = document.createElement('tr');
        const name = document.createElement('td');
        name.textContent = obstacle.id;
        row.append(name);
        for (const key of ['x_m', 'y_m', 'diameter_m']) {
            const cell = document.createElement('td');
            const input = document.createElement('input');
            input.type = 'number';
            input.value = Math.round(obstacle[key] * 1000);
            input.min = key === 'diameter_m' ? 10 : 0;
            input.max = key === 'diameter_m' ? 300 : 2400;
            input.step = 1;
            input.className = 'scene-input';
            input.setAttribute('aria-label', `${obstacle.id} ${key === 'x_m' ? 'X毫米' : key === 'y_m' ? 'Y毫米' : '直径毫米'}`);
            input.onchange = () => {
                const value = input.valueAsNumber;
                if (!Number.isFinite(value) || value < Number(input.min) || value > Number(input.max)) {
                    notify('障碍物坐标或直径超出范围', true);
                    return;
                }
                obstacle[key] = value / 1000;
                drawObstacles(obstacles);
                changed();
            };
            cell.append(input);
            row.append(cell);
        }
        const cell = document.createElement('td');
        const remove = document.createElement('button');
        remove.textContent = '删除';
        remove.className = 'scene-input';
        remove.onclick = () => {
            obstacles = obstacles.filter(item => item.id !== obstacle.id);
            renderList();
            changed();
        };
        cell.append(remove);
        row.append(cell);
        body.append(row);
    }
    $('obstacle-empty').hidden = !!obstacles.length;
    drawObstacles(obstacles);
}

export function initScene(options) {
    ({changed, notify, isRunning} = options);
    for (const id of ['model-rpm', 'gap-raw', 'gap-rough', 'gap-staging']) $(id).oninput = changed;
    $('clear-obstacles').onclick = () => {
        obstacles = [];
        renderList();
        changed();
    };
    $('map-mode').onchange = () => {
        $('map').classList.toggle('placing', $('map-mode').value !== 'view');
        notify('添加/删除障碍只修改模拟场景；自动重规划不会启动模型。');
    };
    onMapPoint(point => {
        if (isRunning() || $('map-mode').value === 'view') return;
        if ($('map-mode').value === 'remove') {
            if (!point.obstacleId) return;
            obstacles = obstacles.filter(item => item.id !== point.obstacleId);
        } else {
            if (obstacles.length >= 20) { notify('最多布置20个障碍物', true); return; }
            let diameter;
            try { diameter = numeric('obstacle-diameter', 10, 300) / 1000; }
            catch (error) { notify(error.message, true); return; }
            let id = 1;
            while (obstacles.some(item => item.id === `B${id}`)) id++;
            obstacles.push({id:`B${id}`, x_m:Math.round(point.x_m * 1000) / 1000,
                y_m:Math.round(point.y_m * 1000) / 1000, diameter_m:diameter});
        }
        renderList();
        changed();
    });
}

export function overlayDraft() {
    if (!isRunning()) drawObstacles(obstacles);
}
