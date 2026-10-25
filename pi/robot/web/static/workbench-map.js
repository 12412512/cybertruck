// 只绘制带明确数据域的模型；画布Y翻转不进入控制坐标。
import {modelView} from './workbench-execution.js?v=1';
const ns = 'http://www.w3.org/2000/svg';
function node(kind, attrs = {}, text = '') {
    const item = document.createElementNS(ns, kind);
    for (const [key, value] of Object.entries(attrs)) item.setAttribute(key, value);
    if (text) item.textContent = text;
    return item;
}
const point = p => `${p[0] * 1000},${(2.4 - p[1]) * 1000}`;
const names = {start_1:'启停区 1', start_2:'启停区 2', qr:'扫码', raw:'原料', rough:'粗加工', staging:'暂存', north:'north'};
let fieldKey = '';
let plannedId = '';
let path, actual, body, labels;
let obstaclesLayer;
let mapObserver = null;

export function onMapPoint(handler) {
    mapObserver = handler;
    const svg = document.getElementById('map');
    svg.addEventListener('click', event => {
        const target = svg.createSVGPoint();
        target.x = event.clientX;
        target.y = event.clientY;
        const local = target.matrixTransform(svg.getScreenCTM().inverse());
        const x = local.x / 1000;
        const y = 2.4 - local.y / 1000;
        if (x >= 0 && x <= 2.4 && y >= 0 && y <= 2.4) {
            mapObserver({x_m:x, y_m:y, obstacleId:event.target.closest('[data-obstacle]')?.dataset.obstacle});
        }
    });
}

export function drawObstacles(items) {
    if (!obstaclesLayer) return;
    obstaclesLayer.replaceChildren();
    for (const item of items) {
        const circle = node('circle', {cx:item.x_m * 1000, cy:(2.4 - item.y_m) * 1000,
            r:item.diameter_m * 500, fill:'#141c23', stroke:'#fff', 'stroke-width':3,
            'data-obstacle':item.id});
        const title = node('title', {}, `${item.id} / 直径${(item.diameter_m * 1000).toFixed(0)}mm`);
        circle.append(title);
        obstaclesLayer.append(circle);
        obstaclesLayer.append(node('text', {x:item.x_m * 1000 + item.diameter_m * 500 + 10,
            y:(2.4 - item.y_m) * 1000 - 15, fill:'#263c49', 'font-size':32}, item.id));
    }
}

export function drawMap(state) {
    const svg = document.getElementById('map');
    if (!state.available || state.domain !== 'simulation') {
        if (body) body.setAttribute('visibility', 'hidden');
        return;
    }
    const active = state.simulation.mode === 'RUNNING' && state.active_plan;
    const field = active ? state.active_plan.field : state.field;
    if (fieldKey !== field.version || !body) {
        svg.replaceChildren();
        fieldKey = field.version;
        const defs = node('defs');
        const pattern = node('pattern', {id:'grid', width:100, height:100, patternUnits:'userSpaceOnUse'});
        pattern.append(node('path', {d:'M 100 0 L 0 0 0 100', fill:'none', stroke:'#e0e9ee', 'stroke-width':2}));
        defs.append(pattern); svg.append(defs);
        svg.append(node('rect', {x:0, y:0, width:2400, height:2400, fill:'#f1f5f7', stroke:'#bacbd4', 'stroke-width':4}));
        svg.append(node('rect', {x:0, y:0, width:2400, height:2400, fill:'url(#grid)'}));
        for (const zone of field.zones) {
            const [x0, y0, x1, y1] = zone.bounds_m;
            const start = zone.id.startsWith('start_');
            const work = zone.id.endsWith('surface');
            svg.append(node('rect', {x:x0 * 1000, y:(2.4 - y1) * 1000,
                width:(x1 - x0) * 1000, height:(y1 - y0) * 1000,
                fill:start ? '#d4e6f7' : work ? '#dbe6e9' : '#f5edca',
                stroke:start ? '#91b8d7' : work ? '#bfced3' : '#dfd5ac',
                'stroke-width':3, rx:start ? 10 : 2}));
        }
        for (const [, a, b] of field.edges) svg.append(node('line', {
            x1:field.nodes[a][0] * 1000, y1:(2.4 - field.nodes[a][1]) * 1000,
            x2:field.nodes[b][0] * 1000, y2:(2.4 - field.nodes[b][1]) * 1000,
            stroke:'#c6d5dd', 'stroke-width':3, 'stroke-dasharray':'13 15'}));
        path = node('polyline', {fill:'none', stroke:'#7caec6', 'stroke-width':15, 'stroke-linejoin':'round', opacity:.75});
        actual = node('polyline', {fill:'none', stroke:'#078c86', 'stroke-width':9, 'stroke-linejoin':'round'});
        svg.append(path, actual);
        labels = node('g');
        for (const [id, text] of Object.entries(names)) {
            const [x, y] = field.nodes[id];
            labels.append(node('circle', {cx:x * 1000, cy:(2.4 - y) * 1000, r:13, fill:'#fff', stroke:'#7196ab', 'stroke-width':5}));
            labels.append(node('text', {x:x * 1000 + 28, y:(2.4 - y) * 1000 - 25,
                fill:'#638092', 'font-size':43}, text));
        }
        svg.append(labels);
        obstaclesLayer = node('g');
        svg.append(obstaclesLayer);
        body = node('g');
        body.append(node('rect', {x:-145, y:-135, width:290, height:270, rx:16,
            fill:'#b9dfda', stroke:'#077e7c', 'stroke-width':8}));
        body.append(node('path', {d:'M -20 -42 L 76 0 L -20 42 Z', fill:'#077e7c'}));
        body.append(node('circle', {cx:0, cy:0, r:8, fill:'#fff'}));
        svg.append(body);
        for (const value of [0, .6, 1.2, 1.8, 2.4]) {
            svg.append(node('text', {x:value * 1000, y:2485, 'text-anchor':'middle', fill:'#9badb7', 'font-size':34}, value.toFixed(1)));
            svg.append(node('text', {x:-28, y:(2.4 - value) * 1000 + 10, 'text-anchor':'end', fill:'#9badb7', 'font-size':34}, value.toFixed(1)));
        }
        svg.append(node('text', {x:2370, y:2535, fill:'#698799', 'font-size':36}, 'X / m'));
        plannedId = '';
    }
    const plan = state.simulation.mode === 'RUNNING' && state.active_plan ? state.active_plan : state.planning?.plan;
    if ((plan?.id || '') !== plannedId) {
        path.setAttribute('points', (plan?.reference || []).map(point).join(' '));
        plannedId = plan?.id || '';
    }
    const model = modelView(state);
    actual.setAttribute('points', (model.trace || []).map(point).join(' '));
    actual.setAttribute('stroke', model === state.simulation ? '#078c86' : '#7c3aed');
    drawObstacles(field.obstacles || []);
    const pose = model.pose;
    body.setAttribute('visibility', 'visible');
    body.setAttribute('transform', `translate(${point(pose).replace(',', ' ')}) rotate(${-pose[2] * 180 / Math.PI})`);
}

export function drawSpeed(plan) {
    const svg = document.getElementById('speed-chart');
    svg.replaceChildren();
    if (!plan?.speed_samples?.length) return;
    const samples = plan.speed_samples;
    const duration = samples.at(-1)[0] || 1;
    const peak = Math.max(.01, ...samples.map(p => p[1]));
    for (let index = 0; index < 4; index++) {
        const y = 20 + index * 30;
        svg.append(node('line', {x1:42, y1:y, x2:625, y2:y, stroke:'#e4ecef'}));
        svg.append(node('text', {x:2, y:y + 4, fill:'#91a5b0', 'font-size':10}, (peak * (1 - index / 3)).toFixed(2)));
    }
    svg.append(node('polyline', {points:samples.map(p => `${42 + 583 * p[0] / duration},${110 - 90 * p[1] / peak}`).join(' '),
        fill:'none', stroke:'#188b88', 'stroke-width':2}));
    svg.append(node('text', {x:42, y:137, fill:'#91a5b0', 'font-size':10}, '0 s'));
    svg.append(node('text', {x:580, y:137, fill:'#91a5b0', 'font-size':10}, `${duration.toFixed(1)} s`));
}
