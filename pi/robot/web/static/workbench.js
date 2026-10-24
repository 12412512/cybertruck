// 界面只经网关调用独立核心；输入草稿、离线执行、实车状态分别表达。
import {drawMap, drawSpeed} from './workbench-map.js';

const $ = id => document.getElementById(id);
const text = (id, value) => { $(id).textContent = value; };
const uid = () => typeof crypto.randomUUID === 'function' ? crypto.randomUUID()
    : [...crypto.getRandomValues(new Uint8Array(16))].map(value => value.toString(16).padStart(2, '0')).join('');
const client = uid(); // 每个标签页独立身份，刷新不自动续跑。
const names = {start_1:'启停区1', start_2:'启停区2', qr:'扫码', raw:'原料', rough:'粗加工', staging:'暂存', north:'north预停点'};
const modes = {IDLE:'待命', MANUAL:'按住点动', RUNNING:'闭环执行中', STOPPING:'正在停车', STOPPED:'已停止', COMPLETE:'动作完成', FAILED:'模型故障'};
const presets = {batch:['qr','raw','rough','staging','raw','rough','staging','start_1'],
    tour:['qr','raw','rough','staging','start_1'], raw:['qr','raw']};
const legNames = {
    raw:['启停区1 → 扫码', '扫码 → 原料'],
    tour:['启停区1 → 扫码', '扫码 → 原料', '原料 → 粗加工', '粗加工 → 暂存', '暂存 → 上方道路', '上方道路 → 启停区1'],
    batch:['启停区1 → 扫码', '扫码 → 第一批原料', '原料 → 粗加工', '粗加工 → 暂存',
        '暂存 → 第二批原料', '原料 → 粗加工', '粗加工 → 暂存', '暂存 → 上方道路', '上方道路 → 启停区1']
};
let token = '', tokenSince = 0, sequence = 0, generation = 0, queue = Promise.resolve();
let current = null, received = 0, planId = '', dirty = true, lastJob = '', lastEvents = '';
let held = null, heldButton = null, manualBusy = false;
let targets = ['qr', 'raw', 'rough', 'staging', 'start_1'];

function notify(message, error = false) {
    text('notice', message);
    $('notice').classList.toggle('error', error);
}
function online() { return current?.available && performance.now() - received < 700; }
async function post(op, args = {}, lease = token) {
    const response = await fetch('/api/workbench/command', {
        method:'POST', headers:{'Content-Type':'application/json', 'X-Workbench':'1'},
        body:JSON.stringify({op, args, client, token:lease, sequence:++sequence, request_id:uid()}),
        signal:AbortSignal.timeout(1200), keepalive:op === 'stop'
    });
    const result = await response.json();
    if (!response.ok) throw new Error(typeof result.detail === 'string' ? result.detail : '参数无效，请检查输入范围');
    return result;
}
function command(op, args = {}) {
    const epoch = generation;
    const next = queue.then(() => {
        if (epoch !== generation) throw new Error('停止前的待发命令已撤销');
        return post(op, args);
    });
    queue = next.catch(() => {});
    return next;
}
function clearHeld() {
    held = null;
    heldButton?.classList.remove('pressed');
    heldButton = null;
}
async function stop(reason = '已请求停止模拟') {
    generation++;
    clearHeld();
    token = '';
    updateButtons();
    try { await post('stop', {}, ''); notify(reason + '；控制权已释放。'); }
    catch (error) { notify('网关未确认停止；离线核心点动租约会独立过期。' + error.message, true); }
}
async function stopHeld(reason) {
    generation++;
    clearHeld();
    if (!token) return;
    try { await post('manual_stop'); notify(`${reason}；等待模型停稳，可继续点动。`); }
    catch (error) { notify(`点动停车未获确认，核心350ms续约保护仍生效。${error.message}`, true); }
}
function number(id, low, high) {
    const value = $(id).valueAsNumber;
    if (!Number.isFinite(value) || value < low || value > high) throw new Error(`${$(id).labels?.[0]?.textContent.trim() || id} 超出范围`);
    return value;
}
async function pulse() {
    if (!held || manualBusy) return;
    if (!token || !online()) { clearHeld(); return; }
    manualBusy = true;
    try {
        const speed = number('jog-speed', .01, .15);
        const yaw = number('jog-yaw', 1, 28) * Math.PI / 180;
        const [x, y, w] = held;
        const norm = Math.max(1, Math.hypot(x, y));
        await command('manual', {vx_m_s:x * speed / norm, vy_m_s:y * speed / norm, wz_rad_s:w * yaw});
    } catch (error) {
        clearHeld(); await stop('点动请求异常，停止模拟'); notify(error.message, true);
    } finally { manualBusy = false; }
}
function hold(button) {
    if (!token || !online() || button.disabled || held) return;
    held = button.dataset.dir.split(',').map(Number);
    heldButton = button;
    button.classList.add('pressed');
    pulse();
}
for (const button of document.querySelectorAll('.jog')) {
    button.addEventListener('pointerdown', event => {
        if (event.button !== 0) return;
        event.preventDefault(); button.setPointerCapture(event.pointerId); hold(button);
    });
    for (const name of ['pointerup', 'pointercancel', 'lostpointercapture']) button.addEventListener(name, () => {
        if (heldButton === button) stopHeld('方向控制已松开');
    });
    button.addEventListener('contextmenu', event => event.preventDefault());
}
const keys = {KeyW:'1,0,0', KeyS:'-1,0,0', KeyA:'0,1,0', KeyD:'0,-1,0', KeyQ:'0,0,1', KeyE:'0,0,-1'};
window.addEventListener('keydown', event => {
    if (event.code === 'Escape' || (event.code === 'Space' && !/INPUT|SELECT|TEXTAREA/.test(event.target.tagName))) {
        event.preventDefault(); stop(); return;
    }
    if (!keys[event.code] || event.repeat || /INPUT|SELECT|TEXTAREA/.test(event.target.tagName) || $('page-drive').hidden) return;
    event.preventDefault();
    hold(document.querySelector(`[data-dir="${keys[event.code]}"]`));
});
window.addEventListener('keyup', event => {
    if (heldButton?.dataset.dir === keys[event.code]) stopHeld('键盘已松开');
});
window.addEventListener('blur', () => { if (held) stopHeld('窗口失焦'); });
document.addEventListener('visibilitychange', () => { if (document.hidden && held) stopHeld('页面已隐藏'); });
window.addEventListener('pagehide', () => { if (held) stop('页面关闭'); });
window.addEventListener('offline', () => { if (held) stop('浏览器断网'); });
$('stop').onclick = () => stop();
$('pad-stop').onclick = () => stop();
$('claim').onclick = async () => {
    const epoch = generation;
    try {
        const result = await command('claim');
        if (epoch !== generation) return;
        token = result.token;
        tokenSince = performance.now();
        notify('已取得离线控制权。按住方向键运动，松开即请求停车。');
        updateButtons();
    } catch (error) { notify(error.message, true); }
};
$('release').onclick = async () => {
    if (held) { await stop(); return; }
    try { await command('release'); token = ''; updateButtons(); }
    catch (error) { notify(error.message, true); }
};
function updateButtons() {
    const enabled = !!token && online();
    const mode = current?.simulation?.mode;
    const idle = current?.simulation?.stopped && mode !== 'FAILED';
    for (const button of document.querySelectorAll('.controlled')) button.disabled = !enabled;
    for (const button of document.querySelectorAll('.jog')) button.disabled = !enabled || (!idle && mode !== 'MANUAL');
    $('move').disabled = !enabled || !idle;
    $('reset').disabled = !enabled || !current?.simulation?.stopped;
    $('plan').disabled = !enabled || current?.planning?.status === 'PLANNING';
    $('run').disabled = !enabled || !idle || dirty || !planId;
    $('save').disabled = !enabled || dirty || !planId;
    $('claim').disabled = !online() || (!!current?.owner?.client && current.owner.client !== client);
    $('claim').hidden = !!token;
    $('release').hidden = !token;
}
function draftChanged() {
    dirty = true;
    text('draft-state', '草稿已修改。重新生成路线后才能运行或保存，不会改变正在执行的版本。');
    updateButtons();
}
function sequenceView() {
    const source = $('source').value;
    $('custom-editor').hidden = source !== 'custom';
    const stops = [$('start').value, ...(source === 'custom' ? targets : presets[source])];
    $('route-sequence').replaceChildren();
    stops.forEach((name, index) => {
        if (index) { const arrow = document.createElement('b'); arrow.textContent = '→'; $('route-sequence').append(arrow); }
        const item = document.createElement('span'); item.textContent = names[name] || name; $('route-sequence').append(item);
    });
    $('targets').replaceChildren();
    targets.forEach((name, index) => {
        const row = document.createElement('li'); row.textContent = names[name] || name;
        for (const [label, fn] of [['↑', () => { if (index) [targets[index - 1], targets[index]] = [targets[index], targets[index - 1]]; }], ['×', () => targets.splice(index, 1)]]) {
            const button = document.createElement('button'); button.textContent = label;
            button.setAttribute('aria-label', `${label === '↑' ? '上移' : '删除'}${names[name] || name}`);
            button.onclick = () => { fn(); sequenceView(); draftChanged(); }; row.append(button);
        }
        $('targets').append(row);
    });
}
for (const [id, label] of Object.entries(names)) {
    const option = document.createElement('option'); option.value = id; option.textContent = label; $('target').append(option);
}
$('add-target').onclick = () => {
    if (targets.length >= 9) { notify('每次最多选择9个站点', true); return; }
    targets.push($('target').value); sequenceView(); draftChanged();
};
for (const id of ['source', 'start']) $(id).addEventListener('change', () => {
    if (id === 'start') {
        targets = targets.map(name => name === 'start_1' || name === 'start_2' ? $('start').value : name);
        if ($('start').value === 'start_2') {
            $('source').value = 'custom';
            notify('启停区2使用自选站点规划，初始车头沿地图−X。');
        }
    }
    sequenceView(); draftChanged();
});
for (const [from, to] of [['route-speed', 'speed-range'], ['speed-range', 'route-speed']]) {
    $(from).addEventListener('input', () => { $(to).value = $(from).value; draftChanged(); });
}
$('plan').onclick = async () => {
    try {
        const args = {source:$('source').value, start:$('start').value, targets:[...targets],
            blocked:$('source').value === 'custom' ? [...document.querySelectorAll('#roads input:checked')].map(input => input.value) : [],
            speed_m_s:number('route-speed', .03, .30)};
        await command('plan', args);
        dirty = true;
        notify('已提交后台规划；可继续查看状态。复杂封路搜索可能需要稍等。');
        updateButtons();
    } catch (error) { notify(error.message, true); }
};
for (const op of ['run', 'save']) $(op).onclick = async () => {
    try { await command(op, {plan_id:planId}); notify(op === 'run' ? '离线闭环已开始，速度和路线版本已冻结。' : '离线版本已保存到 output/workbench/plans。'); }
    catch (error) { notify(error.message, true); }
};
$('move').onclick = async () => {
    try {
        await command('move', {x_m:number('move-x', -500, 500) / 1000,
            y_m:number('move-y', -500, 500) / 1000,
            yaw_rad:number('move-yaw', -180, 180) * Math.PI / 180,
            speed_m_s:number('jog-speed', .01, .15)});
        notify('已开始定量离线动作；完成以闭环到位和稳定停车为准。');
    } catch (error) { notify(error.message, true); }
};
$('reset').onclick = async () => {
    try { await command('reset', {start:$('start').value}); notify('离线场景已重置；这不代表实车配准或软件建零。'); }
    catch (error) { notify(error.message, true); }
};
for (const button of document.querySelectorAll('[data-page]')) button.onclick = () => {
    if (held) stopHeld('切换页面');
    document.querySelectorAll('.page').forEach(page => { page.hidden = page.id !== `page-${button.dataset.page}`; });
    document.querySelectorAll('[data-page]').forEach(item => item.classList.toggle('active', item === button));
    document.querySelector('h1').textContent = {drive:'底盘与导航', mechanism:'机械调试', logs:'运行记录'}[button.dataset.page];
};
function renderPlan(job) {
    const key = `${job.id}:${job.status}`;
    if (key === lastJob) return;
    lastJob = key;
    text('plan-status', {IDLE:'尚未规划', PLANNING:'后台计算中', READY:'离线版本已就绪', FAILED:'规划失败'}[job.status] || job.status);
    if (job.status === 'FAILED') { notify(job.reason, true); planId = ''; dirty = true; }
    if (job.status !== 'READY') return;
    const plan = job.plan;
    planId = plan.id;
    // 其他浏览器生成的路线也能查看；草稿仅在与本页输入一致时标记已应用。
    const params = plan.parameters;
    const same = params.source === $('source').value && params.start === $('start').value && params.speed_m_s === Number($('route-speed').value)
        && (params.source !== 'custom' || (JSON.stringify(params.targets) === JSON.stringify(targets)
        && JSON.stringify(params.blocked) === JSON.stringify([...document.querySelectorAll('#roads input:checked')].map(input => input.value))));
    dirty = !same;
    text('plan-time', `${plan.seconds.toFixed(2)} s`); text('plan-count', plan.legs.length); text('plan-hash', plan.sha256.slice(0, 10));
    text('map-status', `${plan.legs.length} 段 · ${params.speed_m_s.toFixed(2)} m/s`);
    text('draft-state', same ? '当前草稿已生成独立离线版本；实车参数保持原配置。' : '显示核心最新版本；与当前草稿不同，运行前请重新规划。');
    $('leg-list').replaceChildren();
    for (const [index, leg] of plan.legs.entries()) {
        const row = document.createElement('div');
        const name = document.createElement('span'); name.textContent = `${index + 1}. ${legNames[plan.source]?.[index] || leg.name}`;
        const duration = document.createElement('span'); duration.textContent = `${leg.seconds.toFixed(2)} s`;
        row.append(name, duration); $('leg-list').append(row);
    }
    drawSpeed(plan);
}
function render(state) {
    current = state; received = performance.now();
    text('connection', state.available ? '离线核心在线' : '离线核心未连接');
    $('connection').className = `badge ${state.available ? 'good' : 'muted'}`;
    if (!state.available) { invalidate(); return; }
    if ($('notice').textContent.trim().startsWith('正在连接离线核心')) {
        notify('离线核心已连接。先获取控制权；所有操作仅作用于模型，不连接实车。');
    }
    if (token && state.owner.client !== client && performance.now() - tokenSince > 300) { token = ''; clearHeld(); }
    const sim = state.simulation;
    text('mode', modes[sim.mode] || sim.mode);
    text('stop-state', sim.stopped ? '模型轮速已稳定' : '模型运动 / 等待停稳');
    text('pose', `${sim.pose[0].toFixed(3)} / ${sim.pose[1].toFixed(3)}`);
    text('yaw', `航向 ${(Math.atan2(Math.sin(sim.pose[2]), Math.cos(sim.pose[2])) * 180 / Math.PI).toFixed(1)}° · 模拟`);
    text('progress', `${sim.completed} / ${sim.total || '—'}`);
    text('elapsed', `模型运行 ${sim.elapsed_s.toFixed(2)} s`);
    text('rpm', Math.max(...sim.rpm.map(Math.abs)).toFixed(1));
    text('session', sim.session.slice(0, 8)); text('error-xy', `${(sim.position_error_m * 1000).toFixed(2)} mm`);
    text('error-yaw', `${(sim.yaw_error_rad * 180 / Math.PI).toFixed(2)}°`);
    text('fault', sim.fault ? `模型故障 ${sim.fault}` : '0 / 无'); text('odom', `${sim.odom_origin_ms} ms · 连续`);
    text('owner', state.owner.client === client ? '本页持有控制权' : state.owner.client ? '其他浏览器正在控制' : '当前无人控制');
    if (!$('roads').children.length) {
        for (const [road, a, b] of state.field.edges) {
            const label = document.createElement('label'); const input = document.createElement('input');
            input.type = 'checkbox'; input.value = road; input.onchange = draftChanged;
            label.append(input, document.createTextNode(`${names[a] || a} ↔ ${names[b] || b}`)); $('roads').append(label);
        }
    }
    renderPlan(state.planning); drawMap(state); updateButtons();
    const eventKey = JSON.stringify(state.events);
    if (eventKey !== lastEvents) {
        lastEvents = eventKey; $('events').replaceChildren();
        for (const event of state.events) {
            const row = document.createElement('div'); const time = document.createElement('time');
            const name = document.createElement('b'); const detail = document.createElement('p');
            time.textContent = new Date(event.at * 1000).toLocaleTimeString('zh-CN', {hour12:false});
            name.textContent = event.name; detail.textContent = event.detail;
            row.append(time, name, detail); $('events').append(row);
        }
    }
}
function invalidate() {
    if (held) stop('状态流中断');
    token = ''; current = null;
    for (const [id, value] of Object.entries({mode:'状态已过期', pose:'— / —', yaw:'航向未知', rpm:'—', 'stop-state':'无法确认模型停车', 'error-xy':'—', 'error-yaw':'—', fault:'未知', odom:'—'})) text(id, value);
    drawMap({available:false}); updateButtons();
}
function connect() {
    const socket = new WebSocket(`${location.protocol === 'https:' ? 'wss' : 'ws'}://${location.host}/api/workbench/stream`);
    socket.onmessage = event => { try { render(JSON.parse(event.data)); } catch { notify('状态数据异常，等待新帧', true); invalidate(); } };
    socket.onclose = () => { invalidate(); text('connection', '连接中断'); setTimeout(connect, 1500); };
    socket.onerror = () => socket.close();
}
setInterval(pulse, 100);
setInterval(() => {
    if (received && performance.now() - received > 700) invalidate();
    if (token && online() && !held && !document.hidden) command('renew').catch(error => { token = ''; updateButtons(); notify(error.message, true); });
}, 500);
sequenceView(); updateButtons(); connect();
