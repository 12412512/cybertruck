// 网页仅接收状态；掉线或失去核心快照时立即撤销在线显示。
const set = (id, value) => { document.getElementById(id).textContent = value ?? '—'; };
let lastMessage = 0;
const taskStates = {READY: '待启动', RUNNING: '运行中', RECONCILE_REQUIRED: '等待对账',
    COMPLETE: '已完成', ABORTED: '已终止'};
function stageName(stage) {
    const names = {START_SCAN: '启动扫描', QR_NAV: '前往扫码', SCAN_CODE: '锁存任务码', RETURN: '返回启停区'};
    if (!stage) return '已结束';
    if (names[stage]) return names[stage];
    const steps = {RAW_NAV: '前往原料区', RAW_PICK: '取原料并存车', ROUGH_NAV: '前往粗加工区',
        ROUGH_PLACE: '粗加工放置', ROUGH_PICK: '粗加工取回存车', STAGE_NAV: '前往暂存区',
        STAGE_PLACE: stage.startsWith('B1_') ? '第一层放置' : '同色码垛'};
    return `${stage.startsWith('B1_') ? '第一批' : '第二批'} · ${steps[stage.slice(3)] ?? stage}`;
}
function render(state) {
    const fresh = state.core_fresh === true;
    const board = fresh && state.connected ? state.board : null;
    const notice = document.getElementById('source');
    notice.className = `notice ${!fresh ? 'stale' : state.mode === 'sim' ? 'sim' : ''}`;
    set('source', !fresh ? '控制核心状态过期或未启动' : state.mode === 'sim'
        ? '模拟数据 · 未连接实车，不能作为实机验收' : state.mode === 'offline'
        ? '离线模式 · 未打开设备' : '实机只读模式 · 动作接口未开放');
    set('connection', board ? '只读在线' : '状态未知');
    set('message', state.message);
    set('core', fresh ? '运行中' : '状态未知');
    set('age', `核心 / 设备状态时效：${state.core_age_ms ?? '—'} / ${state.state_age_ms ?? '—'} 毫秒`);
    set('firmware', fresh ? state.firmware || '—' : '—');
    set('uptime', `运行时间：${board ? (board.uptime_ms / 1000).toFixed(1) : '—'} 秒`);
    set('fault', board ? board.fault_bits === 0 ? '无上报故障' : `故障 ${board.fault_bits}` : '未知');
    set('boot', board ? state.boot_id : '—');
    set('session', board ? state.session_id : '—');
    set('frames', fresh ? `正确 ${state.rx_good ?? 0} / 错帧 ${state.rx_bad ?? 0}` : '—');
    set('heart', board?.heartbeats);
    set('key', board ? board.flags & 8 ? '按下（只读）' : '释放' : '未知');
    const task = fresh && state.task?.domain === 'simulation' ? state.task : null;
    set('task-code', task ? task.code ?? '等待模拟扫码' : '未接入');
    set('task-stage', task ? `${taskStates[task.status] ?? task.status} / ${stageName(task.stage)}` : '未接入');
    set('task-counts', task ? `${task.simulation_counts.grabs} / ${task.simulation_counts.places}（模拟）` : '未接入');
    set('task-tray', task ? [1, 2, 3].map(slot => task.tray[slot] ?? '空').join(' / ') : '未接入');
    set('task-mapping', task ? Object.entries(task.first_layer_by_color).map(([color, ring]) => `颜色${color}→${ring}号`).join('，') || '待确认' : '未接入');
    set('task-progress', task ? `${task.confirmed_steps} / ${task.planned_steps}，账本版本 ${task.revision}` : '未接入');
    set('task-reason', task?.reason || '—');
    const events = document.getElementById('events');
    events.replaceChildren();
    for (const message of state.events?.length ? state.events.slice(-10) : ['暂无事件']) {
        const item = document.createElement('li');
        item.textContent = message;
        events.appendChild(item);
    }
}
function stale() { render({message: '网页连接中断，等待重新连接'}); }
function connect() {
    const socket = new WebSocket(`${location.protocol === 'https:' ? 'wss' : 'ws'}://${location.host}/ws`);
    socket.onmessage = event => {
        try { render(JSON.parse(event.data)); lastMessage = performance.now(); }
        catch { stale(); }
    };
    socket.onclose = () => { stale(); setTimeout(connect, 1000); };
    socket.onerror = () => socket.close();
}
setInterval(() => { if (performance.now() - lastMessage > 700) stale(); }, 100);
connect();
