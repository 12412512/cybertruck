// 网页仅接收状态；掉线或失去核心快照时立即撤销在线显示。
const set = (id, value) => { document.getElementById(id).textContent = value ?? '—'; };
let lastMessage = 0;
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
