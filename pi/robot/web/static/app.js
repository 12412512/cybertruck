// 网页仅接收状态；掉线或失去核心快照时立即撤销在线显示。
import {renderTracking} from './tracking.js';
const set = (id, value) => { document.getElementById(id).textContent = value ?? '—'; };
const ageText = value => Number.isFinite(value) ? Math.floor(value) : '—';
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
    renderTracking(fresh && state.mode === 'sim' ? state.tracking : null);
    document.getElementById('odom-panel').hidden = fresh && state.tracking?.domain === 'simulation';
    const board = fresh && state.connected ? state.board : null;
    const notice = document.getElementById('source');
    const hasFault = board && (state.mode === 'observer'
        ? board.state === 5 || board.bus_error !== 0 : board.fault_bits !== 0);
    notice.className = `notice ${!fresh || hasFault ? 'stale' : state.mode === 'sim' ? 'sim' : ''}`;
    set('source', !fresh ? '控制核心状态过期或未启动' : state.mode === 'sim'
        ? '模拟数据 · 未连接实车，不能作为实机验收' : state.mode === 'offline'
        ? '离线模式 · 未打开设备' : state.mode === 'observer'
        ? '旧固件实车观察 · 经现有串口所有者读取，GC新协议尚未验收' : '实机只读模式 · 动作接口未开放');
    set('connection', board ? '只读在线' : '状态未知');
    set('message', state.message);
    set('core', fresh ? '运行中' : '状态未知');
    set('age', `核心 / 设备状态时效：${ageText(state.core_age_ms)} / ${ageText(state.state_age_ms)} 毫秒`);
    set('firmware', fresh ? state.firmware || '—' : '—');
    set('uptime', `运行时间：${board ? (board.uptime_ms / 1000).toFixed(1) : '—'} 秒`);
    set('fault', board ? state.mode === 'observer'
        ? `${board.state_name ?? board.state} / 总线 ${board.bus_error} / 原因 ${board.reason}`
        : board.fault_bits === 0 ? '无上报故障' : `故障 ${board.fault_bits}` : '未知');
    set('boot', board ? state.boot_id : '—');
    set('session', board ? state.session_id : '—');
    set('frames', fresh && state.mode !== 'observer' ? `正确 ${state.rx_good ?? 0} / 错帧 ${state.rx_bad ?? 0}` : '—');
    set('heart', board?.heartbeats);
    set('key', board && Number.isInteger(board.flags) ? board.flags & 8 ? '按下（只读）' : '释放' : '未知');
    const observed = board && state.mode === 'observer';
    set('can', observed ? `${board.can_errors} / HAL ${board.can_last_error} / 接收序号 ${board.can_rx_sequence}` : '未接入');
    const errorEvent = observed ? state.diagnostics?.can_error_event : null;
    const hex = value => Number.isInteger(value) ? `0x${value.toString(16).toUpperCase()}` : '—';
    set('can-event', errorEvent?.valid ? `序号 ${errorEvent.sequence}，板时 ${errorEvent.tick_ms} ms，HAL ${hex(errorEvent.hal_error)}，ESR ${hex(errorEvent.esr)}，发送ID ${hex(errorEvent.tx_id)}` : '无有效快照');
    const canReport = observed ? state.diagnostics?.can_report : null;
    set('can-decoded', canReport ? `当前 ESR ${hex(canReport.current.esr)}；冻结记录：${canReport.latched?.decoded_esr.lec_name ?? '未知'}，${canReport.latched?.is_stop_frame ? '停止帧发送期间' : '帧用途待核对'}；软件锁存保留` : '未接入');
    set('legacy-uart', observed ? `${state.diagnostics?.uart_bad_crc ?? '—'} / ${state.diagnostics?.uart_bad_line ?? '—'}` : '未接入');
    set('legacy-owner', observed ? `${state.diagnostics?.owner_busy ? '旧服务有其他控制者' : '未报告其他控制者'}；本页不申请控制权` : '未接入');
    set('timing', observed ? `HTTP往返 ${ageText(state.diagnostics?.rtt_ms)} ms，本机缓存 ${ageText(state.observation?.cache_age_ms)} ms；MCU接收时刻尚未导出` : '未接入');
    const imu = observed ? state.imu : null;
    set('imu', imu?.angle_reported ? `${imu.yaw_deg}°（旧服务报告，原设备方向）` : '未知或源报告过期');
    set('gyro', imu?.gyro_reported ? `${imu.gz_dps} °/s（旧服务报告，原始Z轴）` : '未知或源报告过期');
    set('stopped', observed ? `原服务报告${state.diagnostics?.source_stopped ? '已停' : '停止未确认'}；本工程实物停稳证据未建立` : '未接入');
    const wheels = document.getElementById('wheels');
    wheels.replaceChildren();
    for (const wheel of observed ? state.wheels ?? [] : []) {
        const item = document.createElement('li');
        item.textContent = `地址${wheel.address}：${wheel.speed_reported ? wheel.rpm + ' RPM' : '未知'}；位置 ${wheel.position_reported ? wheel.position + '（原始计数）' : '未知'}；状态 ${wheel.status_reported ? wheel.status : '未知'}（旧服务报告）`;
        wheels.appendChild(item);
    }
    const task = fresh && state.task?.domain === 'simulation' ? state.task : null;
    set('task-code', task ? task.code ?? '等待模拟扫码' : '未接入');
    set('task-stage', task ? `${taskStates[task.status] ?? task.status} / ${stageName(task.stage)}` : '未接入');
    set('task-counts', task ? `${task.simulation_counts.grabs} / ${task.simulation_counts.places}（模拟）` : '未接入');
    set('task-tray', task ? [1, 2, 3].map(slot => task.tray[slot] ?? '空').join(' / ') : '未接入');
    set('task-mapping', task ? Object.entries(task.first_layer_by_color).map(([color, ring]) => `颜色${color}→${ring}号`).join('，') || '待确认' : '未接入');
    set('task-progress', task ? `${task.confirmed_steps} / ${task.planned_steps}，账本版本 ${task.revision}` : '未接入');
    set('task-reason', task?.reason || '—');
    const odom = fresh && state.mode === 'sim' && state.odometry?.domain === 'simulation' ? state.odometry : null;
    set('odom-status', odom ? `${odom.status} / ${odom.pose.valid ? '模型有效' : '等待或失效'} / 故障 ${odom.pose.fault}` : '未运行');
    set('odom-pose', odom?.pose.valid ? `${odom.pose.x_m.toFixed(4)} m / ${odom.pose.y_m.toFixed(4)} m / ${(odom.pose.yaw_rad * 180 / Math.PI).toFixed(2)}°（模拟）` : '未知');
    set('odom-error', Number.isFinite(odom?.error_m) ? `${(odom.error_m * 1000).toFixed(3)} mm（合成模型对照，不代表实车精度）` : '未知');
    set('odom-timing', odom ? `${odom.pose.tick_ms} ms / ${odom.pose.steps} / ${odom.pose.waits}` : '未接入');
    set('odom-source', odom ? `${odom.frames} / ${odom.calibration}` : '未接入');
    const trace = document.getElementById('odom-trace');
    trace.replaceChildren();
    if (odom?.pose.valid && odom.history.length > 1) {
        const points = odom.history;
        const xs = points.map(point => point.x_m), ys = points.map(point => point.y_m);
        const left = Math.min(0, ...xs), bottom = Math.min(0, ...ys);
        const scale = Math.min(560 / Math.max(.2, Math.max(...xs) - left), 200 / Math.max(.2, Math.max(...ys) - bottom));
        const line = document.createElementNS('http://www.w3.org/2000/svg', 'polyline');
        line.setAttribute('points', points.map(point => `${20 + (point.x_m - left) * scale},${220 - (point.y_m - bottom) * scale}`).join(' '));
        line.setAttribute('fill', 'none');
        line.setAttribute('stroke', '#3e729f');
        line.setAttribute('stroke-width', '3');
        trace.appendChild(line);
    }
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
