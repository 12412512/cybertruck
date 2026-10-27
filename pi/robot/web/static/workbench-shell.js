// 共用状态只作展示。复用机械页GET结果，不新增设备连接或动作入口。
const health = document.getElementById('workspace-health');
health.innerHTML = `<div class="card-head"><h2>连接与诊断</h2><button id="workspace-export">导出诊断摘要</button></div>
 <div class="workspace-health-grid"><div><small>实车设备</small><strong id="health-device">等待状态</strong></div><div><small>底盘 / 机构故障</small><strong id="health-fault">未知</strong></div><div><small>停车状态</small><strong id="health-stop">未知</strong></div><div><small>固件</small><strong id="health-firmware">—</strong></div></div>
 <p class="hint">诊断摘要只包含读取到的状态；不保存控制令牌，不发送设备命令。</p>
 <details><summary>高级诊断</summary><p class="hint">供通信、反馈和路线问题排查。</p><a href="/diagnostics" target="_blank" rel="noopener">打开只读诊断页</a></details>`;
const badge = document.createElement('span');
badge.id = 'workspace-device-badge';
badge.className = 'badge muted';
badge.textContent = '实车状态待读取';
document.querySelector('.header-actions').prepend(badge);
let state = null,
    received = 0;
const set = (id, value) => {
    document.getElementById(id).textContent = value;
};
window.addEventListener('gc-device-state', event => {
    state = event.detail;
    received = performance.now();
    render();
});

function render() {
    const connected = state?.connected === true && performance.now() - received < 1500;
    badge.textContent = connected ? (state.base?.fault || state.mechanism?.fault ? '实车故障锁存' : '实车已连接') :
        '实车未连接';
    badge.className = `badge ${connected?(state.base?.fault||state.mechanism?.fault?'warn':'good'):'muted'}`;
    set('health-device', connected ? '已连接' : '未连接 / 反馈中断');
    set('health-fault', connected ? `${state.base?.fault??'—'} / ${state.mechanism?.fault??'—'}` : '未知');
    set('health-stop', connected ? (state.base?.stopped ? '已确认' : '未确认') : '未知');
    set('health-firmware', state?.firmware || '—');
    document.querySelector('.sidebar-note .status-dot')?.classList.toggle('online', connected);
}
document.getElementById('workspace-export').onclick = () => {
    const summary = {
        exported_at: new Date().toISOString(),
        page: document.body.dataset.page,
        received_age_ms: received ? Math.round(performance.now() - received) : null,
        device: state,
        scope: 'readonly_snapshot',
        device_commands: 0
    };
    const url = URL.createObjectURL(new Blob([JSON.stringify(summary, null, 2)], {
        type: 'application/json'
    }));
    const link = document.createElement('a');
    link.href = url;
    link.download = `gc-diagnostic-${Date.now()}.json`;
    link.click();
    setTimeout(() => URL.revokeObjectURL(url), 1000);
};
setInterval(render, 500);
