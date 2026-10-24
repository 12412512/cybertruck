// 只读模拟结果，不提供动作入口；核心过期时由调用者清除结果。
const set = (id, text) => { document.getElementById(id).textContent = text; };
const names = {IDLE: '等待', ACCEPTED: '已接受', RUNNING: '跟踪中', SETTLING: '等待停稳',
    COMPLETE: '已确认到位', FAILED: '失败', CANCELLED: '已取消'};
const blockers = ['参考未结束', '几何误差或输出未收敛', '零速发送未确认',
    '缺少发送后的新反馈', '速度未静止', '编码器漂移', '稳定窗口不足'];
const seconds = value => Number.isFinite(value) ? `${(value / 1000).toFixed(3)} s` : '—';

function draw(reference, actual) {
    const svg = document.getElementById('track-trace');
    svg.replaceChildren();
    const points = [...reference, ...actual];
    if (!points.length) return;
    const xs = points.map(point => point.x_m), ys = points.map(point => point.y_m);
    const left = Math.min(0, ...xs), bottom = Math.min(0, ...ys);
    const scale = Math.min(560 / Math.max(.2, Math.max(...xs) - left),
        200 / Math.max(.2, Math.max(...ys) - bottom));
    for (const [path, color, width] of [[reference, '#3e729f', 4], [actual, '#ce6b28', 2]]) {
        const line = document.createElementNS('http://www.w3.org/2000/svg', 'polyline');
        line.setAttribute('points', path.map(p => `${20 + (p.x_m - left) * scale},${220 - (p.y_m - bottom) * scale}`).join(' '));
        line.setAttribute('fill', 'none');
        line.setAttribute('stroke', color);
        line.setAttribute('stroke-width', width);
        svg.appendChild(line);
    }
}

export function renderTracking(state) {
    if (state?.domain !== 'simulation') state = null;
    const out = state?.output, pose = state?.pose;
    set('track-state', out ? `${state.kind} / #${out.id} / ${names[state.state] ?? state.state} / 故障 ${out.fault}` : '未运行');
    set('track-pose', pose?.valid ? `${pose.x_m.toFixed(4)} m / ${pose.y_m.toFixed(4)} m / ${(pose.yaw_rad * 180 / Math.PI).toFixed(2)}°` : '未接入');
    set('track-error', out ? `${(out.position_error_m * 1000).toFixed(2)} mm / ${(out.yaw_error_rad * 180 / Math.PI).toFixed(2)}°（模拟）` : '未接入');
    set('track-times', out ? `${seconds(state.reference_end_ms)} / ${out.state === 4 ? seconds(state.elapsed_ms) : '尚未确认'}` : '未接入');
    set('track-proof', out ? `${out.zero_ack_ms ? `板时 ${out.zero_ack_ms} ms（模拟）` : '未确认'} / ${out.stable_elapsed_ms} ms` : '未接入');
    set('track-blockers', out ? blockers.filter((_, index) => out.blockers & (1 << index)).join('；') ||
        (out.state === 4 ? '所有到位条件满足（仅模拟）' : '等待运行或查看故障') : '未接入');
    set('track-targets', out ? out.rpm.join(' / ') : '未接入');
    set('track-speeds', state ? state.plant.rpm.map(value => value.toFixed(2)).join(' / ') : '未接入');
    set('track-history', state ? state.transitions.slice(-8).map(item => `${seconds(item.at_ms)} ${names[item.state] ?? item.state}`).join(' → ') : '未接入');
    draw(state?.reference_trace ?? [], state?.trace ?? []);
}
