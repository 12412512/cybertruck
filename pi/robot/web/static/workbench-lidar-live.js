// 实机雷达仅GET通信状态，不启动扫描，不修改当前模拟障碍和路线。
const $ = id => document.getElementById(id);

export function liveLines(value) {
    if (value?.domain !== 'physical_lidar' || value.authorizes_motion !== false ||
        value.scan_allowed !== false || value.real_map_registered !== false) {
        return ['实机雷达未连接', '等待独立雷达只读核心', '尚未读取保存配置', '未启动扫描；没有地图配准'];
    }
    const info = value.information, saved = value.persisted;
    const state = value.connected ? (value.scan ? `实机雷达在线 · 静扫${value.scan.status} · 不驱动车体` : '实机雷达基础通信在线 · 扫描未开放') : `实机雷达${value.status === 'FAULT' ? '通信异常' : '未连接'}`;
    const details = info ? `固件${info.firmware} / 协议${info.protocol} / 节点${value.node} / 当前厂家区${info.zone || '未选择'} / ${info.running ? '设备报告正在扫描（非本页发起）' : '设备未扫描'} / CRC等错误${value.decode_bad} / 隔离扩展${value.decode_unknown || 0}` : '等待HELLO和匹配应答；不猜测设备版本或区号';
    const pose = p => p ? `(${p.x_mm}, ${p.y_mm})mm / ${p.heading_deg}°` : '未知';
    const stored = saved ? `历史存储v${saved.storage_version} / 记录${saved.record_sequence}；区1 ${pose(saved.zone1_pose)}；区2 ${pose(saved.zone2_pose)}；历史格号${saved.grid_ids.join('、') || '无'}（不作当前障碍）` : '尚未读取保存配置';
    return [state, details, stored, value.reason || '厂家坐标未配准，保存位姿不等于当前车位'];
}

export function scanLine(value) {
    const scan = value?.scan;
    if (value?.domain !== 'physical_lidar' || scan?.domain !== 'physical_lidar_scan') return '本次尚未进行实机扫描';
    return `实机区${scan.zone}静扫：${scan.status} · 完整点云报文${scan.cloud_frames} / 结果${scan.result_frames} / 配对${scan.matched_frames} · 最近输出测点${(scan.raw_points || []).length} · ${scan.stop_confirmed ? '扫描STOP双确认' : '停止尚未双确认'} · ${scan.live ? '采集中' : '已结束的历史样本'}；${(scan.errors || []).join('；') || '厂家坐标未配准，不能用作空场或可行道路证明'}`;
}

export function initLiveLidar(renderCalibration = () => {}) {
    let pending = false;
    async function poll() {
        if (pending) return;
        pending = true;
        try {
            const response = await fetch('/api/lidar/state', {cache:'no-store', signal:AbortSignal.timeout(1800)});
            if (!response.ok) throw new Error('只读雷达代理未就绪');
            const value = await response.json(), lines = liveLines(value);
            renderCalibration(value);
            for (const [index,id] of ['lidar-live-state','lidar-live-details','lidar-live-saved','lidar-live-reason'].entries()) $(id).textContent = lines[index];
            $('lidar-live-state').classList.toggle('error', value.status === 'FAULT');
            $('lidar-live-scan').textContent = scanLine(value);
            const svg = $('lidar-live-cloud'); svg.replaceChildren();
            if (value.domain === 'physical_lidar' && value.scan?.domain === 'physical_lidar_scan') {
                for (const point of (value.scan.raw_points || []).slice(0,120)) {
                    if (!point.slice(0,2).every(n => Number.isFinite(n) && Math.abs(n) < 10000)) continue;
                    const circle = document.createElementNS('http://www.w3.org/2000/svg','circle');
                    circle.setAttribute('cx',point[0]); circle.setAttribute('cy',-point[1]);
                    circle.setAttribute('r',16); circle.setAttribute('fill','#147c88'); svg.append(circle);
                }
            }
        } catch {
            renderCalibration(null);
            $('lidar-live-state').textContent = '实机雷达未连接 · 只读代理暂不可达';
            $('lidar-live-details').textContent = '不保留旧在线状态；本页没有打开串口';
            $('lidar-live-saved').textContent = '历史配置待重新读取';
            $('lidar-live-reason').textContent = '未启动扫描，模拟路线不受影响';
            $('lidar-live-scan').textContent = '实机状态不可达，旧样本不作为当前扫描';
            $('lidar-live-cloud').replaceChildren();
        } finally { pending = false; }
    }
    poll();
    const timer = setInterval(poll, 1500);
    window.addEventListener('beforeunload', () => clearInterval(timer), {once:true});
}
