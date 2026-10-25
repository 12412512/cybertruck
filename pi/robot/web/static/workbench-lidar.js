// 雷达回放与实时设备隔离；外部报文未配准时仅显示，不进入自动规划。
const $ = id => document.getElementById(id);
let current = null, options, upload = '';

export function scanMap(state) {
    const scan = state?.lidar;
    if (!scan || scan.domain !== 'simulation' || scan.live !== false ||
        scan.synthetic !== true || scan.real_map_registered !== false ||
        scan.start !== state.draft?.start || !scan.simulation_registered) return null;
    return scan;
}

export function initLidar(value) {
    options = value;
    $('lidar-file').onchange = async () => {
        upload = '';
        try {
            const file = $('lidar-file').files[0];
            if (!file) return;
            if (file.size > 160000) throw new Error('报文文件超过160KB');
            const text = await file.text();
            const raw = file.name.endsWith('.json') ? JSON.parse(text).wire_hex : text.trim();
            if (typeof raw !== 'string' || !raw.length || raw.length > 131072 ||
                raw.length % 2 || !/^[a-fA-F0-9]+$/.test(raw)) throw new Error('需要wire_hex JSON或纯十六进制文件');
            upload = raw;
            options.notify('已载入外部报文；没有真实标定，只可回放诊断。');
        } catch (error) { options.notify(error.message,true); }
    };
    $('lidar-replay').onclick = async () => {
        try {
            const result = await options.command('lidar',{action:'replay',case:$('lidar-case').value,
                wire_hex:upload,parameters:options.settings()});
            if (result.usable && $('lidar-auto-plan').checked) await plan(result.scan_id);
            else options.notify(result.usable ? '扫描回放完成，可以按当前草稿自动规划。' : `回放仅供诊断：${result.reason}`,!result.usable);
        } catch (error) { options.notify(error.message,true); }
    };
    $('lidar-plan').onclick = () => plan(current?.id).catch(error => options.notify(error.message,true));
    $('lidar-clear').onclick = async () => {
        try {
            const result = await options.command('lidar',{action:'clear'});
            options.acceptDraft(result);
            upload = ''; $('lidar-file').value = '';
            options.notify('扫描已清空，旧雷达计划需重新回放及规划；没有启动动作。');
        } catch (error) { options.notify(error.message,true); }
    };
}

async function plan(scan_id) {
    const result = await options.command('lidar',{action:'plan',scan_id,parameters:options.settings()});
    options.acceptDraft(result);
    options.notify('已按扫描圆障碍与未知区域提交自动规划；规划成功后仍需单独点击执行链联调。');
}

export function renderLidar(state, enabled, busy) {
    current = state.lidar;
    const scan = current || {};
    $('lidar-state').textContent = `${scan.status || '等待核心'} · ${scan.synthetic ? '合成报文' : '未配准诊断'} · 实车未连接`;
    $('lidar-details').textContent = scan.id ? `区${scan.start === 'start_1' ? 1 : 2} / 厂家区${scan.vendor_zone} · 完整点云${scan.cloud_frames}帧 / 结果${scan.result_frames}帧 · CRC等错误${scan.decode_bad} · ${scan.stop_confirmed ? '扫描STOP已确认' : 'STOP未确认'} · ${scan.calibration_version || '无标定'}` : '本轮使用厂家v6格式的合成夹具；不连接雷达、树莓派或串口。';
    $('lidar-reason').textContent = scan.reason || (scan.usable ? '模拟标定及覆盖有效；零障碍只有完整覆盖时才可作为空场。黑圈按50mm名义直径，另计位置不确定度。' : '历史/未知观测不能成为当前路线依据。');
    $('lidar-reason').classList.toggle('error',scan.status === 'REJECTED');
    const roads = Object.values(scan.road_states || {});
    $('lidar-roads').textContent = roads.length ? `模拟道路：已观测${roads.filter(v=>v==='OBSERVED').length} / 障碍邻域${roads.filter(v=>v==='OBSTACLE').length} / 未知${roads.filter(v=>v==='UNKNOWN').length}；扫描保持绑定，调速度不会丢失障碍。改区前请清空回放。` : '未知区域不默认开放；厂家栅格编号不直接等同地图道路。';
    const body = $('lidar-objects'); body.replaceChildren();
    for (const candidate of scan.candidates || []) {
        const line = document.createElement('li');
        line.textContent = `栅格${candidate.grid_id} · ${candidate.frames}帧 · 最低置信度${candidate.confidence_percent}% · ${candidate.accepted ? '稳定候选' : '未知/低质量，禁止规划'}`;
        body.append(line);
    }
    for (const id of ['lidar-replay','lidar-clear','lidar-case','lidar-file','lidar-auto-plan']) $(id).disabled = !enabled || busy;
    $('lidar-plan').disabled = !enabled || busy || !scan.usable;
    const svg = $('lidar-cloud'); svg.replaceChildren();
    for (const [x,y] of scan.raw_points || []) {
        const circle = document.createElementNS('http://www.w3.org/2000/svg','circle');
        circle.setAttribute('cx',x); circle.setAttribute('cy',-y); circle.setAttribute('r',24);
        circle.setAttribute('fill','#087f8c'); svg.append(circle);
    }
}
