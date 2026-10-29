// 校准分只读准备与明确提交；写入默认关闭，不输出舵机或底盘命令。
const $ = id => document.getElementById(id);

export function calibrationLines(value) {
    if (value?.domain !== 'physical_lidar' || value.authorizes_motion !== false) throw new Error('校准状态来源不符');
    const caps = value.calibration_capabilities, job = value.calibration;
    if (!caps?.prepare) return ['设备核心尚未部署校准准备接口', '当前仍可查看通信与历史核对记录；没有启动校准'];
    const mode = caps.write ? (caps.board_ready ? '写入入口开放，仍须明确确认及版本匹配' : '写入被停车/故障条件拦截') : '仅开放只读准备 · 校准写入关闭';
    const phase = job?.status === 'CANCELLED' && !job.write_issued ? '只读准备已取消（未校准）' :
        job?.status === 'FAILED' && !job.write_issued ? '只读准备已结束（未校准）' : job?.status || '尚未准备';
    return [`厂家两点校准：${phase} · ${mode}`,
        job?.backup_version ? `新读取备份 ${job.backup_version.slice(0,12)} · ${job.backup_saved ? '已归档' : '未归档，禁止提交'} · ${job.write_issued ? '09已发送，未核对前不宣称成功' : '未发送09'}；${(job.errors || []).join('；') || '厂家校准通过后仍需地图与独立点验证'}` : '只读准备仅发送HELLO/读取保存状态/保活，不扫描、不写参数'];
}

export function initCalibration() {
    let gateway = false, last = null, pending = false, commitRequest = null, jobId = '';
    function render(value) {
        last = value;
        gateway = value.gateway_calibration === true;
        const lines = calibrationLines(value), caps = value.calibration_capabilities || {}, job = value.calibration;
        $('lidar-cal-state').textContent = lines[0]; $('lidar-cal-backup').textContent = lines[1];
        $('lidar-cal-gateway').textContent = gateway ? '网关已支持分步校准；准备不解除CAN故障，提交仍有独立保护。' : '当前网关只读：可显示准备结果；操作按钮需网关重载后使用。';
        if (jobId !== (job?.id || '')) { jobId = job?.id || ''; commitRequest = null; $('lidar-cal-save').checked = false; }
        $('lidar-cal-prepare').disabled = pending || !gateway || !caps.prepare || !value.connected || job?.live || !$('lidar-cal-fixed').checked;
        $('lidar-cal-commit').disabled = pending || !gateway || !caps.write || !caps.board_ready || job?.status !== 'PREPARED' || !job.backup_saved || !$('lidar-cal-fixed').checked || !$('lidar-cal-save').checked;
        $('lidar-cal-cancel').disabled = pending || !gateway || !job?.live;
    }
    async function post(path, body) {
        pending = true; if (last) render(last);
        try {
            const response = await fetch('/api/lidar/calibration/'+path, {method:'POST',
                headers:{'Content-Type':'application/json', 'X-Workbench':'1'}, body:JSON.stringify(body)});
            const value = await response.json();
            if (!response.ok) throw new Error(value.detail || '请求被拒绝');
            $('lidar-cal-message').textContent = `请求已接受：${value.id}；状态由唯一雷达核心回报`;
        } catch (error) { $('lidar-cal-message').textContent = error.message; }
        finally { pending = false; if (last) render(last); }
    }
    $('lidar-cal-fixed').onchange = $('lidar-cal-save').onchange = () => { if (last) render(last); };
    $('lidar-cal-prepare').onclick = () => post('prepare', {request_id:crypto.randomUUID(),
        zone:last.diagnostic_zone, placement_id:crypto.randomUUID(), manual_fixed_deployed:true});
    $('lidar-cal-commit').onclick = () => {
        const job = last.calibration;
        commitRequest ||= {request_id:crypto.randomUUID(), calibration_id:job.id,
            backup_version:job.backup_version, placement_id:job.placement_id, confirm_save:true};
        post('commit', commitRequest);
    };
    $('lidar-cal-cancel').onclick = () => post(last.calibration.id+'/cancel', {});
    return value => {
        try { render(value); }
        catch (error) {
            last = null; $('lidar-cal-state').textContent = error.message;
            $('lidar-cal-backup').textContent = '';
            for (const id of ['lidar-cal-prepare','lidar-cal-commit','lidar-cal-cancel']) $(id).disabled = true;
        }
    };
}
