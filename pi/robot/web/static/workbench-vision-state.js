// 画面坐标、标定输入和来源标签共用纯校验；不包含设备请求。
export function imagePoint(x, y, rect, size) {
    if (!rect.width || !rect.height) throw Error('画面尺寸无效');
    return [Math.max(0, Math.min(size[0] - 1, Math.round((x - rect.left) * size[0] / rect.width))),
        Math.max(0, Math.min(size[1] - 1, Math.round((y - rect.top) * size[1] / rect.height)))
    ];
}
export function imageRect(a, b) {
    const value = [Math.min(a[0], b[0]), Math.min(a[1], b[1]), Math.max(a[0], b[0]), Math.max(a[1], b[1])];
    if (value[2] - value[0] < 2 || value[3] - value[1] < 2) throw Error('请拖出至少2×2像素的区域');
    return value;
}
export function parseRect(text, size) {
    const parts = text.split(/[,，\s]+/).filter(Boolean),
        values = parts.map(Number);
    if (parts.length !== 4 || values.some(v => !Number.isInteger(v)) ||
        values[0] < 0 || values[1] < 0 || values[2] > size[0] || values[3] > size[1] ||
        values[2] <= values[0] || values[3] <= values[1]) throw Error('区域格式为 x1,y1,x2,y2，须位于原图内');
    return values;
}
export function calibrationPoints(rows) {
    if (!rows.length) return [];
    const points = rows.map(p => {
        for (const k of ['u', 'v', 'x_mm', 'y_mm'])
            if (p[k] === null || String(p[k]).trim() === '' || !Number.isFinite(Number(p[k])))
            throw Error('请补齐标定点的像素与毫米坐标');
        return {
            u: Number(p.u),
            v: Number(p.v),
            x_mm: Number(p.x_mm),
            y_mm: Number(p.y_mm),
            check: p.check === true
        };
    });
    if (points.filter(p => !p.check).length < 4 || !points.some(p => p.check)) throw Error(
        '至少4个拟合点和1个独立检查点；建议6点以上');
    return points;
}
export const sourceName = result => ({
    camera_live: '实时相机',
    historical_replay: '旧样本回放',
    saved_sample: '已保存样本'
} [result?.domain] || '来源未知');
export function liveAge(result, received, now, roundTrip) {
    if (result?.domain !== 'camera_live') return null;
    if (!Number.isFinite(result.age_ms) || result.age_ms < 0) return Infinity;
    // 网关处理时间已经包含在age_ms里；仅补浏览器与网关间剩余传输时间。
    const proxy = result.proxy_elapsed_ms;
    const extra = Number.isFinite(proxy) && proxy >= 0 && proxy <= roundTrip ? roundTrip-proxy : roundTrip;
    return result.age_ms + Math.max(0, now - received) + Math.max(0, extra);
}
export function cameraHealth(live, role) {
    const camera = live?.cameras?.[role];
    if (!camera?.available) return {
        ok: false,
        text: camera?.reason || live?.reason || '相机未连接'
    };
    return {
        ok: true,
        text: `已连接 · 帧龄 ${camera.age_ms}ms${Number.isFinite(camera.measured_fps)?` · 采集 ${camera.measured_fps} FPS`:''}`
    };
}

export function previewDelay(fps, elapsed) {
    const limit = [6,12,20].includes(fps) ? fps : 12;
    return Math.max(0, 1000/limit - Math.max(0, Number.isFinite(elapsed) ? elapsed : 0));
}
