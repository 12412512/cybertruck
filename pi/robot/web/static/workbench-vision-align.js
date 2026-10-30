import {liveAge} from './workbench-vision-state.js';

// 独立只读对位辅助；不持有设备token，不访问设备命令接口。
const el = id => document.getElementById(id);
const canvas = el('view'), ctx = canvas.getContext('2d');
let reference, refItem, last, paused = true, busy = false, halted = false;
let timer, controller, lastError = '';

function target(value, color) {
    const items = (value?.items || []).filter(item => item.color_id === color &&
        item.clipped === false && item.shape_consistent === true &&
        Array.isArray(item.center_px) && item.center_px.length === 2 &&
        item.center_px.every(Number.isFinite));
    if (items.length !== 1) throw Error('未找到唯一完整目标，请先移开手并保持物料竖直');
    return items[0];
}
async function read(url) {
    controller = new AbortController();
    const timeout = setTimeout(() => controller.abort(), 4000);
    try {
        const response = await fetch(url, {cache: 'no-store', signal: controller.signal});
        if (!response.ok) throw Error(`读取失败（${response.status}）`);
        return await response.json();
    } finally { clearTimeout(timeout); }
}
function cross(point) {
    const [x,y] = point;
    ctx.strokeStyle = '#00c9ff'; ctx.lineWidth = 3;
    ctx.beginPath(); ctx.moveTo(x-22,y); ctx.lineTo(x+22,y);
    ctx.moveTo(x,y-22); ctx.lineTo(x,y+22); ctx.stroke();
    ctx.beginPath(); ctx.arc(x,y,13,0,Math.PI*2); ctx.stroke();
}
function render() {
    if (!reference) return;
    ctx.clearRect(0,0,canvas.width,canvas.height);
    if (last?.image) ctx.drawImage(last.image,0,0);
    cross(refItem.center_px);
    const age = last ? liveAge(last.observation,last.received,performance.now(),last.roundTrip) : Infinity;
    const fresh = !paused && !document.hidden && !halted && age !== null && age <= 250;
    if (!fresh) {
        el('delta').textContent = '等待新画面 · 当前偏差不作为实时提示';
        if (!halted) el('status').textContent = paused || document.hidden ? '已暂停采图' : (lastError || '保留上一帧，仅供回看');
        return;
    }
    const [x,y] = last.item.center_px, [rx,ry] = refItem.center_px;
    ctx.strokeStyle = '#ff9c21'; ctx.lineWidth = 3;
    ctx.beginPath(); ctx.arc(x,y,8,0,Math.PI*2); ctx.stroke();
    ctx.beginPath(); ctx.moveTo(rx,ry); ctx.lineTo(x,y); ctx.stroke();
    el('delta').textContent = `当前距参考 ${Math.hypot(x-rx,y-ry).toFixed(1)} 像素`;
    el('status').textContent = `画面横向 ${(x-rx).toFixed(1)} / 纵向 ${(y-ry).toFixed(1)} px · 帧龄 ${Math.round(age)} ms`;
}
async function poll() {
    if (paused || halted || document.hidden || busy) return;
    busy = true;
    try {
        const start = performance.now();
        const observation = await read('/api/vision/live/observation?role=material&mode=material');
        const received = performance.now(), roundTrip = received-start;
        if (observation.domain !== 'camera_live' || observation.camera !== reference.camera ||
            observation.stream_id !== reference.stream_id ||
            observation.profile_sha256 !== reference.profile_sha256 ||
            JSON.stringify(observation.image_size) !== JSON.stringify(reference.image_size)) {
            halted = true;
            throw Error('相机身份或识别配置已变化，请重新核对参考；已停止对位');
        }
        const item = target(observation,refItem.color_id);
        const image = new Image();
        image.src = observation.image.startsWith('data:') ? observation.image : `data:image/jpeg;base64,${observation.image}`;
        await image.decode();
        if (image.width !== canvas.width || image.height !== canvas.height) throw Error('图像尺寸不匹配');
        if (paused || document.hidden) return;
        lastError = ''; last = {observation,received,roundTrip,item,image}; render();
    } catch (error) {
        lastError = error.name === 'AbortError' ? '采图中断或超时，未显示旧偏差' : error.message;
        last = null; render(); el('status').textContent = lastError;
    } finally {
        busy = false;
        if (!paused && !halted && !document.hidden) timer = setTimeout(poll,last ? 40 : 800);
    }
}
el('pause').addEventListener('click', () => {
    paused = !paused; clearTimeout(timer); controller?.abort();
    el('pause').textContent = paused ? '继续采图' : '暂停采图'; render();
    if (!paused) poll();
});
document.addEventListener('visibilitychange', () => {
    clearTimeout(timer);
    if (document.hidden) { controller?.abort(); render(); }
    else if (!paused && !halted) poll();
});
async function start() {
    const id = new URLSearchParams(location.search).get('sample');
    if (!/^[0-9a-f]{32}$/.test(id || '')) throw Error('请使用带原始样本编号的对位链接');
    reference = await read(`/api/vision/samples/${id}/observation`);
    if (reference.original_domain !== 'camera_live' || reference.camera !== 'material' ||
        !reference.stream_id || !reference.profile_sha256 ||
        !Array.isArray(reference.image_size) || reference.image_size.length !== 2 ||
        reference.image_size.some(n => !Number.isInteger(n) || n <= 0 || n > 4096)) throw Error('参考样本不是有效的物料相机原图');
    const colors = new Set((reference.items || []).map(item => item.color_id));
    if (colors.size !== 1) throw Error('参考样本需要唯一颜色目标');
    refItem = target(reference,[...colors][0]);
    [canvas.width,canvas.height] = reference.image_size;
    el('original').src = `/api/vision/samples/${id}/image`;
    el('reference-info').textContent = `参考中心：${refItem.center_px.map(n=>n.toFixed(2)).join(' / ')} px；原图 ${reference.image_size.join(' × ')}。`;
    el('pause').disabled = false;
    // 显式开始采图，避免初始化时标签页尚未可见导致首次循环被跳过。
    setInterval(render,80); render();
}
start().catch(error => { halted = true; el('delta').textContent = '无法开始对位'; el('status').textContent = error.message; });
