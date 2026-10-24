// 视觉仅输出观测，不调用底盘、机构或成功计数接口。
const page = document.createElement('section');
page.className = 'page';
page.id = 'page-vision';
page.hidden = true;
page.innerHTML = `<div class="card">
<div class="card-head"><h2>相机预览与识别</h2><span class="badge lab">观测专用</span></div>
<p>沿用旧相机配置 · 像素候选 · 毫米标定待验证 · 不触发实车动作</p>
<div style="display:flex;gap:12px;flex-wrap:wrap;align-items:center">
<label>来源 <select id="vision-source"><option value="replay">旧现场样本回放</option><option value="live">实时相机</option></select></label>
<label>相机 <select id="vision-role"><option value="material">物料相机</option><option value="qr">扫码相机</option></select></label>
<label>识别 <select id="vision-mode"><option value="material">六色物料</option><option value="ring">圆环编号</option><option value="qr">任务码</option></select></label>
<label>样本 <select id="vision-sample"></select></label>
<button id="vision-read" class="primary">读取一帧</button><button id="vision-live">开始预览</button><button id="vision-stop">停止预览</button>
</div><p id="vision-status" role="status">正在读取视觉服务状态…</p>
<canvas id="vision-canvas" width="1280" height="720" style="width:100%;max-height:65vh;object-fit:contain;background:#102532;border-radius:12px"></canvas>
<pre id="vision-result" style="white-space:pre-wrap;max-height:240px;overflow:auto">暂无观测</pre>
</div>`;
document.querySelector('main footer').before(page);
const nav = document.createElement('button');
nav.className = 'nav-item';
nav.dataset.page = 'vision';
nav.innerHTML = '<span>◎</span><span>相机与识别</span>';
nav.setAttribute('aria-label', '相机与识别');
document.querySelector('nav').append(nav);
nav.onclick = () => {
    // 复用既有页面切换，确保按住点动先被松开。
    document.querySelector('[data-page="logs"]').click();
    document.querySelectorAll('.page').forEach(item => { item.hidden = item !== page; });
    document.querySelectorAll('[data-page]').forEach(item => item.classList.toggle('active', item === nav));
    document.querySelector('h1').textContent = '相机与识别';
    history.replaceState(null, '', '#vision');
};
const el = name => document.getElementById(`vision-${name}`);
let timer = null;
let generation = 0;
let busy = false;
let displayedFrame = 0;
function stop() {
    clearInterval(timer);
    timer = null;
    generation += 1;
}
function clear(message) {
    const canvas = el('canvas');
    canvas.getContext('2d').clearRect(0, 0, canvas.width, canvas.height);
    el('status').textContent = message;
    el('result').textContent = '无有效观测；不表示空位或释放成功';
}
async function read() {
    if (busy) return;
    busy = true;
    const epoch = generation;
    try {
        const replay = el('source').value === 'replay';
        const url = replay ? `/api/vision/replay/${encodeURIComponent(el('sample').value)}?mode=${el('mode').value}` :
            `/api/vision/live/observation?role=${el('role').value}&mode=${el('mode').value}`;
        const response = await fetch(url, {cache:'no-store', signal:AbortSignal.timeout(4000)});
        const result = await response.json();
        if (!response.ok) throw new Error(result.detail || '读取失败');
        const image = new Image();
        image.src = result.image;
        await image.decode();
        if (generation !== epoch || page.hidden) return;
        const canvas = el('canvas');
        [canvas.width, canvas.height] = result.image_size;
        const context = canvas.getContext('2d');
        context.drawImage(image, 0, 0);
        context.lineWidth = 3;
        context.font = '22px sans-serif';
        for (const item of result.items) {
            const box = item.bbox_px;
            const center = item.center_px;
            const uncertain = item.clipped || item.frame_clipped || item.roi_clipped || item.shape_consistent === false || item.position_id === null || item.valid === false;
            context.strokeStyle = uncertain ? '#ffb64d' : '#00e2c8';
            context.fillStyle = context.strokeStyle;
            if (box) context.strokeRect(box[0], box[1], result.bbox_format === 'xywh' ? box[2] : box[2]-box[0], result.bbox_format === 'xywh' ? box[3] : box[3]-box[1]);
            if (center) {
                context.beginPath();
                context.arc(center[0], center[1], 6, 0, Math.PI*2);
                context.stroke();
                context.fillText((item.color_name || item.text || `环 ${item.position_id ?? '未知'}`) + (uncertain ? ' · 待确认' : ''), center[0]+10, center[1]-10);
            }
        }
        el('status').textContent = `${replay ? '历史回放（非实时）' : '实时相机'} · 帧 ${result.frame_id} · ${result.items.length}个候选 · 不确认抓放成功`;
        const {image: ignored, ...details} = result;
        el('result').textContent = JSON.stringify(details, null, 2);
        if (!replay) {
            // 无新图或识别报错即清除旧预览，避免旧帧伪装成实时。
            const frameGeneration = generation;
            const display = ++displayedFrame;
            setTimeout(() => { if (generation === frameGeneration && displayedFrame === display) clear('上一帧已过期，等待新帧'); }, 1000);
        }
    } catch (error) {
        if (generation === epoch) clear(error.message);
    } finally { busy = false; }
}
el('read').onclick = () => { stop(); read(); };
el('live').onclick = () => { stop(); el('source').value = 'live'; read(); timer = setInterval(read, 500); };
el('stop').onclick = () => { stop(); clear('预览已停止；不会改变机器人状态'); };
for (const name of ['source','role','mode','sample']) el(name).onchange = () => { stop(); clear('选择已改变，请读取新帧'); };
document.addEventListener('visibilitychange', () => { if (document.hidden) { stop(); clear('页面隐藏，预览已停止'); } });
new MutationObserver(() => { if (page.hidden) stop(); }).observe(page, {attributes:true, attributeFilter:['hidden']});
try {
    const response = await fetch('/api/vision/state');
    if (!response.ok) throw new Error('请重载工作台后启用视觉接口');
    const state = await response.json();
    for (const sample of state.samples) el('sample').add(new Option(sample.name, sample.id));
    el('status').textContent = state.live.available ? '实时服务已连接；请选择相机' : state.live.reason;
} catch (error) { el('status').textContent = error.message; }
if (location.hash === '#vision') nav.click();
