import {
    imagePoint,
    imageRect,
    parseRect,
    calibrationPoints,
    sourceName,
    liveAge,
    cameraHealth
} from './workbench-vision-state.js?v=1';

// 所有保存均为样本或标定草稿；不调用控制、清故障和成功计数接口。
const page = document.getElementById('page-vision');
page.innerHTML = `
<div class="vision-summary"><div><span class="eyebrow">VISION WORKSPACE</span><p>观察与采样 · 标定草稿 · 机械姿态关联</p></div><span id="vision-service" class="badge muted">读取服务状态</span></div>
<div class="vision-layout"><div class="vision-main">
 <article class="card vision-viewer">
  <div class="vision-toolbar">
   <label>来源<select id="vision-source"><option value="replay">旧样本回放</option><option value="live">实时相机</option><option value="saved">采集样本</option></select></label>
   <label>相机<select id="vision-role"><option value="material">物料相机</option><option value="qr">扫码相机</option></select></label>
   <label>识别<select id="vision-mode"><option value="material">六色物料</option><option value="ring">圆环编号</option><option value="qr">任务码</option></select></label>
   <button id="vision-read">读取一帧</button><button id="vision-live" class="primary">开始预览</button><button id="vision-freeze">冻结画面</button>
  </div><label class="vision-source-list">样本<select id="vision-sample"></select></label>
  <div class="vision-canvas-wrap"><canvas id="vision-canvas" width="1280" height="720" aria-label="相机画面，拖动可框选区域"></canvas><div id="vision-empty">请选择样本，或接通相机后读取画面</div></div>
  <div class="vision-framebar"><span id="vision-frame">尚无画面</span><span id="vision-pointer">原图坐标 · px</span></div>
  <div class="vision-tools"><label>画面操作<select id="vision-tool"><option value="inspect">查看识别</option><option value="roi">框选检测ROI</option><option value="grasp">框选抓取ROI</option><option value="color">框选颜色样本</option><option value="point">点击添加标定点</option></select></label><label class="vision-check"><input id="vision-overlay" type="checkbox" checked>识别叠加</label><button id="vision-clear-region">清除框选与点</button></div>
  <p id="vision-status" class="vision-message" role="status" aria-live="polite">正在读取视觉服务…</p>
 </article>
 <article class="card"><div class="card-head"><h2>本帧识别</h2><span id="vision-count" class="badge muted">0个候选</span></div><div id="vision-detections" class="vision-detections">暂无有效观测</div><details class="vision-details"><summary>原始观测与时间信息</summary><pre id="vision-result">尚无观测</pre></details></article>
</div><aside class="vision-panel">
 <div class="vision-tabs" role="tablist" aria-label="视觉工具"><button data-vision-tab="capture" role="tab" aria-selected="true">采集</button><button data-vision-tab="calibration" role="tab" aria-selected="false">标定</button><button data-vision-tab="library" role="tab" aria-selected="false">资料</button></div>
 <article class="card" data-vision-pane="capture"><h2>保存当前样本</h2><p class="vision-help">冻结后标注画面，保存同帧图像、识别结果、关联版本和设备快照。</p>
  <label>样本名称<input id="vision-name" maxlength="64" placeholder="如：料位1_空槽"></label>
  <div class="vision-fields"><label>观察工位<select id="vision-stage"><option value="raw">原料区</option><option value="car_slot">车载料位</option><option value="rough">粗加工区</option><option value="storage">暂存区</option><option value="stack">第二层码垛</option></select></label>
  <label>人工标签<select id="vision-label"><option value="unknown">待确认</option><option value="empty">空位 / 空爪</option><option value="present">目标存在</option><option value="held">持料</option><option value="stored">已存车</option><option value="released">已释放</option><option value="placed">已放置</option></select></label>
  <label>目标颜色<select id="vision-color"><option value="">未指定</option><option value="1">红色</option><option value="2">黄色</option><option value="3">蓝色</option><option value="4">绿色</option><option value="5">黑色</option><option value="6">浅蓝色</option></select></label>
  <label>车载料位<select id="vision-slot"><option value="">未指定</option><option value="1">料位1 · 30°</option><option value="2">料位2 · 150°</option><option value="3">料位3 · 270°</option></select></label></div>
  <label>关联机械版本<select id="vision-binding"><option value="">暂不关联</option></select></label><button id="vision-bind-refresh" class="text-button">刷新已保存姿态 / 联调版本</button>
  <label>实物观察<textarea id="vision-note" maxlength="1000" rows="3" placeholder="如：夹持稳定；命令20mm，尺量19mm"></textarea></label>
  <button id="vision-save-sample" class="primary vision-wide">保存同帧图像与记录</button><p class="vision-help">文字标注暂存本机，图像须明确保存。人工标签不增加成功计数；设备快照采于保存时刻。</p>
 </article>
 <article class="card" data-vision-pane="calibration" hidden><h2>区域与平面标定</h2><p class="vision-help">冻结后框选或添加点。不同高度平面分别保存。</p>
  <label>草稿名称<input id="vision-draft-name" maxlength="64" placeholder="如：原料预抓_当前安装"></label>
  <label>目标平面<select id="vision-plane"><option value="raw_table">原料台</option><option value="floor">地面圆环</option><option value="car_slot">车载料位</option><option value="stack_top">码垛顶面</option></select></label>
  <label>检测ROI / px<input id="vision-roi" placeholder="x1,y1,x2,y2"></label><label>抓取ROI / px（可选）<input id="vision-grasp" placeholder="x1,y1,x2,y2"></label>
  <div class="vision-color-box"><b>颜色采样</b><p id="vision-color-region" class="vision-help">框选均匀表面，在采集页选择目标颜色。</p><button id="vision-save-color">保存颜色样本草稿</button></div>
  <details class="vision-details"><summary>像素 → 毫米标定点</summary><p class="vision-help">至少4个拟合点和1个独立检查点，建议6点以上。毫米坐标使用同一平面基准；“检”不参与拟合。</p><div class="vision-point-head"><span>像素u/v</span><span>毫米X/Y</span><span>检</span></div><div id="vision-points"></div><button id="vision-add-point">手动添加点</button></details>
  <button id="vision-save-draft" class="primary vision-wide">保存新版本并计算误差</button><p id="vision-fit" class="vision-help">未应用草稿，须实物验证后接入正式任务。</p>
 </article>
 <article class="card" data-vision-pane="library" hidden><div class="card-head"><h2>版本与样本</h2><button id="vision-refresh">刷新</button></div><p id="vision-library-status" class="vision-help"></p>
  <label>记录类别<select id="vision-library-kind"><option value="samples">图像样本</option><option value="drafts">标定草稿</option><option value="colors">颜色样本</option></select></label><div id="vision-records" class="vision-records"></div>
  <p class="vision-help">每类最多200条，满额后要求归档。版本不覆盖；读取草稿不修改运行配置。</p>
 </article>
 <details class="card vision-details"><summary>连接与使用说明</summary><div id="vision-cameras"></div><p class="vision-help">安装、分辨率、曝光或高度变化后需复核。当前使用旧六色/圆环算法；NCNN模型尚未接入本页。</p></details>
</aside></div>`;

const el = name => document.getElementById(`vision-${name}`),
    canvas = el('canvas');
const labelNames = {unknown:'待确认',empty:'空位 / 空爪',present:'目标存在',held:'持料',
    stored:'已存车',released:'已释放',placed:'已放置'};
let generation = 0,
    timer = null,
    controller = null,
    current = null,
    bitmap = null,
    frozen = true,
    received = 0,
    rtt = 0;
let points = [],
    colorRect = null,
    dragging = null,
    previewRect = null,
    library = {
        samples: [],
        drafts: [],
        colors: [],
        bindings: []
    },
    saving = false,
    replays = [];
const point = e => imagePoint(e.clientX, e.clientY, canvas.getBoundingClientRect(), [canvas.width, canvas
    .height
]);
const tell = (text, error = false) => {
    el('status').textContent = text;
    el('status').classList.toggle('error', error);
};

// 仅暂存编辑文字，不持久化帧引用、设备状态或控制令牌。
const draftFields = ['name','note','stage','label','color','slot','draft-name','plane'];
try {
    const saved = JSON.parse(localStorage.getItem('gc_vision_notes_v1') || '{}');
    for (const name of draftFields) {
        const input = el(name), value = saved[name];
        if (typeof value !== 'string' || value.length > 1000) continue;
        if (input.tagName === 'SELECT' && ![...input.options].some(o=>o.value===value)) continue;
        input.value = value;
    }
} catch {}
for (const name of draftFields) el(name).addEventListener('input',()=>{
    try {
        localStorage.setItem('gc_vision_notes_v1',JSON.stringify(
            Object.fromEntries(draftFields.map(key=>[key,el(key).value]))));
    } catch { tell('本机文字草稿保存失败，请及时保存服务端记录。',true); }
});

function stop() {
    generation++;
    clearTimeout(timer);
    timer = null;
    controller?.abort();
    controller = null;
}

function buttons() {
    for (const name of ['save-sample', 'save-draft']) el(name).disabled = !current || !frozen || saving;
    el('save-color').disabled = !current || !frozen || !colorRect || saving;
    for (const name of ['read', 'live', 'source', 'role', 'mode', 'sample']) el(name).disabled = saving;
    for (const name of ['role', 'mode']) el(name).disabled = saving || el('source').value === 'saved';
    el('freeze').textContent = frozen ? '画面已冻结' : '冻结画面';
}

function freeze() {
    stop();
    frozen = true;
    buttons();
    frameLabel();
    if (current) tell('已冻结，可框选、标注和保存；不用于实时动作判断。');
}

function frameLabel() {
    if (!current) {
        el('frame').textContent = '尚无画面';
        return;
    }
    const age = liveAge(current, received, performance.now(), rtt);
    el('frame').textContent =
        `${sourceName(current)} · ${frozen?'已冻结':'预览中'} · ${canvas.width}×${canvas.height} · 帧 ${current.frame_id}${age===null?'':` · 接收估计龄 ${Math.round(age)}ms`}`;
}

function draw() {
    const ctx = canvas.getContext('2d');
    ctx.clearRect(0, 0, canvas.width, canvas.height);
    el('empty').hidden = !!bitmap;
    if (!bitmap) return;
    ctx.drawImage(bitmap, 0, 0, canvas.width, canvas.height);
    ctx.lineWidth = Math.max(2, canvas.width / 500);
    ctx.font = `${Math.max(16,canvas.width/60)}px Microsoft YaHei,sans-serif`;
    if (el('overlay').checked)
        for (const item of current.items || []) {
            const box = item.bbox_px,
                center = item.center_px,
                uncertain = item.clipped || item.frame_clipped || item.roi_clipped || item
                .shape_consistent === false || item.position_id === null || item.valid === false;
            ctx.strokeStyle = ctx.fillStyle = uncertain ? '#e8a245' : '#00c5b7';
            if (box) ctx.strokeRect(box[0], box[1], current.bbox_format === 'xywh' ? box[2] : box[2] - box[0],
                current.bbox_format === 'xywh' ? box[3] : box[3] - box[1]);
            if (center) {
                ctx.beginPath();
                ctx.arc(center[0], center[1], 5, 0, Math.PI * 2);
                ctx.stroke();
                ctx.fillText(item.color_name || item.text || `环 ${item.position_id??'未知'}`, center[0] + 8,
                    Math.max(20, center[1] - 10));
            }
        }
    for (const [name, color] of [
            ['roi', '#4ca3ff'],
            ['grasp', '#d29aff']
        ]) try {
        const b = parseRect(el(name).value, [canvas.width, canvas.height]);
        ctx.strokeStyle = color;
        ctx.strokeRect(b[0], b[1], b[2] - b[0], b[3] - b[1]);
    } catch {}
    for (const [i, p] of points.entries()) {
        if (p.u === '' || p.v === '') continue;
        ctx.fillStyle = p.check ? '#ffb14a' : '#f7fbff';
        ctx.beginPath();
        ctx.arc(Number(p.u), Number(p.v), 7, 0, Math.PI * 2);
        ctx.fill();
        ctx.fillText(`${i+1}${p.check?'检':''}`, Number(p.u) + 10, Number(p.v) + 10);
    }
    for (const [b, c] of [
            [colorRect, '#f8c44d'],
            [previewRect, '#ffffff']
        ])
        if (b) {
            ctx.strokeStyle = c;
            ctx.strokeRect(b[0], b[1], b[2] - b[0], b[3] - b[1]);
        }
}

function detections(result) {
    el('count').textContent = `${result.items.length}个候选`;
    el('detections').replaceChildren();
    if (!result.items.length) el('detections').textContent = '本帧无候选；不能据此确认空位或释放成功。';
    for (const item of result.items) {
        const row = document.createElement('div');
        row.className = 'vision-detection';
        const name = document.createElement('b'),
            detail = document.createElement('span');
        name.textContent = item.color_name || item.text || `圆环 ${item.position_id??'未知'}`;
        detail.textContent = item.center_px ?
            `中心 ${item.center_px.map(v=>Number(v).toFixed(1)).join(' / ')} px` : '无可靠中心';
        row.append(name, detail);
        el('detections').append(row);
    }
    const {
        image,
        ...rest
    } = result;
    el('result').textContent = JSON.stringify(rest, null, 2);
}
async function request(path, body) {
    const response = await fetch('/api/vision/' + path, {
        cache: 'no-store',
        signal: AbortSignal.timeout(5000),
        ...(body ? {
            method: 'POST',
            headers: {
                'Content-Type': 'application/json',
                'X-Workbench': '1'
            },
            body: JSON.stringify(body)
        } : {})
    });
    const value = await response.json();
    if (!response.ok) throw Error(typeof value.detail === 'string' ? value.detail : '参数不完整，请检查输入');
    return value;
}
async function read(loop = false) {
    stop();
    const epoch = generation;
    controller = new AbortController();
    const started = performance.now();
    frozen = !loop;
    buttons();
    try {
        const source = el('source').value,
            path = source === 'live' ?
            `live/observation?role=${el('role').value}&mode=${el('mode').value}` : source === 'saved' ?
            `samples/${encodeURIComponent(el('sample').value)}/observation` :
            `replay/${encodeURIComponent(el('sample').value)}?mode=${el('mode').value}`;
        const response = await fetch('/api/vision/' + path, {
            cache: 'no-store',
            signal: AbortSignal.any([controller.signal, AbortSignal.timeout(4500)])
        });
        const result = await response.json();
        if (!response.ok) throw Error(result.detail || '读取失败');
        const image = new Image();
        image.src = result.image;
        await image.decode();
        if (epoch !== generation || page.hidden) return;
        current = result;
        bitmap = image;
        received = performance.now();
        rtt = received - started;
        [canvas.width, canvas.height] = result.image_size;
        if (source === 'saved') {
            if (result.camera) el('role').value = result.camera;
            if (result.recognition_mode) el('mode').value = result.recognition_mode;
        }
        if (loop) {
            points = [];
            colorRect = null;
            el('roi').value = '';
            el('grasp').value = '';
            renderPoints();
        }
        draw();
        detections(result);
        frameLabel();
        buttons();
        tell(`${sourceName(result)} · ${loop?'预览只展示候选，框选时自动冻结':'画面已冻结，可标注保存'}`);
        if (loop) timer = setTimeout(() => read(true), 350);
    } catch (error) {
        if (epoch !== generation) return;
        current = null;
        bitmap = null;
        frozen = true;
        draw();
        buttons();
        frameLabel();
        el('detections').textContent = '读取失败，无有效观测';
        el('result').textContent = '无有效观测';
        tell(error.name === 'TimeoutError' ? '读取超时，请检查相机或转发连接' : error.message, true);
    }
}

function sourceList() {
    const source = el('source').value;
    el('sample').closest('label').hidden = source === 'live';
    el('sample').replaceChildren(...(source === 'saved' ? library.samples.map(s => new Option(
        `${s.name} · v${s.revision} · ${labelNames[s.label] || '待确认'}`, s.id)) : replays.map(s => new Option(s.name, s.id))));
}

function resetFrame() {
    stop();
    current = null;
    bitmap = null;
    frozen = true;
    points = [];
    colorRect = null;
    previewRect = null;
    el('roi').value = '';
    el('grasp').value = '';
    renderPoints();
    draw();
    buttons();
    frameLabel();
    el('count').textContent = '0个候选';
    el('detections').textContent = '请读取新帧';
    el('result').textContent = '尚无观测';
    tell('选择已改变，请读取新帧。');
}
for (const name of ['source', 'role', 'mode', 'sample']) el(name).onchange = () => {
    resetFrame();
    if (name === 'source') sourceList();
};
el('read').onclick = () => read();
el('live').onclick = () => {
    el('source').value = 'live';
    sourceList();
    resetFrame();
    read(true);
};
el('freeze').onclick = freeze;
el('overlay').onchange = draw;
el('tool').onchange = () => {
    if (current) freeze();
    canvas.classList.toggle('selecting', el('tool').value !== 'inspect');
};
for (const name of ['roi', 'grasp']) el(name).onchange = () => {
    try {
        if (el(name).value) parseRect(el(name).value, [canvas.width, canvas.height]);
        draw();
    } catch (e) {
        tell(e.message, true);
    }
};
canvas.addEventListener('pointerdown', e => {
    if (!current || el('tool').value === 'inspect' || e.button !== 0) return;
    freeze();
    canvas.setPointerCapture(e.pointerId);
    const p = point(e);
    if (el('tool').value === 'point') {
        if (points.length >= 24) {
            tell('最多24个标定点', true);
            return;
        }
        points.push({
            u: p[0],
            v: p[1],
            x_mm: '',
            y_mm: '',
            check: false
        });
        renderPoints();
        draw();
        return;
    }
    dragging = p;
});
canvas.addEventListener('pointermove', e => {
    const p = point(e);
    el('pointer').textContent = `u ${p[0]} / v ${p[1]} px`;
    if (dragging) {
        previewRect = [Math.min(p[0], dragging[0]), Math.min(p[1], dragging[1]), Math.max(p[0],
            dragging[0]), Math.max(p[1], dragging[1])];
        draw();
    }
});
canvas.addEventListener('pointerup', e => {
    if (!dragging) return;
    try {
        const rect = imageRect(dragging, point(e));
        if (el('tool').value === 'color') {
            colorRect = rect;
            el('color-region').textContent = `颜色区域：${rect.join(', ')} px`;
        } else el(el('tool').value === 'grasp' ? 'grasp' : 'roi').value = rect.join(', ');
    } catch (error) {
        tell(error.message, true);
    }
    dragging = null;
    previewRect = null;
    draw();
    buttons();
});
canvas.addEventListener('pointercancel', () => {
    dragging = null;
    previewRect = null;
    draw();
});

function renderPoints() {
    el('points').replaceChildren(...points.map((p, i) => {
        const row = document.createElement('div');
        row.className = 'vision-point-row';
        for (const pair of [
                ['u', 'v'],
                ['x_mm', 'y_mm']
            ]) {
            const group = document.createElement('div');
            for (const key of pair) {
                const input = document.createElement('input');
                input.type = 'number';
                input.step = '0.1';
                input.value = p[key];
                input.setAttribute('aria-label', `点${i+1} ${key}`);
                input.oninput = () => {
                    p[key] = input.value;
                };
                input.onchange = draw;
                group.append(input);
            }
            row.append(group);
        }
        const check = document.createElement('input');
        check.type = 'checkbox';
        check.checked = p.check;
        check.setAttribute('aria-label', `点${i+1}为独立检查点`);
        check.onchange = () => {
            p.check = check.checked;
            draw();
        };
        const del = document.createElement('button');
        del.textContent = '×';
        del.setAttribute('aria-label', `删除点${i+1}`);
        del.onclick = () => {
            points.splice(i, 1);
            renderPoints();
            draw();
        };
        row.append(check, del);
        return row;
    }));
}
el('add-point').onclick = () => {
    if (!current) {
        tell('请先读取画面', true);
        return;
    }
    freeze();
    if (points.length < 24) {
        points.push({
            u: '',
            v: '',
            x_mm: '',
            y_mm: '',
            check: false
        });
        renderPoints();
    }
};
el('clear-region').onclick = () => {
    points = [];
    colorRect = null;
    el('roi').value = '';
    el('grasp').value = '';
    el('color-region').textContent = '尚未框选颜色区域';
    renderPoints();
    draw();
    buttons();
};
const binding = () => library.bindings.find(b => `${b.kind}:${b.id}:${b.revision}:${b.sha256}` === el(
    'binding').value) || null;
async function save(action) {
    if (saving || !current || !frozen) return;
    saving = true;
    buttons();
    try {
        await action();
        await refreshLibrary();
    } catch (e) {
        tell(e.message, true);
    } finally {
        saving = false;
        buttons();
    }
}
el('save-sample').onclick = () => save(async () => {
    const name = el('name').value.trim();
    if (!name) throw Error('请填写样本名称');
    const v = await request('samples', {
        observation_id: current.observation_id,
        name,
        stage: el('stage').value,
        label: el('label').value,
        color_id: el('color').value ? Number(el('color').value) : null,
        slot: el('slot').value ? Number(el('slot').value) : null,
        note: el('note').value,
        binding: binding()
    });
    tell(`已保存 ${v.name} · ${v.id.slice(0,8)}；成功计数未改变。`);
});
el('save-draft').onclick = () => save(async () => {
    const name = el('draft-name').value.trim();
    if (!name) throw Error('请填写标定草稿名称');
    const v = await request('drafts', {
        observation_id: current.observation_id,
        name,
        plane: el('plane').value,
        roi: parseRect(el('roi').value, [canvas.width, canvas.height]),
        grasp_roi: el('grasp').value.trim() ? parseRect(el('grasp').value, [canvas.width,
            canvas.height
        ]) : null,
        points: calibrationPoints(points),
        binding: binding()
    });
    el('fit').textContent = v.fit ?
        `拟合最大误差 ${v.fit.fit_max_mm.toFixed(2)}mm；独立检查最大误差 ${v.fit.check_max_mm.toFixed(2)}mm。未应用，须实物验证。` :
        '区域草稿已保存，尚无毫米标定。';
    tell(`已保存 ${v.name} · ${v.id.slice(0,8)}，运行配置未改变。`);
});
el('save-color').onclick = () => save(async () => {
    if (!el('color').value) throw Error('请在采集页选择目标颜色');
    const v = await request('colors', {
        observation_id: current.observation_id,
        roi: colorRect,
        color_id: Number(el('color').value)
    });
    tell(`颜色${v.color_id}样本已保存 · ${v.model.sample_count}个有效像素；尚未应用。`);
});

function renderLibrary() {
    const kind = el('library-kind').value,
        records = library[kind] || [];
    el('library-status').textContent =
        `样本 ${library.samples.length} · 标定 ${library.drafts.length} · 颜色 ${library.colors.length}；保存在PC工程。`;
    el('records').replaceChildren(...records.map(record => {
        const row = document.createElement('div');
        row.className = 'vision-record';
        const title = document.createElement('b'),
            detail = document.createElement('small');
        title.textContent = `${record.name || `颜色${record.color_id}`} · v${record.revision}`;
        detail.textContent =
            `${new Date(record.created_s*1000).toLocaleString('zh-CN',{hour12:false})} · ${record.id.slice(0,8)} · 未应用`;
        const actions = document.createElement('div'),
            download = document.createElement('a');
        download.textContent = '导出JSON';
        download.href = `/api/vision/records/${kind}/${record.id}`;
        download.download = `${record.id}.json`;
        actions.append(download);
        const load = document.createElement('button');
        load.textContent = kind === 'samples' ? '查看画面' : '读取';
        load.onclick = async () => {
            try {
                if (kind === 'samples') {
                    el('source').value = 'saved';
                    sourceList();
                    el('sample').value = record.id;
                    resetFrame();
                    await read();
                } else {
                    const data = await request(`records/${kind}/${record.id}`);
                    el('result').textContent = JSON.stringify(data, null, 2);
                    if (kind === 'drafts') {
                        if (!current || JSON.stringify(current.image_size) !== JSON
                            .stringify(data.image_size) || current.camera !== data.camera)
                            throw Error('请先读取同一相机、同一尺寸画面，再载入草稿');
                        freeze();
                        el('roi').value = data.roi.join(', ');
                        el('grasp').value = data.grasp_roi?.join(', ') || '';
                        el('draft-name').value = data.name;
                        el('plane').value = data.plane;
                        points = data.points;
                        renderPoints();
                        draw();
                        tab('calibration');
                        tell('草稿已载入编辑区，运行配置未改变。');
                    }
                }
            } catch (e) {
                tell(e.message, true);
            }
        };
        actions.prepend(load);
        row.append(title, detail, actions);
        return row;
    }));
    if (!records.length) el('records').textContent = '暂无记录，保存后将在这里显示。';
}
async function refreshLibrary() {
    const old = el('binding').value;
    library = await request('workspace');
    el('binding').replaceChildren(new Option('暂不关联', ''), ...library.bindings.map(b => new Option(
        `${b.kind==='pose'?'姿态':'联调'} ${b.id} · v${b.revision}`,
        `${b.kind}:${b.id}:${b.revision}:${b.sha256}`)));
    if (library.bindings.some(b => `${b.kind}:${b.id}:${b.revision}:${b.sha256}` === old)) el('binding')
        .value = old;
    renderLibrary();
    if (el('source').value === 'saved') {
        const id = el('sample').value;
        sourceList();
        if (library.samples.some(s => s.id === id)) el('sample').value = id;
    }
}
el('bind-refresh').onclick = () => refreshLibrary().catch(e => tell(e.message, true));
el('refresh').onclick = el('bind-refresh').onclick;
el('library-kind').onchange = renderLibrary;

function tab(name) {
    for (const b of page.querySelectorAll('[data-vision-tab]')) b.setAttribute('aria-selected', String(b
        .dataset.visionTab === name));
    for (const pane of page.querySelectorAll('[data-vision-pane]')) pane.hidden = pane.dataset.visionPane !==
        name;
}
for (const b of page.querySelectorAll('[data-vision-tab]')) b.onclick = () => tab(b.dataset.visionTab);
async function health() {
    if (page.hidden || document.hidden) return;
    try {
        const state = await request('state');
        replays = state.samples;
        if (!el('sample').options.length) sourceList();
        el('service').textContent = state.live.available ? '相机服务已连接' : '相机服务未就绪';
        el('service').className = `badge ${state.live.available?'good':'muted'}`;
        el('cameras').replaceChildren(...['material', 'qr'].map(role => {
            const p = document.createElement('p');
            p.className = 'vision-help';
            p.textContent =
                `${role==='material'?'物料':'扫码'}：${cameraHealth(state.live,role).text}`;
            return p;
        }));
    } catch (e) {
        el('service').textContent = '网页接口未就绪';
        tell(e.message, true);
    }
}
new MutationObserver(() => {
    if (page.hidden) {
        if (!frozen) freeze();
        else stop();
    } else {
        health();
        refreshLibrary().catch(e => tell(e.message, true));
    }
}).observe(page, {
    attributes: true,
    attributeFilter: ['hidden']
});
document.addEventListener('visibilitychange', () => {
    if (document.hidden) {
        if (!frozen) freeze();
        else stop();
    }
});
setInterval(() => {
    if (page.hidden) return;
    frameLabel();
    if (!frozen && current && liveAge(current, received, performance.now(), rtt) > 1200) {
        current = null;
        bitmap = null;
        draw();
        buttons();
        tell('画面已过期，等待新帧', true);
    }
}, 300);
setInterval(health, 5000);
buttons();
if (!page.hidden) {
    health();
    refreshLibrary().catch(e => tell(e.message, true));
}
