// 复用旧控件、目标字段和状态机；主界面只保留五路独立输入及姿态联调。
const $ = id => document.getElementById(id);
const names = {lift:'升降电机',reach:'伸缩电机',arm:'转臂舵机',tray:'料盘舵机',gripper:'夹爪舵机'};
const keys = Object.keys(names);
let callbacks;

export function initCompact(root, options) {
    callbacks = options;
    root.querySelector('.section-intro h2')?.remove();
    root.querySelector('.section-intro .eyebrow')?.remove();
    const oldGrid = root.querySelector('.mech-columns');
    const pose = $('pose-id').closest('article');
    pose.querySelector('h2').textContent = '姿态联调与保存';
    $('pose-preview').textContent = '预览安全过渡';
    $('pose-save').textContent = '保存姿态版本';
    $('pose-run').textContent = '联调选中姿态';
    const base = document.createElement('article'); base.className = 'card mech-basebar';
    const title = document.createElement('b'); title.textContent = '离线准备';
    base.append(title,$('mech-confirm').closest('label'),$('mech-zero'),$('mech-anchor'));
    const quick = document.createElement('article'); quick.className = 'card mech-quick';
    quick.innerHTML = '<div class="card-head"><h2>两个电机 · 三个舵机</h2><span class="badge lab">仅模型</span></div><p class="hint">输入后点本行执行；五个目标也用于右侧姿态联调。下降、伸出为正，编辑不会自动运动。</p><table class="mech-table"><thead><tr><th>执行器</th><th>模型值</th><th>目标</th><th></th></tr></thead><tbody id="mech-quick-rows"></tbody></table><div id="quick-footer"></div>';
    for (const axis of keys) {
        const row = document.createElement('tr');
        const name = document.createElement('td'); name.textContent = names[axis];
        const current = document.createElement('td'); current.id = `quick-${axis}`; current.textContent = '未知';
        const target = document.createElement('td');
        const label = document.createElement('label'); label.textContent = axis==='lift'||axis==='reach' ? 'mm' : '°';
        const input = $(`teach-${axis}`); input.setAttribute('aria-label',`${names[axis]}目标`);
        label.append(input); target.append(label);
        const action = document.createElement('td');
        const button = document.createElement('button'); button.className = 'quick-move'; button.dataset.axis = axis;
        button.textContent = '执行'; button.setAttribute('aria-label',`单独执行${names[axis]}`);
        button.onclick = () => {
            try {
                const number = input.valueAsNumber;
                if (!Number.isFinite(number) || number<Number(input.min) || number>Number(input.max))
                    throw new Error('请检查本行目标范围');
                const value = number*((axis==='lift'||axis==='reach') ? .001 : Math.PI/180);
                const release = $('mech-release').checked;
                $('mech-release').checked = false;
                callbacks.invoke('move',{axis,[axis==='lift'||axis==='reach'?'target_m':'target_rad']:value,release_confirmed:release});
            } catch (error) { callbacks.notify(error.message,true); }
        };
        action.append(button); row.append(name,current,target,action);
        quick.querySelector('tbody').append(row);
    }
    pose.querySelector('.mech-targets').remove();
    quick.querySelector('#quick-footer').append($('mech-release').closest('label'),$('tray-presets'),$('pose-capture'));
    const sourceRow = $('pose-candidate').closest('.plan-inputs');
    const sources = document.createElement('details'); sources.innerHTML = '<summary>旧工程候选姿态</summary>';
    sources.append(sourceRow); pose.append(sources);
    const extra = document.createElement('details'); extra.innerHTML = '<summary>更多联调选项与说明</summary>';
    extra.append($('pose-single-step').closest('.button-row'),[...pose.querySelectorAll('p.hint')].at(-1),sources);
    pose.append(extra);
    const layout = document.createElement('div'); layout.className = 'mech-simple-layout'; layout.append(quick,pose);
    const program = document.createElement('details'); program.id = 'mech-program-root'; program.className = 'card mech-program';
    const advanced = document.createElement('details'); advanced.className = 'card mech-advanced';
    advanced.innerHTML = '<summary>更多：点动、互锁、转存与实车接入</summary>';
    const oldCards = [...root.children].filter(child=>child.tagName==='ARTICLE'||child===oldGrid);
    advanced.append(...oldCards);
    root.append(base,layout,program,advanced);
    const device = document.createElement('button'); device.textContent = '按需加载实车面板（换板完成后）';
    device.onclick = () => { device.disabled = true; options.loadDevice(advanced); };
    advanced.append(device);
}

export function renderCompact(snapshot, enabled) {
    const state = snapshot?.mechanism;
    const busy = state?.busy || snapshot?.program?.busy || ['RUNNING','WAIT_OBSERVATION'].includes(snapshot?.transfer?.mode);
    const ready = enabled && state?.zero_valid && state.servos.every(item=>item.estimate_rad!=null) && !state.fault && !busy && snapshot.simulation.stopped && snapshot.simulation.mode !== 'FAILED';
    for (const axis of keys) {
        const value = axis==='lift'||axis==='reach' ? state?.axes.find(v=>v.id===axis)?.position_m : state?.servos.find(v=>v.id===axis)?.estimate_rad;
        $(`quick-${axis}`).textContent = value == null ? '未知' : axis==='lift'||axis==='reach' ? `${(value*1000).toFixed(1)}mm` : `${(value*180/Math.PI).toFixed(1)}°`;
        $(`teach-${axis}`).disabled = !!busy;
        document.querySelector(`.quick-move[data-axis="${axis}"]`).disabled = !ready;
    }
}
