import {initTransfer,renderTransfer} from './workbench-transfer.js';
import { initDevice } from './workbench-device.js';
import {initCompact,renderCompact} from './workbench-mech-simple.js';
import {initProgram,renderProgram} from './workbench-program.js';
// 机构页面只编辑草稿并调用统一核心，不打开设备、不产生真实PWM。
import {initAcceptance} from './workbench-acceptance.js';
const $ = id => document.getElementById(id);
const names = {lift:'升降', reach:'伸缩', arm:'转臂', tray:'料盘', gripper:'夹爪'};
const axes = ['lift','reach'], servos = ['arm','tray','gripper'];
const all = [...axes,...servos];
const degrees = value => value * 180 / Math.PI;
let api, state, held = null, pending = false, versions = '', selected = null;
let activeBefore = '', enabled = false;
let preparationKey = '';
let prepared = null;
const modes = {IDLE:'待命',RUNNING:'执行中',COMPLETE:'模型完成',STOPPED:'已停止',FAILED:'模型故障',CANCELLED:'已取消'};
const statusNames = {ACCEPTED:'已接受', RUNNING:'运行中', WAIT_STEP:'等待本步确认', WAIT_MODEL_CONFIRM:'等待模型确认', COMPLETE:'模型完成', CANCELLED:'取消', FAILED:'失败'};
const phaseNames = {READY:'已冻结姿态版本',WAIT_STEP:'等待本步确认',AXIS_MODEL_MOVING:'轴模型运动',
    AXIS_MODEL_SETTLING:'轴模型停稳',SERVO_MODEL_SLEWING:'舵机命令缓动',SERVO_COMMAND_SETTLING:'舵机命令等待',
    COMMAND_DONE:'命令完成，尚无姿态确认',MODEL_CONFIRMED:'仅模型确认'};
const fmt = (axis, value) => value == null ? '未知' : axes.includes(axis) ? `${(value * 1000).toFixed(2)} mm` : `${degrees(value).toFixed(1)}°`;
const targets = () => Object.fromEntries(all.map(axis => [axis, numeric(`teach-${axis}`) * (axes.includes(axis) ? .001 : Math.PI / 180)]));

function numeric(id) {
    const value = $(id).valueAsNumber;
    if (!Number.isFinite(value) || value < Number($(id).min) || value > Number($(id).max)) throw new Error('请检查机构目标及速度范围');
    return value;
}
function sameTargets(first, second) {
    return all.every(axis => Math.abs(first[axis] - second[axis]) < 1e-8);
}
function goal(action, args = {}) {
    return api.command('mechanism', {action, zero_version:state?.zero_version || null, ...args});
}
async function invoke(action, args = {}) {
    try {
        const value = await goal(action,args);
        api.notify(value.action_id ? `机构模拟动作 #${value.action_id} 已接受；等待结果。` : '机构离线请求已处理；未连接真实设备。');
        return value;
    } catch (error) { api.notify(error.message,true); return null; }
}
export function cancelMechHeld() {
    held?.button.classList.remove('pressed');
    held = null;
}
export async function stopMechHeld() {
    if (!held) return;
    cancelMechHeld();
    await invoke('stop');
}
async function pulse() {
    if (!held || pending || !enabled) return;
    pending = true;
    const request = held;
    try { await goal('jog',{axis:request.axis,velocity_m_s:request.speed}); }
    catch (error) { cancelMechHeld(); api.notify(error.message,true); }
    finally { pending = false; }
}
function loadTargets(value, id='', name='') {
    for (const axis of all) $(`teach-${axis}`).value = +(value[axis] * (axes.includes(axis) ? 1000 : 180 / Math.PI)).toFixed(5);
    if (id) $('pose-id').value = id;
    if (name) $('pose-name').value = name;
    $('mech-preview').replaceChildren();
}
function poseDraft() {
    const id = $('pose-id').value.trim(), name = $('pose-name').value.trim();
    const revision = Math.max(0,...(state?.poses || []).filter(p => p.id === id).map(p => p.revision));
    return {targets:targets(),pose_id:id,name,revision};
}
function showSteps(steps) {
    const root = $('mech-preview'); root.replaceChildren();
    if (!steps.length) root.textContent = '模型已在该目标；执行时仍生成独立动作结果。';
    steps.forEach(([axis,value],index) => {
        const item = document.createElement('li');
        item.textContent = `${index + 1}. ${names[axis]} → ${fmt(axis,value)}，模型停稳后交接`;
        root.append(item);
    });
}
export function initMechanism(options) {
    api = options;
    $('mechanism-root').innerHTML = `
      <div class="section-intro"><span class="eyebrow">MECHANISM / OFFLINE LAB</span>
      <h2>机械调试</h2><p>升降最高、伸缩最近端为人工基准。下降/伸出为正。全部状态来自离线模型，实际位置与舵机角度未知。</p></div>
      <div class="mech-banner"><div><b id="mech-mode">等待核心</b><p id="mech-reason">无设备连接</p></div>
      <button id="mech-claim">获取统一控制权</button><button id="mech-stop" class="danger">停止机构模型 · 保持夹爪</button></div>
      <article id="readonly-acceptance" class="card mech-preparation"></article>
      <article class="card mech-preparation"><div class="card-head"><h2>实车接入准备</h2><span class="badge muted">实车控制未启用</span></div><div id="mech-mapping"></div>
      <details><summary>查看待验收项目</summary><ul id="mech-pending"></ul></details><p class="hint">沿用当前Web及唯一控制核心；此清单不提供使能、清故障或真实动作入口。</p></article>
      <div class="mechanism-grid mech-columns">
       <article class="card"><div class="card-head"><h2>基准与互锁</h2><span class="badge lab">仅模拟</span></div>
        <dl id="mech-interlocks"></dl>
        <label class="mech-confirm"><input type="checkbox" id="mech-confirm">我确认下列基准/复位只作用于离线模型</label>
        <div class="button-row"><button id="mech-zero">建立模拟零点</button><button id="mech-anchor">锚定舵机估计</button></div>
        <div class="button-row"><button id="mech-invalidate">使零点失效</button><button id="mech-reset">复位机构模型</button></div>
        <p class="hint">锚定为转臂180° / 料盘30° / 夹爪0°。复位丢弃模型位置估计，保留示教历史；不发送真实命令。</p>
       </article>
       <article class="card"><div class="card-head"><h2>位置与命令估计</h2><span class="badge muted">真实反馈未接入</span></div>
        <table class="mech-table"><thead><tr><th>执行器</th><th>模型位置/命令估计</th><th>真实反馈</th></tr></thead><tbody id="mech-values"></tbody></table>
        <div id="mech-bars" class="mech-bars"></div>
        <p class="hint">舵机PWM为公式预览；缓动及600ms等待只表示模型过程，不证明实物到位。轴点动取消后模型立即停止，未仿真机械刹车惯性。</p>
       </article>
       <article class="card"><div class="card-head"><h2>单轴调试</h2><span class="badge muted">49 / 50 mm</span></div>
        <div class="plan-inputs"><label>直线轴<select id="mech-axis"><option value="lift">升降 · 向下为正</option><option value="reach">伸缩 · 向外为正</option></select></label>
        <label>目标 / mm<input type="number" id="axis-target" min="0" max="49" step="0.5" value="0"></label>
        <label>点动速度 / mm/s<input type="number" id="axis-speed" min="0.1" max="3" step="0.1" value="2"></label></div>
        <div class="button-row"><button class="mech-jog" data-sign="-1">按住：上升 / 收回</button><button class="mech-jog" data-sign="1">按住：下降 / 伸出</button></div>
        <button id="axis-move" class="primary">执行单轴目标</button>
        <p class="hint">点动松开、失焦、切页、断网或350ms续约过期即停止。目标输入不会直接驱动。</p>
       </article>
       <article class="card"><div class="card-head"><h2>转臂 / 料盘 / 夹爪</h2><span class="badge muted">无真实角度反馈</span></div>
        <div class="plan-inputs"><label>舵机<select id="mech-servo"><option value="arm">转臂 · 83°～180°</option><option value="tray">料盘 · 0°～270°（待实测）</option><option value="gripper">夹爪 · 0°～98°</option></select></label>
        <label>命令目标 / °<input type="number" id="servo-target" min="83" max="180" step="1" value="180"></label></div>
        <div id="tray-presets" class="mech-presets"></div>
        <label class="mech-confirm"><input type="checkbox" id="mech-release">明确允许本次张爪释放（仅模拟）</label>
        <button id="servo-move" class="primary">执行舵机目标</button>
        <p class="hint">夹爪0°观察、60°释放、98°保持。停止不会张爪。料盘0°基准，30°/150°/270°为三个料位；预设只填目标，需另点执行。</p>
       </article>
       <article class="card mech-wide"><div class="card-head"><h2>姿态示教与版本</h2><span class="badge lab">草稿不会自动执行</span></div>
        <div class="plan-inputs"><label>候选姿态<select id="pose-candidate"><option value="">选择旧工程候选…</option></select></label>
        <button id="pose-candidate-load">填入候选草稿</button><button id="pose-capture">读取当前模型到草稿</button></div>
        <div class="plan-inputs"><label>姿态编号（字母/数字/下划线）<input id="pose-id" maxlength="48" value="pose_1"></label><label>名称<input id="pose-name" maxlength="64" value="示教姿态1"></label></div>
        <div class="mech-targets">${all.map(axis => `<label>${names[axis]} / ${axes.includes(axis)?'mm':'°'}<input type="number" id="teach-${axis}" min="${axis==='arm'?83:0}" max="${({lift:49,reach:50,arm:180,tray:270,gripper:98})[axis]}" step="0.1" value="${({lift:0,reach:0,arm:180,tray:30,gripper:0})[axis]}"></label>`).join('')}</div>
        <div class="button-row"><button id="pose-preview">预览逐步过渡</button><button id="pose-save">保存新版本</button></div>
        <ol id="mech-preview" class="mech-preview"></ol>
        <div class="plan-inputs"><label>已保存版本<select id="pose-version"><option value="">暂无示教版本</option></select></label><button id="pose-load">读取到草稿</button><button id="pose-run" class="primary">执行选中版本</button></div>
        <div class="button-row"><label><input type="checkbox" id="pose-single-step">每一步执行前确认（仅离线）</label><button id="pose-next" disabled>确认执行当前步</button></div>
        <p id="pose-stage" class="hint">舵机命令完成与实际姿态确认分开；当前仅提供模型证据。</p>
        <p class="hint">保存只记录相对基准的SI目标与校验，不记录有效零点或真实成功。读取不运动。执行需草稿与选中版本一致、当前零点有效；每一步独立停稳。历史版本保留，重启不自动建零或续跑。</p>
       </article>
       <article class="card mech-wide"><div class="card-head"><h2>动作与互锁记录</h2><span id="mech-step" class="badge muted">无动作</span></div><p id="mech-active"></p><div id="mech-results" class="mech-results"></div>
        <p class="hint">本轮只验证软件状态机。碰撞互锁阈值为保守代理，未建立实物包络。机构动作不会增加真实抓放计数。</p>
       </article>
      </div>`;
    initTransfer($('mechanism-root'),api);
    $('mech-claim').onclick = () => $('claim').click();
    $('mech-stop').onclick = () => { cancelMechHeld();
        if (snapshotBefore?.program?.busy) api.command('program',{action:'stop'}).catch(error=>api.notify(error.message,true));
        else invoke('stop'); };
    $('pose-next').onclick = () => {
        if (!state?.active) return;
        invoke('next',{action_id:state.active.id,step_index:state.active.step,revision:state.active.pose_revision});
    };
    $('mech-zero').onclick = () => invoke('zero',{confirmed:$('mech-confirm').checked});
    $('mech-anchor').onclick = () => invoke('anchor',{confirmed:$('mech-confirm').checked});
    $('mech-invalidate').onclick = () => invoke('invalidate');
    $('mech-reset').onclick = () => invoke('reset_model',{confirmed:$('mech-confirm').checked});
    $('mech-axis').onchange = () => {
        const max = $('mech-axis').value === 'lift' ? 49 : 50;
        $('axis-target').max = max;
        $('axis-target').value = 0;
        $('axis-speed').max = max === 49 ? 3 : 5;
    };
    $('mech-servo').onchange = () => {
        const axis = $('mech-servo').value, limits = state?.config.servos[axis];
        if (!limits) return;
        $('servo-target').min = limits.min_deg; $('servo-target').max = limits.max_deg;
        $('servo-target').value = limits.anchor_deg;
    };
    $('axis-move').onclick = () => {
        try { invoke('move',{axis:$('mech-axis').value,target_m:numeric('axis-target') / 1000}); }
        catch (error) { api.notify(error.message,true); }
    };
    $('servo-move').onclick = () => {
        try {
            const release = $('mech-release').checked;
            const target = numeric('servo-target') * Math.PI / 180;
            $('mech-release').checked = false;
            invoke('move',{axis:$('mech-servo').value,target_rad:target,release_confirmed:release});
        }
        catch (error) { api.notify(error.message,true); }
    };
    document.querySelectorAll('.mech-jog').forEach(button => {
        button.onpointerdown = event => {
            if (event.button !== 0 || !enabled || held || button.disabled) return;
            event.preventDefault(); button.setPointerCapture(event.pointerId);
            try { held = {button,axis:$('mech-axis').value,speed:Number(button.dataset.sign) * numeric('axis-speed') / 1000}; button.classList.add('pressed'); pulse(); }
            catch (error) { cancelMechHeld(); api.notify(error.message,true); }
        };
        for (const event of ['pointerup','pointercancel','lostpointercapture']) button.addEventListener(event,stopMechHeld);
        button.oncontextmenu = event => event.preventDefault();
    });
    for (const event of ['blur','pagehide','offline']) window.addEventListener(event,stopMechHeld);
    document.addEventListener('visibilitychange',() => { if (document.hidden) stopMechHeld(); });
    $('pose-candidate-load').onclick = () => {
        const value = state?.config.candidates.find(p => p.id === $('pose-candidate').value);
        if (!value) return;
        selected = null;
        loadTargets({lift:value.lift_m,reach:value.reach_m,arm:value.arm_deg*Math.PI/180,tray:value.tray_deg*Math.PI/180,gripper:value.gripper_deg*Math.PI/180},value.id,value.name);
    };
    $('pose-capture').onclick = () => {
        if (!state?.zero_valid || state.servos.some(s => s.estimate_rad == null)) { api.notify('请先建立模型基准并锚定舵机估计',true); return; }
        loadTargets(Object.fromEntries([...state.axes.map(a => [a.id,a.position_m]),...state.servos.map(s => [s.id,s.estimate_rad])]));
    };
    $('pose-preview').onclick = async () => {
        try { const value = await invoke('preview',poseDraft()); if (value) showSteps(value.steps); }
        catch (error) { api.notify(error.message,true); }
    };
    $('pose-save').onclick = async () => {
        try { const value = await invoke('teach',poseDraft()); if (value) { selected=value.pose; versions=''; api.notify(`已保存 ${value.pose.name} v${value.pose.revision}；未执行。`); } }
        catch (error) { api.notify(error.message,true); }
    };
    $('pose-load').onclick = () => {
        selected = state?.poses.find(p => `${p.id}:${p.revision}` === $('pose-version').value);
        if (selected) loadTargets(selected.targets,selected.id,selected.name);
    };
    $('pose-run').onclick = () => {
        try {
            const pose = state?.poses.find(p => `${p.id}:${p.revision}` === $('pose-version').value);
            if (!pose || !sameTargets(targets(),pose.targets) || $('pose-id').value !== pose.id) throw new Error('请先读取选中版本；草稿改动需保存后再执行');
            const release = $('mech-release').checked;
            $('mech-release').checked = false;
            invoke('pose',{pose_id:pose.id,revision:pose.revision,release_confirmed:release,
                ...(state.execution_schema>=2?{single_step:$('pose-single-step').checked}:{})});
        } catch (error) { api.notify(error.message,true); }
    };
    setInterval(pulse,100);
    initAcceptance();
    initCompact($('mechanism-root'),{targets,invoke,notify:api.notify,loadDevice:initDevice});
    initProgram({...api,planId:()=>snapshotBefore?.planning?.status==='READY'?snapshotBefore.planning.plan.id:null,
        selectedPose:()=>{
            const pose=state?.poses.find(p=>`${p.id}:${p.revision}`===$('pose-version').value);
            if(!pose||!sameTargets(targets(),pose.targets)||$('pose-id').value!==pose.id)
                throw new Error('先保存姿态并读取对应版本；草稿改动不能当作已保存目标');
            return pose;
        }});
    // 接入准备只读本地源码；不接管控制核心或重建用户的模拟任务。
    fetch('/api/workbench/preparation',{cache:'no-store',signal:AbortSignal.timeout(2000)})
        .then(async response=>{
            if(response.ok) return response.json();
            // 旧网关继续运行时，沿用已有归档GET读取独立的离线准备字段。
            const archived=await fetch('/api/workbench/acceptance',{cache:'no-store',signal:AbortSignal.timeout(2000)});
            if(!archived.ok) throw new Error('准备清单不可读');
            return (await archived.json()).offline_preparation;
        })
        .then(value=>{if(value?.domain==='preparation' && !value.real_connected && !value.real_motion) {prepared=value;preparationKey='';}})
        .catch(()=>{});
}

let snapshotBefore = null;
export function renderMechanism(snapshot, canControl) {
    snapshotBefore = snapshot;
    renderTransfer(snapshot,canControl && !snapshot?.program?.busy);
    renderCompact(snapshot,canControl);
    renderProgram(snapshot,canControl);
    state = snapshot?.mechanism;
    enabled = !!canControl && !snapshot?.program?.busy;
    if (!state) {
        cancelMechHeld(); $('mech-mode').textContent = '状态过期'; $('mech-reason').textContent = '无法确认机构模型状态';
        $('mech-values').replaceChildren(); $('mech-bars').replaceChildren();
        document.querySelectorAll('#mechanism-root button').forEach(button => {button.disabled = true;});
        return;
    }
    const stopped = snapshot.simulation.stopped && snapshot.simulation.mode !== 'FAILED';
    const ready = enabled && stopped && !state.busy;
    const preparation = prepared || snapshot.integration;
    const preparationValue = JSON.stringify(preparation);
    if (preparation && preparationKey !== preparationValue) {
        preparationKey = preparationValue;
        const rows = [...preparation.axes.map(a => `${a.name}：CAN ID${a.address}，${a.direction}，初始限程${a.max_mm}mm`),
            ...preparation.servos.map(s => `${s.name}：${s.pin} / TIM1_CH${s.channel}`),
            `人工基准：${preparation.baseline}`, `接线：${preparation.wiring_confirmed?'用户已确认，待本次只读反馈核对':'待确认'}`];
        if (preparation.axis_executor) rows.push('单轴执行器：正式固件已接入，实车反馈与动作仍待本次验收',
            `到位证据：${preparation.axis_executor.completion}`);
        if (preparation.servo_executor) rows.push('舵机/姿态执行器：正式固件已接入；命令完成与姿态确认分开，实际动作待验收');
        $('mech-mapping').replaceChildren(...rows.map(value => {const item=document.createElement('p');item.textContent=value;return item;}));
        const pendingItems=['板端状态见上方带时间的只读验收记录；每次接入都需重新核对',...preparation.pending.slice(1)];
        $('mech-pending').replaceChildren(...pendingItems.map(value => {const item=document.createElement('li');item.textContent=value;return item;}));
        $('tray-presets').replaceChildren(...[preparation.tray_reference_deg,...preparation.tray_slot_deg].map(angle => {
            const button=document.createElement('button');button.textContent=`料盘${angle}°${angle===preparation.tray_reference_deg?' · 基准':''}`;
            button.onclick=() => {$('mech-servo').value='tray';$('mech-servo').onchange();$('servo-target').value=angle;$('teach-tray').value=angle;api.notify(`已填入料盘${angle}°草稿，未执行。`);};return button;
        }));
    }
    document.querySelectorAll('#tray-presets button').forEach(button => {button.disabled=state.busy;});
    $('mech-mode').textContent = `${modes[state.mode] || state.mode} · 基准${state.zero_valid ? `v${state.zero_version}`:'无效'}`;
    $('mech-reason').textContent = state.fault || state.reason;
    $('mech-claim').hidden = enabled;
    $('mech-claim').disabled = !snapshot.available || (!!snapshot.owner.client && !enabled);
    $('mech-stop').disabled = !canControl;
    $('pose-single-step').disabled = state.busy || !(state.execution_schema>=2);
    $('pose-next').disabled = !enabled || !stopped || !state.active?.waiting_step || !!state.fault;
    $('pose-stage').textContent = state.active ? `${phaseNames[state.active.phase] || '模型动作执行中'} · ${state.active.pose_revision?`姿态v${state.active.pose_revision} · `:''}仅模拟证据，实际舵机角度未知` :
        state.execution_schema>=2?'命令完成→模型确认分开显示；逐步模式每次只放行一步，实际角度始终未知。':'当前核心仍为旧模型；新增逐步模式待离线核心更新，原控制可继续模拟。';
    for (const id of ['mech-zero','mech-anchor','mech-reset']) $(id).disabled = !ready;
    $('mech-invalidate').disabled = !enabled;
    for (const id of ['axis-move','servo-move','pose-preview','pose-save','pose-run']) $(id).disabled = !ready || !state.zero_valid || !!state.fault;
    document.querySelectorAll('.mech-jog').forEach(button => {button.disabled = !enabled || !stopped || !state.zero_valid || !!state.fault || (state.busy && !held);});
    $('pose-capture').disabled = !ready;
    $('pose-candidate-load').disabled = state.busy;
    for (const id of ['mech-axis','axis-target','axis-speed','mech-servo','servo-target']) $(id).disabled = state.busy;
    const checks = [
        ['底盘模型',stopped?'已停车':'未停车 / 故障'],['模拟零点',state.zero_valid?`有效 v${state.zero_version}`:'未建立 / 已失效'],
        ['运输代理互锁',state.transport_ready?'满足':'未满足；建立机构基准后限制底盘'],
        ['旋转', '升降/伸缩≤1mm；料盘转动时转臂180°'],
        ['伸出','升降≥45mm且转臂180°'],['升降','伸缩先收回≤1mm'],
        ['保夹',state.hold_possible?'可能持料锁存（仅模型），张爪需确认':'无保持命令锁存；真实物料未知'],
        ['示教存储',state.storage_error || '正常；仅保存草稿版本']];
    $('mech-interlocks').replaceChildren(...checks.map(([label,value]) => { const row=document.createElement('div'),dt=document.createElement('dt'),dd=document.createElement('dd');dt.textContent=label;dd.textContent=value;row.append(dt,dd);return row; }));
    $('mech-values').replaceChildren(...[...state.axes.map(a => [a.id,a.position_m,'未知']),...state.servos.map(s => [s.id,s.estimate_rad,`未知 · PWM预览${s.pulse_us ?? '—'}µs`])].map(([axis,value,feedback]) => {
        const row=document.createElement('tr'); for (const text of [names[axis],fmt(axis,value),feedback]) {const cell=document.createElement('td');cell.textContent=text;row.append(cell);}return row;
    }));
    $('mech-bars').replaceChildren(...state.axes.map(a => {
        const label=document.createElement('label'),bar=document.createElement('progress');label.textContent=`${names[a.id]}模型行程`;
        bar.max=a.limits_m[1];bar.value=a.position_m ?? 0;label.append(bar);return label;
    }));
    if (!$('pose-candidate').dataset.loaded) {
        for (const value of state.config.candidates) {const option=document.createElement('option');option.value=value.id;option.textContent=value.name;$('pose-candidate').append(option);}
        $('pose-candidate').dataset.loaded='1';
    }
    const key=JSON.stringify(state.poses);
    if (versions !== key) {
        versions=key;
        const old=$('pose-version').value;
        $('pose-version').replaceChildren();
        for (const pose of [...state.poses].reverse()) {const option=document.createElement('option');option.value=`${pose.id}:${pose.revision}`;option.textContent=`${pose.name} / ${pose.id} v${pose.revision} · 待实测`;$('pose-version').append(option);}
        if (!state.poses.length) {const option=document.createElement('option');option.value='';option.textContent='暂无示教版本';$('pose-version').append(option);}
        if (selected) $('pose-version').value=`${selected.id}:${selected.revision}`;
        else if (state.poses.some(p => `${p.id}:${p.revision}`===old)) $('pose-version').value=old;
    }
    $('pose-load').disabled = !ready || !$('pose-version').value;
    $('mech-step').textContent=state.active?`#${state.active.id} · ${state.active.step}/${state.active.total}`:'无运行中动作';
    $('mech-active').textContent=state.active?`${statusNames[state.active.status]} · ${state.active.action} · 零点v${state.active.zero_version}`:'不自动执行示教或重放上次动作';
    if (snapshot.transfer && ['RUNNING','WAIT_OBSERVATION'].includes(snapshot.transfer.mode)) {
        for(const id of ['mech-zero','mech-anchor','mech-reset','axis-move','servo-move','pose-preview','pose-save','pose-run','pose-next'])$(id).disabled=true;
        document.querySelectorAll('.mech-jog').forEach(b=>{b.disabled=true;});
    }
    const resultKey=JSON.stringify(state.results);
    if (activeBefore!==resultKey) {
        activeBefore=resultKey;
        $('mech-results').replaceChildren(...[...state.results].reverse().slice(0,12).map(result=>{const row=document.createElement('p');row.textContent=`#${result.action_id} ${statusNames[result.status]}：${result.evidence}`;return row;}));
    }
}
