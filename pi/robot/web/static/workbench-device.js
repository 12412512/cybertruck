/* 实车通道独立于离线模型：输入仅编辑，所有动作显式执行。 */
export function initDevice(root) {
    const card = document.createElement('article');
    card.className = 'card mech-preparation physical-channel';
    card.innerHTML = `<div class="card-head"><h2>实车机构执行</h2><span id="real-status">未连接</span></div>
    <p>使用唯一设备核心。模拟零点和模拟示教不能驱动实车。本版先开放定距、舵机及逐步姿态；连续点动稍后实测。</p>
    <p id="real-message">等待只读状态</p><pre id="real-feedback"></pre>
    <button id="real-claim">获取实车控制权</button><button id="real-attach">接入机构反馈（不使能）</button>
    <button id="real-stop" class="danger">停止实车 · 保持夹爪</button>
    <p><label><input id="real-confirm" type="checkbox">我已核对现场及人工基准，允许本次明确操作</label></p>
    <button id="real-enable-lift">使能升降</button><button id="real-enable-reach">使能伸缩</button><button id="real-zero">确认最高/最近端并建零</button>
    <div class="field-grid"><label>料盘当前参考/°<input id="real-seed-tray" type="number" value="0" min="0" max="270"></label>
    <label>转臂当前参考/°<input id="real-seed-arm" type="number" value="180" min="83" max="180"></label>
    <label>夹爪当前参考/°<input id="real-seed-gripper" type="number" value="98" min="0" max="98"></label></div>
    <button id="real-anchor">记录人工核对的舵机参考（不输出）</button>
    <div class="field-grid"><label>执行器<select id="real-axis"><option value="lift">升降</option><option value="reach">伸缩</option><option value="tray">料盘</option><option value="arm">转臂</option><option value="gripper">夹爪</option></select></label>
    <label>绝对目标 / mm或°<input id="real-target" type="number" value="0"></label><label>单轴速度 / RPM<input id="real-rpm" type="number" value="5" min="1" max="30"></label></div>
    <label><input type="checkbox" id="real-release">明确允许本次张爪释放</label>
    <button id="real-move">执行实车目标</button><button id="real-evidence">人工确认当前舵机姿态</button>
    <h3>实车姿态示教版本</h3><p>此处保存独立实车草稿。保存不运动；执行仍需当前实车零点、首次PWM逐通道验收和互锁通过。</p>
    <label>编号<input id="real-pose-id" value="physical_pose_1" maxlength="48"></label>
    <div class="field-grid">${['lift','reach','arm','tray','gripper'].map((name,i)=>`<label>${['升降/mm','伸缩/mm','转臂/°','料盘/°','夹爪/°'][i]}<input id="real-pose-${name}" type="number" value="${[0,0,180,0,98][i]}"></label>`).join('')}</div>
    <button id="real-teach">保存实车草稿新版本</button><select id="real-poses"></select>
    <button id="real-pose">执行选中版本（逐步确认）</button><button id="real-next">放行当前实车步骤</button>
    <p>命令已接受不等于实物到位。舵机等待结束后仍须人工确认；故障和失联不自动恢复、不自动张爪。</p>`;
    root.append(card);
    const el = id => card.querySelector(`#real-${id}`);
    for (const button of card.querySelectorAll('button'))
        if (!['real-claim','real-stop'].includes(button.id)) button.disabled = true;
    const client = `physical_${crypto.randomUUID().replaceAll('-','')}`;
    let token = '', state = {}, pending = '', polling = false, sending = false, poseKey = '';
    const message = text => { el('message').textContent = text; };
    const checked = () => { if (!el('confirm').checked) throw Error('请先核对现场并勾选本次明确操作'); };
    async function send(op,args={}) {
        const response = await fetch('/api/device/command', {method:'POST',headers:{'Content-Type':'application/json','X-Workbench':'1'},
            body:JSON.stringify({op,args,client,token,request_id:crypto.randomUUID().replaceAll('-','')})});
        const value = await response.json();
        if (!response.ok) throw Error(value.detail || '实车核心未连接');
        if (value.token) token = value.token;
        if (value.status === 'QUEUED') pending = value.request_id;
        return value;
    }
    async function act(action,args={}) {
        if (sending) return;
        sending = true;
        try {
            if (!['attach','invalidate'].includes(action)) checked();
            const value = await send('mechanism',{action,zero_version:state.mechanism?.zero_version || 0,...args});
            message(value.status === 'QUEUED' ? '请求已排队，尚未证明执行或到位' : value.status);
            el('confirm').checked = false;
        } catch(error) { message(error.message); }
        finally { sending = false; }
    }
    el('claim').onclick = async()=>{try {await send('claim');message('已取得实车控制权；尚未使能或动作');}catch(e){message(e.message);}};
    el('attach').onclick = ()=>act('attach');
    el('stop').onclick = async()=>{try {await send('stop');token='';pending='';message('已请求故障停车；等待实车反馈');}catch(e){message(e.message);}};
    for (const axis of ['lift','reach']) el(`enable-${axis}`).onclick=()=>act('enable',{axis,confirmed:true});
    el('zero').onclick=()=>act('zero',{confirmed:true});
    el('anchor').onclick=()=>act('anchor',{confirmed:true,targets:Object.fromEntries(['tray','arm','gripper'].map(name=>[name,Number(el(`seed-${name}`).value)*Math.PI/180]))});
    el('move').onclick=()=>{
        const axis=el('axis').value,linear=['lift','reach'].includes(axis), target=Number(el('target').value);
        act('move',{axis,[linear?'target_m':'target_rad']:target*(linear?.001:Math.PI/180),rpm:Number(el('rpm').value),release_confirmed:el('release').checked});
    };
    el('evidence').onclick=()=>act('confirm',{confirmed:true});
    el('teach').onclick=()=>act('teach',{pose_id:el('pose-id').value,targets:Object.fromEntries(['lift','reach','arm','tray','gripper'].map(name=>[name,Number(el(`pose-${name}`).value)*(['lift','reach'].includes(name)?.001:Math.PI/180)]))});
    el('pose').onclick=()=>{const selected=(state.poses||[]).find(p=>`${p.id}:${p.revision}`===el('poses').value);if(selected)act('pose',{pose_id:selected.id,revision:selected.revision,physical_taught:true,single_step:true,release_confirmed:el('release').checked});};
    el('next').onclick=()=>act('next',{action_id:state.mechanism?.pose_id,revision:state.mechanism?.revision,step_index:state.mechanism?.step_index});
    async function poll() {
        if (polling || document.hidden) return;
        polling=true;
        try {
            const response=await fetch('/api/device/state'); state=await response.json();
            const m=state.mechanism;
            el('status').textContent=state.connected?'实车状态实时':'未连接或状态过期';
            el('feedback').textContent=m?`固件 ${state.firmware}\n故障 ${state.base?.fault}/${m.fault} · 底盘停车 ${state.base?.stopped?'已确认':'未确认'} · 机构反馈 ${m.attached?'已接入':'未接入'}\n零点 v${m.zero_version} / ${m.zero_valid?'有效':'无效'} · 配置 ${m.config_mask}/3\n轴状态 ${m.axis_state} 原因 ${m.axis_reason} · 舵机状态 ${m.servo_state} 原因 ${m.servo_reason}\n姿态 ${m.pose_id} v${m.revision} 步骤 ${m.step_index}/${m.step_count} 状态 ${m.pose_state}\n升降 ${m.zero_valid?m.lift_um/1000:'未知'}mm · 伸缩 ${m.zero_valid?m.reach_um/1000:'未知'}mm\nPWM通道位图 ${m.pwm_mask} · 实际舵机角度：未知`:(state.reason||'没有实车状态');
            const result=(state.results||[]).find(r=>r.request_id===pending);
            if(result){pending='';message(result.result?`板端拒绝，原因 ${result.result}`:'板端已接受；完成状态另行确认');}
            const busy=m&&((m.axis_state>=1&&m.axis_state<=5)||(m.servo_state>=1&&m.servo_state<=5)||(m.pose_state>=1&&m.pose_state<=5));
            for(const button of card.querySelectorAll('button'))if(!['real-stop','real-claim'].includes(button.id))button.disabled=!token||!state.connected||!!pending||!!busy;
            el('next').disabled=!token||!!pending||m?.pose_state!==4;
            el('evidence').disabled=!token||!!pending||m?.servo_state!==5;
            el('attach').disabled=!token||!state.connected||!!pending||!!m?.attached;
            if (!state.site_ready) {
                for (const name of ['enable-lift','enable-reach','zero','anchor','move','evidence','pose','next']) el(name).disabled=true;
                el('status').textContent=state.connected?'实车只读 · 等待本次现场确认':'未连接';
            }
            const key=JSON.stringify((state.poses||[]).map(p=>[p.id,p.revision]));
            if(key!==poseKey){poseKey=key;el('poses').replaceChildren(...(state.poses||[]).map(p=>new Option(`${p.id} v${p.revision} · 待实测`,`${p.id}:${p.revision}`)));}
        }catch(e){
            state={connected:false};
            el('status').textContent='实车状态未知';
            el('feedback').textContent='状态读取失败；旧反馈已失效';
            for(const button of card.querySelectorAll('button'))
                if(!['real-stop','real-claim'].includes(button.id))button.disabled=true;
            message('实车核心未连接，禁止动作');
        }finally{polling=false;}
    }
    setInterval(poll,200); poll();
    setInterval(async()=>{if(token&&!document.hidden)try{await send('renew');}catch(e){token='';message(e.message);}},400);
    const release=()=>{if(token){fetch('/api/device/command',{method:'POST',keepalive:true,headers:{'Content-Type':'application/json','X-Workbench':'1'},body:JSON.stringify({op:'stop',client,token,request_id:crypto.randomUUID().replaceAll('-','')})}).catch(()=>{});token='';}};
    window.addEventListener('pagehide',release);
    window.addEventListener('offline',release);
    document.addEventListener('visibilitychange',()=>{if(document.hidden)release();});
}
