import {stateName, samePose, rejectionText} from './workbench-mech-guide.js?v=2';
import {actuators, freshDevice, blockedReasons, allowed, moveArgs} from './workbench-remote-state.js?v=2';

/* 仅访问实车网关；页面初始化只读取，不自动取得控制权或恢复动作。 */
export function initDevice(root) {
    const card = document.createElement('div');
    card.className = 'remote-mechanism';
    card.innerHTML = `
      <div class="section-intro remote-intro"><div><span class="eyebrow">LIVE / MECHANISM</span>
        <h2>机械远程控制</h2><p>两轴 · 三舵机 · 实车姿态。执行请求统一交给设备核心。</p></div>
        <span id="real-status" class="badge muted">读取设备状态</span></div>
      <article class="card remote-status"><div class="remote-metrics">
        <div><span>固件</span><strong id="real-firmware">—</strong></div>
        <div><span>故障 · 底盘 / 机构</span><strong id="real-fault">未知</strong></div>
        <div><span>停车状态</span><strong id="real-stopped">未知</strong></div>
        <div><span>人工零点</span><strong id="real-zero-state">无有效反馈</strong></div>
        <div><span>PWM通道</span><strong id="real-pwm">未知</strong></div></div>
        <div class="remote-toolbar"><button id="real-claim">获取控制权</button><button id="real-attach">接入机构反馈</button>
          <span id="real-owner">未持有控制权</span><button id="real-stop" class="danger">停止实车 · 保持夹爪</button></div>
        <p id="real-blockers" class="remote-blockers"></p>
        <p id="real-message" role="status" aria-live="polite">输入只修改目标；点击执行才发送实车请求。</p>
        <label class="remote-confirm"><input id="real-confirm" type="checkbox">现场及目标已核对，允许下一项明确操作</label>
      </article>
      <div class="remote-control-grid"><article class="card"><div class="card-head"><h2>直线轴</h2><span class="badge muted">绝对位置 · mm</span></div>
        ${actuators.slice(0,2).map(a=>`<div class="remote-actuator"><div class="remote-actuator-title"><b>${a.name}</b><small>${a.pin}</small><span id="real-value-${a.id}">位置未知</span></div>
          <div class="remote-inputs"><label>目标 / mm<input id="real-target-${a.id}" type="number" min="${a.min}" max="${a.max}" step="0.5" value="0"></label>
            <label>速度 / RPM<input id="real-rpm-${a.id}" type="number" min="1" max="30" step="1" value="5"></label>
            <button id="real-enable-${a.id}">使能${a.name}</button><button id="real-move-${a.id}" class="primary">执行${a.name}</button></div></div>`).join('')}
        <div class="remote-foot"><button id="real-zero">确认最高 / 最近端并建零</button><span>下降、伸出为正。初期限程49 / 50mm。</span></div>
      </article><article class="card"><div class="card-head"><h2>舵机</h2><span class="badge muted">命令角度 · °</span></div>
        ${actuators.slice(2).map(a=>`<div class="remote-actuator"><div class="remote-actuator-title"><b>${a.name}</b><small>${a.pin}</small><span id="real-value-${a.id}">实际角度未知</span></div>
          <div class="remote-servo-row"><label>目标 / °<input id="real-target-${a.id}" type="number" min="${a.min}" max="${a.max}" step="1" value="${a.initial}"></label>
            <button id="real-move-${a.id}" class="primary">执行${a.name}</button></div>
          ${a.id==='tray'?`<div class="remote-presets">${[0,30,150,270].map(n=>`<button data-tray="${n}">${n}°${n===0?' 基准':''}</button>`).join('')}</div>`:''}</div>`).join('')}
        <label class="remote-confirm"><input id="real-release" type="checkbox">明确允许本次夹爪张开 / 释放</label>
        <div id="real-confirmation" class="remote-confirmation" hidden><p id="real-confirmation-text"></p><button id="real-evidence">人工确认当前舵机姿态</button></div>
        <p class="hint">舵机没有角度反馈；命令完成后需及时人工确认。PE14本页不开放。</p>
      </article></div>
      <details class="card remote-setup"><summary>首次准备 · 记录当前舵机参考</summary>
        <p class="hint">必须与当前实物相符；默认值不能当作实际角度。记录参考不输出PWM，首次执行从该参考开始。</p>
        <div class="remote-reference-fields">${actuators.slice(2).map(a=>`<label>${a.name}当前参考 / °<input id="real-seed-${a.id}" type="number" min="${a.min}" max="${a.max}" step="1" value="${a.initial}"></label>`).join('')}</div>
        <button id="real-anchor">记录已核对的参考角</button>
      </details>
      <details class="card remote-poses"><summary>实车姿态 · 保存与调用</summary>
        <p class="hint">保存只记录目标版本，不运动。调用由板端执行安全过渡，逐步放行。</p>
        <div class="remote-pose-heading"><label>姿态编号<input id="real-pose-id" value="physical_pose_1" maxlength="48"></label>
          <label>已保存版本<select id="real-poses"><option value="">暂无保存版本</option></select></label><button id="real-pose-load">读取选中版本</button></div>
        <div class="remote-pose-fields">${actuators.map(a=>`<label>${a.name} / ${a.unit}<input id="real-pose-${a.id}" type="number" min="${a.min}" max="${a.max}" step="0.1" value="${a.initial}"></label>`).join('')}</div>
        <div class="remote-toolbar"><button id="real-teach">保存为新版本</button><button id="real-pose" class="primary">执行选中姿态</button><button id="real-next">放行当前步骤</button></div>
        <p id="real-version-note" class="hint"></p>
      </details>
      <details class="card remote-diagnostics"><summary>实时反馈与本页请求记录</summary><pre id="real-feedback">等待设备</pre><div id="real-history"></div></details>`;
    root.append(card);
    const el=id=>card.querySelector(`#real-${id}`);
    const client=`physical_${crypto.randomUUID().replaceAll('-','')}`;
    let token='', state={}, receivedAt=0, pending='', pendingAction='', pendingLabel='', pendingAxis='';
    let polling=false, sending=false, poseKey='', identity='', heartbeatBusy=false, renewalUncertain=false;
    const acceptedEnables=new Set(), entries=[];
    const message=text=>{el('message').textContent=text;};
    function log(text) {
        entries.unshift(`${new Date().toLocaleTimeString('zh-CN',{hour12:false})} · ${text}`);
        entries.length=Math.min(entries.length,30);
        el('history').replaceChildren(...entries.map(text=>{const p=document.createElement('p');p.textContent=text;return p;}));
    }
    const current=()=>({...state,connected:freshDevice(state,receivedAt,performance.now())});
    const targets=()=>Object.fromEntries(actuators.map(a=>[a.id,number(`pose-${a.id}`)*(a.unit==='mm'?.001:Math.PI/180)]));
    const selected=()=>(state.poses||[]).find(p=>`${p.id}:${p.revision}`===el('poses').value);
    function number(id) {
        const input=el(id),value=input.valueAsNumber;
        if(!input.value.trim() || !Number.isFinite(value) || !input.checkValidity())throw Error('请填写范围内的明确目标');
        return value;
    }
    function label(action,args) {
        const name=actuators.find(a=>a.id===args.axis)?.name||'';
        return ({enable:`使能${name}`,move:`${name}目标`,zero:'人工建零',anchor:'记录舵机参考',attach:'接入反馈',
            confirm:'人工确认舵机姿态',teach:'保存姿态',pose:'调用姿态',next:'放行步骤'})[action]||action;
    }
    async function send(op,args={}) {
        const requestId=crypto.randomUUID().replaceAll('-',''),modifying=op==='mechanism';
        if(modifying){pending=requestId;pendingAction=args.action;pendingLabel=label(args.action,args);pendingAxis=args.axis||'';}
        let response;
        try {
            response=await fetch('/api/device/command',{method:'POST',headers:{'Content-Type':'application/json','X-Workbench':'1'},
                body:JSON.stringify({op,args,client,token,request_id:requestId})});
            const value=await response.json();
            if(!response.ok){
                if(modifying && response.status<500)pending='';
                const error=Error(value.detail||'设备核心拒绝请求');
                error.status=response.status;
                throw error;
            }
            if(value.token)token=value.token;
            if(modifying && value.status!=='QUEUED'){pending='';log(`${pendingLabel}：${value.status==='SAVED_DRAFT'?'已保存新版本，未执行':value.status}`);}
            return value;
        }catch(error){
            if(!response && modifying)message(`${pendingLabel}结果未知，禁止重复发送；请核对反馈或停止`);
            throw error;
        }
    }
    async function act(action,args={}) {
        if(sending)return;
        sending=true;
        try {
            if(!allowed(current(),!!token,pending||renewalUncertain,action))throw Error('实时反馈、控制权、故障或互锁条件未满足');
            if(!['attach','teach'].includes(action) && !el('confirm').checked)throw Error('请勾选本次现场及目标确认');
            const value=await send('mechanism',{action,zero_version:state.mechanism?.zero_version||0,...args});
            message(value.status==='QUEUED'?`${label(action,args)}请求已排队，等待板端结果`:'姿态已保存，未发送动作');
            el('confirm').checked=false;
        }catch(error){message(error.message);log(error.message);}
        finally{if(args.release_confirmed)el('release').checked=false;sending=false;draw();}
    }
    async function stop(source='panel-stop-button') {
        token=''; // 停止请求等待期间不再续约；请求丢失仍由租约保护停车。
        renewalUncertain=false;
        try{await send('stop',{source});message('停止请求已发送；动作锁定，保留实时反馈，不自动恢复或张爪');log(`停止来源：${source}`);}
        catch(error){message(`停止未确认：${error.message}；请核对实物状态`);}
        finally{draw();}
    }
    const release=(source='page-release')=>{
        if(!token)return;
        const body={op:'release',client,token,request_id:crypto.randomUUID().replaceAll('-',''),args:{source}};token='';renewalUncertain=false;
        fetch('/api/device/command',{method:'POST',keepalive:true,headers:{'Content-Type':'application/json','X-Workbench':'1'},body:JSON.stringify(body)})
            .catch(()=>message('退出时停止未确认，请核对车辆；主控保护继续生效'));
        draw();
    };
    el('claim').onclick=async()=>{
        if(sending)return;sending=true;
        try{await send('claim');message('已取得控制权；尚未使能或动作');}catch(error){message(error.message);}
        finally{sending=false;draw();}
    };
    el('attach').onclick=()=>act('attach');el('stop').onclick=()=>stop('panel-stop-button');
    for(const a of actuators){
        el(`move-${a.id}`).onclick=()=>{try{act('move',moveArgs(a.id,el(`target-${a.id}`).value,a.unit==='mm'?number(`rpm-${a.id}`):5,el('release').checked));}catch(error){message(error.message);}};
        if(a.unit==='mm')el(`enable-${a.id}`).onclick=()=>act('enable',{axis:a.id,confirmed:true});
    }
    for(const button of card.querySelectorAll('[data-tray]'))button.onclick=()=>{el('target-tray').value=button.dataset.tray;message(`料盘目标已填为${button.dataset.tray}°，尚未执行`);};
    el('zero').onclick=()=>act('zero',{confirmed:true});
    el('anchor').onclick=()=>{try{act('anchor',{confirmed:true,targets:Object.fromEntries(actuators.slice(2).map(a=>[a.id,number(`seed-${a.id}`)*Math.PI/180]))});}catch(error){message(error.message);}};
    el('evidence').onclick=()=>act('confirm',{confirmed:true});
    el('teach').onclick=()=>{try{act('teach',{pose_id:el('pose-id').value,targets:targets()});}catch(error){message(error.message);}};
    el('pose-load').onclick=()=>{
        const record=selected();if(!record)return;el('pose-id').value=record.id;
        for(const a of actuators)el(`pose-${a.id}`).value=+(record.targets[a.id]*(a.unit==='mm'?1000:180/Math.PI)).toFixed(5);
        message('已读取姿态目标，未发送动作');draw();
    };
    el('pose').onclick=()=>{
        const record=selected();if(!record)return;
        try{if(!samePose(targets(),record))throw Error('当前目标与选中版本不同，请先读取或保存新版本');
            act('pose',{pose_id:record.id,revision:record.revision,physical_taught:true,single_step:true,release_confirmed:el('release').checked});}
        catch(error){message(error.message);}
    };
    el('next').onclick=()=>act('next',{action_id:state.mechanism?.pose_id,revision:state.mechanism?.revision,step_index:state.mechanism?.step_index});
    card.addEventListener('input',draw);el('poses').onchange=draw;
    function draw(){
        const value=current(),m=value.mechanism;
        el('status').textContent=!value.connected?(state.connected?'反馈过期 · 动作暂停':'状态未知 / 连接中断'):value.action_locked?'动作已锁定 · 反馈在线':renewalUncertain?'控制权核对中 · 动作暂停':value.base?.fault||m?.fault?'故障锁定':!value.site_ready?'实车只读':'实车在线';
        el('firmware').textContent=value.firmware||'—';
        el('fault').textContent=value.connected?`${value.base?.fault??'—'} / ${m?.fault??'—'}`:'未知';
        el('stopped').textContent=value.connected?(value.base?.stopped?'已确认':'未确认'):'未知';
        el('zero-state').textContent=!value.connected?'未知':m?.zero_valid?`有效 · v${m.zero_version}`:'无效 / 未建立';
        el('pwm').textContent=value.connected&&m?actuators.slice(2).filter((a,i)=>m.pwm_mask&(1<<i)).map(a=>a.pin).join('、')||'全部关闭':'未知';
        el('owner').textContent=token?'本页持有控制权':'未持有控制权';
        el('blockers').textContent=blockedReasons(value,!!token,pending||renewalUncertain).join('；')||'接口就绪；每项动作仍由板端复核';
        const permit=action=>!sending&&allowed(value,!!token,pending||renewalUncertain,action);
        el('claim').disabled=!!token||!permit('claim');el('attach').disabled=!permit('attach');
        el('zero').disabled=!permit('zero')||!!m?.zero_valid;el('anchor').disabled=!permit('anchor');
        for(const a of actuators){
            el(`move-${a.id}`).disabled=!permit('move');
            if(a.unit==='mm'){
                const accepted=acceptedEnables.has(a.id);el(`enable-${a.id}`).disabled=!permit('enable')||accepted;
                el(`enable-${a.id}`).textContent=accepted?'使能请求已接受':`使能${a.name}`;
                const position=m?.[a.id==='lift'?'lift_um':'reach_um'];
                el(`value-${a.id}`).textContent=value.connected&&m?.zero_valid?`${(position/1000).toFixed(2)} mm`:'位置未知';
            }else el(`value-${a.id}`).textContent=value.connected&&m?.channel===actuators.slice(2).indexOf(a)+1?`命令 ${(m.target_mdeg/1000).toFixed(1)}° · 实际未知`:'实际角度未知';
        }
        el('confirmation').hidden=!(value.connected&&m?.servo_state===5);
        el('confirmation-text').textContent=m?`${actuators.slice(2)[m.channel-1]?.name||'舵机'}命令已完成。核对实物后勾选本次确认并点击下方按钮，板端超时保护仍生效。`:'';
        el('evidence').disabled=!permit('confirm');el('teach').disabled=!permit('teach');el('next').disabled=!permit('next');
        const record=selected();let matches=false;try{matches=!!record&&samePose(targets(),record);}catch{}
        el('pose-load').disabled=!record;el('pose').disabled=!permit('pose')||!matches;
        el('version-note').textContent=record?`${record.id} · v${record.revision} · ${record.sha256.slice(0,12)} · ${matches?'目标与版本一致':'当前目标不同，禁止调用'}`:'暂无实车姿态；保存不运动。';
        const safety=(value.safety_events||[]).slice(-5).map(e=>`${new Date(e.wall_s*1000).toLocaleTimeString('zh-CN',{hour12:false})} · ${e.source} · ${e.reason}`).join('\n');
        el('feedback').textContent=(value.connected&&m?`轴：${stateName('axis',m.axis_state)} / ${m.axis_reason}\n舵机：${stateName('servo',m.servo_state)} / ${m.servo_reason}\n姿态：${stateName('pose',m.pose_state)} · #${m.pose_id} v${m.revision} · ${m.step_index}/${m.step_count}\n反馈龄 ${value.state_age_ms}ms · 配置 ${m.config_mask}/3 · PWM位图 ${m.pwm_mask}\n启动 ${value.boot_id} · 会话 ${value.session_id}`:value.reason||'没有新鲜设备状态，不使用旧反馈授权动作')+(safety?`\n最近停止来源：\n${safety}`:'');
    }
    async function poll(){
        if(polling||document.hidden)return;polling=true;
        try{
            const response=await fetch('/api/device/state',{cache:'no-store'});if(!response.ok)throw Error('设备状态读取失败');
            state=await response.json();receivedAt=performance.now();const nextIdentity=`${state.boot_id}:${state.session_id}`;
            if(identity&&identity!==nextIdentity){token='';pending='';renewalUncertain=false;acceptedEnables.clear();message('设备身份已变化，旧控制权和参考无效；不自动恢复');}
            identity=nextIdentity;const result=(state.results||[]).find(r=>r.request_id===pending);
            if(result){
                const text=result.result?`${pendingLabel}：${rejectionText(pendingAction,result.result,state)}`:`${pendingLabel}：板端已接受，完成以反馈为准`;
                message(text);log(text);if(pendingAction==='enable'&&!result.result)acceptedEnables.add(pendingAxis);
                pending='';pendingAction='';pendingAxis='';
            }
            const key=JSON.stringify((state.poses||[]).map(p=>[p.id,p.revision]));
            if(key!==poseKey){
                poseKey=key;const old=el('poses').value;
                el('poses').replaceChildren(...(state.poses||[]).map(p=>new Option(`${p.id} · v${p.revision}`,`${p.id}:${p.revision}`)));
                if(!(state.poses||[]).length)el('poses').append(new Option('暂无保存版本',''));
                if((state.poses||[]).some(p=>`${p.id}:${p.revision}`===old))el('poses').value=old;
            }
        }catch(error){state={connected:false,reason:error.message};}
        finally{polling=false;draw();}
    }
    setInterval(poll,100);setInterval(draw,100);
    setInterval(async()=>{
        if(!token||document.hidden||heartbeatBusy)return;heartbeatBusy=true;
        try{await send('renew');renewalUncertain=false;}
        catch(error){
            renewalUncertain=true;
            if(error.status===409 || error.status===403){token='';renewalUncertain=false;message('控制权已失效，动作不自动恢复；反馈继续显示');}
            else message('续约回复未确认，暂停新动作；在原租约期限内核对，不重发动作');
        }
        finally{heartbeatBusy=false;draw();}
    },400);
    window.addEventListener('pagehide',()=>release('page-exit'));
    window.addEventListener('offline',()=>release('browser-offline'));
    document.addEventListener('visibilitychange',()=>{if(document.hidden)release('page-hidden');});
    draw();poll();return{release,stop};
}
