/* 机械实测准备的纯显示规则；不拥有设备、不发送命令。 */
export const names = {lift:'升降',reach:'伸缩',arm:'转臂',tray:'料盘',gripper:'夹爪'};
const states = {
    axis:['空闲','已接受','发送中','运行中','稳定确认','停止中','完成','失败','取消'],
    servo:['空闲','已接受','缓动','等待输出','命令稳定等待','等待人工姿态确认','已确认','失败','取消'],
    pose:['空闲','已接受','执行中','等待舵机确认','等待单步放行','停止中','完成','失败','取消']
};
export function stateName(kind,value) { return states[kind]?.[value] ?? '未知'; }
export function rejectionText(action, result, state={}) {
    const reasons=['成功','未建立零点','反馈过期','机构未稳定或上一操作未结束',
        '驱动器状态不符','配置或使能条件不符','缺少人工确认','会话不符',
        '目标越限','底盘未停车','碰撞互锁','零点版本不符','报文不符','零点已建立','故障锁存'];
    const detail=action==='enable' && result===5
        ? `；同一轴已使能也会拒绝重复请求。当前配置${state.mechanism?.config_mask ?? '未知'}/3，底盘停车${state.base?.stopped?'已确认':'未确认'}。请勿连续重试使能`
        : '';
    return `板端拒绝：${reasons[result] || '未知原因'}（${result}）${detail}`;
}
export function blockers(state,owned,pending) {
    const m=state.mechanism, list=[];
    if (!state.connected || !m) list.push('实时反馈未就绪，旧状态不能授权动作');
    if (state.base?.fault || m?.fault) list.push(`故障锁存：底盘 ${state.base?.fault || 0} / 机构 ${m?.fault || 0}`);
    if (!state.site_ready) list.push('本次现场动作许可未开放');
    if (!owned) list.push('尚未持有实车控制权');
    if (pending) list.push('上一请求尚未完成应答，禁止重复发送');
    return list;
}
export function canAct(state,owned,pending,action) {
    const m=state.mechanism;
    if (blockers(state,owned,pending).length) return false;
    if (!state.base?.stopped || !m.base_stopped || !m.attached) return false;
    if (m.config_mask!==3) return false;
    if (!action.startsWith('enable') && action!=='zero' && !m.zero_valid) return false;
    if (action==='pose' && m.pwm_mask!==7) return false;
    if (action==='next') return m.pose_state===4;
    if (action==='confirm') return m.servo_state===5;
    return !((m.axis_state>=1&&m.axis_state<=5)||(m.servo_state>=1&&m.servo_state<=5)||(m.pose_state>=1&&m.pose_state<=5));
}
export function targetError(axis,value) {
    const bounds={lift:[0,49],reach:[0,50],arm:[83,180],tray:[0,270],gripper:[0,98]}[axis];
    return !bounds || !Number.isFinite(value) || value<bounds[0] || value>bounds[1] ? '目标超出已验证初始范围' : '';
}
export function samePose(targets,record) {
    return record && Object.keys(names).every(key=>Number.isFinite(targets[key]) && Math.abs(targets[key]-record.targets[key])<1e-7);
}
export function procedure(axis) {
    const common=['换PCB后先完成只读验收，CAN故障保持为0；本次另行确认现场后才开放动作。',
        '人工将升降置最高、伸缩置最近端；核对底盘停车、ID5/6配置和新反馈后，分别使能并建零。',
        '逐项输入并核对三个舵机当前参考；记录参考不输出PWM，不能用离线模型角度代替。'];
    const steps={
        lift:['先以5RPM执行下降2mm，使用同一固定点独立尺量。','确认稳定停车和方向后，单独执行回到0mm；不自动连续往返。','合格后再分级扩展，先不超过原49mm范围。'],
        reach:['先完成升降及转臂参考验收；转臂180°、伸缩收回。','升降先分级验证行程，再单独降到45mm并确认；未完成前不执行伸出。','以5RPM伸出2mm，尺量后单独收回0mm；收回确认后才允许升降回0mm。'],
        arm:['升降/伸缩均≤1mm，人工核对转臂真实参考。','首次PWM目标与人工参考相同；先验收是否跳动，再人工确认姿态。','从已确认180°试175°，再单独回180°；83°留待完整包络核对。'],
        tray:['升降/伸缩均≤1mm、转臂已确认180°，夹爪退出料盘碰撞区。','首次PWM与人工参考相同；0°基准只在实际对齐后使用。','先从已确认0°试5°并回0°，再分别验收30°、150°、270°三个料位。'],
        gripper:['空爪、无人接触夹爪；核对当前实际角度后首次输出同参考目标。','仅在空爪且明确允许张爪时，由已确认98°试93°，再单独回98°。','60°释放和真实夹持物料另作验收；停止时不自动张爪。']
    };
    return [...common,...(steps[axis]||[])];
}
export function initGuide(card,readState,fillTarget,readTargets,selectedPose,notify,readReference) {
    const section=document.createElement('section');section.className='mech-test-guide';
    section.innerHTML=`<h3>实测准备与人工记录</h3><p>本区只准备目标和记录观察，不发送设备命令，不影响成功抓放计数。</p>
      <label>准备项目<select id="guide-axis">${Object.entries(names).map(([k,n])=>`<option value="${k}">${n}</option>`).join('')}</select></label>
      <ol id="guide-steps"></ol><div class="button-row"><button id="guide-reference">填入首次PWM参考草稿</button><button id="guide-first">填入首次小步草稿</button><button id="guide-home">填入回程草稿</button></div>
      <p id="guide-target-note"></p><h4>本次动作限制</h4><ul id="guide-blockers"></ul>
      <div class="mech-targets"><label>独立尺量/量角读数<input id="guide-measured" maxlength="120" placeholder="例如：下降约2mm；未量则写未量"></label>
      <label>人工观察结论<select id="guide-result"><option value="unmeasured">未测量/待确认</option><option value="observed">目视正常，精度未验收</option><option value="measured">已独立测量（数值见记录）</option><option value="abnormal">异常，停止并排查</option></select></label>
      <label>现象/备注<input id="guide-note" maxlength="300" placeholder="方向、抖动、干涉、保持及停车情况"></label></div>
      <button id="guide-record">保存人工观察记录（不证明动作成功）</button><button id="guide-export">导出本机记录JSON</button><p id="guide-storage"></p><div id="guide-records"></div>`;
    card.append(section);
    const el=id=>section.querySelector(`#guide-${id}`), key='gc_mech_observations_v1';
    let records=[];
    try { const value=JSON.parse(localStorage.getItem(key)||'[]'); if(Array.isArray(value))records=value.slice(-100); } catch { notify('本机旧记录读取失败，保留原数据；不影响故障停车'); }
    function drawRecords() {
        el('records').replaceChildren(...records.slice(-5).reverse().map(record=>{const p=document.createElement('p');p.textContent=`${new Date(record.at).toLocaleString('zh-CN',{timeZone:'Asia/Shanghai',hour12:false})}（北京时间） · ${names[record.axis]||record.axis} · ${{unmeasured:'未测量/待确认',observed:'目视正常，精度未验收',measured:'有独立测量',abnormal:'异常'}[record.result]||record.result} · ${record.measured||'未量'} · ${record.note||''}`;return p;}));
        el('storage').textContent=`本浏览器保存${records.length}条，最多100条；人工记录不等于板端或实物验收。`;
    }
    function drawSteps(){el('reference').disabled=['lift','reach'].includes(el('axis').value);el('steps').replaceChildren(...procedure(el('axis').value).map(text=>{const li=document.createElement('li');li.textContent=text;return li;}));}
    el('axis').onchange=drawSteps;drawSteps();drawRecords();
    el('reference').onclick=()=>{const axis=el('axis').value,value=readReference(axis),error=targetError(axis,value);if(error){notify('请先填写并核对当前真实参考角度');return;}fillTarget(axis,value);el('target-note').textContent='首次PWM仅填入人工参考的同角度目标；尚未输出，请先核对实物参考。';};
    el('first').onclick=()=>{const axis=el('axis').value;const value={lift:2,reach:2,arm:175,tray:5,gripper:93}[axis];fillTarget(axis,value);el('target-note').textContent='仅填入目标；先完成参考及首次同角度PWM验收，之后再点击正式执行。';};
    el('home').onclick=()=>{const axis=el('axis').value;fillTarget(axis,{lift:0,reach:0,arm:180,tray:0,gripper:98}[axis]);el('target-note').textContent='仅填入回程草稿，仍需满足互锁后独立执行；不会自动回零或张爪。';};
    el('record').onclick=()=>{
        const state=readState();
        const record={at:new Date().toISOString(),axis:el('axis').value,result:el('result').value,
            measured:el('measured').value.trim(),note:el('note').value.trim(),domain:'manual_observation',physical_validated:false,
            boot_id:state.boot_id||null,session_id:state.session_id||null,firmware:state.firmware||null,
            feedback:state.connected?state.mechanism:null,base:state.connected?state.base:null,pose_version:selectedPose()?.revision||null};
        if(record.result==='measured'&&!record.measured){notify('请填写独立测量读数，不以模型或命令值代替');return;}
        const next=[...records,record].slice(-100);
        try {localStorage.setItem(key,JSON.stringify(next));records=next;drawRecords();notify('已保存人工观察；没有发送实车命令');}catch{notify('本机记录保存失败，请先导出备份');}
    };
    el('export').onclick=()=>{const url=URL.createObjectURL(new Blob([JSON.stringify({schema:1,domain:'manual_observation',records},null,2)],{type:'application/json'}));const link=document.createElement('a');link.href=url;link.download='mechanism-observations.json';link.click();setTimeout(()=>URL.revokeObjectURL(url),1000);};
    return (state,owned,pending)=>{
        const reasons=blockers(state,owned,pending);
        if(!state.mechanism?.attached)reasons.push('机构反馈未接入，轴位置未知');
        if(!state.mechanism?.zero_valid)reasons.push('实车零点无效；本区不会自动建零');
        const record=selectedPose();
        if(record&&!samePose(readTargets(),record))reasons.push('姿态草稿与选中版本不同：先读取或另存新版本');
        el('blockers').replaceChildren(...(reasons.length?reasons:['基础条件满足，仍须逐项核对本动作互锁与现场条件']).map(text=>{const li=document.createElement('li');li.textContent=text;return li;}));
    };
}
