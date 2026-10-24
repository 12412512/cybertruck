/* 转存编排仅调用离线核心；不会访问/api/device或导入实车适配器。 */
let api, latest;
const $=id=>document.getElementById(`transfer-${id}`);
const material=value=>({EMPTY:'空',UNKNOWN:'未知',UNSET:'待初始化',red:'红',green:'绿',blue:'蓝',yellow:'黄',cyan:'浅蓝',black:'黑'}[value]||value);
const labels={IDLE:'待命',RUNNING:'模型运行中',WAIT_OBSERVATION:'等待模型物料确认',COMPLETE:'模型闭环完成',FAILED:'失败，物料未知',CANCELLED:'已取消',INTERRUPTED:'上次中断，不自动恢复'};
const observations={PICKED:'确认模型已持料',STORED:'确认模型已存车',RETRIEVED:'确认模型已取出',PLACED:'确认模型已放置'};
function parameters(){
    const value={slot:Number($('slot').value),color:$('color').value,auto_model:$('auto').checked,pace:Number($('pace').value)};
    for(const name of ['raw-lift','raw-reach','slot-lift','place-lift','place-reach']) {
        if(!$(`${name}`).value.trim())throw Error('候选尺寸不能留空');
        value[name.replaceAll('-','_')+'_m']=Number($(name).value)/1000;
    }
    return value;
}
async function request(action,args={}) {
    try{return await api.command('transfer',{action,...args});}
    catch(error){api.notify(error.message,true);return null;}
}
function checked(){if(!$('confirm').checked)throw Error('请明确确认只运行离线模型和合成物料');}
export function initTransfer(root,options){
    api=options;
    const card=document.createElement('article');card.className='card transfer-workflow';
    card.innerHTML=`<div class="card-head"><h2>取料 → 存车 → 取出 → 放置</h2><span id="transfer-mode">离线单件闭环</span></div>
      <p>复用正式离线机构状态机；全部物料状态仅为合成夹具。不连接实车，不增加真实抓放成功次数。</p>
      <div class="plan-inputs"><label>车载料位<select id="transfer-slot"><option value="0">料位1 · 30°</option><option value="1">料位2 · 150°</option><option value="2">料位3 · 270°</option></select></label>
      <label>合成物料<select id="transfer-color">${[['red','红'],['green','绿'],['blue','蓝'],['yellow','黄'],['cyan','浅蓝'],['black','黑']].map(([key,name])=>`<option value="${key}">${name}</option>`).join('')}</select></label>
      <label>演示倍率（仅离线）<select id="transfer-pace"><option value="1">1倍</option><option value="10">10倍</option><option value="20" selected>20倍</option></select></label></div>
      <details><summary>姿态候选尺寸 / mm（放置高度待实测）</summary><div class="mech-targets">${[['raw-lift','原料下降',49],['raw-reach','原料伸出',50],['slot-lift','车载下降',43],['place-lift','放置下降（占位候选）',49],['place-reach','放置伸出（占位候选）',50]].map(([id,name,value])=>`<label>${name}<input id="transfer-${id}" type="number" min="0" max="${id.includes('reach')?50:49}" value="${value}"></label>`).join('')}</div></details>
      <label class="check-label"><input type="checkbox" id="transfer-confirm">我确认只操作离线模型；放置尺寸未标定，不能用于实车</label>
      <label class="check-label"><input type="checkbox" id="transfer-auto">由合成夹具自动给出四处模型观测（不作为实物证据）</label>
      <div class="button-row"><button id="transfer-claim">获取离线控制权</button><button id="transfer-prepare">准备模拟基准与舵机估计</button><button id="transfer-preview">预览完整动作</button><button id="transfer-run">运行完整离线流程</button><button id="transfer-stop">停止离线流程 · 保夹</button></div>
      <p id="transfer-reason"></p><p id="transfer-time"></p><p id="transfer-material"></p>
      <div class="button-row"><button id="transfer-observe">确认当前模型观测</button><button id="transfer-unknown">模型观测未知，停止流程</button><button id="transfer-reset">重置离线物料夹具（不动作）</button></div>
      <ol id="transfer-steps"></ol><div id="transfer-events"></div>`;
    root.prepend(card);
    $('claim').onclick=()=>document.getElementById('claim').click();
    $('prepare').onclick=async()=>{
        try{
            checked();const m=latest.mechanism;let version=m.zero_version;
            if(!m.zero_valid){const r=await api.command('mechanism',{action:'zero',confirmed:true});version=r.zero_version;}
            if(m.servos.every(s=>s.estimate_rad===null))await api.command('mechanism',{action:'anchor',zero_version:version,confirmed:true});
            api.notify('模拟基准已处理，没有实车输出；已有估计不重新锚定');
        }catch(error){api.notify(error.message,true);}
    };
    $('preview').onclick=async()=>{try{const plan=await request('preview',parameters());if(plan)drawPlan(plan);}catch(e){api.notify(e.message,true);}};
    $('run').onclick=async()=>{try{checked();await request('run',{...parameters(),confirmed:true});}catch(e){api.notify(e.message,true);}};
    $('stop').onclick=()=>request('stop');
    $('observe').onclick=()=>{const f=latest.transfer;request('confirm',{action_id:f.id,step_index:f.index,verdict:f.current.expected});};
    $('unknown').onclick=()=>{const f=latest.transfer;request('confirm',{action_id:f.id,step_index:f.index,verdict:'UNKNOWN'});};
    $('reset').onclick=()=>{try{checked();request('reset',{confirmed:true});}catch(e){api.notify(e.message,true);}};
}
let planKey='';
function drawPlan(plan,index=-1){
    const key=`${plan.sha256}:${index}`;if(planKey===key)return;planKey=key;
    $('time').textContent=`冻结流程：料位${plan.configuration.slot+1} / ${plan.configuration.color} / ${plan.configuration.auto_model?'自动合成观测':'逐次模型确认'}。候选模型原速约${plan.model_seconds}s；${plan.configuration.pace}倍只加速离线演示，不修改实车速度或反馈保护。此模型时间不是实测或180秒验收。`;
    $('steps').replaceChildren(...plan.steps.map((step,i)=>{const li=document.createElement('li');li.textContent=`${i===index?'▶ ':''}${step.label}${step.kind==='move'?` · ${step.axis} ${Math.round(step.target*(step.axis==='lift'||step.axis==='reach'?1000:180/Math.PI)*10)/10}${step.axis==='lift'||step.axis==='reach'?'mm':'°'}`:' · 需要独立模型观测'}`;if(i===index)li.className='current';return li;}));
}
export function renderTransfer(snapshot,owned){
    latest=snapshot;const flow=snapshot?.transfer,m=snapshot?.mechanism;
    if(!$('mode'))return;
    const busy=flow&&['RUNNING','WAIT_OBSERVATION'].includes(flow.mode);
    const ready=!!(snapshot?.available&&flow&&owned&&m&&!m.fault&&snapshot.simulation.stopped);
    $('mode').textContent=flow?labels[flow.mode]||flow.mode:'等待离线核心更新';
    $('reason').textContent=flow?`${flow.reason} · ${flow.index}/${flow.total}${flow.current?` · ${flow.current.label}`:''}`:'当前核心尚未加载转存流程；不会转发到实车';
    for(const id of ['prepare','preview','run','reset'])$(id).disabled=!ready||!!busy||!!m?.busy;
    $('run').disabled=$('run').disabled||!m?.zero_valid;
    $('stop').disabled=!owned||!busy;
    $('observe').disabled=$('unknown').disabled=!ready||flow?.mode!=='WAIT_OBSERVATION';
    $('observe').textContent=observations[flow?.current?.expected]||'确认当前模型观测';
    for(const e of document.querySelectorAll('.transfer-workflow input,.transfer-workflow select'))e.disabled=!!busy;
    if(flow?.plan)drawPlan(flow.plan,flow.index);
    if(flow){$('material').textContent=`仅模型：原料 ${material(flow.ledger.source)} / 夹爪 ${material(flow.ledger.gripper)} / 三料位 ${flow.ledger.slots.map(material).join('、')} / 目标 ${material(flow.ledger.destination)}`;
        $('events').replaceChildren(...flow.events.map(event=>{const p=document.createElement('p');p.textContent=`${observations[event.event]||event.event} · ${event.evidence==='synthetic_fixture'?'合成夹具观测':'人工模型确认'} · 非实物证据`;return p;}));}
}
