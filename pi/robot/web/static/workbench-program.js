// 工位流程只保存姿态引用和导航到位点；导出候选报文不发送到实车。
const $ = id => document.getElementById(id);
let api, current, draft=[], frozen=null, key='', versionKey='', arrivalsKey='';
const storageKey = 'gc_mechanism_program_draft_v1';
const stepsOf = record => record.steps.map(step=>({after_segment:step.after_segment,pose_id:step.pose.id,
    revision:step.pose.revision,release_confirmed:step.release_confirmed}));
function values() { return {id:$('program-id').value.trim(),name:$('program-name').value.trim(),steps:structuredClone(draft)}; }
function remember() { try { localStorage.setItem(storageKey,JSON.stringify(values())); } catch {} }
function rows() {
    const root = $('program-steps'); root.replaceChildren();
    draft.forEach((step,index)=>{
        const row = document.createElement('li');
        row.append(document.createTextNode(`段${step.after_segment}到位后 → ${step.pose_id} v${step.revision}${step.release_confirmed?' · 允许模型释放':''} `));
        const remove = document.createElement('button'); remove.textContent = '删除'; remove.className='text-button';
        remove.disabled=!!current?.busy;
        remove.onclick=()=>{draft.splice(index,1);rows();remember();}; row.append(remove);root.append(row);
    });
}
async function invoke(action,args={}) { return api.command('program',{action,...args}); }
function load(record) {
    frozen=record;draft=stepsOf(record);$('program-id').value=record.id;$('program-name').value=record.name;
    rows();remember();
}
function selected() {
    const [id,revision] = $('program-version').value.split(':');
    if (!id || !revision) throw new Error('先保存或读取流程版本');
    if (!frozen || frozen.id!==id || frozen.revision!==Number(revision) ||
        JSON.stringify(values())!==JSON.stringify({id:frozen.id,name:frozen.name,steps:stepsOf(frozen)}))
        throw new Error('请先读取该流程版本；草稿改动需重新保存');
    return {id,revision:Number(revision)};
}
export function initProgram(options) {
    api=options;
    $('mech-program-root').innerHTML=`<summary>工位流程与路线报文</summary><p class="hint">保存已确认的模型姿态引用，编译离线候选；不发送到实车。</p>
      <div class="plan-inputs"><label>流程编号<input id="program-id" value="cycle_1" maxlength="48"></label><label>名称<input id="program-name" value="工位流程" maxlength="64"></label><label>插入位置<select id="program-arrival"></select></label></div>
      <div class="button-row"><button id="program-add">把选中姿态加入工位流程</button><button id="program-save">保存流程版本</button></div>
      <p class="hint">插在导航到位后；按顺序加入取放、存取及收拢姿态。下一导航段前必须收拢；可在上方勾选本步允许模型释放。</p>
      <ol id="program-steps"></ol>
      <div class="plan-inputs"><label>已保存流程<select id="program-version"></select></label><button id="program-load">读取流程</button><button id="program-run">逐姿态联调（底盘不动）</button></div>
      <div class="button-row"><button id="program-compile" class="primary">编译路线报文</button><button id="program-stop" class="danger">停止流程模型</button></div>
      <p id="program-status" class="hint"></p><p id="program-export" class="hint"></p>`;
    try { const old=JSON.parse(localStorage.getItem(storageKey));if(old?.steps?.length<=32){draft=old.steps;$('program-id').value=old.id;$('program-name').value=old.name;} } catch {}
    rows();
    for (const id of ['program-id','program-name']) $(id).oninput=remember;
    $('program-add').onclick=()=>{
        try {
            const pose=api.selectedPose();const after_segment=Number($('program-arrival').value);
            if(!after_segment||!pose)throw new Error('先选工位到位点，并读取已保存姿态');
            if(draft.length>=32)throw new Error('最多32个姿态步骤');
            draft.push({after_segment,pose_id:pose.id,revision:pose.revision,release_confirmed:$('mech-release').checked});
            $('mech-release').checked=false;rows();remember();
        }catch(error){api.notify(error.message,true);}
    };
    $('program-save').onclick=async()=>{
        try {const args=values();const revision=Math.max(0,...(current?.versions||[]).filter(v=>v.id===args.id).map(v=>v.revision));
            const result=await invoke('save',{...args,revision,plan_id:api.planId()});load(result.program);api.notify('流程版本已保存，可读取或编译正式格式候选载荷；未发送设备。');
        }catch(error){api.notify(error.message,true);}
    };
    $('program-load').onclick=async()=>{
        try {const [id,revision]=$('program-version').value.split(':');load((await invoke('load',{id,revision:Number(revision)})).program);}
        catch(error){api.notify(error.message,true);}
    };
    for(const action of ['run','compile']) $(`program-${action}`).onclick=async()=>{
        try {const result=await invoke(action,selected());api.notify(action==='compile'?`已保存候选报文：${result.path}`:'开始逐姿态离线联调，底盘不动。');}
        catch(error){api.notify(error.message,true);}
    };
    $('program-stop').onclick=()=>invoke('stop').catch(error=>api.notify(error.message,true));
}

export function renderProgram(snapshot, enabled) {
    current=snapshot?.program;
    const busy=current?.busy;
    const blocked=busy||snapshot?.mechanism?.busy||['RUNNING','WAIT_OBSERVATION'].includes(snapshot?.transfer?.mode)||!snapshot?.simulation?.stopped;
    const route=api.planId();
    const arrivals=JSON.stringify(current?.arrivals||[]);
    if(arrivals!==arrivalsKey){arrivalsKey=arrivals;$('program-arrival').replaceChildren(...(current?.arrivals||[]).map(v=>new Option(v.label,String(v.after_segment))));}
    const versions=JSON.stringify(current?.versions||[]);
    if(versions!==versionKey){versionKey=versions;const old=$('program-version').value;
        $('program-version').replaceChildren(...(current?.versions||[]).map(v=>new Option(`${v.name} / ${v.id} v${v.revision}`,`${v.id}:${v.revision}`)));
        if(frozen)$('program-version').value=`${frozen.id}:${frozen.revision}`;else if([...$('program-version').options].some(v=>v.value===old))$('program-version').value=old;}
    for(const id of ['program-add','program-save','program-load','program-run','program-compile'])$(id).disabled=!enabled||blocked;
    $('program-save').disabled ||= !route;
    $('program-add').disabled ||= !route || !$('program-arrival').value;
    $('program-stop').disabled=!enabled||!busy;
    for(const id of ['program-id','program-name','program-arrival','program-version'])$(id).disabled=!!blocked;
    if(key!==`${busy}-${draft.length}`){key=`${busy}-${draft.length}`;rows();}
    $('program-status').textContent=current?`${current.mode} · ${current.index}/${current.total} · ${current.reason}${current.error?' · '+current.error:''}`:'离线核心尚未更新';
    $('program-export').textContent=current?.export?`候选包 ${current.export.sha256.slice(0,12)} · 导航${current.export.navigation_steps}段 / 机构${current.export.mechanism_steps}步 · ${current.export.path}；会话、零点和动作ID待运行时绑定，实车仍禁止执行。`:'保存不会运动；候选包沿用34/55正式载荷格式，暂不包含实车有效会话/零点或实测许可。';
}
