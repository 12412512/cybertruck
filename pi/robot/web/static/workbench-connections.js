/* 只准备连接。设备动作仍走唯一核心与原控制权、零点、故障检查。 */
const panel=document.getElementById('workspace-connections');
panel.innerHTML=`<summary><span>设备连接</span><span id="connections-brief" class="badge muted">读取连接管理器</span></summary>
 <div class="connection-body"><div class="connection-status-grid">
 <div><small>树莓派</small><strong id="connections-ssh">未连接</strong></div>
 <div><small>底盘 / STM32</small><strong id="connections-device">未连接</strong></div>
 <div><small>机械反馈</small><strong id="connections-mech">未接入</strong></div>
 <div><small>物料 / 扫码相机</small><strong id="connections-cameras">未连接</strong></div></div>
 <div class="connection-toolbar"><span id="connections-address">172.20.10.4 · pi</span>
 <label>SSH密码<input id="connections-password" type="password" autocomplete="off" placeholder="首次连接填写；仅保留本机内存"></label>
 <button id="connections-all" class="primary">连接全部</button><button id="connections-device-button">连接底盘 / 机械</button>
 <button id="connections-vision">连接视觉</button></div>
 <label class="connection-confirm"><input id="connections-confirm" type="checkbox">车辆和机构已静止、机构有支撑；允许重开失效串口。串口重开可能复位STM32，旧零点和参考需重新建立。</label>
 <p id="connections-message" role="status">点击后才连接；不会取得控制权、使能、建零或执行目标。</p>
 <div id="connections-errors"></div><p class="hint">底盘与机械共用一个STM32核心。在线服务直接复用，即使有故障也不以重连清除；连接成功不代表已开放运动。断线后由你再次点击，不自动重连或续跑。</p></div>`;
let state={},pending=false,polling=false;
const el=id=>document.getElementById(`connections-${id}`);
const buttons=['all','device-button','vision'];
function draw(){
    const d=state.device||{},c=state.cameras||{};
    el('address').textContent=`${state.host||'172.20.10.4'} · ${state.username||'pi'}`;
    el('ssh').textContent=state.ssh_connected?'已连接':'未连接';
    el('device').textContent=d.connected?(d.base_fault||d.mechanism_fault?`在线 · 故障 ${d.base_fault}/${d.mechanism_fault}`:'在线 · 仅连接'):'未连接';
    el('mech').textContent=d.connected?(d.attached?'已接入':'未附着；在机械页接入'):'未连接';
    el('cameras').textContent=`${c.material?.available?'在线':'未就绪'} / ${c.qr?.available?'在线':'未就绪'}`;
    const busy=pending||state.busy;
    for(const id of buttons)el(id).disabled=busy||!state.available;
    el('brief').textContent=busy?'正在连接':d.connected&&state.vision_connected?'实车与视觉在线':d.connected?'实车在线':state.ssh_connected?'SSH在线 · 设备待核对':'未连接';
    el('brief').className=`badge ${d.connected?'good':'muted'}`;
    el('password').placeholder=state.credential_cached?'已保留本次密码，无需重复填写':'首次连接填写；仅保留本机内存';
    el('errors').replaceChildren(...Object.entries(state.results||{}).filter(([,v])=>!v.ok).map(([name,v])=>{
        const p=document.createElement('p');p.className='connection-error';p.textContent=`${name==='device'?'底盘 / 机械':'视觉'}：${v.reason}`;return p;
    }));
}
async function poll(){
    if(polling||document.hidden)return;polling=true;
    try{
        const response=await fetch('/api/connections/state',{cache:'no-store'});
        if(!response.ok)throw Error('连接管理器状态读取失败');
        state=await response.json();el('message').textContent=state.message;
    }catch(error){state={available:false};el('message').textContent=error.message;}
    finally{polling=false;draw();}
}
async function connect(component){
    if(pending||state.busy)return;pending=true;draw();
    const password=el('password').value;el('password').value='';
    try{
        const response=await fetch('/api/connections/connect',{method:'POST',headers:{'Content-Type':'application/json','X-Workbench':'1'},
            body:JSON.stringify({component,password,confirmed_reopen:el('confirm').checked})});
        const value=await response.json();if(!response.ok)throw Error(value.detail||'连接准备被拒绝');
        el('message').textContent='正在准备连接；请等待结果，不要重复点击。';el('confirm').checked=false;
    }catch(error){el('message').textContent=error.message;}
    finally{pending=false;await poll();}
}
el('all').onclick=()=>connect('all');el('device-button').onclick=()=>connect('device');el('vision').onclick=()=>connect('vision');
document.getElementById('connections-open').onclick=()=>{panel.open=!panel.open;if(panel.open)poll();};
setInterval(poll,1500);poll();
