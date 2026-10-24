// 仅取本地验收归档，不连接串口，不续约、不获取控制权。
const $ = id => document.getElementById(id);
let last = '';
const time = value => new Date(value * 1000).toLocaleString('zh-CN',{timeZone:'Asia/Shanghai',hour12:false});

export function initAcceptance() {
    $('readonly-acceptance').innerHTML = `<div class="card-head"><h2>实车只读验收记录</h2><span id="acceptance-status" class="badge muted">等待读取记录</span></div>
        <p id="acceptance-time"></p><p id="acceptance-notice" class="hint">采集已结束；此处不显示实时连接，不授予实车动作权限。</p>
        <dl id="acceptance-values"></dl><details id="can-offline-analysis" hidden><summary>CAN离线分析 · 未接触设备</summary><ul id="can-analysis-values"></ul></details>
        <details id="can-first-capture" hidden><summary>首次CAN错误与故障前历史 · 已结束采集</summary><p id="can-first-values"></p><ul id="can-first-history"></ul></details>
        <details><summary>反馈与限制</summary><div id="acceptance-feedback"></div><ul id="acceptance-limits"></ul></details>`;
    async function poll() {
        try {
            const response=await fetch('/api/workbench/acceptance',{cache:'no-store',signal:AbortSignal.timeout(1500)});
            if (!response.ok) throw new Error('验收摘要暂不可读');
            const value=await response.json(), key=JSON.stringify(value);
            if (key===last) return;
            last=key;
            $('acceptance-status').textContent=value.available?(value.passed?'窗口验收通过 · 已结束':'验收未通过 · 保留故障'):'暂无记录';
            if (!value.available) { $('acceptance-time').textContent=value.reason || '未读取设备'; return; }
            $('acceptance-time').textContent=`采集完成：${time(value.observed_at_s)}（北京时间）；${value.transport==='swd'?'不停核SWD读取，未打开UART；':''}当前只显示历史快照`;
            const board=value.last_execution, ram=value.ram;
            const rows=[['固件 / 启动身份',`${value.firmware} / ${value.boot_id}`],
                ['窗口末状态',`${board.mode_name} / 故障${board.fault}（0x${board.fault.toString(16)}）`],
                ['窗口内停车确认',board.stop_confirmed?'板端已确认；不代表此刻实时状态':'未确认'],
                ['命令与动作',value.transport==='swd'?'SWD只读；未打开UART，未发送配置、使能或运动目标':`只读白名单${value.readonly_commands?'符合':'不符合'}；配置版本${board.config_version}，动作ID${board.action_id}`],
                ['CAN接收 / 窗口错误',value.transport==='swd'?`序号增加${value.can_rx_delta} / 累计错误${value.swd_validation.error_start}→${value.swd_validation.error_end}`:`序号增加${value.can_rx_delta} / ${board.can_errors}次错误`],
                ['故障入口',value.fault_explanation]];
            if (value.swd_validation) rows.push(['烧录与读回',`${value.swd_validation.verified_bytes}字节一致；CPU读取前后运行，未停核/复位`],
                ['匹配回包调度',`回包增加${value.swd_validation.reply_delta}；查询超时${value.swd_validation.query_timeouts}；窗口CAN错误${value.swd_validation.error_start}→${value.swd_validation.error_end}`]);
            if (ram) rows.push(['RAM读取',`${time(ram.observed_at_s)}；CPU前后运行，未停核/复位；固件${ram.verified_bytes}字节${ram.approved_0311_equal?'一致':'不一致'}`],
                ['后续RAM累计',`发送错误${ram.errors}，纯仲裁丢失${ram.arbitration_lost}；最近LEC=${ram.lec}，TEC=${ram.tec}`],
                ['最近错误查询',`ID${ram.error_tx_id>>>8}，CAN命令0x${(ram.error_tx_data0&255).toString(16)}，板端${ram.error_tick}ms；不是首次错误现场`]);
            $('acceptance-values').replaceChildren(...rows.map(([name,text])=>{const row=document.createElement('div'),dt=document.createElement('dt'),dd=document.createElement('dd');dt.textContent=name;dd.textContent=text;row.append(dt,dd);return row;}));
            $('acceptance-feedback').replaceChildren(...value.wheels.map(w=>{const p=document.createElement('p');p.textContent=`轮ID${w.address}：${w.rpm_range?.join('～') ?? '未知'} RPM；速度/位置/状态${value.transport==='swd'?'RAM快照':'样本'}${w.speed_samples}/${w.position_samples}/${w.status_samples}`;return p;}),
                ...value.feedback.map(f=>{const p=document.createElement('p');p.textContent=`${f.name}：${f.samples}帧；采集时板端最大帧龄${f.device_age_max_ms ?? '未知'}ms`;return p;}));
            $('acceptance-limits').replaceChildren(...value.restrictions.map(text=>{const item=document.createElement('li');item.textContent=text;return item;}));
            $('can-offline-analysis').hidden=!value.analysis;
            $('can-offline-analysis').querySelector('summary').textContent=value.transport==='swd'?'CAN只读分析 · SWD现场':'CAN离线分析 · 未接触设备';
            if (value.analysis) {
                const analysis=value.analysis;
                const lines=[...analysis.confirmed,...analysis.uncertain.map(text=>`待验证：${text}`),...analysis.next_steps.map(text=>`下一步：${text}`),...(analysis.validation || []).map(text=>`离线候选：${text}`)];
                $('can-analysis-values').replaceChildren(...lines.map(text=>{const item=document.createElement('li');item.textContent=text;return item;}));
            }
            const capture=value.can_fault_record;
            $('can-first-capture').hidden=!capture?.complete;
            if (capture?.complete) {
                const first=capture.first;
                $('can-first-values').textContent=`来源${capture.source || 'UART冻结分页'}；身份${capture.boot_id}；首次${first.tick_ms}ms，HAL=0x${first.hal_error.toString(16)}，TSR=0x${first.tsr.toString(16)}，ESR=0x${first.esr.toString(16)}；${capture.history.length}条本机事件，非总线SOF抓包。`;
                $('can-first-history').replaceChildren(...capture.history.map(event=>{const item=document.createElement('li');item.textContent=`#${event.sequence} / ${event.tick_ms}ms / ${event.kind_name} / ID 0x${event.id.toString(16)} / ${event.length}字节 / TX事务${event.tx_sequence}`;return item;}));
            }
        } catch {
            last=''; $('acceptance-status').textContent='记录读取失败 · 非实时';
        }
    }
    poll(); setInterval(poll,5000);
}
