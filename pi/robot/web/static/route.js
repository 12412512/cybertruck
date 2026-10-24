// 只回放明确标记的离线记录，或绘制控制核心给出的实时轨迹；无设备命令。
let plan, replay, playing=false, live=false;
const set=(id,text)=>{document.getElementById(id).textContent=text;};
const labels={connect:'建立会话',config:'配置一次',observe:'等待反馈',arm:'使能一次',
    ready:'稳定交接',running:'运行中',finishing:'等待最终停车确认',complete:'完成并停车',failed:'中止',stale:'反馈过期'};
const names={straight:'直线',diagonal:'斜线',arc:'固定朝向弧线',pose:'同步转姿'};
const ns='http://www.w3.org/2000/svg';
function node(tag,attrs){const e=document.createElementNS(ns,tag);for(const [k,v] of Object.entries(attrs))e.setAttribute(k,v);return e;}
function draw(reference,trace,pose){
    const svg=document.getElementById('route-map');svg.replaceChildren();
    const points=[...reference,...trace,[0,0]];
    const minX=Math.min(...points.map(p=>p[0]))-.035, maxX=Math.max(...points.map(p=>p[0]))+.045;
    const minY=Math.min(...points.map(p=>p[1]))-.04, maxY=Math.max(...points.map(p=>p[1]))+.04;
    const scale=Math.min(570/(maxX-minX),230/(maxY-minY));
    const xy=p=>[35+(p[0]-minX)*scale,265-(p[1]-minY)*scale];
    const line=(data,color,width)=>svg.appendChild(node('polyline',{points:data.map(p=>xy(p).join(',')).join(' '),
        fill:'none',stroke:color,'stroke-width':width}));
    line(reference,'#3e729f',4);line(trace,'#ce6b28',2);
    if(pose){const [x,y]=xy(pose);svg.appendChild(node('circle',{cx:x,cy:y,r:6,fill:'#263e50'}));
        line([pose,[pose[0]+.018*Math.cos(pose[2]),pose[1]+.018*Math.sin(pose[2])]],'#263e50',3);}
}
function view(data,prefix){
    set('route-status',`${prefix} · ${labels[data.status]??data.status}`);
    set('route-progress',`${Math.min(data.index+1,data.total)}/${data.total} · 已完成${data.completed} · ${names[data.cases?.[data.index]]??'结束'}`);
    set('route-pose',data.pose?`${(data.pose[0]*1000).toFixed(1)} mm / ${(data.pose[1]*1000).toFixed(1)} mm / ${(data.pose[2]*180/Math.PI).toFixed(2)}°`:'等待新反馈');
    draw(data.reference_trace??plan?.reference_trace??[],data.trace??[],data.pose);
}
export function renderRoute(data){
    if(data && ['commissioning','simulation'].includes(data.domain)){
        playing=false;live=true;view(data,data.domain==='commissioning'?'实车局部里程计':'同会话模型闭环');
    }else if(live){live=false;set('route-status','控制核心状态过期 · 不自动恢复运动');set('route-pose','当前位姿未知，图中仅保留历史轨迹');}
}
async function setup(){
    try{
        plan=await(await fetch('/api/routes/short')).json();
        if(plan.frame_id!=='odom'||plan.executable!==false)throw Error('路线域不符');
        draw(plan.reference_trace,[],null);set('route-status',`冻结路线 ${plan.sha256.slice(0,10)} · 仅预览`);
        replay=await(await fetch('/api/routes/short/replay')).json();
        const button=document.getElementById('route-replay');
        button.disabled=!replay.available;
        if(!replay.available)button.textContent='尚无对应离线闭环记录';
        button.onclick=()=>{
            if(live||!replay.available||replay.domain!=='simulation_replay')return;
            playing=true;const start=performance.now(),frames=replay.frames,base=frames[0].execution.board_ms;
            function tick(now){
                if(!playing)return;
                const target=base+(now-start)*2;
                let index=frames.findIndex(f=>f.execution.board_ms>target);index=index<0?frames.length-1:Math.max(0,index-1);
                const frame=frames[index],end=index===frames.length-1;
                const trace=frames.slice(0,index+1).filter(f=>f.execution.odom_valid).map(f=>[f.execution.x_m,f.execution.y_m,f.execution.yaw_rad]);
                view({status:end?'complete':frame.stage,index:frame.route_segment??0,total:plan.cases.length,
                    completed:end?plan.cases.length:frame.route_segment??0,cases:plan.cases,reference_trace:plan.reference_trace,
                    trace,pose:trace.at(-1)},'离线记录回放 · 不连接实车');
                if(end)playing=false;else requestAnimationFrame(tick);
            }
            requestAnimationFrame(tick);
        };
    }catch{set('route-status','路线或离线记录不可用；不会连接设备');}
}
setup();
