// 仅渲染核心给出的同一计划与模拟结果；画布Y反转不进入物理坐标。
const names = {start_1:'启停区1', start_2:'启停区2', qr:'扫码', raw:'原料', rough:'粗加工', staging:'暂存'};
const actions = {LINE:'直线', DIAGONAL:'短斜移/侧移', TURN:'停车转向', ARC:'平滑弯道'};
const states = {READY:'待模拟', RUNNING:'闭环运行', COMPLETE:'巡回完成', FAILED:'失败', NO_PATH:'无可行路线'};
const svgNS = 'http://www.w3.org/2000/svg';
const point = p => [40+p[0]*230, 600-p[1]*230];
const set = (id,value) => {document.getElementById(id).textContent=value;};
function element(tag,attributes,text) {
    const node=document.createElementNS(svgNS,tag);
    for (const [key,value] of Object.entries(attributes)) node.setAttribute(key,value);
    if (text!==undefined) node.textContent=text;
    return node;
}
function polyline(points,color,width) {
    return element('polyline',{points:points.map(p=>point(p).join(',')).join(' '),fill:'none',stroke:color,'stroke-width':width});
}
function draw(nav) {
    const svg=document.getElementById('nav-map');
    svg.replaceChildren();
    svg.appendChild(element('rect',{x:40,y:48,width:552,height:552,fill:'#eef1f3',stroke:'#94a4b3'}));
    for (const zone of nav.field.zones) {
        const [x0,y0,x1,y1]=zone.bounds_m;
        const [x,y]=point([x0,y1]);
        const blue=zone.id.startsWith('start_');
        svg.appendChild(element('rect',{x,y,width:(x1-x0)*230,height:(y1-y0)*230,
            fill:blue?'#c9def4':zone.id.startsWith('yellow')?'#faeab2':'#d3dce0',stroke:'#b6c3cc'}));
        if (blue) svg.appendChild(element('text',{x:x+6,y:y+18,fill:'#244e70','font-size':12},names[zone.id]));
    }
    svg.appendChild(polyline(nav.reference_trace,'#3e729f',4));
    svg.appendChild(polyline(nav.trace,'#ce6b28',2));
    const pose=nav.map_pose, body=nav.field.body, co=Math.cos(pose[2]), si=Math.sin(pose[2]);
    const corners=[[-body.rear_m,-body.right_m],[body.front_m,-body.right_m],
        [body.front_m,body.left_m],[-body.rear_m,body.left_m]].map(([x,y])=>[pose[0]+co*x-si*y,pose[1]+si*x+co*y]);
    svg.appendChild(element('polygon',{points:corners.map(p=>point(p).join(',')).join(' '),fill:'#ffffffc9',stroke:'#263e50','stroke-width':2}));
    svg.appendChild(polyline([pose,[pose[0]+co*.18,pose[1]+si*.18]],'#263e50',3));
    for (const [text,p] of [['扫码',[1.2,2.39]],['原料',[.04,1.25]],['粗加工',[2.15,1.25]],['暂存',[1.1,.08]]]) {
        const [x,y]=point(p); svg.appendChild(element('text',{x,y,fill:'#263e50','font-size':13},text));
    }
    svg.appendChild(polyline([[0,0],[.3,0]],'#b8463d',3));
    svg.appendChild(polyline([[0,0],[0,.3]],'#30805a',3));
    svg.appendChild(element('text',{x:115,y:623,fill:'#b8463d','font-size':14},'X →'));
    svg.appendChild(element('text',{x:9,y:523,fill:'#30805a','font-size':14},'Y ↑'));
    svg.appendChild(element('text',{x:23,y:623,fill:'#526579','font-size':12},'0'));
    svg.appendChild(element('text',{x:550,y:623,fill:'#526579','font-size':12},'2.4 m'));
}

export function renderNavigation(nav) {
    const valid=nav?.domain==='simulation' && nav.frame_id==='map';
    document.getElementById('nav-panel').hidden=!valid;
    if (!valid) {document.getElementById('nav-map').replaceChildren(); return;}
    set('nav-frame',`${names[nav.start]??'启停区1'}名义中心，朝扫码区 +90°；map：X右/Y上，车体：X前/Y左`);
    set('nav-state',`${states[nav.status]??nav.status} / 已确认 ${nav.completed}/${nav.actions} / 第${nav.leg+1}段`);
    const heading=Math.atan2(Math.sin(nav.map_pose[2]),Math.cos(nav.map_pose[2]))*180/Math.PI;
    set('nav-pose',`${nav.map_pose[0].toFixed(3)} m / ${nav.map_pose[1].toFixed(3)} m / ${heading.toFixed(1)}°（模拟；展开角${(nav.map_pose[2]*180/Math.PI).toFixed(1)}°）`);
    set('nav-time',`${nav.estimate_s.toFixed(1)} s（优化前${nav.before_s.toFixed(1)} s） / ${(nav.elapsed_ms/1000).toFixed(2)} s（模拟时间；演示目标5倍速）`);
    set('nav-origin',`${nav.origin_ms??'待建立'} ms / ${nav.origin_consistent?'全程未清零':'发生改变'}`);
    set('nav-conditions',`实车执行关闭；道路开放为合成场景；待验：${nav.assumptions.join('、')}`);
    set('nav-reason',nav.reason||'—');
    const body=document.getElementById('nav-legs');body.replaceChildren();
    nav.legs.forEach((leg,index)=>{
        const row=document.createElement('tr');
        for (const text of [`${names[leg.start]??leg.start} → ${names[leg.goal]??leg.goal}`,
            `${leg.status} / ${leg.nodes.join(' → ')}`,leg.segments.map(s=>actions[s.kind]??s.kind).join(' / '),
            `${(leg.estimate_s??0).toFixed(1)} s`]) {
            const cell=document.createElement('td');cell.textContent=text;row.appendChild(cell);
        }
        if(index===nav.leg) row.className='active-leg';body.appendChild(row);
    });
    draw(nav);
}
