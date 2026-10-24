// 只显示现场回填和未齐备项，不接收网页确认来启用设备。
const mm=value=>Number.isFinite(value)?`${(value*1000).toFixed(1)}mm`:'待测';
const set=(id,text)=>{document.getElementById(id).textContent=text;};
async function read(){
    try{
        const data=await(await fetch('/api/site/start1-qr')).json(), m=data.measurements;
        const labels={WAITING_MEASUREMENTS:'等待测量',INVALID_DATA:'测量数据需核对',PLACEMENT_UNSAFE:'起点包络/误差余量不足',DATA_COMPLETE:'数据齐备，仍待路线几何/闭环验证'};
        set('site-status',`${labels[data.status]??'未知'} · 未开放实车路线`);
        set('site-body',['front_m','rear_m','left_m','right_m'].map(k=>mm(m.body[k])).join(' / '));
        set('site-field',`${mm(m.field.width_m)} × ${mm(m.field.height_m)} / ${mm(m.start_zone.width_m)} × ${mm(m.start_zone.height_m)}`);
        set('site-start',`${mm(m.start.from_left_m)} / ${mm(m.start.from_top_m)}；地图起点(${mm(m.start.from_left_m)}, ${mm(m.field.height_m-m.start.from_top_m)})，朝向+X`);
        set('site-target',m.target.node?`${m.target.node}：${mm(m.target.map_x_m)} / ${mm(m.target.map_y_m)}（规划目标，非定位地标）`:'待确认north或qr');
        set('site-corridor',`${m.corridor.state==='OPEN'?'人工确认开放':'通行未确认'}；最窄${mm(m.corridor.minimum_width_m)}；位置误差${mm(m.start.position_error_m)}，航向误差${Number.isFinite(m.start.heading_error_rad)?(m.start.heading_error_rad*180/Math.PI).toFixed(2)+'°':'待测'}`);
    }catch{set('site-status','测量记录不可用；不建立配准');}
}
read();setInterval(read,5000);

// 候选路线使用独立画布，不覆盖当前模拟或真实位置。
async function preview(){
    const panel=document.getElementById('site-panel');
    const out=document.createElement('p'); panel.append(out);
    try{
        const data=await(await fetch('/api/site/start1-qr/preview')).json();
        if(!data.available){out.textContent='本段离线报告待生成或配置已改变';return;}
        out.textContent=`待摆放标记方案：斜直线${(data.distance_m*1000).toFixed(0)}mm，车头保持+X；低速模型${data.elapsed_s.toFixed(2)}秒，${data.state}。非实车轨迹。`;
        const svg=document.createElementNS('http://www.w3.org/2000/svg','svg');
        svg.setAttribute('viewBox','0 0 1200 340');svg.style.width='100%';
        svg.setAttribute('aria-label','启停区1到north条件预览，蓝色参考、橙色闭环模型');
        for(const [points,color] of [[data.reference,'#168aad'],[data.trace,'#e98125']]){
            const line=document.createElementNS(svg.namespaceURI,'polyline');
            line.setAttribute('points',points.map(p=>`${30+(p[0]-.15)*1000},${50+(2.25-p[1])*1000}`).join(' '));
            line.setAttribute('fill','none');line.setAttribute('stroke',color);line.setAttribute('stroke-width','3');svg.append(line);
        }
        panel.append(svg);
        const note=document.createElement('p');note.textContent='蓝：正式参考；橙：合成闭环。'+data.blockers.join('；');panel.append(note);
    }catch{out.textContent='离线报告不可用，不开放实车执行';}
}
preview();
