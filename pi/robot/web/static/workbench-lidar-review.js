// 已结束实扫的核对视图；只读取本地摘要，不接收设备命令、不修改规划草稿。
const $ = id => document.getElementById(id);

export function reviewLines(value) {
    if (value?.domain !== 'lidar_commissioning_review' || value.live !== false ||
        value.historical !== true || value.applied !== false || value.real_map_registered !== false ||
        value.auto_execution_ready !== false || value.authorizes_motion !== false) {
        throw new Error('核对记录来源不符，不能作为地图或实车许可');
    }
    const install = value.installation, offset = install.translation_m;
    if (![offset.x, offset.y, install.mount_yaw_rad, value.independent_error_m,
        value.independent_limit_m].every(Number.isFinite)) throw new Error('安装或复核数据缺失');
    return [
        `安装草稿：前后${(offset.x*1000).toFixed(0)} / 左右${(offset.y*1000).toFixed(0)} mm · 正前方向−Y（${(install.mount_yaw_rad*180/Math.PI).toFixed(0)}°）· 盲区＋Y`,
        `独立P4误差 ${(value.independent_error_m*1000).toFixed(1)} mm / 验收线 ${(value.independent_limit_m*1000).toFixed(0)} mm；诊断图不进入路线规划`,
        `已结束的历史核对 · ${value.information.firmware} · 配准与道路覆盖待验收 · 记录${value.version.slice(0,12)}`
    ];
}

export function reviewPoints(value) {
    reviewLines(value);
    if (!Array.isArray(value.points) || value.points.length !== 4) throw new Error('需要四个标记的核对记录');
    for (const row of value.points) {
        if (!Array.isArray(row.map_m) || row.map_m.length !== 2 ||
            !row.map_m.every(n => Number.isFinite(n) && n >= 0 && n <= 2.4)) throw new Error('标记坐标越界');
    }
    return value.points;
}

function draw(svg, tag, attributes, content) {
    const node = document.createElementNS('http://www.w3.org/2000/svg', tag);
    for (const [name, value] of Object.entries(attributes)) node.setAttribute(name, value);
    if (content) node.textContent = content;
    svg.append(node);
}

function map(value) {
    const points = reviewPoints(value), svg = $('lidar-review-map');
    svg.replaceChildren();
    draw(svg, 'rect', {x:0,y:-2400,width:1100,height:1000,fill:'#f5f8fa',stroke:'#8496a4','stroke-width':3});
    const [x,y] = value.nominal_body_pose_m_rad;
    draw(svg, 'rect', {x:x*1000-145,y:-y*1000-135,width:290,height:270,fill:'#d3e9e6',stroke:'#147c88','stroke-width':4});
    draw(svg, 'text', {x:x*1000+175,y:-y*1000,'font-size':30,fill:'#147c88'}, 'O · 名义起点 → +X');
    const [rx,ry] = value.nominal_radar_pose_m_rad;
    draw(svg, 'circle', {cx:rx*1000,cy:-ry*1000,r:13,fill:'#147c88'});
    draw(svg, 'text', {x:rx*1000+25,y:-ry*1000+32,'font-size':28,fill:'#147c88'}, '雷达 · 正前向下');
    for (const row of points) {
        draw(svg, 'circle', {cx:row.map_m[0]*1000,cy:-row.map_m[1]*1000,r:12,fill:'#273746'});
        draw(svg, 'text', {x:row.map_m[0]*1000+20,y:-row.map_m[1]*1000-20,'font-size':30,fill:'#273746'}, row.label);
    }
    const predicted = value.independent_prediction_m;
    if (Array.isArray(predicted) && predicted.length === 2 && predicted.every(n => Number.isFinite(n) && n >= 0 && n <= 2.4)) {
        const actual = points[3].map_m;
        draw(svg, 'line', {x1:actual[0]*1000,y1:-actual[1]*1000,x2:predicted[0]*1000,y2:-predicted[1]*1000,stroke:'#bd681c','stroke-width':5});
        draw(svg, 'circle', {cx:predicted[0]*1000,cy:-predicted[1]*1000,r:20,fill:'none',stroke:'#bd681c','stroke-width':4});
    }
}

export function initReview() {
    async function refresh() {
        try {
            const response = await fetch('/static/lidar-review.json', {cache:'no-store',signal:AbortSignal.timeout(2000)});
            if (!response.ok) throw new Error('本地核对摘要尚未生成');
            const value = await response.json(), lines = reviewLines(value);
            map(value);
            for (const [index,id] of ['lidar-review-install','lidar-review-error','lidar-review-source'].entries()) $(id).textContent = lines[index];
            const body = $('lidar-review-checks'); body.replaceChildren();
            for (const check of value.checks) {
                const row = document.createElement('li');
                const status = {RECORDED:'已记录',HISTORY_PASS:'历史样本通过',PENDING:'待验收'}[check.status] || '未知';
                row.textContent = `${check.name}：${status} · ${check.detail}`; body.append(row);
            }
            $('lidar-review-contract').textContent = `后续实车流程：${value.next_run_contract.join(' → ')}。本页不启动该流程。`;
        } catch (error) {
            $('lidar-review-install').textContent = `核对视图不可用：${error.message}`;
            for (const id of ['lidar-review-error','lidar-review-source','lidar-review-contract']) $(id).textContent = '';
            $('lidar-review-map').replaceChildren(); $('lidar-review-checks').replaceChildren();
        }
    }
    $('lidar-review-refresh').onclick = refresh;
    refresh();
}
