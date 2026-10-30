/* 实时状态只更新变化项，避免轮询反复替换节点、打断选中和读屏。 */
export function setText(node,value) {
    const text=String(value??'');
    if(node.textContent!==text)node.textContent=text;
}
export function setSteps(node,steps) {
    const key=JSON.stringify(steps.map(([name,done])=>[name,!!done]));
    if(node.dataset.stepsKey===key)return;
    node.dataset.stepsKey=key;
    node.replaceChildren(...steps.map(([name,done])=>{
        const item=node.ownerDocument.createElement('span');
        item.textContent=`${done?'✓':'○'} ${name}`;
        item.className=done?'ready':'';return item;
    }));
}
