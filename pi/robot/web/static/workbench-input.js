// 只把明确的方向按键转换为点动；编辑、快捷键和无关keyup不能产生动作。
const keys = {KeyW:'1,0,0', KeyS:'-1,0,0', KeyA:'0,1,0', KeyD:'0,-1,0', KeyQ:'0,0,1', KeyE:'0,0,-1'};

export function isEditing(event) {
    return /INPUT|SELECT|TEXTAREA/.test(event.target?.tagName || '') || event.target?.isContentEditable === true;
}

export function motionDirection(event) {
    if (event.repeat || event.ctrlKey || event.metaKey || event.altKey || isEditing(event)) return null;
    return keys[event.code] || null;
}

export function matchesRelease(event, heldDirection) {
    return typeof heldDirection === 'string' && keys[event.code] === heldDirection;
}

export function stopShortcut(event) {
    if (event.repeat || event.ctrlKey || event.metaKey || event.altKey || isEditing(event)) return false;
    if (event.code === 'Escape') return true;
    if (event.code !== 'Space') return false;
    // 空格聚焦按钮时属于原生激活操作，不再额外触发全局停止。
    const target = event.target;
    return !(/BUTTON|A|SUMMARY/.test(target?.tagName || '') ||
        target?.closest?.('button,a,summary,[role="button"]'));
}
