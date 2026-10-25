import {canAct, targetError} from './workbench-mech-guide.js?v=2';

/* 实车机械页纯显示规则；不保存令牌、零点或模型状态。 */
export const actuators = [
    {id:'lift', name:'升降', pin:'CAN · ID5', unit:'mm', min:0, max:49, initial:0},
    {id:'reach', name:'伸缩', pin:'CAN · ID6', unit:'mm', min:0, max:50, initial:0},
    {id:'tray', name:'料盘', pin:'PE9', unit:'°', min:0, max:270, initial:0},
    {id:'arm', name:'转臂', pin:'PE11', unit:'°', min:83, max:180, initial:180},
    {id:'gripper', name:'夹爪', pin:'PE13', unit:'°', min:0, max:98, initial:98}
];
export function freshDevice(state, receivedAt, now) {
    const age = state.state_age_ms;
    return state.connected === true && state.domain === 'physical' &&
        Number.isFinite(age) && age >= 0 && now >= receivedAt && age + now - receivedAt < 250;
}
export function blockedReasons(state, owned, pending) {
    const reasons = [], m = state.mechanism;
    if (!state.connected || !m) reasons.push(state.reason || '设备反馈未就绪');
    if (state.base?.fault || m?.fault) reasons.push(`故障锁存：底盘 ${state.base?.fault || 0} / 机构 ${m?.fault || 0}`);
    if (state.site_ready === false) reasons.push('设备核心保持只读，现场动作许可未开放');
    if (!owned) reasons.push('尚未取得实车控制权');
    if (pending) reasons.push('上一请求结果待核对，请勿重复发送');
    if (m && !m.attached) reasons.push('机构反馈尚未接入');
    if (m?.attached && m.config_mask !== 3) reasons.push('两轴驱动配置尚未核对完整');
    if (m?.attached && !m.zero_valid) reasons.push('零点无效：需在最高 / 最近端人工建零');
    return reasons;
}
export function allowed(state, owned, pending, action) {
    if (action === 'stop') return true;
    if (action === 'claim') return state.connected === true && !pending;
    if (action === 'load') return true;
    if (action === 'attach' || action === 'teach') return owned && state.connected === true && !pending &&
        (action !== 'attach' || (!state.base?.fault && !state.mechanism?.fault && state.base?.stopped && !state.mechanism?.attached));
    return canAct(state, owned, pending, action);
}
export function moveArgs(id, raw, rpm=5, release=false) {
    if (raw === null || raw === undefined || String(raw).trim() === '') throw Error('请填写明确目标');
    const value = Number(raw), reason = targetError(id, value);
    if (reason) throw Error(reason);
    const linear = id === 'lift' || id === 'reach';
    if (linear && (!Number.isInteger(rpm) || rpm < 1 || rpm > 30)) throw Error('轴速度必须为1～30RPM的整数');
    return {axis:id, [linear ? 'target_m' : 'target_rad']:value * (linear ? .001 : Math.PI/180),
        ...(linear ? {rpm} : {}), release_confirmed:release === true};
}
