// 两种离线模型各有身份；执行链结果不会写入真实设备状态或成功计数。
export function modelView(state) {
    const check = state?.execution_check;
    if (!check?.visible || check.source_plan_id !== state.planning?.plan?.id || !check.trace?.length) return state.simulation;
    const execution = check.execution || {};
    return {...state.simulation, mode:check.status === 'RUNNING' ? 'RUNNING' : check.status === 'COMPLETE' ? 'COMPLETE' : check.status === 'CANCELLED' ? 'STOPPED' : 'FAILED',
        pose:check.trace.at(-1), trace:check.trace, session:check.id, stopped:!!check.stop_confirmed,
        completed:check.completed ?? check.segment ?? 0, total:check.total, elapsed_s:check.elapsed_s,
        rpm:check.rpm || [], position_error_m:execution.position_error_m,
        yaw_error_rad:execution.yaw_error_rad, fault:execution.fault,
        reason:check.reason || '', active_plan_id:check.source_plan_id, odom_origin_ms:'执行链模型'};
}
