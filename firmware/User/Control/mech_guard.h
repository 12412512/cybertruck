#ifndef MECH_GUARD_H
#define MECH_GUARD_H

#include "drive.h"

enum
{
    MECH_OK, MECH_NO_ZERO, MECH_STALE, MECH_UNSTABLE, MECH_DRIVER,
    MECH_CONFIG, MECH_MANUAL, MECH_SESSION, MECH_RANGE, MECH_BASE,
    MECH_COLLISION, MECH_VERSION, MECH_FRAME, MECH_ALREADY_ZERO, MECH_FAULT
};

typedef struct
{
    uint32_t captured, tick_ms, reason, axis, zero_version, flags;
    int64_t actual[2], target[2], zero_actual[2], zero_target[2], position_um[2];
    int32_t rpm[2];
    uint32_t age_ms[2];
} MechFailure;

typedef struct
{
    uint64_t boot, session;
    MotorReply sample[8]; /* ID5/6各自的35速度、36实际、3A状态、33目标 */
    int64_t zero_actual[2], zero_target[2], anchor_actual[2], anchor_target[2];
    uint32_t zero_version, stable_since, stable_samples, completed_sequence[8];
    uint32_t checked_sequence[8]; /* 最近一次通过静止、时效和锚点检查的反馈；不增加稳定轮数。 */
    uint32_t rejected, fault;
    uint8_t zero_valid, config_mask, base_stopped, stable_started, stop_required;
    uint8_t manual_range; /* 手动调试不套用自动姿态的49/50mm上限。 */
    uint8_t concurrent_axes; /* 仅联调组可登记正在执行的轴，STOP即撤销。 */
    MechFailure failure; /* 首次触发时冻结，STOP和后续GET不覆盖。 */
} MechGuard;

/* 本模块不发送CAN，不写驱动零点，不启动PWM。当前未接App或Comm。 */
void Mech_Init(MechGuard *guard, uint64_t boot, uint64_t session);
void Mech_Config(MechGuard *guard, uint8_t verified_mask);
uint8_t Mech_Feed(MechGuard *guard, const MotorReply *reply, uint32_t now_ms);
void Mech_Poll(MechGuard *guard, uint32_t now_ms, uint8_t base_stopped);
void Mech_Invalidate(MechGuard *guard, uint32_t reason);
void Mech_Freeze(MechGuard *guard, uint32_t reason, uint8_t axis, uint32_t now_ms);
int64_t Mech_RawTolerance(uint8_t axis, int32_t tolerance_um);
/* 仅验证正常人工停止后的重新准备；不修改故障、零点或输出。 */
uint8_t Mech_CanPrepare(const MechGuard *guard, uint32_t now_ms);
uint8_t Mech_Zero(MechGuard *guard, uint32_t now_ms, uint8_t manual_confirmed);
/* 只检查当前基准、反馈与底盘静止，不产生轴运动或PWM。 */
uint8_t Mech_Ready(const MechGuard *guard, uint32_t now_ms, uint32_t zero_version);
uint8_t Mech_GroupReady(const MechGuard *guard, uint32_t now_ms, uint32_t zero_version);
int64_t Mech_Position(const MechGuard *guard, uint8_t axis_index);
uint8_t Mech_CheckAxis(const MechGuard *guard, uint32_t now_ms, uint32_t zero_version,
                       uint8_t axis_index, int32_t target_um, uint8_t arm_known, int32_t arm_mdeg);
/* 人工单轴调试：保留Ready和范围，几何碰撞由本次现场人工核对。 */
uint8_t Mech_AxisManual(const MechGuard *guard, uint32_t now_ms, uint32_t zero_version,
                        uint8_t axis_index, int32_t target_um);
uint8_t Mech_CheckRotate(const MechGuard *guard, uint32_t now_ms, uint32_t zero_version,
                         uint8_t tray, uint8_t arm_known, int32_t arm_mdeg);
/* 纯公式预览，不代表角度反馈或PWM已经发出。0表示参数无效。 */
uint16_t Mech_Pulse(uint8_t channel, int32_t target_mdeg);
uint32_t Mech_Pulses(uint8_t axis_index, int32_t target_um);
uint32_t Mech_ManualPulses(uint8_t axis_index, int32_t target_um);

#endif
