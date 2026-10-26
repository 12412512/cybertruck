#ifndef MECH_AXIS_H
#define MECH_AXIS_H

#include "mech_guard.h"

enum
{
    AXIS_IDLE, AXIS_ACCEPTED, AXIS_SENDING, AXIS_RUNNING, AXIS_SETTLING,
    AXIS_STOPPING, AXIS_COMPLETE, AXIS_FAILED, AXIS_CANCELLED
};
enum
{
    AXIS_ID = 100, AXIS_BUSY, AXIS_TX, AXIS_ACK, AXIS_TIMEOUT,
    AXIS_OTHER, AXIS_REVERSE, AXIS_TARGET, AXIS_CANCEL, AXIS_ZERO_CHANGED
};
typedef struct
{
    uint32_t id, zero_version, timeout_ms;
    int32_t target_um;
    uint16_t rpm;
    uint8_t axis;
} AxisGoal;
typedef struct
{
    MechGuard *guard;
    uint64_t boot, session;
    AxisGoal goal;
    uint32_t state, reason, started_ms, sent_ms, settled_ms;
    uint32_t last_sequence[8], ack_barrier, zero_version;
    int32_t pulse_position[2], goal_pulses;
    int64_t before_actual, before_target, before_other, delta_raw, anchor[2];
    uint8_t ack_seen, emitted, settled_samples, stop_sent_mask, stop_confirmed, manual;
} Axis;

/* 无HAL、无Bus、无自动使能/PWM。仅产出意图，未来唯一设备所有者显式消费。
 * 当前不接App/Comm；所有Sent/ACK必须来自本动作真实证据，不能用计时伪造。 */
void Axis_Init(Axis *axis, MechGuard *guard);
uint32_t Axis_Begin(Axis *axis, const AxisGoal *goal, uint32_t now_ms,
                    uint8_t arm_known, int32_t arm_mdeg);
/* 只允许由显式手动调试入口调用；不用于Pose自动过渡。 */
uint32_t Axis_Manual(Axis *axis, const AxisGoal *goal, uint32_t now_ms);
uint8_t Axis_Take(Axis *axis, BusFrame frames[2], uint32_t now_ms);
void Axis_Sent(Axis *axis, uint32_t id, uint8_t tx_ok, uint32_t now_ms);
void Axis_Ack(Axis *axis, const MotorReply *reply, uint32_t now_ms);
void Axis_Poll(Axis *axis, uint32_t now_ms, uint8_t base_stopped);
void Axis_Cancel(Axis *axis, uint32_t now_ms);
/* 上层姿态故障时，即使单轴空闲也保留两轴停止请求；仍无IO。 */
void Axis_Stop(Axis *axis, uint32_t reason, uint32_t now_ms);
/* mask位0/1为ID5/6停止事务TXOK；仍需新反馈确认实物停稳。 */
void Axis_StopSent(Axis *axis, uint8_t mask, uint32_t now_ms);

#endif
