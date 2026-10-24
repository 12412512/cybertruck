#include "mech_axis.h"
#include "mech_hw.h"
#include <string.h>

static int64_t Abs(int64_t value)
{
    return value < 0 ? -value : value;
}

static int64_t Raw(int32_t pulses)
{
    /* 旧工程每圈65536编码器单位、3200命令脉冲，保留有符号四舍五入。 */
    return ((int64_t)pulses * 65536 + (pulses < 0 ? -1600 : 1600)) / 3200;
}

static int Busy(const Axis *axis)
{
    return axis->state >= AXIS_ACCEPTED && axis->state <= AXIS_STOPPING;
}

static void Barrier(Axis *axis)
{
    unsigned index;
    for (index = 0; index < 8; index++)
        axis->last_sequence[index] = axis->guard->sample[index].rx_sequence;
}

static void Fail(Axis *axis, uint32_t reason, uint32_t now)
{
    if (axis->state == AXIS_STOPPING)
        return;
    axis->reason = reason;
    axis->state = AXIS_STOPPING;
    axis->stop_sent_mask = axis->stop_confirmed = axis->settled_samples = 0;
    axis->settled_ms = now;
    Mech_Invalidate(axis->guard, reason);
    /* stop_required只表示请求，不冒充已经下发ID5/6停止。 */
    axis->guard->stop_required = 1;
    Barrier(axis);
}

void Axis_Init(Axis *axis, MechGuard *guard)
{
    if (!axis)
        return;
    memset(axis, 0, sizeof(*axis));
    axis->guard = guard;
    if (guard)
    {
        axis->boot = guard->boot;
        axis->session = guard->session;
    }
}

static int Same(const AxisGoal *first, const AxisGoal *second)
{
    return first->id == second->id && first->zero_version == second->zero_version &&
           first->timeout_ms == second->timeout_ms && first->target_um == second->target_um &&
           first->rpm == second->rpm && first->axis == second->axis;
}

uint32_t Axis_Begin(Axis *axis, const AxisGoal *goal, uint32_t now,
                    uint8_t arm_known, int32_t arm_mdeg)
{
    MechGuard *guard;
    uint32_t result;
    int32_t pulses, difference;
    int sign;
    if (!axis || !goal || !(guard = axis->guard) || guard->boot != axis->boot ||
        guard->session != axis->session)
        return MECH_SESSION;
    if (!goal->id || goal->id < axis->goal.id)
        return AXIS_ID;
    /* 相同动作只返回原状态；冲突或旧ID拒绝，绝不重新发FD。 */
    if (goal->id == axis->goal.id)
        return Same(goal, &axis->goal) ? MECH_OK : AXIS_ID;
    if (Busy(axis))
        return AXIS_BUSY;
    if (!goal->rpm || goal->rpm > MECH_AXIS_MAX_RPM || goal->timeout_ms < 500 ||
        goal->timeout_ms > 15000)
        return MECH_RANGE;
    result = Mech_CheckAxis(guard, now, goal->zero_version, goal->axis, goal->target_um,
                            arm_known, arm_mdeg);
    if (result)
        return result;
    if (axis->zero_version != guard->zero_version)
    {
        axis->pulse_position[0] = axis->pulse_position[1] = 0;
        axis->zero_version = guard->zero_version;
    }
    sign = goal->axis ? MECH_REACH_SIGN : MECH_LIFT_SIGN;
    if (Abs(guard->sample[goal->axis * 4 + 3].value - guard->zero_target[goal->axis] -
            sign * Raw(axis->pulse_position[goal->axis])) > MECH_AXIS_TARGET_RAW)
    {
        Fail(axis, AXIS_TARGET, now);
        return AXIS_TARGET;
    }
    pulses = (int32_t)Mech_Pulses(goal->axis, goal->target_um);
    difference = pulses - axis->pulse_position[goal->axis];
    if (!difference)
        return MECH_RANGE;
    axis->goal = *goal;
    axis->goal_pulses = pulses;
    axis->delta_raw = sign * Raw(difference);
    axis->before_actual = guard->sample[goal->axis * 4 + 1].value;
    axis->before_target = guard->sample[goal->axis * 4 + 3].value;
    axis->before_other = guard->sample[(1U - goal->axis) * 4 + 1].value;
    axis->started_ms = now;
    axis->reason = 0;
    axis->ack_seen = axis->emitted = axis->settled_samples = axis->stop_confirmed = 0;
    axis->ack_barrier = guard->sample[0].rx_sequence;
    for (unsigned index = 1; index < 8; index++)
        if ((int32_t)(guard->sample[index].rx_sequence - axis->ack_barrier) > 0)
            axis->ack_barrier = guard->sample[index].rx_sequence;
    axis->state = AXIS_ACCEPTED;
    Barrier(axis);
    return MECH_OK;
}

uint8_t Axis_Take(Axis *axis, BusFrame frames[2], uint32_t now)
{
    int32_t difference;
    uint8_t count;
    if (!axis || !frames || axis->state != AXIS_ACCEPTED || axis->emitted)
        return 0;
    Axis_Poll(axis, now, axis->guard->base_stopped);
    if (axis->state != AXIS_ACCEPTED)
        return 0;
    difference = axis->goal_pulses - axis->pulse_position[axis->goal.axis];
    /* 保留旧轴FD模式0：相对上一输入目标；以目标账本检查手动/外部变更。 */
    count = Drive_Build(axis->goal.axis ? MECH_REACH_ID : MECH_LIFT_ID, 0xFD,
                        (uint8_t)(axis->delta_raw < 0), axis->goal.rpm, 1,
                        (uint32_t)(difference < 0 ? -difference : difference), 0, 0, frames);
    if (count != 2)
    {
        Fail(axis, AXIS_TX, now);
        return 0;
    }
    axis->emitted = 1;
    axis->sent_ms = now;
    axis->state = AXIS_SENDING;
    return count;
}

void Axis_Sent(Axis *axis, uint32_t id, uint8_t tx_ok, uint32_t now)
{
    if (!axis || axis->state != AXIS_SENDING || id != axis->goal.id)
        return;
    if (!tx_ok || (uint32_t)(now - axis->sent_ms) >= MECH_AXIS_TX_MS)
        Fail(axis, AXIS_TX, now);
    else
    {
        axis->sent_ms = now;
        axis->state = AXIS_RUNNING;
        Barrier(axis);
    }
}

void Axis_Ack(Axis *axis, const MotorReply *reply, uint32_t now)
{
    if (!axis || !reply || axis->state < AXIS_SENDING || axis->state > AXIS_SETTLING ||
        !reply->valid || reply->code != 0xFD ||
        reply->address != (axis->goal.axis ? MECH_REACH_ID : MECH_LIFT_ID) ||
        (int32_t)(reply->rx_sequence - axis->ack_barrier) <= 0 ||
        (uint32_t)(now - reply->tick) > MECH_FEEDBACK_AGE_MS ||
        (uint32_t)(reply->tick - axis->sent_ms) >= 0x80000000UL)
        return;
    if (reply->result == 2)
        axis->ack_seen = 1;
    else if (reply->result != 0x9F)
        Fail(axis, AXIS_ACK, now);
}

static int NewRound(Axis *axis, uint32_t now)
{
    unsigned index;
    for (index = 0; index < 8; index++)
    {
        const MotorReply *sample = axis->guard->sample + index;
        if (!sample->valid || (uint32_t)(now - sample->tick) > MECH_FEEDBACK_AGE_MS ||
            (uint32_t)(sample->tick - axis->sent_ms) >= 0x80000000UL ||
            (int32_t)(sample->rx_sequence - axis->last_sequence[index]) <= 0)
            return 0;
    }
    Barrier(axis);
    return 1;
}

static void Settle(Axis *axis, uint32_t now, uint8_t stopping)
{
    MechGuard *guard = axis->guard;
    unsigned moving = axis->goal.axis * 4;
    int64_t actual[2] = {guard->sample[1].value, guard->sample[5].value};
    if (!NewRound(axis, now))
        return;
    if (guard->sample[0].value || guard->sample[4].value ||
        (!stopping && (!axis->ack_seen || !(guard->sample[moving + 2].value & 2) ||
         Abs(guard->sample[moving + 1].value - axis->before_actual - axis->delta_raw) > MECH_AXIS_ARRIVE_RAW ||
         Abs(guard->sample[moving + 3].value - axis->before_target - axis->delta_raw) > MECH_AXIS_TARGET_RAW ||
         Abs(guard->sample[moving + 3].value - guard->sample[moving + 1].value) > MECH_AXIS_ARRIVE_RAW)))
    {
        axis->settled_samples = 0;
        return;
    }
    if (!axis->settled_samples || Abs(actual[0] - axis->anchor[0]) > MECH_STABLE_RAW ||
        Abs(actual[1] - axis->anchor[1]) > MECH_STABLE_RAW)
    {
        axis->anchor[0] = actual[0];
        axis->anchor[1] = actual[1];
        axis->settled_samples = 1;
        axis->settled_ms = now;
    }
    else if (axis->settled_samples < 255)
        axis->settled_samples++;
    if (stopping)
    {
        if (axis->settled_samples >= MECH_STABLE_SAMPLES &&
            (uint32_t)(now - axis->settled_ms) >= MECH_ZERO_STABLE_MS)
        {
            axis->stop_confirmed = 1;
            axis->guard->stop_required = 0;
            axis->state = axis->reason == AXIS_CANCEL ? AXIS_CANCELLED : AXIS_FAILED;
        }
    }
    else
    {
        axis->state = AXIS_SETTLING;
        if (axis->settled_samples >= MECH_AXIS_ARRIVE_SAMPLES)
        {
            axis->pulse_position[axis->goal.axis] = axis->goal_pulses;
            axis->state = AXIS_COMPLETE;
        }
    }
}

void Axis_Poll(Axis *axis, uint32_t now, uint8_t base_stopped)
{
    MechGuard *guard;
    unsigned moving, other;
    int64_t traveled, tolerance;
    if (!axis || !(guard = axis->guard) || !Busy(axis))
        return;
    if (axis->state == AXIS_STOPPING)
    {
        if (axis->stop_sent_mask == 3 && guard->boot == axis->boot && guard->session == axis->session)
            Settle(axis, now, 1);
        return;
    }
    Mech_Poll(guard, now, base_stopped);
    if (guard->fault || !guard->zero_valid || guard->zero_version != axis->zero_version ||
        guard->boot != axis->boot || guard->session != axis->session || !base_stopped)
    {
        Fail(axis, guard->fault ? guard->fault : !base_stopped ? MECH_BASE : AXIS_ZERO_CHANGED, now);
        return;
    }
    if ((uint32_t)(now - axis->started_ms) >= axis->goal.timeout_ms)
    {
        Fail(axis, AXIS_TIMEOUT, now);
        return;
    }
    if (axis->state == AXIS_SENDING && (uint32_t)(now - axis->sent_ms) >= MECH_AXIS_TX_MS)
    {
        Fail(axis, AXIS_TX, now);
        return;
    }
    if (axis->state < AXIS_RUNNING)
    {
        if (axis->state == AXIS_ACCEPTED &&
            (guard->sample[0].value || guard->sample[4].value ||
             Abs(guard->sample[axis->goal.axis * 4 + 1].value - axis->before_actual) > MECH_STABLE_RAW ||
             Abs(guard->sample[axis->goal.axis * 4 + 3].value - axis->before_target) > MECH_AXIS_TARGET_RAW ||
             Abs(guard->sample[(1U - axis->goal.axis) * 4 + 1].value - axis->before_other) > MECH_AXIS_OTHER_RAW))
            Fail(axis, AXIS_TARGET, now);
        return;
    }
    moving = axis->goal.axis * 4;
    other = (1U - axis->goal.axis) * 4;
    if (guard->sample[other].value || Abs(guard->sample[other + 1].value - axis->before_other) > MECH_AXIS_OTHER_RAW)
        Fail(axis, AXIS_OTHER, now);
    else if (!axis->ack_seen && (uint32_t)(now - axis->sent_ms) >= MECH_AXIS_ACK_MS)
        Fail(axis, AXIS_ACK, now);
    else
    {
        traveled = guard->sample[moving + 1].value - axis->before_actual;
        tolerance = (MECH_FEEDBACK_TOL_UM * (int64_t)MECH_RAW_PER_320 +
                     (axis->goal.axis ? MECH_REACH_UM_PER_320 : MECH_LIFT_UM_PER_320) - 1) /
                    (axis->goal.axis ? MECH_REACH_UM_PER_320 : MECH_LIFT_UM_PER_320);
        if (Abs(guard->sample[moving].value) > MECH_AXIS_SPEED_LIMIT ||
            Abs(traveled) > Abs(axis->delta_raw) + tolerance ||
            traveled * (axis->delta_raw > 0 ? 1 : -1) < -MECH_AXIS_REVERSE_RAW)
            Fail(axis, AXIS_REVERSE, now);
        else
            Settle(axis, now, 0);
    }
}

void Axis_Cancel(Axis *axis, uint32_t now)
{
    if (axis && axis->guard && Busy(axis))
        Fail(axis, AXIS_CANCEL, now);
}

void Axis_StopSent(Axis *axis, uint8_t mask, uint32_t now)
{
    if (!axis || axis->state != AXIS_STOPPING)
        return;
    if (!(mask & 3 & ~axis->stop_sent_mask))
        return;
    axis->stop_sent_mask |= (uint8_t)(mask & 3);
    axis->sent_ms = now;
    axis->settled_samples = 0;
    axis->settled_ms = now;
    Barrier(axis);
}

void Axis_Stop(Axis *axis, uint32_t reason, uint32_t now)
{
    if (axis && axis->guard)
        Fail(axis, reason ? reason : AXIS_CANCEL, now);
}
