#include "mech_guard.h"
#include "mech_hw.h"
#include "bus_config.h"
#include <string.h>

static const int32_t distance_um[2] = {MECH_LIFT_UM_PER_320, MECH_REACH_UM_PER_320};
static const int32_t maximum_um[2] = {MECH_LIFT_MAX_UM, MECH_REACH_MAX_UM};
static const int8_t signs[2] = {MECH_LIFT_SIGN, MECH_REACH_SIGN};

static int64_t Abs(int64_t value)
{
    return value < 0 ? -value : value;
}

static uint8_t Fresh(const MechGuard *guard, uint32_t now)
{
    uint8_t index;
    for (index = 0; index < 8; index++)
    {
        if (!guard->sample[index].valid || !guard->sample[index].rx_sequence ||
            (uint32_t)(now - guard->sample[index].tick) > MECH_FEEDBACK_AGE_MS)
            return MECH_STALE;
    }
    return MECH_OK;
}

static uint8_t Stationary(const MechGuard *guard)
{
    uint8_t axis, index;
    for (axis = 0; axis < 2; axis++)
    {
        index = (uint8_t)(axis * 4);
        if ((guard->sample[index + 2].value & 0x0D) != 1)
            return MECH_DRIVER;
        if (guard->sample[index].value ||
            Abs(guard->sample[index + 1].value - guard->sample[index + 3].value) > MECH_STILL_RAW)
            return MECH_UNSTABLE;
    }
    return MECH_OK;
}

static uint8_t Stable(const MechGuard *guard, uint32_t now)
{
    uint8_t index;
    if (!guard->stable_started || guard->stable_samples < MECH_STABLE_SAMPLES ||
        (uint32_t)(now - guard->stable_since) < MECH_ZERO_STABLE_MS)
        return MECH_UNSTABLE;
    for (index = 0; index < 8; index++)
        if (guard->completed_sequence[index] != guard->sample[index].rx_sequence)
            return MECH_UNSTABLE;
    return MECH_OK;
}

static uint8_t Ready(const MechGuard *guard, uint32_t now, uint32_t version)
{
    uint8_t result;
    if (guard->fault)
        return guard->fault <= MECH_FAULT ? (uint8_t)guard->fault : MECH_FAULT;
    if (!guard->zero_valid)
        return MECH_NO_ZERO;
    if (version != guard->zero_version)
        return MECH_VERSION;
    if (!guard->base_stopped)
        return MECH_BASE;
    result = Fresh(guard, now);
    if (result)
        return result;
    result = Stationary(guard);
    return result ? result : Stable(guard, now);
}

void Mech_Init(MechGuard *guard, uint64_t boot, uint64_t session)
{
    if (!guard)
        return;
    memset(guard, 0, sizeof(*guard));
    guard->boot = boot;
    guard->session = session;
}

void Mech_Invalidate(MechGuard *guard, uint32_t reason)
{
    if (!guard)
        return;
    if (guard->zero_valid)
        guard->stop_required = 1;
    guard->zero_valid = 0;
    guard->stable_started = 0;
    guard->stable_samples = 0;
    if (!guard->fault)
        guard->fault = reason ? reason : MECH_MANUAL;
}

void Mech_Config(MechGuard *guard, uint8_t verified_mask)
{
    if (!guard)
        return;
    if (guard->zero_valid && verified_mask != 3)
        Mech_Invalidate(guard, MECH_CONFIG);
    guard->config_mask = (uint8_t)(verified_mask & 3);
}

uint8_t Mech_Feed(MechGuard *guard, const MotorReply *reply, uint32_t now)
{
    uint8_t index, axis;
    MotorReply *previous;
    if (!guard || !reply || !reply->valid || !reply->rx_sequence ||
        (reply->address != MECH_LIFT_ID && reply->address != MECH_REACH_ID) ||
        (uint32_t)(now - reply->tick) > MECH_FEEDBACK_AGE_MS)
        return MECH_FRAME;
    axis = (uint8_t)(reply->address == MECH_REACH_ID);
    if (reply->code == 0x35)
    {
        if (reply->value < -(int64_t)MOTOR_MAX_REPLY_RPM || reply->value > MOTOR_MAX_REPLY_RPM)
            return MECH_FRAME;
        index = 0;
    }
    else if (reply->code == 0x36)
        index = 1;
    else if (reply->code == 0x3A)
        index = 2;
    else if (reply->code == 0x33)
        index = 3;
    else
        return MECH_FRAME;
    if ((reply->code == 0x33 || reply->code == 0x36) &&
        (reply->value < -(int64_t)UINT32_MAX || reply->value > UINT32_MAX))
        return MECH_FRAME;
    if (reply->code == 0x3A && (reply->value < 0 || reply->value > 255))
        return MECH_FRAME;
    index = (uint8_t)(index + axis * 4);
    previous = &guard->sample[index];
    if (previous->valid && ((int32_t)(reply->rx_sequence - previous->rx_sequence) <= 0 ||
                            (int32_t)(reply->tick - previous->tick) < 0))
    {
        guard->rejected++;
        return MECH_FRAME;
    }
    *previous = *reply;
    return MECH_OK;
}

void Mech_Poll(MechGuard *guard, uint32_t now, uint8_t base_stopped)
{
    uint8_t index, axis, changed = 1, result;
    if (!guard)
        return;
    guard->base_stopped = (uint8_t)!!base_stopped;
    result = Fresh(guard, now);
    if (!result)
        result = Stationary(guard);
    if (guard->zero_valid && (result == MECH_STALE || result == MECH_DRIVER))
        Mech_Invalidate(guard, result);
    if (guard->zero_valid)
    {
        for (axis = 0; axis < 2; axis++)
        {
            int64_t position = Mech_Position(guard, axis);
            if (position < -MECH_FEEDBACK_TOL_UM || position > maximum_um[axis] + MECH_FEEDBACK_TOL_UM)
                Mech_Invalidate(guard, MECH_RANGE);
        }
    }
    if (result || !guard->base_stopped || guard->config_mask != 3 || guard->fault)
    {
        guard->stable_started = 0;
        guard->stable_samples = 0;
        return;
    }
    for (index = 0; index < 8; index++)
        if (guard->completed_sequence[index] == guard->sample[index].rx_sequence)
            changed = 0;
    if (!changed)
        return; /* 重复轮询同一批反馈不能积累静止证据。 */
    if (guard->stable_started)
    {
        for (axis = 0; axis < 2; axis++)
        {
            if (Abs(guard->sample[axis * 4 + 1].value - guard->anchor_actual[axis]) > MECH_STABLE_RAW ||
                Abs(guard->sample[axis * 4 + 3].value - guard->anchor_target[axis]) > MECH_STABLE_RAW)
                guard->stable_started = 0;
        }
    }
    if (!guard->stable_started)
    {
        guard->stable_started = 1;
        guard->stable_samples = 0;
        guard->stable_since = now;
        for (axis = 0; axis < 2; axis++)
        {
            guard->anchor_actual[axis] = guard->sample[axis * 4 + 1].value;
            guard->anchor_target[axis] = guard->sample[axis * 4 + 3].value;
        }
    }
    for (index = 0; index < 8; index++)
        guard->completed_sequence[index] = guard->sample[index].rx_sequence;
    if (guard->stable_samples < 255)
        guard->stable_samples++;
}

uint8_t Mech_Zero(MechGuard *guard, uint32_t now, uint8_t manual_confirmed)
{
    uint8_t axis, result;
    if (!guard || !guard->boot || !guard->session)
        return MECH_SESSION;
    if (guard->fault)
        return guard->fault <= MECH_FAULT ? (uint8_t)guard->fault : MECH_FAULT;
    if (guard->zero_valid)
        return MECH_ALREADY_ZERO;
    if (!manual_confirmed)
        return MECH_MANUAL;
    if (!guard->base_stopped)
        return MECH_BASE;
    if (guard->config_mask != 3)
        return MECH_CONFIG;
    result = Fresh(guard, now);
    if (!result)
        result = Stationary(guard);
    if (result)
        return result;
    result = Stable(guard, now);
    if (result)
        return result;
    for (axis = 0; axis < 2; axis++)
    {
        guard->zero_actual[axis] = guard->sample[axis * 4 + 1].value;
        guard->zero_target[axis] = guard->sample[axis * 4 + 3].value;
    }
    guard->zero_version++;
    guard->zero_valid = 1;
    return MECH_OK;
}

int64_t Mech_Position(const MechGuard *guard, uint8_t axis)
{
    if (!guard || axis > 1 || !guard->zero_valid)
        return INT64_MIN;
    return (guard->sample[axis * 4 + 1].value - guard->zero_actual[axis]) *
           signs[axis] * distance_um[axis] / MECH_RAW_PER_320;
}

uint8_t Mech_CheckAxis(const MechGuard *guard, uint32_t now, uint32_t version,
                       uint8_t axis, int32_t target_um, uint8_t arm_known, int32_t arm_mdeg)
{
    uint8_t result;
    if (!guard || axis > 1 || target_um < 0 || target_um > maximum_um[axis])
        return MECH_RANGE;
    result = Ready(guard, now, version);
    if (result)
        return result;
    if (!axis && Mech_Position(guard, 1) > MECH_CLEARANCE_UM)
        return MECH_COLLISION;
    if (axis && target_um > MECH_CLEARANCE_UM && (Mech_Position(guard, 0) < MECH_EXTEND_MIN_LIFT_UM ||
        !arm_known || arm_mdeg != MECH_ARM_CLEAR_MDEG))
        return MECH_COLLISION;
    return MECH_OK;
}

uint8_t Mech_CheckRotate(const MechGuard *guard, uint32_t now, uint32_t version,
                         uint8_t tray, uint8_t arm_known, int32_t arm_mdeg)
{
    uint8_t result;
    if (!guard)
        return MECH_NO_ZERO;
    result = Ready(guard, now, version);
    if (result)
        return result;
    if (Mech_Position(guard, 0) > MECH_CLEARANCE_UM || Mech_Position(guard, 1) > MECH_CLEARANCE_UM ||
        (tray && (!arm_known || arm_mdeg != MECH_ARM_CLEAR_MDEG)))
        return MECH_COLLISION;
    return MECH_OK;
}

uint32_t Mech_Pulses(uint8_t axis, int32_t target_um)
{
    if (axis > 1 || target_um < 0 || target_um > maximum_um[axis])
        return UINT32_MAX;
    return (uint32_t)((int64_t)target_um * 320 / distance_um[axis]);
}

uint16_t Mech_Pulse(uint8_t channel, int32_t target_mdeg)
{
    int32_t range = channel == MECH_ARM_CHANNEL ? MECH_ARM_RANGE_MDEG : MECH_TRAY_RANGE_MDEG;
    if (channel < 1 || channel > 3 || target_mdeg < 0 || target_mdeg > range ||
        (channel == MECH_GRIP_CHANNEL && target_mdeg > MECH_GRIP_MAX_MDEG))
        return 0;
    if (channel == MECH_ARM_CHANNEL)
        target_mdeg = range - target_mdeg;
    return (uint16_t)(MECH_PWM_MIN_US + ((int64_t)target_mdeg * MECH_PWM_SPAN_US + range / 2) / range);
}
