#include "mech_servo.h"
#include "mech_hw.h"
#include <string.h>

static uint8_t Bit(uint8_t channel)
{
    return channel >= 1 && channel <= 3 ? (uint8_t)(1U << (channel - 1U)) : 0;
}

static int Busy(const Servo *servo)
{
    return servo->state >= SERVO_ACCEPTED && servo->state <= SERVO_COMMAND_DONE;
}

static int Bounds(uint8_t channel, int32_t target)
{
    return Mech_Pulse(channel, target) &&
           (channel != MECH_ARM_CHANNEL ||
            (target >= MECH_ARM_MIN_MDEG && target <= MECH_ARM_MAX_MDEG));
}

uint8_t Servo_Reference(const Servo *servo, uint8_t channel)
{
    uint8_t bit = Bit(channel);
    return servo && bit && servo->guard && servo->guard->zero_valid &&
           servo->guard->zero_version == servo->zero_version &&
           servo->guard->boot == servo->boot && servo->guard->session == servo->session &&
           (servo->reference_mask & bit) && (servo->confirmed_mask & bit);
}

static uint32_t Check(const Servo *servo, uint8_t channel, uint32_t now)
{
    uint8_t arm = Servo_Reference(servo, MECH_ARM_CHANNEL);
    int32_t angle = servo->command_mdeg[MECH_ARM_CHANNEL - 1];
    if (!servo->guard || servo->guard->boot != servo->boot || servo->guard->session != servo->session)
        return MECH_SESSION;
    if (channel == MECH_GRIP_CHANNEL)
        return Mech_Ready(servo->guard, now, servo->zero_version);
    return Mech_CheckRotate(servo->guard, now, servo->zero_version,
                            (uint8_t)(channel == MECH_TRAY_CHANNEL), arm, angle);
}

void Servo_Init(Servo *servo, MechGuard *guard, uint8_t simulation)
{
    if (!servo)
        return;
    memset(servo, 0, sizeof(*servo));
    servo->guard = guard;
    servo->simulation = (uint8_t)!!simulation;
    /* 实物是否持料未知，默认保夹；模型域可由自己的合成物料状态管理。 */
    servo->hold_possible = (uint8_t)!simulation;
    if (guard)
    {
        servo->boot = guard->boot;
        servo->session = guard->session;
    }
}

uint32_t Servo_Seed(Servo *servo, const int32_t initial[3], uint32_t now, uint8_t confirmed)
{
    uint32_t result;
    unsigned index;
    if (!servo || !servo->guard || !initial || !confirmed)
        return MECH_MANUAL;
    if (servo->reference_mask || Busy(servo))
        return SERVO_REFERENCE;
    if (servo->guard->boot != servo->boot || servo->guard->session != servo->session)
        return MECH_SESSION;
    result = Mech_Ready(servo->guard, now, servo->guard->zero_version);
    if (result)
        return result;
    if (Mech_Position(servo->guard, 0) > MECH_CLEARANCE_UM ||
        Mech_Position(servo->guard, 1) > MECH_CLEARANCE_UM)
        return MECH_COLLISION;
    for (index = 0; index < 3; index++)
        if (!Bounds((uint8_t)(index + 1), initial[index]))
            return MECH_RANGE;
    memcpy(servo->command_mdeg, initial, sizeof(servo->command_mdeg));
    servo->zero_version = servo->guard->zero_version;
    servo->reference_mask = servo->confirmed_mask = 7;
    servo->confirmation_kind = servo->simulation ? SERVO_MODEL : SERVO_MANUAL;
    if (initial[MECH_GRIP_CHANNEL - 1] == MECH_GRIP_MAX_MDEG)
        servo->hold_possible = 1;
    /* 参考确认并不表示PWM已输出，output_mask仍为0。 */
    return MECH_OK;
}

static int Same(const ServoGoal *a, const ServoGoal *b)
{
    return a->id == b->id && a->zero_version == b->zero_version &&
           a->timeout_ms == b->timeout_ms && a->target_mdeg == b->target_mdeg &&
           a->channel == b->channel && a->release_confirmed == b->release_confirmed;
}

uint32_t Servo_Begin(Servo *servo, const ServoGoal *goal, uint32_t now)
{
    uint32_t result;
    if (!servo || !goal || !servo->guard)
        return MECH_SESSION;
    if (!goal->id || goal->id < servo->goal.id)
        return SERVO_ID;
    if (goal->id == servo->goal.id)
        return Same(goal, &servo->goal) ? MECH_OK : SERVO_ID;
    if (Busy(servo))
        return SERVO_BUSY;
    if (servo->state == SERVO_FAILED || servo->state == SERVO_CANCELLED)
        return MECH_FAULT;
    if (!Bounds(goal->channel, goal->target_mdeg) || goal->timeout_ms < 1000 || goal->timeout_ms > 30000)
        return MECH_RANGE;
    if (goal->zero_version != servo->zero_version)
        return MECH_VERSION;
    if (!(servo->reference_mask & Bit(goal->channel)))
        return SERVO_REFERENCE;
    result = Check(servo, goal->channel, now);
    if (result)
        return result;
    if (goal->channel == MECH_GRIP_CHANNEL && servo->hold_possible &&
        goal->target_mdeg != MECH_GRIP_MAX_MDEG && !goal->release_confirmed)
        return SERVO_HOLD;
    servo->goal = *goal;
    servo->started_ms = servo->last_step_ms = now;
    servo->state = SERVO_ACCEPTED;
    servo->reason = 0;
    servo->taken = 0;
    if (goal->channel == MECH_GRIP_CHANNEL && goal->target_mdeg == MECH_GRIP_MAX_MDEG)
        servo->hold_possible = 1;
    /* 首个微步前可使用刚才核对的参考；从第一次输出起撤销该通道确认。 */
    return MECH_OK;
}

void Servo_Stop(Servo *servo, uint32_t reason, uint32_t now)
{
    uint8_t bit;
    if (!servo || !servo->guard || !Busy(servo))
        return;
    bit = Bit(servo->goal.channel);
    if (servo->state == SERVO_OUTPUT && servo->taken)
        servo->output_mask &= (uint8_t)~bit; /* 可能已部分写入，保留未知。 */
    servo->confirmed_mask &= (uint8_t)~bit;
    servo->reason = reason ? reason : SERVO_CANCEL;
    servo->state = servo->reason == SERVO_CANCEL ? SERVO_CANCELLED : SERVO_FAILED;
    servo->taken = 1;
    servo->issued_ms = now;
    Mech_Invalidate(servo->guard, servo->reason);
    servo->guard->stop_required = 1;
    /* 不输出任何新脉宽、不关闭PWM、不张爪。 */
}

void Servo_Poll(Servo *servo, uint32_t now, uint8_t base_stopped)
{
    uint32_t result;
    int32_t current, target, next;
    uint8_t bit;
    uint8_t wait_stable = 0;
    if (!servo || !servo->guard || !Busy(servo))
        return;
    Mech_Poll(servo->guard, now, base_stopped);
    result = Check(servo, servo->goal.channel, now);
    /* 部分新反馈尚未凑成稳定轮次时先暂停输出；真实轴运动不能等。 */
    if (result == MECH_UNSTABLE && !servo->guard->sample[0].value && !servo->guard->sample[4].value)
    {
        wait_stable = 1;
        result = MECH_OK;
    }
    if (!base_stopped || result || !servo->guard->zero_valid ||
        (servo->goal.channel != MECH_GRIP_CHANNEL &&
         (Mech_Position(servo->guard, 0) > MECH_CLEARANCE_UM ||
          Mech_Position(servo->guard, 1) > MECH_CLEARANCE_UM)))
    {
        Servo_Stop(servo, result ? result : !base_stopped ? MECH_BASE : MECH_COLLISION, now);
        return;
    }
    if ((uint32_t)(now - servo->started_ms) >= servo->goal.timeout_ms)
    {
        Servo_Stop(servo, SERVO_TIMEOUT, now);
        return;
    }
    if (servo->state == SERVO_OUTPUT)
    {
        if ((uint32_t)(now - servo->last_step_ms) >= MECH_SERVO_OUTPUT_MS)
            Servo_Stop(servo, SERVO_OUTPUT_ERROR, now);
        return;
    }
    if (servo->state == SERVO_SETTLING)
    {
        if ((uint32_t)(now - servo->issued_ms) >= MECH_SERVO_WAIT_MS)
            servo->state = SERVO_COMMAND_DONE;
        return;
    }
    if (servo->state == SERVO_COMMAND_DONE)
        return;
    if (wait_stable)
        return;
    if (servo->state != SERVO_ACCEPTED && (uint32_t)(now - servo->last_step_ms) < MECH_SERVO_STEP_MS)
        return;
    current = servo->command_mdeg[servo->goal.channel - 1];
    target = servo->goal.target_mdeg;
    bit = Bit(servo->goal.channel);
    if (!(servo->output_mask & bit))
        next = current; /* 首次显式输出必须从已核对的初始参考起步。 */
    else if (current < target)
        next = target - current > MECH_SERVO_STEP_MDEG ? current + MECH_SERVO_STEP_MDEG : target;
    else
        next = current - target > MECH_SERVO_STEP_MDEG ? current - MECH_SERVO_STEP_MDEG : target;
    servo->pending.id = servo->goal.id;
    servo->pending.zero_version = servo->goal.zero_version;
    servo->pending.sequence = ++servo->output_sequence;
    servo->pending.channel = servo->goal.channel;
    servo->pending.command_mdeg = next;
    servo->pending.pulse_us = Mech_Pulse(servo->goal.channel, next);
    servo->last_step_ms = now;
    servo->state = SERVO_OUTPUT;
    servo->taken = 0;
}

uint8_t Servo_Take(Servo *servo, ServoOutput *output, uint32_t now)
{
    if (!servo || !output)
        return 0;
    Servo_Poll(servo, now, servo->guard ? servo->guard->base_stopped : 0);
    if (servo->state != SERVO_OUTPUT || servo->taken)
        return 0;
    *output = servo->pending;
    servo->taken = 1;
    return 1;
}

void Servo_Applied(Servo *servo, uint32_t id, uint32_t sequence, uint8_t ok, uint32_t now)
{
    uint8_t bit;
    if (servo && servo->guard)
        Servo_Poll(servo, now, servo->guard->base_stopped);
    if (!servo || servo->state != SERVO_OUTPUT || !servo->taken || id != servo->goal.id ||
        sequence != servo->pending.sequence)
        return;
    if (!ok || (uint32_t)(now - servo->last_step_ms) >= MECH_SERVO_OUTPUT_MS)
    {
        Servo_Stop(servo, SERVO_OUTPUT_ERROR, now);
        return;
    }
    bit = Bit(servo->goal.channel);
    servo->command_mdeg[servo->goal.channel - 1] = servo->pending.command_mdeg;
    servo->output_mask |= bit;
    servo->confirmed_mask &= (uint8_t)~bit;
    servo->issued_ms = servo->last_step_ms = now;
    servo->state = servo->pending.command_mdeg == servo->goal.target_mdeg ? SERVO_SETTLING : SERVO_SLEWING;
}

uint32_t Servo_Confirm(Servo *servo, const ServoEvidence *evidence, uint32_t now)
{
    if (servo && servo->guard)
        Servo_Poll(servo, now, servo->guard->base_stopped);
    if (!servo || !evidence || servo->state != SERVO_COMMAND_DONE ||
        evidence->boot != servo->boot || evidence->session != servo->session ||
        evidence->id != servo->goal.id || evidence->zero_version != servo->zero_version ||
        evidence->channel != servo->goal.channel || evidence->target_mdeg != servo->goal.target_mdeg ||
        !evidence->sequence || (servo->evidence_sequence &&
        (int32_t)(evidence->sequence - servo->evidence_sequence) <= 0) ||
        (uint32_t)(now - evidence->captured_ms) > MECH_FEEDBACK_AGE_MS ||
        (uint32_t)(evidence->captured_ms - servo->issued_ms) < MECH_SERVO_WAIT_MS ||
        (uint32_t)(evidence->captured_ms - servo->issued_ms) >= 0x80000000UL ||
        (servo->simulation ? evidence->kind != SERVO_MODEL :
         (evidence->kind != SERVO_MANUAL && evidence->kind != SERVO_VISION)))
        return SERVO_EVIDENCE;
    if (Check(servo, servo->goal.channel, now))
        return SERVO_EVIDENCE;
    servo->evidence_sequence = evidence->sequence;
    servo->confirmation_kind = evidence->kind;
    servo->confirmed_mask |= Bit(servo->goal.channel);
    servo->state = SERVO_CONFIRMED;
    if (servo->goal.channel == MECH_GRIP_CHANNEL && servo->goal.release_confirmed &&
        servo->goal.target_mdeg != MECH_GRIP_MAX_MDEG)
        servo->hold_possible = 0;
    return MECH_OK;
}
