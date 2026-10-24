#include "mech_pose.h"
#include "mech_hw.h"
#include <string.h>

static int Busy(const Pose *pose)
{
    return pose->state >= POSE_ACCEPTED && pose->state <= POSE_STOPPING;
}

static int Same(const PoseGoal *a, const PoseGoal *b)
{
    unsigned index;
    if (a->id != b->id || a->zero_version != b->zero_version || a->revision != b->revision ||
        a->timeout_ms != b->timeout_ms || a->release_confirmed != b->release_confirmed || a->single_step != b->single_step)
        return 0;
    for (index = 0; index < 5; index++)
        if (a->target[index] != b->target[index])
            return 0;
    return 1;
}

void Pose_Init(Pose *pose, Axis *axis, Servo *servo)
{
    if (!pose)
        return;
    memset(pose, 0, sizeof(*pose));
    pose->axis = axis;
    pose->servo = servo;
    if (axis && axis->guard)
    {
        pose->boot = axis->guard->boot;
        pose->session = axis->guard->session;
    }
}

static uint32_t Add(PoseStep steps[8], uint8_t *count, int32_t state[5], int32_t pulses[2],
                     uint8_t actuator, int32_t target)
{
    int32_t desired;
    uint8_t same;
    if (actuator < 2)
    {
        desired = (int32_t)Mech_Pulses(actuator, target);
        same = (uint8_t)(desired == pulses[actuator]);
        pulses[actuator] = desired;
        if (!same && !actuator && state[1] > MECH_CLEARANCE_UM)
            return MECH_COLLISION;
        if (!same && actuator && target > MECH_CLEARANCE_UM &&
            (state[0] < MECH_EXTEND_MIN_LIFT_UM || state[2] != MECH_ARM_CLEAR_MDEG))
            return MECH_COLLISION;
    }
    else
    {
        same = (uint8_t)(state[actuator] == target);
        if (!same && actuator < 4 && (state[0] > MECH_CLEARANCE_UM ||
            state[1] > MECH_CLEARANCE_UM || (actuator == 3 && state[2] != MECH_ARM_CLEAR_MDEG)))
            return MECH_COLLISION;
    }
    if (!same)
    {
        if (*count >= 8)
            return POSE_SEQUENCE;
        steps[*count].kind = actuator < 2 ? POSE_AXIS : POSE_SERVO;
        steps[*count].actuator = actuator < 2 ? actuator : actuator == 2 ? MECH_ARM_CHANNEL :
                                                        actuator == 3 ? MECH_TRAY_CHANNEL : MECH_GRIP_CHANNEL;
        steps[*count].target = target;
        (*count)++;
    }
    state[actuator] = target;
    return MECH_OK;
}

uint32_t Pose_Preview(const Pose *pose, const PoseGoal *goal, PoseStep steps[8], uint8_t *count,
                      uint32_t now)
{
    static const uint8_t order[8] = {1, 0, 2, 3, 2, 0, 1, 4};
    MechGuard *guard;
    int32_t state[5], pulses[2], target;
    unsigned index;
    uint32_t result;
    if (!pose || !goal || !steps || !count || !pose->axis || !pose->servo ||
        !(guard = pose->axis->guard) || guard != pose->servo->guard ||
        guard->boot != pose->boot || guard->session != pose->session)
        return MECH_SESSION;
    if (!goal->id || !goal->revision || goal->timeout_ms < 1000 || goal->timeout_ms > 120000)
        return MECH_RANGE;
    if (Mech_Pulses(0, goal->target[0]) == UINT32_MAX || Mech_Pulses(1, goal->target[1]) == UINT32_MAX ||
        !Mech_Pulse(MECH_TRAY_CHANNEL, goal->target[3]) || !Mech_Pulse(MECH_GRIP_CHANNEL, goal->target[4]) ||
        goal->target[2] < MECH_ARM_MIN_MDEG || goal->target[2] > MECH_ARM_MAX_MDEG)
        return MECH_RANGE;
    if (goal->target[1] > MECH_CLEARANCE_UM &&
        (goal->target[0] < MECH_EXTEND_MIN_LIFT_UM || goal->target[2] != MECH_ARM_CLEAR_MDEG))
        return MECH_COLLISION;
    result = Mech_Ready(guard, now, goal->zero_version);
    if (result)
        return result;
    for (index = 1; index <= 3; index++)
        if (!Servo_Reference(pose->servo, (uint8_t)index))
            return SERVO_REFERENCE;
    /* 实车先逐舵机验收首次PWM；姿态执行不隐式激活未输出的通道。 */
    if (!pose->servo->simulation && pose->servo->output_mask != 7)
        return SERVO_REFERENCE;
    if (pose->servo->hold_possible && goal->target[4] != MECH_GRIP_MAX_MDEG && !goal->release_confirmed)
        return SERVO_HOLD;
    state[0] = (int32_t)Mech_Position(guard, 0);
    state[1] = (int32_t)Mech_Position(guard, 1);
    state[2] = pose->servo->command_mdeg[MECH_ARM_CHANNEL - 1];
    state[3] = pose->servo->command_mdeg[MECH_TRAY_CHANNEL - 1];
    state[4] = pose->servo->command_mdeg[MECH_GRIP_CHANNEL - 1];
    pulses[0] = pose->axis->zero_version == goal->zero_version ? pose->axis->pulse_position[0] : 0;
    pulses[1] = pose->axis->zero_version == goal->zero_version ? pose->axis->pulse_position[1] : 0;
    *count = 0;
    for (index = 0; index < 8; index++)
    {
        target = index < 2 ? 0 : index == 2 ? MECH_ARM_CLEAR_MDEG : goal->target[order[index]];
        result = Add(steps, count, state, pulses, order[index], target);
        if (result)
            return result;
    }
    return MECH_OK;
}

uint32_t Pose_Begin(Pose *pose, const PoseGoal *goal, uint32_t now)
{
    PoseStep steps[8] = {{0}};
    uint8_t count;
    uint32_t result;
    if (!pose || !goal)
        return MECH_SESSION;
    if (!goal->id || goal->id < pose->goal.id)
        return POSE_ID;
    if (goal->id == pose->goal.id)
        return Same(goal, &pose->goal) ? MECH_OK : POSE_ID;
    if (Busy(pose) || !pose->axis || !pose->servo ||
        (pose->axis->state >= AXIS_ACCEPTED && pose->axis->state <= AXIS_STOPPING) ||
        (pose->servo->state >= SERVO_ACCEPTED && pose->servo->state <= SERVO_COMMAND_DONE))
        return POSE_BUSY;
    result = Pose_Preview(pose, goal, steps, &count, now);
    if (result)
        return result;
    pose->goal = *goal;
    memcpy(pose->steps, steps, sizeof(steps));
    pose->count = count;
    pose->index = pose->child_started = pose->stop_confirmed = 0;
    pose->step_authorized = 0;
    pose->reason = 0;
    pose->started_ms = now;
    pose->state = count ? POSE_ACCEPTED : POSE_COMPLETE;
    return MECH_OK;
}

static void Stop(Pose *pose, uint32_t reason, uint32_t now)
{
    if (pose->state == POSE_STOPPING)
        return;
    pose->reason = reason;
    pose->state = POSE_STOPPING;
    Servo_Stop(pose->servo, reason == POSE_CANCEL ? SERVO_CANCEL : reason, now);
    Axis_Stop(pose->axis, reason == POSE_CANCEL ? AXIS_CANCEL : reason, now);
}

void Pose_Cancel(Pose *pose, uint32_t now)
{
    if (pose && pose->axis && pose->servo && Busy(pose))
        Stop(pose, POSE_CANCEL, now);
}

uint32_t Pose_Next(Pose *pose, uint32_t id, uint32_t revision, uint8_t index, uint32_t now)
{
    if (!pose || pose->state != POSE_STEP_READY || id != pose->goal.id ||
        revision != pose->goal.revision || index != pose->index)
        return POSE_SEQUENCE;
    if (Mech_Ready(pose->axis->guard, now, pose->goal.zero_version))
        return MECH_UNSTABLE;
    pose->step_authorized = 1;
    pose->state = POSE_RUNNING;
    return MECH_OK;
}

void Pose_Poll(Pose *pose, uint32_t now, uint8_t base_stopped)
{
    PoseStep *step;
    MechGuard *guard;
    uint32_t result, remaining;
    if (!pose || !Busy(pose) || !pose->axis || !pose->servo || !(guard = pose->axis->guard))
        return;
    if (pose->state == POSE_STOPPING)
    {
        Axis_Poll(pose->axis, now, base_stopped);
        if (pose->axis->stop_confirmed)
        {
            pose->stop_confirmed = 1;
            pose->state = pose->reason == POSE_CANCEL ? POSE_CANCELLED : POSE_FAILED;
        }
        return;
    }
    Mech_Poll(guard, now, base_stopped);
    if (guard->fault || !base_stopped || !guard->zero_valid ||
        guard->zero_version != pose->goal.zero_version || guard->boot != pose->boot || guard->session != pose->session)
    {
        Stop(pose, guard->fault ? guard->fault : !base_stopped ? MECH_BASE : MECH_VERSION, now);
        return;
    }
    if ((uint32_t)(now - pose->started_ms) >= pose->goal.timeout_ms)
    {
        Stop(pose, POSE_TIMEOUT, now);
        return;
    }
    if (pose->index >= pose->count)
    {
        pose->state = POSE_COMPLETE;
        return;
    }
    step = pose->steps + pose->index;
    if (pose->child_started)
    {
        if (step->kind == POSE_AXIS)
        {
            if (pose->axis->goal.id != pose->child_id || pose->axis->goal.target_um != step->target ||
                pose->axis->goal.axis != step->actuator)
            {
                Stop(pose, POSE_SEQUENCE, now);
                return;
            }
            Axis_Poll(pose->axis, now, base_stopped);
            if (pose->axis->state == AXIS_COMPLETE)
            {
                pose->index++;
                pose->child_started = 0;
                pose->step_authorized = 0;
            }
            else if (pose->axis->state >= AXIS_STOPPING)
                Stop(pose, pose->axis->reason ? pose->axis->reason : POSE_SEQUENCE, now);
        }
        else
        {
            if (pose->servo->goal.id != pose->child_id || pose->servo->goal.target_mdeg != step->target ||
                pose->servo->goal.channel != step->actuator)
            {
                Stop(pose, POSE_SEQUENCE, now);
                return;
            }
            Servo_Poll(pose->servo, now, base_stopped);
            if (pose->servo->state == SERVO_CONFIRMED)
            {
                pose->index++;
                pose->child_started = 0;
                pose->step_authorized = 0;
                pose->state = POSE_RUNNING;
            }
            else if (pose->servo->state == SERVO_COMMAND_DONE)
                pose->state = POSE_WAIT_CONFIRM;
            else if (pose->servo->state == SERVO_FAILED || pose->servo->state == SERVO_CANCELLED)
                Stop(pose, pose->servo->reason, now);
        }
        return;
    }
    if (pose->goal.single_step && !pose->step_authorized)
    {
        pose->state = POSE_STEP_READY;
        return;
    }
    remaining = pose->goal.timeout_ms - (uint32_t)(now - pose->started_ms);
    if (step->kind == POSE_AXIS)
    {
        AxisGoal goal = {0};
        if (pose->axis->goal.id == UINT32_MAX || remaining < 500)
        {
            Stop(pose, POSE_TIMEOUT, now);
            return;
        }
        goal.id = pose->axis->goal.id + 1;
        goal.zero_version = pose->goal.zero_version;
        goal.axis = step->actuator;
        goal.target_um = step->target;
        goal.rpm = MECH_AXIS_MAX_RPM;
        goal.timeout_ms = remaining > 15000 ? 15000 : remaining;
        result = Axis_Begin(pose->axis, &goal, now, Servo_Reference(pose->servo, MECH_ARM_CHANNEL),
                             pose->servo->command_mdeg[MECH_ARM_CHANNEL - 1]);
        pose->child_id = goal.id;
    }
    else
    {
        ServoGoal goal = {0};
        if (pose->servo->goal.id == UINT32_MAX || remaining < 1000)
        {
            Stop(pose, POSE_TIMEOUT, now);
            return;
        }
        goal.id = pose->servo->goal.id + 1;
        goal.zero_version = pose->goal.zero_version;
        goal.channel = step->actuator;
        goal.target_mdeg = step->target;
        goal.release_confirmed = pose->goal.release_confirmed;
        goal.timeout_ms = remaining > 30000 ? 30000 : remaining;
        result = Servo_Begin(pose->servo, &goal, now);
        pose->child_id = goal.id;
    }
    if (result == MECH_UNSTABLE)
        return;
    if (result)
        Stop(pose, result, now);
    else
    {
        pose->child_started = 1;
        pose->state = POSE_RUNNING;
    }
}
