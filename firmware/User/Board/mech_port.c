#include "mech_port.h"
#include "mech_pose.h"
#include "mech_hw.h"
#include "bus_internal.h"
#include "comm.h"
#include "tim.h"
#include <string.h>

static MechGuard guard;
static Axis axis;
static Servo servo;
static Pose pose;
static MotorConfigRx config_rx[2];
static MotorConfig config[2];
static uint8_t attached, pending_kind, pending_axis, enabled_mask, query_index;
static uint8_t pwm_mask, stopping_mask;
static uint8_t axis_done;
static uint8_t enable_pending;
static uint32_t last_token, last_result, query_tick;
static uint8_t last_type, last_data[96];
static uint16_t last_size;
static uint32_t pwm_channels[3] = {TIM_CHANNEL_1, TIM_CHANNEL_2, TIM_CHANNEL_3};

static uint8_t AxisBusy(void)
{
    return axis.state >= AXIS_ACCEPTED && axis.state <= AXIS_STOPPING;
}

static uint8_t ServoBusy(void)
{
    return servo.state >= SERVO_ACCEPTED && servo.state <= SERVO_COMMAND_DONE;
}

static uint8_t PoseBusy(void)
{
    return pose.state >= POSE_ACCEPTED && pose.state <= POSE_STOPPING;
}

uint8_t MechPort_Owned(void)
{
    return attached;
}

void MechPort_Init(void)
{
    attached = pending_kind = enabled_mask = query_index = pwm_mask = stopping_mask = axis_done = enable_pending = 0;
    last_token = last_result = query_tick = 0;
    memset(config, 0, sizeof(config));
    Mech_Init(&guard, 0, 0);
    Axis_Init(&axis, &guard);
    Servo_Init(&servo, &guard, 0);
    Pose_Init(&pose, &axis, &servo);
}

void MechPort_Feed(const BusFrame *frame)
{
    MotorReply reply;
    unsigned index;
    uint32_t now = HAL_GetTick();
    if (!attached || (frame->id >> 8) < 5 || (frame->id >> 8) > 6)
        return;
    index = (unsigned)((frame->id >> 8) - 5);
    if (frame->len && frame->data[0] == 0x42)
    {
        if (!(frame->id & 255U))
            config[index].valid = 0;
        if (Drive_Config(&config_rx[index], frame, (uint8_t)(index + 5), &config[index]) == 2)
            config[index].valid = 0;
    }
    else if (Drive_Parse(frame, (uint8_t)(index + 5), &reply))
    {
        reply.handled_ms = now;
        Axis_Ack(&axis, &reply, now);
        if (reply.code == 0x33 || reply.code == 0x35 || reply.code == 0x36 || reply.code == 0x3A)
            (void)Mech_Feed(&guard, &reply, now);
        else if (reply.result != 2 && reply.result != 0x9F)
            MechPort_Stop(MECH_DRIVER, now);
    }
}

void MechPort_Stop(uint32_t reason, uint32_t now)
{
    if (!attached)
        return;
    enable_pending = 0;
    Mech_Invalidate(&guard, reason ? reason : MECH_MANUAL);
    Pose_Cancel(&pose, now);
    Servo_Stop(&servo, reason ? reason : SERVO_CANCEL, now);
    Axis_Stop(&axis, reason ? reason : AXIS_CANCEL, now);
    guard.stop_required = 1;
}

void MechPort_Run(uint32_t now, uint8_t base_stopped, uint32_t fault)
{
    unsigned index;
    uint8_t verified = 0;
    ServoOutput output;
    if (!attached)
        return;
    if (!guard.fault && (guard.session != Comm_Session() || guard.boot != Comm_Boot()))
        MechPort_Stop(MECH_SESSION, now);
    if (fault && !guard.fault)
        MechPort_Stop(MECH_FAULT, now);
    for (index = 0; index < 2; index++)
    {
        /* 旧驱动换算依赖200步电机、16细分、固定校验；不修改驱动配置。 */
        if (config[index].valid && config[index].motor_type == 0x19 &&
            config[index].microsteps == 16 && config[index].check_type == 0)
            verified |= (uint8_t)(1U << index);
    }
    Mech_Config(&guard, verified);
    Mech_Poll(&guard, now, base_stopped);
    if (axis.state == AXIS_COMPLETE && Mech_Ready(&guard, now, guard.zero_version) == MECH_OK)
        axis_done = 1;
    Pose_Poll(&pose, now, base_stopped);
    Axis_Poll(&axis, now, base_stopped);
    Servo_Poll(&servo, now, base_stopped);
    if (!fault && !guard.fault && Servo_Take(&servo, &output, now))
    {
        uint32_t channel = pwm_channels[output.channel - 1];
        uint8_t bit = (uint8_t)(1U << (output.channel - 1));
        uint8_t ok = 1;
        __HAL_TIM_SET_COMPARE(&htim1, channel, output.pulse_us);
        if (!(pwm_mask & bit))
        {
            ok = (uint8_t)(HAL_TIM_PWM_Start(&htim1, channel) == HAL_OK);
            if (ok)
                pwm_mask |= bit;
        }
        Servo_Applied(&servo, output.id, output.sequence, ok, HAL_GetTick());
    }
    if (guard.stop_required && !stopping_mask)
    {
        Bus_Stop();
        stopping_mask = 3;
    }
    if (stopping_mask && !bus_diag.pending_stop_count &&
        bus_diag.stop_sequence == bus_diag.stop_sent_sequence)
    {
        Axis_StopSent(&axis, stopping_mask, now);
        stopping_mask = 0;
        guard.stop_required = 0;
    }
}

uint8_t MechPort_Query(BusFrame frames[2], uint32_t now)
{
    static const uint8_t kinds[4] = {0x35, 0x36, 0x3A, 0x33};
    uint8_t address, code;
    if (!attached || (uint32_t)(now - query_tick) < 5)
        return 0;
    if (!config[0].valid || !config[1].valid)
    {
        address = (uint8_t)(!config[0].valid ? 5 : 6);
        code = 0x42;
    }
    else
    {
        address = (uint8_t)(5 + query_index / 4);
        code = kinds[query_index % 4];
        query_index = (uint8_t)((query_index + 1) % 8);
    }
    query_tick = now;
    return Drive_Build(address, code, 0, 0, 0, 0, 0, 0, frames);
}

uint8_t MechPort_Output(BusFrame frames[2], uint32_t now)
{
    uint8_t count;
    if (!attached || pending_kind || guard.fault)
        return 0;
    if (enable_pending)
    {
        count = Drive_Build((uint8_t)(5 + pending_axis), 0xF3, 0, 0, 0, 0, 0, 1, frames);
        if (count)
        {
            pending_kind = 2;
            enable_pending = 0;
        }
        return count;
    }
    if (bus_diag.motion_uncertain)
    {
        if (!axis_done || !Bus_MechDone())
            return 0;
    }
    count = Axis_Take(&axis, frames, now);
    if (count)
    {
        axis_done = 0;
        pending_kind = 1;
    }
    return count;
}

void MechPort_Sent(uint8_t success, uint32_t now)
{
    if (pending_kind == 1)
        Axis_Sent(&axis, axis.goal.id, success, now);
    if (pending_kind == 2 && success)
        enabled_mask |= (uint8_t)(1U << pending_axis);
    if (pending_kind && !success)
        MechPort_Stop(AXIS_TX, now);
    pending_kind = 0;
}

static uint32_t Mutate(uint8_t type, const uint8_t *data, uint16_t size, uint32_t now)
{
    unsigned index;
    uint32_t version = (uint32_t)Frame_Get(data + 4, 4);
    if (type == 48 && size == 8 && version == 0x4D454348UL)
    {
        if (attached)
            return guard.session == Comm_Session() ? MECH_OK : MECH_SESSION;
        attached = 1;
        Mech_Init(&guard, Comm_Boot(), Comm_Session());
        Axis_Init(&axis, &guard);
        Servo_Init(&servo, &guard, 0);
        Pose_Init(&pose, &axis, &servo);
        Bus_MechWatch();
        return MECH_OK;
    }
    if (!attached || guard.session != Comm_Session())
        return MECH_SESSION;
    if (type == 57 && size == 8)
    {
        MechPort_Stop(MECH_MANUAL, now);
        return MECH_OK;
    }
    if (guard.fault || (bus_diag.error & ~BUS_ERROR_STOP) || can_diag.errors)
        return MECH_FAULT;
    if (version != guard.zero_version && type != 50)
        return MECH_VERSION;
    if (type == 56 && size == 17)
        return Pose_Next(&pose, (uint32_t)Frame_Get(data + 8, 4),
                         (uint32_t)Frame_Get(data + 12, 4), data[16], now);
    if (type == 54 && size == 29)
    {
        ServoEvidence evidence;
        memset(&evidence, 0, sizeof(evidence));
        evidence.boot = guard.boot;
        evidence.session = guard.session;
        evidence.zero_version = version;
        evidence.id = (uint32_t)Frame_Get(data + 8, 4);
        evidence.sequence = (uint32_t)Frame_Get(data + 12, 4);
        evidence.captured_ms = (uint32_t)Frame_Get(data + 16, 4);
        evidence.target_mdeg = (int32_t)Frame_Get(data + 20, 4);
        evidence.channel = data[24];
        evidence.kind = SERVO_MANUAL;
        if (Frame_Get(data + 25, 4) != 1)
            return MECH_MANUAL;
        return Servo_Confirm(&servo, &evidence, now);
    }
    if (AxisBusy() || ServoBusy() || PoseBusy() || enable_pending || pending_kind)
        return MECH_UNSTABLE;
    if (type == 49 && size == 9 && data[8] < 2)
    {
        if (!guard.base_stopped || guard.config_mask != 3 || enabled_mask & (1U << data[8]))
            return MECH_CONFIG;
        enable_pending = 1;
        pending_axis = data[8];
        return MECH_OK;
    }
    if (type == 50 && size == 9)
        return Mech_Zero(&guard, now, data[8] == 1);
    if (type == 51 && size == 21)
    {
        int32_t initial[3];
        for (index = 0; index < 3; index++)
            initial[index] = (int32_t)Frame_Get(data + 8 + index * 4, 4);
        return Servo_Seed(&servo, initial, now, data[20] == 1);
    }
    if (type == 52 && size == 23)
    {
        AxisGoal goal;
        memset(&goal, 0, sizeof(goal));
        goal.id = (uint32_t)Frame_Get(data + 8, 4);
        goal.zero_version = version;
        goal.target_um = (int32_t)Frame_Get(data + 12, 4);
        goal.timeout_ms = (uint32_t)Frame_Get(data + 16, 4);
        goal.rpm = (uint16_t)Frame_Get(data + 20, 2);
        goal.axis = data[22];
        if (goal.id <= pose.child_id || goal.id <= servo.goal.id)
            return AXIS_ID;
        if (bus_diag.motion_uncertain && !Bus_MechDone())
            return AXIS_BUSY;
        return Axis_Begin(&axis, &goal, now, Servo_Reference(&servo, 2), servo.command_mdeg[1]);
    }
    if (type == 53 && size == 22)
    {
        ServoGoal goal;
        memset(&goal, 0, sizeof(goal));
        goal.id = (uint32_t)Frame_Get(data + 8, 4);
        goal.zero_version = version;
        goal.target_mdeg = (int32_t)Frame_Get(data + 12, 4);
        goal.timeout_ms = (uint32_t)Frame_Get(data + 16, 4);
        goal.channel = data[20];
        goal.release_confirmed = data[21];
        if (goal.id <= pose.child_id || goal.id <= axis.goal.id)
            return SERVO_ID;
        return Servo_Begin(&servo, &goal, now);
    }
    if (type == 55 && size == 42)
    {
        PoseGoal goal;
        memset(&goal, 0, sizeof(goal));
        goal.zero_version = version;
        goal.id = (uint32_t)Frame_Get(data + 8, 4);
        goal.revision = (uint32_t)Frame_Get(data + 12, 4);
        goal.timeout_ms = (uint32_t)Frame_Get(data + 16, 4);
        for (index = 0; index < 5; index++)
            goal.target[index] = (int32_t)Frame_Get(data + 20 + index * 4, 4);
        goal.single_step = data[40];
        goal.release_confirmed = data[41];
        if (goal.id <= axis.goal.id || goal.id <= servo.goal.id || goal.id <= pose.child_id)
            return POSE_ID;
        if (bus_diag.motion_uncertain && !Bus_MechDone())
            return AXIS_BUSY;
        return Pose_Begin(&pose, &goal, now);
    }
    return MECH_FRAME;
}

uint8_t MechPort_Command(uint8_t type, const uint8_t *data, uint16_t size,
                         uint8_t *reply, uint16_t *reply_size, uint32_t now)
{
    uint32_t token, result;
    unsigned index;
    if (type == 58 && !size)
    {
        uint32_t values[28];
        memset(values, 0, sizeof(values));
        values[0] = 1;
        values[1] = now;
        values[2] = attached;
        values[3] = guard.fault;
        values[4] = guard.zero_version;
        values[5] = guard.zero_valid;
        values[6] = guard.config_mask;
        values[7] = guard.base_stopped;
        values[8] = axis.state;
        values[9] = axis.reason;
        values[10] = axis.goal.id;
        values[11] = servo.state;
        values[12] = servo.reason;
        values[13] = servo.goal.id;
        values[14] = pose.state;
        values[15] = pose.reason;
        values[16] = pose.goal.id;
        values[17] = pose.goal.revision;
        values[18] = pose.index;
        values[19] = pose.count;
        values[20] = servo.goal.channel;
        values[21] = (uint32_t)servo.goal.target_mdeg;
        values[22] = pwm_mask;
        values[23] = axis.stop_confirmed;
        values[24] = (uint32_t)Mech_Position(&guard, 0);
        values[25] = (uint32_t)Mech_Position(&guard, 1);
        values[26] = guard.stable_samples;
        values[27] = bus_diag.motion_uncertain;
        for (index = 0; index < 28; index++)
            Frame_Put(reply + index * 4, values[index], 4);
        *reply_size = 112;
        return COMM_OK;
    }
    if (size < 8 || size > sizeof(last_data))
        return COMM_FORMAT;
    if (attached && guard.session != Comm_Session())
        return COMM_SESSION;
    token = (uint32_t)Frame_Get(data, 4);
    if (!token || token < last_token ||
        (token == last_token && (type != last_type || size != last_size || memcmp(data, last_data, size))))
        return COMM_FORMAT;
    if (token == last_token)
        result = last_result;
    else
    {
        result = Mutate(type, data, size, now);
        last_token = token;
        last_type = type;
        last_size = size;
        memcpy(last_data, data, size);
        last_result = result;
    }
    Frame_Put(reply, result, 4);
    *reply_size = 4;
    return COMM_OK;
}
