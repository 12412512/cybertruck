#include "mech_port.h"
#include "mech_pose.h"
#include "mech_group.h"
#include "mech_hw.h"
#include "bus_internal.h"
#include "comm.h"
#include "tim.h"
#include <string.h>

static MechGuard guard;
static Axis axis;
static Servo servo;
static Pose pose;
static Group group;
static MotorConfigRx config_rx[2];
static MotorConfig config[2];
static uint8_t attached, pending_kind, pending_axis, enabled_mask, query_index;
static uint8_t pwm_mask, stopping_mask, manual_mode;
static uint8_t axis_done;
static uint8_t hold_phase;
static uint32_t hold_epoch, hold_id, hold_started;
static int64_t hold_position[2];
static uint8_t hold_contract[72]; /* id/epoch/zero/flags/五目标/SHA；由主机核对版本来源。 */
/* 1待输出、2稳定等待、3待人工确认、4已确认、5已中止；不伪造角度反馈。 */
static uint32_t home_id, home_state, home_reason, home_started, home_issued;
static uint8_t enable_pending;
static uint8_t query_kind, query_pending, query_finished;
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
    return attached && hold_phase != 2;
}

uint8_t MechPort_Holding(void)
{
    return attached && hold_phase == 2;
}

uint32_t MechPort_HoldTicket(void)
{
    return MechPort_Holding() ? hold_id : 0;
}

uint8_t MechPort_HoldCheck(uint32_t now)
{
    MechGuard check = guard;
    unsigned index;
    if (!attached || !guard.zero_valid || guard.fault || guard.config_mask != 3 ||
        guard.zero_version != Frame_Get(hold_contract + 8, 4) || pwm_mask != 7 ||
        AxisBusy() || PoseBusy() || ServoBusy() || group.state == GROUP_RUNNING ||
        (home_state >= 1 && home_state <= 3) || enable_pending || pending_kind ||
        guard.concurrent_axes || guard.stop_required || stopping_mask)
        return 0;
    /* 只在副本上检验保持轴；真实base_stopped字段仍报告底盘实际状态。 */
    check.base_stopped = 1;
    if (Mech_GroupReady(&check, now, guard.zero_version))
        return 0;
    for (index = 0; index < 2; index++)
    {
        int64_t position = Mech_Position(&guard, (uint8_t)index);
        int32_t target = (int32_t)Frame_Get(hold_contract + 16 + index * 4, 4);
        int64_t delta = position - target;
        if (target < 0)
            return 0;
        if (delta < -MECH_MANUAL_ENDPOINT_UM || delta > MECH_MANUAL_ENDPOINT_UM)
            return 0;
        /* 接纳终点残差与后续漂移分别监督；不能重新收紧已验收的1mm终点档。 */
        delta = position - hold_position[index];
        if (hold_phase && (delta < -MECH_MANUAL_TOL_UM || delta > MECH_MANUAL_TOL_UM))
            return 0;
    }
    for (index = 0; index < 3; index++)
        if (!(servo.reference_mask & (1U << index)) ||
            servo.command_mdeg[index] != (int32_t)Frame_Get(hold_contract + 24 + index * 4, 4))
            return 0;
    return guard.boot == Comm_Boot() && guard.session == Comm_Session();
}

uint8_t MechPort_Handoff(uint8_t type, const uint8_t *data, uint16_t size,
                         uint8_t *reply, uint16_t *reply_size, uint32_t now, uint8_t stopped)
{
    uint32_t op, id, epoch, reason = MECH_OK;
    unsigned index, digest = 0;
    if (type == 72 && !size)
    {
        uint32_t fields[6] = {1, hold_phase, hold_epoch, hold_id, guard.zero_version, now};
        memcpy(reply, fields, sizeof(fields));
        *reply_size = sizeof(fields);
        return COMM_OK;
    }
    if (type != 71 || size != 76)
        return COMM_FORMAT;
    op = (uint32_t)Frame_Get(data, 4);
    id = (uint32_t)Frame_Get(data + 4, 4);
    epoch = (uint32_t)Frame_Get(data + 8, 4);
    if (op < 1 || op > 4 || !id || epoch == UINT32_MAX || Frame_Get(data + 72, 4))
        return COMM_FORMAT;
    if (op == 1)
    {
        if (id == hold_id)
            reason = hold_phase == 1 && !memcmp(hold_contract, data + 4, 72) ? MECH_OK : MECH_VERSION;
        else if (id < hold_id || epoch != hold_epoch || hold_phase || !stopped ||
                 Frame_Get(data + 16, 4) != 1 || !manual_mode)
            reason = MECH_VERSION;
        else
        {
            for (index = 40; index < 72; index++)
                digest |= data[index];
            if (!digest)
                reason = MECH_VERSION;
            else
            {
                memcpy(hold_contract, data + 4, 72);
                if (!MechPort_HoldCheck(now))
                    reason = MECH_UNSTABLE;
                else
                {
                    for (index = 0; index < 2; index++)
                        hold_position[index] = Mech_Position(&guard, (uint8_t)index);
                    hold_id = id;
                    hold_started = now;
                    hold_phase = 1;
                }
            }
        }
    }
    else if (id != hold_id || memcmp(hold_contract, data + 4, 72))
        reason = MECH_VERSION;
    else if (op == 2)
    {
        if (hold_phase != 2)
        {
            if (hold_phase != 1 || (uint32_t)(now - hold_started) > 2000 ||
                !stopped || !MechPort_HoldCheck(now))
                reason = MECH_UNSTABLE;
            else
            {
                hold_phase = 2;
                hold_epoch++;
            }
        }
    }
    else if (op == 3 || op == 4)
    {
        if (hold_phase && (!stopped || !MechPort_HoldCheck(now)))
            reason = MECH_UNSTABLE;
        else if (hold_phase)
        {
            hold_phase = 0;
            hold_epoch++;
        }
    }
    Frame_Put(reply, reason, 4);
    *reply_size = 4;
    return COMM_OK;
}

uint8_t MechPort_Manual(void)
{
    return attached && manual_mode && !guard.fault && !PoseBusy();
}

uint8_t MechPort_Active(uint32_t *id, uint32_t *channel)
{
    *channel = 0;
    *id = group.state == GROUP_RUNNING ? group.goal.id : PoseBusy() ? pose.goal.id :
          AxisBusy() ? axis.goal.id : ServoBusy() ? servo.goal.id :
          home_state >= 1 && home_state <= 3 ? home_id : 0;
    if (ServoBusy())
        *channel = servo.goal.channel;
    if (group.state == GROUP_RUNNING)
        for (unsigned i = 0; i < 3; i++)
            if (group.goal.mask == (1U << (i + 2)))
                *channel = i + 1;
    return attached && *id != 0;
}

uint8_t MechPort_CanReset(uint32_t now, uint32_t version)
{
    unsigned index;
    if (!attached || version != guard.zero_version || guard.config_mask != 3)
        return 0;
    for (index = 0; index < 8; index++)
        if (!guard.sample[index].valid || !guard.sample[index].rx_sequence ||
            (uint32_t)(now - guard.sample[index].tick) > 250)
            return 0;
    /* 停止后的旧目标可以尚未到达，不能要求实际位置等于中断目标。 */
    return !guard.sample[0].value && !guard.sample[4].value &&
           !(guard.sample[2].value & 12) && !(guard.sample[6].value & 12);
}

uint8_t MechPort_PwmOff(void)
{
    unsigned index;
    uint8_t success = 1;
    for (index = 0; index < 3; index++)
        if (HAL_TIM_PWM_Stop(&htim1, pwm_channels[index]) != HAL_OK)
            success = 0;
    if (success)
        pwm_mask = 0;
    return success;
}

void MechPort_Init(void)
{
    hold_phase = 0;
    hold_epoch = hold_id = hold_started = 0;
    memset(hold_contract, 0, sizeof(hold_contract));
    attached = pending_kind = enabled_mask = query_index = pwm_mask = stopping_mask = axis_done = enable_pending = 0;
    manual_mode = 0;
    home_id = home_state = home_reason = home_started = home_issued = 0;
    last_token = last_result = query_tick = 0;
    query_kind = query_pending = query_finished = 0;
    memset(config, 0, sizeof(config));
    Mech_Init(&guard, 0, 0);
    Axis_Init(&axis, &guard);
    Servo_Init(&servo, &guard, 0);
    Pose_Init(&pose, &axis, &servo);
    Group_Init(&group, &axis, &servo);
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
        Group_Ack(&group, &reply, now);
        if (reply.code == 0x33 || reply.code == 0x35 || reply.code == 0x36 || reply.code == 0x3A)
            (void)Mech_Feed(&guard, &reply, now);
        else if (reply.result != 2 && reply.result != 0x9F)
            MechPort_Stop(MECH_DRIVER, now);
    }
}

void MechPort_Stop(uint32_t reason, uint32_t now)
{
    if (hold_phase)
    {
        hold_phase = 0;
        hold_epoch++;
    }
    if (home_state >= 1 && home_state <= 4)
    {
        home_state = 5;
        home_reason = reason ? reason : MECH_MANUAL;
    }
    if (!attached)
        return;
    manual_mode = 0;
    enable_pending = 0;
    query_kind = query_pending = 0;
    query_tick = now;
    query_finished = 1;
    Mech_Freeze(&guard, reason, 0, now);
    Mech_Invalidate(&guard, reason ? reason : MECH_MANUAL);
    Group_Stop(&group, reason ? reason : MECH_MANUAL, now);
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
    if (hold_phase == 1 && (uint32_t)(now - hold_started) > 2000)
    {
        hold_phase = 0;
        hold_epoch++;
    }
    if (hold_phase && !MechPort_HoldCheck(now))
        MechPort_Stop(MECH_UNSTABLE, now);
    if (axis.state == AXIS_COMPLETE && Mech_Ready(&guard, now, guard.zero_version) == MECH_OK)
        axis_done = 1;
    Pose_Poll(&pose, now, base_stopped);
    Axis_Poll(&axis, now, base_stopped);
    Servo_Poll(&servo, now, base_stopped);
    Group_Poll(&group, now, base_stopped);
    if (home_state >= 1 && home_state <= 3)
    {
        uint32_t result = Mech_Ready(&guard, now, guard.zero_version);
        if (fault || guard.fault || !base_stopped ||
            (result && result != MECH_UNSTABLE) || (uint32_t)(now - home_started) >= 60000)
            MechPort_Stop(result ? result : fault ? MECH_FAULT : !base_stopped ? MECH_BASE : SERVO_TIMEOUT, now);
        else if (!result && home_state == 1)
        {
            /* 三个通道在同一轮明确输出原命令0°；未知初始位置不能假装限速。 */
            for (index = 0; index < 3; index++)
            {
                uint8_t bit = (uint8_t)(1U << index);
                __HAL_TIM_SET_COMPARE(&htim1, pwm_channels[index], Mech_Pulse((uint8_t)(index + 1), 0));
                if (!(pwm_mask & bit) && HAL_TIM_PWM_Start(&htim1, pwm_channels[index]) != HAL_OK)
                {
                    MechPort_Stop(SERVO_OUTPUT_ERROR, now);
                    break;
                }
                pwm_mask |= bit;
            }
            if (home_state == 1)
            {
                home_issued = now;
                home_state = 2;
            }
        }
        else if (home_state == 2 && (uint32_t)(now - home_issued) >= MECH_SERVO_WAIT_MS)
            home_state = 3;
    }
    if (group.state == GROUP_DONE && Mech_Ready(&guard, now, guard.zero_version) == MECH_OK)
        axis_done = 1;
    for (index = 0; index < 4; index++)
    if (!fault && !guard.fault && (index < 3 ? Group_Pwm(&group, index, &output, now) : Servo_Take(&servo, &output, now)))
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
        if (index < 3)
            Group_Applied(&group, index, &output, ok, HAL_GetTick());
        else
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
    if (!attached || query_pending ||
        (query_finished && (uint32_t)(now - query_tick) < MECH_QUERY_GAP_MS))
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
    }
    query_kind = code;
    return Drive_Build(address, code, 0, 0, 0, 0, 0, 0, frames);
}

void MechPort_QueryAccept(void)
{
    /* 接纳不代表TXOK或回包；拒收时重复构造同一反馈项，不跳号。 */
    if (!query_kind || query_pending)
        return;
    query_pending = 1;
    if (query_kind != 0x42)
        query_index = (uint8_t)((query_index + 1) % 8);
    query_kind = 0;
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
    if (group.state == GROUP_RUNNING)
    {
        count = Group_Take(&group, frames, now);
        if (count)
        {
            axis_done = 0;
            pending_kind = 3;
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
    if (query_pending)
    {
        query_pending = 0;
        query_tick = now;
        query_finished = 1;
    }
    if (pending_kind == 1)
        Axis_Sent(&axis, axis.goal.id, success, now);
    if (pending_kind == 2 && success)
        enabled_mask |= (uint8_t)(1U << pending_axis);
    if (pending_kind == 3)
        Group_Sent(&group, success, now);
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
        Group_Init(&group, &axis, &servo);
        Bus_MechWatch();
        return MECH_OK;
    }
    if (!attached || guard.session != Comm_Session())
        return MECH_SESSION;
    if (home_state >= 1 && home_state <= 3 && type != 67 && type != 57)
        return MECH_UNSTABLE;
    if (type == 61 && size == 9)
    {
        uint8_t result;
        uint32_t axis_id = axis.goal.id, servo_id = servo.goal.id;
        uint32_t pose_id = pose.goal.id, child_id = pose.child_id;
        uint32_t group_id = group.goal.id;
        /* 正常停止专用：真实CAN/反馈/机构故障不得通过此入口清除。 */
        if (data[8] != 1 || version != guard.zero_version)
            return MECH_MANUAL;
        if (can_diag.errors || can_diag.rx_overflow || bus_diag.rx_overflow ||
            bus_diag.error != BUS_ERROR_STOP || pwm_mask)
            return MECH_FAULT;
        if (AxisBusy() || ServoBusy() || PoseBusy() || pending_kind || enable_pending ||
            stopping_mask || guard.stop_required || bus_diag.pending_stop_count ||
            bus_diag.stop_sequence != bus_diag.stop_sent_sequence)
            return MECH_UNSTABLE;
        result = Mech_CanPrepare(&guard, now);
        if (result)
            return result;
        if (!Bus_Rearm())
            return MECH_UNSTABLE;
        guard.fault = 0;
        guard.stable_started = 0;
        guard.stable_samples = 0;
        enabled_mask = axis_done = 0;
        Axis_Init(&axis, &guard);
        Servo_Init(&servo, &guard, 0);
        Pose_Init(&pose, &axis, &servo);
        Group_Init(&group, &axis, &servo);
        group.goal.id = group_id;
        /* 不回绕动作编号；重建零点和舵机参考仍必须另行人工确认。 */
        axis.goal.id = axis_id;
        servo.goal.id = servo_id;
        pose.goal.id = pose_id;
        pose.child_id = child_id;
        return MECH_OK;
    }
    if (type == 57 && size == 8)
    {
        MechPort_Stop(MECH_MANUAL, now);
        return MECH_OK;
    }
    if (guard.fault || (bus_diag.error & ~BUS_ERROR_STOP) || can_diag.errors)
        return MECH_FAULT;
    if (version != guard.zero_version && type != 50)
        return MECH_VERSION;
    if ((type == 66 || type == 67) && size == 13)
    {
        uint32_t id = (uint32_t)Frame_Get(data + 8, 4), result;
        if (!manual_mode || data[12] != 1)
            return MECH_MANUAL;
        result = Mech_Ready(&guard, now, version);
        if (result)
            return result;
        if (type == 66)
        {
            if (!id || id <= home_id || id <= axis.goal.id || id <= servo.goal.id ||
                id <= pose.goal.id || id <= group.goal.id || AxisBusy() || ServoBusy() ||
                PoseBusy() || group.state == GROUP_RUNNING || pending_kind || enable_pending)
                return MECH_UNSTABLE;
            home_id = id;
            home_state = 1;
            home_reason = home_issued = 0;
            home_started = now;
            Servo_Init(&servo, &guard, 0);
            servo.goal.id = id; /* 新动作必须继续递增，旧目标不能复活。 */
            return MECH_OK;
        }
        if ((uint32_t)(now - home_started) >= 60000)
            return SERVO_TIMEOUT;
        if (home_state != 3 || id != home_id || pwm_mask != 7)
            return MECH_VERSION;
        {
            const int32_t zero[3] = {0, 0, 0};
            result = Servo_SeedManual(&servo, zero, now, 1);
            if (!result)
            {
                servo.output_mask = 7;
                servo.state = SERVO_MANUAL_DONE;
                home_state = 4;
            }
            return result;
        }
    }
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
    if (group.state == GROUP_RUNNING || AxisBusy() || ServoBusy() || PoseBusy() || enable_pending || pending_kind)
        return MECH_UNSTABLE;
    if (type == 63 && (size == 60 || size == 62))
    {
        GroupGoal goal;
        if (!manual_mode)
            return MECH_MANUAL;
        memset(&goal, 0, sizeof(goal));
        goal.id = (uint32_t)Frame_Get(data + 8, 4);
        goal.zero_version = version;
        goal.timeout_ms = (uint32_t)Frame_Get(data + 12, 4);
        goal.mask = (uint32_t)Frame_Get(data + 16, 4);
        if (size == 62) { goal.accel[0] = data[60]; goal.accel[1] = data[61]; goal.accel_set = 1; }
        if (goal.id <= axis.goal.id || goal.id <= servo.goal.id || goal.id <= pose.child_id)
            return AXIS_ID;
        if (bus_diag.motion_uncertain && !Bus_MechDone())
            return AXIS_BUSY;
        for (index = 0; index < 5; index++)
        {
            goal.target[index] = (int32_t)Frame_Get(data + 20 + index * 4, 4);
            goal.speed[index] = (uint32_t)Frame_Get(data + 40 + index * 4, 4);
        }
        uint32_t result = Group_Begin(&group, &goal, now);
        if (!result && !Bus_MechGroup(group.axis_mask))
        {
            Group_Stop(&group, MECH_FAULT, now);
            return MECH_FAULT;
        }
        return result;
    }
    if (type == 62 && size == 10 && data[8] <= 1)
    {
        uint32_t result;
        if (data[9] != 1)
            return MECH_MANUAL;
        result = Mech_Ready(&guard, now, version);
        if (result)
            return result;
        if (!data[8] && (Mech_Position(&guard, 0) < -MECH_FEEDBACK_TOL_UM ||
                         Mech_Position(&guard, 1) < -MECH_FEEDBACK_TOL_UM ||
                         Mech_Position(&guard, 0) > MECH_LIFT_MAX_UM + MECH_FEEDBACK_TOL_UM ||
                         Mech_Position(&guard, 1) > MECH_REACH_MAX_UM + MECH_FEEDBACK_TOL_UM))
            return MECH_RANGE;
        if (!data[8] && (servo.reference_mask & (1U << (MECH_ARM_CHANNEL - 1U))) &&
            (servo.command_mdeg[MECH_ARM_CHANNEL - 1U] < MECH_ARM_MIN_MDEG ||
             servo.command_mdeg[MECH_ARM_CHANNEL - 1U] > MECH_ARM_MAX_MDEG))
            return MECH_RANGE; /* 扩展手动角度不能直接交给旧自动姿态。 */
        manual_mode = data[8];
        guard.manual_range = manual_mode;
        return MECH_OK;
    }
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
        return manual_mode ? Servo_SeedManual(&servo, initial, now, data[20] == 1) :
            Servo_Seed(&servo, initial, now, data[20] == 1);
    }
    if (type == 52 && (size == 23 || size == 24))
    {
        AxisGoal goal;
        if (size == 24 && !manual_mode)
            return MECH_MANUAL;
        memset(&goal, 0, sizeof(goal));
        goal.id = (uint32_t)Frame_Get(data + 8, 4);
        goal.zero_version = version;
        goal.target_um = (int32_t)Frame_Get(data + 12, 4);
        goal.timeout_ms = (uint32_t)Frame_Get(data + 16, 4);
        goal.rpm = (uint16_t)Frame_Get(data + 20, 2);
        goal.axis = data[22];
        if (size == 24) { goal.accel = data[23]; goal.accel_set = 1; }
        if (goal.id <= pose.child_id || goal.id <= servo.goal.id)
            return AXIS_ID;
        if (bus_diag.motion_uncertain && !Bus_MechDone())
            return AXIS_BUSY;
        return manual_mode ? Axis_Manual(&axis, &goal, now) :
            Axis_Begin(&axis, &goal, now, Servo_Reference(&servo, 2), servo.command_mdeg[1]);
    }
    if (type == 53 && (size == 22 || size == 26))
    {
        if (size == 26 && !manual_mode)
            return MECH_MANUAL;
        ServoGoal goal;
        memset(&goal, 0, sizeof(goal));
        goal.id = (uint32_t)Frame_Get(data + 8, 4);
        goal.zero_version = version;
        goal.target_mdeg = (int32_t)Frame_Get(data + 12, 4);
        goal.timeout_ms = (uint32_t)Frame_Get(data + 16, 4);
        goal.channel = data[20];
        goal.release_confirmed = data[21];
        goal.speed_mdeg_s = size == 26 ? (uint32_t)Frame_Get(data + 22, 4) : 0;
        if (goal.id <= pose.child_id || goal.id <= axis.goal.id)
            return SERVO_ID;
        return manual_mode ? Servo_Manual(&servo, &goal, now) : Servo_Begin(&servo, &goal, now);
    }
    if (type == 55 && size == 42)
    {
        if (manual_mode)
            return MECH_MANUAL;
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
    if (type == 70 && !size)
    {
        const MechFailure *record = &guard.failure;
        uint32_t header[7] = {1, record->captured, record->tick_ms, record->reason,
                              record->axis, record->zero_version, record->flags};
        for (index = 0; index < 7; index++)
            Frame_Put(reply + index * 4, header[index], 4);
        for (index = 0; index < 2; index++)
        {
            Frame_Put(reply + 28 + index * 8, (uint64_t)record->actual[index], 8);
            Frame_Put(reply + 44 + index * 8, (uint64_t)record->target[index], 8);
            Frame_Put(reply + 60 + index * 8, (uint64_t)record->zero_actual[index], 8);
            Frame_Put(reply + 76 + index * 8, (uint64_t)record->zero_target[index], 8);
            Frame_Put(reply + 92 + index * 8, (uint64_t)record->position_um[index], 8);
            Frame_Put(reply + 108 + index * 4, (uint32_t)record->rpm[index], 4);
            Frame_Put(reply + 116 + index * 4, record->age_ms[index], 4);
        }
        *reply_size = 124;
        return COMM_OK;
    }
    if (type == 64 && !size)
    {
        uint32_t values[10] = {1, now, group.goal.id, group.state, group.reason, group.goal.mask, group.axis_mask,
                              (uint32_t)servo.command_mdeg[0], (uint32_t)servo.command_mdeg[1], (uint32_t)servo.command_mdeg[2]};
        for (index = 0; index < 10; index++)
            Frame_Put(reply + index * 4, values[index], 4);
        *reply_size = 40;
        return COMM_OK;
    }
    if (type == 68 && !size)
    {
        uint32_t values[8] = {1, now, home_id, home_state, home_reason,
                              guard.zero_version, pwm_mask, home_issued};
        for (index = 0; index < 8; index++)
            Frame_Put(reply + index * 4, values[index], 4);
        *reply_size = 32;
        return COMM_OK;
    }
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
