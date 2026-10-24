#include "base_port.h"
#include "exec.h"
#include "bus_internal.h"
#include "port.h"
#include "comm.h"
#include "execute.h"
#include "addresses.h"
#include "can.h"
#include <math.h>
#include <stdlib.h>
#include <string.h>

static Exec executor;
static uint32_t diag_busy, query_index, query_ms, enable_sent;
static uint32_t zero_sequence, zero_action, zero_id;
static uint32_t stop_barrier, stable_tick;
static int64_t stable_position[4];
static ImuSample gyro;
static const uint8_t addresses[4] = BASE_SAFE_ADDRESSES;
static uint32_t startup_pending, startup_ms;
typedef char ExecConfigSize[(sizeof(ExecConfig) == 52) ? 1 : -1];
typedef char ExecGoalSize[(sizeof(TrackGoal) == 96) ? 1 : -1];

static int Ready(uint32_t now, int enabled, uint32_t since)
{
    const BusWheel *wheels = Bus_Wheels();
    unsigned index;
    for (index = 0; index < 4; index++)
    {
        const BusWheel *wheel = wheels + index;
        if (!wheel->speed.valid || !wheel->status.valid || !wheel->position.valid ||
            (uint32_t)(now - wheel->speed.tick) > 250 || (uint32_t)(now - wheel->status.tick) > 250 ||
            (uint32_t)(now - wheel->position.tick) > 250 || llabs(wheel->speed.value) > 1 ||
            (wheel->status.value & 12) || (enabled && !(wheel->status.value & 1)) ||
            (enabled && (uint32_t)(wheel->status.tick - since) >= 0x80000000UL))
        {
            return 0;
        }
    }
    return 1;
}

static int EnableWheel(unsigned wheel)
{
    BusFrame frames[2];
    uint8_t count;
    if (diag_busy)
    {
        return 0;
    }
    if (enable_sent)
    {
        enable_sent = 0;
        return 1;
    }
    count = Drive_Build(executor.config.address[wheel], 0xF3, 0, 0, 0, 0, 0, 1, frames);
    if (count && Bus_Send(frames, count))
    {
        diag_busy = 1;
        enable_sent = 1;
    }
    return 0;
}

static int Send(const int16_t rpm[4], uint32_t action, uint32_t zero)
{
    if (!Bus_Speed(rpm))
    {
        return 0;
    }
    if (!zero || !action)
    {
        zero_id = zero_action = zero_sequence = 0;
    }
    else if (zero_id != zero || zero_action != action)
    {
        zero_id = zero;
        zero_action = action;
        zero_sequence = bus_diag.speed_sequence;
    }
    return 1;
}

static void Stop(void)
{
    Bus_Stop();
    diag_busy = enable_sent = 0;
    zero_id = zero_sequence = zero_action = 0;
    stop_barrier = stable_tick = 0;
    Bus_Release();
}

static uint32_t Fault(void)
{
    return (bus_diag.error & ~BUS_ERROR_STOP) | can_diag.errors | can_diag.rx_overflow;
}

static int Rearm(void)
{
    /* 仅清理正常STOP的本地发送锁；任何CAN/RX/驱动异常不自动清除。 */
    return !Fault() && bus_diag.error == BUS_ERROR_STOP && Bus_Rearm();
}

static int Stopped(uint32_t since)
{
    const BusWheel *wheels = Bus_Wheels();
    uint32_t now = HAL_GetTick();
    uint32_t oldest = now;
    unsigned index;
    if (bus_diag.pending_stop_count || bus_diag.stop_sequence != bus_diag.stop_sent_sequence)
    {
        return 0;
    }
    if (!stop_barrier)
    {
        stop_barrier = now;
    }
    if (!Ready(now, 0, 0) ||
        (executor.config.version && (!Stamp_Check(&gyro.stamp, now, 250) ||
                                     fabs(gyro.axes[2] * executor.config.gyro_scale_rad_s) > 0.035)))
    {
        stable_tick = 0;
        return 0;
    }
    for (index = 0; index < 4; index++)
    {
        uint32_t tick = wheels[index].position.tick;
        if ((uint32_t)(tick - stop_barrier) >= 0x80000000UL || tick == stop_barrier ||
            (uint32_t)(wheels[index].speed.tick - stop_barrier) >= 0x80000000UL ||
            (uint32_t)(tick - since) >= 0x80000000UL)
        {
            return 0;
        }
        if ((uint32_t)(now - tick) > (uint32_t)(now - oldest))
        {
            oldest = tick;
        }
        if (stable_tick && llabs(wheels[index].position.value - stable_position[index]) > 32)
        {
            stable_tick = 0;
        }
    }
    if (!stable_tick)
    {
        for (index = 0; index < 4; index++)
        {
            stable_position[index] = wheels[index].position.value;
        }
        stable_tick = oldest;
        return 0;
    }
    return (uint32_t)(oldest - stable_tick) >= 150;
}

static void StopCommand(uint32_t reason, uint32_t now)
{
    Exec_Stop(&executor, reason, now);
}

static uint8_t Command(uint8_t type, const uint8_t *data, uint16_t size, uint8_t *reply, uint16_t *reply_size,
                       uint32_t now)
{
    uint32_t configured = executor.config.version;
    if (startup_pending && type != EXEC_STATE && type != 16)
    {
        return COMM_BUSY;
    }
    uint8_t result = Execute_Command(&executor, type, data, size, reply, reply_size, now, can_diag.errors,
                                     can_diag.rx_sequence);
    if (!configured && executor.config.version && result == COMM_OK)
    {
        Bus_Setup(executor.config.address);
    }
    return result;
}

void Base_Init(void)
{
    CAN_FilterTypeDef filter = {0};
    ExecIo io = {Ready, EnableWheel, Bus_Start, Send, Stop, Fault, Stopped, Rearm};
    CommHooks hooks = {StopCommand, Command};
    Exec_Init(&executor, io);
    /* 启动接收不等于使能或动作；配置/使能/目标分别显式提交。 */
    filter.FilterMode = CAN_FILTERMODE_IDMASK;
    filter.FilterScale = CAN_FILTERSCALE_32BIT;
    filter.FilterIdLow = CAN_ID_EXT;
    filter.FilterMaskIdLow = CAN_ID_EXT | CAN_RTR_REMOTE;
    filter.FilterFIFOAssignment = CAN_RX_FIFO0;
    filter.FilterActivation = ENABLE;
    filter.SlaveStartFilterBank = 14;
    if (HAL_CAN_ConfigFilter(&hcan1, &filter) != HAL_OK || HAL_CAN_Start(&hcan1) != HAL_OK ||
        HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING | CAN_IT_RX_FIFO0_OVERRUN |
                                                 CAN_IT_TX_MAILBOX_EMPTY | CAN_IT_ERROR | CAN_IT_BUSOFF) !=
            HAL_OK)
    {
        Exec_Stop(&executor, 0x400U, HAL_GetTick());
    }
    /* 复位不能让外部驱动继续保持旧RPM；只发送停止，不使能、不启动轨迹。 */
    Bus_Setup(addresses);
    Bus_Stop();
    startup_pending = 1;
    startup_ms = HAL_GetTick();
    Comm_Bind(hooks);
}

static void Query(uint32_t now)
{
    BusFrame frames[2];
    uint8_t count;
    const uint8_t kinds[3] = {0x35, 0x36, 0x3A};
    if (diag_busy || (executor.mode == EXEC_DISABLED && !startup_pending) || executor.mode == EXEC_READY ||
        executor.mode == EXEC_ACTIVE || (uint32_t)(now - query_ms) < 5)
    {
        return;
    }
    count = Drive_Build(addresses[query_index / 3], kinds[query_index % 3], 0, 0, 0, 0, 0, 0, frames);
    if (count && Bus_Send(frames, count))
    {
        diag_busy = 1;
        query_index = (query_index + 1) % 12;
        query_ms = now;
    }
}

void Base_Run(void)
{
    ImuSample sample;
    MotorSample motor;
    BusFrame ignored;
    const BusWheel *wheels;
    const MotorReply *reply;
    uint32_t now = HAL_GetTick();
    unsigned index, kind;
    Bus_Poll();
    if (diag_busy && Bus_Status() != CAN_BUSY && Bus_Status() != CAN_CANCELLING)
    {
        if (Bus_Status() != CAN_SENT)
        {
            Exec_Stop(&executor, 0x800U, now);
        }
        while (Bus_Read(&ignored))
        {
        }
        Bus_Release();
        diag_busy = 0;
    }
    wheels = Bus_Wheels();
    for (index = 0; index < 4; index++)
    {
        for (kind = 0; kind < 3; kind++)
        {
            reply = kind == 2   ? &wheels[index].status
                    : kind == 1 ? &wheels[index].position
                                : &wheels[index].speed;
            if (!reply->valid)
            {
                continue;
            }
            memset(&motor, 0, sizeof(motor));
            motor.address = reply->address;
            motor.kind = reply->code;
            motor.value = reply->value;
            motor.stamp.first_ms = motor.stamp.last_ms = reply->tick;
            motor.stamp.handled_ms = reply->handled_ms;
            motor.stamp.sequence = reply->rx_sequence;
            motor.stamp.epoch = 1;
            Exec_Motor(&executor, &motor, now);
        }
    }
    while (Port_Imu(&sample))
    {
        if (sample.kind == 0x52)
        {
            gyro = sample;
        }
        Exec_Imu(&executor, &sample, now);
    }
    if (startup_pending && Stopped(startup_ms) && Rearm())
    {
        startup_pending = 0;
        executor.stop_confirmed = 1;
    }
    if (zero_sequence && (uint32_t)(bus_diag.speed_sent_sequence - zero_sequence) < 0x80000000UL &&
        !bus_diag.sent_rpm[0] && !bus_diag.sent_rpm[1] && !bus_diag.sent_rpm[2] && !bus_diag.sent_rpm[3])
    {
        Exec_Sent(&executor, zero_action, zero_id, bus_diag.sent_tick);
    }
    Exec_Run(&executor, now);
    Query(now);
}
