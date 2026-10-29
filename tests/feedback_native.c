/* 正式反馈/时间负载的主机桥，不链接HAL，不访问设备。 */
#include "motor.h"
#include "imu.h"
#include "feedback.h"
#include <string.h>

#ifdef _WIN32
#define EXPORT __declspec(dllexport)
#else
#define EXPORT
#endif

static ImuParser parser;

EXPORT int Test_Stamp(const uint32_t *times, uint32_t now)
{
    RxStamp stamp = {times[0], times[1], times[2], times[3], times[4]};
    return Stamp_Check(&stamp, now, 250);
}

EXPORT int Test_Next(const uint32_t *before, const uint32_t *after)
{
    RxStamp first = {before[0], before[1], before[2], before[3], before[4]};
    RxStamp last = {after[0], after[1], after[2], after[3], after[4]};
    return Stamp_Next(&first, &last);
}

EXPORT int Test_Motor(uint32_t id, const uint8_t *bytes, uint8_t length,
                      uint32_t received, uint32_t handled, uint8_t extended,
                      uint8_t remote, int64_t *value, uint32_t *times)
{
    MotorFrame frame = {0};
    MotorSample sample;
    int result;
    frame.id = id;
    frame.length = length;
    memcpy(frame.data, bytes, 8);
    frame.received_ms = received;
    frame.sequence = 73;
    frame.epoch = 4;
    frame.extended = extended;
    frame.remote = remote;
    result = Motor_Read(&frame, handled, &sample);
    *value = sample.value;
    times[0] = sample.stamp.first_ms;
    times[1] = sample.stamp.last_ms;
    times[2] = sample.stamp.handled_ms;
    times[3] = sample.stamp.sequence;
    times[4] = sample.stamp.epoch;
    times[5] = sample.kind;
    return result;
}

EXPORT void Test_ImuInit(uint32_t epoch)
{
    Imu_Init(&parser, epoch);
}

EXPORT void Test_Byte(uint8_t byte, uint32_t received, uint32_t handled, uint32_t epoch)
{
    Imu_Feed(&parser, byte, received, handled, epoch);
}

EXPORT void Test_Loss(uint32_t epoch)
{
    Imu_Loss(&parser, epoch);
}

EXPORT void Test_Tick(uint32_t now)
{
    Imu_Tick(&parser, now);
}

EXPORT int Test_Read(uint32_t now, int16_t *axes, uint32_t *times, uint8_t *bytes)
{
    ImuSample sample;
    if (!Imu_Read(&parser, &sample))
    {
        return 0;
    }
    memcpy(axes, sample.axes, sizeof(sample.axes));
    memcpy(bytes, sample.raw, sizeof(sample.raw));
    times[0] = sample.stamp.first_ms;
    times[1] = sample.stamp.last_ms;
    times[2] = sample.stamp.handled_ms;
    times[3] = sample.stamp.sequence;
    times[4] = sample.stamp.epoch;
    times[5] = sample.source_epoch;
    times[6] = sample.kind;
    return Imu_Valid(&parser, &sample, now) ? 1 : 2;
}

EXPORT void Test_Counts(uint32_t *counts)
{
    counts[0] = parser.good;
    counts[1] = parser.bad;
    counts[2] = parser.gaps;
    counts[3] = parser.overflow;
    counts[4] = parser.count;
}

EXPORT int Test_Encode(uint8_t kind, uint8_t address, const uint32_t *times,
                       const int64_t *values, uint32_t report, uint8_t *output)
{
    FeedSample sample;
    sample.kind = kind;
    sample.address = address;
    sample.stamp.first_ms = times[0];
    sample.stamp.last_ms = times[1];
    sample.stamp.handled_ms = times[2];
    sample.stamp.sequence = times[3];
    sample.stamp.epoch = times[4];
    sample.source_epoch = times[5];
    memcpy(sample.values, values, sizeof(sample.values));
    return Feed_Encode(&sample, report, output);
}
