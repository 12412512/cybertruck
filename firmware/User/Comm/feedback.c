#include "feedback.h"
#include <string.h>

static void Put32(uint8_t *output, uint32_t value)
{
    unsigned index;
    for (index = 0; index < 4; index++)
    {
        output[index] = (uint8_t)(value >> (8 * index));
    }
}

static int Valid(const FeedSample *sample, uint32_t report_ms)
{
    unsigned index;
    if (!sample || !sample->source_epoch || !Stamp_Check(&sample->stamp, report_ms, 250))
    {
        return 0;
    }
    if (sample->kind >= FEED_SPEED && sample->kind <= FEED_STATUS)
    {
        if (!sample->address || sample->values[1] || sample->values[2] ||
            sample->stamp.first_ms != sample->stamp.last_ms)
        {
            return 0;
        }
        if (sample->kind == FEED_SPEED)
        {
            return sample->values[0] >= -5000 && sample->values[0] <= 5000;
        }
        if (sample->kind == FEED_POSITION)
        {
            return sample->values[0] >= -4294967295LL && sample->values[0] <= 4294967295LL;
        }
        return sample->values[0] >= 0 && sample->values[0] <= 255;
    }
    if ((sample->kind != FEED_ANGLE && sample->kind != FEED_GYRO) || sample->address)
    {
        return 0;
    }
    for (index = 0; index < 3; index++)
    {
        if (sample->values[index] < -32768 || sample->values[index] > 32767)
        {
            return 0;
        }
    }
    return 1;
}

int Feed_Encode(const FeedSample *sample, uint32_t report_ms, uint8_t output[FEED_WIRE_SIZE])
{
    unsigned axis;
    unsigned index;
    if (!output)
    {
        return 0;
    }
    memset(output, 0, FEED_WIRE_SIZE);
    if (!Valid(sample, report_ms))
    {
        return 0;
    }
    output[0] = 1;
    output[1] = sample->kind;
    output[2] = sample->address;
    /* 第3字节保留为0：传感器物理采样时刻未知，不能置“已知”标志。 */
    Put32(output + 4, sample->stamp.first_ms);
    Put32(output + 8, sample->stamp.last_ms);
    Put32(output + 12, sample->stamp.handled_ms);
    Put32(output + 16, sample->stamp.sequence);
    Put32(output + 20, sample->stamp.epoch);
    Put32(output + 24, sample->source_epoch);
    Put32(output + 28, report_ms);
    for (axis = 0; axis < 3; axis++)
    {
        uint64_t value = (uint64_t)sample->values[axis];
        for (index = 0; index < 8; index++)
        {
            output[32 + axis * 8 + index] = (uint8_t)(value >> (8 * index));
        }
    }
    return 1;
}
