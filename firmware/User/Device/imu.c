#include "imu.h"
#include <string.h>

static int Known(uint8_t kind)
{
    return (kind >= 0x50 && kind <= 0x5A) || kind == 0x5F;
}

static void Invalidate(ImuParser *parser)
{
    parser->epoch++;
    if (!parser->epoch)
    {
        parser->epoch++;
    }
    parser->gaps++;
}

static void Partial(ImuParser *parser, uint32_t received_ms)
{
    if (parser->used && (uint32_t)(received_ms - parser->previous_ms) > IMU_PARTIAL_MS)
    {
        parser->used = 0;
        Invalidate(parser);
    }
}

static void Resync(ImuParser *parser)
{
    unsigned start;
    unsigned remaining;
    for (start = 1; start < parser->used; start++)
    {
        remaining = parser->used - start;
        if (parser->bytes[start] != 0x55 ||
            (remaining > 1 && !Known(parser->bytes[start + 1])))
        {
            continue;
        }
        /* 移窗同步保留每个原字节的接收时间，不改贴成当前处理时间。 */
        memmove(parser->bytes, parser->bytes + start, remaining);
        memmove(parser->ticks, parser->ticks + start, remaining * sizeof(parser->ticks[0]));
        parser->used = (uint8_t)remaining;
        return;
    }
    parser->used = 0;
}

static int16_t Signed(const uint8_t *bytes)
{
    uint32_t raw = bytes[0] | ((uint32_t)bytes[1] << 8);
    int32_t value = raw < 32768 ? (int32_t)raw : (int32_t)raw - 65536;
    return (int16_t)value;
}

static void Publish(ImuParser *parser, uint32_t handled_ms)
{
    ImuSample sample;
    unsigned axis;
    unsigned index;
    memset(&sample, 0, sizeof(sample));
    sample.stamp.first_ms = parser->ticks[0];
    sample.stamp.last_ms = parser->ticks[10];
    sample.stamp.handled_ms = handled_ms;
    sample.stamp.sequence = ++parser->sequence;
    sample.stamp.epoch = parser->epoch;
    sample.source_epoch = parser->source_epoch;
    sample.kind = parser->bytes[1];
    if (!Stamp_Check(&sample.stamp, handled_ms, 250))
    {
        Invalidate(parser);
        return;
    }
    if (sample.kind != 0x52 && sample.kind != 0x53)
    {
        return;
    }
    for (axis = 0; axis < 3; axis++)
    {
        sample.axes[axis] = Signed(parser->bytes + 2 + 2 * axis);
    }
    memcpy(sample.raw, parser->bytes, sizeof(sample.raw));
    if (parser->count == IMU_SLOTS)
    {
        /* 丢新样本、保留序号缺口，同时使旧缓存失效，不能继续积分。 */
        parser->overflow++;
        Invalidate(parser);
        return;
    }
    index = (parser->head + parser->count) % IMU_SLOTS;
    parser->queue[index] = sample;
    parser->count++;
}

void Imu_Init(ImuParser *parser, uint32_t source_epoch)
{
    if (!parser)
    {
        return;
    }
    memset(parser, 0, sizeof(*parser));
    parser->source_epoch = source_epoch;
    parser->epoch = 1;
}

void Imu_Loss(ImuParser *parser, uint32_t source_epoch)
{
    if (!parser)
    {
        return;
    }
    parser->used = 0;
    parser->seen = 0;
    parser->source_epoch = source_epoch;
    Invalidate(parser);
}

void Imu_Feed(ImuParser *parser, uint8_t byte, uint32_t received_ms,
              uint32_t handled_ms, uint32_t source_epoch)
{
    unsigned index;
    uint8_t sum = 0;
    if (!parser)
    {
        return;
    }
    if (source_epoch != parser->source_epoch)
    {
        Imu_Loss(parser, source_epoch);
    }
    if (!source_epoch)
    {
        return;
    }
    if (parser->seen && (uint32_t)(received_ms - parser->previous_ms) >= 0x80000000UL)
    {
        parser->used = 0;
        Invalidate(parser);
        return;
    }
    Partial(parser, received_ms);
    parser->previous_ms = received_ms;
    parser->seen = 1;
    if (!parser->used && byte != 0x55)
    {
        Invalidate(parser);
        return;
    }
    parser->bytes[parser->used] = byte;
    parser->ticks[parser->used] = received_ms;
    parser->used++;
    if (parser->used >= 2 && !Known(parser->bytes[1]))
    {
        parser->bad++;
        Invalidate(parser);
        Resync(parser);
        return;
    }
    if (parser->used < 11)
    {
        return;
    }
    for (index = 0; index < 10; index++)
    {
        sum = (uint8_t)(sum + parser->bytes[index]);
    }
    if (sum != parser->bytes[10])
    {
        parser->bad++;
        Invalidate(parser);
        Resync(parser);
        return;
    }
    parser->good++;
    Publish(parser, handled_ms);
    parser->used = 0;
}

void Imu_Tick(ImuParser *parser, uint32_t now_ms)
{
    if (parser)
    {
        Partial(parser, now_ms);
    }
}

int Imu_Read(ImuParser *parser, ImuSample *sample)
{
    if (!sample)
    {
        return 0;
    }
    memset(sample, 0, sizeof(*sample));
    if (!parser || !parser->count)
    {
        return 0;
    }
    *sample = parser->queue[parser->head];
    parser->head = (parser->head + 1) % IMU_SLOTS;
    parser->count--;
    return 1;
}

int Imu_Valid(const ImuParser *parser, const ImuSample *sample, uint32_t now_ms)
{
    return parser && sample && parser->source_epoch &&
        parser->source_epoch == sample->source_epoch && parser->epoch == sample->stamp.epoch &&
        Stamp_Check(&sample->stamp, now_ms, 250);
}
