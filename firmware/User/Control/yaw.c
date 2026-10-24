#include "yaw.h"
#include <float.h>
#include <math.h>
#include <stddef.h>
#include <string.h>

#define YAW_PI 3.14159265358979323846
#define YAW_HALF UINT32_C(0x80000000)

static int Finite(double value)
{
    return value == value && value >= -FLT_MAX && value <= FLT_MAX;
}

static int Valid(const YawHistory *history)
{
    return history && history->fault == YAW_OK && history->first < YAW_SLOTS &&
        history->count > 0U && history->count <= YAW_SLOTS;
}

int Yaw_Init(YawHistory *history, uint32_t tick_ms, float yaw_rad,
             float max_rate_rad_s, uint32_t max_gap_ms)
{
    if (!history)
    {
        return 0;
    }
    memset(history, 0, sizeof(*history));
    history->fault = YAW_CONFIG;
    if (!Finite(yaw_rad) || fabs((double)yaw_rad) > YAW_PI + 1e-6 ||
        !Finite(max_rate_rad_s) || max_rate_rad_s <= 0 || max_gap_ms == 0U || max_gap_ms > 250U)
    {
        return 0;
    }
    history->samples[0].tick_ms = tick_ms;
    history->samples[0].yaw_rad = yaw_rad;
    history->last_raw_rad = yaw_rad;
    history->max_rate_rad_s = max_rate_rad_s;
    history->max_gap_ms = max_gap_ms;
    history->count = 1;
    history->fault = YAW_OK;
    return 1;
}

int Yaw_Push(YawHistory *history, uint32_t tick_ms, float yaw_rad)
{
    unsigned latest;
    unsigned next;
    unsigned oldest;
    uint32_t elapsed;
    double delta;
    double continuous;
    if (!Valid(history))
    {
        return 0;
    }
    if (!Finite(yaw_rad) || fabs((double)yaw_rad) > YAW_PI + 1e-6)
    {
        history->fault = YAW_INPUT;
        return 0;
    }
    latest = (history->first + history->count - 1U) % YAW_SLOTS;
    elapsed = tick_ms - history->samples[latest].tick_ms;
    if (elapsed == 0U || elapsed >= YAW_HALF)
    {
        history->fault = YAW_TIME;
        return 0;
    }
    if (elapsed > history->max_gap_ms)
    {
        history->fault = YAW_GAP;
        return 0;
    }
    delta = (double)yaw_rad - history->last_raw_rad;
    if (delta > YAW_PI)
    {
        delta -= 2.0 * YAW_PI;
    }
    else if (delta < -YAW_PI)
    {
        delta += 2.0 * YAW_PI;
    }
    if (fabs(delta) >= YAW_PI - 1e-6 ||
        fabs(delta) > history->max_rate_rad_s * ((double)elapsed / 1000.0) + 1e-6)
    {
        history->fault = YAW_JUMP;
        return 0;
    }
    continuous = history->samples[latest].yaw_rad + delta;
    if (!Finite(continuous))
    {
        history->fault = YAW_INPUT;
        return 0;
    }
    oldest = history->count == YAW_SLOTS ? (history->first + 1U) % YAW_SLOTS : history->first;
    if (tick_ms - history->samples[oldest].tick_ms >= YAW_HALF)
    {
        history->fault = YAW_TIME;
        return 0;
    }
    next = (latest + 1U) % YAW_SLOTS;
    history->samples[next].tick_ms = tick_ms;
    history->samples[next].yaw_rad = (float)continuous;
    history->last_raw_rad = yaw_rad;
    if (history->count < YAW_SLOTS)
    {
        history->count++;
    }
    else
    {
        history->first = oldest;
    }
    return 1;
}

int Yaw_At(const YawHistory *history, uint32_t tick_ms, float *yaw_rad)
{
    const YawSample *previous;
    const YawSample *current;
    unsigned latest;
    unsigned index;
    uint32_t span;
    uint32_t offset;
    uint32_t before = 0;
    uint32_t after;
    double value;
    if (yaw_rad)
    {
        *yaw_rad = 0;
    }
    if (!Valid(history) || !yaw_rad)
    {
        return -1;
    }
    previous = &history->samples[history->first];
    latest = (history->first + history->count - 1U) % YAW_SLOTS;
    span = history->samples[latest].tick_ms - previous->tick_ms;
    offset = tick_ms - previous->tick_ms;
    if (offset >= YAW_HALF || span >= YAW_HALF)
    {
        return -1;
    }
    if (offset > span)
    {
        return 0;
    }
    if (offset == 0U)
    {
        *yaw_rad = previous->yaw_rad;
        return 1;
    }
    for (index = 1; index < history->count; index++)
    {
        current = &history->samples[(history->first + index) % YAW_SLOTS];
        after = current->tick_ms - history->samples[history->first].tick_ms;
        if (offset <= after)
        {
            value = previous->yaw_rad + ((double)current->yaw_rad - previous->yaw_rad) *
                (double)(offset - before) / (double)(after - before);
            if (!Finite(value))
            {
                return -1;
            }
            *yaw_rad = (float)value;
            return 1;
        }
        previous = current;
        before = after;
    }
    return -1;
}

int Yaw_Fresh(const YawHistory *history, uint32_t now_ms)
{
    unsigned latest;
    if (!Valid(history))
    {
        return 0;
    }
    latest = (history->first + history->count - 1U) % YAW_SLOTS;
    return (uint32_t)(now_ms - history->samples[latest].tick_ms) <= history->max_gap_ms;
}
