#include "stamp.h"

#define STAMP_HALF 0x80000000UL

int Stamp_Check(const RxStamp *stamp, uint32_t now_ms, uint32_t max_age_ms)
{
    uint32_t span;
    uint32_t delay;
    uint32_t age;
    if (!stamp || !stamp->epoch || !max_age_ms || max_age_ms >= STAMP_HALF)
    {
        return 0;
    }
    span = stamp->last_ms - stamp->first_ms;
    delay = stamp->handled_ms - stamp->last_ms;
    age = now_ms - stamp->last_ms;
    return span < STAMP_HALF && delay < STAMP_HALF && age < max_age_ms &&
        delay <= age && (uint32_t)(now_ms - stamp->first_ms) < STAMP_HALF;
}

int Stamp_Next(const RxStamp *previous, const RxStamp *next)
{
    uint32_t sequence_delta;
    uint32_t time_delta;
    if (!previous || !next || !previous->epoch || previous->epoch != next->epoch)
    {
        return 0;
    }
    sequence_delta = next->sequence - previous->sequence;
    time_delta = next->last_ms - previous->last_ms;
    return sequence_delta > 0 && sequence_delta < STAMP_HALF && time_delta < STAMP_HALF;
}
