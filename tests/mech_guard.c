#include "mech_guard.h"
#include "mech_hw.h"
#ifdef NDEBUG
#undef NDEBUG
#endif
#include <assert.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#undef assert
#define assert(condition) do { if (!(condition)) { fprintf(stderr, "check failed line %d: %s\n", __LINE__, #condition); exit(1); } } while (0)

static MechGuard guard;
static uint32_t now = 100, sequence = 1;

static void Feed(uint8_t axis, uint8_t code, int64_t value)
{
    MotorReply reply;
    memset(&reply, 0, sizeof(reply));
    reply.address = axis ? MECH_REACH_ID : MECH_LIFT_ID;
    reply.code = code;
    reply.value = value;
    reply.tick = now;
    reply.rx_sequence = sequence++;
    if (!reply.rx_sequence)
        reply.rx_sequence = sequence++;
    reply.valid = 1;
    assert(Mech_Feed(&guard, &reply, now) == MECH_OK);
}

static void Round(int64_t lift, int64_t reach)
{
    Feed(0, 0x35, 0);
    Feed(0, 0x36, lift);
    Feed(0, 0x3A, 1);
    Feed(0, 0x33, lift);
    Feed(1, 0x35, 0);
    Feed(1, 0x36, reach);
    Feed(1, 0x3A, 1);
    Feed(1, 0x33, reach);
    Mech_Poll(&guard, now, 1);
}

static void StableAt(int64_t lift, int64_t reach)
{
    unsigned index;
    for (index = 0; index < 5; index++)
    {
        now += 80;
        Round(lift, reach);
    }
}

static void Init(void)
{
    Mech_Init(&guard, 123, 456);
    Mech_Config(&guard, 3);
}

static void Zero(void)
{
    Init();
    StableAt(10000, -20000); /* 非零编码器也可建立软件相对基准。 */
    assert(Mech_Zero(&guard, now, 1) == MECH_OK);
    assert(guard.zero_valid && guard.zero_version == 1);
}

static void Protocol(void)
{
    BusFrame frames[2], frame;
    MotorReply reply;
    assert(Drive_Build(5, 0x33, 0, 0, 0, 0, 0, 0, frames) == 1);
    assert(frames[0].id == 0x500 && frames[0].len == 2);
    assert(frames[0].data[0] == 0x33 && frames[0].data[1] == 0x6B);
    memset(&frame, 0, sizeof(frame));
    frame.id = 0x600; frame.len = 7; frame.tick = 123; frame.rx_sequence = 42;
    frame.data[0] = 0x33; frame.data[1] = 1;
    frame.data[2] = frame.data[3] = frame.data[4] = frame.data[5] = 0xFF;
    frame.data[6] = 0x6B;
    assert(Drive_Parse(&frame, 6, &reply));
    assert(reply.value == -(int64_t)UINT32_MAX && reply.tick == 123 && reply.rx_sequence == 42);
    assert(!Drive_Parse(&frame, 5, &reply));
    frame.data[1] = 2;
    assert(!Drive_Parse(&frame, 6, &reply));
    frame.data[1] = 0; frame.data[6] = 0;
    assert(!Drive_Parse(&frame, 6, &reply));
    puts("protocol 33/sign/check/address: PASS");
}

int main(void)
{
    MotorReply duplicate;
    uint32_t count;
    int64_t lift49 = (int64_t)49000 * MECH_RAW_PER_320 / MECH_LIFT_UM_PER_320;
    Protocol();
    Init();
    assert(!guard.zero_valid && Mech_Position(&guard, 0) == INT64_MIN);
    assert(Mech_Zero(&guard, now, 0) == MECH_MANUAL);
    assert(Mech_Zero(&guard, now, 1) == MECH_BASE);
    Round(10000, -20000);
    assert(Mech_Zero(&guard, now, 1) == MECH_UNSTABLE);
    count = guard.stable_samples;
    now += 200;
    Mech_Poll(&guard, now, 1);
    assert(guard.stable_samples == count);
    assert(Mech_Zero(&guard, now, 1) == MECH_UNSTABLE);
    puts("manual baseline/new samples: PASS");

    Zero();
    assert(Mech_Position(&guard, 0) == 0 && Mech_Position(&guard, 1) == 0);
    assert(Mech_Zero(&guard, now, 1) == MECH_ALREADY_ZERO);
    assert(Mech_CheckAxis(&guard, now, 2, 0, 1000, 1, 180000) == MECH_VERSION);
    assert(Mech_CheckAxis(&guard, now, 1, 0, -1, 1, 180000) == MECH_RANGE);
    assert(Mech_CheckAxis(&guard, now, 1, 0, 49001, 1, 180000) == MECH_RANGE);
    assert(Mech_CheckAxis(&guard, now, 1, 0, 1000, 1, 180000) == MECH_OK);
    assert(Mech_CheckAxis(&guard, now, 1, 1, 50000, 1, 180000) == MECH_COLLISION);
    puts("zero/version/command limits: PASS");

    duplicate = guard.sample[1];
    assert(Mech_Feed(&guard, &duplicate, now) == MECH_FRAME);
    duplicate.rx_sequence = sequence++; duplicate.tick = now + 1;
    assert(Mech_Feed(&guard, &duplicate, now) == MECH_FRAME);
    duplicate.tick = now; duplicate.value = INT64_MAX;
    assert(Mech_Feed(&guard, &duplicate, now) == MECH_FRAME);
    duplicate.value = 10000; duplicate.tick = now - 1;
    assert(Mech_Feed(&guard, &duplicate, now) == MECH_FRAME);
    puts("duplicate/future/backward/overflow frame: PASS");

    Init(); StableAt(10000, -20000);
    Feed(0, 0x36, 10001);
    assert(Mech_Zero(&guard, now, 1) == MECH_UNSTABLE);
    puts("unpolled newer feedback cannot reuse stable evidence: PASS");

    Zero();
    now += 251; Mech_Poll(&guard, now, 1);
    assert(!guard.zero_valid && guard.fault == MECH_STALE && guard.stop_required);
    StableAt(10000, -20000);
    assert(Mech_Zero(&guard, now, 1) == MECH_STALE);
    puts("250ms/fault latched: PASS");

    Zero(); Mech_Config(&guard, 1);
    assert(!guard.zero_valid && guard.fault == MECH_CONFIG && guard.stop_required);
    Zero(); Feed(0, 0x3A, 5); Mech_Poll(&guard, now, 1);
    assert(guard.fault == MECH_DRIVER && !guard.zero_valid);
    puts("config/driver fault: PASS");

    Zero(); StableAt(10000 + 6554, -20000 - 6554);
    assert(Mech_Position(&guard, 0) == 4098);
    assert(Mech_Position(&guard, 1) == 12400);
    assert(Mech_CheckRotate(&guard, now, 1, 0, 1, 180000) == MECH_COLLISION);
    assert(Mech_CheckAxis(&guard, now, 1, 0, 0, 1, 180000) == MECH_COLLISION);
    puts("old measured scale/sign and collision gates: PASS");

    Zero(); StableAt(10000 + lift49, -20000);
    assert(Mech_CheckAxis(&guard, now, 1, 1, 50000, 0, 180000) == MECH_COLLISION);
    assert(Mech_CheckAxis(&guard, now, 1, 1, 50000, 1, 83000) == MECH_COLLISION);
    assert(Mech_CheckAxis(&guard, now, 1, 1, 50000, 1, 180000) == MECH_OK);
    puts("extension arm/lift preconditions: PASS");

    Zero();
    assert(Mech_CheckRotate(&guard, now, 1, 1, 0, 0) == MECH_COLLISION);
    assert(Mech_CheckRotate(&guard, now, 1, 1, 1, 180000) == MECH_OK);
    Mech_Poll(&guard, now, 0);
    assert(Mech_CheckAxis(&guard, now, 1, 0, 1000, 1, 180000) == MECH_BASE);
    puts("tray clearance/base mutual exclusion: PASS");

    Zero(); Feed(0, 0x36, 10000 + lift49 + 6554); Feed(0, 0x35, 10);
    Mech_Poll(&guard, now, 1);
    assert(guard.fault == MECH_RANGE && !guard.zero_valid);
    puts("feedback limit checked while moving: PASS");

    now = UINT32_MAX - 500; sequence = UINT32_MAX - 30;
    Zero();
    assert(guard.zero_valid && !guard.fault);
    StableAt(10000, -20000);
    assert(guard.zero_valid && !guard.fault);
    Mech_Invalidate(&guard, MECH_SESSION);
    assert(!guard.zero_valid && guard.stop_required);
    Mech_Init(&guard, 124, 789);
    assert(!guard.zero_valid && !guard.config_mask);
    Mech_Invalidate(&guard, 256); /* 外部故障码不能截断为uint8_t的成功0。 */
    assert(Mech_Zero(&guard, now, 1) == MECH_FAULT);
    assert(Mech_CheckRotate(&guard, now, 1, 0, 1, 180000) == MECH_FAULT);
    assert(guard.fault == 256);
    puts("timestamp/sequence wrap and new session: PASS");

    assert(Mech_Pulses(0, 49000) == 3826);
    assert(Mech_Pulses(0, 43000) == 3357);
    assert(Mech_Pulses(1, 50000) == 1290);
    assert(Mech_Pulses(1, 50001) == UINT32_MAX);
    assert(Mech_Pulse(1, 0) == 500 && Mech_Pulse(1, 30000) == 722);
    assert(Mech_Pulse(1, 150000) == 1611 && Mech_Pulse(1, 270000) == 2500);
    assert(Mech_Pulse(2, 180000) == 1497 && Mech_Pulse(2, 83000) == 2038);
    assert(Mech_Pulse(3, 98000) == 1226 && !Mech_Pulse(3, 99000));
    assert(!Mech_Pulse(0, 0) && !Mech_Pulse(1, -1));
    puts("old pulse whitelist/PWM formulas: PASS");
    puts("13 passive mechanism groups: PASS; no device IO linked");
    return 0;
}
