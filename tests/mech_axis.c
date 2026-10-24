/* 合成反馈驱动正式单轴执行器；不打开串口或连接HAL。 */
#include "mech_axis.h"
#include "mech_hw.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#define CHECK(x) do { if (!(x)) { fprintf(stderr,"line %d: %s\n",__LINE__,#x); exit(1); } } while (0)
static MechGuard guard;
static Axis axis;
static uint32_t now = 100, sequence = 1;
static int64_t lift = 10000, reach = -20000;
static uint8_t status = 3;
static AxisGoal goal = {1, 1, 5000, 2000, 30, 0};
static void Feed(unsigned a, uint8_t code, int64_t value)
{
    MotorReply reply = {0};
    reply.address = a ? 6 : 5; reply.code = code; reply.valid = 1;
    reply.value = value; reply.tick = now; reply.rx_sequence = sequence++;
    if (!sequence) sequence++;
    CHECK(Mech_Feed(&guard, &reply, now) == MECH_OK);
}
static void Round(void)
{
    now += 80;
    Feed(0, 0x35, 0); Feed(0, 0x36, lift); Feed(0, 0x3A, status); Feed(0, 0x33, lift);
    Feed(1, 0x35, 0); Feed(1, 0x36, reach); Feed(1, 0x3A, status); Feed(1, 0x33, reach);
    Mech_Poll(&guard, now, 1);
}
static void Init(void)
{
    unsigned n;
    Mech_Init(&guard, 123, 456); Mech_Config(&guard, 3);
    for (n = 0; n < 5; n++) Round();
    CHECK(Mech_Zero(&guard, now, 1) == MECH_OK);
    Axis_Init(&axis, &guard);
}
static void Start(void)
{
    BusFrame frames[2];
    CHECK(Axis_Begin(&axis, &goal, now, 1, 180000) == MECH_OK);
    CHECK(axis.state == AXIS_ACCEPTED && !axis.emitted);
    CHECK(Axis_Take(&axis, frames, now) == 2);
    CHECK(frames[0].id == (goal.axis ? 0x600U : 0x500U));
    CHECK(frames[0].data[0] == 0xFD && frames[0].data[1] == (goal.axis ? 1 : 0));
    CHECK(frames[0].data[4] == 1 && frames[1].data[2] == 0);
    CHECK(!Axis_Take(&axis, frames, now));
    Axis_Sent(&axis, goal.id, 1, ++now);
    CHECK(axis.state == AXIS_RUNNING);
}
static void Ack(uint8_t result)
{
    MotorReply reply = {0};
    reply.address = goal.axis ? 6 : 5; reply.code = 0xFD; reply.result = result;
    reply.valid = 1; reply.tick = now; reply.rx_sequence = sequence++;
    Axis_Ack(&axis, &reply, now);
}
static void Arrive(void)
{
    unsigned n;
    if (goal.axis) reach += axis.delta_raw; else lift += axis.delta_raw;
    for (n = 0; n < 3; n++) { Round(); Axis_Poll(&axis, now, 1); }
}
int main(int argc, char **argv)
{
    unsigned which = argc > 1 ? (unsigned)atoi(argv[1]) : 0, n;
    BusFrame frames[2];
    if (which == 11) { now = UINT32_MAX - 600; sequence = UINT32_MAX - 60; }
    Init();
    if (which == 1)
    {
        guard.zero_valid = 0;
        CHECK(Axis_Begin(&axis, &goal, now, 1, 180000) == MECH_NO_ZERO);
        CHECK(!Axis_Take(&axis, frames, now));
    }
    else if (which == 2)
    {
        goal.target_um = 49001;
        CHECK(Axis_Begin(&axis, &goal, now, 1, 180000) == MECH_RANGE);
    }
    else if (which == 3)
    {
        goal.axis = 1;
        CHECK(Axis_Begin(&axis, &goal, now, 1, 180000) == MECH_COLLISION);
    }
    else if (which == 12 || which == 27)
    {
        CHECK(Axis_Begin(&axis, &goal, now, 1, 180000) == MECH_OK);
        if (which == 12) { lift += 40; Round(); }
        else now += 251;
        CHECK(!Axis_Take(&axis, frames, now));
        CHECK(axis.reason == (which == 12 ? AXIS_TARGET : MECH_STALE));
    }
    else if (which == 17 || which == 18)
    {
        CHECK(Axis_Begin(&axis, &goal, now, 1, 180000) == MECH_OK);
        CHECK(Axis_Take(&axis, frames, now) == 2);
        if (which == 17)
        {
            Axis_Sent(&axis, 99, 1, now); CHECK(axis.state == AXIS_SENDING);
            now += 50; Axis_Poll(&axis, now, 1);
        }
        else Axis_Sent(&axis, goal.id, 0, now);
        CHECK(axis.reason == AXIS_TX && axis.state == AXIS_STOPPING);
    }
    else
    {
        if (which == 10)
        {
            /* 非零升降姿态已经稳定；初始零点不因这个软件反馈而重置。 */
            lift += 76000;
            for (n = 0; n < 5; n++) Round();
            goal.axis = 1;
        }
        Start();
        if (which == 0 || which == 10 || which == 11)
        {
            Ack(2); Arrive(); CHECK(axis.state == AXIS_COMPLETE);
            CHECK(Axis_Begin(&axis, &goal, now, 1, 180000) == MECH_OK);
            CHECK(!Axis_Take(&axis, frames, now));
            goal.target_um++;
            CHECK(Axis_Begin(&axis, &goal, now, 1, 180000) == AXIS_ID);
        }
        else if (which == 4)
        {
            Ack(2); now += 251; Axis_Poll(&axis, now, 1);
            CHECK(axis.state == AXIS_STOPPING && axis.reason == MECH_STALE && !guard.zero_valid);
        }
        else if (which == 5)
        {
            for (n = 0; n < 4; n++) { Round(); Axis_Poll(&axis, now, 1); }
            CHECK(axis.reason == AXIS_ACK && axis.state == AXIS_STOPPING);
        }
        else if (which == 6)
        {
            Ack(0xE2); CHECK(axis.reason == AXIS_ACK && axis.state == AXIS_STOPPING);
        }
        else if (which == 7)
        {
            Ack(2); reach += 100; Round(); Axis_Poll(&axis, now, 1);
            CHECK(axis.reason == AXIS_OTHER && axis.state == AXIS_STOPPING);
        }
        else if (which == 8)
        {
            Ack(2); Axis_Cancel(&axis, now);
            CHECK(axis.state == AXIS_STOPPING && !axis.stop_confirmed && guard.stop_required);
            Axis_StopSent(&axis, 1, now);
            for (n = 0; n < 5; n++) { Round(); Axis_Poll(&axis, now, 1); }
            CHECK(!axis.stop_confirmed);
            Axis_StopSent(&axis, 2, now);
            for (n = 0; n < 5; n++) { Round(); Axis_StopSent(&axis, 2, now); Axis_Poll(&axis, now, 1); }
            CHECK(axis.stop_confirmed && axis.state == AXIS_CANCELLED && !guard.zero_valid && guard.fault);
        }
        else if (which == 9)
        {
            Ack(2); Arrive();
            CHECK(axis.state == AXIS_COMPLETE);
            Axis_Poll(&axis, now + 1000, 1); /* 完成是历史结果，不变成新动作。 */
            CHECK(axis.state == AXIS_COMPLETE);
            goal.id++; goal.target_um = 0;
            CHECK(Axis_Begin(&axis, &goal, now + 1000, 1, 180000) == MECH_STALE);
        }
        else if (which == 13)
        {
            Ack(2); Round(); Axis_Poll(&axis, now, 0);
            CHECK(axis.reason == MECH_BASE && axis.state == AXIS_STOPPING);
        }
        else if (which == 14)
        {
            Ack(2); guard.session++; Axis_Poll(&axis, now, 1);
            CHECK(axis.reason == AXIS_ZERO_CHANGED && axis.state == AXIS_STOPPING);
        }
        else if (which == 15)
        {
            Ack(2); status = 1; Arrive(); CHECK(axis.state != AXIS_COMPLETE);
        }
        else if (which == 16)
        {
            Ack(2);
            for (n = 0; n < 60; n++) { Round(); Axis_Poll(&axis, now, 1); }
            now = axis.started_ms + goal.timeout_ms; Round(); Axis_Poll(&axis, now, 1);
            CHECK(axis.reason == AXIS_TIMEOUT && axis.state == AXIS_STOPPING);
        }
        else if (which == 19)
        {
            Ack(0x9F); Arrive(); CHECK(axis.state != AXIS_COMPLETE && !axis.ack_seen);
            Round(); Axis_Poll(&axis, now, 1); CHECK(axis.reason == AXIS_ACK);
        }
        else if (which == 20)
        {
            Ack(2); lift -= 100; Round(); Axis_Poll(&axis, now, 1);
            CHECK(axis.reason == AXIS_REVERSE && axis.state == AXIS_STOPPING);
        }
        else if (which == 21)
        {
            Ack(2); status = 13; Round(); Axis_Poll(&axis, now, 1);
            CHECK(axis.reason == MECH_DRIVER && axis.state == AXIS_STOPPING);
        }
        else if (which == 22)
        {
            Axis_Cancel(&axis, now); Axis_StopSent(&axis, 3, now);
            for (n = 0; n < 10; n++) { now += 80; Axis_Poll(&axis, now, 1); }
            CHECK(axis.state == AXIS_STOPPING && !axis.stop_confirmed && guard.stop_required);
        }
        else if (which == 23)
        {
            goal.id++;
            CHECK(Axis_Begin(&axis, &goal, now, 1, 180000) == AXIS_BUSY && axis.goal.id == 1);
        }
        else if (which == 24)
        {
            MotorReply reply = {0};
            reply.valid = 1; reply.address = 5; reply.code = 0xFD; reply.result = 2;
            reply.tick = now - 1; reply.rx_sequence = sequence++;
            Axis_Ack(&axis, &reply, now); CHECK(!axis.ack_seen);
        }
        else if (which == 25)
        {
            Ack(2); Arrive(); CHECK(axis.state == AXIS_COMPLETE);
            for (n = 0; n < 3; n++) Round();
            goal.id++; goal.target_um = 0;
            CHECK(Axis_Begin(&axis, &goal, now, 1, 180000) == MECH_OK);
            CHECK(Axis_Take(&axis, frames, now) == 2 && frames[0].data[1] == 1);
            Axis_Sent(&axis, goal.id, 1, ++now); Ack(2); Arrive();
            CHECK(axis.state == AXIS_COMPLETE && axis.pulse_position[0] == 0 && lift == 10000);
        }
        else if (which == 26)
        {
            Ack(2); guard.zero_version++; Axis_Poll(&axis, now, 1);
            CHECK(axis.state == AXIS_STOPPING && axis.reason == AXIS_ZERO_CHANGED);
        }
        else if (which == 28)
        {
            uint32_t checked = now;
            Ack(2); lift += axis.delta_raw; now = axis.sent_ms - 1;
            for (n = 0; n < 3; n++)
            {
                Feed(0, 0x35, 0); Feed(0, 0x36, lift); Feed(0, 0x3A, 3); Feed(0, 0x33, lift);
                Feed(1, 0x35, 0); Feed(1, 0x36, reach); Feed(1, 0x3A, 3); Feed(1, 0x33, reach);
                Axis_Poll(&axis, checked, 1);
            }
            CHECK(axis.state != AXIS_COMPLETE && !axis.settled_samples);
        }
        else CHECK(0);
    }
    printf("axis intent case %u PASS: no IO\n", which);
    return 0;
}
