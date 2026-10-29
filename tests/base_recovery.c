/* 正式板级65入口；合成静止反馈，不访问硬件。 */
#define BASE_RESET_COUNTER
#define BASE_STARTUP_NO_MAIN
#include "base_startup.c"

int main(int argc, char **argv)
{
    unsigned which = argc > 1 ? (unsigned)atoi(argv[1]) : 0, i;
    uint8_t data[16] = {0}, reply[128];
    uint16_t size = 0;
    Can_Refresh(); Comm_Init(123, tick); Base_Init();
    tick = 1000;
    BusWheel *items = (BusWheel *)(void *)Bus_Wheels();
    for (i = 0; i < 4; i++)
    {
        items[i].speed.valid = items[i].position.valid = items[i].status.valid = 1;
        items[i].speed.tick = items[i].position.tick = items[i].status.tick = tick;
        items[i].speed.value = items[i].position.value = 0;
        items[i].status.value = 3;
    }
    bus_diag.pending_stop_count = 0;
    bus_diag.stop_sent_sequence = bus_diag.stop_sequence;
    executor.mode = EXEC_LOCKED;
    executor.fault = 0x800;
    executor.stop_confirmed = 1;
    stop_barrier = 100;
    stable_tick = 100;
    reset_ready = 1;
    Frame_Put(data, 1, 4); Frame_Put(data+4, 0x52454356, 4);
    Frame_Put(data+8, 7, 4); Frame_Put(data+12, 7, 4);
    if (which == 1) data[8] = 0;
    if (which == 2) executor.stop_confirmed = 0;
    if (which == 3) items[2].speed.tick = tick-251;
    if (which == 4) reset_ready = 0;
    if (which == 5) executor.mode = EXEC_ACTIVE;
    if (which == 7) reset_pwm_off = 0;
    uint8_t result = Command(65, data, sizeof(data), reply, &size, tick);
    if (which == 1)
        CHECK(result == COMM_FORMAT && !recovery_pending);
    else if (which >= 2 && which <= 5)
        CHECK(result == COMM_OK && Frame_Get(reply,4) == MECH_UNSTABLE && !recovery_pending);
    else if (which == 7)
        CHECK(result == COMM_OK && Frame_Get(reply,4) == MECH_FAULT && !recovery_pending);
    else
    {
        CHECK(result == COMM_OK && !Frame_Get(reply,4) && recovery_pending);
        CHECK(executor.fault && !sends); /* 接受不是清故障，更不是发送运动。 */
        if (which == 6 || which == 8)
        {
            bus_diag.pending_stop_count = 0;
            bus_diag.stop_sent_sequence = bus_diag.stop_sequence;
            stop_barrier = stable_tick = 100;
            tick += 249;
            for (i=0;i<4;i++) items[i].speed.tick=items[i].position.tick=items[i].status.tick=tick;
            Recover_Run(tick); CHECK(!reset_calls && recovery_pending);
            tick++;
            if (which == 8) reset_ready = 0;
            Recover_Run(tick);
            if (which == 6) CHECK(reset_calls == 1 && !recovery_pending);
            else {CHECK(!reset_calls && recovery_pending);tick+=2000;Recover_Run(tick);CHECK(!recovery_pending);}
        }
        else {StopCommand(MECH_MANUAL, tick+1);CHECK(!recovery_pending && !reset_calls);
            if(which==9){CHECK(Command(65,data,sizeof(data),reply,&size,tick+1)==COMM_FORMAT);CHECK(!recovery_pending);}}
    }
    return 0;
}
