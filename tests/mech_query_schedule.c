/* 六节点正式调度联调：合成HAL和回包，不连接设备、不建零或使能。 */
#define BASE_REAL_MECH
#define BASE_STARTUP_NO_MAIN
#include "base_startup.c"

int htim1;
uint32_t HAL_TIM_PWM_Start(int *timer, uint32_t channel)
{ (void)timer; (void)channel; CHECK(0); return 1; }
void Test_Compare(uint32_t channel, uint32_t pulse)
{ (void)channel; (void)pulse; CHECK(0); }

static void ConfigReply(unsigned address)
{
    uint8_t raw[32] = {0x42, 0x21, 0x15, 0x19};
    unsigned packet, offset = 1;
    raw[8] = 16;
    raw[9] = 1;
    raw[19] = (uint8_t)address;
    raw[31] = 0x6B;
    for (packet = 0; offset < sizeof(raw); packet++)
    {
        unsigned size = sizeof(raw) - offset;
        if (size > 7) size = 7;
        memset(&incoming, 0, sizeof(incoming));
        incoming.id = (address << 8) | packet;
        incoming.len = (uint8_t)(size + 1);
        incoming.data[0] = 0x42;
        memcpy(incoming.data + 1, raw + offset, size);
        HAL_CAN_RxFifo0MsgPendingCallback(&board_can);
        offset += size;
    }
}

static void FeedbackReply(unsigned address, unsigned code)
{
    memset(&incoming, 0, sizeof(incoming));
    incoming.id = address << 8;
    incoming.len = code == 0x35 ? 5 : code == 0x36 || code == 0x33 ? 7 : 3;
    incoming.data[0] = (uint8_t)code;
    if (code == 0x3A && address >= 5) incoming.data[1] = 3;
    incoming.data[incoming.len - 1] = 0x6B;
    HAL_CAN_RxFifo0MsgPendingCallback(&board_can);
}

int main(int argc, char **argv)
{
    unsigned which = argc > 1 ? (unsigned)atoi(argv[1]) : 0;
    unsigned attached_at = 1800, seen = 0, pending_address = 0, pending_code = 0;
    unsigned injected = 0, omitted = 0, post_stop = 0, queries = 0;
    unsigned stale_gate = 0, prepared = 0;
    uint32_t due = 0, last_axis_done = 0, maximum_age = 0;
    uint32_t axis_stamps[8] = {0}, maximum_axis_age = 0, last_axis_reply = 0;
    unsigned axis_seen = 0, axis_reply_valid = 0;
    uint8_t attach[8] = {1, 0, 0, 0, 0x48, 0x43, 0x45, 0x4D}, response[128];
    uint16_t size = 0;
    unsigned elapsed, wheel, kind;
    if (which == 5) tick = UINT32_MAX - 2000U;
    uint32_t start = tick;
    Can_Refresh();
    Comm_Init(123, tick);
    Base_Init();
    for (elapsed = 0; elapsed < 6000; elapsed++)
    {
        tick = start + elapsed;
        if (occupied)
        {
            unsigned address = last.id >> 8, code = last.data[0];
            CHECK(code == 0xFE || code == 0x42 || code == 0x35 ||
                  code == 0x36 || code == 0x3A || code == 0x33);
            if (!injected && elapsed > 2400 &&
                ((address == 5 && ((which == 2 && code == 0x3A) || (which == 3 && code == 0x36))) ||
                 (which == 7 && address == 6 && code == 0x33)))
            {
                injected = 1;
                Fail(HAL_CAN_ERROR_TX_TERR0);
            }
            else
            {
                Done();
                if (code == 0xFE)
                {
                    post_stop += elapsed > 2400;
                }
                else
                {
                    CHECK(!pending_code);
                    pending_address = address;
                    pending_code = code;
                    due = tick + (which == 1 ? 7U : which == 6 ? 8U : 0U);
                    queries++;
                    if (address >= 5 && last_axis_done)
                        CHECK((uint32_t)(tick - 1U - last_axis_done) >= 10U);
                }
            }
        }
        if (pending_code && (uint32_t)(tick - due) < 0x80000000U)
        {
            if (which == 4 && !omitted && pending_address == 5 &&
                pending_code == 0x36 && elapsed > 2400)
                omitted = 1;
            else if (pending_code == 0x42)
            {
                ConfigReply(pending_address);
                last_axis_reply = tick;
                axis_reply_valid = 1;
            }
            else
            {
                FeedbackReply(pending_address, pending_code);
                if (pending_address >= 5)
                {
                    unsigned channel = pending_code == 0x35 ? 0 : pending_code == 0x36 ? 1 :
                                       pending_code == 0x3A ? 2 : 3;
                    unsigned index = (pending_address - 5) * 4 + channel;
                    axis_stamps[index] = tick;
                    axis_seen |= 1U << index;
                    last_axis_reply = tick;
                    axis_reply_valid = 1;
                }
            }
            pending_code = 0;
        }
        if (elapsed == attached_at)
        {
            CHECK(!startup_pending && !executor.fault && executor.stop_confirmed);
            CHECK(MechPort_Command(48, attach, sizeof(attach), response, &size, tick) == COMM_OK);
            CHECK(size == 4 && !Frame_Get(response, 4));
        }
        /* 扫过40个毫秒相位，覆盖正在发出/等待回包/完成交接的STOP。 */
        unsigned stop_at = which == 8 ? 3000U : which == 49 ? 2401U : 2400U + which - 9U;
        if(which>=8 && elapsed==stop_at)
        {
            StopCommand(0,tick);
            if(which==49){Fail(HAL_CAN_ERROR_TX_TERR0);injected=1;}
        }
        unsigned was_busy = diag_busy;
        unsigned was_axis = (last.id >> 8) >= 5 && last.data[0] != 0xFE;
        Base_Run();
        if(which>=8 && elapsed>4000 && !prepared && !diag_busy && !can_diag.tx_pending)
        {
            uint8_t prepare[9]={2,0,0,0,0,0,0,0,1};
            uint8_t result=Command(61,prepare,9,response,&size,tick);
            if(result==COMM_OK && size==4 && !Frame_Get(response,4))prepared=1;
        }
        if (was_busy && !diag_busy && was_axis) last_axis_done = tick;
        if (sends != seen)
        {
            seen = sends;
            if (getenv("QUERY_TRACE") && elapsed > 2100 && elapsed < 2500)
                printf("%u ID%u %02x slot%u\n", elapsed, last.id >> 8, last.data[0], query_slot);
            if ((last.id >> 8) >= 5 && last.data[0] != 0xFE && last_axis_done)
                CHECK((uint32_t)(tick - last_axis_done) >= 10U);
            if ((last.id >> 8) >= 5 && last.data[0] != 0xFE && axis_reply_valid)
                CHECK((uint32_t)(tick - last_axis_reply) >= 12U);
        }
        CHECK(!executor.enabled_wheels && !executor.have_goal && !executor.arm_ms);
        if (!injected && !omitted && elapsed > 2400 && which<8)
        {
            if (executor.fault || can_diag.errors || bus_diag.query_timeouts)
                fprintf(stderr, "tick %u fault %u bus %u timeout %u ID%u/0x%x\n",
                        tick, executor.fault, bus_diag.error, bus_diag.query_timeouts,
                        bus_timeout.address, bus_timeout.code);
            CHECK(!executor.fault && !can_diag.errors && !bus_diag.query_timeouts);
            for (wheel = 0; wheel < 4; wheel++)
            {
                const BusWheel *feedback = Bus_Wheels() + wheel;
                const MotorReply *items[3] = {&feedback->speed, &feedback->position, &feedback->status};
                for (kind = 0; kind < 3; kind++)
                {
                    CHECK(items[kind]->valid);
                    uint32_t age = tick - items[kind]->tick;
                    if (which == 6 && age > 250U)
                    {
                        stale_gate++;
                        CHECK(!Ready(tick, 0, 0));
                    }
                    else CHECK(age <= 250U);
                    if (age > maximum_age) maximum_age = age;
                }
            }
            CHECK(axis_seen == 255U);
            for (kind = 0; kind < 8; kind++)
            {
                uint32_t age = tick - axis_stamps[kind];
                if (which != 6) CHECK(age <= 250U);
                if (age > maximum_axis_age) maximum_axis_age = age;
            }
        }
    }
    CHECK(queries > 30);
    if(which>=8 && which!=49)CHECK(prepared&&!executor.fault&&!can_diag.errors);
    if (which == 2 || which == 3 || which == 7 || which == 49)
    {
        CHECK(injected && can_diag.errors == 1 && can_capture.valid);
        CHECK(executor.fault && bus_diag.locked && post_stop >= 6);
        CHECK(!bus_diag.query_timeouts);
        if (which == 7 || which == 49)
        {
            /* 对应本次现场ID6/33真实TERR：首次错误不应变成查询超时或自动恢复。 */
            uint8_t prepare[9]={2,0,0,0,0,0,0,0,1};
            CHECK(Command(61,prepare,9,response,&size,tick)==COMM_OK && size==4 &&
                  Frame_Get(response,4)==14U);
            if(which==7)CHECK(can_capture.first[6] == 0x600 && (can_capture.first[8] & 0xFF) == 0x33);
            CHECK(MechPort_Command(58, 0, 0, response, &size, tick) == COMM_OK && size == 112);
            CHECK(!Frame_Get(response + 20, 4) && !Frame_Get(response + 88, 4));
        }
    }
    else if (which == 4)
    {
        CHECK(omitted && bus_timeout.valid && bus_timeout.address == 5 && bus_timeout.code == 0x36);
        CHECK((uint32_t)(bus_timeout.tick - bus_timeout.start) == 100U);
        CHECK(executor.fault && bus_diag.locked && post_stop >= 6 && !can_diag.errors);
    }
    else
    {
        CHECK(!executor.fault && !bus_diag.locked && !can_diag.errors);
        CHECK(MechPort_Command(58, 0, 0, response, &size, tick) == COMM_OK && size == 112);
        /* 配置、稳定采样和停车成立；人工零点仍无效，未开放动作。 */
        CHECK(Frame_Get(response + 8, 4) == 1 && !Frame_Get(response + 12, 4));
        CHECK(!Frame_Get(response + 20, 4) && Frame_Get(response + 24, 4) == 3);
        if (which == 6) CHECK(stale_gate);
        else CHECK(Frame_Get(response + 28, 4) == 1 && Frame_Get(response + 104, 4) > 0);
        CHECK(!Frame_Get(response + 40, 4) && !Frame_Get(response + 52, 4));
        CHECK(!Frame_Get(response + 64, 4) && !Frame_Get(response + 88, 4));
    }
    printf("six-node case %u PASS: max wheel/axis age %u/%u ms, queries %u, offline only\n",
           which, maximum_age, maximum_axis_age, queries);
    return 0;
}

uint32_t HAL_TIM_PWM_Stop(int *timer, uint32_t channel) { (void)timer;(void)channel;return HAL_OK; }
