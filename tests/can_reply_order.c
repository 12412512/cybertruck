/* 复用正式联合夹具，单独注入硬件TXOK与HAL回调的不同先后顺序。 */
#define main ExistingQueryMain
#include "can_query_capture.c"
#undef main

static void WireDone(void)
{
    /* 邮箱0实际发送成功，但TX中断尚未被服务。 */
    occupied = 0;
    regs.TSR = 3U;
}

static void CallbackDone(void)
{
    occupied = 0;
    regs.TSR = 0;
    HAL_CAN_TxMailbox0CompleteCallback(&board_can);
}

static unsigned inject_error;
static void ReceiveError(const BusFrame *frame)
{
    (void)frame;
    if (inject_error == 1)
    {
        inject_error = 0;
        board_can.ErrorCode = HAL_CAN_ERROR_RX_FOV0;
        HAL_CAN_ErrorCallback(&board_can);
    }
    else if (inject_error == 2)
    {
        inject_error = 0;
        can_diag.rx_overflow++;
    }
}

int main(int argc, char **argv)
{
    unsigned which = argc > 1 ? (unsigned)atoi(argv[1]) : 0;
    Can_Refresh();
    Bus_Setup(addresses);
    if (which == 3)
        tick = UINT32_MAX - 2U;
    if (which == 12 || which == 13)
    {
        Query(6, 0x33);
        Fail(HAL_CAN_ERROR_TX_TERR0);
        if (which == 12)
        {
            CallbackDone();
            CHECK(can_diag.tx_done_sequence == 0);
        }
        else
            can_diag.rx_overflow++;
        Bus_Poll();
        CHECK(bus_diag.locked && (bus_diag.error & BUS_ERROR_CAN));
        if (which == 13) CHECK(bus_diag.error & BUS_ERROR_RX);
        CHECK(Bus_Status() != CAN_SENT && sends == 1 && !Bus_Rearm());
        return 0;
    }
    if (which >= 8)
    {
        /* ID6/33专用：不能借错误ID、畸形帧、旧回复或TERR推进查询。 */
        Query(6, 0x33);
        CHECK(can_diag.last_tx_id==0x600 && can_diag.last_tx_length==2 &&
              can_diag.last_tx_data0==0x6B33);
        tick++;
        if(which!=9)WireDone();
        if(which==10)
        {
            Reply(5,0x33,0);Reply(6,0x36,0);Reply(6,0x33,1);
            Bus_Poll();CHECK(!bus_diag.query_replies);
        }
        if(which==11)
        {
            Fail(HAL_CAN_ERROR_TX_TERR0);Reply(6,0x33,0);Bus_Poll();
            CHECK(!bus_diag.query_replies&&can_diag.errors==1&&bus_diag.locked);
            CHECK(!Bus_Rearm());
            return 0;
        }
        Reply(6,0x33,0);Bus_Poll();
        if(which==9)
        {
            CHECK(!bus_diag.query_replies);CallbackDone();Bus_Poll();tick=BUS_REPLY_TIMEOUT_MS;
            Bus_Poll();CHECK(bus_diag.query_timeouts==1&&!Bus_Rearm());
        }
        else
        {
            CHECK(bus_diag.query_replies==1&&Bus_Status()==CAN_BUSY);
            tick++;CallbackDone();tick++;Bus_Poll();
            CHECK(Bus_Status()==CAN_SENT&&!bus_diag.query_timeouts);
            Reply(6,0x33,0);Bus_Poll();CHECK(bus_diag.query_replies==1);
        }
        return 0;
    }
    Query(1, 0x35);
    tick++;
    if (which == 0 || which == 3 || which == 5)
    {
        WireDone();
        if (which == 5)
        {
            Reply(2, 0x35, 0);
            Reply(1, 0x36, 0);
            Bus_Poll();
            CHECK(!bus_diag.query_replies);
        }
        Reply(1, 0x35, 0);
        Bus_Poll();
        /* 回复可先锁存，但不能在HAL回调前完成事务。 */
        CHECK(bus_diag.query_replies == 1 && Bus_Status() == CAN_BUSY);
        tick++;
        CallbackDone();
        Bus_Poll();
        CHECK(Bus_Status() == CAN_BUSY);
        tick++;
        Bus_Poll();
        CHECK(Bus_Status() == CAN_SENT && !bus_diag.query_timeouts);
    }
    else if (which == 1 || which == 4)
    {
        /* 没有真实TXOK的帧，即使稍后处理时已有回调，也不能冒充新回复。 */
        Reply(1, 0x35, 0);
        if (which == 1)
            Bus_Poll();
        tick++;
        CallbackDone();
        Bus_Poll();
        CHECK(!bus_diag.query_replies);
        tick = BUS_REPLY_TIMEOUT_MS;
        Bus_Poll();
        CHECK(bus_diag.query_timeouts == 1 && Bus_Status() == CAN_FAILED);
    }
    else if (which == 2)
    {
        /* 邮箱结束或空闲不等于TXOK；TERR不能被回复解锁。 */
        regs.TSR = 9U;
        Reply(1, 0x35, 0);
        Bus_Poll();
        CHECK(!bus_diag.query_replies);
        Fail(HAL_CAN_ERROR_TX_TERR0);
        Bus_Poll();
        CHECK(bus_diag.error & BUS_ERROR_CAN);
        CHECK(!bus_diag.query_replies && can_capture.valid);
    }
    else if (which == 6 || which == 7)
    {
        Done();
        Bus_Poll();
        tick++;
        Reply(1, 0x35, 0);
        tick += CAN_TX_TURNAROUND_MS;
        inject_error = which == 6 ? 1U : 2U;
        Bus_Observe(ReceiveError);
        Bus_Poll();
        CHECK(bus_diag.locked && Bus_Status() != CAN_SENT);
        CHECK(bus_diag.error & (which == 6 ? BUS_ERROR_CAN : BUS_ERROR_RX));
        CHECK(sends == 1);
    }
    else
        CHECK(0);
    printf("reply order case %u PASS: offline only\n", which);
    return 0;
}
