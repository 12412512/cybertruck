/* 正式CAN传输的跨毫秒回调与RX静默夹具；仅主机HAL桩。 */
#include "can_hal_stub.h"
#include "can_internal.h"
#include "bus_config.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define CHECK(value) do { if (!(value)) { fprintf(stderr, "line %d: %s\n", __LINE__, #value); exit(1); } } while (0)

static TestCanRegisters registers;
CAN_HandleTypeDef board_can = {&registers, 0};
static uint32_t tick;
static unsigned occupied, sends, inject;

uint32_t HAL_GetTick(void) { return tick; }
uint32_t HAL_CAN_GetTxMailboxesFreeLevel(const CAN_HandleTypeDef *h)
{ (void)h; return occupied ? 2U : 3U; }
uint32_t HAL_CAN_IsTxMessagePending(const CAN_HandleTypeDef *h, uint32_t boxes)
{ (void)h; (void)boxes; return occupied; }
HAL_StatusTypeDef HAL_CAN_AbortTxRequest(CAN_HandleTypeDef *h, uint32_t boxes)
{ (void)h; (void)boxes; occupied = 0; return HAL_OK; }
HAL_StatusTypeDef HAL_CAN_AddTxMessage(CAN_HandleTypeDef *h, const CAN_TxHeaderTypeDef *header,
                                     const uint8_t *data, uint32_t *mailbox)
{
    (void)h; (void)header; (void)data;
    CHECK(!occupied);
    occupied = 1;
    sends++;
    *mailbox = CAN_TX_MAILBOX0;
    return HAL_OK;
}
HAL_StatusTypeDef HAL_CAN_GetRxMessage(CAN_HandleTypeDef *h, uint32_t fifo,
                                     CAN_RxHeaderTypeDef *header, uint8_t *data)
{
    (void)h; (void)fifo;
    memset(header, 0, sizeof(*header));
    header->IDE = CAN_ID_EXT;
    header->ExtId = 0x100;
    header->DLC = 5;
    memset(data, 0, 8);
    data[0] = 0x35;
    data[4] = 0x6B;
    return HAL_OK;
}
uint32_t HAL_CAN_GetError(const CAN_HandleTypeDef *h) { return h->ErrorCode; }
HAL_StatusTypeDef HAL_CAN_ResetError(CAN_HandleTypeDef *h)
{ h->ErrorCode = 0; return HAL_OK; }
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *h);

/* 模拟CanStep取时后、真正屏蔽中断前发生的跨毫秒回调。 */
void Test_Disable(void)
{
    unsigned event = inject;
    if (!event)
        return;
    inject = 0;
    tick++;
    if (event == 1)
    {
        occupied = 0;
        HAL_CAN_TxMailbox0CompleteCallback(&board_can);
    }
    else if (event == 2)
    {
        occupied = 0;
        board_can.ErrorCode = HAL_CAN_ERROR_TX_ALST0;
        HAL_CAN_ErrorCallback(&board_can);
    }
    else
        HAL_CAN_RxFifo0MsgPendingCallback(&board_can);
}

static void Queue(unsigned count, uint8_t command)
{
    BusFrame frames[2] = {{0}};
    unsigned index;
    for (index = 0; index < count; index++)
    {
        frames[index].id = 0x100 + index;
        frames[index].len = 2;
        frames[index].data[0] = command;
        frames[index].data[1] = 0x6B;
    }
    CHECK(Can_Send(frames, (uint8_t)count));
}

int main(int argc, char **argv)
{
    unsigned which = argc > 1 ? (unsigned)atoi(argv[1]) : 0, index;
    if (which == 0 || which == 1)
    {
        Queue(which == 0 ? 2U : 1U, 0x35);
        Can_Poll();
        CHECK(sends == 1);
        tick = 10;
        inject = which == 0 ? 1U : 2U;
        Can_Poll();
        CHECK(sends == 1);
        tick = which == 0 ? 12U : 11U;
        Can_Poll();
        CHECK(sends == 1);
        tick = which == 0 ? 13U : 12U;
        Can_Poll();
        CHECK(sends == 2);
    }
    else if (which == 2)
    {
        Queue(1, 0x35);
        tick = 10;
        inject = 3;
        Can_Poll();
        CHECK(!sends);
        tick = 12;
        Can_Poll();
        CHECK(!sends);
        tick = 13;
        Can_Poll();
        CHECK(sends == 1);
    }
    else if (which == 3 || which == 4)
    {
        tick = 10;
        HAL_CAN_RxFifo0MsgPendingCallback(&board_can);
        Queue(1, which == 4 ? 0xFE : 0x35);
        Can_Poll();
        CHECK(sends == (which == 4 ? 1U : 0U));
        if (which == 3)
        {
            tick = 11;
            HAL_CAN_RxFifo0MsgPendingCallback(&board_can);
            Can_Poll();
            tick = 12;
            Can_Poll();
            CHECK(!sends);
            tick = 13;
            Can_Poll();
            CHECK(sends == 1);
        }
    }
    else if (which == 5)
    {
        Queue(1, 0x35);
        for (index = 0; index < CAN_TX_TIMEOUT_MS; index++)
        {
            tick = index;
            HAL_CAN_RxFifo0MsgPendingCallback(&board_can);
            Can_Poll();
            CHECK(!sends);
        }
        tick = CAN_TX_TIMEOUT_MS;
        Can_Poll();
        CHECK(Can_Status() == CAN_FAILED && can_diag.tx_timeouts == 1 && !sends);
    }
    else if (which == 6)
    {
        tick = UINT32_MAX - 1U;
        HAL_CAN_RxFifo0MsgPendingCallback(&board_can);
        Queue(1, 0x35);
        Can_Poll();
        CHECK(!sends);
        tick = UINT32_MAX;
        Can_Poll();
        CHECK(!sends);
        tick = 0;
        Can_Poll();
        CHECK(sends == 1);
    }
    else if (which == 7 || which == 8 || which == 9)
    {
        registers.RF0R = 2;
        Queue(1, which == 8 ? 0xFE : 0x35);
        Can_Poll();
        CHECK(sends == (which == 8 ? 1U : 0U));
        if (which == 7)
        {
            registers.RF0R = 0;
            Can_Poll();
            CHECK(sends == 1);
        }
        else if (which == 9)
        {
            tick = CAN_TX_TIMEOUT_MS;
            Can_Poll();
            CHECK(Can_Status() == CAN_FAILED && can_diag.tx_timeouts == 1 && !sends);
        }
    }
    else
        CHECK(0);
    printf("CAN turnaround case %u PASS: simulation only\n", which);
    return 0;
}
