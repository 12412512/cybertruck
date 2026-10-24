/* 正式CAN/总线/电机解析联合夹具；仅模拟寄存器和回包，不连接硬件。 */
#include "can_hal_stub.h"
#include "bus_internal.h"
#include "bus_config.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#define CHECK(x) do { if (!(x)) { fprintf(stderr, "line %d: %s\n", __LINE__, #x); exit(1); } } while (0)

static TestCanRegisters regs;
CAN_HandleTypeDef board_can = {&regs, 0};
static uint32_t tick;
static unsigned occupied, sends, stops;
static BusFrame last, incoming;
static const uint8_t addresses[4] = {1, 2, 3, 4};
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
    (void)h;
    CHECK(!occupied);
    memset(&last, 0, sizeof(last));
    last.id = header->ExtId; last.len = (uint8_t)header->DLC;
    memcpy(last.data, data, 8);
    occupied = 1; sends++; stops += data[0] == 0xFE;
    *mailbox = CAN_TX_MAILBOX0;
    return HAL_OK;
}
HAL_StatusTypeDef HAL_CAN_GetRxMessage(CAN_HandleTypeDef *h, uint32_t fifo,
                                     CAN_RxHeaderTypeDef *header, uint8_t *data)
{
    (void)h; (void)fifo;
    memset(header, 0, sizeof(*header)); header->IDE = CAN_ID_EXT;
    header->ExtId = incoming.id; header->DLC = incoming.len;
    memcpy(data, incoming.data, 8);
    return HAL_OK;
}
uint32_t HAL_CAN_GetError(const CAN_HandleTypeDef *h) { return h->ErrorCode; }
HAL_StatusTypeDef HAL_CAN_ResetError(CAN_HandleTypeDef *h)
{ h->ErrorCode = 0; return HAL_OK; }
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *h);
static void Done(void)
{
    CHECK(occupied); occupied = 0;
    HAL_CAN_TxMailbox0CompleteCallback(&board_can);
}
static void Reply(unsigned address, unsigned code, int malformed)
{
    memset(&incoming, 0, sizeof(incoming)); incoming.id = address << 8;
    incoming.len = code == 0x35 ? 5 : code == 0x36 || code == 0x33 ? 7 : 3;
    incoming.data[0] = (uint8_t)code;
    incoming.data[incoming.len - 1] = malformed ? 0 : 0x6B;
    HAL_CAN_RxFifo0MsgPendingCallback(&board_can);
}
static void Query(unsigned address, unsigned code)
{
    BusFrame frames[2];
    CHECK(Bus_Send(frames, Drive_Build((uint8_t)address, (uint8_t)code, 0, 0, 0, 0, 0, 0, frames)));
    Bus_Poll(); Bus_Poll(); CHECK(occupied && sends == 1);
}
static void Fail(uint32_t error)
{
    regs.TSR = 0x1C000009U; regs.ESR = 0x80040U;
    Can_Irq(); board_can.ErrorCode = error; occupied = 0;
    HAL_CAN_ErrorCallback(&board_can);
}
static void Match(void)
{
    Reply(1, 0x35, 0); Bus_Poll(); CHECK(Bus_Status() == CAN_BUSY);
    tick++; Bus_Poll(); CHECK(Bus_Status() == CAN_BUSY);
    tick++; Bus_Poll(); CHECK(Bus_Status() == CAN_SENT && bus_diag.query_replies == 1);
}
static void Capture(void)
{
    uint8_t data[128]; CanCapture frozen; BusFrame frame;
    unsigned n;
    CHECK(Can_Capture(0, data) == 32 && !data[4]);
    CHECK(!Can_Capture(65, data));
    Query(1, 0x35); tick++; Done();
    for (n = 0; n < 80; n++)
    {
        tick++; Reply(1, 0x35, 0); CHECK(Can_Read(&frame));
    }
    Fail(HAL_CAN_ERROR_TX_TERR0);
    CHECK(can_capture.valid && can_capture.count == CAN_HISTORY_SIZE);
    CHECK(can_capture.first[0] == tick && can_capture.first[1] == HAL_CAN_ERROR_TX_TERR0);
    CHECK(can_capture.first[2] == 0x1C000009U && can_capture.first[3] == 0x80040U);
    frozen = can_capture;
    CHECK(Can_Capture(0, data) == 96);
    if (getenv("CAN_DUMP_PAGES"))
    {
        unsigned page, byte, size;
        for (page = 0; page <= CAN_HISTORY_SIZE; page++)
        {
            size = Can_Capture((uint8_t)page, data);
            printf("PAGE %u ", page);
            for (byte = 0; byte < size; byte++) printf("%02x", data[byte]);
            puts("");
        }
    }
    for (n = 1; n <= CAN_HISTORY_SIZE; n++)
    {
        unsigned index = (can_capture.head + n - 1U) % CAN_HISTORY_SIZE;
        CHECK(Can_Capture((uint8_t)n, data) == 72);
        CHECK(can_capture.history[index].sequence == frozen.sequence - CAN_HISTORY_SIZE + n);
        CHECK(data[20] == n);
    }
    tick++; Reply(2, 0x36, 0); Fail(HAL_CAN_ERROR_RX_FOV0);
    Can_Cancel(); Can_Poll();
    CHECK(can_diag.errors == 2 && !memcmp(&frozen, (const void *)&can_capture, sizeof(frozen)));
    CHECK(can_diag.error_tick == tick && frozen.first[0] != tick);
}
static void Active(int omit)
{
    uint32_t due = 0;
    unsigned n, wheel;
    int16_t rpm[4] = {6, 6, 6, 6};
    CHECK(Bus_Start() && Bus_Speed(rpm));
    for (n = 0; n < 1200; n++)
    {
        tick++;
        if (occupied)
        {
            Done();
            if (last.data[0] == 0x35 || last.data[0] == 0x36)
                due = tick + 8U;
        }
        if (!omit && due && tick == due)
        {
            Reply(last.id >> 8, last.data[0], 0);
            due = 0;
        }
        if (bus_diag.active) CHECK(Bus_Speed(rpm));
        Bus_Poll();
        if (!omit && n > 300)
            for (wheel = 0; wheel < 4; wheel++)
            {
                const BusWheel *feedback = Bus_Wheels() + wheel;
                CHECK(feedback->speed.valid && feedback->position.valid);
                CHECK((uint32_t)(tick - feedback->speed.tick) < 250U);
                CHECK((uint32_t)(tick - feedback->position.tick) < 250U);
            }
    }
    if (omit)
        CHECK(bus_diag.locked && (bus_diag.error & BUS_ERROR_REPLY) && stops == 4 && sends == 5);
    else
        CHECK(bus_diag.active && !bus_diag.error && bus_diag.query_replies > 30 && bus_diag.speed_batches > 15);
}
int main(int argc, char **argv)
{
    unsigned which = argc > 1 ? (unsigned)atoi(argv[1]) : 0;
    Can_Refresh(); Bus_Setup(addresses);
    if (which == 0) Capture();
    else if (which == 10 || which == 11) Active(which == 11);
    else
    {
        if (which == 6) tick = UINT32_MAX - 2U;
        Query(1, 0x35);
        if (which == 5)
        {
            Reply(1, 0x35, 0); Bus_Poll(); CHECK(!bus_diag.query_replies);
        }
        tick++; Done(); Bus_Poll();
        CHECK(Bus_Status() == CAN_BUSY && bus_diag.query_waiting && !bus_diag.query_replies);
        if (which == 1 || which == 5 || which == 6) Match();
        else if (which == 2)
        {
            Reply(2, 0x35, 0); Reply(1, 0x36, 0); Reply(1, 0x35, 1);
            Bus_Poll(); CHECK(!bus_diag.query_replies && Bus_Status() == CAN_BUSY);
            Match();
        }
        else if (which == 3 || which == 7)
        {
            tick = BUS_REPLY_TIMEOUT_MS;
            if (which == 7) Reply(1, 0x35, 0);
            Bus_Poll();
            CHECK(Bus_Status() == CAN_FAILED && bus_diag.query_timeouts == 1);
            CHECK(bus_diag.locked && (bus_diag.error & BUS_ERROR_REPLY) && sends == 1);
            tick += 500; Bus_Poll(); CHECK(sends == 1);
        }
        else if (which == 4)
        {
            unsigned n;
            Bus_Stop();
            for (n = 0; n < 30; n++)
            {
                tick++; if (occupied) Done(); Bus_Poll();
            }
            CHECK(stops == 4 && sends == 5 && !bus_diag.pending_stop_count);
            CHECK(!bus_diag.query_timeouts && !bus_diag.query_waiting && bus_diag.locked);
        }
        else if (which == 8)
        {
            Fail(HAL_CAN_ERROR_TX_TERR0); Bus_Poll(); Bus_Poll();
            CHECK(Bus_Status() == CAN_FAILED && (bus_diag.error & BUS_ERROR_CAN));
            CHECK(sends == 1 && can_capture.valid);
        }
        else if (which == 9)
        {
            tick = BUS_REPLY_TIMEOUT_MS - 1; Reply(1, 0x35, 0); Bus_Poll();
            tick += 2; Bus_Poll();
            CHECK(Bus_Status() == CAN_SENT && !bus_diag.query_timeouts);
        }
        else CHECK(0);
    }
    printf("query/capture case %u PASS: offline only\n", which);
    return 0;
}
