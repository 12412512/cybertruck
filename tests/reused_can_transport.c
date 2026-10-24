#include "can_hal_stub.h"
#include "can_internal.h"
#include "bus_config.h"
#include <assert.h>
#include <stdio.h>
#include <string.h>
static TestCanRegisters regs;
CAN_HandleTypeDef board_can={&regs,0};
static uint32_t tick;
static unsigned occupied, sends;
static BusFrame sent[100];
uint32_t HAL_GetTick(void) { return tick; }
uint32_t HAL_CAN_GetTxMailboxesFreeLevel(const CAN_HandleTypeDef *h) { (void)h; return occupied?2U:3U; }
uint32_t HAL_CAN_IsTxMessagePending(const CAN_HandleTypeDef *h,uint32_t boxes) { (void)h; (void)boxes; return occupied; }
HAL_StatusTypeDef HAL_CAN_AbortTxRequest(CAN_HandleTypeDef *h,uint32_t boxes) { (void)h; (void)boxes; occupied=0; return HAL_OK; }
HAL_StatusTypeDef HAL_CAN_AddTxMessage(CAN_HandleTypeDef *h,const CAN_TxHeaderTypeDef *header,const uint8_t *data,uint32_t *mailbox) {
    (void)h; assert(!occupied && sends<100); occupied=1; *mailbox=CAN_TX_MAILBOX0;
    sent[sends].id=header->ExtId; sent[sends].len=(uint8_t)header->DLC;
    memcpy(sent[sends++].data,data,8); return HAL_OK;
}
HAL_StatusTypeDef HAL_CAN_GetRxMessage(CAN_HandleTypeDef *h,uint32_t fifo,CAN_RxHeaderTypeDef *header,uint8_t *data) { (void)h; (void)fifo; (void)header; (void)data; return HAL_ERROR; }
uint32_t HAL_CAN_GetError(const CAN_HandleTypeDef *h) { return h->ErrorCode; }
HAL_StatusTypeDef HAL_CAN_ResetError(CAN_HandleTypeDef *h) { h->ErrorCode=0; return HAL_OK; }
static void lost(uint32_t error) {
    occupied=0; board_can.ErrorCode|=error; HAL_CAN_ErrorCallback(&board_can);
    assert(board_can.ErrorCode==0); /* A prior TERR must not contaminate the next ALST. */
}
static void completed(void) {
    occupied=0; HAL_CAN_TxMailbox0CompleteCallback(&board_can); Can_Poll();
}
static void release(void) { Can_Poll(); assert(!can_diag.tx_isolated); }
int main(void) {
    BusFrame f[2]; unsigned n; uint32_t errors, arb;
    memset(f,0,sizeof(f)); f[0].id=0x100; f[0].len=7; f[0].data[0]=0xF6;
    f[0].data[3]=6; f[0].data[6]=0x6B;
    assert(Can_Send(f,1)); Can_Poll(); assert(sends==1 && occupied);
    errors=can_diag.errors; arb=can_diag.arbitration_lost;
    lost(HAL_CAN_ERROR_TX_ALST0); Can_Poll(); assert(sends==1); /* 1 ms backoff */
    tick++; Can_Poll(); assert(sends==2 && !memcmp(sent[0].data,sent[1].data,8));
    completed(); assert(Can_Status()==CAN_SENT && can_diag.errors==errors && can_diag.arbitration_lost==arb+1);
    assert(can_diag.tx_done_sequence==1);
    assert(can_diag.error_sequence==0); /* Pure ALST is not a real-fault snapshot. */
    tick+=CAN_TX_TURNAROUND_MS;

    /* Cancel between an arbitration event and its retry: only a later STOP may send. */
    assert(Can_Send(f,1)); Can_Poll(); lost(HAL_CAN_ERROR_TX_ALST0); n=sends;
    Can_Cancel(); tick++; release(); assert(sends==n && Can_Status()==CAN_FAILED);
    f[0].len=4; f[0].data[0]=0xFE; f[0].data[1]=0x98; f[0].data[2]=0; f[0].data[3]=0x6B;
    assert(Can_Send(f,1)); Can_Poll(); assert(sends==n); /* Bounded cancel turnaround. */
    tick++; Can_Poll(); assert(sends==n+1 && sent[n].data[0]==0xFE); completed();

    /* Real transmit error and combined errors remain fail-closed, with no replay. */
    tick+=CAN_TX_TURNAROUND_MS;
    assert(Can_Send(f,1)); Can_Poll(); n=sends; errors=can_diag.errors;
    can_diag.irq_tsr=0x1C000009U; can_diag.irq_esr=0x00080030U;
    lost(HAL_CAN_ERROR_TX_TERR0); Can_Poll(); release();
    assert(Can_Status()==CAN_FAILED && sends==n && can_diag.errors==errors+1);
    assert(can_diag.error_tsr==0x1C000009U && can_diag.error_esr==0x00080030U);
    assert(can_diag.error_sequence==errors+1 && can_diag.error_tick==tick);
    assert(can_diag.error_tx_id==0x100 && can_diag.error_tx_length==4);
    assert(can_diag.error_tx_data0==0x6B0098FEU && can_diag.error_tx_pending==1);
    assert(can_diag.error_state==CAN_BUSY && can_diag.error_mailbox==CAN_TX_MAILBOX0);
    can_diag.irq_tsr=0x1C000005U; can_diag.irq_esr=0;
    lost(HAL_CAN_ERROR_TX_ALST0);
    assert(can_diag.errors==errors+1 && can_diag.last_error==HAL_CAN_ERROR_TX_TERR0);
    assert(can_diag.error_tsr==0x1C000009U && can_diag.error_esr==0x00080030U);
    assert(can_diag.error_tx_data0==0x6B0098FEU && can_diag.error_sequence==errors+1);
    tick+=CAN_TX_TURNAROUND_MS;
    assert(Can_Send(f,1)); Can_Poll(); n=sends; errors=can_diag.errors;
    lost(HAL_CAN_ERROR_TX_ALST0|HAL_CAN_ERROR_RX_FOV0); Can_Poll(); release();
    assert(Can_Status()==CAN_FAILED && sends==n && can_diag.errors==errors+1);

    /* Arbitration retries cannot extend the original 50 ms transaction deadline. */
    tick=100; assert(Can_Send(f,1)); Can_Poll(); lost(HAL_CAN_ERROR_TX_ALST0);
    tick=149; Can_Poll(); n=sends; assert(Can_Status()==CAN_BUSY);
    lost(HAL_CAN_ERROR_TX_ALST0); tick=150; Can_Poll(); release();
    assert(Can_Status()==CAN_FAILED && sends==n && can_diag.tx_timeouts==1);

    /* In a two-packet position command only the unsent packet is retried. */
    tick=200; memset(f,0,sizeof(f)); f[0].id=0x100; f[0].len=8; f[0].data[0]=0xFD;
    f[1].id=0x101; f[1].len=5; f[1].data[0]=0xFD; n=sends;
    assert(Can_Send(f,2)); Can_Poll(); completed(); assert(sends==n+1);
    tick++; Can_Poll(); assert(sends==n+1);
    tick++; Can_Poll(); assert(sends==n+2);
    lost(HAL_CAN_ERROR_TX_ALST0); tick++; Can_Poll(); completed();
    assert(sends==n+3 && sent[n].id==0x100 && sent[n+1].id==0x101 && sent[n+2].id==0x101);
    assert(Can_Status()==CAN_SENT);
    /* The 2 ms interval also works across uint32 tick wrap. */
    tick=UINT32_MAX-1U; assert(Can_Send(f,1)); Can_Poll(); completed(); n=sends;
    assert(Can_Send(f,1)); Can_Poll(); assert(sends==n);
    tick++; Can_Poll(); assert(sends==n);
    tick++; Can_Poll(); assert(sends==n+1); completed();
    assert(Can_Status()==CAN_SENT);
    puts("PASS: pure arbitration retry; 1 ms backoff; cancellation discards retry; STOP only after cancel; TERR/combined error no replay; original 50 ms deadline; split-packet TXOK; 2 ms turnaround including cancel/tick wrap; real error snapshot survives later ALST.");
    return 0;
}
