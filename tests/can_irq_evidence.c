/* 执行正式传输和原版HAL中断函数，只模拟寄存器，不连接设备。 */
#include "can_irq_stub.h"
#include "can_internal.h"
#include "bus_config.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#define REQUIRE(condition) do { if (!(condition)) { fprintf(stderr,"line %d: %s\n",__LINE__,#condition); exit(1); } } while (0)

static TestCanRegisters regs;
CAN_HandleTypeDef board_can = {&regs, 0};
static uint32_t tick;
static unsigned occupied, sends;

uint32_t HAL_GetTick(void) { return tick; }
uint32_t HAL_CAN_GetTxMailboxesFreeLevel(const CAN_HandleTypeDef *h) { (void)h; return occupied ? 2U : 3U; }
uint32_t HAL_CAN_IsTxMessagePending(const CAN_HandleTypeDef *h, uint32_t boxes) { (void)h; (void)boxes; return occupied; }
HAL_StatusTypeDef HAL_CAN_AbortTxRequest(CAN_HandleTypeDef *h, uint32_t boxes) { (void)h; (void)boxes; occupied=0; return HAL_OK; }
HAL_StatusTypeDef HAL_CAN_AddTxMessage(CAN_HandleTypeDef *h,const CAN_TxHeaderTypeDef *header,const uint8_t *data,uint32_t *mailbox)
{
    (void)h; REQUIRE(!occupied); REQUIRE(header->ExtId==0x300 && header->DLC==2);
    REQUIRE(data[0]==0x3A && data[1]==0x6B); occupied=1; sends++; *mailbox=CAN_TX_MAILBOX0; return HAL_OK;
}
HAL_StatusTypeDef HAL_CAN_GetRxMessage(CAN_HandleTypeDef *h,uint32_t fifo,CAN_RxHeaderTypeDef *header,uint8_t *data)
{ (void)h; (void)fifo; (void)header; (void)data; return HAL_ERROR; }
uint32_t HAL_CAN_GetError(const CAN_HandleTypeDef *h) { return h->ErrorCode; }
HAL_StatusTypeDef HAL_CAN_ResetError(CAN_HandleTypeDef *h) { h->ErrorCode=0; return HAL_OK; }
void Test_ClearFlag(CAN_HandleTypeDef *h,uint32_t flag)
{
    if (flag==CAN_FLAG_RQCP0) h->Instance->TSR &= ~0xFU;
    else if (flag==CAN_FLAG_RQCP1) h->Instance->TSR &= ~0xF00U;
    else if (flag==CAN_FLAG_RQCP2) h->Instance->TSR &= ~0xF0000U;
    else if (flag==CAN_FLAG_ERRI) h->Instance->MSR &= ~CAN_MSR_ERRI;
}
void HAL_CAN_RxFifo0FullCallback(CAN_HandleTypeDef *h) { (void)h; }
void HAL_CAN_RxFifo1FullCallback(CAN_HandleTypeDef *h) { (void)h; }
void HAL_CAN_RxFifo1MsgPendingCallback(CAN_HandleTypeDef *h) { (void)h; }
void HAL_CAN_SleepCallback(CAN_HandleTypeDef *h) { (void)h; }
void HAL_CAN_WakeUpFromRxMsgCallback(CAN_HandleTypeDef *h) { (void)h; }

static void Start(void)
{
    BusFrame frame;
    memset(&frame,0,sizeof(frame)); frame.id=0x300; frame.len=2; frame.data[0]=0x3A; frame.data[1]=0x6B;
    REQUIRE(Can_Send(&frame,1)); Can_Poll(); REQUIRE(occupied);
}
static void Irq(uint32_t tsr,uint32_t esr)
{
    regs.TSR=tsr; regs.ESR=esr; regs.MSR=CAN_MSR_ERRI; occupied=0;
    Can_Irq(); HAL_CAN_IRQHandler(&board_can); Can_Poll();
}
static void Released(void) { Can_Poll(); REQUIRE(!can_diag.tx_isolated); }

int main(int argc,char **argv)
{
    unsigned which=argc>1?(unsigned)atoi(argv[1]):0, previous;
    regs.IER=CAN_IT_TX_MAILBOX_EMPTY|CAN_IT_ERROR|CAN_IT_BUSOFF;
    if (which==1) regs.IER|=CAN_IT_LAST_ERROR_CODE;
    Start(); previous=sends;
    if (which==2)
    {
        Irq(0x1C000005U,0); REQUIRE(!can_diag.errors && can_diag.arbitration_lost==1);
        tick=1; Can_Poll(); REQUIRE(sends==previous+1 && Can_Status()==CAN_BUSY);
    }
    else if (which==3)
    {
        regs.ESR=0x80040; regs.MSR=CAN_MSR_ERRI;
        Can_Irq(); HAL_CAN_IRQHandler(&board_can); Can_Poll();
        REQUIRE(!can_diag.errors && Can_Status()==CAN_BUSY);
        tick=50; Can_Poll(); REQUIRE(can_diag.tx_timeouts==1 && Can_Status()==CAN_FAILED);
    }
    else if (which==4)
    {
        Irq(0x1C000003U,0x80040); REQUIRE(!can_diag.errors && Can_Status()==CAN_SENT);
    }
    else
    {
        Irq(0x1C000009U,0x80040); Released();
        REQUIRE(can_diag.errors==1 && can_diag.tx_failed==1 && Can_Status()==CAN_FAILED);
        REQUIRE(can_diag.error_tsr==0x1C000009U && can_diag.error_esr==0x80040);
        REQUIRE(can_diag.error_tx_id==0x300 && can_diag.error_tx_data0==0x6B3A && can_diag.error_tx_pending==1);
        REQUIRE(can_diag.last_error==(which==1?0x1040U:0x1000U));
        REQUIRE(sends==previous && board_can.ErrorCode==0);
        tick=2; Start(); previous=sends;
        Irq(0x1C000005U,0); REQUIRE(can_diag.errors==1 && can_diag.arbitration_lost==1);
        tick=3; Can_Poll(); REQUIRE(sends==previous+1 && Can_Status()==CAN_BUSY);
        Irq(0x1C000003U,0); REQUIRE(Can_Status()==CAN_SENT && can_diag.errors==1);
        REQUIRE(can_diag.error_tsr==0x1C000009U && can_diag.error_tx_data0==0x6B3A);
    }
    printf("HAL IRQ case %u PASS: simulation only\n",which);
    return 0;
}
