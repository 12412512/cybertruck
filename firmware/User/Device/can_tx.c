/* 复用CARCAN13已核对CAN链；本阶段仅改名、归类和增加接收序号。 */
/* 仅复用旧CAN传输，USART1仍由唯一Port管理。 */
#include "can_internal.h"
#include "bus_board.h"
#include "bus_config.h"
#include <string.h>
volatile CanDiag can_diag;
void Can_Irq(void)
{
    can_diag.irq_tsr = board_can.Instance->TSR;
    can_diag.irq_esr = board_can.Instance->ESR;
}
/* ISR只写head，主循环只写tail，DMB后发布索引。索引为对齐32位。 */
static BusFrame rx[CAN_RING_SIZE], tx[2];
static volatile uint32_t rx_head, rx_tail;
static uint8_t tx_count, tx_index, tx_hold, abort_warn;
static volatile uint8_t tx_pending;
static volatile uint32_t tx_mailbox;
static uint32_t tx_tick, abort_tick;
static uint32_t tx_sequence, rx_sequence;
static volatile CanState tx_state;
static volatile uint8_t tx_ok, tx_bad;
static volatile uint8_t tx_arbitration;
static volatile uint32_t arbitration_tick;
static volatile uint32_t turnaround_tick;
static volatile uint8_t turnaround_valid;
#define CAN_TX_ALL (CAN_TX_MAILBOX0 | CAN_TX_MAILBOX1 | CAN_TX_MAILBOX2)
static uint32_t Packed(const uint8_t *bytes)
{
    return (uint32_t)bytes[0] | (uint32_t)bytes[1] << 8 | (uint32_t)bytes[2] << 16 | (uint32_t)bytes[3] << 24;
}
static void CanDiagUpdate(void)
{
    can_diag.free_mailboxes = HAL_CAN_GetTxMailboxesFreeLevel(&board_can);
    can_diag.tx_pending = tx_pending;
    can_diag.tx_index = tx_index;
    can_diag.tx_count = tx_count;
    can_diag.tx_isolated = tx_hold;
    can_diag.esr = board_can.Instance->ESR;
    can_diag.tsr = board_can.Instance->TSR;
    can_diag.btr = board_can.Instance->BTR;
    can_diag.state = (uint32_t)tx_state;
}
void Can_DiagCopy(CanDiag *snapshot)
{
    uint32_t mask = __get_PRIMASK();
    __disable_irq();
    *snapshot = can_diag;
    __set_PRIMASK(mask);
}
uint8_t Can_Send(const BusFrame *frames, uint8_t count)
{
    uint8_t i;
    CanDiagUpdate();
    if (!frames || !count || count > 2 || tx_hold || tx_state == CAN_BUSY || tx_state == CAN_CANCELLING ||
        can_diag.free_mailboxes != 3 || HAL_CAN_IsTxMessagePending(&board_can, CAN_TX_ALL))
        return 0;
    for (i = 0; i < count; i++)
        if (frames[i].len < 2 || frames[i].len > 8 || (frames[i].id >> 8) == 0 || (frames[i].id >> 8) > 255 ||
            (frames[i].id & 255) != i || (i && (frames[i].id >> 8) != (frames[0].id >> 8)))
            return 0;
    memcpy(tx, frames, count * sizeof(*frames));
    tx_count = count;
    tx_index = 0;
    tx_pending = 0;
    tx_mailbox = 0;
    tx_ok = 0;
    tx_bad = 0;
    tx_arbitration = 0;
    tx_tick = HAL_GetTick();
    tx_state = CAN_BUSY;
    can_diag.tx_sequence = ++tx_sequence;
    CanDiagUpdate();
    return 1;
}
static void CanFail(void)
{
    /* 先隔离回调；中止请求不代表邮箱已经释放。 */
    tx_state = CAN_FAILED;
    tx_hold = 1;
    abort_tick = HAL_GetTick();
    abort_warn = 0;
    can_diag.tx_failed++;
    turnaround_tick = HAL_GetTick();
    turnaround_valid = 1;
    if (tx_pending)
        HAL_CAN_AbortTxRequest(&board_can, tx_mailbox);
}
static void CanRelease(uint32_t now)
{
    uint32_t mask = __get_PRIMASK();
    __disable_irq();
    if (HAL_CAN_GetTxMailboxesFreeLevel(&board_can) == 3 &&
        !HAL_CAN_IsTxMessagePending(&board_can, CAN_TX_ALL))
    {
        /* 全部释放后清旧完成标志，再开放新事务；保留失败结果。 */
        __HAL_CAN_CLEAR_FLAG(&board_can, CAN_FLAG_RQCP0);
        __HAL_CAN_CLEAR_FLAG(&board_can, CAN_FLAG_RQCP1);
        __HAL_CAN_CLEAR_FLAG(&board_can, CAN_FLAG_RQCP2);
        tx_pending = 0;
        tx_mailbox = 0;
        tx_ok = 0;
        tx_bad = 0;
        tx_hold = 0;
        if (tx_state == CAN_CANCELLING)
            tx_state = CAN_FAILED;
    }
    __set_PRIMASK(mask);
    if (tx_hold && !abort_warn && (uint32_t)(now - abort_tick) >= CAN_TX_TIMEOUT_MS)
    {
        abort_warn = 1;
        can_diag.cancel_timeouts++;
    }
}
static void CanStep(void)
{
    CAN_TxHeaderTypeDef h = {0};
    uint32_t now = HAL_GetTick(), mailbox, mask;
    HAL_StatusTypeDef result;
    if (tx_hold)
    {
        CanRelease(now);
        return;
    }
    /* 错误检查、分包推进和提交同段屏蔽，错误标志在事务内保持。 */
    mask = __get_PRIMASK();
    __disable_irq();
    if (tx_state != CAN_BUSY)
    {
        __set_PRIMASK(mask);
        return;
    }
    if ((uint32_t)(now - tx_tick) >= CAN_TX_TIMEOUT_MS)
    {
        can_diag.tx_timeouts++;
        CanFail();
        __set_PRIMASK(mask);
        return;
    }
    if (tx_bad)
    {
        CanFail();
        __set_PRIMASK(mask);
        return;
    }
    if (tx_arbitration)
    {
        /* 禁止自动重发时，纯ALST表示未完成本次发送。
         * 仅此情况允许在原50ms期限内重试；取消会在下次发送前丢弃重试。 */
        if ((uint32_t)(now - arbitration_tick) < 1U || HAL_CAN_IsTxMessagePending(&board_can, tx_mailbox))
        {
            __set_PRIMASK(mask);
            return;
        }
        tx_arbitration = 0;
        tx_pending = 0;
        tx_ok = 0;
    }
    if (tx_pending)
    {
        /* 只有当前邮箱的 HAL 完成回调代表 TXOK。 */
        if (!tx_ok)
        {
            __set_PRIMASK(mask);
            return;
        }
        tx_pending = 0;
        ++tx_index;
        if (tx_index == tx_count)
        {
            tx_state = CAN_SENT;
            __set_PRIMASK(mask);
            return;
        }
    }
    if (!HAL_CAN_GetTxMailboxesFreeLevel(&board_can))
    {
        __set_PRIMASK(mask);
        return;
    }
    /* 回复型命令后保留有界周转时间，等待仍计入原事务期限。 */
    if (turnaround_valid && (uint32_t)(now - turnaround_tick) < CAN_TX_TURNAROUND_MS)
    {
        __set_PRIMASK(mask);
        return;
    }
    h.IDE = CAN_ID_EXT;
    h.RTR = CAN_RTR_DATA;
    h.ExtId = tx[tx_index].id;
    h.DLC = tx[tx_index].len;
    tx_ok = 0;
    can_diag.last_tx_id = tx[tx_index].id;
    can_diag.last_tx_length = tx[tx_index].len;
    can_diag.last_tx_data0 = Packed(tx[tx_index].data);
    can_diag.last_tx_data1 = Packed(tx[tx_index].data + 4);
    result = HAL_CAN_AddTxMessage(&board_can, &h, tx[tx_index].data, &mailbox);
    if (result == HAL_OK)
    {
        tx_mailbox = mailbox;
        tx_pending = 1;
    }
    else
        CanFail();
    __set_PRIMASK(mask);
}
void Can_Poll(void)
{
    CanStep();
    CanDiagUpdate();
}
void Can_Refresh(void)
{
    CanDiagUpdate();
}
CanState Can_Status(void)
{
    return tx_state;
}
void Can_Cancel(void)
{
    /* 先丢软件分包并隔离回调；重复取消不重置中止计时。 */
    tx_count = 0;
    tx_state = CAN_CANCELLING;
    tx_arbitration = 0;
    if (!tx_hold)
    {
        tx_hold = 1;
        abort_tick = HAL_GetTick();
        abort_warn = 0;
        turnaround_tick = abort_tick;
        turnaround_valid = 1;
    }
    HAL_CAN_AbortTxRequest(&board_can, CAN_TX_ALL);
    CanDiagUpdate();
}
void Can_Drop(void)
{
    uint32_t mask = __get_PRIMASK();
    __disable_irq();
    rx_tail = rx_head;
    __DMB();
    __set_PRIMASK(mask);
}
uint8_t Can_Read(BusFrame *frame)
{
    uint32_t tail = rx_tail;
    if (!frame || tail == rx_head)
        return 0;
    __DMB();
    *frame = rx[tail];
    __DMB();
    rx_tail = (tail + 1) % CAN_RING_SIZE;
    return 1;
}
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *h)
{
    CAN_RxHeaderTypeDef header;
    BusFrame f;
    uint32_t next;
    if (h != &board_can)
        return;
    if (HAL_CAN_GetRxMessage(h, CAN_RX_FIFO0, &header, f.data) != HAL_OK)
    {
        can_diag.errors++;
        return;
    }
    if (header.IDE != CAN_ID_EXT || header.RTR != CAN_RTR_DATA || header.DLC < 2 || header.DLC > 8 ||
        (header.ExtId >> 8) == 0 || (header.ExtId >> 8) > 255)
    {
        can_diag.rx_invalid++;
        return;
    }
    f.id = header.ExtId;
    f.len = (uint8_t)header.DLC;
    f.tick = HAL_GetTick();
    can_diag.last_rx_id = f.id;
    can_diag.last_rx_length = f.len;
    can_diag.last_rx_data0 = Packed(f.data);
    can_diag.last_rx_data1 = Packed(f.data + 4);
    can_diag.last_rx_tick = f.tick;
    /* 环满也推进序号，诊断缺号不能冒充设备丢帧；原溢出处理不变。 */
    f.rx_sequence = ++rx_sequence;
    can_diag.rx_sequence = rx_sequence;
    next = (rx_head + 1) % CAN_RING_SIZE;
    if (next == rx_tail)
    {
        can_diag.rx_overflow++;
        return;
    }
    rx[rx_head] = f;
    __DMB();
    rx_head = next;
}
void HAL_CAN_ErrorCallback(CAN_HandleTypeDef *h)
{
    if (h == &board_can)
    {
        uint32_t error = HAL_CAN_GetError(h);
        const uint32_t arbitration_mask =
            HAL_CAN_ERROR_TX_ALST0 | HAL_CAN_ERROR_TX_ALST1 | HAL_CAN_ERROR_TX_ALST2;
        if (error && !(error & ~arbitration_mask))
        {
            can_diag.arbitration_lost++;
            if (tx_state == CAN_BUSY && tx_pending &&
                ((tx_mailbox == CAN_TX_MAILBOX0 && error == HAL_CAN_ERROR_TX_ALST0) ||
                 (tx_mailbox == CAN_TX_MAILBOX1 && error == HAL_CAN_ERROR_TX_ALST1) ||
                 (tx_mailbox == CAN_TX_MAILBOX2 && error == HAL_CAN_ERROR_TX_ALST2)))
            {
                arbitration_tick = HAL_GetTick();
                tx_arbitration = 1;
            }
        }
        else
        {
            can_diag.errors++;
            can_diag.last_error = error;
            can_diag.error_sequence = can_diag.errors;
            can_diag.error_tick = HAL_GetTick();
            can_diag.error_tsr = can_diag.irq_tsr;
            can_diag.error_esr = can_diag.irq_esr;
            can_diag.error_tx_id = can_diag.last_tx_id;
            can_diag.error_tx_length = can_diag.last_tx_length;
            can_diag.error_tx_data0 = can_diag.last_tx_data0;
            can_diag.error_tx_data1 = can_diag.last_tx_data1;
            can_diag.error_state = (uint32_t)tx_state;
            can_diag.error_mailbox = tx_mailbox;
            can_diag.error_tx_pending = tx_pending;
            can_diag.error_tx_index = tx_index;
            can_diag.error_tx_count = tx_count;
            if (tx_state == CAN_BUSY)
                tx_bad = 1;
        }
        /* 记录本次事件后清HAL累积标志，避免旧错误污染后续纯仲裁丢失。
         * 自有错误计数和冻结现场始终保留，不自动解除执行故障。 */
        (void)HAL_CAN_ResetError(h);
    }
}
static void CanTxDone(CAN_HandleTypeDef *h, uint32_t mailbox, uint8_t bad)
{
    if (h != &board_can || tx_state != CAN_BUSY || !tx_pending || tx_mailbox != mailbox)
        return;
    if (bad)
        tx_bad = 1;
    else
    {
        turnaround_tick = HAL_GetTick();
        turnaround_valid = 1;
        if (!tx_ok && tx_index + 1U == tx_count)
        {
            can_diag.tx_done_tick = HAL_GetTick();
            can_diag.tx_done_sequence = tx_sequence;
        }
        tx_ok = 1;
    }
}
void HAL_CAN_TxMailbox0CompleteCallback(CAN_HandleTypeDef *h)
{
    CanTxDone(h, CAN_TX_MAILBOX0, 0);
}
void HAL_CAN_TxMailbox1CompleteCallback(CAN_HandleTypeDef *h)
{
    CanTxDone(h, CAN_TX_MAILBOX1, 0);
}
void HAL_CAN_TxMailbox2CompleteCallback(CAN_HandleTypeDef *h)
{
    CanTxDone(h, CAN_TX_MAILBOX2, 0);
}
void HAL_CAN_TxMailbox0AbortCallback(CAN_HandleTypeDef *h)
{
    CanTxDone(h, CAN_TX_MAILBOX0, 1);
}
void HAL_CAN_TxMailbox1AbortCallback(CAN_HandleTypeDef *h)
{
    CanTxDone(h, CAN_TX_MAILBOX1, 1);
}
void HAL_CAN_TxMailbox2AbortCallback(CAN_HandleTypeDef *h)
{
    CanTxDone(h, CAN_TX_MAILBOX2, 1);
}
