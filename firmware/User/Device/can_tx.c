/* 复用CARCAN13已核对CAN链；本阶段仅改名、归类和增加接收序号。 */
/* 仅复用旧CAN传输，USART1仍由唯一Port管理。 */
#include "can_internal.h"
#include "bus_board.h"
#include "bus_config.h"
#include <string.h>
volatile CanDiag can_diag;
volatile CanCapture can_capture;
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
static void History(uint32_t kind, const BusFrame *frame)
{
    uint32_t mask = __get_PRIMASK();
    volatile CanEvent *event;
    __disable_irq();
    if (!can_capture.valid)
    {
        event = &can_capture.history[can_capture.head];
        event->sequence = ++can_capture.sequence;
        event->tick = HAL_GetTick();
        event->kind = kind;
        event->id = frame ? frame->id : can_diag.last_tx_id;
        event->length = frame ? frame->len : can_diag.last_tx_length;
        event->data0 = frame ? Packed(frame->data) : can_diag.last_tx_data0;
        event->data1 = frame ? Packed(frame->data + 4) : can_diag.last_tx_data1;
        event->tx_sequence = tx_sequence;
        event->tsr = kind == CAN_EVENT_ERROR ? can_diag.irq_tsr : board_can.Instance->TSR;
        event->esr = kind == CAN_EVENT_ERROR ? can_diag.irq_esr : board_can.Instance->ESR;
        can_capture.head = (can_capture.head + 1U) % CAN_HISTORY_SIZE;
        if (can_capture.count < CAN_HISTORY_SIZE)
            can_capture.count++;
    }
    __set_PRIMASK(mask);
}
static void FirstError(uint32_t error)
{
    uint32_t mask = __get_PRIMASK();
    __disable_irq();
    History(CAN_EVENT_ERROR, 0);
    if (!can_capture.valid)
    {
        can_capture.first[0] = HAL_GetTick();
        can_capture.first[1] = error;
        can_capture.first[2] = can_diag.irq_tsr;
        can_capture.first[3] = can_diag.irq_esr;
        can_capture.first[4] = tx_sequence;
        can_capture.first[5] = rx_sequence;
        can_capture.first[6] = can_diag.last_tx_id;
        can_capture.first[7] = can_diag.last_tx_length;
        can_capture.first[8] = can_diag.last_tx_data0;
        can_capture.first[9] = can_diag.last_tx_data1;
        can_capture.first[10] = (uint32_t)tx_state;
        can_capture.first[11] = tx_mailbox;
        can_capture.first[12] = tx_pending;
        can_capture.first[13] = tx_index;
        can_capture.first[14] = tx_count;
        can_capture.first[15] = can_diag.errors;
        __DMB();
        can_capture.valid = 1;
    }
    __set_PRIMASK(mask);
}
uint16_t Can_Capture(uint8_t page, uint8_t *output)
{
    uint32_t words[24] = {1U}, count, index, mask;
    unsigned size = 8, byte;
    if (!output || page > CAN_HISTORY_SIZE)
        return 0;
    mask = __get_PRIMASK();
    __disable_irq();
    count = can_capture.valid ? can_capture.count : 0;
    words[1] = can_capture.valid;
    words[2] = can_capture.sequence;
    words[3] = count;
    words[4] = CAN_HISTORY_SIZE;
    words[5] = page;
    words[6] = sizeof(CanEvent);
    if (page && page > count)
    {
        __set_PRIMASK(mask);
        return 0;
    }
    if (!page && can_capture.valid)
    {
        for (index = 0; index < 16; index++)
            words[8 + index] = can_capture.first[index];
        size = 24;
    }
    else if (page)
    {
        CanEvent event;
        index = (can_capture.head + CAN_HISTORY_SIZE - count + page - 1U) % CAN_HISTORY_SIZE;
        event = can_capture.history[index];
        memcpy(words + 8, &event, sizeof(event));
        size += sizeof(event) / sizeof(uint32_t);
    }
    __set_PRIMASK(mask);
    for (byte = 0; byte < size * 4U; byte++)
        output[byte] = (uint8_t)(words[byte / 4U] >> (8U * (byte % 4U)));
    return (uint16_t)(size * 4U);
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
    uint32_t now, mailbox, mask;
    HAL_StatusTypeDef result;
    if (tx_hold)
    {
        CanRelease(HAL_GetTick());
        return;
    }
    /* 错误检查、分包推进和提交同段屏蔽，错误标志在事务内保持。 */
    mask = __get_PRIMASK();
    __disable_irq();
    /* 取时必须晚于已完成的回调，避免跨毫秒后相减下溢绕过周转/退避。 */
    now = HAL_GetTick();
    if (tx_state != CAN_BUSY)
    {
        __set_PRIMASK(mask);
        return;
    }
    if ((uint32_t)(now - tx_tick) >= CAN_TX_TIMEOUT_MS)
    {
        can_diag.tx_timeouts++;
        History(CAN_EVENT_TIMEOUT, 0);
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
    /* 查询可能收到多个回包；非STOP需等最近RX后的静默周转。
     * 等待仍计入原50ms期限，故障STOP不受额外RX静默门槛阻塞。 */
    if (tx[tx_index].data[0] != 0xFE &&
        ((board_can.Instance->RF0R & 3U) ||
         (can_diag.rx_sequence && (uint32_t)(now - can_diag.last_rx_tick) < CAN_TX_TURNAROUND_MS)))
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
        History(CAN_EVENT_TX, &tx[tx_index]);
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
    History(CAN_EVENT_CANCEL, 0);
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
static uint32_t ReplyTransaction(void)
{
    uint32_t shift, result;
    if (tx_state == CAN_SENT && !tx_hold && !tx_bad &&
        can_diag.tx_done_sequence == tx_sequence)
        return tx_sequence;
    if (tx_state != CAN_BUSY || !tx_pending || tx_bad || tx_arbitration)
        return 0;
    if (tx_ok)
        return tx_sequence;
    /* RX中断可能先于TX完成中断；只读当前邮箱真实RQCP/TXOK，不能仅凭邮箱空闲。
     * 每个邮箱结果占8位：RQCP=bit0、TXOK=bit1、ALST=bit2、TERR=bit3。
     * 不清标志、不推进事务；最终完成仍必须经过正式HAL回调。 */
    if (tx_mailbox == CAN_TX_MAILBOX0)
        shift = 0;
    else if (tx_mailbox == CAN_TX_MAILBOX1)
        shift = 8;
    else if (tx_mailbox == CAN_TX_MAILBOX2)
        shift = 16;
    else
        return 0;
    result = (board_can.Instance->TSR >> shift) & 15U;
    return result == 3U ? tx_sequence : 0;
}
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *h)
{
    CAN_RxHeaderTypeDef header;
    BusFrame f = {0};
    uint32_t next;
    if (h != &board_can)
        return;
    if (HAL_CAN_GetRxMessage(h, CAN_RX_FIFO0, &header, f.data) != HAL_OK)
    {
        can_diag.errors++;
        FirstError(HAL_CAN_GetError(h));
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
    f.tx_sequence = ReplyTransaction();
    can_diag.last_rx_id = f.id;
    can_diag.last_rx_length = f.len;
    can_diag.last_rx_data0 = Packed(f.data);
    can_diag.last_rx_data1 = Packed(f.data + 4);
    can_diag.last_rx_tick = f.tick;
    /* 环满也推进序号，诊断缺号不能冒充设备丢帧；原溢出处理不变。 */
    f.rx_sequence = ++rx_sequence;
    can_diag.rx_sequence = rx_sequence;
    History(CAN_EVENT_RX, &f);
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
            History(CAN_EVENT_ARBITRATION, 0);
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
            FirstError(error);
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
    /* 错误已锁定时，迟到完成回调不得发布成功序号或DONE历史。 */
    if (h != &board_can || tx_state != CAN_BUSY || !tx_pending || tx_mailbox != mailbox || tx_bad)
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
        History(CAN_EVENT_DONE, 0);
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
