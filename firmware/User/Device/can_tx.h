/* 复用CARCAN13已核对CAN链；本阶段仅改名、归类和增加接收序号。 */
#ifndef TRANSPORT_H
#define TRANSPORT_H
#include "drive.h"
typedef enum
{
    CAN_IDLE,
    CAN_BUSY,
    CAN_SENT,
    CAN_FAILED,
    CAN_CANCELLING
} CanState;
typedef struct
{
    uint32_t rx_overflow, rx_invalid, errors, tx_failed, last_error;
    uint32_t free_mailboxes, tx_timeouts, cancel_timeouts;
    /* 只读本机时序：TX序号在CanSend接受时推进；done仅当前事务末帧成功回调更新。
     * done_tick是MCU观察回调的HAL毫秒，不代表驱动执行或设备采样时刻。 */
    uint32_t tx_sequence, tx_done_sequence, tx_done_tick, rx_sequence;
    uint8_t tx_pending, tx_index, tx_count, tx_isolated;
    /* 只读实发证据；帧字节按小端字打包。 */
    uint32_t irq_tsr, irq_esr, error_tsr, error_esr;
    /* 真实错误现场一起冻结，后续仲裁/TX/RX不得覆盖。 */
    uint32_t error_sequence, error_tick, error_tx_id, error_tx_length;
    uint32_t error_tx_data0, error_tx_data1, error_state, error_mailbox;
    uint32_t error_tx_pending, error_tx_index, error_tx_count;
    uint32_t esr, tsr, btr, state;
    uint32_t last_tx_id, last_tx_length, last_tx_data0, last_tx_data1;
    uint32_t last_rx_id, last_rx_length, last_rx_data0, last_rx_data1, last_rx_tick;
    uint32_t arbitration_lost;
} CanDiag;
extern volatile CanDiag can_diag;
/* 首次真实CAN错误冻结此前64个事件；启动后只写一次，不随清故障丢弃。
 * 时间为本机回调时刻，不是总线SOF或设备采样时间。 */
#define CAN_HISTORY_SIZE 64U
enum
{
    CAN_EVENT_TX = 1, CAN_EVENT_RX, CAN_EVENT_DONE, CAN_EVENT_ARBITRATION,
    CAN_EVENT_ERROR, CAN_EVENT_CANCEL, CAN_EVENT_TIMEOUT
};
typedef struct
{
    uint32_t sequence, tick, kind, id, length, data0, data1, tx_sequence, tsr, esr;
} CanEvent;
typedef struct
{
    uint32_t valid, head, count, sequence;
    uint32_t first[16];
    CanEvent history[CAN_HISTORY_SIZE];
} CanCapture;
extern volatile CanCapture can_capture;
/* 只读分页：0=首次错误，1..count=由早到晚事件；无记录返回版本头。
 * 输出最多96字节；不推进CAN、不清锁存；非法页返回0。 */
uint16_t Can_Capture(uint8_t page, uint8_t *output);
void Can_Irq(void);

#endif
