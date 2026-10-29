/* 复用CARCAN13已核对CAN链；本阶段仅改名、归类和增加接收序号。 */
#ifndef TRANSPORT_INTERNAL_H
#define TRANSPORT_INTERNAL_H
#include "can_tx.h"
/* 私有CAN事务原语；只有Bus发送和消费接收，ISR只搬运。 */
void Can_Poll(void);
/* 只刷新邮箱诊断，不推进发送，不提交CAN帧。 */
void Can_Refresh(void);
uint8_t Can_Send(const BusFrame *frames, uint8_t count);
CanState Can_Status(void);
uint8_t Can_Read(BusFrame *frame);
void Can_Cancel(void);
void Can_Drop(void);
void Can_DiagCopy(CanDiag *snapshot);
#endif
