#ifndef CAN_IRQ_TEST_STUB_H
#define CAN_IRQ_TEST_STUB_H
#include "can_hal_stub.h"
#include "can_irq_constants.h"
#define USE_HAL_CAN_REGISTER_CALLBACKS 0
#define READ_REG(value) (value)
#define CLEAR_BIT(value, mask) ((value) &= ~(mask))
#undef __HAL_CAN_CLEAR_FLAG
#define __HAL_CAN_CLEAR_FLAG(handle, flag) Test_ClearFlag(handle, flag)
void Test_ClearFlag(CAN_HandleTypeDef *handle, uint32_t flag);
void HAL_CAN_IRQHandler(CAN_HandleTypeDef *handle);
void HAL_CAN_TxMailbox1CompleteCallback(CAN_HandleTypeDef *handle);
void HAL_CAN_TxMailbox2CompleteCallback(CAN_HandleTypeDef *handle);
void HAL_CAN_TxMailbox0AbortCallback(CAN_HandleTypeDef *handle);
void HAL_CAN_TxMailbox1AbortCallback(CAN_HandleTypeDef *handle);
void HAL_CAN_TxMailbox2AbortCallback(CAN_HandleTypeDef *handle);
void HAL_CAN_RxFifo0FullCallback(CAN_HandleTypeDef *handle);
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *handle);
void HAL_CAN_RxFifo1FullCallback(CAN_HandleTypeDef *handle);
void HAL_CAN_RxFifo1MsgPendingCallback(CAN_HandleTypeDef *handle);
void HAL_CAN_SleepCallback(CAN_HandleTypeDef *handle);
void HAL_CAN_WakeUpFromRxMsgCallback(CAN_HandleTypeDef *handle);
#endif
