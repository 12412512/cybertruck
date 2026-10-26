/* 底盘板级启动夹具；替代外设IO，正式业务与CAN状态机仍使用原源码。 */
#ifndef BASE_HAL_STUB_H
#define BASE_HAL_STUB_H
#include "can_hal_stub.h"
#define __CAN_H__
#define __GPIO_H__
#define hcan1 board_can
typedef struct
{
    uint32_t FilterMode, FilterScale, FilterIdLow, FilterMaskIdLow;
    uint32_t FilterFIFOAssignment, FilterActivation, SlaveStartFilterBank;
} CAN_FilterTypeDef;
#define CAN_FILTERMODE_IDMASK 0U
#define CAN_FILTERSCALE_32BIT 1U
#define CAN_RTR_REMOTE 2U
#define ENABLE 1U
#define CAN_IT_RX_FIFO0_MSG_PENDING 2U
#define CAN_IT_RX_FIFO0_OVERRUN 8U
#define CAN_IT_TX_MAILBOX_EMPTY 1U
#define CAN_IT_ERROR 0x8000U
#define CAN_IT_BUSOFF 0x400U
#define GPIO_PIN_RESET 0U
#define GPIO_PIN_SET 1U
#define START_KEY_GPIO_Port ((void *)1)
#define ZONE_SWITCH_GPIO_Port ((void *)2)
#define BUZZER_GPIO_Port ((void *)3)
#define START_KEY_Pin 1U
#define ZONE_SWITCH_Pin 2U
#define BUZZER_Pin 4U
HAL_StatusTypeDef HAL_CAN_ConfigFilter(CAN_HandleTypeDef *can, const CAN_FilterTypeDef *filter);
HAL_StatusTypeDef HAL_CAN_Start(CAN_HandleTypeDef *can);
HAL_StatusTypeDef HAL_CAN_ActivateNotification(CAN_HandleTypeDef *can, uint32_t flags);
uint32_t HAL_GPIO_ReadPin(void *port, uint32_t pin);
void HAL_GPIO_WritePin(void *port, uint32_t pin, uint32_t state);
void NVIC_SystemReset(void);
#endif
