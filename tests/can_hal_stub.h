#ifndef CAN_HAL_TEST_STUB_H
#define CAN_HAL_TEST_STUB_H
#define CAR_CHASSIS_BOARD_ADAPTER_H
#include <stdint.h>
typedef struct { uint32_t ESR,TSR,BTR,IER,MSR,RF0R,RF1R; } TestCanRegisters;
typedef struct { TestCanRegisters *Instance; uint32_t ErrorCode; } CAN_HandleTypeDef;
typedef struct { uint32_t IDE,RTR,ExtId,DLC; } CAN_TxHeaderTypeDef;
typedef CAN_TxHeaderTypeDef CAN_RxHeaderTypeDef;
typedef enum { HAL_OK, HAL_ERROR } HAL_StatusTypeDef;
extern CAN_HandleTypeDef board_can;
#define CAN_TX_MAILBOX0 1U
#define CAN_TX_MAILBOX1 2U
#define CAN_TX_MAILBOX2 4U
#define CAN_ID_EXT 4U
#define CAN_RTR_DATA 0U
#define CAN_RX_FIFO0 0U
#define CAN_FLAG_RQCP0 0U
#define CAN_FLAG_RQCP1 1U
#define CAN_FLAG_RQCP2 2U
#define HAL_CAN_ERROR_TX_ALST0 0x0800U
#define HAL_CAN_ERROR_TX_ALST1 0x2000U
#define HAL_CAN_ERROR_TX_ALST2 0x8000U
#define HAL_CAN_ERROR_TX_TERR0 0x1000U
#define HAL_CAN_ERROR_RX_FOV0 0x0200U
#define __get_PRIMASK() 0U
#define __disable_irq() ((void)0)
#define __set_PRIMASK(x) ((void)(x))
#define __DMB() ((void)0)
#define __HAL_CAN_CLEAR_FLAG(h,f) ((void)(h),(void)(f))
uint32_t HAL_GetTick(void);
uint32_t HAL_CAN_GetTxMailboxesFreeLevel(const CAN_HandleTypeDef *h);
uint32_t HAL_CAN_IsTxMessagePending(const CAN_HandleTypeDef *h,uint32_t boxes);
HAL_StatusTypeDef HAL_CAN_AbortTxRequest(CAN_HandleTypeDef *h,uint32_t boxes);
HAL_StatusTypeDef HAL_CAN_AddTxMessage(CAN_HandleTypeDef *h,const CAN_TxHeaderTypeDef *header,const uint8_t *data,uint32_t *mailbox);
HAL_StatusTypeDef HAL_CAN_GetRxMessage(CAN_HandleTypeDef *h,uint32_t fifo,CAN_RxHeaderTypeDef *header,uint8_t *data);
uint32_t HAL_CAN_GetError(const CAN_HandleTypeDef *h);
HAL_StatusTypeDef HAL_CAN_ResetError(CAN_HandleTypeDef *h);
void HAL_CAN_ErrorCallback(CAN_HandleTypeDef *h);
void HAL_CAN_TxMailbox0CompleteCallback(CAN_HandleTypeDef *h);
#endif
