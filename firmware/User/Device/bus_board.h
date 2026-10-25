/* 复用CARCAN13已核对CAN链；本阶段仅改名、归类和增加接收序号。 */
#ifndef CAR_CHASSIS_BOARD_ADAPTER_H
#define CAR_CHASSIS_BOARD_ADAPTER_H
#if defined(CAR_CHASSIS_HOST_TEST)
#include <stdint.h>
uint32_t HAL_GetTick(void);
/* 主机夹具单线程运行；真实板端继续使用CMSIS中断原语。 */
#ifndef __get_PRIMASK
#define __get_PRIMASK() 0U
#endif
#ifndef __disable_irq
#define __disable_irq() ((void)0)
#endif
#ifndef __set_PRIMASK
#define __set_PRIMASK(mask) ((void)(mask))
#endif
#ifndef __DMB
#define __DMB() ((void)0)
#endif
#else
#include "main.h"
#include "can.h"
/* 不复用旧UART、PWM或板级初始化；硬件参数仍归本工程IOC。 */
#define board_can hcan1
#endif
#endif
