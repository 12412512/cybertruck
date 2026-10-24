#include "app.h"
#include "port.h"

void App_Init(void)
{
    /* 只启用USART1只读协议；CAN、PWM和运动入口均不启动。 */
    Port_Init();
}

void App_Run(void)
{
    Port_Run();
}
