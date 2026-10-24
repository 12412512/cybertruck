#include "app.h"
#include "port.h"
#include "base_port.h"

void App_Init(void)
{
    /* 上电仅启动接收；运动需要新会话、配置、使能和独立目标。 */
    Port_Init();
    Base_Init();
}

void App_Run(void)
{
    Port_Run();
    Base_Run();
}
