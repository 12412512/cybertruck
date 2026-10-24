#ifndef APP_H
#define APP_H

/* 统一只读通信入口；不启动CAN、PWM或运动任务。 */
void App_Init(void);
void App_Run(void);

#endif
