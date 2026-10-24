#ifndef TEST_MECH_TIM_H
#define TEST_MECH_TIM_H
#include <stdint.h>
#define TIM_CHANNEL_1 0U
#define TIM_CHANNEL_2 4U
#define TIM_CHANNEL_3 8U
#define HAL_OK 0U
extern int htim1;
uint32_t HAL_GetTick(void);
uint32_t HAL_TIM_PWM_Start(int *timer, uint32_t channel);
void Test_Compare(uint32_t channel, uint32_t pulse);
#define __HAL_TIM_SET_COMPARE(timer, channel, pulse) Test_Compare(channel, pulse)
#endif
