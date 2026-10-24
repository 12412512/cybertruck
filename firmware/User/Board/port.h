#ifndef PORT_H
#define PORT_H

#include "imu.h"

void Port_Init(void);
void Port_Run(void);
int Port_Imu(ImuSample *sample);

#endif
