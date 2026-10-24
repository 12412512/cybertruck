#ifndef PORT_H
#define PORT_H

#include "imu.h"

typedef struct
{
    uint32_t rx_bytes, overflows, errors, restart_errors;
    uint32_t first_error_code, first_error_ms, last_error_code, last_error_ms;
    uint32_t startup_flags;
} PortDiag;

void Port_Init(void);
void Port_Run(void);
int Port_Imu(ImuSample *sample);
uint16_t Port_Diag(uint8_t *data);

#endif
