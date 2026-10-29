#ifndef EXECUTE_H
#define EXECUTE_H

#include "exec.h"

/* 单一线协议实现，由板端与主机故障注入共用；不直接访问总线。 */
uint8_t Execute_Command(Exec *exec, uint8_t type, const uint8_t *data, uint16_t size, uint8_t *reply,
                        uint16_t *reply_size, uint32_t now, uint32_t can_errors, uint32_t can_rx_sequence);

#endif
