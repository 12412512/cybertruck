#ifndef START_H
#define START_H

#include <stdint.h>

enum { START_IDLE, START_RELEASE, START_PRESS, START_BEEP, START_FIRED, START_ABORTED };
enum { START_NO_ERROR, START_NOT_READY, START_IDENTITY, START_TIMEOUT, START_STOP };
typedef struct
{
    uint64_t boot, session;
    uint32_t ticket, version, phase, prepared_ms, changed_ms, beep_ms;
    uint8_t raw_key, stable_key, beep;
    uint32_t reason, aborted_ms;
} Start;

void Start_Init(Start *start);
int Start_Prepare(Start *start, uint64_t boot, uint64_t session, uint32_t ticket,
                  uint32_t version, uint32_t now, uint8_t key, uint8_t ready);
void Start_Run(Start *start, uint64_t boot, uint64_t session, uint32_t now,
               uint8_t key, uint8_t ready);
void Start_Cancel(Start *start);

#endif
