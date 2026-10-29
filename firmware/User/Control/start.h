#ifndef START_H
#define START_H

#include <stdint.h>

enum { START_IDLE, START_RELEASE, START_PRESS, START_BEEP, START_FIRED, START_ABORTED };
enum { START_NO_ERROR, START_NOT_READY, START_IDENTITY, START_TIMEOUT, START_STOP, START_SELECTION };
/* PC1低电平为区1、高电平为区2；转换瞬间失效，稳定50ms后才可准备。 */
typedef struct
{
    uint32_t changed_ms, epoch;
    uint8_t raw, stable, valid;
} Select;
typedef struct
{
    uint64_t boot, session;
    uint32_t ticket, version, phase, prepared_ms, changed_ms, beep_ms;
    uint8_t raw_key, stable_key, beep;
    uint32_t reason, aborted_ms;
    uint32_t zone, select_epoch;
} Start;

void Start_Init(Start *start);
int Start_Prepare(Start *start, uint64_t boot, uint64_t session, uint32_t ticket,
                  uint32_t version, uint32_t now, uint8_t key, uint8_t ready);
void Start_Run(Start *start, uint64_t boot, uint64_t session, uint32_t now,
               uint8_t key, uint8_t ready);
void Start_Cancel(Start *start);
void Select_Init(Select *select, uint32_t now, uint8_t level);
void Select_Run(Select *select, uint32_t now, uint8_t level);
uint32_t Select_Zone(const Select *select);
int Start_Bind(Start *start, const Select *select, uint32_t zone, uint32_t epoch);
void Start_Check(Start *start, const Select *select, uint32_t now);

#endif
