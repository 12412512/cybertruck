#include "start.h"
#include <string.h>

void Start_Init(Start *start)
{
    memset(start, 0, sizeof(*start));
}

void Start_Cancel(Start *start)
{
    if (start->ticket)
    {
        start->phase = START_ABORTED;
        if (!start->reason)
            start->reason = START_STOP;
    }
    start->beep = 0;
}

int Start_Prepare(Start *start, uint64_t boot, uint64_t session, uint32_t ticket,
                  uint32_t version, uint32_t now, uint8_t key, uint8_t ready)
{
    if (!boot || !session || !ticket || !version || !ready)
        return 0;
    if (start->ticket && start->boot == boot && start->session == session)
        return start->ticket == ticket && start->version == version && start->phase != START_ABORTED;
    Start_Init(start);
    start->boot = boot;
    start->session = session;
    start->ticket = ticket;
    start->version = version;
    start->prepared_ms = start->changed_ms = now;
    start->raw_key = start->stable_key = (uint8_t)!!key;
    /* 每次准备均先等待稳定松开，开机/准备时按住不能产生启动事件。 */
    start->phase = START_RELEASE;
    return 1;
}

void Start_Run(Start *start, uint64_t boot, uint64_t session, uint32_t now,
               uint8_t key, uint8_t ready)
{
    if (start->phase < START_RELEASE || start->phase > START_BEEP)
        return;
    if (!ready || boot != start->boot || session != start->session ||
        (uint32_t)(now - start->prepared_ms) >= 60000U)
    {
        start->reason = !ready ? START_NOT_READY :
            (boot != start->boot || session != start->session) ? START_IDENTITY : START_TIMEOUT;
        start->aborted_ms = now;
        Start_Cancel(start);
        return;
    }
    key = (uint8_t)!!key;
    if (key != start->raw_key)
    {
        start->raw_key = key;
        start->changed_ms = now;
    }
    if ((uint32_t)(now - start->changed_ms) >= 30U)
        start->stable_key = start->raw_key;
    if (start->phase == START_RELEASE && !start->stable_key &&
        (uint32_t)(now - start->changed_ms) >= 30U)
        start->phase = START_PRESS;
    else if (start->phase == START_PRESS && start->stable_key)
    {
        start->beep = 1;
        start->beep_ms = now;
        start->phase = START_BEEP;
    }
    else if (start->phase == START_BEEP && (uint32_t)(now - start->beep_ms) >= 150U)
    {
        start->beep = 0;
        start->phase = START_FIRED;
    }
}
