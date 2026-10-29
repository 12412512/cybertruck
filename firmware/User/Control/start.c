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
    {
        if (start->ticket == ticket)
            return start->version == version && start->phase != START_ABORTED;
        /* 仅未启动的选区变化允许同会话重准备；超时、STOP、故障均不能重试。 */
        if (start->phase != START_ABORTED || start->reason != START_SELECTION)
            return 0;
    }
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

void Select_Init(Select *select, uint32_t now, uint8_t level)
{
    memset(select, 0, sizeof(*select));
    select->raw = select->stable = (uint8_t)!!level;
    select->changed_ms = now;
    select->epoch = 1;
}

void Select_Run(Select *select, uint32_t now, uint8_t level)
{
    level = (uint8_t)!!level;
    if (select->raw != level)
    {
        select->raw = level;
        select->valid = 0;
        select->changed_ms = now;
        if (!++select->epoch)
            select->epoch = 1;
    }
    if ((uint32_t)(now - select->changed_ms) >= 50U)
    {
        select->stable = select->raw;
        select->valid = 1;
    }
}

uint32_t Select_Zone(const Select *select)
{
    return select->valid ? (uint32_t)select->stable + 1U : 0U;
}

int Start_Bind(Start *start, const Select *select, uint32_t zone, uint32_t epoch)
{
    if (start->version == 9 && start->phase >= START_RELEASE && start->phase <= START_FIRED &&
        start->zone == zone && start->select_epoch == epoch && zone && epoch)
        return 1;
    if (start->version != 9 || start->phase < START_RELEASE || start->phase > START_PRESS ||
        !epoch || epoch != select->epoch || !zone || zone != Select_Zone(select))
        return 0;
    start->zone = zone;
    start->select_epoch = epoch;
    return 1;
}

void Start_Check(Start *start, const Select *select, uint32_t now)
{
    /* 短鸣结束即锁定本轮。途中拨档只更新只读输入，不切换轨迹或重发动作。 */
    if (start->zone && start->phase >= START_RELEASE && start->phase <= START_BEEP &&
        (Select_Zone(select) != start->zone || select->epoch != start->select_epoch))
    {
        start->reason = START_SELECTION;
        start->aborted_ms = now;
        Start_Cancel(start);
    }
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
