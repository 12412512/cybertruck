#include "start.h"
#include <assert.h>
#include <stdint.h>

static void Prepare(Start *start, uint32_t now, uint8_t key)
{
    Start_Init(start);
    assert(Start_Prepare(start, 1, 2, 3, 5, now, key, 1));
}

int main(void)
{
    Start start;
    uint32_t tick;
    Prepare(&start, 1000, 0);
    for (tick = 1000; tick < 61000; tick += 10)
    {
        Start_Run(&start, 1, 2, tick, 0, 1);
        assert(start.phase == (tick < 1030 ? START_RELEASE : START_PRESS));
        assert(!start.beep && !start.reason);
    }
    Start_Run(&start, 1, 2, 61000, 0, 1);
    assert(start.phase == START_ABORTED && start.reason == START_TIMEOUT);
    Prepare(&start, 0, 1);
    Start_Run(&start, 1, 2, 1000, 1, 1);
    assert(start.phase == START_RELEASE && !start.beep);
    Start_Run(&start, 1, 2, 1010, 0, 1);
    Start_Run(&start, 1, 2, 1040, 0, 1);
    assert(start.phase == START_PRESS);
    Start_Run(&start, 1, 2, 1050, 1, 1);
    Start_Run(&start, 1, 2, 1079, 1, 1);
    assert(!start.beep);
    Start_Run(&start, 1, 2, 1080, 1, 1);
    assert(start.phase == START_BEEP && start.beep);
    Start_Run(&start, 1, 2, 1229, 1, 1);
    assert(start.phase == START_BEEP);
    Start_Run(&start, 1, 2, 1230, 1, 1);
    assert(start.phase == START_FIRED && !start.beep);
    Start_Run(&start, 1, 2, 2000, 1, 1);
    assert(start.phase == START_FIRED);
    assert(Start_Prepare(&start, 1, 2, 3, 5, 2000, 1, 1));
    assert(!Start_Prepare(&start, 1, 2, 4, 5, 2000, 0, 1));
    Prepare(&start, 0, 0);
    Start_Run(&start, 1, 2, 30, 0, 1);
    Start_Run(&start, 1, 2, 40, 1, 1);
    Start_Run(&start, 1, 2, 50, 0, 1);
    Start_Run(&start, 1, 2, 80, 0, 1);
    assert(start.phase == START_PRESS && !start.beep);
    Start_Run(&start, 1, 2, 90, 1, 1);
    Start_Run(&start, 1, 2, 120, 1, 1);
    Start_Run(&start, 1, 2, 130, 1, 0);
    assert(start.phase == START_ABORTED && !start.beep);
    assert(start.reason == START_NOT_READY && start.aborted_ms == 130);
    Start_Run(&start, 1, 2, 150, 0, 1);
    assert(start.reason == START_NOT_READY && start.aborted_ms == 130);
    Prepare(&start, 0, 0);
    Start_Run(&start, 1, 3, 30, 0, 1);
    assert(start.phase == START_ABORTED);
    assert(start.reason == START_IDENTITY);
    Prepare(&start, 0, 0);
    Start_Run(&start, 1, 2, 60000, 0, 1);
    assert(start.phase == START_ABORTED);
    assert(start.reason == START_TIMEOUT && start.aborted_ms == 60000);
    Prepare(&start, UINT32_MAX - 40, 0);
    Start_Run(&start, 1, 2, UINT32_MAX - 10, 0, 1);
    Start_Run(&start, 1, 2, 0, 1, 1);
    Start_Run(&start, 1, 2, 30, 1, 1);
    Start_Run(&start, 1, 2, 180, 1, 1);
    assert(start.phase == START_FIRED && !start.beep);
    Start_Cancel(&start);
    assert(start.phase == START_ABORTED);
    assert(!Start_Prepare(&start, 1, 2, 5, 5, 200, 0, 1));
    assert(Start_Prepare(&start, 1, 3, 5, 5, 200, 0, 1));
    {
        Select select;
        uint32_t epoch;
        Select_Init(&select, UINT32_MAX - 20U, 0);
        assert(!Select_Zone(&select));
        Select_Run(&select, 29, 0);
        assert(Select_Zone(&select) == 1);
        Start_Init(&start);
        assert(Start_Prepare(&start, 1, 2, 10, 9, 30, 0, 1));
        epoch = select.epoch;
        assert(!Start_Bind(&start, &select, 2, epoch));
        assert(!Start_Bind(&start, &select, 1, epoch + 1));
        assert(Start_Bind(&start, &select, 1, epoch));
        Start_Run(&start, 1, 2, 60, 0, 1);
        Start_Run(&start, 1, 2, 70, 1, 1);
        Start_Run(&start, 1, 2, 100, 1, 1);
        assert(start.phase == START_BEEP);
        Select_Run(&select, 110, 1);
        Start_Check(&start, &select, 110);
        assert(start.phase == START_ABORTED && start.reason == START_SELECTION && !start.beep);
        assert(!Start_Bind(&start, &select, 1, epoch));
        Select_Run(&select, 120, 0);
        Select_Run(&select, 130, 1);
        Select_Run(&select, 179, 1);
        assert(!Select_Zone(&select));
        Select_Run(&select, 180, 1);
        assert(Select_Zone(&select) == 2 && select.epoch != epoch);
        assert(!Start_Prepare(&start, 1, 2, 10, 9, 180, 1, 1));
        assert(Start_Prepare(&start, 1, 2, 11, 9, 180, 1, 1));
        assert(Start_Bind(&start, &select, 2, select.epoch));
        Start_Run(&start, 1, 2, 500, 1, 1);
        assert(start.phase == START_RELEASE);
        Start_Run(&start, 1, 2, 510, 0, 1);
        Start_Run(&start, 1, 2, 540, 0, 1);
        Start_Run(&start, 1, 2, 550, 1, 1);
        Start_Run(&start, 1, 2, 580, 1, 1);
        Start_Run(&start, 1, 2, 730, 1, 1);
        assert(start.phase == START_FIRED);
        Select_Run(&select, 740, 0);
        Start_Check(&start, &select, 740);
        assert(start.phase == START_FIRED && start.zone == 2);
        assert(!Start_Prepare(&start, 1, 2, 12, 9, 750, 0, 1));
    }
    return 0;
}
