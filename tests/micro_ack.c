/* 正式Base Send + Bus合并队列：1RPM尚未发出即被2RPM取代。 */
#define BASE_STARTUP_NO_MAIN
#include "base_startup.c"
int main(void)
{
    int16_t first[4]={-1,-1,1,1},next[4]={-2,-2,2,2},zero[4]={0};
    Can_Refresh();Bus_Setup(fixture_addresses);CHECK(Bus_Start());
    CHECK(Send(first,2,222));uint32_t first_seq=zero_sequence;
    CHECK(Send(next,2,222));uint32_t final_seq=zero_sequence;
    CHECK(final_seq!=first_seq);CHECK(!memcmp(acknowledged_rpm,next,sizeof(next)));
    CHECK(Send(next,2,222));CHECK(zero_sequence==final_seq);
    /* 运行真实CAN发送，旧1RPM队列被替换，实际整批只有2RPM。 */
    for(unsigned n=0;n<100 && bus_diag.speed_sent_sequence<final_seq;n++)
    {
        tick++;
        if(occupied){unsigned id=last.id>>8,code=last.data[0];Done();if(code==0x35||code==0x36||code==0x3A)Reply(id,code,0);}
        Bus_Poll();
    }
    CHECK(bus_diag.speed_sent_sequence>=final_seq);
    CHECK(!memcmp((const void *)bus_diag.sent_rpm,acknowledged_rpm,sizeof(next)));
    CHECK(Send(zero,2,223));CHECK(zero_id==223 && zero_sequence>final_seq);
    CHECK(memcmp((const void *)bus_diag.sent_rpm,acknowledged_rpm,sizeof(next)));
    puts("real Base/Bus replaced ramp ACK and zero isolation PASS");return 0;
}
