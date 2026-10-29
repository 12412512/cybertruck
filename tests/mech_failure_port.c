/* 正式板级70只读报文的冻结、签名和无输出检查。 */
#define main PortFixtureMain
#include "mech_port.c"
#undef main
int main(void)
{
    uint8_t response[128];uint16_t length=0;
    MechPort_Init();
    guard.zero_valid=1;guard.zero_version=2;guard.manual_range=1;
    guard.zero_actual[0]=3;guard.zero_actual[1]=249;
    guard.sample[1].value=1;guard.sample[5].value=-43905;
    guard.sample[3].value=2;guard.sample[7].value=-44131;
    guard.sample[1].tick=80;guard.sample[5].tick=90;
    Mech_Freeze(&guard,104,2,100);
    CHECK(MechPort_Command(70,0,0,response,&length,tick)==COMM_OK&&length==124);
    CHECK(Frame_Get(response,4)==1&&Frame_Get(response+4,4)==1);
    CHECK(Frame_Get(response+12,4)==104&&Frame_Get(response+16,4)==2);
    CHECK((int64_t)Frame_Get(response+36,8)==-43905);
    CHECK((int64_t)Frame_Get(response+100,8)>83000);
    uint8_t saved[124];memcpy(saved,response,124);
    Mech_Invalidate(&guard,104);guard.sample[5].value=100000;
    Mech_Freeze(&guard,8,2,200);
    CHECK(MechPort_Command(70,0,0,response,&length,tick)==COMM_OK);
    CHECK(!memcmp(saved,response,124)&&!writes&&!starts&&!sends);
    return 0;
}
