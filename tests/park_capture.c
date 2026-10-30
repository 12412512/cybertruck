/* 正式Base取证函数；故障注入只修改本进程的合成反馈。 */
#define BASE_STARTUP_NO_MAIN
#include "base_startup.c"

int main(int argc,char **argv)
{
    unsigned which=argc>1?(unsigned)atoi(argv[1]):0;
    BusWheel *w=(BusWheel *)Bus_Wheels();
    uint8_t reply[128],first[128];uint16_t size=0;
    tick=12345;mech_active=1;executor.stop_confirmed=1;
    memset(w,0,4*sizeof(*w));
    for(unsigned i=0;i<4;i++)
    {
        w[i].speed.valid=w[i].status.valid=w[i].position.valid=1;
        w[i].speed.tick=w[i].status.tick=w[i].position.tick=tick-10;
        w[i].status.value=3;
    }
    if(which==0)w[2].speed.value=-3;
    if(which==1)w[1].position.tick=tick-251;
    if(which==2)w[0].speed.valid=0;
    if(which==3)w[3].status.value=7;
    if(which==4)executor.stop_confirmed=0;
    if(which==5)can_diag.errors=1;
    if(which==6){mech_active=0;w[0].speed.value=4;}
    Park_Capture(tick);
    CHECK(Command(69,0,0,reply,&size,tick)==COMM_OK && size==124);
    CHECK(Frame_Get(reply,4)==2 && Frame_Get(reply+4,4)==(which<6));
    if(which<6)
    {
        unsigned expected[]={4,2,1,8,16,32};
        CHECK(Frame_Get(reply+12,4)==expected[which]);
        CHECK(Frame_Get(reply+32,4)==32 && Frame_Get(reply+36,4)==2);
        if(which==0)CHECK(Frame_Get(reply+16,4)==3 && (int32_t)Frame_Get(reply+80,4)==-3);
        if(which==1)CHECK(Frame_Get(reply+72,4)==251);
        if(which==2)CHECK(Frame_Get(reply+48,4)==UINT32_MAX);
        memcpy(first,reply,size);
        w[0].speed.value=100;can_diag.errors=5;executor.stop_confirmed=0;
        Park_Capture(tick+20);
        CHECK(Command(69,0,0,reply,&size,tick+20)==COMM_OK && !memcmp(reply,first,120));
        CHECK(Command(69,reply,1,first,&size,tick)==COMM_FORMAT);
    }
    else CHECK(!park_failure.captured);
    printf("parking capture case %u PASS: frozen read-only evidence\n",which);
    return 0;
}
