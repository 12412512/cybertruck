/* 新旧报文长度及手动模式校验，复用正式板级适配器。 */
#define main PortFixtureMain
#include "mech_port.c"
#undef main
int main(int argc,char **argv)
{
    unsigned which=argc>1?(unsigned)atoi(argv[1]):0;
    uint8_t data[96]={0};MechPort_Init();Prepare();
    Frame_Put(data+4,1,4);data[8]=data[9]=1;CHECK(!Command(62,data,10));
    memset(data,0,sizeof(data));Frame_Put(data+4,1,4);Frame_Put(data+8,1,4);
    if(which<3)
    {
        Frame_Put(data+12,1000,4);Frame_Put(data+16,5000,4);Frame_Put(data+20,100,2);data[22]=0;data[23]=160;
        if(which==2){manual_mode=0;CHECK(Command(52,data,24)==MECH_MANUAL);return 0;}
        CHECK(!Command(52,data,which==0?23:24));
        CHECK(axis.goal.accel_set==(which==1));CHECK(which==0||axis.goal.accel==160);
    }
    else
    {
        Frame_Put(data+12,5000,4);Frame_Put(data+16,3,4);
        Frame_Put(data+20,1000,4);Frame_Put(data+24,1000,4);Frame_Put(data+40,100,4);Frame_Put(data+44,120,4);
        data[60]=100;data[61]=200;
        CHECK(!Command(63,data,which==3?60:62));
        CHECK(group.goal.accel_set==(which==4));
        CHECK(which==3||(group.axes[0].goal.accel==100&&group.axes[1].goal.accel==200));
    }
    CHECK(!writes&&!starts);return 0;
}
