#define PORT_MAIN ExistingPortMain
#include "mech_port.c"

int main(int argc,char **argv)
{
    unsigned which=argc>1?(unsigned)atoi(argv[1]):0;
    uint8_t data[96]={0};
    MechPort_Init();Prepare();CHECK(!MechPort_Manual());
    Frame_Put(data+4,1,4);data[8]=data[9]=1;
    CHECK(!Command(62,data,10) && MechPort_Manual());
    if(which==1)pose.state=POSE_ACCEPTED;
    if(which==2)guard.fault=MECH_DRIVER;
    if(which==3)attached=0;
    if(which==4)MechPort_Stop(MECH_MANUAL,tick);
    CHECK(MechPort_Manual()==(which==0));
    CHECK(!writes&&!starts&&!sends);
    return 0;
}
