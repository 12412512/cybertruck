/* 复用真实板级适配器与合成HAL，观察100度完成/停止后仍保持PWM。 */
#define PORT_MAIN ExistingPortMain
#include "mech_port.c"

int main(int argc, char **argv)
{
    unsigned which=argc>1?(unsigned)atoi(argv[1]):0,n;
    uint8_t data[96]={0};
    MechPort_Init();Prepare();
    Frame_Put(data+4,1,4);data[8]=data[9]=1;CHECK(!Command(62,data,10));
    memset(data,0,sizeof(data));Frame_Put(data+4,1,4);Frame_Put(data+8,1,4);
    Frame_Put(data+12,which==3?100001:which==0?98000:which==1?99000:100000,4);
    Frame_Put(data+16,10000,4);data[20]=3;Frame_Put(data+22,45000,4);
    uint32_t result=Command(53,data,26);
    if(which==3){CHECK(result==MECH_RANGE&&!writes&&!starts&&!pwm_mask);return 0;}
    CHECK(!result);
    for(n=0;n<80 && servo.state!=SERVO_MANUAL_DONE;n++)Round();
    CHECK(n<80 && pwm_mask==4 && starts==1 && pulses[2]==(which==0?1226:which==1?1233:1241));
    uint32_t pulse=pulses[2],before=writes;
    for(n=0;n<20;n++)Round();
    CHECK(pwm_mask==4 && pulses[2]==pulse && writes==before);
    if(which==4)MechPort_Stop(MECH_MANUAL,tick);
    if(which==5){tick+=251;MechPort_Run(tick,1,0);}
    if(which>=4)CHECK(!guard.zero_valid && guard.fault && pwm_mask==4 && pulses[2]==pulse && writes==before);
    return 0;
}
