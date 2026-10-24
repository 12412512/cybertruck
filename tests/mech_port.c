/* 板级适配器使用合成CAN/HAL：测试期间不访问任何设备。 */
#include "../firmware/User/Board/mech_port.c"
#include <stdio.h>
#include <stdlib.h>
#define CHECK(x) do { if (!(x)) { fprintf(stderr,"line %d: %s\n",__LINE__,#x); exit(1); } } while (0)
volatile BusDiag bus_diag;
volatile CanDiag can_diag;
int htim1;
static uint32_t tick = 100, seq = 1, token = 0, writes, starts, stops, sends;
static uint64_t session = 456;
static int64_t positions[2];
uint32_t HAL_GetTick(void) { return tick; }
uint64_t Comm_Boot(void) { return 123; }
uint64_t Comm_Session(void) { return session; }
uint32_t HAL_TIM_PWM_Start(int *timer, uint32_t channel) { (void)timer;(void)channel;starts++;return 0; }
void Test_Compare(uint32_t channel, uint32_t pulse) { CHECK(channel<=8&&pulse>=500&&pulse<=2500); writes++; }
void Bus_MechWatch(void) { }
void Bus_Stop(void) { stops++; bus_diag.pending_stop_count=6; bus_diag.error|=1; }
uint8_t Bus_MechDone(void) { if(bus_diag.error)return 0; bus_diag.motion_uncertain=0;return 1; }
uint8_t Bus_Send(const BusFrame *frames,uint8_t count) { CHECK(count&&frames[0].len);sends++;return 1; }
static uint32_t Command(uint8_t type, uint8_t *data, uint16_t size)
{
    uint8_t response[128]; uint16_t length=0;
    Frame_Put(data, ++token, 4);
    CHECK(MechPort_Command(type,data,size,response,&length,tick)==COMM_OK);
    CHECK(length==4);
    return (uint32_t)Frame_Get(response,4);
}
static void Feed(unsigned a,uint8_t code,int64_t value)
{
    BusFrame f={0}; uint32_t magnitude=(uint32_t)(value<0?-value:value);
    f.id=(a+5)<<8;f.tick=tick;f.rx_sequence=seq++;f.data[0]=code;
    if(code==0x35){f.len=5;f.data[1]=value<0;f.data[2]=magnitude>>8;f.data[3]=magnitude;}
    else if(code==0x3A){f.len=3;f.data[1]=value;}
    else {f.len=7;f.data[1]=value<0;f.data[2]=magnitude>>24;f.data[3]=magnitude>>16;f.data[4]=magnitude>>8;f.data[5]=magnitude;}
    f.data[f.len-1]=0x6B;MechPort_Feed(&f);
}
static void Round(void)
{
    unsigned a;tick+=80;
    for(a=0;a<2;a++){Feed(a,0x35,0);Feed(a,0x36,positions[a]);Feed(a,0x33,positions[a]);Feed(a,0x3A,3);}
    MechPort_Run(tick,1,0);
}
static void Prepare(void)
{
    uint8_t data[96]={0}; unsigned a,n;
    Frame_Put(data+4,0x4D454348,4);CHECK(Command(48,data,8)==0);
    for(a=0;a<2;a++){config[a].valid=1;config[a].motor_type=0x19;config[a].microsteps=16;}
    for(n=0;n<6;n++)Round();
    memset(data,0,sizeof(data));data[8]=1;CHECK(Command(50,data,9)==0);
    CHECK(guard.zero_valid&&guard.zero_version==1);
    Frame_Put(data+4,1,4);Frame_Put(data+8,30000,4);Frame_Put(data+12,180000,4);Frame_Put(data+16,98000,4);data[20]=1;
    CHECK(Command(51,data,21)==0);CHECK(!writes&&!starts);
}
int main(int argc,char **argv)
{
    unsigned which=argc>1?(unsigned)atoi(argv[1]):0,n;
    uint8_t data[96]={0},response[128];uint16_t size=0;BusFrame frames[2];
    MechPort_Init();CHECK(!attached&&!writes&&!starts&&!sends);
    if(which==0){MechPort_Run(tick,1,0);CHECK(!MechPort_Output(frames,tick)&&!MechPort_Query(frames,tick));}
    else if(which==1){Frame_Put(data+4,0x4D454348,4);CHECK(Command(48,data,8)==0);uint32_t old=token;CHECK(MechPort_Command(48,data,8,response,&size,tick)==0);CHECK(token==old&&!writes&&!sends);data[7]^=1;CHECK(MechPort_Command(48,data,8,response,&size,tick)==COMM_FORMAT);}
    else {
        Prepare();Frame_Put(data+4,1,4);
        if(which==2){CHECK(!writes&&!starts&&guard.config_mask==3);CHECK(MechPort_Command(58,0,0,response,&size,tick)==0&&size==112);}
        else if(which==3){data[8]=0;CHECK(Command(49,data,9)==0&&enable_pending);CHECK(!enabled_mask);MechPort_Sent(1,tick);CHECK(enable_pending&&!enabled_mask);CHECK(MechPort_Output(frames,tick)==1&&frames[0].data[0]==0xF3);MechPort_Sent(1,tick);CHECK(enabled_mask==1&&!writes);}
        else if(which==4){Frame_Put(data+8,1,4);Frame_Put(data+12,2000,4);Frame_Put(data+16,15000,4);Frame_Put(data+20,5,2);data[22]=0;CHECK(Command(52,data,23)==0);CHECK(MechPort_Output(frames,tick)==2);CHECK(frames[0].id==0x500&&frames[0].data[0]==0xFD);MechPort_Sent(1,++tick);CHECK(axis.state==AXIS_RUNNING);tick+=251;MechPort_Run(tick,1,0);CHECK(guard.fault&&stops);}
        else if(which==5){Frame_Put(data+8,1,4);Frame_Put(data+12,180000,4);Frame_Put(data+16,30000,4);data[20]=2;CHECK(Command(53,data,22)==0);for(n=0;n<12;n++)Round();CHECK(starts==1&&writes&&servo.state==SERVO_COMMAND_DONE);CHECK(!Servo_Reference(&servo,2));ServoEvidence evidence={123,456,1,1,1,tick,180000,2,SERVO_MODEL};CHECK(Servo_Confirm(&servo,&evidence,tick)==SERVO_EVIDENCE);evidence.kind=SERVO_MANUAL;CHECK(Servo_Confirm(&servo,&evidence,tick)==0);CHECK(Servo_Reference(&servo,2));}
        else if(which==6){session=0;MechPort_Run(++tick,1,0);CHECK(guard.fault==MECH_SESSION&&stops&&!writes);}
        else if(which==7){MechPort_Run(++tick,1,4);CHECK(guard.fault&&stops&&!writes);}
        else if(which==8){MechPort_Run(tick+251,1,0);CHECK(!guard.zero_valid&&guard.fault==MECH_STALE&&stops);}
        else if(which==9){Frame_Put(data+8,1,4);Frame_Put(data+12,1,4);Frame_Put(data+16,120000,4);Frame_Put(data+20,0,4);Frame_Put(data+28,180000,4);Frame_Put(data+32,30000,4);Frame_Put(data+36,98000,4);data[40]=1;CHECK(Command(55,data,42)==SERVO_REFERENCE);CHECK(!writes&&!sends);}
        else if(which==10){Frame_Put(data+8,1,4);Frame_Put(data+12,180000,4);Frame_Put(data+16,30000,4);data[20]=2;CHECK(Command(53,data,22)==0);CHECK(MechPort_Command(53,data,22,response,&size,tick)==0);CHECK(!Frame_Get(response,4));data[12]^=1;CHECK(MechPort_Command(53,data,22,response,&size,tick)==COMM_FORMAT);CHECK(!writes);}
        else if(which==11){config[0].microsteps=32;MechPort_Run(++tick,1,0);CHECK(guard.fault==MECH_CONFIG&&stops&&!writes);}
        else if(which==12){MechPort_Run(++tick,0,0);Frame_Put(data+8,1,4);Frame_Put(data+12,180000,4);Frame_Put(data+16,30000,4);data[20]=2;CHECK(Command(53,data,22)==MECH_BASE);CHECK(!writes);}
    }
    printf("mech port case %u PASS\n",which);return 0;
}
