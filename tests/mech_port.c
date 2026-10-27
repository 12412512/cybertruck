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
static uint32_t pulses[3], pwm_start_fail;
static int64_t positions[2];
uint32_t HAL_GetTick(void) { return tick; }
uint64_t Comm_Boot(void) { return 123; }
uint64_t Comm_Session(void) { return session; }
uint32_t HAL_TIM_PWM_Start(int *timer, uint32_t channel) { (void)timer;starts++;return pwm_start_fail==channel+1; }
void Test_Compare(uint32_t channel, uint32_t pulse) { CHECK(channel<=8&&pulse>=500&&pulse<=2500); pulses[channel/4]=pulse; writes++; }
void Bus_MechWatch(void) { }
uint8_t Bus_MechGroup(uint8_t axes) { return axes<=3; }
int Bus_Rearm(void) { bus_diag.error=0;bus_diag.locked=0;return 1; }
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
    if(which>=56){
        Prepare();Frame_Put(data+4,1,4);data[8]=data[9]=1;CHECK(!Command(62,data,10));
        Servo_Init(&servo,&guard,0);
        memset(data,0,sizeof(data));Frame_Put(data+4,1,4);Frame_Put(data+8,1,4);data[12]=1;
        if(which==57)manual_mode=0;
        if(which==58)guard.zero_valid=0;
        if(which==59)tick+=251;
        if(which==60)data[12]=0;
        if(which==61)Frame_Put(data+4,2,4);
        uint32_t accepted=Command(66,data,13);
        if(which>=57&&which<=61){CHECK(accepted&&!writes&&!starts&&!home_state);return 0;}
        CHECK(!accepted&&home_state==1&&!writes&&!servo.reference_mask);
        if(which==69){session++;MechPort_Run(tick,1,0);CHECK(home_state==5&&!writes);return 0;}
        if(which==68)pwm_start_fail=TIM_CHANNEL_2+1;
        Round();
        if(which==68){CHECK(home_state==5&&!servo.reference_mask&&pwm_mask==1);return 0;}
        CHECK(home_state==2&&pwm_mask==7&&writes==3&&starts==3);
        CHECK(pulses[0]==500&&pulses[1]==2500&&pulses[2]==500);
        CHECK(!servo.reference_mask&&!servo.confirmed_mask);
        if(which==62){CHECK(Command(51,data,13)==MECH_UNSTABLE);CHECK(Command(67,data,13));CHECK(!servo.reference_mask);return 0;}
        if(which==63){MechPort_Stop(MECH_MANUAL,tick);CHECK(home_state==5&&!guard.zero_valid&&!servo.reference_mask);return 0;}
        if(which==64){MechPort_Run(tick,1,1);CHECK(home_state==5&&!servo.reference_mask&&writes==3);return 0;}
        if(which==65){MechPort_Run(tick+60000,1,0);CHECK(home_state==5&&!servo.reference_mask);return 0;}
        if(which==67){uint32_t old_writes=writes;CHECK(MechPort_Command(66,data,13,response,&size,tick)==0&&!Frame_Get(response,4));CHECK(writes==old_writes);}
        for(n=0;n<8;n++)Round();CHECK(home_state==3&&!servo.reference_mask);
        CHECK(MechPort_Command(68,0,0,response,&size,tick)==0&&size==32&&Frame_Get(response+12,4)==3);
        if(which==70){for(n=0;n<750&&home_state==3;n++)Round();CHECK(home_state==5&&!servo.reference_mask);return 0;}
        if(which==71){home_started=tick-60000;CHECK(Command(67,data,13)==SERVO_TIMEOUT&&!servo.reference_mask);return 0;}
        if(which==66)Frame_Put(data+8,2,4);
        uint32_t confirmed=Command(67,data,13);
        if(which==66){CHECK(confirmed==MECH_VERSION&&!servo.reference_mask);return 0;}
        CHECK(!confirmed&&home_state==4&&servo.reference_mask==7&&servo.confirmed_mask==7&&servo.output_mask==7);
        CHECK(!servo.command_mdeg[0]&&!servo.command_mdeg[1]&&!servo.command_mdeg[2]&&writes==3&&!sends);
        if(which==56){unsigned ch,k;uint32_t id=2;int32_t targets[3]={5000,10000,0};
            for(ch=1;ch<=3;ch++)for(k=0;k<3;k++){
                memset(data,0,sizeof(data));Frame_Put(data+4,1,4);Frame_Put(data+8,id++,4);
                Frame_Put(data+12,(uint32_t)targets[k],4);Frame_Put(data+16,10000,4);
                data[20]=(uint8_t)ch;data[21]=1;Frame_Put(data+22,90000,4);
                CHECK(!Command(53,data,26));
                for(n=0;n<80&&servo.state!=SERVO_MANUAL_DONE;n++)Round();
                CHECK(servo.state==SERVO_MANUAL_DONE&&servo.command_mdeg[ch-1]==targets[k]&&home_state==4);
            }
        }
        return 0;
    }
    if(which>=49){Prepare();CHECK(MechPort_CanReset(tick,1));if(which==49){guard.fault=MECH_FAULT;CHECK(MechPort_CanReset(tick,1));}if(which==50){tick+=251;CHECK(!MechPort_CanReset(tick,1));}if(which==51){guard.sample[0].value=1;CHECK(!MechPort_CanReset(tick,1));}if(which==52){guard.sample[6].value|=8;CHECK(!MechPort_CanReset(tick,1));}if(which==53)CHECK(!MechPort_CanReset(tick,2));if(which==54){guard.config_mask=1;CHECK(!MechPort_CanReset(tick,1));}if(which==55){pwm_mask=7;CHECK(MechPort_PwmOff()&&!pwm_mask&&!starts&&!sends);}return 0;}
    if(which==0){MechPort_Run(tick,1,0);CHECK(!MechPort_Output(frames,tick)&&!MechPort_Query(frames,tick));}
    else if(which==1){Frame_Put(data+4,0x4D454348,4);CHECK(Command(48,data,8)==0);uint32_t old=token;CHECK(MechPort_Command(48,data,8,response,&size,tick)==0);CHECK(token==old&&!writes&&!sends);data[7]^=1;CHECK(MechPort_Command(48,data,8,response,&size,tick)==COMM_FORMAT);}
    else if(which >= 26)
    {
        Prepare();
        Frame_Put(data+4,1,4);data[8]=data[9]=1;
        if(which==27)tick+=251;
        if(which==28)Frame_Put(data+4,2,4);
        if(which==29)can_diag.errors=1;
        if(which==33)data[9]=0;
        uint32_t result=Command(62,data,10);
        if(which==27||which==28||which==29||which==33)CHECK(result&&!manual_mode);
        else
        {
            CHECK(!result&&manual_mode&&!writes&&!starts&&!sends);
            if(which>=45)
            {
                if(which==47){data[8]=0;CHECK(!Command(62,data,10));}
                memset(data,0,sizeof(data));Frame_Put(data+4,1,4);Frame_Put(data+8,1,4);
                Frame_Put(data+12,15000,4);Frame_Put(data+16,31,4);
                int32_t targets[5]={2000,2000,0,359000,98000};
                for(n=0;n<5;n++){Frame_Put(data+20+n*4,(uint32_t)(which==46&&n==4?99000:targets[n]),4);Frame_Put(data+40+n*4,n<2?5:180000,4);}
                uint32_t accepted=Command(63,data,60);
                if(which==46||which==47){CHECK(accepted==(which==46?MECH_RANGE:MECH_MANUAL)&&!writes&&!starts&&!sends);return 0;}
                CHECK(!accepted&&group.state==GROUP_RUNNING);
                uint32_t id=group.goal.id;CHECK(MechPort_Command(63,data,60,response,&size,tick)==COMM_OK&&Frame_Get(response,4)==0&&group.goal.id==id);
                for(n=0;n<200&&group.state==GROUP_RUNNING;n++)
                {
                    Round();
                    uint8_t count=MechPort_Output(frames,tick);
                    if(count)
                    {
                        unsigned a=(frames[0].id>>8)-5;
                        MechPort_Sent(1,tick);positions[a]=group.axes[a].before_target+group.axes[a].delta_raw;
                        BusFrame ack={0};ack.id=(a+5)<<8;ack.len=3;ack.data[0]=0xFD;ack.data[1]=2;ack.data[2]=0x6B;ack.tick=tick;ack.rx_sequence=seq++;MechPort_Feed(&ack);
                    }
                    if(which==48&&n==1){MechPort_Stop(MECH_MANUAL,tick);CHECK(group.state==GROUP_CANCELLED&&!manual_mode);return 0;}
                }
                CHECK(group.state==GROUP_DONE&&pwm_mask==7&&writes&&starts);
                CHECK(MechPort_Command(64,0,0,response,&size,tick)==COMM_OK&&size==40&&Frame_Get(response+12,4)==GROUP_DONE);
                return 0;
            }
            if(which==30){MechPort_Stop(MECH_MANUAL,tick);CHECK(!manual_mode);}
            if(which==31)CHECK(Command(55,data,42)==MECH_MANUAL&&!writes&&!sends);
            if(which==32){data[8]=0;CHECK(!Command(62,data,10)&&!manual_mode&&guard.zero_valid);}
            if(which>=34 && which<=37)
            {
                if(which==37){data[8]=0;CHECK(!Command(62,data,10)&&!manual_mode);}
                memset(data,0,sizeof(data));Frame_Put(data+4,1,4);Frame_Put(data+8,1,4);
                Frame_Put(data+12,175000,4);Frame_Put(data+16,120000,4);data[20]=2;
                Frame_Put(data+22,which==35?360001:90000,4);
                uint32_t servo_result=Command(53,data,which==36?22:26);
                if(which==35)CHECK(servo_result==MECH_RANGE&&!writes);
                else if(which==37)CHECK(servo_result==MECH_MANUAL&&!writes);
                else CHECK(!servo_result&&servo.goal.speed_mdeg_s==(which==36?0:90000));
            }
            if(which==38)
            {
                memset(data,0,sizeof(data));Frame_Put(data+4,1,4);Frame_Put(data+8,1,4);
                Frame_Put(data+12,100000,4);Frame_Put(data+16,120000,4);Frame_Put(data+20,1000,2);
                CHECK(!Command(52,data,23)&&axis.goal.target_um==100000&&guard.manual_range);
            }
            if(which==39)
            {
                positions[0]=160000;for(n=0;n<6;n++)Round();
                CHECK(guard.zero_valid&&Mech_Position(&guard,0)>49000);
                data[8]=0;CHECK(Command(62,data,10)==MECH_RANGE&&manual_mode);
            }
            if(which==40)
            {
                positions[0]=(int64_t)49200*MECH_RAW_PER_320/MECH_LIFT_UM_PER_320;
                for(n=0;n<6;n++)Round();
                CHECK(guard.zero_valid);
                data[8]=0;CHECK(!Command(62,data,10)&&!manual_mode&&!guard.manual_range);
                Round();CHECK(guard.zero_valid&&!guard.fault);
            }
            if(which==41||which==42)
            {
                /* 手动扩展角度不输出PWM；旧自动模式不能从该角度直接接管。 */
                servo.command_mdeg[1]=which==41?0:270000;
                data[8]=0;CHECK(Command(62,data,10)==MECH_RANGE&&manual_mode);
                servo.command_mdeg[1]=180000;
                CHECK(!Command(62,data,10)&&!manual_mode);
            }
            if(which==43||which==44)
            {
                memset(data,0,sizeof(data));Frame_Put(data+4,1,4);Frame_Put(data+8,1,4);
                Frame_Put(data+12,which==43?0:270000,4);Frame_Put(data+16,120000,4);data[20]=2;
                Frame_Put(data+22,90000,4);
                CHECK(!Command(53,data,26)&&servo.goal.target_mdeg==(which==43?0:270000));
            }
            if(which==26)
            {
                memset(data,0,sizeof(data));Frame_Put(data+4,1,4);Frame_Put(data+8,1,4);
                Frame_Put(data+12,2000,4);Frame_Put(data+16,15000,4);Frame_Put(data+20,5,2);data[22]=1;
                CHECK(!Command(52,data,23)&&axis.manual&&axis.state==AXIS_ACCEPTED);
                CHECK(Command(62,data,10)==MECH_UNSTABLE);
            }
        }
        CHECK(!writes&&!starts&&!sends);
    }
    else if(which >= 18)
    {
        Prepare();
        axis.goal.id=7;servo.goal.id=8;pose.goal.id=9;pose.child_id=10;
        MechPort_Stop(MECH_MANUAL,tick);
        MechPort_Run(tick,1,0);
        bus_diag.pending_stop_count=0;
        bus_diag.stop_sent_sequence=bus_diag.stop_sequence;
        MechPort_Run(tick,1,0);
        for(n=0;n<6;n++)Round();
        Frame_Put(data+4,guard.zero_version,4);data[8]=1;
        if(which==19)can_diag.errors=1;
        if(which==20)guard.fault=MECH_STALE;
        if(which==21)pwm_mask=1;
        if(which==22)tick+=251;
        if(which==23)bus_diag.pending_stop_count=1;
        if(which==24)data[8]=0;
        if(which==25)Feed(1,0x35,2);
        uint32_t result=Command(61,data,9);
        if(which==18)
        {
            if(result)fprintf(stderr,"prepare result %u stop %u mask %u pending %u axis %u servo %u pose %u base %u\n",result,guard.stop_required,stopping_mask,pending_kind,axis.state,servo.state,pose.state,guard.base_stopped);
            CHECK(!result&&!guard.fault&&!guard.zero_valid&&guard.zero_version==1);
            CHECK(!enabled_mask&&!bus_diag.error&&!Servo_Reference(&servo,2));
            CHECK(axis.goal.id==7&&servo.goal.id==8&&pose.goal.id==9&&pose.child_id==10);
            CHECK(!MechPort_Output(frames,tick));
            CHECK(MechPort_Command(61,data,9,response,&size,tick)==COMM_OK&&!Frame_Get(response,4));
        }
        else CHECK(result&&guard.fault&&!guard.zero_valid);
        CHECK(!writes&&!starts&&!sends);
    }
    else if(which>=13) {
        Frame_Put(data+4,0x4D454348,4);CHECK(Command(48,data,8)==0);
        if(which!=15) {config[0].valid=config[1].valid=1;}
        CHECK(MechPort_Query(frames,tick)==1);
        CHECK(frames[0].id==0x500&&frames[0].data[0]==(which==15?0x42:0x35));
        if(which==13) {
            /* 模拟Bus_Send连续拒收，不能跳过当前反馈项。 */
            CHECK(MechPort_Query(frames,++tick)==1&&frames[0].data[0]==0x35);
            MechPort_QueryAccept();MechPort_QueryAccept();
            CHECK(!MechPort_Query(frames,++tick));
            MechPort_Sent(1,tick);tick+=10;
            CHECK(MechPort_Query(frames,tick)==1&&frames[0].data[0]==0x36);
        } else if(which==14||which==17) {
            MechPort_QueryAccept();
            if(which==17)tick=UINT32_MAX-5U;
            tick+=8;MechPort_Sent(1,tick);
            CHECK(!MechPort_Query(frames,tick+9));
            MechPort_Sent(1,tick+9); /* 轮查询完成不得重启轴冷却。 */
            CHECK(MechPort_Query(frames,tick+10)==1&&frames[0].data[0]==0x36);
        } else if(which==15) {
            MechPort_QueryAccept();MechPort_Sent(1,tick);
            config[0].valid=config[1].valid=1;tick+=10;
            CHECK(MechPort_Query(frames,tick)==1&&frames[0].data[0]==0x35);
        } else if(which==16) {
            MechPort_QueryAccept();MechPort_Stop(MECH_MANUAL,tick);
            CHECK(guard.fault&&!query_pending);
            CHECK(!MechPort_Query(frames,tick+9));
            CHECK(MechPort_Query(frames,tick+10)==1);
            CHECK(!MechPort_Output(frames,tick+10));
        } else CHECK(0);
        CHECK(!writes&&!starts&&!sends&&!guard.zero_valid);
    }
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

uint32_t HAL_TIM_PWM_Stop(int *timer, uint32_t channel) { (void)timer;(void)channel;return HAL_OK; }
