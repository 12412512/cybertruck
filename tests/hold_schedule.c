#define QUERY_SCHEDULE_MAIN PreviousScheduleMain
#include "mech_query_schedule.c"

static unsigned request_counter;
static void SessionPacket(unsigned type, unsigned value)
{
    Frame frame = {0};
    uint8_t wire[160], reply[160];
    frame.type = (uint8_t)type;
    frame.boot = 123;
    frame.session = 456;
    frame.request = ++request_counter;
    frame.ttl_ms = 250;
    frame.size = 4;
    Frame_Put(frame.data, value, 4);
    unsigned size = Frame_Encode(&frame, wire);
    for (unsigned i = 0; i < size; i++) Comm_Feed(wire[i], tick, tick);
    CHECK(Comm_Read(reply) > 0);
}

int main(int argc, char **argv)
{
    unsigned starvation = argc > 1 && atoi(argv[1]);
    unsigned active_at = 0, zero_at = 0, lost_at = 0;
    unsigned delay = argc > 1 && atoi(argv[1]) > 1 ? (unsigned)atoi(argv[1]) + 5U : 0;
    unsigned pending_address = 0, pending_code = 0, due = 0;
    unsigned axis_queries_after = 0, wheel_queries_after = 0;
    unsigned axis_stamps[8]={0},maximum_axis_age=0,maximum_wheel_age=0,maximum_status_age=0;
    unsigned zero_version = 0, final_fault = 0, final_zero = 0;
    uint8_t response[128]; uint16_t size = 0;
    Can_Refresh(); Comm_Init(123, tick); Base_Init();
    SessionPacket(COMM_OPEN, 1);
    CHECK(Comm_Session() == 456);
    for (tick = 0; tick < 8000; tick++)
    {
        if (occupied)
        {
            unsigned address = last.id >> 8, code = last.data[0];
            if(tick>=3700 && getenv("HOLD_TRACE"))fprintf(stderr,"%u tx ID%u/%x\n",tick,address,code);
            Done();
            if (code == 0x42) ConfigReply(address);
            else if (code != 0xFE && code != 0xF6)
            {
                CHECK(code == 0x35 || code == 0x36 || code == 0x3A || code == 0x33);
                CHECK(!pending_code); pending_address=address; pending_code=code; due=tick+delay;
            }
        }
        if (pending_code && tick >= due)
        {
            FeedbackReply(pending_address,pending_code);
            if(pending_address>=5){
                unsigned kind=pending_code==0x35?0:pending_code==0x36?1:pending_code==0x3A?2:3;
                axis_stamps[(pending_address-5)*4+kind]=tick;
            }
            if (active_at) { if (pending_address >= 5) axis_queries_after++; else wheel_queries_after++; }
            pending_code=0;
        }
        if (active_at && tick % 10 == 0)
        { int16_t rpm[4]={0}; CHECK(Bus_Speed(rpm)); }
        if (tick % 100 == 0) SessionPacket(COMM_HEART, tick / 100 + 1);
        if (tick == 1800)
        {
            uint8_t data[8] = {1,0,0,0,0x48,0x43,0x45,0x4D};
            CHECK(MechPort_Command(48,data,sizeof(data),response,&size,tick)==COMM_OK);
            CHECK(size==4 && Frame_Get(response,4)==0);
        }
        if (tick == 3500)
        {
            uint8_t data[9] = {2,0,0,0,0,0,0,0,1};
            CHECK(MechPort_Command(50,data,sizeof(data),response,&size,tick)==COMM_OK);
            if (delay == 8)
            {
                CHECK(size==4 && Frame_Get(response,4)!=0 && !executor.enabled_wheels);
                printf("8ms baseline feedback cannot establish zero; motion admission rejected\n");
                return 0;
            }
            CHECK(size==4 && Frame_Get(response,4)==0);
            zero_at=tick;
        }
        if (!starvation || (!active_at && tick < 4000)) Base_Run();
        else
        {
            /* The current host cannot request this handoff. Inject only the
             * would-be owner state; keep production Bus/Query/MechPort intact.
             * No Exec_Run or velocity command, physical motion not modeled. */
            Bus_Poll();
            if (diag_busy && Bus_Status()!=CAN_BUSY && Bus_Status()!=CAN_CANCELLING)
            {
                MechPort_Sent((uint8_t)(Bus_Status()==CAN_SENT),tick);
                Bus_Release(); diag_busy=0;
            }
            if (active_at) Query(tick);
            MechPort_Run(tick,0,0);
        }
        if (starvation && !active_at && tick >= 4000 && !diag_busy &&
            !bus_diag.pending_stop_count && (Bus_HoldWatch(1), Bus_Start()))
        {
            executor.mode=EXEC_READY;
            active_at=tick;
        }
        if (zero_at)
        {
            CHECK(MechPort_Command(58,0,0,response,&size,tick)==COMM_OK && size==112);
            final_fault=(unsigned)Frame_Get(response+12,4);
            zero_version=(unsigned)Frame_Get(response+16,4);
            final_zero=(unsigned)Frame_Get(response+20,4);
            if (starvation && active_at && !final_zero) { lost_at=tick; MechPort_Command(70,0,0,response,&size,tick); fprintf(stderr,"failure age %llu/%llu reason%llu\n",(unsigned long long)Frame_Get(response+116,4),(unsigned long long)Frame_Get(response+120,4),(unsigned long long)Frame_Get(response+12,4)); break; }
            if (!starvation) CHECK(final_zero && !final_fault);
        }
        if(active_at){
            for(unsigned i=0;i<8;i++){
                unsigned age=tick-axis_stamps[i];CHECK(age<250);
                if(age>maximum_axis_age)maximum_axis_age=age;
            }
            for(unsigned i=0;i<4;i++){
                const BusWheel *wheel=Bus_Wheels()+i;
                const MotorReply *samples[3]={&wheel->speed,&wheel->position,&wheel->status};
                for(unsigned k=0;k<3;k++){
                    unsigned age=tick-samples[k]->tick;
                    if(k<2){CHECK(age<250);if(age>maximum_wheel_age)maximum_wheel_age=age;}
                    else if(age>maximum_status_age)maximum_status_age=age;
                }
            }
        }
    }
    CHECK(!can_diag.errors && !bus_diag.query_timeouts);
    CHECK(Comm_Boot()==123 && Comm_Session()==456);
    if (starvation) fprintf(stderr,"active%u lost%u fault%u axis%u wheel%u\n",active_at,lost_at,final_fault,axis_queries_after,wheel_queries_after);
    if (starvation) CHECK(active_at && !lost_at && final_zero && !final_fault && axis_queries_after && wheel_queries_after);
    else CHECK(!active_at && final_zero && zero_version==1);
    printf("{\"case\":\"%s\",\"diagnostic_only\":true,\"hardware_access\":false,"
           "\"feature_complete\":false,\"zero_at_ms\":%u,\"base_active_injected_ms\":%u,"
           "\"zero_lost_at_ms\":%u,\"delay_ms\":%u,\"zero_version\":%u,\"zero_valid\":%u,"
           "\"mechanism_fault\":%u,\"axis_queries_after\":%u,\"wheel_queries_after\":%u,"
           "\"can_errors\":%u,\"query_timeouts\":%u,\"max_wheel_age_ms\":%u,\"max_axis_age_ms\":%u,\"max_status_age_ms\":%u,\"boot\":123,\"session\":456}\n",
           starvation?"active_six_node_hold":"idle_six_node_control",zero_at,active_at,lost_at,
           delay,zero_version,final_zero,final_fault,axis_queries_after,wheel_queries_after,
           (unsigned)can_diag.errors,(unsigned)bus_diag.query_timeouts,maximum_wheel_age,maximum_axis_age,maximum_status_age);
    return 0;
}
