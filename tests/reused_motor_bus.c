#include "bus_internal.h"
#include "bus_config.h"
#include "bus_config.h"
#include <assert.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
typedef struct { BusFrame frames[2]; uint8_t count; uint32_t tick; } Record;
static Record sent[256];
static BusFrame receive[256];
static unsigned sends, input, output, polls, reads, cancels;
static uint32_t now=100;
static CanState state;
static uint8_t available=1;
volatile CanDiag can_diag;
#define DRIVE_MAX_SKEW_MS 80U /* 保留旧夹具的边界用例，不改变正式阈值。 */
static const uint8_t address[4]={1,2,3,4};
uint32_t HAL_GetTick(void) { return now; }
uint8_t BoardButton(void) { return 0; }
void Can_Poll(void) { polls++; }
CanState Can_Status(void) { return state; }
uint8_t Can_Send(const BusFrame *frames,uint8_t count)
{
    if (!available || state==CAN_BUSY || state==CAN_CANCELLING) return 0;
    assert(count && count<=2 && sends<256);
    memcpy(sent[sends].frames,frames,count*sizeof(*frames));
    sent[sends].count=count; sent[sends].tick=now; sends++;
    state=CAN_BUSY; can_diag.tx_pending=1; can_diag.free_mailboxes=2; return 1;
}
void Can_Cancel(void) { cancels++; state=CAN_CANCELLING; can_diag.tx_isolated=1; }
void Can_Drop(void) { output=input; }
uint8_t Can_Read(BusFrame *frame)
{
    reads++; if (output==input) return 0;
    *frame=receive[output++]; return 1;
}
static void Finish(void)
{
    state=CAN_SENT; can_diag.tx_pending=0; can_diag.tx_isolated=0; can_diag.free_mailboxes=3;
}
static void FreeHardware(void)
{
    state=CAN_FAILED; can_diag.tx_pending=0; can_diag.tx_isolated=0; can_diag.free_mailboxes=3;
}
static void Drain(void) { BusFrame f; while (Bus_Read(&f)) {} }
static void Pump(void)
{
    now+=10;
    if (state==CAN_BUSY) Finish();
    else if (state==CAN_CANCELLING) FreeHardware();
    Bus_Poll(); Drain();
}
static void Stops(void)
{
    unsigned n;
    for (n=0;n<64 && (bus_diag.pending_stop_count || state==CAN_BUSY || state==CAN_CANCELLING);n++) Pump();
    assert(!bus_diag.pending_stop_count && !bus_diag.stop_pending && n<64);
}
static uint8_t Op(unsigned index) { assert(index<sends); return sent[index].frames[0].data[0]; }
static uint8_t Addr(unsigned index) { assert(index<sends); return (uint8_t)(sent[index].frames[0].id>>8); }
static int Rpm(unsigned index)
{
    const uint8_t *p=sent[index].frames[0].data;
    int v=(int)(((unsigned)p[2]<<8)|p[3]);
    return p[1]?-v:v;
}
static unsigned InitialQueries(unsigned next)
{
    unsigned base=sends,i,index;
    /* 新动作先刷新最旧反馈；完成预算后才允许首轮速度报文。 */
    for(i=0;i<BUS_QUERIES_PER_BATCH;i++) {
        if(i) now+=BUS_QUERY_INTERVAL_MS;
        Bus_Poll(); index=(next+i)%8U;
        assert(sends==base+i+1U && Op(base+i)==((index&1U)?0x36:0x35) && Addr(base+i)==index/2U+1U);
        Finish(); Bus_Poll();
    }
    return base+BUS_QUERIES_PER_BATCH;
}
static void Reply(uint8_t addr,uint8_t code,uint8_t result)
{
    BusFrame *f=&receive[input++];
    assert(input<=256); memset(f,0,sizeof(*f));
    f->id=(uint32_t)addr<<8; f->tick=now; f->len=3;
    f->data[0]=code; f->data[1]=result; f->data[2]=MOTOR_CHECK_BYTE;
}
static uint8_t Build(uint8_t addr,uint8_t op,BusFrame f[2])
{
    return Drive_Build(addr,op,0,12,0,0x12345678U,2,1,f);
}
static void Config(uint8_t addr)
{
    uint8_t raw[32]={0},packet;
    raw[0]=0x42; raw[1]=0x21; raw[2]=0x15; raw[3]=0x19;
    raw[8]=16; raw[19]=addr; raw[31]=MOTOR_CHECK_BYTE;
    for (packet=0;packet<5;packet++) {
        BusFrame *f=&receive[input++]; uint8_t n=packet==4?3:7;
        memset(f,0,sizeof(*f)); f->id=((uint32_t)addr<<8)|packet; f->tick=now;
        f->len=(uint8_t)(n+1); f->data[0]=0x42;
        memcpy(&f->data[1],&raw[1+packet*7],n);
    }
}
static void TestSetup(void)
{
    uint8_t bad[4]={1,1,3,4};
    Bus_Setup(bad); assert(!Bus_Start() && !sends);
    Bus_Setup(address); assert(Bus_Start() && !sends);
    assert(!Bus_Rearm() && bus_diag.active);
    Bus_End(); assert(Bus_Rearm());
    puts("PASS 0: setup sends nothing; invalid addresses and active rearm refused");
}
static void TestLatest(void)
{
    int16_t a[4]={10,-20,30,-40},b[4]={11,-22,33,-44},c[4]={100,200,300,400};
    unsigned i,base,query;
    assert(Bus_Start()); assert(Bus_Speed(a) && Bus_Speed(b));
    assert(bus_diag.replaced==1); base=InitialQueries(0);
    assert(sends==base+1U && Rpm(base)==11);
    assert(Bus_Speed(c));
    for(i=0;i<3;i++) Pump();
    for(i=0;i<4;i++) {
        assert(Op(base+i)==0xF6 && Addr(base+i)==i+1 && Rpm(base+i)==b[i]);
        assert(!sent[base+i].frames[0].data[4] && !sent[base+i].frames[0].data[5]);
    }
    Pump(); assert(bus_diag.speed_sent_sequence==2 && bus_diag.speed_span_ms==40);
    for(i=0;i<BUS_QUERIES_PER_BATCH;i++) {
        query=(BUS_QUERIES_PER_BATCH+i)%8U;
        assert(sends==base+5U+i && Op(base+4U+i)==((query&1U)?0x36:0x35) && Addr(base+4U+i)==query/2U+1U);
        if(i+1U<BUS_QUERIES_PER_BATCH) {
            Finish(); Bus_Poll(); assert(sends==base+5U+i);
            now+=BUS_QUERY_INTERVAL_MS-1U; Bus_Poll(); assert(sends==base+5U+i);
            now++; Bus_Poll();
        } else Pump();
    }
    assert(sends==base+5U+BUS_QUERIES_PER_BATCH && Op(base+4U+BUS_QUERIES_PER_BATCH)==0xF6 &&
        Rpm(base+4U+BUS_QUERIES_PER_BATCH)==100);
    puts("PASS 1: latest snapshot replaces pending only; four wheel batch stays contiguous, then bounded query budget precedes next batch");
}
static void TestFd(void)
{
    BusFrame frames[2]; uint8_t count=Build(7,0xFD,frames);
    assert(count==2 && Bus_Send(frames,count)); Bus_Poll();
    assert(sends==1 && sent[0].count==2 && !memcmp(frames,sent[0].frames,sizeof(frames)));
    Finish(); Bus_Poll(); assert(Bus_Status()==CAN_SENT && !Bus_Start());
    Reply(7,0xFD,2); Bus_Poll(); Drain(); assert(!Bus_Start());
    Bus_Release(); assert(!Bus_Start() && bus_diag.motion_uncertain);
    count=Build(7,0x35,frames); assert(Bus_Send(frames,count)); Bus_Poll(); Finish(); Bus_Poll(); Bus_Release();
    count=Build(7,0xF3,frames); assert(!Bus_Send(frames,count));
    count=Build(7,0xFD,frames); assert(!Bus_Send(frames,count));
    Reply(7,0xFD,0x9F); Bus_Poll(); Drain(); assert(bus_diag.motion_uncertain && !Bus_Start());
    assert(Bus_Rearm() && !bus_diag.motion_uncertain && Bus_Start());
    puts("PASS 2: FD two frames sent once unchanged; ACK/9F/Release never rearm motion");
}
static void TestStop(void)
{
    int16_t rpm[4]={1,2,3,4}; unsigned before,n;
    assert(Bus_Start() && Bus_Speed(rpm)); (void)InitialQueries(0); before=sends;
    assert(Bus_Speed(rpm)); Bus_Stop(); assert(cancels==1 && bus_diag.stop_pending==15);
    now+=100; Bus_Poll(); assert(sends==before && bus_diag.pending_stop_count==4);
    assert(!Bus_Rearm() && !Bus_Start());
    FreeHardware(); Bus_Poll(); Stops();
    for(n=before;n<sends;n++) assert(Op(n)==0xFE);
    assert(bus_diag.locked && Bus_Rearm()); before=sends;
    assert(Bus_Start()); Bus_Poll(); assert(sends==before+1U && Op(before)==0x35);
    puts("PASS 3: Stop survives abort timeout; hardware release sends only retained stops");
}
static void TestFault(void)
{
    int16_t rpm[4]={1,2,3,4}; unsigned before,n;
    assert(Bus_Start() && Bus_Speed(rpm)); (void)InitialQueries(0); assert(Bus_Speed(rpm)); before=sends;
    FreeHardware(); Bus_Poll(); assert(bus_diag.locked && (bus_diag.error&BUS_ERROR_CAN));
    Stops(); for(n=before;n<sends;n++) assert(Op(n)==0xFE);
    assert(!Bus_Speed(rpm)); assert(Bus_Rearm());
    assert(Bus_Start()); before=sends; Bus_Poll(); assert(sends==before+1U && Op(before)==0x35);
    puts("PASS 4: CAN failure locks and clears both current and pending speed without replay");
}
static void TestQueries(void)
{
    unsigned i,last=0,q=0,wheel=0,batches=0,gap_queries=0; int16_t rpm[4]={1,1,1,1};
    assert(Bus_Start());
    Bus_Poll(); assert(sends==1 && Op(0)==0x35);
    Finish(); Bus_Poll(); assert(sends==1);
    now+=BUS_QUERY_INTERVAL_MS-1U; Bus_Poll(); assert(sends==1);
    now++; Bus_Poll(); assert(Op(1)==0x36 && Addr(1)==1);
    Finish(); Bus_Poll(); Bus_End(); assert(Bus_Rearm() && Bus_Start());
    for(i=0;i<80;i++) { assert(Bus_Speed(rpm)); Pump(); }
    assert(bus_diag.query_count>=8);
    for(i=2;i<sends;i++) {
        if(Op(i)==0x35 || Op(i)==0x36) {
            assert(!wheel && ++gap_queries<=BUS_QUERIES_PER_BATCH);
            assert(Addr(i)==(q/2U)%4U+1U && Op(i)==((q&1U)?0x36:0x35)); q++;
            if(last) assert(sent[i].tick-sent[last].tick>=BUS_QUERY_INTERVAL_MS);
            last=i;
        } else {
            assert(Op(i)==0xF6 && Addr(i)==wheel+1U);
            if(!wheel) {
                assert(gap_queries==BUS_QUERIES_PER_BATCH);
                gap_queries=0;
            }
            if(++wheel==4U) { wheel=0; batches++; }
        }
    }
    assert(last && q>=8 && batches>=4);
    printf("PASS 5: queries staggered by %lu ms, bounded to %lu between intact four-wheel batches; both streams stay live\n",
        (unsigned long)BUS_QUERY_INTERVAL_MS,(unsigned long)BUS_QUERIES_PER_BATCH);
}
static void TestExclusive(void)
{
    BusFrame frames[2]; uint8_t count=Build(1,0x42,frames); unsigned before;
    assert(Bus_Send(frames,count)); assert(!Bus_Start()); Bus_Poll(); Finish(); Bus_Poll();
    assert(Bus_Status()==CAN_SENT && !Bus_Start()); Bus_Release(); assert(Bus_Start());
    count=Build(1,0x35,frames); assert(!Bus_Send(frames,count));
    count=Build(7,0xFE,frames); before=sends; assert(Bus_Send(frames,count));
    assert(bus_diag.locked && bus_diag.stop_pending==15 && bus_diag.pending_stop_count==5);
    Stops();
    for(;before<sends;before++) assert(Op(before)==0xFE);
    assert(Bus_Status()==CAN_SENT); Bus_Release(); assert(Bus_Rearm());
    puts("PASS 6: diagnostic waits retain exclusivity; FE preempts drive and stops all four wheels");
}
static void TestReceive(void)
{
    BusFrame frame,request[2]; unsigned n,base=reads;
    assert(Bus_Send(request,Build(1,0x42,request))); Bus_Poll(); Finish(); Bus_Poll();
    assert(Bus_Status()==CAN_SENT && !Bus_Start());
    Config(1); Bus_Poll(); assert(Bus_Wheels()[0].config.valid && Bus_Wheels()[0].config.microsteps==16);
    for(n=0;n<5;n++) { assert(Bus_Read(&frame)); assert((frame.id&255U)==n && frame.data[0]==0x42); }
    assert(!Bus_Read(&frame));
    Config(1); receive[input-1].data[3]=0; Bus_Poll(); assert(!Bus_Wheels()[0].config.valid); Drain();
    Reply(1,0x35,1); receive[input-1].len=5; receive[input-1].data[2]=0;
    receive[input-1].data[3]=12; receive[input-1].data[4]=MOTOR_CHECK_BYTE;
    Reply(2,0x36,0); receive[input-1].len=7; receive[input-1].data[2]=0;
    receive[input-1].data[3]=1; receive[input-1].data[4]=0; receive[input-1].data[5]=0;
    receive[input-1].data[6]=MOTOR_CHECK_BYTE;
    Reply(3,0x3A,5); Bus_Poll(); Drain();
    assert(Bus_Wheels()[0].speed.valid && Bus_Wheels()[0].speed.value==-12 && Bus_Wheels()[0].speed.tick==now);
    assert(Bus_Wheels()[1].position.valid && Bus_Wheels()[1].position.value==65536 && Bus_Wheels()[2].status.value==5);
    for(n=0;n<20;n++) Reply(1,0xF6,2);
    base=reads; Bus_Poll(); assert(reads-base==CAN_RING_SIZE && bus_diag.ack_observed_mask==1);
    assert(bus_diag.speed_sent_sequence==0 && Bus_Status()==CAN_SENT && !bus_diag.rx_overflow);
    Bus_Poll(); assert(bus_diag.rx_overflow==4 && (bus_diag.error&BUS_ERROR_RX));
    for(n=0;n<CAN_RING_SIZE;n++) {
        assert(Bus_Read(&frame) && frame.id==0x100 && frame.data[0]==0xF6 && frame.data[1]==2);
    }
    assert(!Bus_Read(&frame) && !Bus_Start()); Bus_Release();
    puts("PASS 7: exclusive diagnostic preserves config packet order, invalidates bad config, and bounds inbox to 16 frames with 4 overflow");
}
static void TestTelemetry(void)
{
    BusFrame frame;
    unsigned n,w;
    int64_t speeds[4]={0},positions[4]={0};
    assert(Bus_Start());
    /* 不调用Drain或BusRead清垃圾：整车反馈只能更新轮状态，不占诊断收件箱。 */
    for(n=0;n<64;n++) {
        uint8_t addr=(uint8_t)((n/2U)%4U+1U),code=(uint8_t)((n&1U)?0x36:0x35);
        BusFrame *f;
        now++;
        Reply(addr,code,0); f=&receive[input-1];
        if(code==0x35) {
            f->len=5; f->data[2]=0; f->data[3]=(uint8_t)(n+10U); f->data[4]=MOTOR_CHECK_BYTE;
            speeds[addr-1U]=(int64_t)(n+10U);
        } else {
            f->len=7; f->data[2]=0; f->data[3]=0; f->data[4]=3;
            f->data[5]=(uint8_t)(n+100U); f->data[6]=MOTOR_CHECK_BYTE;
            positions[addr-1U]=(int64_t)(768U+n+100U);
        }
        if(n==63U) f->tick=now-(DRIVE_MAX_SKEW_MS-1U); /* RR位置最旧，其余七项仍较新。 */
        if(state==CAN_BUSY) Finish();
        Bus_Poll();
        assert(bus_diag.active && !bus_diag.locked && !bus_diag.error && !bus_diag.rx_overflow);
    }
    for(w=0;w<4;w++) {
        assert(Bus_Wheels()[w].speed.valid && Bus_Wheels()[w].speed.value==speeds[w]);
        assert(Bus_Wheels()[w].position.valid && Bus_Wheels()[w].position.value==positions[w]);
    }
    assert(!Bus_Read(&frame) && !can_diag.rx_overflow);
    Bus_End(); if(state==CAN_BUSY) Finish(); Bus_Poll();
    n=sends; assert(Bus_Start()); Bus_Poll();
    assert(sends==n+1U && Op(n)==0x36 && Addr(n)==4);
    puts("PASS 15: 64 telemetry replies cause no inbox/overflow/lock; next action refreshes oldest RR position instead of resetting to FL");
}
static void TestRejectAck(void)
{
    int16_t rpm[4]={1,1,1,1}; unsigned before;
    assert(Bus_Start() && Bus_Speed(rpm)); (void)InitialQueries(0); before=sends;
    Reply(1,0xF6,0xE2); Bus_Poll(); Drain();
    assert(bus_diag.locked && (bus_diag.error&BUS_ERROR_DRIVER) && bus_diag.stop_pending==15 && cancels);
    Stops(); for(;before<sends;before++) assert(Op(before)==0xFE);
    puts("PASS 8: observed negative driver ACK faults and retains all four stops");
}
static void TestAsyncError(int overflow)
{
    unsigned before;
    assert(Bus_Start()); before=sends;
    if(overflow) can_diag.rx_overflow++; else can_diag.errors++;
    Bus_Poll(); assert(bus_diag.locked && bus_diag.stop_pending==15 && cancels);
    assert(bus_diag.error&(overflow?BUS_ERROR_RX:BUS_ERROR_CAN));
    Stops(); for(;before<sends;before++) assert(Op(before)==0xFE);
    printf("PASS %d: CAN %s during idle slot faults without a transmit job\n",overflow?10:9,overflow?"RX overflow":"error");
}
static void TestTwoStops(void)
{
    BusFrame frames[2]; unsigned i,mask=0;
    assert(Bus_Send(frames,Build(7,0xFE,frames))); Bus_Poll(); assert(sends==1 && Addr(0)==7);
    assert(Bus_Send(frames,Build(8,0xFE,frames))); Bus_Cancel(); now+=100; Bus_Poll();
    assert(bus_diag.pending_stop_count==2 && !Bus_Rearm());
    FreeHardware(); Bus_Poll(); Stops();
    for(i=1;i<sends;i++) { assert(Op(i)==0xFE); if(Addr(i)==7) mask|=1; if(Addr(i)==8) mask|=2; }
    assert(mask==3 && Bus_Status()==CAN_SENT); Bus_Release(); assert(Bus_Rearm());
    puts("PASS 11: two different diagnostic Stop addresses survive replacement and cancellation");
}
static void TestDiagStopFault(void)
{
    BusFrame frames[2]; unsigned before;
    assert(Bus_Send(frames,Build(7,0xFE,frames))); Bus_Poll(); before=sends;
    FreeHardware(); Bus_Poll(); assert(Bus_Status()==CAN_FAILED && bus_diag.pending_stop_count==1);
    assert(!bus_diag.stop_pending); Bus_Release(); assert(!Bus_Rearm());
    now+=9; Bus_Poll(); assert(sends==before); now++; Bus_Poll(); assert(sends==before+1 && Addr(before)==7);
    Finish(); Bus_Poll(); assert(Bus_Status()==CAN_SENT && !bus_diag.pending_stop_count);
    assert(Bus_Rearm());
    puts("PASS 12: failed single diagnostic Stop retries safely after Release without extra wheel stops");
}
static void TestWait(int diagnostic)
{
    int16_t rpm[4]={1,1,1,1}; BusFrame frames[2]; unsigned before;
    if(diagnostic) assert(Bus_Send(frames,Build(7,0x35,frames)));
    else { assert(Bus_Start()); (void)InitialQueries(0); assert(Bus_Speed(rpm)); }
    available=0; before=sends; Bus_Poll(); now+=49; Bus_Poll(); assert(!bus_diag.locked);
    now++; Bus_Poll(); assert(bus_diag.locked && bus_diag.wait_ms>=50 && cancels && sends==before);
    if(diagnostic) { assert(Bus_Status()==CAN_FAILED && !bus_diag.pending_stop_count); Bus_Release(); }
    else assert(bus_diag.stop_pending==15);
    available=1; FreeHardware(); Bus_Poll(); Stops();
    assert(Bus_Rearm());
    printf("PASS %d: mailbox wait has finite failure state for %s\n",diagnostic?14:13,diagnostic?"diagnostic":"four wheel batch");
}
static const uint8_t finite_dirs[4]={0,1,1,0};
static const uint32_t finite_steps[4]={32,33,34,35};
static void FiniteStops(unsigned before)
{
    unsigned i,mask=0,after;
    assert(bus_diag.pending_stop_count==4 && bus_diag.stop_pending==15);
    if(state==CAN_CANCELLING) FreeHardware();
    Bus_Poll(); Stops();
    for(i=before;i<sends;i++) {
        assert(Op(i)==0xFE && Addr(i)>=1 && Addr(i)<=4);
        mask|=1U<<(Addr(i)-1U);
    }
    assert(mask==15 && bus_diag.locked && bus_diag.motion_uncertain);
    after=sends; Pump(); Pump(); assert(sends==after);
    assert(!Bus_Finite(finite_dirs,10,finite_steps));
    Bus_Release(); assert(Bus_Rearm());
}
static void TestFiniteAdmission(void)
{
    uint8_t dirs[4],bad_address[4]={1,1,3,4};
    uint32_t steps[4]; BusFrame frames[2]; unsigned i,before;
    memcpy(dirs,finite_dirs,sizeof(dirs)); memcpy(steps,finite_steps,sizeof(steps));
    assert(!Bus_Finite(0,10,steps) && !Bus_Finite(dirs,10,0));
    assert(!Bus_Finite(dirs,0,steps) && !Bus_Finite(dirs,MOTOR_MAX_COMMAND_RPM+1U,steps));
    for(i=0;i<4;i++) {
        dirs[i]=2; assert(!Bus_Finite(dirs,10,steps)); dirs[i]=finite_dirs[i];
        steps[i]=0; assert(!Bus_Finite(dirs,10,steps)); steps[i]=finite_steps[i];
    }
    Bus_Setup(bad_address); assert(!Bus_Finite(dirs,10,steps)); Bus_Setup(address);
    bus_diag.locked=1; assert(!Bus_Finite(dirs,10,steps)); bus_diag.locked=0;
    bus_diag.motion_uncertain=1; assert(!Bus_Finite(dirs,10,steps)); bus_diag.motion_uncertain=0;
    bus_diag.error=BUS_ERROR_CAN; assert(!Bus_Finite(dirs,10,steps)); bus_diag.error=0;
    can_diag.free_mailboxes=2; assert(!Bus_Finite(dirs,10,steps)); can_diag.free_mailboxes=3;
    can_diag.tx_pending=1; assert(!Bus_Finite(dirs,10,steps)); can_diag.tx_pending=0;
    can_diag.tx_isolated=1; assert(!Bus_Finite(dirs,10,steps)); can_diag.tx_isolated=0;
    state=CAN_BUSY; assert(!Bus_Finite(dirs,10,steps));
    state=CAN_CANCELLING; assert(!Bus_Finite(dirs,10,steps)); state=CAN_IDLE;
    assert(!sends && !bus_diag.finite_sequence && !bus_diag.motion_uncertain && !bus_diag.queue_depth);
    assert(Bus_Start() && !Bus_Finite(dirs,10,steps)); Bus_End();
    assert(Bus_Send(frames,Build(1,0x35,frames)) && !Bus_Finite(dirs,10,steps));
    Bus_Poll(); Finish(); Bus_Poll(); Bus_Release();
    before=sends; assert(Bus_Finite(dirs,10,steps));
    assert(sends==before && bus_diag.motion_uncertain && bus_diag.queue_depth==4 &&
           bus_diag.finite_sequence==1 && !bus_diag.finite_sent_mask && Bus_Status()==CAN_BUSY);
    assert(!Bus_Finite(dirs,10,steps) && !Bus_Start() && !Bus_Rearm());
    Bus_Cancel(); FiniteStops(before);
    puts("PASS 16: finite admission validates all four targets atomically and rejects busy, faulted or occupied hardware");
}
static void TestFiniteSequence(void)
{
    uint8_t dirs[4],other[4]={5,6,7,8}; uint32_t steps[4];
    BusFrame frames[2],received; unsigned i,before;
    memcpy(dirs,finite_dirs,sizeof(dirs)); memcpy(steps,finite_steps,sizeof(steps));
    assert(Bus_Finite(dirs,10,steps));
    memset(dirs,0,sizeof(dirs)); memset(steps,0,sizeof(steps)); Bus_Setup(other);
    Bus_Poll(); assert(sends==1 && !bus_diag.finite_sent_mask);
    Reply(4,0xF3,2); Bus_Poll();
    assert(Bus_Read(&received) && received.id==0x400 && received.data[0]==0xF3);
    for(i=0;i<4;i++) {
        assert(sends==i+1U && Addr(i)==address[i] && Op(i)==0xFD && sent[i].count==2);
        assert(Drive_Build(address[i],0xFD,finite_dirs[i],10,0,finite_steps[i],2,0,frames)==2);
        assert(!memcmp(frames,sent[i].frames,sizeof(frames)));
        assert(bus_diag.finite_sent_mask==((1U<<i)-1U) && Bus_Status()==CAN_BUSY);
        Bus_Poll(); assert(bus_diag.finite_sent_mask==((1U<<i)-1U));
        Finish(); Bus_Poll();
        assert(bus_diag.finite_sent_mask==((1U<<(i+1U))-1U));
        assert(bus_diag.finite_sent_sequence==(i==3U?1U:0U));
    }
    assert(sends==4 && Bus_Status()==CAN_SENT && !bus_diag.queue_depth && !Bus_Rearm());
    Bus_Release(); Bus_Setup(other);
    assert(!Bus_Finite(finite_dirs,10,finite_steps) && !Bus_Start());
    assert(!Bus_Send(frames,Build(1,0xFD,frames)) && !Bus_Send(frames,Build(1,0xF3,frames)));
    before=sends; assert(Bus_Send(frames,Build(1,0x35,frames))); Bus_Poll();
    assert(sends==before+1 && Addr(before)==1 && Op(before)==0x35);
    Finish(); Bus_Poll(); Bus_Release(); assert(Bus_Rearm());
    Bus_Setup(other); assert(Bus_Finite(finite_dirs,10,finite_steps)); Bus_Poll();
    assert(Addr(sends-1)==5 && bus_diag.finite_sequence==2 && !bus_diag.finite_sent_mask);
    puts("PASS 17: finite batch latches four addresses and goals, credits only complete TXOK, retains ownership and allows later queries");
}
static void TestFiniteFailure(void)
{
    unsigned before;
    assert(Bus_Finite(finite_dirs,10,finite_steps)); Bus_Poll(); Finish(); Bus_Poll();
    assert(sends==2 && bus_diag.finite_sent_mask==1 && !bus_diag.finite_sent_sequence);
    /* 第二轮仅一个分包已完成，整个两帧事务失败不能计入sent_mask。 */
    can_diag.tx_index=1; can_diag.tx_count=2; before=sends; FreeHardware(); Bus_Poll();
    assert(Bus_Status()==CAN_FAILED && bus_diag.finite_sent_mask==1 && !bus_diag.finite_sent_sequence);
    assert(bus_diag.error&BUS_ERROR_CAN);
    /* 首条FE也失败，保留四个锁存目标，后续只能重试FE。 */
    assert(Op(before)==0xFE); FreeHardware(); Bus_Poll();
    assert(bus_diag.stop_retries && bus_diag.pending_stop_count==4);
    FiniteStops(before);
    assert(bus_diag.finite_sent_mask==1 && !bus_diag.finite_sent_sequence);
    puts("PASS 18: partial second-wheel FD failure never credits it or replays remaining FD; all four FE survive a Stop failure");
}
static void TestFiniteCancel(void)
{
    unsigned before;
    assert(Bus_Finite(finite_dirs,10,finite_steps)); Bus_Poll(); before=sends;
    Bus_Cancel(); assert(cancels==1 && Bus_Status()==CAN_CANCELLING);
    now+=100; Bus_Poll(); assert(sends==before && bus_diag.pending_stop_count==4);
    FiniteStops(before); assert(!bus_diag.finite_sent_mask && !bus_diag.finite_sent_sequence);
    puts("PASS 19: cancelling an in-flight finite batch cancels all remaining FD and retains FE for even unsent wheels");
}
static void TestFiniteStop(void)
{
    unsigned i,before; uint8_t other[4]={5,6,7,8};
    assert(Bus_Finite(finite_dirs,10,finite_steps)); Bus_Poll();
    for(i=0;i<4;i++) { Finish(); Bus_Poll(); }
    assert(bus_diag.finite_sent_mask==15 && bus_diag.finite_sent_sequence==1);
    Bus_Release(); Bus_Setup(other); before=sends; Bus_Stop();
    assert(bus_diag.pending_stop_count==4 && bus_diag.stop_pending==15);
    FiniteStops(before);
    assert(bus_diag.finite_sent_sequence==1 && bus_diag.finite_sent_mask==15);
    puts("PASS 20: Stop after complete TX and Release still targets the locked group, not a rejected replacement setup");
}
static void TestFiniteRejectAck(uint8_t result)
{
    unsigned before;
    assert(Bus_Finite(finite_dirs,10,finite_steps)); Bus_Poll(); before=sends;
    Reply(4,0xFD,result); Bus_Poll();
    assert((bus_diag.error&BUS_ERROR_DRIVER) && cancels && !bus_diag.finite_sent_mask);
    FiniteStops(before); assert(!bus_diag.finite_sent_sequence);
    printf("PASS %u: finite negative ACK %02X from a group member faults the entire group\n",21U+(result==0xEE?1U:result==0x12?2U:result==0x22?3U:0U),result);
}
static void TestFiniteAsync(int overflow)
{
    unsigned before;
    assert(Bus_Finite(finite_dirs,10,finite_steps)); before=sends;
    if(overflow) can_diag.rx_overflow++; else can_diag.errors++;
    Bus_Poll(); assert(bus_diag.error&(overflow?BUS_ERROR_RX:BUS_ERROR_CAN));
    FiniteStops(before); assert(!bus_diag.finite_sent_mask && !bus_diag.finite_sent_sequence);
    printf("PASS %d: finite CAN %s before first FD retains all four Stops without sending FD\n",25+overflow,overflow?"RX overflow":"error");
}
static void TestFiniteInbox(void)
{
    unsigned i,before;
    assert(Bus_Finite(finite_dirs,10,finite_steps)); Bus_Poll(); before=sends;
    for(i=0;i<CAN_RING_SIZE;i++) Reply(1,0xFD,2);
    Bus_Poll(); assert(!bus_diag.rx_overflow && !bus_diag.error);
    Reply(2,0xFD,2); Bus_Poll();
    assert(bus_diag.rx_overflow==1 && (bus_diag.error&BUS_ERROR_RX) && cancels);
    Drain(); FiniteStops(before);
    assert(!bus_diag.finite_sent_mask && !bus_diag.finite_sent_sequence);
    puts("PASS 27: finite diagnostic inbox overflow aborts the batch and retains all group Stops");
}
static void TestFiniteWait(void)
{
    unsigned before;
    assert(Bus_Finite(finite_dirs,10,finite_steps)); available=0; before=sends;
    Bus_Poll(); now+=49; Bus_Poll(); assert(!bus_diag.locked);
    now++; Bus_Poll();
    assert(Bus_Status()==CAN_FAILED && (bus_diag.error&BUS_ERROR_CAN) && cancels && sends==before);
    available=1; FiniteStops(before);
    assert(!bus_diag.finite_sent_mask && !bus_diag.finite_sent_sequence);
    puts("PASS 28: finite mailbox wait timeout aborts before FD and retains all four Stops");
}
static void TestFiniteStopNegative(uint8_t result,unsigned test)
{
    unsigned i,before; BusFrame response;
    assert(Bus_Finite(finite_dirs,10,finite_steps)); Bus_Poll();
    for(i=0;i<4;i++) { Finish(); Bus_Poll(); }
    assert(bus_diag.finite_sent_sequence==1 && bus_diag.finite_sent_mask==15);
    Bus_Release(); before=sends; Bus_Stop(); FreeHardware(); Bus_Poll();
    assert(sends==before+1U && Op(before)==0xFE);
    Reply(1,0xFE,result); Bus_Poll();
    assert(Bus_Read(&response) && response.id==0x100 && response.data[0]==0xFE && response.data[1]==result);
    assert(!Bus_Read(&response) && (bus_diag.error&BUS_ERROR_DRIVER) && bus_diag.pending_stop_count==4);
    assert(Bus_Status()!=CAN_SENT && cancels>=2);
    FiniteStops(before);
    assert(bus_diag.finite_sent_sequence==1 && bus_diag.finite_sent_mask==15);
    printf("PASS %u: group FE negative ACK %02X survives Release and faults/retries all four Stops\n",test,result);
}
static void TestFiniteStopInbox(void)
{
    unsigned i,before; BusFrame response,*telemetry;
    assert(Bus_Finite(finite_dirs,10,finite_steps)); Bus_Poll();
    Reply(1,0xFE,2); Bus_Poll();
    assert(Bus_Read(&response) && response.id==0x100 && !Bus_Read(&response));
    for(i=0;i<4;i++) { Finish(); Bus_Poll(); }
    Bus_Release();
    Reply(2,0xFE,2); Reply(7,0xFE,2); Reply(3,0x35,0); telemetry=&receive[input-1];
    telemetry->len=5; telemetry->data[2]=0; telemetry->data[3]=7; telemetry->data[4]=MOTOR_CHECK_BYTE;
    Bus_Poll();
    assert(Bus_Read(&response) && response.id==0x200 && response.data[0]==0xFE && !Bus_Read(&response));
    assert(Bus_Wheels()[2].speed.valid && Bus_Wheels()[2].speed.value==7 && !bus_diag.error);
    before=sends; Bus_Stop(); FiniteStops(before);
    puts("PASS 33: finite group FE is copied once with owner, survives Release, and does not retain unrelated FE or telemetry");
}
static void TestLegacyStopInbox(void)
{
    BusFrame frames[2],response;
    assert(Bus_Send(frames,Build(1,0xFE,frames))); Bus_Poll(); Bus_Release();
    Reply(1,0xFE,0xE2); Bus_Poll();
    assert(!Bus_Read(&response) && !(bus_diag.error&BUS_ERROR_DRIVER));
    Finish(); Bus_Poll(); assert(Bus_Status()==CAN_SENT && !bus_diag.pending_stop_count && Bus_Rearm());
    puts("PASS 34: non-finite FE inbox and negative-response handling preserve legacy behavior");
}
int main(int argc,char **argv)
{
    int test;
    assert(argc==2); test=atoi(argv[1]); can_diag.free_mailboxes=3; Bus_Setup(address);
    switch(test) {
    case 0: TestSetup(); break; case 1: TestLatest(); break; case 2: TestFd(); break;
    case 3: TestStop(); break; case 4: TestFault(); break; case 5: TestQueries(); break;
    case 6: TestExclusive(); break; case 7: TestReceive(); break; case 8: TestRejectAck(); break;
    case 9: TestAsyncError(0); break; case 10: TestAsyncError(1); break;
    case 11: TestTwoStops(); break; case 12: TestDiagStopFault(); break;
    case 13: TestWait(0); break; case 14: TestWait(1); break; case 15: TestTelemetry(); break;
    case 16: TestFiniteAdmission(); break; case 17: TestFiniteSequence(); break;
    case 18: TestFiniteFailure(); break; case 19: TestFiniteCancel(); break; case 20: TestFiniteStop(); break;
    case 21: TestFiniteRejectAck(0xE2); break; case 22: TestFiniteRejectAck(0xEE); break;
    case 23: TestFiniteRejectAck(0x12); break; case 24: TestFiniteRejectAck(0x22); break;
    case 25: TestFiniteAsync(0); break; case 26: TestFiniteAsync(1); break;
    case 27: TestFiniteInbox(); break; case 28: TestFiniteWait(); break;
    case 29: TestFiniteStopNegative(0xE2,29); break; case 30: TestFiniteStopNegative(0xEE,30); break;
    case 31: TestFiniteStopNegative(0x12,31); break; case 32: TestFiniteStopNegative(0x22,32); break;
    case 33: TestFiniteStopInbox(); break; case 34: TestLegacyStopInbox(); break; default: assert(0);
    }
    if(test) assert(polls); return 0;
}
