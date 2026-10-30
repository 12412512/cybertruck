/* 真实Track：TXOK之前不能消耗小步时间；票据/航向不能由旧状态放行。 */
#define main closure_main
#include "micro_closure.c"
#undef main

static void Begin(Track *t, TrackObservation *o, double tolerance)
{
    TrackConfig c=Configuration(10,.05);TrackGoal g={0};
    c.yaw_tol_rad=tolerance;assert(Track_Init(t,&c));
    memset(o,0,sizeof(*o));Fresh(o,100);
    g.id=1;g.timeout_ms=10000;g.reference.kind=1;g.reference.y_m=.005f;
    g.reference.linear=(CurveLimit){.01f,.05f,.2f};g.reference.angular=(CurveLimit){.1f,.3f,1};
    assert(Track_Start(t,&g,o,100)==1);
}

int main(void)
{
    Track t;TrackObservation o;unsigned tick=100,token=0;
    /* 提高起步幅值仍受配置限幅；近终点缩短时间，纯转向不提速。 */
    for(unsigned mode=0;mode<5;mode++)
    {
        Begin(&t,&o,.0034906585);
        if(mode==1)t.config.max_rpm=1;
        if(mode==2)t.config.correction_m_s=.006;
        if(mode==3)o.pose.y_m=.004;
        if(mode==4)o.yaw_rad=.02;
        Fresh(&o,110);assert(Track_Step(&t,&o,110)>=0);
        double expected=(mode==0||mode==3)?2:1;
        for(unsigned i=0;i<4;i++)assert(fabs(t.micro_rpm[i])==expected);
        assert(t.micro_duration_ms<=500);
        if(mode==3)assert(t.micro_duration_ms<100);
        if(mode==4)assert(t.micro_duration_ms<=200);
    }
    Begin(&t,&o,.0034906585);
    while(!token){tick+=10;Fresh(&o,tick);assert(Track_Step(&t,&o,tick)>=0);token=Track_Token(&t);}
    unsigned requested=tick;
    assert(!Track_Sent(&t,2,token,tick));
    assert(!Track_Sent(&t,1,token+100,tick));
    assert(!Track_Sent(&t,1,token,requested-1));
    for(;tick<requested+100;){tick+=10;Fresh(&o,tick);assert(Track_Step(&t,&o,tick)>=0);}
    assert(!t.micro_acked && t.micro_phase==1);
    assert(Track_Sent(&t,1,token,tick));
    assert(!Track_Sent(&t,1,token,tick));
    unsigned sent=tick;
    unsigned duration=t.micro_duration_ms;
    for(;tick+10<sent+duration;){tick+=10;Fresh(&o,tick);assert(Track_Step(&t,&o,tick)>=0);}
    assert(t.micro_phase==1); /* 旧的从计算时刻计时会提前100ms停止。 */
    tick=sent+duration;Fresh(&o,tick);assert(Track_Step(&t,&o,tick)>=0);assert(t.micro_phase==2);
    Track_Cancel(&t);assert(!Track_Sent(&t,1,token,tick));

    for(unsigned legacy=0;legacy<2;legacy++)
    {
        Begin(&t,&o,legacy?.035:.0034906585);
        for(tick=110;tick<2200;tick+=10)
        {
            Fresh(&o,tick);o.pose.y_m=.005;o.yaw_rad=o.pose.yaw_rad=.02;
            assert(Track_Step(&t,&o,tick)>=0);
            token=Track_Token(&t);if(token)(void)Track_Sent(&t,1,token,tick);
            if(t.output.state==TRACK_COMPLETE)break;
        }
        if(legacy)assert(t.output.state==TRACK_COMPLETE);
        else assert(t.output.state!=TRACK_COMPLETE && t.micro_rpm[0]==-t.micro_rpm[1] &&
                    t.micro_rpm[0]==t.micro_rpm[2] && t.micro_rpm[1]==t.micro_rpm[3]);
    }
    /* 新目标不能把原来的偏航重定义为零误差。 */
    TrackGoal g=t.goal;g.id=2;Fresh(&o,tick+10);
    assert(Track_Start(&t,&g,&o,tick+10)==1);
    assert(t.micro_heading==0);
    Fresh(&o,tick+20);assert(Track_Step(&t,&o,tick+20)>=0);
    assert(fabs(t.output.yaw_error_rad-.02)<1e-6);
    puts("micro timing / stale ACK / heading negative / fixed heading PASS");
    return 0;
}
