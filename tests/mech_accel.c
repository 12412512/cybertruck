/* 正式单轴/联调输出加速度字节与重复ID检查，不访问总线。 */
#define GROUP_MAIN GroupFixtureMain
#include "mech_group.c"
#undef GROUP_MAIN
int main(int argc,char **argv)
{
    unsigned which=argc>1?(unsigned)atoi(argv[1]):0;
    BusFrame frames[2];Setup();
    if(which<5)
    {
        goal.axis=1;goal.target_um=84000;goal.rpm=100;
        goal.accel_set=which!=0;goal.accel=which==1?0:which==2?100:which==3?160:255;
        CHECK(!Axis_Manual(&axis,&goal,now));CHECK(Axis_Take(&axis,frames,now)==2);
        CHECK(frames[0].data[4]==(which?goal.accel:1));
        AxisGoal changed=goal;changed.accel_set=1;changed.accel=50;
        CHECK(Axis_Manual(&axis,&changed,now)==AXIS_ID);return 0;
    }
    request.mask=3;request.accel_set=which!=5;request.accel[0]=100;request.accel[1]=200;
    CHECK(!Group_Begin(&group,&request,now));
    for(unsigned i=0;i<2;i++)
    {
        CHECK(Group_Take(&group,frames,now)==2);
        CHECK(frames[0].data[4]==(which==5?1:request.accel[i]));
        Group_Sent(&group,1,now);
    }
    GroupGoal changed=request;changed.accel[0]=160;changed.accel_set=1;
    CHECK(Group_Begin(&group,&changed,now)==AXIS_ID);
    return 0;
}
