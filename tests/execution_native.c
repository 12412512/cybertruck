/* 复用正式执行器与GC入口，只有设备IO和物理传输由夹具代替。 */
#include "execute.h"
#include "comm.h"
#include "start.h"
#include <math.h>
#include <string.h>

#define API __declspec(dllexport)
static Exec executor;
static Start key_gate;
static ExecGate key_failure;

API int Device_StartReady(uint32_t now_ms)
{
    return Exec_Ready(&executor, now_ms);
}

API uint32_t Device_ObservationValid(void)
{
    return executor.observation.pose.valid;
}
API uint32_t Device_StartMask(void)
{
    return executor.gate.mask;
}
static uint32_t now, sequence, error, starts, stops, sends, enabled;
static int16_t target[4];
static uint32_t pending_id, pending_zero, pending_tick;
static int allow_ack = 1;
static uint32_t drop;
static uint32_t imu_epoch;

static int Ready(uint32_t tick, int enable, uint32_t since)
{ (void)tick; (void)enable; (void)since; return 1; }
static int Enable(unsigned wheel) { (void)wheel; enabled++; return 1; }
static int StartBus(void) { starts++; return 1; }
static int Send(const int16_t rpm[4], uint32_t id, uint32_t zero)
{
    memcpy(target, rpm, sizeof(target)); sends++;
    if (zero && (!pending_zero || pending_zero != zero))
    { pending_id=id; pending_zero=zero; pending_tick=now+12; }
    if (!zero) { pending_zero=0; }
    return 1;
}
static void Stop(void) { memset(target,0,sizeof(target)); stops++; }
static uint32_t Fault(void) { return error; }
static int Stopped(uint32_t since) { return now-since>180; }
static int Rearm(void) { return 1; }
static void StopCommand(uint32_t reason,uint32_t tick) { Exec_Stop(&executor,reason,tick); }
static uint8_t Command(uint8_t type,const uint8_t *data,uint16_t size,
                       uint8_t *reply,uint16_t *reply_size,uint32_t tick)
{ return Execute_Command(&executor,type,data,size,reply,reply_size,tick,error,sequence); }

API void Device_Init(void)
{
    ExecIo io={Ready,Enable,StartBus,Send,Stop,Fault,Stopped,Rearm};
    CommHooks hooks={StopCommand,Command};
    now=sequence=error=starts=stops=sends=enabled=drop=0;
    pending_id=pending_zero=pending_tick=0; allow_ack=1;
    imu_epoch=1;
    memset(target,0,sizeof(target));
    Exec_Init(&executor,io); executor.stop_confirmed=1;
    Start_Init(&key_gate);
    memset(&key_failure,0,sizeof(key_failure));
    Comm_Init(123,0); Comm_Bind(hooks);
}

/* 只在主机夹具接入正式Start与Exec；GPIO和应答传输仍为合成输入。 */
API int Device_KeyPrepare(uint32_t ticket, uint32_t version)
{
    return version == executor.config.version && Start_Prepare(&key_gate, Comm_Boot(), Comm_Session(),
        ticket, version, now, 0, (uint8_t)Exec_Ready(&executor, now));
}

API void Device_KeyTick(uint32_t tick, uint8_t pressed)
{
    uint32_t previous = key_gate.phase;
    uint8_t ready = (uint8_t)Exec_Ready(&executor, tick);
    Start_Run(&key_gate, Comm_Boot(), Comm_Session(), tick, pressed, ready);
    if (previous >= START_RELEASE && previous <= START_BEEP && key_gate.phase == START_ABORTED)
        key_failure = executor.gate;
}

API void Device_KeyRead(uint32_t values[21])
{
    memset(values, 0, 21 * sizeof(uint32_t));
    values[0] = 2;
    values[1] = key_gate.phase;
    values[2] = key_gate.ticket;
    values[3] = key_gate.version;
    values[4] = key_gate.phase == START_FIRED;
    values[5] = key_gate.beep;
    values[6] = key_gate.reason;
    values[7] = key_gate.aborted_ms;
    memcpy(values + 8, &key_failure, sizeof(key_failure));
}
API void Device_Tick(uint32_t tick,const int64_t *counts,const int16_t *rpm,int16_t yaw,int16_t gyro)
{
    MotorSample motor={0}; ImuSample imu={0}; unsigned index;
    now=tick;
    for(index=0;index<4;index++)
    {
        motor.address=(uint8_t)(index+1); motor.stamp.first_ms=motor.stamp.last_ms=motor.stamp.handled_ms=tick;
        motor.stamp.epoch=1; motor.stamp.sequence=++sequence; motor.kind=0x36; motor.value=counts[index];
        if(!(drop&1)) Exec_Motor(&executor,&motor,tick);
        motor.stamp.sequence=++sequence; motor.kind=0x35; motor.value=rpm[index];
        if(!(drop&2)) Exec_Motor(&executor,&motor,tick);
    }
    imu.stamp.first_ms=imu.stamp.last_ms=imu.stamp.handled_ms=tick;
    imu.stamp.epoch=imu_epoch; imu.source_epoch=1; imu.stamp.sequence=++sequence;
    imu.kind=0x53; imu.axes[2]=yaw; if(!(drop&4)) Exec_Imu(&executor,&imu,tick);
    imu.stamp.sequence=++sequence; imu.kind=0x52; imu.axes[2]=gyro; if(!(drop&8)) Exec_Imu(&executor,&imu,tick);
    if(pending_zero && tick>=pending_tick && allow_ack) { Exec_Sent(&executor,pending_id,pending_zero,tick); }
    Comm_Tick(tick,0); Exec_Run(&executor,tick);
}
API void Device_Push(const uint8_t *data,unsigned size)
{ unsigned index;for(index=0;index<size;index++) Comm_Feed(data[index],now,now); }
API uint16_t Device_Read(uint8_t *wire) { return Comm_Read(wire); }
API void Device_Target(int16_t *rpm) { memcpy(rpm,target,sizeof(target)); }
API uint32_t Device_Value(unsigned key)
{
    switch(key) { case 0:return executor.mode;case 1:return starts;case 2:return stops;
        case 3:return sends;case 4:return enabled;case 5:return executor.track.output.state;
        case 6:return executor.track.output.id;case 7:return executor.fault;default:return 0; }
}
API void Device_Error(uint32_t value) { error=value; }
API void Device_Ack(int value) { allow_ack=value; }
API void Device_Drop(uint32_t value) { drop=value; }
API void Device_Epoch(uint32_t value) { imu_epoch=value; }
API void Device_LinkFault(uint32_t value) { Comm_Fault(value); }
/* 正式参考计划的只读采样，验证整段边界，不导出设备动作接口。 */
API int Device_Ref(double fraction, RefPoint *point)
{
    return Ref_At(&executor.track.plan, executor.track.plan.duration_s * fraction, point);
}

/* 合成跨毫秒输入，用于复现旧取时顺序；只属于主机夹具。 */
API void Device_PosTime(uint32_t received, uint32_t handled, uint32_t check)
{
    MotorSample sample = {0};
    now = check;
    sample.address = 1;
    sample.kind = 0x36;
    sample.stamp.first_ms = sample.stamp.last_ms = received;
    sample.stamp.handled_ms = handled;
    sample.stamp.sequence = ++sequence;
    sample.stamp.epoch = 1;
    Exec_Motor(&executor, &sample, check);
    Exec_Run(&executor, check);
}
