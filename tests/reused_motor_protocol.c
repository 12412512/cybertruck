#include "drive.h"
#include <assert.h>
#include <string.h>
#include <stdio.h>
static unsigned checks;
#define CHECK(x) do { assert(x); ++checks; } while (0)
static void ConfigFrames(BusFrame frames[5],uint8_t address,uint8_t micro)
{
    uint8_t raw[32]={0}, offset=1, p, n;
    raw[0]=0x42; raw[1]=0x21; raw[2]=0x15; raw[3]=0x19;
    raw[8]=micro; raw[9]=1; raw[19]=address; raw[20]=0; raw[31]=0x6B;
    memset(frames,0,5*sizeof(*frames));
    for (p=0;p<5;p++) {
        n=(p==4)?3:7; frames[p].id=((uint32_t)address<<8)|p;
        frames[p].data[0]=0x42; frames[p].len=(uint8_t)(n+1); frames[p].tick=100+p;
        memcpy(&frames[p].data[1],&raw[offset],n); offset=(uint8_t)(offset+n);
    }
}
int main(void)
{
    BusFrame f[2], config_frames[5]; MotorReply r; MotorConfigRx rx; MotorConfig cfg;
    unsigned address, mode, p; uint8_t n;
    const uint8_t pos0[]={0xFD,1,0,60,100,0x89,0xAB,0xCD};
    const uint8_t pos1[]={0xFD,0xEF,2,0,0x6B};
    CHECK(Drive_Build(0,0x35,0,0,0,0,0,0,f)==0);
    CHECK(Drive_Build(1,0x35,0,0,0,0,0,0,0)==0);
    CHECK(Drive_Build(1,0x99,0,0,0,0,0,0,f)==0);
    CHECK(Drive_Build(1,0xF6,2,60,1,0,0,0,f)==0);
    CHECK(Drive_Build(1,0xF6,0,3001,1,0,0,0,f)==0);
    CHECK(Drive_Build(1,0xFD,0,60,1,0,3,0,f)==0);
    CHECK(Drive_Build(1,0xF3,0,0,0,0,0,2,f)==0);
    CHECK(Drive_Build(7,0xFD,1,60,100,0x89ABCDEFU,2,0,f)==2);
    CHECK(f[0].id==0x700 && f[1].id==0x701 && f[0].len==8 && f[1].len==5);
    CHECK(!memcmp(f[0].data,pos0,8) && !memcmp(f[1].data,pos1,5));
    for (address=1;address<=255;address++) for (mode=0;mode<=2;mode++) {
        CHECK(Drive_Build((uint8_t)address,0xFD,0,3000,255,0xFFFFFFFFU,(uint8_t)mode,0,f)==2);
        CHECK(f[0].id==(address<<8) && f[1].id==((address<<8)|1));
        CHECK(f[1].data[2]==mode && f[1].data[3]==0 && f[1].data[4]==0x6B);
    }
    CHECK(Drive_Build(1,0xFE,0,0,0,0,0,0,f)==1 && f[0].len==4);
    CHECK(f[0].data[0]==0xFE && f[0].data[1]==0x98 && f[0].data[2]==0 && f[0].data[3]==0x6B);
    CHECK(Drive_Build(1,0xF6,1,3000,255,0,0,0,f)==1 && f[0].len==7);
    CHECK(f[0].data[2]==0x0B && f[0].data[3]==0xB8 && f[0].data[4]==255);
    CHECK(Drive_Build(1,0x42,0,0,0,0,0,0,f)==1 && f[0].len==3 && f[0].data[1]==0x6C);
    memset(f,0,sizeof(f)); f[0].id=0x100; f[0].tick=123; f[0].len=7;
    { const uint8_t d[]={0x36,1,255,255,255,255,0x6B}; memcpy(f[0].data,d,7); }
    CHECK(Drive_Parse(f,1,&r) && r.value==-4294967295LL && r.tick==123);
    CHECK(!Drive_Parse(f,2,&r)); f[0].id=0x101; CHECK(!Drive_Parse(f,1,&r)); f[0].id=0x100;
    f[0].data[1]=2; CHECK(!Drive_Parse(f,1,&r)); f[0].data[1]=1;
    f[0].data[6]=0; CHECK(!Drive_Parse(f,1,&r)); f[0].data[6]=0x6B;
    for (n=0;n<=8;n++) { f[0].len=n; if(n!=7) CHECK(!Drive_Parse(f,1,&r)); }
    f[0].len=5; f[0].data[0]=0x35; f[0].data[1]=0; f[0].data[2]=0x13; f[0].data[3]=0x88; f[0].data[4]=0x6B;
    CHECK(Drive_Parse(f,1,&r) && r.value==5000); f[0].data[3]++; CHECK(!Drive_Parse(f,1,&r));
    f[0].len=3; f[0].data[0]=0xFD; f[0].data[1]=0x02; f[0].data[2]=0x6B;
    CHECK(Drive_Parse(f,1,&r) && r.result==2); f[0].data[1]=0x9F;
    CHECK(Drive_Parse(f,1,&r) && r.result==0x9F); f[0].data[1]=0xE2;
    CHECK(Drive_Parse(f,1,&r) && r.result==0xE2); f[0].data[1]=0xFF; CHECK(!Drive_Parse(f,1,&r));
    ConfigFrames(config_frames,7,16); Drive_Reset(&rx);
    for (p=0;p<5;p++) CHECK(Drive_Config(&rx,&config_frames[p],7,&cfg)==(p==4?1:0));
    CHECK(cfg.valid && cfg.address==7 && cfg.microsteps==16 && cfg.motor_type==0x19 && cfg.interpolation==1);
    ConfigFrames(config_frames,7,0); Drive_Reset(&rx);
    for (p=0;p<5;p++) CHECK(Drive_Config(&rx,&config_frames[p],7,&cfg)==(p==4?1:0));
    CHECK(cfg.microsteps==256);
    Drive_Reset(&rx); CHECK(Drive_Config(&rx,&config_frames[1],7,&cfg)==2);
    CHECK(Drive_Config(&rx,&config_frames[0],6,&cfg)==2);
    config_frames[0].len=7; CHECK(Drive_Config(&rx,&config_frames[0],7,&cfg)==2);
    ConfigFrames(config_frames,7,16); Drive_Reset(&rx); CHECK(Drive_Config(&rx,&config_frames[0],7,&cfg)==0);
    config_frames[1].tick=160; CHECK(Drive_Config(&rx,&config_frames[1],7,&cfg)==2);
    ConfigFrames(config_frames,7,16); config_frames[4].data[3]=0;
    for (p=0;p<4;p++) CHECK(Drive_Config(&rx,&config_frames[p],7,&cfg)==0);
    CHECK(Drive_Config(&rx,&config_frames[4],7,&cfg)==2);
    printf("PASS: %u protocol checks (actual C execution)\n",checks); return 0;
}
