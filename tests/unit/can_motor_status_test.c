#include "software/communication/protocol/can_motor_status.h"
#include <assert.h>
#include <math.h>
#include <string.h>
#include <limits.h>
#include <stdio.h>

static MotorStatus sample = {
    7,3,-1.25f,1.5f,2.5f,-2.0f,1.25f,-1.5f,.75f,-.5f,65.25f,48.75f};
static uint8_t data[50];
static uint16_t id;

static void reset(void)
{
    MotorStatus scratch;
    if (MotorStatus_IsRequested()) MotorStatus_Publish(&sample);
    (void)MotorStatus_Take(&scratch);
    CanMotorStatus_Init();
}
static bool tick(uint32_t now)
{
    bool result = CanMotorStatus_Prepare(now,4,&id,data,48);
    if (MotorStatus_IsRequested()) MotorStatus_Publish(&sample);
    return CanMotorStatus_Prepare(now,4,&id,data,48) || result;
}
static void encoding(void)
{
    static const uint8_t expected[48] = {
        0,7,0,3, 0xff,0xff,0xfb,0x1e, 0,0,5,0xdc, 0,0,0,0xfa,
        0xff,0xff,0xff,0x38, 4,0xe2,0xfa,0x24, 0,0,2,0xee, 0xff,0xff,0xff,0xce,
        0x19,0x7d,0x13,0x0b, 0,0,0,0,0,0,0,0,0,0,0,0};
    MotorStatus s = sample;
    memset(data,0xa5,sizeof(data));
    assert(!CanMotorStatus_Encode(&s,data,47));
    assert(data[0] == 0xa5);
    assert(!CanMotorStatus_Encode(NULL,data,48));
    assert(!CanMotorStatus_Encode(&s,NULL,48));
    assert(CanMotorStatus_Encode(&s,data,48));
    assert(memcmp(data,expected,48)==0 && data[48]==0xa5 && data[49]==0xa5);
    s.position_target=NAN; s.position_feedback=INFINITY;
    s.speed_target=3e30f; s.speed_feedback=-3e30f;
    s.current_reference=NAN; s.current_feedback=-40.f;
    s.temperature=NAN; s.bus_voltage=INFINITY;
    assert(CanMotorStatus_Encode(&s,data,48));
    assert(data[4]==0x80 && data[7]==0 && data[8]==0x80);
    assert(data[12]==0x7f && data[15]==0xff);
    assert(data[16]==0x80 && data[19]==1);
    assert(data[20]==0x80 && data[21]==0);
    assert(data[22]==0x80 && data[23]==1);
    assert(data[32]==0x80 && data[33]==0);
    assert(data[34]==0x80 && data[35]==0);
}
static void rates(void)
{
    unsigned hz,ms,count;
    reset();assert(CanMotorStatus_Rate()==0);
    assert(!tick(0) && !tick(500));
    assert(CanMotorStatus_Configure(1));assert(CanMotorStatus_Rate()==20);
    assert(!tick(1000) && !tick(1049) && tick(1050));assert(id==0x7f4);
    assert(CanMotorStatus_Configure(0));assert(!tick(1100));
    assert(CanMotorStatus_Configure(1));assert(CanMotorStatus_Rate()==20);
    for (hz=10;hz<=200;++hz) {
        reset();assert(CanMotorStatus_Configure((float)hz));count=0;
        for(ms=0;ms<=10000;++ms) if(tick(ms)) ++count;
        assert(count == hz*10);
    }
    reset();assert(CanMotorStatus_Configure(17));count=0;
    for(ms=0;ms<=10000;++ms) if(tick(UINT32_MAX-200U+ms)) ++count;
    assert(count==170); /* wrap and non-divisor rate */
    assert(CanMotorStatus_Configure(0));assert(CanMotorStatus_Configure(1));
    assert(CanMotorStatus_Rate()==17);
    { const float bad[]={-1,2,9,201,10.5f,NAN,INFINITY,1e30f};
      for(ms=0;ms<sizeof(bad)/sizeof(bad[0]);++ms) {
          assert(!CanMotorStatus_Configure(bad[ms]));assert(CanMotorStatus_Rate()==17);
      }
    }
}
static void mailbox_and_backlog(void)
{
    MotorStatus s;
    reset(); assert(!MotorStatus_Take(&s));
    MotorStatus_Publish(&sample);assert(!MotorStatus_Take(&s));
    MotorStatus_Request();MotorStatus_Request();assert(MotorStatus_IsRequested());
    MotorStatus_Publish(&sample);sample.mode=18;MotorStatus_Publish(&sample);
    assert(MotorStatus_Take(&s) && s.mode==3);assert(!MotorStatus_Take(&s));sample.mode=3;
    reset();assert(CanMotorStatus_Configure(200));assert(!tick(0));
    assert(tick(5000)); assert(!tick(5000) && !tick(5001));assert(tick(5005));
    assert(!CanMotorStatus_Prepare(6000,8,&id,data,48));
    assert(!CanMotorStatus_Prepare(6000,4,&id,data,47));
    reset();assert(CanMotorStatus_Configure(20));assert(!tick(0));
    assert(!CanMotorStatus_Prepare(50,4,&id,data,48));MotorStatus_Publish(&sample);
    /* Main loop delayed across another period: request fresh, do not send old. */
    assert(!CanMotorStatus_Prepare(150,4,&id,data,48));
    sample.mode=18;MotorStatus_Publish(&sample);
    assert(CanMotorStatus_Prepare(150,4,&id,data,48));assert(data[3]==18);sample.mode=3;
}
int main(void)
{
    encoding();rates();mailbox_and_backlog();
    puts("PASS CAN status: 48-byte golden frame, bounds/sentinels, all 191 rates, wrap, command validation, mailbox and stale/backlog handling");
    return 0;
}
