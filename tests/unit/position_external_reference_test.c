/* 外部轨迹输入与反馈均为显式测试数据，不代表电机台架验证。 */
#ifdef NDEBUG
#undef NDEBUG
#endif
#include <assert.h>
#include <math.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include "position_cascade.h"
#include "position_cascade_config.h"
#include "config.inc"

static PositionCascadeConfig_TypeDef tuning;
static PositionExternalInput sample;
static PositionCascadeControlOutput_TypeDef output;
static PositionCascadeTelemetry_TypeDef telemetry;

static void reset(void)
{
    PositionCascade_Reset();
    tuning=config();
    tuning.acceleration=tuning.deceleration=.52f;
    tuning.maximum_speed=.78f;tuning.speed_limit=3.14f;tuning.current_limit=7;
    tuning.position_kp=8;tuning.position_kd=3;
    tuning.speed_kp=3.0f/7.0f;tuning.speed_ki=6.0f/7.0f;
    tuning.acceleration_feedforward_gain=0;
    sample=(PositionExternalInput){0,0,0,1,true,-.35f,.35f};
}

static void step(float position,float speed)
{
    assert(PositionCascade_UpdateExternalControl(&tuning,&sample,position,speed,&output));
    assert(PositionCascade_GetTelemetry(&telemetry));
    assert(isfinite(output.iq_reference) && fabsf(output.iq_reference)<=7.00001f);
    assert(fabsf(output.speed_reference)<=.780001f);
    assert(fabsf(telemetry.trajectory_speed_reference)<=.78001f);
    assert(output.position_reference>=sample.minimum_position_rad-.000001f);
    assert(output.position_reference<=sample.maximum_position_rad+.000001f);
}

static void expiry_and_recovery(void)
{
    reset();step(0,0);
    sample.now_ms=99;step(0,0);assert(!telemetry.external_reference_stale);
    sample.now_ms=100;step(0,0);assert(telemetry.external_reference_stale);
    /* 旧序号即使被重新赋予新时间，也不能恢复新鲜度。 */
    sample.received_ms=sample.now_ms=200;step(0,0);assert(telemetry.external_reference_stale);
    /* 长间隔后的合法首点以实测位置播种，不与旧reference比较。 */
    sample.position_rad=.2f;sample.sequence=2;sample.received_ms=sample.now_ms=500;
    step(.2f,0);assert(!telemetry.external_reference_stale);
    assert(fabsf(output.position_reference-.2f)<.000001f);
    /* 新序号的相同值刷新；老帧和越限跳变都不能刷新。 */
    sample.sequence=3;sample.received_ms=sample.now_ms=580;step(.2f,0);
    sample.now_ms=679;step(.2f,0);assert(!telemetry.external_reference_stale);
    sample.now_ms=680;step(.2f,0);assert(telemetry.external_reference_stale);
    sample.sequence=4;sample.received_ms=690;sample.now_ms=790;step(.2f,0);
    assert(telemetry.external_reference_stale && telemetry.external_rejected_points==1);
    sample.sequence=5;sample.received_ms=sample.now_ms=800;sample.position_rad=-.2f;
    step(.2f,0);assert(telemetry.external_reference_stale && telemetry.external_rejected_points==2);
    sample.sequence=6;sample.position_rad=.2f;sample.received_ms=sample.now_ms=810;
    step(.2f,0);assert(!telemetry.external_reference_stale);
    assert(fabsf(output.position_reference-.2f)<.000001f);
    puts("PASS external freshness [0,100), duplicate values/sequences, expired packet and measured-position recovery");
}

static void wrapping(void)
{
    reset();sample.sequence=UINT32_MAX-1;sample.received_ms=sample.now_ms=UINT32_MAX-50;
    step(0,0);assert(!telemetry.external_reference_stale);
    sample.sequence=UINT32_MAX;sample.position_rad=.001f;sample.received_ms=sample.now_ms=UINT32_MAX-30;
    step(0,0);
    sample.sequence=0;sample.position_rad=.002f;sample.received_ms=sample.now_ms=UINT32_MAX-10;
    step(0,0);assert(!telemetry.external_reference_stale && telemetry.external_rejected_points==0);
    sample.now_ms=88;step(0,0);assert(!telemetry.external_reference_stale);
    sample.now_ms=89;step(0,0);assert(telemetry.external_reference_stale);
    puts("PASS sequence and uint32 millisecond wrap, including exact timeout");
}

static void continuous_tracking(void)
{
    reset();
    const unsigned spacing[]={12,28,20,20};
    unsigned next=0,packets=0;
    float previous=0,max_error=0;
    for(unsigned tick=0;tick<20000;++tick) {
        float time=tick*.0005f;
        unsigned now=tick/2U;
        sample.now_ms=now;
        if(now>=next) {
            sample.position_rad=.349f*sinf(1.9f*time);
            sample.received_ms=now;sample.sequence++;
            next=now+spacing[packets++%4];
        }
        step(previous,0); /* 显式给定反馈，只验证参考连续性和控制输出边界。 */
        assert(!telemetry.external_reference_stale && telemetry.external_rejected_points==0);
        assert(fabsf(output.position_reference-previous)<=.78f*.0005f+.000001f);
        float error=fabsf(sample.position_rad-output.position_reference);
        if(error>max_error) max_error=error;
        previous=output.position_reference;
    }
    assert(max_error<.035f);
    /* 重复最终点让速度连续收敛；无逐帧PI复位或持续参考漂移。 */
    float last=sample.position_rad;
    for(unsigned tick=0;tick<2000;++tick) {
        sample.now_ms=10000+tick/2;
        if(tick%40==0) {sample.received_ms=sample.now_ms;sample.sequence++;}
        step(previous,0);previous=output.position_reference;
    }
    assert(fabsf(output.position_reference-last)<.00001f);
    assert(fabsf(telemetry.trajectory_speed_reference)<.00001f);
    assert(telemetry.external_rejected_points==0);
    printf("PASS jittered 12/28/20ms trajectory, velocity/current/range limits, final reference error=%.8f rad\n",output.position_reference-last);
}

static void range_and_feedback(void)
{
    reset();sample.position_rad=.345f;step(.345f,0);
    for(unsigned k=1;k<=4;++k) {
        sample.sequence++;sample.received_ms=sample.now_ms=20*k;
        sample.position_rad=.345f+.001f*k;
        for(unsigned tick=0;tick<40;++tick) {sample.now_ms=20*k+tick/2;step(.345f,0);}
    }
    assert(output.position_reference<=.350001f);
    sample.sequence++;sample.received_ms=sample.now_ms=101;sample.position_rad=.351f;
    step(.345f,0);assert(telemetry.external_reference_stale);
    reset();sample.position_rad=0;
    for(unsigned k=0;k<200;++k) {sample.now_ms=k/2;step(0,2.0f);}
    assert(output.speed_feedback>1.9f); /* 真实超速反馈不得伪造为受限速度。 */
    puts("PASS prediction limited by axis envelope; actual speed feedback remains unmodified");
}

/** @brief 验证5ms段确实在5ms完成，而非仍被旧10ms下限拉长；不模拟机械运动。 */
static void interpolation_200hz(void)
{
    reset();step(0,0);
    sample.position_rad=.003f;sample.sequence++;sample.received_ms=sample.now_ms=5;
    float previous=0;
    for(unsigned tick=0;tick<10;tick++) {
        step(previous,0);
        assert(fabsf(output.position_reference-previous)<=.78f*.0005f+.000001f);
        previous=output.position_reference;
    }
    assert(output.position_reference>=.00299f && !telemetry.external_reference_stale);
    /* 新序号同毫秒到达须避免零除，逆序时刻则拒绝且不刷新流。 */
    sample.position_rad=.004f;sample.sequence++;step(previous,0);
    assert(!telemetry.external_reference_stale && telemetry.external_rejected_points==0);
    sample.received_ms=4;sample.sequence++;step(previous,0);
    assert(telemetry.external_reference_stale && telemetry.external_rejected_points==1);
    puts("PASS 5ms interpolation, coalesced receive timestamp, out-of-order rejection");
}

/** @brief 用量化正弦目标验证200Hz及3/7、4/6、1/9ms接收抖动和最终保持。 */
static void jitter_200hz(void)
{
    reset();
    const unsigned spacing[]={5,5,4,6,3,7,1,9};
    unsigned next=0,packets=0;
    float previous=0,max_error=0;
    for(unsigned tick=0;tick<20000;tick++) {
        float time=(float)tick*.0005f;
        sample.now_ms=tick/2;
        if(sample.now_ms>=next) {
            sample.position_rad=roundf(.349f*sinf(1.9f*time)*1000.0f)*.001f;
            sample.received_ms=sample.now_ms;sample.sequence++;
            next=sample.now_ms+spacing[packets++%8];
        }
        step(previous,0);
        assert(!telemetry.external_reference_stale && telemetry.external_rejected_points==0);
        assert(fabsf(output.position_reference-previous)<=.78f*.0005f+.000001f);
        float error=fabsf(sample.position_rad-output.position_reference);
        if(error>max_error) max_error=error;
        previous=output.position_reference;
    }
    assert(max_error<.012f);
    float target=sample.position_rad;
    for(unsigned tick=0;tick<1000;tick++) {
        sample.now_ms=10000+tick/2;
        if(tick%10==0) {sample.sequence++;sample.received_ms=sample.now_ms;}
        step(previous,0);previous=output.position_reference;
    }
    assert(fabsf(output.position_reference-target)<.00001f);
    assert(fabsf(telemetry.trajectory_speed_reference)<.00001f);
    assert(telemetry.external_rejected_points==0);
    printf("PASS 200Hz quantized jitter stream, reference max lag=%.6f rad, final hold\n",max_error);
}

/** @brief 执行外部参考新旧周期的原生回归，不打开任何硬件。
 * @return 所有断言通过返回0，失败由assert终止。
 */
int main(void)
{
    interpolation_200hz();jitter_200hz();
    expiry_and_recovery();wrapping();continuous_tracking();range_and_feedback();return 0;
}
