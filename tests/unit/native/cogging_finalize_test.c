#include <assert.h>
#include <stdio.h>
#include <string.h>
#include <math.h>

/* 包含真实模块以注入已自然完成的采样状态，不对外增加可写测试接口。 */
#include "foc_cogging_calibration.c"

static uint32_t critical_depth;
uint32_t critical_hw_enter(void)
{
    return critical_depth++;
}
void critical_hw_exit(uint32_t mask)
{
    assert(critical_depth == mask + 1U);
    critical_depth = mask;
}
bool Encoder_IsOnline(const Encoder_TypeDef *e)
{
    return e->bad_frame_streak == 0U;
}
uint8_t Encoder_GetCalibFlag(const Encoder_TypeDef *e)
{
    return e->calib_flag;
}
uint8_t Encoder_GetReverse(const Encoder_TypeDef *e)
{
    return e->reverse;
}
float Encoder_GetMecPos(const Encoder_TypeDef *e)
{
    return e->theta_mech;
}
float Encoder_GetMecVel(const Encoder_TypeDef *e)
{
    return e->vel_mech;
}
float Encoder_GetMecVelContinuous(const Encoder_TypeDef *e)
{
    return e->vel_mech_continuous;
}
float Encoder_GetElePhase(const Encoder_TypeDef *e)
{
    return e->theta_elec;
}
float Encoder_GetEleVel(const Encoder_TypeDef *e)
{
    return e->vel_elec;
}
void MotorFault_Report(MotorControl_TypeDef *m, ErrorNow_TypeDef error)
{
    if (m->ErrorNow == No_Error)
    {
        m->ErrorNow = error;
    }
}
void FOC_CurrentController_Reset(FOC_TypeDef *f)
{
    (void)f;
}
void PI_Controller_Reset(PI_Controller_TypeDef *p)
{
    (void)p;
}
void PI_Controller_Configure(
    PI_Controller_TypeDef *p, float kp, float ki, float ts, float low, float high)
{
    (void)p;
    (void)kp;
    (void)ki;
    (void)ts;
    (void)low;
    (void)high;
}
float PI_Controller_Run(PI_Controller_TypeDef *p, float reference, float feedback)
{
    (void)p;
    return reference - feedback;
}
void FOC_Current(FOC_TypeDef *f, MotorControl_TypeDef *m, float phase, float speed)
{
    (void)f;
    (void)m;
    (void)phase;
    (void)speed;
}
void power_stage_hw_start(void)
{
}
float constrain(float value, float low, float high)
{
    return fminf(fmaxf(value, low), high);
}

static void completed_samples(int16_t magnitude)
{
    memset(&CoggingCalib, 0, sizeof(CoggingCalib));
    CoggingCalib.state = COGGING_COMPLETE;
    CoggingCalib.full_scale_a = CURRENT_SENSE_PROFILE_FULL_SCALE_A;
    for (unsigned i = 0U; i < COGGING_MAP_POINTS; ++i)
    {
        CoggingCalib.iq_q15[i] = (i & 1U) != 0U ? magnitude : -magnitude;
    }
    finalize_pending = true;
    session_started = false;
}

int main(void)
{
    MotorControl_TypeDef motor = {0};
    Encoder_TypeDef encoder = {0};
    CoggingMapRecord original, candidate, exported;
    motor.axis_profile_valid = true;
    motor.motor_pole_pairs = 21;
    encoder.calib_flag = ENC_CALIB_ALL;
    encoder.electrical_zero_q15 = 3155U;
    completed_samples(123);
    assert(CoggingMap_Build(
        CoggingCalib.iq_q15, CoggingCalib.full_scale_a, Signature(&motor, &encoder), &original));
    FocCogging_ImportMap(&original);

    completed_samples(321);
    assert(FocCogging_BeginFinalize() && !FocCogging_IsReleased());
    assert(FocCogging_BuildCandidate(&candidate, &motor, &encoder));
    assert(candidate.crc32 != original.crc32);
    FocCogging_Abort();
    assert(CoggingCalib.state == COGGING_COMPLETE && CoggingCalib.iq_q15[1] == 321);
    assert(!FocCogging_PublishCandidate(&candidate));
    assert(!FocCogging_IsReleased());
    FocCogging_ReleaseFinalize(false);
    assert(FocCogging_IsReleased() && CoggingCalib.reason == COGGING_CANCELLED);
    FocCogging_ExportMap(&exported, &motor, &encoder);
    assert(memcmp(&exported, &original, sizeof(original)) == 0);
    puts("PASS real cogging candidate cancellation preserves borrowed samples and previous valid "
         "map");

    completed_samples(321);
    assert(FocCogging_BeginFinalize());
    assert(FocCogging_BuildCandidate(&candidate, &motor, &encoder));
    assert(FocCogging_PublishCandidate(&candidate));
    FocCogging_ReleaseFinalize(true);
    assert(FocCogging_IsReleased() && FocCogging_GetState() == COGGING_SAVING);
    FocCogging_SaveResult(false);
    assert(FocCogging_GetState() == COGGING_FAILED && CoggingCalib.reason == COGGING_SAVE_FAILED);
    FocCogging_ExportMap(&exported, &motor, &encoder);
    assert(memcmp(&exported, &candidate, sizeof(candidate)) == 0);
    encoder.calib_flag = ENC_CALIB_LINEARIZED;
    FocCogging_ExportMap(&exported, &motor, &encoder);
    assert(exported.magic == 0U);
    puts("PASS RAM publication differs from Flash result; partial calibration cannot export a "
         "usable table");
    assert(critical_depth == 0U);
    return 0;
}
