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
bool Encoder_IsOnline(const EncoderState *e)
{
    return e->bad_frame_streak == 0U;
}
uint8_t Encoder_GetCalibFlag(const EncoderState *e)
{
    return e->calib_flag;
}
uint8_t Encoder_GetReverse(const EncoderState *e)
{
    return e->reverse;
}
float Encoder_GetMecPos(const EncoderState *e)
{
    return e->mechanical_position_rad;
}
float Encoder_GetMecVel(const EncoderState *e)
{
    return e->mechanical_velocity_rad_s;
}
float Encoder_GetMecVelContinuous(const EncoderState *e)
{
    return e->mechanical_velocity_continuous_rad_s;
}
float Encoder_GetElePhase(const EncoderState *e)
{
    return e->electrical_angle_rad;
}
float Encoder_GetEleVel(const EncoderState *e)
{
    return e->electrical_velocity_rad_s;
}
void MotorFault_Report(MotorControlState *m, MotorFaultCode error)
{
    if (m->fault_code == MOTOR_FAULT_NONE)
    {
        m->fault_code = error;
    }
}
void FOC_CurrentController_Reset(FocState *f)
{
    (void)f;
}
void PiController_Reset(PiController *p)
{
    (void)p;
}
void PiController_Configure(PiController *p, float kp, float ki, float ts, float low, float high)
{
    (void)p;
    (void)kp;
    (void)ki;
    (void)ts;
    (void)low;
    (void)high;
}
float PiController_Run(PiController *p, float reference, float feedback)
{
    (void)p;
    return reference - feedback;
}
void FOC_Current(FocState *f, MotorControlState *m, float phase, float speed)
{
    (void)f;
    (void)m;
    (void)phase;
    (void)speed;
}
void power_stage_hw_start(void)
{
}
float Math_Constrain(float value, float low, float high)
{
    return fminf(fmaxf(value, low), high);
}

static void completed_samples(int16_t magnitude)
{
    memset(&s_cogging_calib, 0, sizeof(s_cogging_calib));
    s_cogging_calib.state = COGGING_COMPLETE;
    s_cogging_calib.full_scale_a = CURRENT_SENSE_PROFILE_FULL_SCALE_A;
    for (unsigned i = 0U; i < COGGING_MAP_POINTS; ++i)
    {
        s_cogging_calib.iq_q15[i] = (i & 1U) != 0U ? magnitude : -magnitude;
    }
    s_finalize_pending = true;
    s_session_active = false;
}

int main(void)
{
    MotorControlState motor = {0};
    EncoderState encoder = {0};
    CoggingMapRecord original, candidate, exported;
    motor.axis_profile_valid = true;
    motor.motor_pole_pairs = 21;
    encoder.calib_flag = ENC_CALIB_ALL;
    encoder.electrical_zero_q15 = 3155U;
    completed_samples(123);
    assert(CoggingMap_Build(s_cogging_calib.iq_q15,
                            s_cogging_calib.full_scale_a,
                            signature(&motor, &encoder),
                            &original));
    FocCogging_ImportMap(&original);

    completed_samples(321);
    assert(FocCogging_BeginFinalize() && !FocCogging_IsReleased());
    assert(FocCogging_BuildCandidate(&candidate, &motor, &encoder));
    assert(candidate.crc32 != original.crc32);
    FocCogging_Abort();
    assert(s_cogging_calib.state == COGGING_COMPLETE && s_cogging_calib.iq_q15[1] == 321);
    assert(!FocCogging_PublishCandidate(&candidate));
    assert(!FocCogging_IsReleased());
    FocCogging_ReleaseFinalize(false);
    assert(FocCogging_IsReleased() && s_cogging_calib.reason == COGGING_CANCELLED);
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
    assert(FocCogging_GetState() == COGGING_FAILED &&
           s_cogging_calib.reason == COGGING_SAVE_FAILED);
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
