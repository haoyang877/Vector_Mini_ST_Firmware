#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <string.h>
#include "phase_resistance_task.h"
#include "friction_identification_task.h"
#include "maintenance_profiles.h"
#include "foc_errhandle.h"
#include "foc_speed.h"
#include "motor_hw.h"
#include "utils.h"

/* 理想相阻与摩擦模型仅替换电流/速度执行和传感器；实际功能及辨识核心参与。 */
static MotorControlState motor;
static FocState foc;
static EncoderState encoder;
static PiController pi;

void motor_hw_pwm_force_high_sides(void)
{
}
void FOC_CurrentController_Reset(FocState *f)
{
    f->id_a = f->iq_a = 0.0f;
}
void PiController_Reset(PiController *p)
{
    memset(p, 0, sizeof(*p));
}
void MotorFault_Report(MotorControlState *m, MotorFaultCode e)
{
    if (m != NULL && m->fault_code == MOTOR_FAULT_NONE)
    {
        m->fault_code = e;
    }
}
float Math_Abs(float x)
{
    return fabsf(x);
}
bool Encoder_IsOnline(const EncoderState *e)
{
    return e->bad_frame_streak == 0U;
}
uint8_t Encoder_GetCalibFlag(const EncoderState *e)
{
    return e->calib_flag;
}
float Encoder_GetMecPos(const EncoderState *e)
{
    return e->mechanical_position_rad;
}
float Encoder_GetMecVel(const EncoderState *e)
{
    return e->mechanical_velocity_rad_s;
}
float Encoder_GetElePhase(const EncoderState *e)
{
    return e->mechanical_position_rad;
}
float Encoder_GetEleVel(const EncoderState *e)
{
    return e->mechanical_velocity_rad_s;
}
void FOC_Current(FocState *f, MotorControlState *m, float phase, float vel)
{
    (void)phase;
    (void)vel;
    f->id_a = f->id_filtered_a = m->id_target_a;
    f->iq_a = f->iq_filtered_a = 0.0f;
    f->mod_d = 0.5f * m->id_target_a * 1.5f / f->bus_voltage_v;
    f->mod_q = 0.0f;
}
void SpeedMode_Run(FocState *f,
                   MotorControlState *m,
                   PiController *p,
                   float angle,
                   float electrical_vel,
                   float mechanical_vel)
{
    (void)p;
    (void)angle;
    (void)electrical_vel;
    (void)mechanical_vel;
    m->speed_ramped_rad_s = m->speed_target_rad_s;
    encoder.mechanical_velocity_rad_s = m->speed_target_rad_s;
    encoder.mechanical_position_rad += encoder.mechanical_velocity_rad_s * 0.00005f;
    f->iq_filtered_a = encoder.mechanical_velocity_rad_s == 0.0f
                           ? 0.0f
                           : copysignf(0.2f, encoder.mechanical_velocity_rad_s) +
                                 0.01f * encoder.mechanical_velocity_rad_s;
    m->iq_target_a = f->iq_filtered_a;
}
static void reset(void)
{
    memset(&motor, 0, sizeof(motor));
    memset(&foc, 0, sizeof(foc));
    memset(&encoder, 0, sizeof(encoder));
    motor.current_limit_a = 6.0f;
    motor.speed_limit_rad_s = 20.0f;
    motor.speed_kp = 1.0f;
    motor.speed_ki = 1.0f;
    motor.control_mode = MOTOR_CONTROL_NONE;
    foc.bus_voltage_v = foc.bus_voltage_filtered_v = 30.0f;
    foc.temperature_c = 25.0f;
    encoder.calib_flag = ENC_CALIB_ALL;
    PhaseResistanceTask_Cancel(&foc, &motor);
    FrictionIdentificationTask_Abort(&motor, &pi);
}
static uint32_t trace_hash;
static uint32_t trace_ticks;

/* 逐拍比较命令、反馈、故障与结果，不包含结构填充字节。 */
static void trace_step(MotorTaskResult result)
{
    const float values[] = {motor.id_target_a,
                            motor.iq_target_a,
                            motor.speed_target_rad_s,
                            motor.speed_ramped_rad_s,
                            foc.id_a,
                            foc.iq_a,
                            foc.iq_filtered_a,
                            encoder.mechanical_position_rad,
                            encoder.mechanical_velocity_rad_s,
                            motor.phase_resistance_a_ohm,
                            motor.phase_resistance_b_ohm,
                            motor.phase_resistance_c_ohm};
    for (unsigned n = 0U; n < sizeof(values) / sizeof(values[0]); ++n)
    {
        uint32_t bits;
        memcpy(&bits, &values[n], sizeof(bits));
        trace_hash = (trace_hash ^ bits) * 16777619U;
    }
    trace_hash = (trace_hash ^ (uint32_t)result.result) * 16777619U;
    trace_hash = (trace_hash ^ (uint32_t)result.error) * 16777619U;
    ++trace_ticks;
}
int main(void)
{
    for (unsigned trial = 0U; trial < 2U; ++trial)
    {
        reset();
        trace_hash = 2166136261U;
        trace_ticks = 0U;
        MotorTaskResult r = {MOTOR_TASK_RUNNING, MOTOR_FAULT_NONE};
        for (unsigned n = 0U; n < 1000000U && r.result == MOTOR_TASK_RUNNING; ++n)
        {
            r = PhaseResistanceTask_Run(&foc, &motor, &s_phase_resistance_config);
            trace_step(r);
        }
        assert(r.result == MOTOR_TASK_COMPLETED && r.error == MOTOR_FAULT_NONE);
        assert(motor.phase_resistance_valid && PhaseResistanceTask_IsReleased());
        assert(motor.id_target_a == 0.0f && motor.iq_target_a == 0.0f);
        printf("phase trial=%u ticks=%u trace=%08x\n", trial, trace_ticks, trace_hash);
    }
    reset();
    (void)PhaseResistanceTask_Run(&foc, &motor, &s_phase_resistance_config);
    assert(!PhaseResistanceTask_IsReleased());
    foc.bus_voltage_filtered_v = 20.0f;
    MotorTaskResult r = PhaseResistanceTask_Run(&foc, &motor, &s_phase_resistance_config);
    assert(r.result == MOTOR_TASK_FAILED && r.error == MOTOR_FAULT_UNDERVOLTAGE);
    assert(PhaseResistanceTask_IsReleased());
    puts("PASS actual phase-resistance completion/failure releases without idle-dispatch "
         "cancellation");
    for (unsigned trial = 0U; trial < 2U; ++trial)
    {
        reset();
        trace_hash = 2166136261U;
        trace_ticks = 0U;
        r = (MotorTaskResult){MOTOR_TASK_RUNNING, MOTOR_FAULT_NONE};
        for (unsigned n = 0U; n < 4000000U && r.result == MOTOR_TASK_RUNNING; ++n)
        {
            r = FrictionIdentificationTask_Task(&foc, &motor, &pi, &encoder, &s_friction_config);
            trace_step(r);
        }
        assert(r.result == MOTOR_TASK_COMPLETED && r.error == MOTOR_FAULT_NONE);
        assert(FrictionIdentificationTask_IsReleased());
        assert(FrictionIdentificationTask_ApplyCandidate(&motor) && motor.friction_model_valid);
        assert(fabsf(motor.friction_coulomb_pos_a - 0.2f) < 0.01f);
        assert(motor.id_target_a == 0.0f && motor.iq_target_a == 0.0f);
        printf("friction trial=%u ticks=%u trace=%08x model=%a,%a,%a,%a\n",
               trial,
               trace_ticks,
               trace_hash,
               (double)motor.friction_coulomb_pos_a,
               (double)motor.friction_coulomb_neg_a,
               (double)motor.friction_viscous_pos_a_per_rad_s,
               (double)motor.friction_viscous_neg_a_per_rad_s);
    }
    reset();
    (void)FrictionIdentificationTask_Task(&foc, &motor, &pi, &encoder, &s_friction_config);
    assert(!FrictionIdentificationTask_IsReleased());
    encoder.bad_frame_streak = 1U;
    r = FrictionIdentificationTask_Task(&foc, &motor, &pi, &encoder, &s_friction_config);
    assert(r.result == MOTOR_TASK_FAILED && r.error == MOTOR_FAULT_ENCODER);
    assert(FrictionIdentificationTask_IsReleased());
    puts("PASS actual friction completion/failure releases and candidate remains available across "
         "repeat requests");
#ifndef MAINTENANCE_LEGACY_REFERENCE
    reset();
    r = PhaseResistanceTask_Run(&foc, &motor, NULL);
    assert(r.result == MOTOR_TASK_FAILED && r.error == MOTOR_FAULT_INVALID_PARAMETERS);
    assert(PhaseResistanceTask_IsReleased() && motor.id_target_a == 0.0f &&
           motor.iq_target_a == 0.0f);
    reset();
    r = FrictionIdentificationTask_Task(&foc, &motor, &pi, &encoder, NULL);
    assert(r.result == MOTOR_TASK_FAILED && r.error == MOTOR_FAULT_FRICTION_IDENTIFICATION);
    assert(FrictionIdentificationTask_IsReleased());
    FrictionIdentificationTaskConfig invalid = s_friction_config;
    invalid.current_ratio_max = NAN;
    reset();
    r = FrictionIdentificationTask_Task(&foc, &motor, &pi, &encoder, &invalid);
    assert(r.result == MOTOR_TASK_FAILED && r.error == MOTOR_FAULT_FRICTION_IDENTIFICATION);
    assert(FrictionIdentificationTask_IsReleased());
#endif
    return 0;
}
