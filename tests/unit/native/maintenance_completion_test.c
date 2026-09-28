#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <string.h>
#include "foc_phase_resistance.h"
#include "foc_friction_identification.h"
#include "foc_errhandle.h"
#include "foc_speed.h"
#include "motor_hw.h"
#include "utils.h"

/* 理想相阻与摩擦模型仅替换电流/速度执行和传感器；实际功能及辨识核心参与。 */
static MotorControl_TypeDef motor;
static FOC_TypeDef foc;
static Encoder_TypeDef encoder;
static PI_Controller_TypeDef pi;

void motor_hw_pwm_force_high_sides(void)
{
}
void FOC_CurrentController_Reset(FOC_TypeDef *f)
{
    f->Id = f->Iq = 0.0f;
}
void PI_Controller_Reset(PI_Controller_TypeDef *p)
{
    memset(p, 0, sizeof(*p));
}
void MotorFault_Report(MotorControl_TypeDef *m, ErrorNow_TypeDef e)
{
    if (m != NULL && m->ErrorNow == No_Error)
    {
        m->ErrorNow = e;
    }
}
float fast_abs(float x)
{
    return fabsf(x);
}
bool Encoder_IsOnline(const Encoder_TypeDef *e)
{
    return e->bad_frame_streak == 0U;
}
uint8_t Encoder_GetCalibFlag(const Encoder_TypeDef *e)
{
    return e->calib_flag;
}
float Encoder_GetMecPos(const Encoder_TypeDef *e)
{
    return e->theta_mech;
}
float Encoder_GetMecVel(const Encoder_TypeDef *e)
{
    return e->vel_mech;
}
float Encoder_GetElePhase(const Encoder_TypeDef *e)
{
    return e->theta_mech;
}
float Encoder_GetEleVel(const Encoder_TypeDef *e)
{
    return e->vel_mech;
}
void FOC_Current(FOC_TypeDef *f, MotorControl_TypeDef *m, float phase, float vel)
{
    (void)phase;
    (void)vel;
    f->Id = f->Id_filt = m->idRef;
    f->Iq = f->Iq_filt = 0.0f;
    f->mod_d = 0.5f * m->idRef * 1.5f / f->Vbus;
    f->mod_q = 0.0f;
}
void SpeedMode_Run(FOC_TypeDef *f,
                   MotorControl_TypeDef *m,
                   PI_Controller_TypeDef *p,
                   float angle,
                   float electrical_vel,
                   float mechanical_vel)
{
    (void)p;
    (void)angle;
    (void)electrical_vel;
    (void)mechanical_vel;
    m->speedShadow = m->speedRef;
    encoder.vel_mech = m->speedRef;
    encoder.theta_mech += encoder.vel_mech * 0.00005f;
    f->Iq_filt = encoder.vel_mech == 0.0f
                     ? 0.0f
                     : copysignf(0.2f, encoder.vel_mech) + 0.01f * encoder.vel_mech;
    m->iqRef = f->Iq_filt;
}
static void reset(void)
{
    memset(&motor, 0, sizeof(motor));
    memset(&foc, 0, sizeof(foc));
    memset(&encoder, 0, sizeof(encoder));
    motor.current_limit = 6.0f;
    motor.speed_limit = 20.0f;
    motor.speed_Kp = 1.0f;
    motor.speed_Ki = 1.0f;
    motor.control_mode = MOTOR_CONTROL_NONE;
    foc.Vbus = foc.Vbus_filt = 30.0f;
    foc.temp = 25.0f;
    encoder.calib_flag = ENC_CALIB_ALL;
    PhaseResistanceMode_Cancel(&foc, &motor);
    FocFrictionIdentification_Abort(&motor, &pi);
}
int main(void)
{
    for (unsigned trial = 0U; trial < 2U; ++trial)
    {
        reset();
        MotorTaskResult r = {MOTOR_TASK_RUNNING, No_Error};
        for (unsigned n = 0U; n < 1000000U && r.result == MOTOR_TASK_RUNNING; ++n)
        {
            r = PhaseResistanceMode_Run(&foc, &motor);
        }
        assert(r.result == MOTOR_TASK_COMPLETED && r.error == No_Error);
        assert(motor.phase_resistance_valid && PhaseResistanceMode_IsReleased());
        assert(motor.idRef == 0.0f && motor.iqRef == 0.0f);
    }
    reset();
    (void)PhaseResistanceMode_Run(&foc, &motor);
    assert(!PhaseResistanceMode_IsReleased());
    foc.Vbus_filt = 20.0f;
    MotorTaskResult r = PhaseResistanceMode_Run(&foc, &motor);
    assert(r.result == MOTOR_TASK_FAILED && r.error == Under_Voltage);
    assert(PhaseResistanceMode_IsReleased());
    puts("PASS actual phase-resistance completion/failure releases without idle-dispatch "
         "cancellation");
    for (unsigned trial = 0U; trial < 2U; ++trial)
    {
        reset();
        r = (MotorTaskResult){MOTOR_TASK_RUNNING, No_Error};
        for (unsigned n = 0U; n < 4000000U && r.result == MOTOR_TASK_RUNNING; ++n)
        {
            r = FocFrictionIdentification_Task(&foc, &motor, &pi, &encoder);
        }
        assert(r.result == MOTOR_TASK_COMPLETED && r.error == No_Error);
        assert(FocFrictionIdentification_IsReleased());
        assert(FocFrictionIdentification_ApplyCandidate(&motor) && motor.friction_model_valid);
        assert(fabsf(motor.friction_coulomb_pos_a - 0.2f) < 0.01f);
        assert(motor.idRef == 0.0f && motor.iqRef == 0.0f);
    }
    reset();
    (void)FocFrictionIdentification_Task(&foc, &motor, &pi, &encoder);
    assert(!FocFrictionIdentification_IsReleased());
    encoder.bad_frame_streak = 1U;
    r = FocFrictionIdentification_Task(&foc, &motor, &pi, &encoder);
    assert(r.result == MOTOR_TASK_FAILED && r.error == Encoder_Error);
    assert(FocFrictionIdentification_IsReleased());
    puts("PASS actual friction completion/failure releases and candidate remains available across "
         "repeat requests");
    return 0;
}
