#include "foc_internal.h"

#include "bsp_pwm.h"
#include "motor_fault.h"

/* 唯一PWM提交入口；先计算，再提交，再更新诊断和积分跟踪。 */
void Foc_Step(const MotorConfiguration *configuration,
              FocState *foc,
              MotorControlState *motor,
              FocControlLaw law,
              float phase,
              float phase_vel,
              float d_reference,
              float q_reference)
{
    if (!Foc_Prepare(configuration, foc, motor, law, phase, phase_vel, d_reference, q_reference))
    {
        return;
    }
    void (*set_duty)(float, float, float) = foc->set_duty != NULL ? foc->set_duty : BspPwm_SetDuty;
    set_duty(foc->duty_abc.a, foc->duty_abc.b, foc->duty_abc.c);
    Foc_Complete(foc, law);
}

/* 标定和台架兼容入口只绑定策略与参考，不再分别实现一套FOC。 */
void Foc_RunCurrent(const MotorConfiguration *configuration,
                    FocState *foc,
                    MotorControlState *motor_state,
                    float phase,
                    float phase_vel)
{
    Foc_Step(configuration,
             foc,
             motor_state,
             FOC_CONTROL_CURRENT,
             phase,
             phase_vel,
             motor_state->id_target_a,
             motor_state->iq_target_a);
}

void Foc_RunCurrentRef(const MotorConfiguration *configuration,
                       FocState *foc,
                       MotorControlState *motor_state,
                       float phase,
                       float phase_vel,
                       float iq_reference)
{
    Foc_Step(configuration,
             foc,
             motor_state,
             FOC_CONTROL_CURRENT,
             phase,
             phase_vel,
             motor_state->id_target_a,
             iq_reference);
}

void Foc_RunVq(const MotorConfiguration *configuration,
               FocState *foc,
               MotorControlState *motor_state,
               float phase,
               float phase_vel)
{
    Foc_Step(configuration,
             foc,
             motor_state,
             FOC_CONTROL_ID_VQ,
             phase,
             phase_vel,
             0.0f,
             motor_state->vq_target_v);
}
