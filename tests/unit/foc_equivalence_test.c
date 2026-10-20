/* 冻结旧 FOC 与当前实际入口逐拍对照；硬件输出只记录，不访问设备。 */
#ifdef NDEBUG
#undef NDEBUG
#endif
#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <string.h>
#include "foc_algorithm.h"
#include "fast_loop_profile.h"
#include "motor_thermal_control.h"
#include "control_config.h"

void FOC_Current_Reference(
    const MotorConfiguration *, FocState *, MotorControlState *, float, float);
void FOC_CurrentWithReference_Reference(
    const MotorConfiguration *, FocState *, MotorControlState *, float, float, float);
void FOC_Vq_Mode_Reference(
    const MotorConfiguration *, FocState *, MotorControlState *, float, float);

typedef struct
{
    float duty[3];
    unsigned events[8];
    unsigned count, writes;
} Capture;
static Capture captures[2];

static void record(unsigned side, unsigned event)
{
    assert(captures[side].count < 8);
    captures[side].events[captures[side].count++] = event;
}
void motor_hw_pwm_set_duty(float a, float b, float c)
{
    record(0, 2);
    captures[0].writes++;
    captures[0].duty[0] = a;
    captures[0].duty[1] = b;
    captures[0].duty[2] = c;
}
void motor_hw_pwm_set_duty_Reference(float a, float b, float c)
{
    record(1, 2);
    captures[1].writes++;
    captures[1].duty[0] = a;
    captures[1].duty[1] = b;
    captures[1].duty[2] = c;
}
void FastLoopProfile_Begin(unsigned stage)
{
    record(0, 100 + stage);
}
void FastLoopProfile_End(unsigned stage)
{
    record(0, 200 + stage);
}
void FastLoopProfile_Begin_Reference(unsigned stage)
{
    record(1, 100 + stage);
}
void FastLoopProfile_End_Reference(unsigned stage)
{
    record(1, 200 + stage);
}

void MotorFault_Report(MotorControlState *motor, MotorFaultCode fault)
{
    (void)motor;
    (void)fault;
    assert(false);
}

/* 与Axis配置提交边界一致，只绑定固定参数，不改变历史积分或动态限幅。 */
static void bind_current_configuration(FocState *foc, const MotorConfiguration *configuration)
{
    PiController_Configure(&foc->id_pi,
                           configuration->model.id_kp,
                           configuration->model.id_ki,
                           Current_Ts,
                           foc->id_pi.output_min,
                           foc->id_pi.output_max);
    PiController_Configure(&foc->iq_pi,
                           configuration->model.iq_kp,
                           configuration->model.iq_ki,
                           Current_Ts,
                           foc->iq_pi.output_min,
                           foc->iq_pi.output_max);
}

int main(void)
{
    /* 等价比较限定在不触发新增热限制的输入，热限流另有真实入口反例。 */
    MotorWindingThermalConfig thermal;
    MotorWindingThermal_Default(&thermal);
    thermal.peak_a = 100.0f;
    assert(MotorThermalControl_Init(&thermal, 0U));
    unsigned compared = 0;
    const float bus[] = {24, 48, 0, -1, NAN, INFINITY, 0.0001f, 12};
    for (unsigned mode = 0; mode < 3; mode++)
    {
        for (unsigned scenario = 0; scenario < 40; scenario++)
        {
            FocState actual = {0}, reference = {0};
            MotorConfiguration configuration = {0};
            MotorControlState motor = {0};
            configuration.application.current_limit_a = 100.0f;
            motor.thermal_current_limit_a = MotorThermalControl_CurrentLimit(100.0f);
            configuration.model.id_kp = .03f * (scenario % 5);
            configuration.model.iq_kp = .05f * (scenario % 7);
            configuration.model.id_ki = .3f * (scenario % 6);
            configuration.model.iq_ki = .2f * (scenario % 8);
            bind_current_configuration(&actual, &configuration);
            bind_current_configuration(&reference, &configuration);
            for (unsigned tick = 0; tick < 512; tick++)
            {
                /* 模拟停机提交后的首次控制拍；不用重置PI历史来掩盖绑定错误。 */
                if (tick == 256U)
                {
                    configuration.model.id_kp += 0.015f;
                    configuration.model.iq_ki += 0.025f;
                    bind_current_configuration(&actual, &configuration);
                    bind_current_configuration(&reference, &configuration);
                }
                memset(captures, 0, sizeof(captures));
                actual.bus_voltage_filtered_v =
                    tick % 37 == 0 ? 0 : bus[(scenario + tick / 73U) % 8];
                actual.phase_a_current_a = .2f * (float)((int)((tick * 7) % 41) - 20);
                actual.phase_b_current_a = .3f * (float)((int)((tick * 11) % 37) - 18);
                actual.phase_c_current_a = -(actual.phase_a_current_a + actual.phase_b_current_a);
                reference.bus_voltage_filtered_v = actual.bus_voltage_filtered_v;
                reference.phase_a_current_a = actual.phase_a_current_a;
                reference.phase_b_current_a = actual.phase_b_current_a;
                reference.phase_c_current_a = actual.phase_c_current_a;
                motor.id_target_a = .1f * (float)((int)(tick % 61) - 30);
                motor.iq_target_a = .5f * (float)((int)(tick % 81) - 40);
                motor.vq_target_v = (float)((int)(tick % 121) - 60);
                float phase = .01f * (float)((int)(tick % 601) - 300);
                float velocity = (float)((int)((tick * 19) % 6001) - 3000);
                unsigned active_mode = (mode + tick / 128U) % 3U;
                if (active_mode == 0)
                {
                    FOC_CurrentWithReference(
                        &configuration, &actual, &motor, phase, velocity, motor.iq_target_a + .5f);
                    FOC_CurrentWithReference_Reference(&configuration,
                                                       &reference,
                                                       &motor,
                                                       phase,
                                                       velocity,
                                                       motor.iq_target_a + .5f);
                }
                else if (active_mode == 1)
                {
                    FOC_Current(&configuration, &actual, &motor, phase, velocity);
                    FOC_Current_Reference(&configuration, &reference, &motor, phase, velocity);
                }
                else
                {
                    FOC_Vq_Mode(&configuration, &actual, &motor, phase, velocity);
                    FOC_Vq_Mode_Reference(&configuration, &reference, &motor, phase, velocity);
                }
                if (memcmp(&actual, &reference, sizeof(actual)) ||
                    memcmp(&captures[0], &captures[1], sizeof(Capture)))
                {
                    fprintf(stderr,
                            "FOC mismatch mode=%u scenario=%u tick=%u\n",
                            active_mode,
                            scenario,
                            tick);
                    return 1;
                }
                assert(captures[0].writes == 1);
#if defined(FAST_LOOP_STAGE_PROFILE) && FAST_LOOP_STAGE_PROFILE
                assert(captures[0].count == (active_mode == 2 ? 1U : 3U));
#else
                assert(captures[0].count == 1U);
#endif
                ++compared;
            }
        }
    }
    printf("PASS %u FOC comparisons: complete state, PI, PWM and profiling order\n", compared);
    return 0;
}
