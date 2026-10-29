#include "bsp_pwm.h"
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
static bool allow_expected_fault;
static BspResult pwm_result = BSP_OK;

static void record(unsigned side, unsigned event)
{
    assert(captures[side].count < 8);
    captures[side].events[captures[side].count++] = event;
}
BspResult BspPwm_SetDuty(float a, float b, float c)
{
    record(0, 2);
    captures[0].writes++;
    captures[0].duty[0] = a;
    captures[0].duty[1] = b;
    captures[0].duty[2] = c;
    return pwm_result;
}
BspResult BspPwm_SetDuty_Reference(float a, float b, float c)
{
    record(1, 2);
    captures[1].writes++;
    captures[1].duty[0] = a;
    captures[1].duty[1] = b;
    captures[1].duty[2] = c;
    return BSP_OK;
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
    assert(allow_expected_fault);
    if (motor->fault_code == MOTOR_FAULT_NONE)
    {
        motor->fault_code = fault;
    }
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

static void test_voltage_law(void)
{
    MotorConfiguration configuration = {0};
    configuration.application.current_limit_a = 3.0f;
    MotorControlState motor = {.thermal_current_limit_a = 3.0f};
    FocState foc = {.bus_voltage_filtered_v = 29.0f,
                    .phase_a_current_a = 0.2f,
                    .phase_b_current_a = -0.1f,
                    .phase_c_current_a = -0.1f};
    foc.id_pi.integral_output = 0.7f;
    foc.iq_pi.integral_output = -0.3f;
    PiController id = foc.id_pi, iq = foc.iq_pi;
    memset(captures, 0, sizeof(captures));
    FOC_Step(&configuration, &foc, &motor, FOC_CONTROL_VOLTAGE, 0.0f, 0.0f, 0.0f, 2.0f);
    assert(fabsf(foc.id_a - 0.2f) < 1e-6f && fabsf(foc.iq_a) < 1e-6f);
    assert(fabsf(foc.mod_q - 3.0f / 29.0f) < 1e-6f && foc.mod_d == 0.0f);
    assert(captures[0].writes == 1U && fabsf(foc.duty_a - 0.5f) < 1e-6f);
    float delta = 3.0f / (29.0f * sqrtf(3.0f));
    assert(fabsf(foc.duty_b - (0.5f - delta)) < 1e-6f);
    assert(fabsf(foc.duty_c - (0.5f + delta)) < 1e-6f);
    assert(memcmp(&id, &foc.id_pi, sizeof(id)) == 0 && memcmp(&iq, &foc.iq_pi, sizeof(iq)) == 0);
    FOC_Step(&configuration, &foc, &motor, FOC_CONTROL_VOLTAGE, 0.0f, 0.0f, 100.0f, 100.0f);
    assert(fabsf(foc.mod_d - foc.mod_q) < 1e-6f);
    assert(fabsf(hypotf(foc.mod_d, foc.mod_q) - 0.95f * sqrtf(3.0f) * 0.5f) < 1e-6f);
    assert(foc.duty_a >= 0 && foc.duty_a <= 1 && foc.duty_b >= 0 && foc.duty_b <= 1 &&
           foc.duty_c >= 0 && foc.duty_c <= 1);
    allow_expected_fault = true;
    memset(captures, 0, sizeof(captures));
    foc.phase_a_current_a = 4.0f;
    foc.phase_b_current_a = foc.phase_c_current_a = -2.0f;
    FOC_Step(&configuration, &foc, &motor, FOC_CONTROL_VOLTAGE, 0, 0, 0, 2);
    assert(motor.fault_code == MOTOR_FAULT_WINDING_THERMAL && captures[0].writes == 0);
    motor.fault_code = MOTOR_FAULT_NONE;
    foc.phase_a_current_a = foc.phase_b_current_a = foc.phase_c_current_a = 0;
    pwm_result = BSP_IO_ERROR;
    FOC_Step(&configuration, &foc, &motor, FOC_CONTROL_VOLTAGE, 0, 0, 0, 2);
    assert(motor.fault_code == MOTOR_FAULT_CONTROL_OVERRUN && captures[0].writes == 1);
    pwm_result = BSP_OK;
    allow_expected_fault = false;
    puts("PASS voltage law: measured dq, analytical SVPWM, vector limit, unchanged PI, current/PWM "
         "faults");
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
                /* NaN符号不属于无效功率值的合同；有限值与其它状态仍逐字节核对。 */
                if (isnan(actual.power_filtered_w) && isnan(reference.power_filtered_w))
                {
                    reference.power_filtered_w = actual.power_filtered_w;
                }
                if (memcmp(&actual, &reference, sizeof(actual)) ||
                    memcmp(&captures[0], &captures[1], sizeof(Capture)))
                {
                    fprintf(stderr,
                            "FOC mismatch mode=%u scenario=%u tick=%u\n",
                            active_mode,
                            scenario,
                            tick);
                    const unsigned char *a = (const unsigned char *)&actual;
                    const unsigned char *r = (const unsigned char *)&reference;
                    for (unsigned byte = 0; byte < sizeof(actual); ++byte)
                    {
                        if (a[byte] != r[byte])
                        {
                            fprintf(
                                stderr, "state byte %u: %02x != %02x\n", byte, a[byte], r[byte]);
                        }
                    }
                    return 1;
                }
                assert(captures[0].writes == 1);
#if defined(BSP_ENABLED_FAST_LOOP_PROFILE) && BSP_ENABLED_FAST_LOOP_PROFILE
                assert(captures[0].count == (active_mode == 2 ? 1U : 3U));
#else
                assert(captures[0].count == 1U);
#endif
                ++compared;
            }
        }
    }
    test_voltage_law();
    printf("PASS %u FOC comparisons: complete state, PI, PWM and profiling order\n", compared);
    return 0;
}
