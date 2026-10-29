/* 绕组热纯核与真实APP/FOC入口定向检查；只有PWM记录桩，无硬件访问。 */
#include <assert.h>
#include "bsp_pwm.h"
#include <math.h>
#include <stdio.h>
#include <string.h>
#include "motor_thermal_control.h"
#include "foc_algorithm.h"
#include "control_config.h"

static unsigned s_pwm_writes;
static const FocState *s_submission_state;
static float s_integral_at_submission, s_filter_at_submission;
static BspResult s_submission_result = BSP_OK;
BspResult BspPwm_SetDuty(float a, float b, float c)
{
    assert(isfinite(a) && isfinite(b) && isfinite(c));
    ++s_pwm_writes;
    if (s_submission_state != NULL)
    {
        s_integral_at_submission = s_submission_state->id_pi.integral_output;
        s_filter_at_submission = s_submission_state->id_filtered_a;
    }
    return s_submission_result;
}
void MotorFault_Report(MotorControlState *motor, MotorFaultCode fault)
{
    if (motor->fault_code == MOTOR_FAULT_NONE)
    {
        motor->fault_code = fault;
    }
}

/* 按实际20kHz节拍喂入三相电流；每20拍推进1ms，不跳过热采样。 */
static void window(float ia, float ib, float ic, uint32_t *now_ms)
{
    ++*now_ms;
    for (unsigned i = 0U; i < 20U; ++i)
    {
        MotorThermalControl_Sample(ia, ib, ic, false, *now_ms);
    }
}

static void test_core(void)
{
    MotorWindingThermalConfig config;
    MotorWindingThermal_Default(&config);
    assert(config.continuous_vector_a == 3.0f && config.peak_a == 7.0f);
    MotorThermalConfig derived;
    assert(MotorWindingThermal_Derive(&config, &derived));
    assert(derived.samples_per_update == 20U && derived.cooling_tau_s == 120.0f);
    MotorThermalState state;
    assert(MotorThermal_Init(&state, &derived, 0.0f));
    for (unsigned i = 0U; i < 20U; ++i)
    {
        assert(MotorThermal_Accumulate(&state, 3.0f, -1.5f, -1.5f));
    }
    assert(MotorThermal_Update(&state));
    assert(state.heat[1] > state.heat[0] && state.heat[0] > 0.0f);
    assert(MotorThermal_ResolveLimit(6.0f, 7.0f, true) == 6.0f);
    float id = 3.0f, iq = 4.0f;
    assert(MotorThermal_LimitVector(2.5f, &id, &iq));
    assert(fabsf(id - 1.5f) < 0.000001f && fabsf(iq - 2.0f) < 0.000001f);
    assert(!MotorThermal_Accumulate(&state, NAN, 0.0f, 0.0f));
    assert(!state.valid && state.fault_latched && state.available_a == 0.0f);
    assert(!MotorThermal_ClearFault(&state));
    puts(
        "PASS thermal pure config, per-phase hot spot, min limit, vector norm and invalid samples");
}

static void test_runtime(void)
{
    MotorWindingThermalConfig config;
    MotorWindingThermal_Default(&config);
    config.heating_tau_s = 0.1f;
    assert(!MotorThermalControl_CanRun(true));
    assert(MotorThermalControl_CurrentLimit(6.0f) == 0.0f);
    assert(MotorThermalControl_Init(&config, 0U));
    uint32_t now_ms = 0U;
    for (unsigned i = 0U; i < 100U && MotorThermalControl_CanRun(false); ++i)
    {
        window(6.0f, -3.0f, -3.0f, &now_ms);
    }
    assert(MotorThermalControl_CanRun(true) && !MotorThermalControl_CanRun(false));
    MotorThermalDiagnostics before, after;
    assert(MotorThermalControl_GetDiagnostics(&before));
    MotorWindingThermalConfig invalid = config;
    invalid.peak_a = 2.0f;
    assert(!MotorThermalControl_Configure(&invalid, true));
    assert(!MotorThermalControl_Configure(&config, false));
    assert(MotorThermalControl_GetDiagnostics(&after));
    assert(memcmp(&before, &after, sizeof(before)) == 0);
    assert(MotorThermalControl_Configure(&config, true));
    assert(MotorThermalControl_GetDiagnostics(&after));
    assert(memcmp(before.model.heat, after.model.heat, sizeof(before.model.heat)) == 0);
    assert(MotorThermalControl_Init(&config, now_ms));
    assert(MotorThermalControl_GetDiagnostics(&after));
    assert(after.model.max_heat == before.model.max_heat);
    assert(!MotorThermalControl_Guard(false));
    assert(MotorThermalControl_CurrentLimit(6.0f) == 0.0f);
    assert(!MotorThermalControl_CanClearFault() && !MotorThermalControl_ClearFault());
    for (unsigned i = 0U; i < 120000U && !MotorThermalControl_CanClearFault(); ++i)
    {
        window(0.0f, 0.0f, 0.0f, &now_ms);
    }
    assert(MotorThermalControl_GetDiagnostics(&before));
    assert(before.model.max_heat > 0.0f && before.model.fault_latched);
    assert(MotorThermalControl_ClearFault());
    assert(MotorThermalControl_GetDiagnostics(&after));
    assert(after.model.max_heat == before.model.max_heat && !after.model.fault_latched);
    assert(MotorThermalControl_CurrentLimit(6.0f) == 6.0f);
    puts("PASS real thermal runtime: derating policy, reject atomicity, history-preserving config "
         "and explicit cooled CLEAR");
}

static void test_watchdog(void)
{
    MotorWindingThermalConfig config;
    MotorWindingThermal_Default(&config);
    assert(MotorThermalControl_Init(&config, 0U));
    uint32_t now_ms = 0U;
    window(0.0f, 0.0f, 0.0f, &now_ms);
    assert(MotorThermalControl_CheckWatchdog(3U, false));
    assert(!MotorThermalControl_CheckWatchdog(4U, false));
    assert(MotorThermalControl_CurrentLimit(6.0f) == 0.0f);
    assert(!MotorThermalControl_CanRun(true) && !MotorThermalControl_ClearFault());
    assert(MotorThermalControl_CheckWatchdog(100U, true));
    assert(!MotorThermalControl_CanRun(true));
    puts("PASS real thermal watchdog: 2ms boundary, missing fast samples latch and no implicit "
         "recovery");
}

static void test_protection(void)
{
    MotorConfiguration configuration = {0};
    MotorControlState motor = {0};
    FocState foc = {0};
    configuration.application.current_limit_a = 2.5f;
    /* 未完成零偏只发布关闭额度，不能把无样本解释为绕组故障。 */
    MotorThermalControl_UpdateProtection(&configuration, &foc, &motor, false, true, false, true);
    assert(motor.thermal_current_limit_a == 0.0f && motor.thermal_limit_active);
    assert(!motor.thermal_clear_ready && motor.fault_code == MOTOR_FAULT_NONE);
    MotorThermalControl_UpdateProtection(&configuration, &foc, &motor, true, true, false, true);
    assert(motor.fault_code == MOTOR_FAULT_WINDING_THERMAL);

    MotorWindingThermalConfig config;
    MotorWindingThermal_Default(&config);
    config.heating_tau_s = 0.1f;
    assert(MotorThermalControl_Init(&config, 0U));
    motor.fault_code = MOTOR_FAULT_NONE;
    motor.control_mode = MOTOR_CONTROL_VQ;
    foc.phase_a_current_a = 3.0f;
    foc.phase_b_current_a = foc.phase_c_current_a = -1.5f;
    MotorThermalControl_UpdateProtection(&configuration, &foc, &motor, true, true, true, false);
    assert(motor.fault_code == MOTOR_FAULT_NONE && motor.thermal_current_limit_a == 2.5f);
    MotorThermalControl_UpdateProtection(&configuration, &foc, &motor, true, false, true, false);
    assert(motor.fault_code == MOTOR_FAULT_WINDING_THERMAL);
    motor.fault_code = MOTOR_FAULT_NONE;
    foc.phase_a_current_a = 2.5f;
    foc.phase_b_current_a = foc.phase_c_current_a = -1.25f;
    MotorThermalControl_UpdateProtection(&configuration, &foc, &motor, true, false, true, false);
    assert(motor.fault_code == MOTOR_FAULT_NONE);
    foc.phase_a_current_a = NAN;
    MotorThermalControl_UpdateProtection(&configuration, &foc, &motor, true, false, true, false);
    assert(motor.fault_code == MOTOR_FAULT_WINDING_THERMAL);

    motor = (MotorControlState){0};
    motor.control_mode = MOTOR_CONTROL_CURRENT;
    uint32_t now_ms = 0U;
    for (unsigned i = 0U; i < 100U && MotorThermalControl_CanRun(false); ++i)
    {
        window(6.0f, -3.0f, -3.0f, &now_ms);
    }
    assert(MotorThermalControl_CanRun(true) && !MotorThermalControl_CanRun(false));
    MotorThermalControl_UpdateProtection(&configuration, &foc, &motor, true, false, true, true);
    assert(motor.fault_code == MOTOR_FAULT_NONE);
    MotorThermalDiagnostics before, after;
    assert(MotorThermalControl_GetDiagnostics(&before));
    MotorThermalControl_UpdateProtection(&configuration, &foc, &motor, true, false, false, false);
    assert(motor.fault_code == MOTOR_FAULT_NONE);
    MotorThermalControl_UpdateProtection(&configuration, &foc, &motor, true, false, true, false);
    assert(motor.fault_code == MOTOR_FAULT_WINDING_THERMAL);
    assert(MotorThermalControl_GetDiagnostics(&after));
    assert(memcmp(before.model.heat, after.model.heat, sizeof(before.model.heat)) == 0);
    assert(after.model.fault_latched);
    puts("PASS protection owner: startup qualification, Vq vector boundary, finite inputs, "
         "derating policy and preserved heat history");
}

static void test_foc(void)
{
    MotorWindingThermalConfig config;
    MotorWindingThermal_Default(&config);
    assert(MotorThermalControl_Init(&config, 0U));
    MotorConfiguration configuration = {0};
    MotorControlState motor = {0};
    FocState foc = {0};
    configuration.application.current_limit_a = 2.5f;
    motor.thermal_current_limit_a = MotorThermalControl_CurrentLimit(2.5f);
    configuration.model.id_kp = configuration.model.iq_kp = 1.0f;
    foc.id_pi.proportional_gain = configuration.model.id_kp;
    foc.iq_pi.proportional_gain = configuration.model.iq_kp;
    foc.id_pi.sample_period_s = foc.iq_pi.sample_period_s = Current_Ts;
    motor.id_target_a = 3.0f;
    motor.iq_target_a = 4.0f;
    foc.bus_voltage_filtered_v = 33.0f;
    FOC_Current(&configuration, &foc, &motor, 0.0f, 0.0f);
    assert(s_pwm_writes == 1U && motor.fault_code == MOTOR_FAULT_NONE);
    assert(fabsf(foc.id_pi.reference - 1.5f) < 0.000001f);
    assert(fabsf(foc.iq_pi.reference - 2.0f) < 0.000001f);
    foc.phase_a_current_a = 3.0f;
    foc.phase_b_current_a = foc.phase_c_current_a = -1.5f;
    FOC_Vq_Mode(&configuration, &foc, &motor, 0.0f, 0.0f);
    assert(s_pwm_writes == 1U && motor.fault_code == MOTOR_FAULT_WINDING_THERMAL);
    motor.fault_code = MOTOR_FAULT_NONE;
    FOC_CurrentWithReference(&configuration, &foc, &motor, 0.0f, 0.0f, NAN);
    assert(s_pwm_writes == 1U && motor.fault_code == MOTOR_FAULT_WINDING_THERMAL);
    puts("PASS real FOC final dq-vector limiting, Vq actual-current boundary and invalid target "
         "rejection");
}

static void test_submission(void)
{
    MotorConfiguration config = {0};
    MotorControlState motor = {0};
    FocState foc = {0};
    config.application.current_limit_a = motor.thermal_current_limit_a = 3.0f;
    foc.bus_voltage_filtered_v = 20.0f;
    foc.id_pi.proportional_gain = foc.iq_pi.proportional_gain = 1.0f;
    foc.id_pi.integral_output = foc.iq_pi.integral_output = 30.0f;
    foc.id_filtered_a = 99.0f;
    s_submission_state = &foc;
    FOC_Current(&config, &foc, &motor, 0.0f, 0.0f);
    assert(s_integral_at_submission == 30.0f && s_filter_at_submission == 99.0f);
    assert(foc.id_pi.integral_output <= foc.id_pi.output_max);
    assert(fabsf(foc.id_filtered_a - 98.01f) < .00001f);
    assert(foc.id_pi.output == foc.mod_d * foc.bus_voltage_filtered_v / 1.5f);
    s_submission_result = BSP_TIMEOUT;
    FOC_Current(&config, &foc, &motor, 0.0f, 0.0f);
    assert(motor.fault_code == MOTOR_FAULT_CONTROL_OVERRUN);
    assert(foc.id_pi.output == foc.mod_d * foc.bus_voltage_filtered_v / 1.5f);
    s_submission_state = NULL;
    puts("PASS PWM submitted before next-tick PI tracking and statistics; failure state retained");
}

static void test_disabled(void)
{
    MotorWindingThermalConfig config;
    MotorWindingThermal_Default(&config);
    assert(!MotorThermalControl_CanRun(false));
    assert(MotorThermalControl_CurrentLimit(3.0f) == 0.0f);
    assert(MotorThermalControl_Init(&config, 0U));
    assert(!MotorThermalControl_Configure(&config, false));
    assert(MotorThermalControl_Configure(&config, true));
    for (unsigned i = 0U; i < 100000U; ++i)
    {
        MotorThermalControl_Sample(NAN, INFINITY, 100.0f, false, i);
    }
    assert(MotorThermalControl_CanRun(false) && MotorThermalControl_Guard(false));
    assert(MotorThermalControl_CheckWatchdog(1000000U, false));
    assert(MotorThermalControl_CurrentLimit(3.0f) == 3.0f);
    assert(MotorThermalControl_CurrentLimit(NAN) == 0.0f);
    MotorThermalDiagnostics diagnostics, before;
    memset(&diagnostics, 0xA5, sizeof(diagnostics));
    before = diagnostics;
    assert(!MotorThermalControl_GetDiagnostics(&diagnostics));
    assert(memcmp(&before, &diagnostics, sizeof(before)) == 0);

    MotorConfiguration configuration = {0};
    MotorControlState motor = {0};
    FocState foc = {0};
    configuration.application.current_limit_a = 3.0f;
    motor.control_mode = MOTOR_CONTROL_CURRENT;
    MotorThermalControl_UpdateProtection(&configuration, &foc, &motor, true, false, true, false);
    assert(motor.thermal_current_limit_a == 3.0f && !motor.thermal_limit_active);
    foc.id_pi.proportional_gain = foc.iq_pi.proportional_gain = 1.0f;
    foc.id_pi.sample_period_s = foc.iq_pi.sample_period_s = Current_Ts;
    foc.bus_voltage_filtered_v = 30.0f;
    motor.id_target_a = 3.0f;
    motor.iq_target_a = 4.0f;
    FOC_Current(&configuration, &foc, &motor, 0.0f, 0.0f);
    assert(s_pwm_writes == 1U && motor.fault_code == MOTOR_FAULT_NONE);
    assert(fabsf(foc.id_pi.reference - 1.8f) < 0.000001f);
    assert(fabsf(foc.iq_pi.reference - 2.4f) < 0.000001f);
    motor.control_mode = MOTOR_CONTROL_VQ;
    foc.phase_a_current_a = 4.0f;
    foc.phase_b_current_a = foc.phase_c_current_a = -2.0f;
    MotorThermalControl_UpdateProtection(&configuration, &foc, &motor, true, false, true, false);
    assert(motor.fault_code == MOTOR_FAULT_WINDING_THERMAL);
    motor.fault_code = MOTOR_FAULT_NONE;
    FOC_CurrentWithReference(&configuration, &foc, &motor, 0.0f, 0.0f, NAN);
    assert(s_pwm_writes == 1U && motor.fault_code == MOTOR_FAULT_WINDING_THERMAL);
    puts("PASS disabled thermal model: no fabricated diagnostics, 3A vector/current guards "
         "retained");
}

int main(int argc, char **argv)
{
    assert(argc == 2);
    if (strcmp(argv[1], "submission") == 0)
    {
        test_submission();
    }
    else if (strcmp(argv[1], "disabled") == 0)
    {
        test_disabled();
    }
    else if (strcmp(argv[1], "core") == 0)
    {
        test_core();
    }
    else if (strcmp(argv[1], "runtime") == 0)
    {
        test_runtime();
    }
    else if (strcmp(argv[1], "watchdog") == 0)
    {
        test_watchdog();
    }
    else if (strcmp(argv[1], "protection") == 0)
    {
        test_protection();
    }
    else if (strcmp(argv[1], "foc") == 0)
    {
        test_foc();
    }
    else
    {
        return 2;
    }
    return 0;
}
