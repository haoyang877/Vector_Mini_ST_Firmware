/* 真实热纯核、配置与当前关闭热调度的兼容边界；不访问硬件。 */
#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <string.h>
#include "motor_thermal_control.h"
#include "application_config.h"
void MotorFault_Report(MotorControlState *motor, MotorFaultCode fault)
{
    if (motor->fault_code == MOTOR_FAULT_NONE)
    {
        motor->fault_code = fault;
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
static void window(MotorThermalState *state, float current)
{
    for (unsigned i = 0; i < state->config.samples_per_update; i++)
    {
        assert(MotorThermal_Accumulate(state, current, -current * 0.5f, -current * 0.5f));
    }
    assert(MotorThermal_Update(state));
}
static void test_history(void)
{
    MotorWindingThermalConfig physical;
    MotorWindingThermal_Default(&physical);
    physical.heating_tau_s = 0.1f;
    MotorThermalConfig config;
    assert(MotorWindingThermal_Derive(&physical, &config));
    MotorThermalState state;
    assert(MotorThermal_Init(&state, &config, 0.0f));
    bool derated = false;
    for (unsigned i = 0; i < 1000 && !state.fault_latched; i++)
    {
        window(&state, 6.0f);
        derated |= state.derating && state.available_a < config.peak_a;
    }
    assert(derated && state.fault_latched && state.available_a == 0 && state.max_heat >= 1.0f);
    assert(!MotorThermal_ClearFault(&state));
    MotorThermalState before = state;
    MotorThermalConfig invalid = config;
    invalid.peak_a = 0;
    assert(!MotorThermal_Reconfigure(&state, &invalid, state.heating_alpha, state.cooling_alpha));
    assert(memcmp(&state, &before, sizeof(state)) == 0);
    assert(MotorThermal_Reconfigure(&state, &config, state.heating_alpha, state.cooling_alpha));
    assert(memcmp(state.heat, before.heat, sizeof(state.heat)) == 0 && state.fault_latched);
    for (unsigned i = 0; i < 200000 && state.max_heat > config.clear_heat; i++)
    {
        window(&state, 0.0f);
    }
    assert(state.max_heat > 0 && state.max_heat <= config.clear_heat && state.fault_latched);
    float heat = state.max_heat;
    assert(MotorThermal_ClearFault(&state));
    assert(state.max_heat == heat && !state.fault_latched);
    for (unsigned i = 1; i <= 10000; i++)
    {
        float x = (float)i / 1000000.0f;
        assert(fabs((double)MotorThermal_Alpha(x, 1.0f) - (-expm1(-(double)x))) < 2e-9);
    }
}
static void test_disabled_runtime(void)
{
    assert(APP_ENABLED_WINDING_THERMAL_MODEL == 0);
    assert(!MotorThermalControl_CanRun(true) && MotorThermalControl_CurrentLimit(4.0f) == 0);
    MotorWindingThermalConfig config;
    MotorWindingThermal_Default(&config);
    assert(MotorThermalControl_Init(&config, 0));
    assert(MotorThermalControl_CurrentLimit(4.0f) == 4.0f);
    assert(!MotorThermalControl_Configure(&config, false));
    assert(MotorThermalControl_Configure(&config, true));
    MotorThermalDiagnostics diagnostics;
    MotorThermalControl_Sample(NAN, NAN, NAN, false, 1000);
    assert(!MotorThermalControl_GetDiagnostics(&diagnostics));
    assert(MotorThermalControl_CheckWatchdog(10000, false));
    MotorConfiguration settings = {0};
    settings.application.current_limit_a = 2.5f;
    MotorControlState motor = {0};
    motor.control_mode = MOTOR_CONTROL_VQ;
    FocState foc = {0};
    foc.phase_current_a.a = 3;
    foc.phase_current_a.b = foc.phase_current_a.c = -1.5f;
    MotorThermalControl_UpdateProtection(&settings, &foc, &motor, true, true, true, false);
    assert(motor.fault_code == MOTOR_FAULT_NONE && motor.thermal_current_limit_a == 2.5f);
    MotorThermalControl_UpdateProtection(&settings, &foc, &motor, true, false, true, false);
    assert(motor.fault_code == MOTOR_FAULT_WINDING_THERMAL);
}
int main(void)
{
    test_core();
    test_history();
    test_disabled_runtime();
    puts("PASS thermal: heat history, derating/trip/cooling, atomic reject, alpha, disabled "
         "runtime voltage guard");
    return 0;
}
